#!/usr/bin/env python3
"""
path_smoother.py — A* 全局路径平滑节点（带碰撞检测）

订阅 /move_base/GlobalPlanner/plan (A* 输出的栅格折线)
  → Douglas-Peucker 简化关键点 (costmap 碰撞验证)
  → 三次 Clamped B样条插值平滑 (costmap 碰撞验证)
  → 发布 /move_base/GlobalPlanner/plan

纯 numpy 实现，不依赖 scipy。
"""

import math
import numpy as np
import rospy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped


# ============================================================
#  Ramer–Douglas–Peucker 路径简化
# ============================================================

def perpendicular_distance(pt, line_start, line_end):
    """点到线段的垂直距离"""
    if np.allclose(line_start, line_end):
        return np.linalg.norm(pt - line_start)
    v = line_end - line_start
    w = pt - line_start
    t = np.dot(w, v) / np.dot(v, v)
    t = np.clip(t, 0.0, 1.0)
    projection = line_start + t * v
    return np.linalg.norm(pt - projection)


def rdp_simplify(points, eps):
    """Ramer-Douglas-Peucker 递归简化

    Args:
        points: (N, 2) numpy array
        eps:    简化容差 (米)

    Returns:
        简化后的 (M, 2) numpy array, M <= N
    """
    if len(points) < 3:
        return points

    start = points[0]
    end = points[-1]

    # 找最远点
    max_dist = 0.0
    max_idx = 0
    for i in range(1, len(points) - 1):
        d = perpendicular_distance(points[i], start, end)
        if d > max_dist:
            max_dist = d
            max_idx = i

    if max_dist <= eps:
        return np.array([start, end])

    left = rdp_simplify(points[:max_idx + 1], eps)
    right = rdp_simplify(points[max_idx:], eps)

    return np.vstack([left[:-1], right])


# ============================================================
#  三次 Clamped B样条 (Cox-de Boor, 纯 numpy)
# ============================================================

def _bspline_basis(i, k, t, knots):
    """Cox-de Boor 递推: 计算第 i 个 k 阶基函数在参数 t 处的值"""
    if k == 1:
        return 1.0 if knots[i] <= t < knots[i + 1] else 0.0

    denom1 = knots[i + k - 1] - knots[i]
    denom2 = knots[i + k] - knots[i + 1]
    c1 = 0.0 if abs(denom1) < 1e-15 else (t - knots[i]) / denom1 * _bspline_basis(i, k - 1, t, knots)
    c2 = 0.0 if abs(denom2) < 1e-15 else (knots[i + k] - t) / denom2 * _bspline_basis(i + 1, k - 1, t, knots)
    return c1 + c2


def bspline_interpolate(control_points, n_samples, order=3):
    """Clamped uniform cubic B-spline 插值

    构造 clamped 节点向量使得曲线精确经过首尾控制点，
    然后均匀采样 n_samples 个点。

    Args:
        control_points: (M, 2) numpy array — 控制点 (DP简化后的关键点)
        n_samples:      输出采样点数
        order:          B样条阶数 (3 = 三次, 默认)

    Returns:
        (n_samples, 2) numpy array — 平滑曲线
    """
    M = len(control_points)
    if M < order + 1:
        # 控制点太少，直接线性插值
        t_orig = np.linspace(0, 1, M)
        t_new = np.linspace(0, 1, n_samples)
        x = np.interp(t_new, t_orig, control_points[:, 0])
        y = np.interp(t_new, t_orig, control_points[:, 1])
        return np.column_stack([x, y])

    # 节点数量: n_knots = M + order + 1
    n_knots = M + order + 1

    # Clamped 均匀节点向量 (首尾多重度为 order+1)
    knots = np.zeros(n_knots)
    # 内部节点均匀分布
    inner_count = n_knots - 2 * (order + 1)
    if inner_count > 0:
        knots[order + 1 : n_knots - order - 1] = np.linspace(0, 1, inner_count + 2)[1:-1]
    knots[n_knots - order - 1:] = 1.0

    # 采样 B样条在 [0, 1] 上
    t_vals = np.linspace(0.0, 1.0 - 1e-12, n_samples)

    curve = np.zeros((n_samples, 2))
    for s, t in enumerate(t_vals):
        pt = np.zeros(2)
        for i in range(M):
            b = _bspline_basis(i, order + 1, t, knots)
            pt += b * control_points[i]
        curve[s] = pt

    return curve


# ============================================================
#  路径工具
# ============================================================

def path_to_xy_array(path_msg):
    """nav_msgs/Path → (N, 2) numpy array"""
    pts = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
    return np.array(pts) if pts else np.empty((0, 2))


def xy_array_to_path(xy, header, frame_id):
    """(N, 2) numpy array → nav_msgs/Path"""
    path = Path()
    path.header = header
    path.header.frame_id = frame_id

    for i in range(len(xy)):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(xy[i, 0])
        pose.pose.position.y = float(xy[i, 1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        # 计算朝向 (沿路径切线方向)
        if i < len(xy) - 1:
            dx = float(xy[i + 1, 0] - xy[i, 0])
            dy = float(xy[i + 1, 1] - xy[i, 1])
            yaw = math.atan2(dy, dx)
        elif i > 0:
            dx = float(xy[i, 0] - xy[i - 1, 0])
            dy = float(xy[i, 1] - xy[i - 1, 1])
            yaw = math.atan2(dy, dx)
        else:
            yaw = 0.0

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(pose)

    return path


def compute_path_length(xy):
    """计算路径总长度"""
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


# ============================================================
#  碰撞检测辅助函数
# ============================================================

def _sample_segment(p1, p2, step):
    """沿线段 p1→p2 以 step (m) 为步长均匀采样，包含两端点"""
    dist = float(np.linalg.norm(p2 - p1))
    if dist < 1e-9:
        return np.array([p1])
    n = max(2, int(np.ceil(dist / step)) + 1)
    t = np.linspace(0, 1, n)
    return p1 + t[:, None] * (p2 - p1)


def _check_collision(points, costmap, origin, resolution, width, height, cost_threshold):
    """检查一组点是否与障碍物碰撞

    Args:
        points:         (K, 2) 世界坐标点
        costmap:        一维 costmap 数据 (row-major, gy * width + gx)
        origin:         (ox, oy) costmap 原点世界坐标
        resolution:     分辨率 m/cell
        width, height:  costmap 尺寸
        cost_threshold: cost >= 此值视为障碍 (unknown=-1 也视为障碍)

    Returns:
        collision: bool 数组, length=K, True=碰撞
    """
    # world → grid
    gx = np.floor((points[:, 0] - origin[0]) / resolution).astype(int)
    gy = np.floor((points[:, 1] - origin[1]) / resolution).astype(int)

    # 越界检查
    valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)

    collision = np.zeros(len(points), dtype=bool)

    for i in range(len(points)):
        if valid[i]:
            idx = gy[i] * width + gx[i]
            c = costmap[idx]
            # unknown (-1) 或 cost >= threshold 均视为障碍
            if c < 0 or c >= cost_threshold:
                collision[i] = True
        else:
            # 越界视作碰撞
            collision[i] = True

    return collision


# ============================================================
#  ROS 节点
# ============================================================

class PathSmoother:
    def __init__(self):
        rospy.init_node('path_smoother', anonymous=False)

        # 平滑参数
        self.simplify_tolerance = rospy.get_param('~simplify_tolerance', 0.15)
        self.smooth_step = rospy.get_param('~smooth_step', 0.05)
        self.smooth_order = rospy.get_param('~smooth_order', 3)

        # 碰撞检测参数
        self.collision_check = rospy.get_param('~collision_check', True)
        self.cost_threshold = rospy.get_param('~cost_threshold', 100)  # >=100 视为障碍 (inscribed=253, lethal=254)
        self.validation_step = rospy.get_param('~validation_step', 0.05)  # 碰撞检测采样步长 (米)

        # 防回环: 如果平均点间距小于此值，说明已被平滑处理，跳过
        self.min_avg_spacing = rospy.get_param('~min_avg_spacing', self.smooth_step * 2.0)

        rospy.loginfo(f'[path_smoother] simplify_tolerance={self.simplify_tolerance}m, '
                      f'smooth_step={self.smooth_step}m, order={self.smooth_order}, '
                      f'collision_check={self.collision_check}, cost_threshold={self.cost_threshold}, '
                      f'validation_step={self.validation_step}m')

        # costmap 缓存
        self.costmap = None
        self.costmap_info = None

        if self.collision_check:
            self.costmap_sub = rospy.Subscriber(
                '/move_base/global_costmap/costmap', OccupancyGrid,
                self.costmap_callback, queue_size=1)
            rospy.loginfo('[path_smoother] Subscribed to /move_base/global_costmap/costmap')

        # 订阅 raw plan → 平滑后发布
        self.plan_sub = rospy.Subscriber(
            '/move_base/GlobalPlanner/plan', Path,
            self.plan_callback, queue_size=1)
        self.smooth_pub = rospy.Publisher(
            '/move_base/GlobalPlanner/plan', Path, queue_size=1)

        rospy.loginfo('[path_smoother] Ready — smoothing /move_base/GlobalPlanner/plan in-place')

    # ── costmap ─────────────────────────────────────────────

    def costmap_callback(self, msg):
        """缓存最新的 global costmap"""
        self.costmap = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.costmap_info = {
            'origin': (msg.info.origin.position.x, msg.info.origin.position.y),
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
        }

    def _costmap_ready(self):
        return self.costmap is not None and self.costmap_info is not None

    # ── 碰撞检测 ────────────────────────────────────────────

    def _validate_segment(self, p1, p2):
        """验证线段 p1→p2 是否无碰撞 (对线段采样后逐点查询 costmap)

        Returns:
            True  线段安全 (所有采样点 cost < threshold 且不是 unknown)
            False 线段穿过障碍物或 unknown 区域
        """
        if not self._costmap_ready():
            return True  # costmap 未就绪时不拦截

        samples = _sample_segment(p1, p2, self.validation_step)
        collision = _check_collision(
            samples, self.costmap,
            self.costmap_info['origin'],
            self.costmap_info['resolution'],
            self.costmap_info['width'],
            self.costmap_info['height'],
            self.cost_threshold)
        return not np.any(collision)

    def _validate_path(self, xy):
        """验证整条路径是否无碰撞

        Returns:
            (True, -1)     路径安全
            (False, index) 第一个碰撞点在 xy 中的索引
        """
        if not self._costmap_ready() or len(xy) < 1:
            return True, -1

        collision = _check_collision(
            xy, self.costmap,
            self.costmap_info['origin'],
            self.costmap_info['resolution'],
            self.costmap_info['width'],
            self.costmap_info['height'],
            self.cost_threshold)
        collided_indices = np.where(collision)[0]
        if len(collided_indices) > 0:
            return False, int(collided_indices[0])
        return True, -1

    # ── 安全 RDP 简化 ──────────────────────────────────────

    def _rdp_simplify_safe(self, raw_xy, eps):
        """带碰撞检测的 RDP 简化

        先执行标准 RDP 得到候选简化点，然后逐段检查：
        - 如果简化段无碰撞 → 接受简化
        - 如果简化段穿过障碍物 → 保留该段原始 A* 点中每隔一定间距的关键点

        这样在空旷区域获得充分简化，在障碍物附近保持安全精度。
        """
        simplified = rdp_simplify(raw_xy, eps)

        if not self.collision_check or not self._costmap_ready():
            return simplified

        # 建立 simplified → raw_xy 索引映射
        # RDP 返回的点来自输入数组的逐元素 copy，值完全相同
        simp_indices = _match_points(simplified, raw_xy)

        # 逐段验证并修复
        safe_points = [simplified[0]]
        collisions_fixed = 0

        for i in range(len(simplified) - 1):
            p1 = simplified[i]
            p2 = simplified[i + 1]
            idx1 = simp_indices[i]
            idx2 = simp_indices[i + 1]

            if self._validate_segment(p1, p2):
                safe_points.append(p2)
            else:
                # 该简化段碰撞 → 从原始 A* 路径插入中间点
                collisions_fixed += 1
                sub_pts = raw_xy[idx1 + 1:idx2]  # 原始 A* 点 (不含两端)
                if len(sub_pts) > 0:
                    # 降采样：每隔约 0.5m 保留一个点，避免过度密集
                    skip = max(1, int(0.5 / max(self.validation_step, 0.01)))
                    safe_points.extend(sub_pts[::skip].tolist())
                safe_points.append(p2)

        if collisions_fixed > 0:
            rospy.loginfo_throttle(5.0,
                f'[path_smoother] RDP: fixed {collisions_fixed} colliding segment(s) '
                f'→ {len(safe_points)} safe control points')

        return np.array(safe_points)

    # ── 主回调 ─────────────────────────────────────────────

    def plan_callback(self, msg):
        if not msg.poses:
            return

        raw_xy = path_to_xy_array(msg)
        n_raw = len(raw_xy)
        raw_length = compute_path_length(raw_xy)

        if n_raw < 4:
            # 路径太短，直接转发
            msg.header.stamp = rospy.Time.now()
            self.smooth_pub.publish(msg)
            return

        # 防回环: 如果点已经够密，说明已被平滑过，跳过
        if n_raw >= 2 and raw_length > 0:
            avg_spacing = raw_length / (n_raw - 1)
            if avg_spacing < self.min_avg_spacing:
                return  # 已是平滑路径，不重复处理

        # ── Step 1: RDP 简化（带 costmap 碰撞验证）──
        simplified = self._rdp_simplify_safe(raw_xy, self.simplify_tolerance)
        n_simple = len(simplified)

        # ── Step 2: B样条插值 ──
        if raw_length > 0:
            n_samples = max(n_raw, int(raw_length / self.smooth_step))
        else:
            n_samples = n_raw

        if n_simple >= self.smooth_order + 1 and n_simple >= 3:
            smooth_order = min(self.smooth_order, n_simple - 1)
            smoothed = bspline_interpolate(simplified, n_samples, order=smooth_order)

            # ── Step 3: 碰撞检测 (验证 B样条曲线) ──
            if self.collision_check and self._costmap_ready():
                safe, collision_idx = self._validate_path(smoothed)
                if not safe:
                    rospy.logwarn_throttle(3.0,
                        f'[path_smoother] B-spline collides at idx={collision_idx}, '
                        f'falling back to linear interpolation on validated control points')
                    # 回退: 对已验证安全的简化控制点做线性插值
                    # 控制点的每个相邻段都通过了 _validate_segment，线性插值不会出界
                    n_fallback = max(n_raw, int(compute_path_length(simplified) / self.smooth_step))
                    if len(simplified) >= 2:
                        t_orig = np.linspace(0, 1, len(simplified))
                        t_new = np.linspace(0, 1, n_fallback)
                        smoothed = np.column_stack([
                            np.interp(t_new, t_orig, simplified[:, 0]),
                            np.interp(t_new, t_orig, simplified[:, 1])
                        ])
                    else:
                        smoothed = simplified
        else:
            smoothed = simplified

        n_smooth = len(smoothed)

        # ── 发布平滑路径 ──
        smooth_msg = xy_array_to_path(
            smoothed,
            msg.header,
            msg.header.frame_id if msg.header.frame_id else 'map')
        smooth_msg.header.stamp = rospy.Time.now()

        self.smooth_pub.publish(smooth_msg)

        rospy.loginfo(f'[path_smoother] {n_raw} raw → {n_simple} simplified → {n_smooth} smooth '
                      f'({raw_length:.1f}m path)')

    def run(self):
        rospy.spin()


# ============================================================
#  工具函数
# ============================================================

def _match_points(query, reference):
    """将 query 中每行匹配到 reference 中最近邻的索引

    因为 RDP 返回的点是 reference 的精确子集 (逐元素 copy)，
    使用距离阈值 1e-6 快速匹配。
    """
    indices = []
    for q in query:
        dists = np.sum((reference - q) ** 2, axis=1)
        idx = int(np.argmin(dists))
        # 断言匹配到的是精确子集
        if dists[idx] > 1e-6:
            rospy.logwarn_throttle(10.0,
                f'[path_smoother] Imprecise point match: dist={math.sqrt(dists[idx]):.3e}')
        indices.append(idx)
    return indices


if __name__ == '__main__':
    try:
        smoother = PathSmoother()
        smoother.run()
    except rospy.ROSInterruptException:
        pass
