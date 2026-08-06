#!/usr/bin/env python3
"""
path_smoother.py — A* 全局路径平滑节点

订阅 /move_base/GlobalPlanner/plan (A* 输出的栅格折线)
  → Douglas-Peucker 简化关键点
  → 三次 Clamped B样条插值平滑
  → 发布 /move_base/GlobalPlanner/plan_smooth

纯 numpy 实现，不依赖 scipy。
"""

import math
import numpy as np
import rospy
from nav_msgs.msg import Path
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


def xy_length(xy1, xy2):
    return float(np.linalg.norm(xy1 - xy2))


# ============================================================
#  ROS 节点
# ============================================================

class PathSmoother:
    def __init__(self):
        rospy.init_node('path_smoother', anonymous=False)

        # 参数
        self.simplify_tolerance = rospy.get_param('~simplify_tolerance', 0.15)
        self.smooth_step = rospy.get_param('~smooth_step', 0.05)
        self.smooth_order = rospy.get_param('~smooth_order', 3)

        rospy.loginfo(f'[path_smoother] simplify_tolerance={self.simplify_tolerance}m, '
                      f'smooth_step={self.smooth_step}m, order={self.smooth_order}')

        # 订阅 raw plan → 平滑后发布回同 topic 替换原始路径
        self.plan_sub = rospy.Subscriber(
            '/move_base/GlobalPlanner/plan', Path,
            self.plan_callback, queue_size=1)
        self.smooth_pub = rospy.Publisher(
            '/move_base/GlobalPlanner/plan', Path, queue_size=1)

        rospy.loginfo('[path_smoother] Ready — smoothing /move_base/GlobalPlanner/plan in-place')

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

        # 防回环: 如果点已经够密 (平均间距 < smooth_step*2)，说明已被平滑过，跳过
        if n_raw >= 2 and raw_length > 0:
            avg_spacing = raw_length / (n_raw - 1)
            if avg_spacing < self.smooth_step * 2.0:
                return  # 已是平滑路径，不重复处理

        # ── Step 1: RDP 简化 ──
        simplified = rdp_simplify(raw_xy, self.simplify_tolerance)
        n_simple = len(simplified)

        # ── Step 2: B样条插值 ──
        if raw_length > 0:
            n_samples = max(n_raw, int(raw_length / self.smooth_step))
        else:
            n_samples = n_raw

        if n_simple >= self.smooth_order + 1 and n_simple >= 3:
            smoothed = bspline_interpolate(simplified, n_samples, order=min(self.smooth_order, n_simple - 1))
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


if __name__ == '__main__':
    try:
        smoother = PathSmoother()
        smoother.run()
    except rospy.ROSInterruptException:
        pass
