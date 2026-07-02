#!/usr/bin/env python3
"""深度分析 scans_4 飞走的根因——空间结构 + 与前后 scan 对比"""
import struct
import numpy as np

def read_header(filepath):
    with open(filepath, 'rb') as f:
        h = {}
        while True:
            line = f.readline().decode('ascii', errors='replace')
            if line.startswith('FIELDS'): h['f'] = line.split()[1:]
            elif line.startswith('SIZE'): h['sz'] = [int(x) for x in line.split()[1:]]
            elif line.startswith('POINTS'): h['n'] = int(line.split()[1])
            if line.startswith('DATA'): h['ds'] = f.tell(); break
    h['ps'] = sum(h['sz'])
    ox = oy = oz = bo = 0
    for fi, si in zip(h['f'], h['sz']):
        if fi == 'x': ox = bo
        elif fi == 'y': oy = bo
        elif fi == 'z': oz = bo
        bo += si
    h['ox'], h['oy'], h['oz'] = ox, oy, oz
    return h

def sample_xyz(filepath, h, max_n=500000):
    total, ps = h['n'], h['ps']
    ox, oy, oz = h['ox'], h['oy'], h['oz']
    step = max(1, total // max_n)
    xs, ys, zs = [], [], []
    with open(filepath, 'rb') as f:
        f.seek(h['ds'])
        for start in range(0, total, 2_000_000):
            end = min(start + 2_000_000, total)
            raw = f.read((end - start) * ps)
            for i in range(0, end - start, step):
                b = i * ps
                if b + ps > len(raw): break
                xs.append(struct.unpack_from('f', raw, ox + b)[0])
                ys.append(struct.unpack_from('f', raw, oy + b)[0])
                zs.append(struct.unpack_from('f', raw, oz + b)[0])
    return np.array(xs), np.array(ys), np.array(zs)

base = "/home/robot/go2_nav/lite_cog/system/map"

# =========================================================================
# Part 1: scans_4 整体结构分析
# =========================================================================
print("=" * 70)
print("  Part 1: scans_4 空间结构深度分析")
print("=" * 70)

h4 = read_header(f"{base}/scans_4/scans_4.pcd")
x4, y4, z4 = sample_xyz(f"{base}/scans_4/scans_4.pcd", h4)
xyz4 = np.column_stack([x4, y4, z4])

print(f"\n总点数: {h4['n']:,}")
print(f"采样点: {len(x4):,}")

# Z 分布
print(f"\n--- Z 分布 ---")
print(f"Z min={z4.min():.1f}  Z max={z4.max():.1f}  span={z4.ptp():.1f}m")
for pcts in [1, 5, 10, 25, 50, 75, 90, 95, 99, 99.9]:
    print(f"  Z P{pcts:>4}: {np.percentile(z4, pcts):.1f}")

# 识别"正常"点 vs "飞走"点
# 正常区域应该和 scans_3/5 类似: X ~[-10,100], Y ~[-60,45]
# 飞走的点会在远处

# 关键：看 Z 的极端值分布
z_sorted = np.sort(z4)
n_total = len(z4)
# Z < -100 的点比例
deep_neg = (z4 < -100).sum()
deep_pos = (z4 > 50).sum()
near_ground = ((z4 > -2) & (z4 < 2)).sum()

print(f"\n--- 异常点统计 ---")
print(f"Z < -100m (深地下):    {deep_neg:>10,} ({100*deep_neg/n_total:.1f}%)")
print(f"Z > 50m (高空):        {deep_pos:>10,} ({100*deep_pos/n_total:.1f}%)")
print(f"Z in [-2, 2] (近地面): {near_ground:>10,} ({100*near_ground/n_total:.1f}%)")

# Y 分布 - 识别飞走的主方向
print(f"\n--- Y (主漂移方向) 分布 ---")
for pcts in [1, 5, 10, 25, 50, 75, 90, 95, 99, 99.9]:
    print(f"  Y P{pcts:>4}: {np.percentile(y4, pcts):.1f}")

# 按 Y 分段分析 Z 的行为
print(f"\n--- 按 Y 区间的 Z 分布 (追踪漂移路径) ---")
y_bins = [-200, -100, -50, 0, 50, 100, 200, 500, 1000, 2000]
for i in range(len(y_bins)-1):
    mask = (y4 >= y_bins[i]) & (y4 < y_bins[i+1])
    if mask.sum() > 10:
        z_seg = z4[mask]
        print(f"  Y=[{y_bins[i]:>6}, {y_bins[i+1]:>6}): "
              f"{mask.sum():>10,} pts, "
              f"Z=[{z_seg.min():.1f}, {z_seg.max():.1f}] "
              f"mean={z_seg.mean():.1f}")

# XY 二维密度热力分析 - 识别"正常区域"和"飞走轨迹"
print(f"\n--- XY 象限分析 ---")
# 正常区域: X in [-10, 20], Y in [-15, 15]
normal_mask = (x4 > -10) & (x4 < 20) & (y4 > -15) & (y4 < 15)
n_normal = normal_mask.sum()
print(f"正常区域 (X[-10,20] Y[-15,15]): {n_normal:,} pts")
if n_normal > 0:
    z_normal = z4[normal_mask]
    print(f"  正常区域 Z: [{z_normal.min():.1f}, {z_normal.max():.1f}]")
    # 正常区域地面拟合
    z_thresh = np.percentile(z_normal, 15)
    ground_mask = z_normal <= z_thresh
    ground = xyz4[normal_mask][ground_mask]
    if len(ground) > 10:
        c = ground.mean(axis=0)
        _, S, Vt = np.linalg.svd(ground - c, full_matrices=False)
        n = Vt[-1]
        if n[2] < 0: n = -n
        tilt = np.degrees(np.arccos(np.clip(np.dot(n, [0,0,1]), -1, 1)))
        print(f"  正常区域地面法向量: [{n[0]:.4f} {n[1]:.4f} {n[2]:.4f}] tilt={tilt:.1f}°")

# =========================================================================
# Part 2: 对比 scans_3, scans_4, scans_5 — 看漂移如何开始
# =========================================================================
print(f"\n\n{'='*70}")
print(f"  Part 2: scans_3 → scans_4 → scans_5 漂移演进")
print(f"{'='*70}")

for scan in ["scans_3", "scans_5"]:
    pcd = f"{base}/{scan}/{scan}.pcd"
    try:
        h = read_header(pcd)
        x, y, z = sample_xyz(pcd, h)
        print(f"\n--- {scan} ({h['n']:,} pts) ---")
        print(f"  X: [{x.min():.1f}, {x.max():.1f}] span={x.ptp():.1f}")
        print(f"  Y: [{y.min():.1f}, {y.max():.1f}] span={y.ptp():.1f}")
        print(f"  Z: [{z.min():.2f}, {z.max():.2f}] span={z.ptp():.2f}")
        # 地面
        zt = np.percentile(z, 5)
        gm = z <= zt
        if gm.sum() > 10:
            g = np.column_stack([x[gm], y[gm], z[gm]])
            c = g.mean(axis=0)
            _, S, Vt = np.linalg.svd(g - c, full_matrices=False)
            n = Vt[-1]
            if n[2] < 0: n = -n
            tilt = np.degrees(np.arccos(np.clip(np.dot(n, [0,0,1]), -1, 1)))
            print(f"  地面 P5: n=[{n[0]:.4f} {n[1]:.4f} {n[2]:.4f}] tilt={tilt:.1f}°")
    except Exception as e:
        print(f"  ERROR: {e}")

# =========================================================================
# Part 3: 点密度分析 — 区分"扫描到的区域"和"漂移轨迹"
# =========================================================================
print(f"\n\n{'='*70}")
print(f"  Part 3: 点密度与漂移模式分析")
print(f"{'='*70}")

# 将 scans_4 按 XY 分块
x_bins = np.arange(-200, 300, 20)
y_bins = np.arange(-200, 1700, 20)

print(f"\nXY 块密度分布 (cell=20m):")
print(f"{'X_range':<20} {'Y_range':<20} {'n_pts':>10} {'密度':>10} {'Z_mean':>10}")
print("-" * 72)

cells = []
for i in range(len(x_bins)-1):
    for j in range(len(y_bins)-1):
        mask = (x4 >= x_bins[i]) & (x4 < x_bins[i+1]) & \
               (y4 >= y_bins[j]) & (y4 < y_bins[j+1])
        n = mask.sum()
        if n > 0:
            z_mean = z4[mask].mean()
            cells.append((x_bins[i], y_bins[j], n, z_mean))

# 按点数排序显示 top 和 bottom
cells.sort(key=lambda c: -c[2])
print("TOP 10 最密集区域 (可能是正常扫描区域):")
for x0, y0, n, zm in cells[:10]:
    print(f"  X[{x0:>5.0f},{x0+20:>5.0f}) Y[{y0:>5.0f},{y0+20:>5.0f})  "
          f"{n:>8,} pts  Z_mean={zm:.1f}")

print(f"\n稀疏区域 (点数<100, 可能是漂移噪声):")
sparse = [c for c in cells if c[2] < 100]
print(f"  稀疏cell数: {len(sparse)}/{len(cells)}, "
      f"稀疏点点数: {sum(c[2] for c in sparse):,}")

# 找出"主体区域" — 包含最多点的连续区域
print(f"\n--- 主体点云区域 (Z在合理范围 -2~20) ---")
reasonable_z = (z4 > -2) & (z4 < 20)
n_reasonable = reasonable_z.sum()
print(f"Z∈[-2,20] 的点: {n_reasonable:,} ({100*n_reasonable/len(z4):.1f}%)")
if n_reasonable > 0:
    xr, yr = x4[reasonable_z], y4[reasonable_z]
    print(f"  主体区域 X: [{xr.min():.1f}, {xr.max():.1f}] span={xr.ptp():.1f}")
    print(f"  主体区域 Y: [{yr.min():.1f}, {yr.max():.1f}] span={yr.ptp():.1f}")

# =========================================================================
# Part 4: 内存估算
# =========================================================================
print(f"\n\n{'='*70}")
print(f"  Part 4: 内存压力分析")
print(f"{'='*70}")

# 每个点 8 fields × 4 bytes = 32 bytes
raw_bytes = h4['n'] * 32
print(f"scans_4 原始点云数据: {raw_bytes/1024/1024:.0f} MB")
print(f"  {h4['n']:,} points × 32 bytes = {raw_bytes/1024/1024:.0f} MB")

# SLAM 运行时的额外内存
# hdl_graph_slam 中:
# - Keyframe 保存压缩点云 (通常保留最近 N 个)
# - 图结构: g2o 位姿图 (~几MB)
# - ICP 匹配: 需要 source+target 点云同时在内存
# - floor detection: 需要整个点云
print(f"\nSLAM 运行时估算 (hdl_graph_slam):")
print(f"  点云存储:              ~{raw_bytes/1024/1024:.0f} MB")
print(f"  g2o 位姿图 (3040万点): ~{h4['n'] * 64 / 1024 / 1024:.0f} MB (估算)")
print(f"  ICP 中间数据:          ~{raw_bytes/1024/1024/2:.0f} MB")
print(f"  ROS 消息缓冲:          ~{raw_bytes/1024/1024/2:.0f} MB")
total_est = raw_bytes * 2.5 / 1024 / 1024
print(f"  总计估算:              ~{total_est:.0f} MB")
print(f"  系统可用内存:           15 GB")

if total_est > 12000:
    print(f"  ⚠️ 估算内存 ({total_est/1024:.1f} GB) 接近系统上限 (15 GB)")
    print(f"  → 内存压力很可能加剧了漂移！")
else:
    print(f"  → 内存压力不是主因，但会显著拖慢处理速度")
