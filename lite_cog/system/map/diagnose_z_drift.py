#!/usr/bin/env python3
"""
Z 轴漂移诊断：分块统计地面 Z，生成热力图（向量化高效版）。
"""
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, os

PCD_PATH = sys.argv[1] if len(sys.argv) > 1 else "/home/robot/go2_nav/lite_cog/system/map/scans_8/scans_8.pcd"
GRID_SIZE = 1.0
GROUND_PERCENTILE = 5

def read_pcd_xyz(filepath):
    with open(filepath, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='replace')
            header_lines.append(line.strip())
            if line.startswith('DATA'):
                break
        fields, sizes, counts, total_points = [], [], [], 0
        for line in header_lines:
            if line.startswith('FIELDS'): fields = line.split()[1:]
            elif line.startswith('SIZE'): sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('COUNT'): counts = [int(x) for x in line.split()[1:]]
            elif line.startswith('POINTS'): total_points = int(line.split()[1])
        point_size = sum(s * c for s, c in zip(sizes, counts))
        offset_x = offset_y = offset_z = 0
        byte_offset = 0
        for field, size, count in zip(fields, sizes, counts):
            for _ in range(count):
                if field == 'x': offset_x = byte_offset
                elif field == 'y': offset_y = byte_offset
                elif field == 'z': offset_z = byte_offset
                byte_offset += size
        data_start = f.tell()
        chunk_size = 2000000
        xyz_list = []
        for start in range(0, total_points, chunk_size):
            end = min(start + chunk_size, total_points)
            n = end - start
            f.seek(data_start + start * point_size)
            raw = f.read(n * point_size)
            interleaved = np.zeros(n * 3, dtype=np.float32)
            for i in range(n):
                base = i * point_size
                interleaved[i * 3 + 0] = struct.unpack_from('f', raw, offset_x + base)[0]
                interleaved[i * 3 + 1] = struct.unpack_from('f', raw, offset_y + base)[0]
                interleaved[i * 3 + 2] = struct.unpack_from('f', raw, offset_z + base)[0]
            xyz_list.append(interleaved.reshape(-1, 3))
        return np.vstack(xyz_list).astype(np.float32)

print(f"Loading {PCD_PATH} ...")
xyz = read_pcd_xyz(PCD_PATH)
print(f"Loaded {xyz.shape[0]:,} points")
print(f"Z range: [{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}]")

x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
x_min, x_max = x.min(), x.max()
y_min, y_max = y.min(), y.max()

xbins = int(np.ceil((x_max - x_min) / GRID_SIZE))
ybins = int(np.ceil((y_max - y_min) / GRID_SIZE))
print(f"Grid: {xbins} x {ybins}")

# --- 向量化分块：用 linear index ---
xi = np.clip(((x - x_min) / GRID_SIZE).astype(np.int32), 0, xbins - 1)
yi = np.clip(((y - y_min) / GRID_SIZE).astype(np.int32), 0, ybins - 1)
linear_idx = yi * xbins + xi
n_cells = xbins * ybins

# 每格：计数、Z 最小值、Z percentile 近似
count = np.bincount(linear_idx, minlength=n_cells).reshape(ybins, xbins)

# 用 argsort 分组取 percentile —— 高效但内存大。改为取每格最小 500 个点的均值近似
# 更简单：对每个 linear_idx 组内的 Z 排序太贵。用分位数近似：取 min N 个。
# 实际上对于地面估计，取每格 Z 最小的几个点取平均就够了。
# 用 np.lexsort 一次排序：先按 linear_idx，再按 z
print("Sorting by grid cell + Z ...")
sorter = np.lexsort((z, linear_idx))  # 按 (z, cell) 排序，但 lexsort 最后一个 key 是主 key
# Wait: lexsort((z, linear_idx)) sorts by z first, then linear_idx. We want by linear_idx first.
sorter = np.lexsort((z, linear_idx))
sorted_z = z[sorter]
sorted_cell = linear_idx[sorter]

# 找每个 cell 的起止边界
cell_starts = np.searchsorted(sorted_cell, np.arange(n_cells + 1))
print("Computing per-cell ground Z ...")
grid_z = np.full(n_cells, np.nan, dtype=np.float32)
for c in range(n_cells):
    lo, hi = cell_starts[c], cell_starts[c + 1]
    n = hi - lo
    if n < 20:
        continue
    # 取底部 5% 的均值
    k = max(1, n * GROUND_PERCENTILE // 100)
    grid_z[c] = sorted_z[lo:lo + k].mean()

grid_z = grid_z.reshape(ybins, xbins)
grid_count = count

# --- 可视化 ---
fig, axes = plt.subplots(1, 3, figsize=(24, 7))
extent = [x_min, x_max, y_min, y_max]

im1 = axes[0].imshow(grid_z, origin='lower', extent=extent, cmap='turbo', aspect='equal')
axes[0].set_title(f'Ground Z (bottom {GROUND_PERCENTILE}% per {GRID_SIZE}m cell)')
axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)')
plt.colorbar(im1, ax=axes[0], label='Ground Z (m)', shrink=0.75)

im2 = axes[1].imshow(grid_count, origin='lower', extent=extent, cmap='viridis',
                      norm=matplotlib.colors.LogNorm(), aspect='equal')
axes[1].set_title(f'Point count per {GRID_SIZE}m cell')
axes[1].set_xlabel('X (m)'); axes[1].set_ylabel('Y (m)')
plt.colorbar(im2, ax=axes[1], label='Points', shrink=0.75)

flat = grid_z[~np.isnan(grid_z)]
z_mean = np.nanmean(grid_z)
z_std = np.nanstd(grid_z)
axes[2].hist(flat, bins=100, color='steelblue', edgecolor='white', alpha=0.8)
axes[2].axvline(0, color='red', linestyle='--', label='Z=0 (ideal)')
axes[2].axvline(z_mean, color='orange', linestyle='-', label=f'Mean={z_mean:.3f}m')
axes[2].axvline(z_mean - z_std, color='orange', linestyle=':', label=f'±1σ ({z_std:.3f}m)')
axes[2].axvline(z_mean + z_std, color='orange', linestyle=':')
axes[2].set_title(f'Ground Z distribution\nmean={z_mean:.3f}m, std={z_std:.3f}m')
axes[2].set_xlabel('Ground Z (m)'); axes[2].set_ylabel('Cell count')
axes[2].legend()

plt.tight_layout()
outdir = os.path.dirname(PCD_PATH)
outpath = os.path.join(outdir, 'z_drift_diagnosis.png')
plt.savefig(outpath, dpi=150)
print(f"\nSaved: {outpath}")

print(f"\nGround Z stats across {len(flat)} valid cells:")
print(f"  Mean: {z_mean:.3f} m")
print(f"  Std:  {z_std:.3f} m")
print(f"  Min:  {np.nanmin(grid_z):.3f} m")
print(f"  Max:  {np.nanmax(grid_z):.3f} m")
print(f"  Cells with Z > 0.10m: {np.sum(flat > 0.10)} / {len(flat)} ({100*np.sum(flat>0.10)/len(flat):.1f}%)")
print(f"  Cells with Z > 0.30m: {np.sum(flat > 0.30)} / {len(flat)} ({100*np.sum(flat>0.30)/len(flat):.1f}%)")
print(f"  Cells with Z > 0.50m: {np.sum(flat > 0.50)} / {len(flat)} ({100*np.sum(flat>0.50)/len(flat):.1f}%)")
print(f"  Cells with Z < -0.10m: {np.sum(flat < -0.10)} / {len(flat)} ({100*np.sum(flat<-0.10)/len(flat):.1f}%)")
