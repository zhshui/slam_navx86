#!/usr/bin/env python3
"""
可视化 scans.pcd，按 Z 轴范围 [-0.1, 0.6] 着色：
  - 绿色: 保留范围内的点 (z ∈ [-0.1, 0.6])
  - 红色: 被丢弃的点 (z < -0.1 或 z > 0.6)
"""

import struct
import numpy as np
import matplotlib
# 优先尝试有 GUI 的后端，没有则保存图片
try:
    import tkinter
    matplotlib.use('TkAgg')
except:
    matplotlib.use('Agg')
    SAVE_ONLY = True
else:
    SAVE_ONLY = False

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

PCD_PATH = "/home/shui/LY/lite_cog/system/map/scans.pcd"
Z_MIN = -0.1
Z_MAX = 0.6
TARGET_SAMPLE = 80000  # 目标采样点数

def read_pcd_binary_stream(filepath, target_points):
    """流式读取二进制 PCD，均匀采样约 target_points 个点"""
    with open(filepath, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii')
            header_lines.append(line.strip())
            if line.startswith('DATA'):
                break

        fields = []
        sizes = []
        counts = []
        total_points = 0
        for line in header_lines:
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('COUNT'):
                counts = [int(x) for x in line.split()[1:]]
            elif line.startswith('POINTS'):
                total_points = int(line.split()[1])

        point_size = sum(s * c for s, c in zip(sizes, counts))
        print(f"Total points: {total_points:,}, point size: {point_size} bytes")

        # 找到 x,y,z 偏移
        offset_x = offset_y = offset_z = 0
        byte_offset = 0
        for field, size, count in zip(fields, sizes, counts):
            for _ in range(count):
                if field == 'x': offset_x = byte_offset
                if field == 'y': offset_y = byte_offset
                if field == 'z': offset_z = byte_offset
                byte_offset += size
        fmt = 'f'

        data_start = f.tell()
        chunk_size = 2000000  # 每次读 200 万个点
        step = max(1, total_points // target_points)
        print(f"Sampling every {step} points, target ~{target_points} points")

        all_xyz = []
        for start in range(0, total_points, chunk_size):
            end = min(start + chunk_size, total_points)
            n = end - start
            f.seek(data_start + start * point_size)
            raw = f.read(n * point_size)

            indices = np.arange(start, end, step, dtype=np.int64)
            local_indices = (indices - start) * point_size

            for loc in local_indices:
                if loc + point_size > len(raw):
                    break
                x = struct.unpack_from(fmt, raw, offset_x + loc)[0]
                y = struct.unpack_from(fmt, raw, offset_y + loc)[0]
                z = struct.unpack_from(fmt, raw, offset_z + loc)[0]
                all_xyz.append([x, y, z])

        return np.array(all_xyz, dtype=np.float32)

def main():
    print(f"Reading {PCD_PATH} ...")
    xyz = read_pcd_binary_stream(PCD_PATH, TARGET_SAMPLE)
    print(f"Sampled {len(xyz)} points")

    z = xyz[:, 2]
    z_min_all, z_max_all = z.min(), z.max()
    in_range = (z >= Z_MIN) & (z <= Z_MAX)
    out_range = ~in_range

    n_in = np.sum(in_range)
    n_out = np.sum(out_range)
    ratio = 100.0 * n_in / len(xyz)
    print(f"Z range of data: [{z_min_all:.2f}, {z_max_all:.2f}]")
    print(f"Within  [{Z_MIN}, {Z_MAX}]: {n_in} ({ratio:.1f}%) -> GREEN (retained)")
    print(f"Outside [{Z_MIN}, {Z_MAX}]: {n_out} ({100-ratio:.1f}%) -> RED (discarded)")

    fig = plt.figure(figsize=(16, 10))
    ax = fig.add_subplot(111, projection='3d')

    if n_out > 0:
        ax.scatter(xyz[out_range, 0], xyz[out_range, 1], xyz[out_range, 2],
                   c='red', s=0.4, alpha=0.5, label=f'Discarded ({n_out})')
    if n_in > 0:
        ax.scatter(xyz[in_range, 0], xyz[in_range, 1], xyz[in_range, 2],
                   c='green', s=0.4, alpha=0.5, label=f'Retained ({n_in})')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'scans.pcd — Z filter [{Z_MIN}, {Z_MAX}] m\n'
                 f'Total sampled: {len(xyz)} points  |  '
                 f'Retained ratio: {ratio:.1f}%')
    ax.legend(markerscale=12, loc='upper right')
    ax.set_zlim(np.percentile(z, [0.5, 99.5]))

    plt.tight_layout()
    out_path = "/home/shui/LY/lite_cog/system/map/scans_z_range.png"
    plt.savefig(out_path, dpi=150)
    print(f"\nPlot saved to {out_path}")

    if not SAVE_ONLY:
        print("Displaying interactive plot...")
        plt.show()

if __name__ == '__main__':
    main()
