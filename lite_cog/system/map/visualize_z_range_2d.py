#!/usr/bin/env python3
"""
2D 俯视图：显示 scans.pcd 按 Z 范围着色的 XY 投影
"""
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PCD_PATH = "/home/shui/LY/lite_cog/system/map/scans.pcd"
Z_MIN, Z_MAX = -0.1, 0.6
TARGET_SAMPLE = 100000

def read_pcd_binary_stream(filepath, target_points):
    with open(filepath, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii')
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
                if field == 'y': offset_y = byte_offset
                if field == 'z': offset_z = byte_offset
                byte_offset += size

        data_start = f.tell()
        chunk_size = 2000000
        step = max(1, total_points // target_points)
        all_xyz = []
        for start in range(0, total_points, chunk_size):
            end = min(start + chunk_size, total_points)
            n = end - start
            f.seek(data_start + start * point_size)
            raw = f.read(n * point_size)
            indices = np.arange(start, end, step, dtype=np.int64)
            for idx in indices:
                loc = (idx - start) * point_size
                if loc + point_size > len(raw): break
                x = struct.unpack_from('f', raw, offset_x + loc)[0]
                y = struct.unpack_from('f', raw, offset_y + loc)[0]
                z = struct.unpack_from('f', raw, offset_z + loc)[0]
                all_xyz.append([x, y, z])
        return np.array(all_xyz, dtype=np.float32)

def main():
    print(f"Reading {PCD_PATH} ...")
    xyz = read_pcd_binary_stream(PCD_PATH, TARGET_SAMPLE)
    print(f"Sampled {len(xyz)} points")

    z = xyz[:, 2]
    in_range = (z >= Z_MIN) & (z <= Z_MAX)
    out_range = ~in_range
    n_in, n_out = np.sum(in_range), np.sum(out_range)
    ratio = 100.0 * n_in / len(xyz)
    print(f"Z range: [{z.min():.2f}, {z.max():.2f}]")
    print(f"Retained [{Z_MIN}, {Z_MAX}]: {n_in} ({ratio:.1f}%)  -> 绿色")
    print(f"Discarded: {n_out} ({100-ratio:.1f}%)  -> 红色")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

    # --- 子图1: 全量 XY 投影, discard(红)在底层, retain(绿)在上层 ---
    if n_out > 0:
        ax1.scatter(xyz[out_range, 0], xyz[out_range, 1],
                    c='red', s=0.3, alpha=0.4, label=f'Discarded ({n_out})')
    if n_in > 0:
        ax1.scatter(xyz[in_range, 0], xyz[in_range, 1],
                    c='green', s=0.3, alpha=0.7, label=f'Retained ({n_in})')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)')
    ax1.set_title(f'Full XY projection\nZ filter [{Z_MIN}, {Z_MAX}]')
    ax1.legend(markerscale=12); ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # --- 子图2: 仅保留的点 ---
    if n_in > 0:
        ax2.scatter(xyz[in_range, 0], xyz[in_range, 1],
                    c='green', s=0.5, alpha=0.8)
    ax2.set_xlabel('X (m)'); ax2.set_ylabel('Y (m)')
    ax2.set_title(f'Retained points only (z ∈ [{Z_MIN}, {Z_MAX}])\n{n_in} points')
    ax2.set_aspect('equal'); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = "/home/shui/LY/lite_cog/system/map/scans_z_range_2d.png"
    plt.savefig(out_path, dpi=150)
    print(f"Saved to {out_path}")

if __name__ == '__main__':
    main()
