#!/usr/bin/env python3
"""
并排对比原始 PCD 和 RANSAC 摆平后的 2D 俯视图（XY 投影）。
Z 过滤 [-0.1, 0.6]：绿色=保留，红色=丢弃。
"""
import struct, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PCD_ORIG  = "/home/robot/go2_nav/lite_cog/system/map/scans_23/scans_23.pcd"
PCD_LEVEL = "/home/robot/go2_nav/lite_cog/system/map/scans_23/scans_23_leveled.pcd"
Z_MIN, Z_MAX = -0.1, 0.6
TARGET_SAMPLE = 150000

def read_pcd_xyz(filepath, target_points):
    with open(filepath, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='replace')
            header_lines.append(line.strip())
            if line.startswith('DATA'): break
        fields, sizes, counts, total_points = [], [], [], 0
        for line in header_lines:
            if line.startswith('FIELDS'): fields = line.split()[1:]
            elif line.startswith('SIZE'): sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('COUNT'): counts = [int(x) for x in line.split()[1:]]
            elif line.startswith('POINTS'): total_points = int(line.split()[1])
        point_size = sum(s * c for s, c in zip(sizes, counts))
        ox = oy = oz = 0; bo = 0
        for field, size, count in zip(fields, sizes, counts):
            for _ in range(count):
                if field == 'x': ox = bo
                elif field == 'y': oy = bo
                elif field == 'z': oz = bo
                bo += size
        data_start = f.tell()
        step = max(1, total_points // target_points)
        xyz = []
        for i in range(0, total_points, step):
            f.seek(data_start + i * point_size)
            raw = f.read(point_size)
            x = struct.unpack_from('f', raw, ox)[0]
            y = struct.unpack_from('f', raw, oy)[0]
            z = struct.unpack_from('f', raw, oz)[0]
            xyz.append([x, y, z])
        return np.array(xyz, dtype=np.float32)

def main():
    print("Loading original PCD ...")
    orig = read_pcd_xyz(PCD_ORIG, TARGET_SAMPLE)
    print("Loading leveled PCD ...")
    levl = read_pcd_xyz(PCD_LEVEL, TARGET_SAMPLE)

    fig, axes = plt.subplots(1, 2, figsize=(20, 9))

    for ax, xyz, title, tag in [
        (axes[0], orig, "Original PCD", "orig"),
        (axes[1], levl, "RANSAC-Leveled PCD", "leveled"),
    ]:
        z = xyz[:, 2]
        in_r = (z >= Z_MIN) & (z <= Z_MAX)
        out_r = ~in_r
        n_in, n_out = np.sum(in_r), np.sum(out_r)
        r = 100.0 * n_in / len(xyz)

        print(f"\n{title}:")
        print(f"  Z range: [{z.min():.2f}, {z.max():.2f}]")
        print(f"  Retained [{Z_MIN},{Z_MAX}]: {n_in:,} ({r:.1f}%)")
        print(f"  Discarded: {n_out:,} ({100-r:.1f}%)")

        # Discarded (red) behind, retained (green) on top
        if n_out > 0:
            ax.scatter(xyz[out_r, 0], xyz[out_r, 1],
                       c='#e74c3c', s=0.4, alpha=0.35, rasterized=True,
                       label=f'Discarded ({n_out:,})')
        if n_in > 0:
            ax.scatter(xyz[in_r, 0], xyz[in_r, 1],
                       c='#2ecc71', s=0.7, alpha=0.7, rasterized=True,
                       label=f'Retained ({n_in:,}, {r:.1f}%)')

        # Reference grid lines at 5m intervals
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        ax.grid(True, alpha=0.25, linestyle='-', linewidth=0.5)
        # Thicker grid every 10m
        for v in np.arange(-100, 200, 10):
            ax.axvline(v, alpha=0.15, color='white', linewidth=0.3)
            ax.axhline(v, alpha=0.15, color='white', linewidth=0.3)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'{title}\nZ filter [{Z_MIN}, {Z_MAX}] m  —  {n_in:,} retained ({r:.1f}%)')
        ax.legend(markerscale=10, loc='upper right')
        ax.set_aspect('equal')

    plt.suptitle('Ground flatness check — RANSAC leveled vs Original',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    out = '/home/robot/go2_nav/lite_cog/system/map/scans_23/compare_leveled_2d.png'
    plt.savefig(out, dpi=200, bbox_inches='tight')
    print(f"\nSaved: {out}")

if __name__ == '__main__':
    main()
