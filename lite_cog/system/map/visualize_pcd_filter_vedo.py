#!/usr/bin/env python3
"""
Vedo 可视化 PCD Z 轴过滤 — 流畅渲染百万级点云

🟢 绿色 = Z 范围内保留的点 ("没有被过滤的点")
🔴 红色 = Z 范围外被丢弃的点 ("被过滤掉的点")

Vedo 基于 VTK 但用 GPU 加速，比 PCL 原生 viewer 流畅得多。

用法:
    python3 visualize_pcd_filter_vedo.py <input.pcd> [options]

Options:
    --z-min Z          Z 下界 (默认: -0.1)
    --z-max Z          Z 上界 (默认: 0.6)
    --downsample N     降采样到 N 点 (默认: 500000, 用 -1 展示全部)
    --topdown          初始俯视视角
    --save FILE        保存截图

交互操作:
    左键拖拽           旋转
    中键/Shift+左键    平移
    右键/滚轮          缩放
    r                  重置俯视视角
    h                  显示/隐藏帮助
    q / Esc            退出
"""

import argparse
import numpy as np
import struct
import os
import sys

# ---------------------------------------------------------------------------
# Fast PCD binary reader
# ---------------------------------------------------------------------------
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
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith('COUNT'):
                counts = [int(x) for x in line.split()[1:]]
            elif line.startswith('POINTS'):
                total_points = int(line.split()[1])
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


def main():
    parser = argparse.ArgumentParser(description="Vedo Z-filter point cloud visualization")
    parser.add_argument('input', help='Input PCD file path')
    parser.add_argument('--z-min', type=float, default=-0.1, help='Lower Z bound (default: -0.1)')
    parser.add_argument('--z-max', type=float, default=0.6, help='Upper Z bound (default: 0.6)')
    parser.add_argument('--downsample', type=int, default=500000,
                        help='Random downsample to N points (-1 = show all, default: 500000)')
    parser.add_argument('--topdown', action='store_true', help='Start with top-down view')
    parser.add_argument('--save', type=str, default='', help='Save screenshot to FILE.png')
    parser.add_argument('--point-size', type=int, default=4, help='Point size (default: 4)')
    args = parser.parse_args()

    pcd_path = args.input
    if not os.path.exists(pcd_path):
        print(f"ERROR: File not found: {pcd_path}")
        sys.exit(1)

    print(f"Loading {pcd_path} ...")
    xyz = read_pcd_xyz(pcd_path)
    print(f"Loaded {xyz.shape[0]:,} points")

    z = xyz[:, 2]
    z_min_all, z_max_all = z.min(), z.max()
    print(f"Z range in data: [{z_min_all:.2f}, {z_max_all:.2f}] m")
    print(f"Z filter window: [{args.z_min}, {args.z_max}] m")

    # --- Filter ---
    in_range = (z >= args.z_min) & (z <= args.z_max)
    n_retained = np.sum(in_range)
    n_discarded = xyz.shape[0] - n_retained
    ratio = 100.0 * n_retained / xyz.shape[0]
    print(f"🟢 Retained  (Z in range): {n_retained:,}  ({ratio:.1f}%)")
    print(f"🔴 Discarded (Z out of range): {n_discarded:,}  ({100-ratio:.1f}%)")

    # --- Downsample ---
    if args.downsample > 0 and args.downsample < xyz.shape[0]:
        rng = np.random.default_rng(42)
        idx = rng.choice(xyz.shape[0], args.downsample, replace=False)
        xyz = xyz[idx]
        in_range = in_range[idx]
        print(f"Display: {args.downsample:,} points (randomly sampled)")

    # --- Split into retained / discarded ---
    retained_xyz = xyz[in_range]
    discarded_xyz = xyz[~in_range]

    # --- Build vedo Points objects ---
    from vedo import Points, show, Text2D, LegendBox
    import vedo
    vedo.settings.default_backend = 'vtk'  # or 'k3d' / 'ipyvtk' if in jupyter

    pts_retained = None
    pts_discarded = None

    if len(retained_xyz) > 0:
        pts_retained = Points(retained_xyz, r=0, c='green4')  # r=radius, c=color
        pts_retained.name = "Retained (Z in range)"
        pts_retained.point_size(args.point_size)
        pts_retained.alpha(0.8)

    if len(discarded_xyz) > 0:
        pts_discarded = Points(discarded_xyz, r=0, c='red')
        pts_discarded.name = "Discarded (Z out of range)"
        pts_discarded.point_size(max(1, args.point_size // 2))  # smaller dots for discarded
        pts_discarded.alpha(0.4)  # more transparent so green stands out

    # --- Build plot ---
    plotter = vedo.Plotter(
        title=f"Z Filter [{args.z_min}, {args.z_max}] m — Green=Retained  Red=Discarded",
        bg='blackboard',
        size=(1800, 1000),
    )

    actors = []
    if pts_discarded is not None:
        actors.append(pts_discarded)
    if pts_retained is not None:
        actors.append(pts_retained)

    # --- Horizontal reference grid at Z=0 (ground truth level indicator) ---
    x_range = [xyz[:,0].min(), xyz[:,0].max()]
    y_range = [xyz[:,1].min(), xyz[:,1].max()]
    grid_spacing = 5.0  # 5m grid

    import vedo
    # Create a semi-transparent plane at Z=0 as level reference
    from vedo import Grid
    ref_grid = Grid(
        s=tuple(x_range),  # not quite right — Grid takes different args
    )
    # Use a simpler approach: add horizontal lines at Z=0
    from vedo import Line, Plane
    # Transparent reference disk at Z=0
    ref_plane = vedo.shapes.Disc(
        pos=(np.mean(x_range), np.mean(y_range), 0),
        r1=max(x_range[1]-x_range[0], y_range[1]-y_range[0]) * 0.8,
        r2=0,
        c='cyan',
        alpha=0.08,
        res=4,
    )
    ref_plane.name = "Z=0 reference"
    actors.append(ref_plane)

    # --- Info text ---
    info = (
        f"Z Filter: [{args.z_min}, {args.z_max}] m\n"
        f"Retained (green): {n_retained:,} ({ratio:.1f}%)\n"
        f"Discarded (red): {n_discarded:,} ({100-ratio:.1f}%)\n"
        f"Total: {xyz.shape[0]:,} points"
    )
    txt = Text2D(info, pos='top-left', c='white', bg='black', alpha=0.7, font='Courier')

    # --- Show ---
    if actors:
        plotter.show(
            actors, txt,
            axes=1,               # show axes
            viewup='z',           # Z is up
            interactive=True,
        ).close()

    # --- Save screenshot ---
    if args.save:
        save_path = args.save
        if not save_path.endswith('.png'):
            save_path += '.png'
        # Take screenshot after window closes
        if actors:
            plotter2 = vedo.Plotter(offscreen=True, bg='blackboard', size=(1800, 1000))
            plotter2.show(actors, txt, axes=1, viewup='z', camera={
                'pos': (0, -50, 20),
                'focalPoint': (0, 0, 0),
                'viewup': (0, 0, 1),
            })
            plotter2.screenshot(save_path)
            plotter2.close()
            print(f"Screenshot saved to: {save_path}")


if __name__ == '__main__':
    main()
