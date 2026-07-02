#!/usr/bin/env python3
"""
Open3D 可视化 PCD Z 轴过滤 — 流畅的 GPU 加速渲染

🟢 绿色 = 保留的点 (Z 在范围内, "没有被过滤的点")
🔴 红色 = 被丢弃的点 (Z 在范围外, "被过滤掉的点")

百万级点云也能流畅旋转/缩放/平移。

用法:
    python3 visualize_pcd_filter_o3d.py <input.pcd> [options]

Options:
    --z-min Z          Z 下界 (默认: -0.1)
    --z-max Z          Z 上界 (默认: 0.6)
    --downsample N     随机降采样到 N 点 (默认: 全部, 建议 1000000)
    --topdown          初始俯视视角
    --save FILE        保存彩色点云到 FILE.pcd

交互操作:
    鼠标左键拖拽        旋转
    Ctrl + 鼠标左键     平移
    滚轮                缩放
    鼠标中键拖拽        平移
    Shift + 鼠标左键    旋转
    r                   重置视角
    f                   适配窗口
    h                   显示帮助
    Esc / q             退出
"""

import argparse
import numpy as np
import struct
import os
import sys

# ---------------------------------------------------------------------------
# Fast PCD binary parser (handles xyz only, much faster than open3d's reader
# for huge files because it streams)
# ---------------------------------------------------------------------------
def read_pcd_xyz(filepath):
    """Read a binary PCD file and return xyz as (N,3) float32 array."""
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

            chunk_xyz = np.zeros((n, 3), dtype=np.float32)
            # Read x,y,z interleaved via struct
            fmt = '<' + 'f' * (n * 3)
            # Build interleaved buffer
            interleaved = np.zeros(n * 3, dtype=np.float32)
            for i in range(n):
                base = i * point_size
                interleaved[i * 3 + 0] = struct.unpack_from('f', raw, offset_x + base)[0]
                interleaved[i * 3 + 1] = struct.unpack_from('f', raw, offset_y + base)[0]
                interleaved[i * 3 + 2] = struct.unpack_from('f', raw, offset_z + base)[0]
            xyz_list.append(interleaved.reshape(-1, 3))

        return np.vstack(xyz_list).astype(np.float32)


def main():
    parser = argparse.ArgumentParser(description="Open3D Z-filter point cloud visualization")
    parser.add_argument('input', help='Input PCD file path')
    parser.add_argument('--z-min', type=float, default=-0.1, help='Lower Z bound (default: -0.1)')
    parser.add_argument('--z-max', type=float, default=0.6, help='Upper Z bound (default: 0.6)')
    parser.add_argument('--downsample', type=int, default=0,
                        help='Random downsample to N points (0=show all)')
    parser.add_argument('--topdown', action='store_true', help='Start with top-down view')
    parser.add_argument('--save', type=str, default='', help='Save colored point cloud to FILE')
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

    # Filter
    in_range = (z >= args.z_min) & (z <= args.z_max)
    n_retained = np.sum(in_range)
    n_discarded = xyz.shape[0] - n_retained
    ratio = 100.0 * n_retained / xyz.shape[0]
    print(f"Retained (green):  {n_retained:,}  ({ratio:.1f}%)")
    print(f"Discarded (red):   {n_discarded:,}  ({100-ratio:.1f}%)")

    # Downsample if requested
    if args.downsample > 0 and args.downsample < xyz.shape[0]:
        rng = np.random.default_rng(42)
        idx = rng.choice(xyz.shape[0], args.downsample, replace=False)
        xyz = xyz[idx]
        in_range = in_range[idx]
        print(f"Downsampled to {args.downsample:,} points for display")
    else:
        # If no downsampling requested but >2M points, suggest
        if xyz.shape[0] > 2_000_000:
            print(f"⚠  {xyz.shape[0]:,} points — may be slow. "
                  f"Try --downsample 1000000 for smoother interaction.")

    # Build colors: green=retained, red=discarded
    colors = np.zeros((xyz.shape[0], 3), dtype=np.float64)
    colors[in_range, 1] = 1.0    # green
    colors[~in_range, 0] = 1.0   # red

    # Create Open3D point cloud
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Save if requested
    if args.save:
        save_path = args.save
        if not save_path.endswith('.pcd'):
            save_path += '.pcd'
        o3d.io.write_point_cloud(save_path, pcd)
        print(f"Colored PCD saved to: {save_path}")

    # --- Launch interactive viewer ---
    print("\n=== Interactive viewer ===")
    print("  Left-drag       Rotate")
    print("  Ctrl+Left-drag  Pan")
    print("  Scroll          Zoom")
    print("  r               Reset view")
    print("  h               Help")
    print("  Esc / q         Quit")
    print()

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"Z Filter [{args.z_min}, {args.z_max}] — Green=Retained  Red=Discarded",
                      width=1600, height=900)

    vis.add_geometry(pcd)

    # Set dark background
    opt = vis.get_render_option()
    opt.background_color = np.array([0.05, 0.05, 0.05])
    opt.point_size = 1.5

    # Key callbacks
    def reset_view(vis):
        ctr = vis.get_view_control()
        if args.topdown:
            # Top-down view: looking down from above
            ctr.set_front([0, 0, -1])
            ctr.set_up([0, 1, 0])
            ctr.set_lookat([0, 0, 0])
        else:
            ctr.set_front([0, 0, -1])
            ctr.set_up([0, -1, 0])  # y-up
            ctr.set_lookat([0, 0, 0])
        ctr.set_zoom(0.05)
        return False

    vis.register_key_callback(ord('R'), reset_view)
    vis.register_key_callback(ord('r'), reset_view)

    # Initial top-down if requested
    if args.topdown:
        reset_view(vis)

    vis.run()
    vis.destroy_window()


if __name__ == '__main__':
    main()
