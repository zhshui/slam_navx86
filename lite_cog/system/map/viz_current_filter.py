#!/usr/bin/env python3
"""
Vedo 可视化 current.pcd，用颜色区分 Z 轴过滤的结果，保存为 PNG。

🟢 绿色 = 保留的点 (Z 在范围内, "没有被过滤的点")
🔴 红色 = 被丢弃的点 (Z 在范围外, "被过滤掉的点")
🟡 黄色/橙色半透明平面 = Z filter 上下界
🔵 蓝色半透明平面 = Z=0 参考面

用法:
    python3 viz_current_filter.py [--z-min Z_MIN] [--z-max Z_MAX]
"""

import struct
import numpy as np
import os

PCD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'current.pcd')
OUT_DIR  = os.path.dirname(os.path.abspath(__file__))

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
        chunk_size = 2_000_000
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

# ---------------------------------------------------------------------------
def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--z-min', type=float, default=-0.1)
    parser.add_argument('--z-max', type=float, default=0.6)
    parser.add_argument('--downsample', type=int, default=800_000)
    args = parser.parse_args()

    print(f"Loading {PCD_PATH} ...")
    xyz = read_pcd_xyz(PCD_PATH)
    print(f"Loaded {xyz.shape[0]:,} points")

    z = xyz[:, 2]
    print(f"Z range: [{z.min():.2f}, {z.max():.2f}] m")
    print(f"Z filter: [{args.z_min}, {args.z_max}] m")

    in_range = (z >= args.z_min) & (z <= args.z_max)
    n_retained = np.sum(in_range)
    n_discarded = xyz.shape[0] - n_retained
    pct = 100.0 * n_retained / xyz.shape[0]
    print(f"  Retained  (green): {n_retained:,}  ({pct:.1f}%)")
    print(f"  Discarded (red):   {n_discarded:,}  ({100-pct:.1f}%)")

    # Downsample
    if args.downsample > 0 and args.downsample < xyz.shape[0]:
        rng = np.random.default_rng(42)
        idx = rng.choice(xyz.shape[0], args.downsample, replace=False)
        xyz = xyz[idx]
        in_range = in_range[idx]
        print(f"Display: {args.downsample:,} pts")

    retained_xyz = xyz[in_range]
    discarded_xyz = xyz[~in_range]

    # --- Build vedo scene ---
    from vedo import Points, show, Text2D
    import vedo

    actors = []
    x_mid, y_mid = xyz[:,0].mean(), xyz[:,1].mean()
    radius = max(xyz[:,0].ptp(), xyz[:,1].ptp()) * 0.5

    # Discarded = red (small, transparent, behind)
    if len(discarded_xyz) > 0:
        pts = Points(discarded_xyz, r=3, c='#cc3333', alpha=0.20)
        actors.append(pts)

    # Retained = green (brighter, on top)
    if len(retained_xyz) > 0:
        pts = Points(retained_xyz, r=4, c='#44ff44', alpha=0.65)
        actors.append(pts)

    # Z=0 reference plane
    actors.append(vedo.shapes.Disc(pos=(x_mid, y_mid, 0), r1=radius, r2=0,
                                    c='#4488ff', alpha=0.06, res=4))
    # Filter boundary planes
    actors.append(vedo.shapes.Disc(pos=(x_mid, y_mid, args.z_min), r1=radius, r2=0,
                                    c='yellow', alpha=0.05, res=4))
    actors.append(vedo.shapes.Disc(pos=(x_mid, y_mid, args.z_max), r1=radius, r2=0,
                                    c='orange', alpha=0.05, res=4))

    # Info text
    info = (
        f"current.pcd  |  {xyz.shape[0]:,} pts\n"
        f"Z filter: [{args.z_min}, {args.z_max}] m\n"
        f"Retained (green): {n_retained:,} ({pct:.1f}%)\n"
        f"Discarded (red):  {n_discarded:,} ({100-pct:.1f}%)\n"
        f"Data Z: [{z.min():.2f}, {z.max():.2f}] m"
    )
    txt = Text2D(info, pos='top-left', c='white', bg='black', alpha=0.7, font='Courier')
    actors.append(txt)

    # ---- Save screenshots ----
    # Top-down view
    out_top = os.path.join(OUT_DIR, 'current_filter_topdown.png')
    print(f"\nSaving top-down view to {out_top} ...")
    vp_top = vedo.Plotter(offscreen=True, bg='blackboard', size=(1800, 1000))
    cam_top = {'pos': (x_mid, y_mid, max(40, z.max() + 25)),
               'focalPoint': (x_mid, y_mid, 0),
               'viewup': (0, 1, 0)}
    vp_top.show(actors, viewup='z', camera=cam_top, axes=1)
    vp_top.screenshot(out_top)
    vp_top.close()
    print(f"  -> {out_top}")

    # Perspective view
    out_per = os.path.join(OUT_DIR, 'current_filter_perspective.png')
    print(f"Saving perspective view to {out_per} ...")
    vp_per = vedo.Plotter(offscreen=True, bg='blackboard', size=(1800, 1000))
    cam_per = {'pos': (x_mid - 20, y_mid - 40, 25),
               'focalPoint': (x_mid, y_mid, 0),
               'viewup': (0, 0, 1)}
    vp_per.show(actors, viewup='z', camera=cam_per, axes=1)
    vp_per.screenshot(out_per)
    vp_per.close()
    print(f"  -> {out_per}")

    # Side view (from +Y)
    out_side = os.path.join(OUT_DIR, 'current_filter_side.png')
    print(f"Saving side view to {out_side} ...")
    vp_side = vedo.Plotter(offscreen=True, bg='blackboard', size=(1800, 1000))
    cam_side = {'pos': (x_mid, y_mid + max(40, y_mid * 2), z.mean()),
                'focalPoint': (x_mid, y_mid, 0),
                'viewup': (0, 0, 1)}
    vp_side.show(actors, viewup='z', camera=cam_side, axes=1)
    vp_side.screenshot(out_side)
    vp_side.close()
    print(f"  -> {out_side}")

    print("\nDone! 3 views saved.")


if __name__ == '__main__':
    main()
