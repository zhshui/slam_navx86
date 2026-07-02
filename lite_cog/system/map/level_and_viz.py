#!/usr/bin/env python3
"""
用 SVD (PCA) 拟合地面平面，修正 Z 轴倾斜，然后可视化。

方法：
  1. 取点云 Z 轴底部的点作为地面候选（更鲁棒）
  2. 用 SVD 拟合平面（PCA — 最小法向量对应平面法向量）
  3. 旋转使地面法向量对齐 (0,0,1)
  4. 保存修正后的 PCD
  5. 用 vedo 可视化对比 (修正前 / 修正后)

用法:
    python3 level_and_viz.py <input.pcd> [--ground-percentile 15] [--downsample N]
"""

import argparse
import numpy as np
import struct
import os
import sys

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
        return np.vstack(xyz_list).astype(np.float64)


def save_pcd_xyz(filepath, xyz):
    """Save xyz (N,3) as binary PCD."""
    with open(filepath, 'wb') as f:
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {len(xyz)}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(xyz)}\n"
            "DATA binary\n"
        )
        f.write(header.encode('ascii'))
        xyz.astype(np.float32).tofile(f)


def fit_ground_plane_svd(xyz, ground_percentile=15):
    """
    Fit a plane to ground points using SVD (PCA).

    1. Take the bottom `ground_percentile`% of points by Z as ground candidates
    2. Compute their centroid
    3. SVD on the centered points → the singular vector with smallest
       singular value is the plane normal
    """
    z = xyz[:, 2]
    z_thresh = np.percentile(z, ground_percentile)
    ground_mask = z <= z_thresh
    ground_pts = xyz[ground_mask]

    print(f"Ground candidates (Z <= P{ground_percentile}={z_thresh:.3f}): "
          f"{ground_pts.shape[0]:,} / {xyz.shape[0]:,} "
          f"({100 * ground_pts.shape[0] / xyz.shape[0]:.1f}%)")

    if ground_pts.shape[0] < 10:
        raise RuntimeError("Too few ground points")

    # Center
    centroid = ground_pts.mean(axis=0)
    centered = ground_pts - centroid

    # SVD
    U, S, Vt = np.linalg.svd(centered, full_matrices=False)
    normal = Vt[-1]  # smallest singular value → plane normal

    # Ensure normal points upward (positive Z)
    if normal[2] < 0:
        normal = -normal

    # Compute tilt angle from Z axis
    tilt_rad = np.arccos(np.clip(np.dot(normal, [0, 0, 1]), -1, 1))
    tilt_deg = np.degrees(tilt_rad)

    print(f"Ground centroid: [{centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f}]")
    print(f"Plane normal:    [{normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f}]")
    print(f"Tilt from Z:     {tilt_deg:.2f}°")
    print(f"SVD singular values: {S}")

    return normal, centroid, tilt_deg


def compute_rotation(normal):
    """Compute rotation matrix that maps `normal` → [0, 0, 1]."""
    target = np.array([0.0, 0.0, 1.0])
    axis = np.cross(target, normal)
    axis_norm = np.linalg.norm(axis)

    if axis_norm < 1e-9:
        return np.eye(3)  # Already level

    axis = axis / axis_norm
    cos_angle = np.dot(normal, target)
    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))

    # Rodrigues rotation formula
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K

    print(f"Rotation axis:  [{axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}]")
    print(f"Rotation angle: {np.degrees(angle):.2f}°")

    return R


def main():
    parser = argparse.ArgumentParser(description="PCA ground leveling + visualization")
    parser.add_argument('input', help='Input PCD file')
    parser.add_argument('--ground-percentile', type=float, default=15,
                        help='Percentile of Z to use as ground (default: 15)')
    parser.add_argument('--downsample', type=int, default=500000,
                        help='Display downsample (default: 500000)')
    parser.add_argument('--output', type=str, default='',
                        help='Save leveled PCD to file')
    parser.add_argument('--z-min', type=float, default=-0.1)
    parser.add_argument('--z-max', type=float, default=0.6)
    args = parser.parse_args()

    # --- Load ---
    print(f"Loading {args.input} ...")
    xyz = read_pcd_xyz(args.input)
    print(f"Loaded {xyz.shape[0]:,} points")
    print(f"Z range: [{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}]")

    # --- Fit ground plane ---
    normal, centroid, tilt_deg = fit_ground_plane_svd(xyz, args.ground_percentile)

    # --- Compute rotation ---
    R = compute_rotation(normal)

    # --- Apply rotation to entire cloud ---
    xyz_leveled = (xyz - centroid) @ R.T + centroid
    print(f"\nAfter leveling:")
    print(f"Z range: [{xyz_leveled[:,2].min():.2f}, {xyz_leveled[:,2].max():.2f}]")

    # --- Verify: re-fit to leveled data ---
    normal2, _, tilt2 = fit_ground_plane_svd(xyz_leveled, args.ground_percentile)
    if tilt2 < 0.1:
        print("✅ Leveling verified: ground is now flat (tilt < 0.1°)")
    else:
        print(f"⚠  Residual tilt: {tilt2:.3f}° — may need manual adjustment")

    # --- Save leveled PCD ---
    if args.output:
        outpath = args.output
        if not outpath.endswith('.pcd'):
            outpath += '.pcd'
        save_pcd_xyz(outpath, xyz_leveled)
        print(f"\nLeveled PCD saved to: {outpath}")
    else:
        # Auto-name
        base = os.path.splitext(args.input)[0]
        outpath = base + '_leveled_svd.pcd'
        save_pcd_xyz(outpath, xyz_leveled)
        print(f"\nLeveled PCD saved to: {outpath}")

    # --- Prepare display sample ---
    if args.downsample > 0 and args.downsample < xyz.shape[0]:
        rng = np.random.default_rng(42)
        idx = rng.choice(xyz.shape[0], min(args.downsample, xyz.shape[0]), replace=False)
        xyz_disp = xyz[idx]
        xyz_lev_disp = xyz_leveled[idx]
    else:
        xyz_disp = xyz
        xyz_lev_disp = xyz_leveled

    # --- Z-filter coloring for leveled data ---
    z_lev = xyz_lev_disp[:, 2]
    in_range = (z_lev >= args.z_min) & (z_lev <= args.z_max)
    n_ret = in_range.sum()
    print(f"\nZ filter [{args.z_min}, {args.z_max}] on leveled data: "
          f"{n_ret:,} retained / {len(z_lev):,} ({100*n_ret/len(z_lev):.1f}%)")

    # --- Visualization ---
    from vedo import Points, Plotter, Text2D

    # Left: original
    pc_orig = Points(xyz_disp, r=4, c='white')
    pc_orig.name = "Original"
    pc_orig.alpha(0.3)

    # Right: leveled with Z-filter color
    retained = xyz_lev_disp[in_range]
    discarded = xyz_lev_disp[~in_range]

    actors_orig = [pc_orig]
    actors_lev = []

    if len(discarded) > 0:
        pd = Points(discarded, r=2, c='red')
        pd.name = "Discarded"
        pd.alpha(0.35)
        actors_lev.append(pd)

    if len(retained) > 0:
        pr = Points(retained, r=5, c='green4')
        pr.name = "Retained"
        pr.alpha(0.85)
        actors_lev.append(pr)

    # Create two side-by-side plots
    plotter = Plotter(title="Left: Original  |  Right: Leveled + Z-filter",
                       bg='blackboard', size=(2000, 950), shape=(1, 2))

    # Info text
    info1 = (
        f"ORIGINAL\n"
        f"Ground tilt: {tilt_deg:.2f}°\n"
        f"Normal: [{normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}]\n"
        f"{len(xyz_disp):,} points"
    )
    info2 = (
        f"LEVELED + Z-FILTER [{args.z_min}, {args.z_max}]\n"
        f"Retained (green): {n_ret:,} ({100*n_ret/len(z_lev):.1f}%)\n"
        f"Discarded (red): {len(z_lev)-n_ret:,}\n"
        f"Residual tilt: {tilt2:.2f}°"
    )

    txt1 = Text2D(info1, pos='top-left', c='white', bg='black', alpha=0.7, font='Courier')
    txt2 = Text2D(info2, pos='top-left', c='white', bg='black', alpha=0.7, font='Courier')

    plotter.at(0).show(actors_orig, txt1, axes=1, viewup='z')
    plotter.at(1).show(actors_lev, txt2, axes=1, viewup='z')

    plotter.interactive().close()


if __name__ == '__main__':
    main()
