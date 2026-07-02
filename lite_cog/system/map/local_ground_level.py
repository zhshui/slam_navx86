#!/usr/bin/env python3
"""
纯 numpy 局部地面归一化：分块估计地面 Z，减去局部地面。
无 scipy 依赖。
"""
import struct
import numpy as np
import sys, os, argparse

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
        ox = oy = oz = 0; bo = 0
        for field, size, count in zip(fields, sizes, counts):
            for _ in range(count):
                if field == 'x': ox = bo
                elif field == 'y': oy = bo
                elif field == 'z': oz = bo
                bo += size
        data_start = f.tell()
        chunk_size = 2000000
        xyz_list = []
        for start in range(0, total_points, chunk_size):
            end = min(start + chunk_size, total_points)
            n = end - start
            f.seek(data_start + start * point_size)
            raw = f.read(n * point_size)
            arr = np.zeros(n * 3, dtype=np.float32)
            for i in range(n):
                base = i * point_size
                arr[i*3+0] = struct.unpack_from('f', raw, ox+base)[0]
                arr[i*3+1] = struct.unpack_from('f', raw, oy+base)[0]
                arr[i*3+2] = struct.unpack_from('f', raw, oz+base)[0]
            xyz_list.append(arr.reshape(-1, 3))
        return np.vstack(xyz_list).astype(np.float64)

def save_pcd_xyz(filepath, xyz):
    with open(filepath, 'wb') as f:
        header = (
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            f"COUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(xyz)}\nDATA binary\n"
        )
        f.write(header.encode('ascii'))
        xyz.astype(np.float32).tofile(f)

def box_blur(grid, radius):
    """2D box blur over (2r+1)×(2r+1) window, handling NaN."""
    h, w = grid.shape
    r = int(radius)
    if r < 1:
        r = 1
    result = np.full_like(grid, np.nan)
    for i in range(h):
        i0, i1 = max(0, i-r), min(h, i+r+1)
        for j in range(w):
            j0, j1 = max(0, j-r), min(w, j+r+1)
            patch = grid[i0:i1, j0:j1]
            mask = ~np.isnan(patch)
            if mask.sum() > 0:
                result[i, j] = patch[mask].mean()
    return result

def interp_bilinear(ground_grid, x_centers, y_centers, xq, yq):
    """Bilinear interpolation on regular grid. Returns interpolated values."""
    x_min, x_max = x_centers[0] - (x_centers[1]-x_centers[0])/2, x_centers[-1] + (x_centers[1]-x_centers[0])/2
    y_min, y_max = y_centers[0] - (y_centers[1]-y_centers[0])/2, y_centers[-1] + (y_centers[1]-y_centers[0])/2
    dx = x_centers[1] - x_centers[0]
    dy = y_centers[1] - y_centers[0]

    fx = (xq - x_min) / dx
    fy = (yq - y_min) / dy
    ix = np.clip(np.floor(fx).astype(int), 0, len(x_centers)-1)
    iy = np.clip(np.floor(fy).astype(int), 0, len(y_centers)-1)
    ix1 = np.clip(ix + 1, 0, len(x_centers)-1)
    iy1 = np.clip(iy + 1, 0, len(y_centers)-1)

    wx = fx - ix
    wy = fy - iy
    wx = np.clip(wx, 0, 1)
    wy = np.clip(wy, 0, 1)

    z00 = ground_grid[iy, ix]
    z01 = ground_grid[iy, ix1]
    z10 = ground_grid[iy1, ix]
    z11 = ground_grid[iy1, ix1]

    z0 = z00 * (1 - wx) + z01 * wx
    z1 = z10 * (1 - wx) + z11 * wx
    return z0 * (1 - wy) + z1 * wy

def main():
    parser = argparse.ArgumentParser(description="Local ground leveling (pure numpy)")
    parser.add_argument('input', help='Input PCD')
    parser.add_argument('--grid', type=float, default=1.0)
    parser.add_argument('--ground-percentile', type=float, default=5)
    parser.add_argument('--smooth-radius', type=float, default=2.0, help='Smooth radius in grid cells')
    parser.add_argument('--output', type=str, default='')
    parser.add_argument('--z-min', type=float, default=0.0)
    parser.add_argument('--z-max', type=float, default=0.6)
    args = parser.parse_args()

    print(f"Loading {args.input} ...")
    xyz = read_pcd_xyz(args.input)
    print(f"Loaded {xyz.shape[0]:,} points")

    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()
    print(f"XY: X[{x_min:.1f}, {x_max:.1f}] Y[{y_min:.1f}, {y_max:.1f}]")

    grid = args.grid
    xbins = int(np.ceil((x_max - x_min) / grid)) + 1
    ybins = int(np.ceil((y_max - y_min) / grid)) + 1
    print(f"Grid: {xbins}x{ybins} ({grid}m)")

    xi = np.clip(((x - x_min) / grid).astype(np.int32), 0, xbins - 1)
    yi = np.clip(((y - y_min) / grid).astype(np.int32), 0, ybins - 1)
    linear_idx = yi * xbins + xi
    n_cells = xbins * ybins

    # Sort by cell then Z
    sorter = np.lexsort((z, linear_idx))
    sorted_z = z[sorter]
    sorted_cell = linear_idx[sorter]
    cell_starts = np.searchsorted(sorted_cell, np.arange(n_cells + 1))

    ground_z = np.full(n_cells, np.nan)
    pct = args.ground_percentile
    for c in range(n_cells):
        lo, hi = cell_starts[c], cell_starts[c + 1]
        n = hi - lo
        if n < 30:
            continue
        k = max(1, int(n * pct / 100))
        ground_z[c] = sorted_z[lo:lo + k].mean()

    ground_grid = ground_z.reshape(ybins, xbins)
    valid = ~np.isnan(ground_grid)
    print(f"Valid cells: {valid.sum()}/{n_cells} ({100*valid.sum()/n_cells:.1f}%)")
    print(f"Raw ground Z: [{np.nanmin(ground_grid):.3f}, {np.nanmax(ground_grid):.3f}]")

    # Nearest-neighbor fill NaN (fast: fill with nearest valid value per row, then per column)
    if valid.sum() < n_cells:
        print("Filling NaN cells ...")
        # Forward/backward fill along rows
        for _ in range(2):
            for i in range(ybins):
                row = ground_grid[i]
                nans = np.isnan(row)
                if not nans.any() or nans.all():
                    continue
                # Fill forward
                last_val = None
                for j in range(xbins):
                    if not np.isnan(row[j]):
                        last_val = row[j]
                    elif last_val is not None:
                        row[j] = last_val
                # Fill backward
                last_val = None
                for j in range(xbins-1, -1, -1):
                    if not np.isnan(row[j]):
                        last_val = row[j]
                    elif last_val is not None:
                        row[j] = last_val
            # Same along columns
            for j in range(xbins):
                col = ground_grid[:, j]
                nans = np.isnan(col)
                if not nans.any() or nans.all():
                    continue
                last_val = None
                for i in range(ybins):
                    if not np.isnan(col[i]):
                        last_val = col[i]
                    elif last_val is not None:
                        col[i] = last_val
                last_val = None
                for i in range(ybins-1, -1, -1):
                    if not np.isnan(col[i]):
                        last_val = col[i]
                    elif last_val is not None:
                        col[i] = last_val
    print(f"Filled ground Z: [{np.nanmin(ground_grid):.3f}, {np.nanmax(ground_grid):.3f}]")

    # Smooth
    print(f"Smoothing (radius={args.smooth_radius} cells) ...")
    ground_smooth = box_blur(ground_grid, args.smooth_radius)
    print(f"Smoothed ground Z: [{np.nanmin(ground_smooth):.3f}, {np.nanmax(ground_smooth):.3f}]")

    # Cell centers
    x_centers = x_min + (np.arange(xbins) + 0.5) * grid
    y_centers = y_min + (np.arange(ybins) + 0.5) * grid

    # Subtract local ground
    print("Subtracting local ground ...")
    chunk = 2000000
    z_new = np.empty_like(z)
    for start in range(0, len(xyz), chunk):
        end = min(start + chunk, len(xyz))
        local_g = interp_bilinear(ground_smooth, x_centers, y_centers, x[start:end], y[start:end])
        z_new[start:end] = z[start:end] - local_g

    print(f"Z after local leveling: [{z_new.min():.2f}, {z_new.max():.2f}]")
    in_range = (z_new >= args.z_min) & (z_new <= args.z_max)
    print(f"Z filter [{args.z_min},{args.z_max}]: {in_range.sum():,}/{len(z_new):,} ({100*in_range.sum()/len(z_new):.1f}%)")

    # Save
    outpath = args.output
    if not outpath:
        base = os.path.splitext(args.input)[0]
        outpath = base + '_local_leveled.pcd'
    xyz_out = np.column_stack([x, y, z_new])
    save_pcd_xyz(outpath, xyz_out)
    print(f"Saved: {outpath}")

if __name__ == '__main__':
    main()
