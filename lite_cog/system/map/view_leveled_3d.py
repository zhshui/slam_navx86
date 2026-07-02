#!/usr/bin/env python3
"""3D view of leveled PCD with horizontal reference grid to verify ground flatness."""
import struct, numpy as np, argparse, os

def read_pcd_xyz(filepath, target):
    with open(filepath, 'rb') as f:
        while True:
            line = f.readline().decode('ascii', errors='replace')
            if line.startswith('DATA'): break
            if line.startswith('POINTS'): total = int(line.split()[1])
        point_size = 12
        data_start = f.tell()
        step = max(1, total // target)
        xyz = []
        for i in range(0, total, step):
            f.seek(data_start + i * point_size)
            raw = f.read(point_size)
            x = struct.unpack_from('f', raw, 0)[0]
            y = struct.unpack_from('f', raw, 4)[0]
            z = struct.unpack_from('f', raw, 8)[0]
            xyz.append([x, y, z])
        return np.array(xyz, dtype=np.float32)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('--z-min', type=float, default=-0.1)
    ap.add_argument('--z-max', type=float, default=0.6)
    ap.add_argument('--downsample', type=int, default=400000)
    args = ap.parse_args()

    xyz = read_pcd_xyz(args.pcd, args.downsample)
    z = xyz[:, 2]
    print(f"Z range: [{z.min():.2f}, {z.max():.2f}]")
    in_r = (z >= args.z_min) & (z <= args.z_max)
    n_in = in_r.sum()
    print(f"Retained [{args.z_min},{args.z_max}]: {n_in:,} / {len(z):,} ({100*n_in/len(z):.1f}%)")

    from vedo import Points, Grid, Text2D, Plotter

    retained = xyz[in_r]
    discarded = xyz[~in_r]

    actors = []

    # Horizontal reference grid at Z=0 — 5m spacing, 80m extent
    g = Grid(s=(80, 80), res=(16, 16), pos=(0, 0, 0), c='cyan', alpha=0.3)
    g.name = "Z=0 grid"
    actors.append(g)

    # Also add a grid at z-min and z-max
    g_low = Grid(s=(80, 80), res=(16, 16), pos=(0, 0, args.z_min), c='yellow', alpha=0.15)
    g_low.name = f"Z={args.z_min}"
    actors.append(g_low)

    g_high = Grid(s=(80, 80), res=(16, 16), pos=(0, 0, args.z_max), c='yellow', alpha=0.15)
    g_high.name = f"Z={args.z_max}"
    actors.append(g_high)

    # Discarded (red) — draw first so retained goes on top
    if len(discarded) > 0:
        pd = Points(discarded, r=2, c='red')
        pd.alpha(0.3)
        pd.name = "Discarded"
        actors.append(pd)

    # Retained (green)
    if len(retained) > 0:
        pr = Points(retained, r=5, c='green4')
        pr.alpha(0.75)
        pr.name = "Retained"
        actors.append(pr)

    info = (f"Z ref grids: cyan=Z0  yellow=[{args.z_min},{args.z_max}]\n"
            f"Retained: {n_in:,} ({100*n_in/len(z):.1f}%)  |  "
            f"Discarded: {len(z)-n_in:,}")

    plotter = Plotter(title="Leveled PCD — Z filter check with reference grids",
                       bg='blackboard', size=(1800, 1000))
    plotter.show(actors, Text2D(info, pos='top-left', c='white', bg='black', alpha=0.7),
                 axes=1, viewup='z', interactive=True).close()

if __name__ == '__main__':
    main()
