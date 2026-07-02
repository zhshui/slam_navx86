#!/usr/bin/env python3
"""
Render PCD to PNG images with intensity-based coloring.
Top-down view + side view.
Usage: python3 render_pcd_intensity.py [pcd_file]
"""

import sys
import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_pcd(filepath):
    with open(filepath, "rb") as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.strip() == b"DATA binary":
                break

    header_str = header.decode("ascii", errors="replace")
    fields = []
    sizes = []
    types = []
    counts = []
    points = 0

    for line in header_str.split("\n"):
        line = line.strip()
        if line.startswith("FIELDS"):
            fields = line.split()[1:]
        elif line.startswith("SIZE"):
            sizes = [int(x) for x in line.split()[1:]]
        elif line.startswith("TYPE"):
            types = line.split()[1:]
        elif line.startswith("COUNT"):
            counts = [int(x) for x in line.split()[1:]]
        elif line.startswith("POINTS"):
            points = int(line.split()[1])

    # Calculate byte offsets
    col = 0
    col_map = {}
    for i, field in enumerate(fields):
        c = counts[i] if i < len(counts) else 1
        col_map[field] = col
        col += c

    total_row_bytes = sum(
        sizes[i] * (counts[i] if i < len(counts) else 1) for i in range(len(fields))
    )
    num_cols = total_row_bytes // 4

    data_start = len(header)
    with open(filepath, "rb") as f:
        f.seek(data_start)
        raw = f.read()

    n = min(points, len(raw) // total_row_bytes)
    flat = np.frombuffer(raw[: n * total_row_bytes], dtype=np.float32).reshape((n, num_cols))

    pts = np.zeros((n, 3), dtype=np.float32)
    pts[:, 0] = flat[:, col_map["x"]]
    pts[:, 1] = flat[:, col_map["y"]]
    pts[:, 2] = flat[:, col_map["z"]]

    intensity = None
    if "intensity" in col_map:
        intensity = flat[:, col_map["intensity"]].copy()

    return pts, intensity, fields


def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else "/home/robot/go2_nav/lite_cog/system/map/current.pcd"
    pts, intensity, fields = parse_pcd(filepath)
    print(f"File: {filepath}")
    print(f"Points: {pts.shape[0]}, Fields: {fields}")
    if intensity is not None:
        print(f"Intensity: min={intensity.min():.1f}, max={intensity.max():.1f}, mean={intensity.mean():.1f}")

    has_intensity = intensity is not None

    # Determine color
    if has_intensity:
        p1, p99 = np.percentile(intensity, [1, 99])
        p1, p99 = float(p1), float(p99)
        colors = np.clip(intensity, p1, p99)
        color_label = "Intensity"
    else:
        # Color by height (Z)
        p1, p99 = float(np.percentile(pts[:, 2], [1, 99]))
        colors = np.clip(pts[:, 2], p1, p99)
        color_label = "Height (Z)"

    # Downsample for rendering speed if too many points
    n = pts.shape[0]
    if n > 300000:
        step = max(1, n // 300000)
        pts = pts[::step]
        colors = colors[::step]
        print(f"Downsampled to {pts.shape[0]} for rendering")

    fig, axes = plt.subplots(1, 2, figsize=(20, 10))
    fig.suptitle(f"PCD: {filepath.split('/')[-1]}  ({pts.shape[0]} pts, color={color_label})", fontsize=14)

    # --- Top-down view (X-Y) ---
    ax1 = axes[0]
    sc1 = ax1.scatter(pts[:, 0], pts[:, 1], c=colors, s=0.3, cmap="jet",
                      vmin=float(np.percentile(colors, 1)), vmax=float(np.percentile(colors, 99)),
                      rasterized=True)
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("Top-down View (X-Y)")
    ax1.set_aspect("equal")
    plt.colorbar(sc1, ax=ax1, label=color_label, shrink=0.8)

    # --- Side view (X-Z) ---
    ax2 = axes[1]
    sc2 = ax2.scatter(pts[:, 0], pts[:, 2], c=colors, s=0.3, cmap="jet",
                      vmin=float(np.percentile(colors, 1)), vmax=float(np.percentile(colors, 99)),
                      rasterized=True)
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Z (m)")
    ax2.set_title("Side View (X-Z)")
    ax2.set_aspect("equal")
    plt.colorbar(sc2, ax=ax2, label=color_label, shrink=0.8)

    plt.tight_layout()
    out = filepath.replace(".pcd", "_intensity.png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved: {out}")
    plt.close()


if __name__ == "__main__":
    main()
