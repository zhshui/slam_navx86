#!/usr/bin/env python3
"""
PCD visualization with intensity-based coloring.
Parses PCD binary format manually (xyz + intensity).
Usage: python3 visualize_pcd_intensity.py [pcd_file]
"""

import sys
import struct
import numpy as np
import vtk


def parse_pcd(filepath):
    """Parse a binary PCD file, returning points and intensity arrays."""
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

    print(f"Fields: {fields}")
    print(f"Points: {points}")

    # Calculate byte offsets and total row size
    offsets = []
    total_row_bytes = 0
    type_map = {"F": ("f", 4), "I": ("i", 4), "U": ("B", 1)}

    # Build flat dtype: all fields as float32 or whatever
    dtype_parts = []
    for i, field in enumerate(fields):
        s = sizes[i] if i < len(sizes) else 4
        t = types[i] if i < len(types) else "F"
        c = counts[i] if i < len(counts) else 1
        pcd_type = type_map.get(t, ("f", 4))
        for j in range(c):
            dtype_parts.append((f"{field}_{j}", f"f{s}"))  # all as float
    total_row_bytes = sum(sizes[i] * (counts[i] if i < len(counts) else 1) for i in range(len(fields)))

    # Read binary data
    data_start = len(header)
    with open(filepath, "rb") as f:
        f.seek(data_start)
        raw = f.read()

    n = min(points, len(raw) // total_row_bytes)

    # Use numpy frombuffer directly
    dtype = np.dtype([(name, f"<f{s}") for name, s in zip(
        [dp[0] for dp in dtype_parts],
        [str(sizes[i//max(1, counts[i] if i < len(counts) else 1)]) for i in range(len(dtype_parts))]
    )])

    # Simpler: read as flat float32 array, reshape
    flat = np.frombuffer(raw[:n * total_row_bytes], dtype=np.float32)
    num_cols = total_row_bytes // 4
    flat = flat.reshape((n, num_cols))

    points_np = np.zeros((n, 3), dtype=np.float32)
    # Find column indices
    col = 0
    col_map = {}
    for i, field in enumerate(fields):
        c = counts[i] if i < len(counts) else 1
        col_map[field] = col
        col += c

    points_np[:, 0] = flat[:, col_map["x"]]
    points_np[:, 1] = flat[:, col_map["y"]]
    points_np[:, 2] = flat[:, col_map["z"]]

    intensity = None
    if "intensity" in col_map:
        intensity = flat[:, col_map["intensity"]].copy()
        print(f"Intensity: min={intensity.min():.3f}, max={intensity.max():.3f}, mean={intensity.mean():.3f}")
    else:
        print("No intensity field found")

    return points_np, intensity


def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else "/home/robot/go2_nav/lite_cog/system/map/scans/scans_5.pcd"
    print(f"Reading: {filepath}")

    points_np, intensity = parse_pcd(filepath)
    n = points_np.shape[0]
    print(f"Loaded {n} points")

    # Downsample for interactivity
    if n > 600000:
        step = max(1, n // 600000)
        points_np = points_np[::step]
        if intensity is not None:
            intensity = intensity[::step]
        print(f"Downsampled to {points_np.shape[0]} points")

    # Build VTK polydata
    vtk_points = vtk.vtkPoints()
    vtk_points.SetNumberOfPoints(points_np.shape[0])
    for i in range(points_np.shape[0]):
        vtk_points.SetPoint(i, float(points_np[i, 0]), float(points_np[i, 1]), float(points_np[i, 2]))

    vtk_verts = vtk.vtkCellArray()
    for i in range(points_np.shape[0]):
        vtk_verts.InsertNextCell(1)
        vtk_verts.InsertCellPoint(i)

    polydata = vtk.vtkPolyData()
    polydata.SetPoints(vtk_points)
    polydata.SetVerts(vtk_verts)

    # Intensity color mapping
    p1, p99 = 0.0, 255.0
    if intensity is not None:
        p1, p99 = [float(x) for x in np.percentile(intensity, [1, 99])]
        p1 = max(p1, 0.0)
        print(f"Color range (1-99%): [{p1:.1f}, {p99:.1f}]")

        clamped = np.clip(intensity.astype(np.float32), p1, p99)

        vi = vtk.vtkFloatArray()
        vi.SetName("intensity")
        vi.SetNumberOfComponents(1)
        vi.SetNumberOfTuples(points_np.shape[0])
        for i in range(points_np.shape[0]):
            vi.SetValue(i, float(clamped[i]))
        polydata.GetPointData().SetScalars(vi)

    # Point gaussian mapper for efficient rendering
    glyph = vtk.vtkVertexGlyphFilter()
    glyph.SetInputData(polydata)
    glyph.Update()

    mapper = vtk.vtkPointGaussianMapper()
    mapper.SetInputConnection(glyph.GetOutputPort())
    mapper.SetScaleFactor(0.04)
    mapper.EmissiveOn()  # No lighting, pure color

    lut = vtk.vtkLookupTable()
    lut.SetNumberOfTableValues(256)
    lut.SetRange(p1, p99)

    # Jet colormap: blue -> cyan -> green -> yellow -> red
    for i in range(256):
        t = i / 255.0
        t = max(0.0, min(1.0, t))
        if t < 0.25:
            r, g, b = 0.0, t * 4.0, 1.0
        elif t < 0.5:
            r, g, b = 0.0, 1.0, 1.0 - (t - 0.25) * 4.0
        elif t < 0.75:
            r, g, b = (t - 0.5) * 4.0, 1.0, 0.0
        else:
            r, g, b = 1.0, 1.0 - (t - 0.75) * 4.0, 0.0
        lut.SetTableValue(i, r, g, b, 1.0)
    lut.Build()

    if intensity is not None:
        mapper.SetScalarModeToUsePointData()
        mapper.SetColorModeToMapScalars()
        mapper.SetScalarRange(p1, p99)
        mapper.SetLookupTable(lut)
        mapper.ScalarVisibilityOn()

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    # Disable lighting so emissive colors show properly
    actor.GetProperty().LightingOff()

    renderer = vtk.vtkRenderer()
    renderer.AddActor(actor)
    renderer.SetBackground(0.08, 0.08, 0.10)
    renderer.SetUseFXAA(True)

    # Scalar bar
    if intensity is not None:
        scalar_bar = vtk.vtkScalarBarActor()
        scalar_bar.SetLookupTable(lut)
        scalar_bar.SetTitle("Intensity (0-255)")
        scalar_bar.SetNumberOfLabels(6)
        scalar_bar.SetPosition(0.02, 0.05)
        scalar_bar.SetWidth(0.08)
        scalar_bar.SetHeight(0.9)
        scalar_bar.GetTitleTextProperty().SetColor(1, 1, 1)
        scalar_bar.GetLabelTextProperty().SetColor(1, 1, 1)
        renderer.AddActor2D(scalar_bar)

    # Coordinate axes
    axes = vtk.vtkAxesActor()
    axes.SetTotalLength(1.0, 1.0, 1.0)
    axes.GetXAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone()
    axes.GetYAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone()
    axes.GetZAxisCaptionActor2D().GetTextActor().SetTextScaleModeToNone()
    renderer.AddActor(axes)

    window = vtk.vtkRenderWindow()
    window.AddRenderer(renderer)
    window.SetSize(1600, 900)
    window.SetWindowName("PCD Intensity Visualization — Jet colormap (blue=low → red=high)")

    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(window)
    style = vtk.vtkInteractorStyleTrackballCamera()
    interactor.SetInteractorStyle(style)

    renderer.ResetCamera()
    camera = renderer.GetActiveCamera()
    camera.Elevation(-70)
    camera.Azimuth(30)

    print("\nControls:")
    print("  Left-drag: rotate      Middle-drag: pan")
    print("  Scroll: zoom           r: reset camera")
    print("  q / Esc: quit")
    print("  Color: blue(low) → cyan → green → yellow → red(high)")

    window.Render()
    interactor.Start()


if __name__ == "__main__":
    main()
