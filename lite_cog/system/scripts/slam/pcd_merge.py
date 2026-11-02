#!/usr/bin/env python3
"""Merge binary PCD files (written by pcl::PCDWriter) into a single PCD.

Fallback for pcl_concatenate_points_pcd (pcl-tools not installed).
Pure python3 stdlib, no third-party deps.

Usage: pcd_merge.py output.pcd input1.pcd input2.pcd ...
All inputs must share the same field layout (FIELDS/SIZE/TYPE/COUNT);
An invalid input aborts the merge so the caller can retain every segment.
"""
import sys


def split_header(raw: bytes):
    """Return (header_text, data_offset, point_size, header_fields)."""
    end = raw.find(b"\nDATA")
    if end == -1:
        raise ValueError("no DATA line found")
    line_end = raw.find(b"\n", end + 1)
    if line_end == -1:
        raise ValueError("unterminated DATA line")
    header = raw[:line_end].decode("ascii")
    data_start = line_end + 1
    fields = {}
    for line in header.splitlines():
        if line and not line.startswith("#"):
            key, _, val = line.partition(" ")
            fields[key] = val
    sizes = [int(x) for x in fields["SIZE"].split()]
    counts = [int(x) for x in fields["COUNT"].split()]
    point_size = sum(s * c for s, c in zip(sizes, counts))
    return header, data_start, point_size, fields


def main():
    argv = sys.argv[1:]
    if len(argv) < 2:
        print("usage: pcd_merge.py out.pcd in1.pcd [in2.pcd ...]", file=sys.stderr)
        return 2
    out, inputs = argv[0], argv[1:]

    merged = bytearray()
    first_header = None
    first_layout = None
    total = 0
    skipped = 0
    for path in inputs:
        try:
            with open(path, "rb") as f:
                raw = f.read()
        except OSError as e:
            print(f"cannot read {path}: {e}", file=sys.stderr)
            return 1
        header, data_start, psize, fields = split_header(raw)
        layout = (fields.get("FIELDS"), fields.get("SIZE"),
                  fields.get("TYPE"), fields.get("COUNT"))
        if fields.get("DATA") != "binary" or psize <= 0:
            print(f"unsupported binary PCD layout: {path}", file=sys.stderr)
            return 1
        if first_layout is None:
            first_layout, first_header = layout, header
        elif layout != first_layout:
            print(f"field layout mismatch: {path}", file=sys.stderr)
            return 1
        payload = raw[data_start:]
        n = len(payload) // psize
        if len(payload) % psize or n != int(fields["POINTS"]):
            print(f"incomplete PCD data: {path}", file=sys.stderr)
            return 1
        if n == 0:
            print(f"skip {path}: no points", file=sys.stderr)
            skipped += 1
            continue
        merged += payload
        total += n

    if first_header is None or total == 0:
        print("no points merged", file=sys.stderr)
        return 1

    out_lines = []
    for line in first_header.splitlines():
        if line.startswith("WIDTH"):
            out_lines.append(f"WIDTH {total}")
        elif line.startswith("POINTS"):
            out_lines.append(f"POINTS {total}")
        else:
            out_lines.append(line)
    with open(out, "wb") as f:
        f.write(("\n".join(out_lines) + "\n").encode("ascii"))
        f.write(merged)
    print(f"merged {len(inputs) - skipped}/{len(inputs)} files, "
          f"{total} points -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
