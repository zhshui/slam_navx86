#!/usr/bin/env python3

import math


def build_route_fingerprint(geometry_fingerprint, frame_id, source_yaw, target_yaw):
    return (
        tuple(geometry_fingerprint),
        str(frame_id),
        round(float(source_yaw), 4),
        round(float(target_yaw), 4),
    )


def build_route_geometry(points, max_points=200):
    normalized = _normalize_points(points)
    if not normalized:
        raise ValueError("global path is empty")
    if max_points < 2 and len(normalized) > 1:
        raise ValueError("max_points must be at least 2")

    path_length = sum(
        math.hypot(x2 - x1, y2 - y1)
        for (x1, y1), (x2, y2) in zip(normalized, normalized[1:])
    )
    sampled = [normalized[index] for index in _sample_indices(len(normalized), max_points)]
    fingerprint = tuple((round(x, 3), round(y, 3)) for x, y in normalized)

    return {
        "path": [{"x": x, "y": y} for x, y in sampled],
        "path_length": path_length,
        "fingerprint": fingerprint,
    }


def _normalize_points(points):
    normalized = []
    for point in points:
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("global path contains a non-finite coordinate")
        normalized.append((x, y))
    return normalized


def _sample_indices(point_count, max_points):
    if point_count <= max_points:
        return range(point_count)
    return [
        round(index * (point_count - 1) / (max_points - 1))
        for index in range(max_points)
    ]
