#!/usr/bin/env python3

import math
import unittest

from route_geometry import build_route_fingerprint, build_route_geometry


class RouteGeometryTest(unittest.TestCase):
    def test_long_path_keeps_both_endpoints_and_full_length(self):
        points = [(float(i), 0.0) for i in range(401)]

        geometry = build_route_geometry(points, max_points=200)

        self.assertEqual(200, len(geometry["path"]))
        self.assertEqual({"x": 0.0, "y": 0.0}, geometry["path"][0])
        self.assertEqual({"x": 400.0, "y": 0.0}, geometry["path"][-1])
        self.assertAlmostEqual(400.0, geometry["path_length"])

    def test_internal_path_change_changes_fingerprint(self):
        straight = build_route_geometry([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
        detour = build_route_geometry([(0.0, 0.0), (1.0, 0.5), (2.0, 0.0)])

        self.assertNotEqual(straight["fingerprint"], detour["fingerprint"])

    def test_endpoint_yaw_change_changes_route_fingerprint(self):
        geometry = build_route_geometry([(0.0, 0.0), (2.0, 0.0)])

        first = build_route_fingerprint(geometry["fingerprint"], "map", 0.0, 0.0)
        second = build_route_fingerprint(geometry["fingerprint"], "map", 0.0, 1.57)

        self.assertNotEqual(first, second)

    def test_rejects_non_finite_coordinates(self):
        for invalid in (math.nan, math.inf, -math.inf):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    build_route_geometry([(0.0, 0.0), (invalid, 1.0)])

    def test_rejects_empty_path(self):
        with self.assertRaises(ValueError):
            build_route_geometry([])


if __name__ == "__main__":
    unittest.main()
