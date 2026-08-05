#!/usr/bin/env python3

import math
import threading


class RouteRefTracker:
    def __init__(self, goal_tolerance=0.5):
        self._goal_tolerance = float(goal_tolerance)
        self._lock = threading.Lock()
        self._ref_cmd_id = ""
        self._goal = None

    def arm(self, ref_cmd_id, goal_x, goal_y):
        with self._lock:
            self._ref_cmd_id = str(ref_cmd_id or "")
            self._goal = (
                (float(goal_x), float(goal_y))
                if self._ref_cmd_id
                else None
            )

    def clear(self):
        with self._lock:
            self._ref_cmd_id = ""
            self._goal = None

    def observe_goal(self, goal_x, goal_y):
        with self._lock:
            if not self._ref_cmd_id or self._goal is None:
                return
            if self._distance(goal_x, goal_y) > self._goal_tolerance:
                self._ref_cmd_id = ""
                self._goal = None

    def consume_for_route(self, target_x, target_y):
        with self._lock:
            if not self._ref_cmd_id or self._goal is None:
                return None
            if self._distance(target_x, target_y) > self._goal_tolerance:
                return None
            ref_cmd_id = self._ref_cmd_id
            self._ref_cmd_id = ""
            self._goal = None
            return ref_cmd_id

    def _distance(self, x, y):
        return math.hypot(float(x) - self._goal[0], float(y) - self._goal[1])
