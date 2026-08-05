#!/usr/bin/env python3

import unittest

from route_ref_tracker import RouteRefTracker


class RouteRefTrackerTest(unittest.TestCase):
    def test_ref_cmd_id_is_consumed_only_once(self):
        tracker = RouteRefTracker()
        tracker.arm("cmd-123", 6.3, -0.3)

        self.assertEqual("cmd-123", tracker.consume_for_route(6.3, -0.3))
        self.assertIsNone(tracker.consume_for_route(6.3, -0.3))

    def test_matching_goal_keeps_pending_ref_cmd_id(self):
        tracker = RouteRefTracker()
        tracker.arm("cmd-123", 6.3, -0.3)

        tracker.observe_goal(6.3, -0.3)

        self.assertEqual("cmd-123", tracker.consume_for_route(6.3, -0.3))

    def test_different_goal_clears_pending_ref_cmd_id(self):
        tracker = RouteRefTracker()
        tracker.arm("cmd-123", 6.3, -0.3)

        tracker.observe_goal(-1.1, -1.6)

        self.assertIsNone(tracker.consume_for_route(-1.1, -1.6))

    def test_missing_msg_id_does_not_add_a_route_ref(self):
        tracker = RouteRefTracker()
        tracker.arm("", 6.3, -0.3)

        self.assertIsNone(tracker.consume_for_route(6.3, -0.3))


if __name__ == "__main__":
    unittest.main()
