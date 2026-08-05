# MQ Global Route Geometry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish self-consistent MQ global routes derived entirely from the ROS global planner path.

**Architecture:** Add a ROS-independent geometry helper that validates, samples, measures, and fingerprints path positions. Keep ROS pose/yaw conversion and MQ publication in `mq_adapter.py`.

**Tech Stack:** Python 3, `unittest`, ROS Noetic `nav_msgs/Path`, `tf`

## Global Constraints

- Route geometry comes only from `/move_base/GlobalPlanner/plan`.
- Published paths contain at most 200 points and always include both endpoints.
- Length and change detection use the complete original path.
- Existing `ref_cmd_id` one-shot matching behavior is preserved.

---

### Task 1: Route Geometry Helper

**Files:**
- Create: `lite_cog/system/scripts/mq/route_geometry.py`
- Create: `lite_cog/system/scripts/mq/test_route_geometry.py`

**Interfaces:**
- Produces: `build_route_geometry(points, max_points=200)` returning `path`, `path_length`, and `fingerprint`.
- Produces: `build_route_fingerprint(geometry_fingerprint, frame_id, source_yaw, target_yaw)`.

- [ ] **Step 1: Write failing tests**

Cover a 401-point path preserving indices 0 and 400 in 200 samples, full
400-meter length, fingerprints changing for an internal point or endpoint yaw,
and rejection of non-finite coordinates.

- [ ] **Step 2: Verify tests fail**

Run: `python3 test_route_geometry.py`

Expected: FAIL because `route_geometry` does not exist.

- [ ] **Step 3: Implement minimal helper**

Validate finite `(x, y)` values, calculate full polyline length, select evenly
distributed indices including both endpoints, and fingerprint all rounded
positions.

- [ ] **Step 4: Verify tests pass**

Run: `python3 test_route_geometry.py`

Expected: all tests pass.

### Task 2: ROS Path Integration

**Files:**
- Modify: `lite_cog/system/scripts/mq/mq_adapter.py:1082`
- Modify: `mq_interface.md:259`

**Interfaces:**
- Consumes: `build_route_geometry(points, max_points=200)`.
- Produces: route body fields `frame_id`, `source`, `target`, `path`, and `path_length`.

- [ ] **Step 1: Replace endpoint sources**

Use the first and last ROS poses for endpoint position/yaw. Remove route
endpoint dependence on `state.last_goal` and live TF.

- [ ] **Step 2: Replace truncation and route hash**

Build geometry from all planner poses, publish sampled points, and use the
complete geometry fingerprint for change detection.

- [ ] **Step 3: Add visible error handling**

Catch route conversion failures and call `rospy.logwarn_throttle(5.0, ...)`
without publishing malformed route data.

- [ ] **Step 4: Update protocol documentation**

Document `frame_id`, planner-pose endpoint semantics, 200-point sampling, and
full-path length calculation.

- [ ] **Step 5: Run complete MQ verification**

Run:
`python3 test_route_geometry.py && python3 test_route_ref_tracker.py && python3 -m py_compile mq_adapter.py mq_send.py route_geometry.py route_ref_tracker.py test_route_geometry.py test_route_ref_tracker.py`

Expected: all tests pass and compilation exits successfully.
