# TEB Global-Path Heading Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make TEB normally drive forward along a stable global-path tangent while retaining low-speed reverse recovery.

**Architecture:** Add a small, stateless path-heading helper that estimates the tangent over a fixed cumulative lookback distance. `TebLocalPlannerROS` uses it before XY arrival and retains final-yaw alignment after arrival. Persist forward-preference limits in the existing TEB YAML.

**Tech Stack:** ROS Noetic, C++11, catkin, GoogleTest, TEB local planner.

## Global Constraints

- Test route is `(-1.1, -1.6)` to `(6.3, -0.3)`, nominal heading `9.963 deg`.
- Use a fixed `0.8 m` heading lookback window.
- Initialize new trajectories forward while retaining bounded reverse recovery.
- Use the active `odom` topic instead of the unserved `leg_odom` topic.
- Limit reverse velocity to `0.08 m/s`.
- Use `weight_kinematics_forward_drive: 40`.
- Ignore requested final yaw until goal XY is reached.
- Rotate in place to within `10 deg` of the path heading before translating for
  each new user goal.
- Preserve unrelated working-tree changes.

---

### Task 1: Smoothed Global-Path Heading

**Files:**
- Create: `lite_cog/nav/src/teb_local_planner/include/teb_local_planner/path_heading.h`
- Create: `lite_cog/nav/src/teb_local_planner/src/path_heading.cpp`
- Create: `lite_cog/nav/src/teb_local_planner/test/path_heading_test.cpp`
- Modify: `lite_cog/nav/src/teb_local_planner/CMakeLists.txt`

**Interfaces:**
- Consumes: `std::vector<geometry_msgs::PoseStamped>`, fallback goal X/Y, robot X/Y/yaw.
- Produces: `double estimatePathHeading(const std::vector<geometry_msgs::PoseStamped>&, double, double, double, double, double, double)`.

- [ ] **Step 1: Write the failing tests**

Add tests with hand-derived expected values:

```cpp
TEST(PathHeading, followsTwoPointTestRoute)
{
  const auto path = makePath({{-1.1, -1.6}, {6.3, -0.3}});
  EXPECT_NEAR(estimatePathHeading(path, 0.8, 6.3, -0.3, -1.1, -1.6, 0.0),
              0.17390118913788663, 1e-5);
}

TEST(PathHeading, ignoresNoisyFinalEdge)
{
  const auto path = makePath({{0.0, 0.0}, {0.4, 0.0}, {0.8, 0.0},
                              {1.2, 0.0}, {1.2, 0.05}});
  EXPECT_NEAR(estimatePathHeading(path, 0.8, 1.2, 0.05, 0.0, 0.0, 0.0),
              std::atan2(0.05, 0.8), 1e-5);
}

TEST(PathHeading, fallsBackForDuplicatePath)
{
  const auto path = makePath({{1.0, 1.0}, {1.0, 1.0}});
  EXPECT_NEAR(estimatePathHeading(path, 0.8, 2.0, 2.0, 1.0, 1.0, -0.5),
              M_PI / 4.0, 1e-5);
}
```

- [ ] **Step 2: Register and run the test to verify RED**

Add `path_heading.cpp` to `teb_local_planner`, add `catkin_add_gtest(test_path_heading test/path_heading_test.cpp)`, and link it to the library.

Run:

```bash
catkin_make run_tests_teb_local_planner_gtest_test_path_heading
```

Expected: compile/link failure because `estimatePathHeading` is not implemented.

- [ ] **Step 3: Implement the minimal helper**

Walk backward from the final pose, accumulating segment lengths. Pick the first
distinct pose where cumulative length reaches `0.8 m`, or the earliest
available distinct pose. Return `atan2(end.y - start.y, end.x - start.x)`.
For a degenerate path, return robot-to-goal direction, then robot yaw.

- [ ] **Step 4: Run the focused test to verify GREEN**

Run:

```bash
catkin_make run_tests_teb_local_planner_gtest_test_path_heading
catkin_test_results build/test_results/teb_local_planner
```

Expected: all path-heading tests pass.

### Task 2: Integrate Heading and Forward Preference

**Files:**
- Modify: `lite_cog/nav/src/teb_local_planner/src/teb_local_planner_ros.cpp`
- Modify: `lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml`

**Interfaces:**
- Consumes: `estimatePathHeading(...)` from Task 1.
- Produces: stable local-goal yaw before XY arrival and bounded reverse preference.

- [ ] **Step 1: Replace the final-two-pose lambda**

Include `teb_local_planner/path_heading.h` and call:

```cpp
estimatePathHeading(transformed_plan, 0.8,
                    robot_goal_.x(), robot_goal_.y(),
                    robot_pose_.x(), robot_pose_.y(), robot_pose_.theta());
```

Keep the existing `public_xy_reached` branch that selects `final_goal_yaw`.

- [ ] **Step 2: Persist motion-preference parameters**

Set:

```yaml
odom_topic: odom
allow_init_with_backwards_motion: false
max_vel_x_backwards: 0.08
weight_kinematics_forward_drive: 40
```

Keep `global_plan_viapoint_sep: 0.2`, `via_points_ordered: true`, and
`weight_viapoint: 8`.

- [ ] **Step 3: Build and run package tests**

Run:

```bash
catkin_make
catkin_make run_tests_teb_local_planner
catkin_test_results build/test_results/teb_local_planner
```

Expected: build succeeds and all TEB tests pass.

### Task 3: Runtime Route Verification

**Files:**
- No source files.

**Interfaces:**
- Consumes: rebuilt TEB library and active ROS navigation stack.
- Produces: recorded evidence for the route acceptance criteria.

- [ ] **Step 1: Reload TEB**

Restart `move_base`, or reload the three dynamic parameters if the running
process already contains the rebuilt library.

- [ ] **Step 2: Execute the route**

Place or verify the robot at `(-1.1, -1.6)`, then send `(6.3, -0.3)` as the
goal. Use the route tangent yaw during translation; final goal yaw may be any
requested value because it is applied only after XY arrival.

- [ ] **Step 3: Capture evidence**

Record `/move_base/GlobalPlanner/plan`,
`/move_base/TebLocalPlannerROS/local_plan`, `/cmd_vel`, robot pose, and
`/move_base/status`.

- [ ] **Step 4: Evaluate acceptance**

Confirm local-plan direction stays near the global-path tangent, normal motion
uses non-negative X velocity, any negative X velocity is temporary and no less
than `-0.08 m/s`, and final-yaw alignment starts only after XY arrival.
