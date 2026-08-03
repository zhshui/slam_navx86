# Terminal PID Convergence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace terminal TEB velocity output with a latched holonomic PID that converges XY and final yaw together without state ping-pong.

**Architecture:** A focused `TerminalPidController` owns PID history and produces bounded robot-frame commands from global-frame goal errors. `TebLocalPlannerROS` selects between normal TEB path tracking and terminal PID, checks projected PID motion against the local costmap, and resets the PID on every state transition or new goal.

**Tech Stack:** ROS Noetic, C++14, `geometry_msgs`, `costmap_2d`, GoogleTest, catkin.

## Global Constraints

- Enter terminal PID at XY distance `<= 0.15 m`.
- Stay in terminal PID until XY distance is `> 0.30 m`.
- Keep goal tolerances at `0.08 m` XY and `0.05 rad` yaw.
- Cap terminal translation at `0.20 m/s` and yaw at `0.60 rad/s`.
- Use XY gains `Kp=1.5`, `Ki=0.05`, `Kd=0.10`.
- Use yaw gains `Kp=2.0`, `Ki=0.05`, `Kd=0.10`.
- Clamp XY integral to `0.10 m*s` and yaw integral to `0.20 rad*s`.
- Use derivative low-pass factor `0.20` and clamp cycle time to `0.01-0.20 s`.
- Require five stable goal cycles and no post-debounce hold cycles.
- Preserve initial path alignment and normal TEB path tracking outside terminal PID.

---

### Task 1: Terminal PID Unit

**Files:**
- Create: `lite_cog/nav/src/teb_local_planner/include/teb_local_planner/terminal_pid.h`
- Create: `lite_cog/nav/src/teb_local_planner/src/terminal_pid.cpp`
- Create: `lite_cog/nav/src/teb_local_planner/test/terminal_pid_test.cpp`
- Modify: `lite_cog/nav/src/teb_local_planner/CMakeLists.txt`

**Interfaces:**
- Produces: `TerminalPidConfig`, `TerminalPidController::reset()`, `TerminalPidController::calculate(...)`, and `updateTerminalConvergence(...)`.
- `calculate(dx_global, dy_global, yaw_error, robot_yaw, dt, xy_tolerance, yaw_tolerance)` returns `geometry_msgs::Twist` in the robot frame.

- [ ] **Step 1: Add failing state-latch tests**

```cpp
EXPECT_TRUE(updateTerminalConvergence(false, 0.15, 0.15, 0.30));
EXPECT_TRUE(updateTerminalConvergence(true, 0.22, 0.15, 0.30));
EXPECT_FALSE(updateTerminalConvergence(true, 0.31, 0.15, 0.30));
```

- [ ] **Step 2: Add failing PID behavior tests**

Test simultaneous nonzero XY/yaw output, signed robot-frame correction at a
rotated yaw, `0.20/0.60` output caps, zero per-axis output inside tolerance,
integral limits after repeated cycles, filtered derivative response, and zero
derivative kick after `reset()`.

- [ ] **Step 3: Run the test target and verify RED**

Run:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=teb_local_planner tests
devel/lib/teb_local_planner/test_terminal_pid
```

Expected: compilation fails because `terminal_pid.h` and its interfaces do not
exist.

- [ ] **Step 4: Implement the minimal controller**

Keep PID state in global XY coordinates. Clamp `dt`; clamp integrals before
forming output; low-pass raw derivatives; cap the global XY vector magnitude;
then rotate it into the robot frame. Normalize yaw errors and zero each
controller axis independently inside its public tolerance.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=teb_local_planner tests
devel/lib/teb_local_planner/test_terminal_pid
```

Expected: all terminal PID cases pass.

### Task 2: TEB ROS Integration

**Files:**
- Modify: `lite_cog/nav/src/teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h`
- Modify: `lite_cog/nav/src/teb_local_planner/src/teb_local_planner_ros.cpp`
- Modify: `lite_cog/nav/src/teb_local_planner/test/path_heading_test.cpp`
- Modify: `lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml`

**Interfaces:**
- Consumes: `TerminalPidController` and `updateTerminalConvergence` from Task 1.
- Produces: a terminal branch that returns a valid PID command before TEB optimization.

- [ ] **Step 1: Replace old final-yaw state tests with failing latch tests**

Remove the behavior that exits final yaw when yaw is aligned but XY is outside
`0.08 m`. Assert that the terminal state remains active to `0.30 m`.

- [ ] **Step 2: Integrate the terminal state before TEB optimization**

Rename `final_yaw_alignment_active_` to `terminal_convergence_active_`. Enter at
`0.15 m`; reset the PID and timestamp on state changes. In the active branch,
calculate PID from the original global goal XY and user final yaw and do not
call `planner_->plan()` or `planner_->getVelocityCommand()`.

- [ ] **Step 3: Add projected footprint validation**

Update a dynamic footprint when configured. Check `footprintCost()` at the
current pose and at the pose projected `0.5 s` using the PID command. Return a
zero command with `NO_VALID_CMD` when either cost is negative; otherwise apply
the existing `saturateVelocity()`, store `last_cmd_`, and return `SUCCESS`.

- [ ] **Step 4: Reset terminal state on a new goal**

In `userGoalCB()`, clear the terminal latch, call `terminal_pid_.reset()`, clear
its timestamp, and retain `initial_path_alignment_pending_ = true`.

- [ ] **Step 5: Shorten only the success settling delay**

Set:

```yaml
arrival_debounce_cycles: 5
arrival_hold_cycles: 0
```

Use the configured debounce count directly rather than enforcing a minimum of
ten in `outer_goal_settle_count_`.

- [ ] **Step 6: Build and run all package tests**

Run:

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=teb_local_planner
catkin_make -DCATKIN_WHITELIST_PACKAGES=teb_local_planner run_tests_teb_local_planner
devel/lib/teb_local_planner/test_teb_basics
devel/lib/teb_local_planner/test_path_heading
devel/lib/teb_local_planner/test_terminal_pid
```

Expected: build succeeds and every direct GoogleTest binary reports zero
failures.

### Task 3: Runtime Verification

**Files:**
- Runtime artifact: `/tmp/teb_terminal_pid_B_20260727.bag`

**Interfaces:**
- Consumes: built planner plugin and goal B `(6.3, -0.3, 10 deg)`.
- Produces: measured terminal convergence duration and final errors.

- [ ] **Step 1: Restart only move_base**

Use the existing `lite_cog/system/scripts/nav/restart_move_base.sh`, then verify
the loaded process logs the terminal PID version marker.

- [ ] **Step 2: Record the relevant topics**

Record `/cmd_vel`, `/odom`, `/move_base/status`,
`/move_base/GlobalPlanner/plan`, `/move_base/TebLocalPlannerROS/local_plan`,
`/move_base/current_goal`, `/tf`, and `/tf_static`.

- [ ] **Step 3: Send goal B and monitor safety**

Send `(6.3, -0.3, yaw=10 deg)`. Cancel and publish zero immediately if command
output oscillates, the terminal state exits below `0.30 m`, or motion approaches
an obstacle.

- [ ] **Step 4: Analyze acceptance criteria**

From the bag, measure first entry below `0.6 m`, terminal PID entry, goal success,
final XY/yaw errors, and terminal state transition count. Acceptance requires
one terminal entry, no exit before success, configured final tolerances, and
approximately five seconds from `0.6 m` entry to success.

- [ ] **Step 5: Run final scoped checks**

Run:

```bash
git diff --check -- \
  lite_cog/nav/src/teb_local_planner \
  lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml
```

Confirm the robot is stopped, stop the bag recorder, and report measured values.
