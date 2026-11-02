# Go2 Navigation Cruise Speed 0.8 m/s Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Raise Go2 navigation cruise speed to approximately 0.8 m/s and verify stable localization and planning between `(0,0)` and `(38,-17)`.

**Architecture:** Persist the two TEB velocity limits in the existing planner YAML, then apply the same values through dynamic reconfigure so the running planner changes without restarting localization. Validate live parameter state before executing the Web-equivalent simple-goal route while monitoring action status, pose, convergence, command speed, and logs.

**Tech Stack:** ROS Noetic, `move_base`, `teb_local_planner`, dynamic_reconfigure, YAML, Bash/ROS CLI.

## Global Constraints

- `max_vel_x` and `max_vel_trans` must both equal `0.8` m/s.
- Keep `acc_lim_x` at `0.5` m/s^2.
- Do not change reverse, lateral, angular, obstacle, goal-tolerance, or terminal-PID parameters.
- Keep obstacle-proximity slowdown active.
- Cancel active goals before changing live parameters.
- Commanded translational speed must not exceed 0.8 m/s.
- Endpoint XY error must remain at or below 0.15 m.
- On unsafe oscillation, localization loss/jump, repeated infeasible trajectories, or materially worse loop overruns, cancel the goal and restore both limits to their previous values (`0.5` and `1.0`).

---

### Task 1: Persist and activate the TEB velocity limits

**Files:**
- Modify: `lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml:33`
- Modify: `lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml:36`

**Interfaces:**
- Consumes: ROS parameters `/move_base/TebLocalPlannerROS/max_vel_x` and `/move_base/TebLocalPlannerROS/max_vel_trans`.
- Produces: persistent and live TEB translational limits of exactly `0.8` m/s.

- [ ] **Step 1: Verify the robot is stopped and no goal is active**

Run:

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}"
timeout 3 rostopic echo -n 1 /move_base/status/status_list
timeout 2 rostopic echo -n 1 /cmd_vel
```

Expected: the latest goal is PREEMPTED (`status: 2`) or already terminal, and no nonzero `/cmd_vel` message is received.

- [ ] **Step 2: Run the persistent-config test and verify it fails**

Run:

```bash
python3 -c "import yaml; p=yaml.safe_load(open('/home/unitree/go2_nav/lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml'))['TebLocalPlannerROS']; assert p['max_vel_x'] == 0.8 and p['max_vel_trans'] == 0.8, (p['max_vel_x'], p['max_vel_trans'])"
```

Expected: `AssertionError: (0.5, 1.0)`.

- [ ] **Step 3: Make the minimal persistent configuration change**

Change exactly:

```yaml
  max_vel_x: 0.8
  max_vel_trans: 0.8
```

Leave `acc_lim_x: 0.5` and all other values unchanged.

- [ ] **Step 4: Run the persistent-config test and verify it passes**

Run the Step 2 command again.

Expected: exit status 0 and no output.

- [ ] **Step 5: Apply both limits to the running planner**

Run:

```bash
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_vel_x 0.8
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_vel_trans 0.8
rosparam get /move_base/TebLocalPlannerROS/max_vel_x
rosparam get /move_base/TebLocalPlannerROS/max_vel_trans
rosparam get /move_base/TebLocalPlannerROS/acc_lim_x
```

Expected output values: `0.8`, `0.8`, and `0.5`.

- [ ] **Step 6: Review and commit only the configuration change**

Run:

```bash
git -C /home/unitree/go2_nav diff --check -- lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml
git -C /home/unitree/go2_nav diff -- lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml
git -C /home/unitree/go2_nav add lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml
git -C /home/unitree/go2_nav commit -m "config: raise navigation cruise speed to 0.8 mps"
```

Expected diff: only `max_vel_x: 0.5 -> 0.8` and `max_vel_trans: 1.0 -> 0.8`.

### Task 2: Validate live behavior and full route

**Files:**
- Inspect: `lite_cog/system/scripts/nav/logs/nav.log`
- No additional file modifications.

**Interfaces:**
- Consumes: `/move_base_simple/goal`, `/move_base/status`, `/odom`, `/status`, `/cmd_vel`, and the navigation log.
- Produces: recorded pass/fail evidence for the four-leg route and final live state at `(38,-17)`.

- [ ] **Step 1: Record the pre-run state**

Run:

```bash
date --iso-8601=seconds
timeout 3 rostopic echo -n 1 /odom/pose/pose
timeout 3 rostopic echo -n 1 /status/has_converged
wc -l /home/unitree/go2_nav/lite_cog/system/scripts/nav/logs/nav.log
```

Expected: a finite pose, `True`, and a log line number used as the diagnostic baseline.

- [ ] **Step 2: Navigate from the current pose to `(0,0)`**

Run:

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: {stamp: {secs: 0, nsecs: 0}, frame_id: 'map'}
pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

Monitor `/move_base/status/status_list`, `/odom/pose/pose`, `/status/has_converged`, and `/cmd_vel` until terminal status.

Expected: `status: 3`, XY error <= 0.15 m, continuous convergence, sustained clear-path `linear.x` approximately 0.75-0.8 m/s, and translational magnitude <= 0.8 m/s.

- [ ] **Step 3: Navigate `(0,0)` to `(38,-17)`**

Run:

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header: {stamp: {secs: 0, nsecs: 0}, frame_id: 'map'}
pose: {position: {x: 38.0, y: -17.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

Apply the same monitoring and acceptance criteria as Step 2.

- [ ] **Step 4: Repeat the return leg to `(0,0)`**

Publish the exact Step 2 goal again and apply the same monitoring and acceptance criteria.

- [ ] **Step 5: Repeat the final leg to `(38,-17)`**

Publish the exact Step 3 goal again and apply the same monitoring and acceptance criteria.

- [ ] **Step 6: Inspect only new navigation diagnostics**

Run `sed -n '<baseline>,\$p'` on `nav.log`, using the line number recorded in Step 1, and inspect for `ERROR`, `FATAL`, `WARN`, `NO PATH`, `trajectory is not feasible`, control-loop overruns, TF errors, and localization rejection.

Expected: no fatal error, unexplained preemption, localization loss, or repeated planning failure. Occasional isolated loop-rate warnings are acceptable only if goals continue progressing and complete.

- [ ] **Step 7: Verify final state and configuration**

Run:

```bash
timeout 3 rostopic echo -n 1 /odom/pose/pose
timeout 3 rostopic echo -n 1 /move_base/status/status_list
rosparam get /move_base/TebLocalPlannerROS/max_vel_x
rosparam get /move_base/TebLocalPlannerROS/max_vel_trans
rosparam get /move_base/TebLocalPlannerROS/acc_lim_x
git -C /home/unitree/go2_nav status --short --branch
```

Expected: final pose within 0.15 m of `(38,-17)`, latest status `3`, live parameters `0.8`, `0.8`, `0.5`, and no unexpected task-created file changes.

- [ ] **Step 8: Roll back if an acceptance criterion fails**

Run:

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}"
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_vel_x 0.5
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS max_vel_trans 1.0
```

Restore the two YAML values to `0.5` and `1.0`, verify them with the Task 1 test adjusted to the rollback values, and commit the rollback separately with the observed failure reason.

