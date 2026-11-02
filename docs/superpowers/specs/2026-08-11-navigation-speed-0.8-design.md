# Go2 Navigation Cruise Speed 0.8 m/s Design

## Objective

Raise straight-line navigation cruise speed from 0.5 m/s to approximately
0.8 m/s while preserving obstacle-proximity slowdown, goal-approach control,
localization monitoring, and the existing acceleration limit.

## Current State

- `TebLocalPlannerROS/max_vel_x`: 0.5 m/s
- `TebLocalPlannerROS/max_vel_trans`: 1.0 m/s
- `TebLocalPlannerROS/acc_lim_x`: 0.5 m/s^2
- The Go2 `cmd_vel` bridge adds EMA smoothing but no additional linear-speed
  clamp.
- Navigation between `(0, 0)` and `(38, -17)` succeeds at the current speed.
- The active goal was canceled before changing parameters. The stopped pose was
  approximately `(29.86, -6.86)`.

## Selected Design

Change only these TEB limits:

- `max_vel_x`: 0.5 -> 0.8 m/s
- `max_vel_trans`: 1.0 -> 0.8 m/s

Keep these controls unchanged:

- `acc_lim_x`: 0.5 m/s^2, preventing a step-like acceleration increase.
- `max_vel_y`, reverse speed, angular speed, and angular acceleration.
- Obstacle-proximity speed ratio and costmap obstacle handling.
- Goal tolerances and terminal PID behavior.

Setting both forward and total translational limits to 0.8 m/s avoids the
existing TEB warning where `max_vel_trans` exceeds the component velocity limit
and prevents diagonal motion from exceeding the requested translational speed.

## Application

1. Persist both values in
   `lite_cog/nav/src/navigation/config/teb_local_planner_params.yaml`.
2. Apply both values to the running TEB planner with dynamic reconfigure.
3. Read back live parameters before sending a goal.
4. If dynamic application fails, leave the robot stopped and restart only
   `move_base` through the project's existing navigation workflow.

## Validation

Run the following route using the same `/move_base_simple/goal` interface as the
Web UI:

1. Current pose -> `(0, 0)`
2. `(0, 0)` -> `(38, -17)`
3. `(38, -17)` -> `(0, 0)`
4. `(0, 0)` -> `(38, -17)`

Monitor throughout:

- action status and goal preemption;
- localization pose continuity and `has_converged`;
- `/cmd_vel` peak and sustained straight-line speed;
- planner, control-loop, TF, and costmap errors;
- final XY error at each endpoint.

Acceptance criteria:

- sustained clear straight-line command reaches approximately 0.75-0.8 m/s;
- commanded translational speed does not exceed 0.8 m/s;
- every specified goal returns `SUCCEEDED` without unexplained preemption;
- no localization jump, loss of convergence, or new fatal navigation error;
- endpoint XY error remains within 0.15 m.

If the faster run causes repeated infeasible trajectories, unsafe oscillation,
localization jumps, or materially worse control-loop overruns, cancel the goal,
restore 0.5 m/s, and report the observed limit rather than increasing other
parameters.
