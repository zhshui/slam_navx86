# TEB Global-Path Heading Design

## Goal

Improve local planning for the route from `(-1.1, -1.6)` to `(6.3, -0.3)`.
The nominal route heading is `atan2(1.3, 7.4)`, approximately `9.963 deg`.

The robot should normally move forward with its body heading aligned to the
global path. Reverse motion remains available as a low-speed recovery option.
The requested final yaw is ignored until the goal XY position is reached.

## Current Problem

The current local goal yaw is estimated from only the final two poses in the
transformed global plan. Small spacing, quantization, or a bend at the local
lookahead boundary can therefore cause abrupt heading changes.

The persisted and currently loaded parameters also use
`weight_kinematics_forward_drive: 1` and `max_vel_x_backwards: 0.2`. This makes
reverse trajectories comparatively cheap and permits substantial reverse
speed.

Runtime inspection also shows that TEB subscribes to `/leg_odom`, which has no
publisher, while `/odom` publishes the robot pose and twist continuously.

## Design

### Smoothed path heading

Estimate the local goal yaw from a path segment extending backward from the
transformed plan endpoint. Select the earliest pose within a fixed `0.8 m`
distance window, then compute the direction from that pose to the endpoint.

If the available path is shorter than the window, use its first and last
distinct poses. If no distinct pair exists, fall back to the robot-to-local-goal
direction, then to the current robot yaw.

This estimates the local path tangent over a meaningful distance instead of
using one potentially noisy edge.

### Motion preference

Initialize new trajectories in the forward direction, but retain bounded
reverse motion in the optimizer as a recovery option:

- `allow_init_with_backwards_motion: false`
- `odom_topic: odom`
- `max_vel_x_backwards: 0.08`
- `weight_kinematics_forward_drive: 40`
- retain ordered via points at `0.2 m` spacing and `weight_viapoint: 8`

Disabling backward initialization does not prohibit reverse velocity. These
values allow short recovery reversals without making reverse travel competitive
with a feasible forward trajectory.

### Goal behavior

For each newly received goal, the robot first rotates in place until its body
heading is within `10 deg` of the smoothed local global-path heading. This
one-shot state is triggered by the user goal, not by recurring global-plan
updates. The optimizer then follows the path normally. Within `0.6 m` of the
goal, a low-speed controller points the robot toward the goal position and
limits forward speed to `0.16 m/s`. After the goal XY tolerance is met, final
yaw mode is latched: the controller receives the user-requested final yaw and
does not return to path-heading mode while rotating in place.

## Verification

Add focused tests for the heading estimator:

- a straight path from `(-1.1, -1.6)` to `(6.3, -0.3)` returns approximately
  `9.963 deg`;
- a noisy final edge does not dominate the heading;
- a short path uses its available distinct endpoints;
- duplicate poses use the documented fallback.

Build the local planner, restart or dynamically reload the planner as needed,
then execute the two-point route. Record the global plan, local plan, robot
pose, and `cmd_vel`.

Acceptance criteria:

- the local trajectory follows the global route without visible cutting or
  repeated side-to-side heading changes;
- normal-route linear X velocity is non-negative;
- any reverse command is bounded by `0.08 m/s` and is temporary;
- heading during translation stays close to the local global-path tangent;
- final yaw alignment begins only after XY arrival.

## Scope

Only the TEB path-heading calculation, its focused tests, and the relevant TEB
configuration are changed. Obstacle handling, global planning, localization,
MQ status handling, and base command filtering are outside this change.
