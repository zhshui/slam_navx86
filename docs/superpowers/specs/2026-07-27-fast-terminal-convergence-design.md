# Fast Terminal Convergence Design

## Goal

Make the robot converge at a navigation goal quickly without repeatedly
switching between position approach and final-yaw alignment.

The existing path behavior remains unchanged:

1. Rotate in place toward the initial global-path direction.
2. Track the global path toward the goal.
3. At the goal, converge position and the user-requested yaw.

Success remains defined by the configured tolerances:

- XY error below `0.08 m`
- yaw error below `0.05 rad`
- both conditions stable for five controller cycles

The runtime target is to report success about five seconds after first entering
the final `0.6 m` approach region in an obstacle-free test.

## Root Cause

The current terminal controller latches final-yaw alignment only inside the XY
goal tolerance and exits it when XY error exceeds approximately `0.10 m`.
Localization noise and rotation-induced XY drift can cross that narrow band.
The controller then rotates toward the goal position, translates, and rotates
back to the requested final yaw. Repeating this sequence dominates convergence
time.

Final yaw is also capped at `0.30 rad/s`, so each repeated rotation is slow.

## State Model

The controller has three terminal-related states:

- `path_tracking`: normal TEB path tracking and initial path alignment.
- `terminal_convergence`: latched joint XY and final-yaw control.
- `settled`: both public tolerances have held for five cycles.

Enter `terminal_convergence` when XY distance is at most `0.15 m`. Once entered,
remain in it while XY distance is below `0.30 m`. Small localization drift must
not return the controller to path tracking.

Exit to `path_tracking` only when XY distance exceeds `0.30 m`. This indicates a
real displacement that should be recovered through the path rather than a
normal terminal correction.

## Terminal PID Controller

In `terminal_convergence`, ignore the TEB velocity output and command holonomic
translation and yaw correction concurrently with an independent PID controller.
TEB remains responsible for path tracking outside this state.

- Maximum translation speed: `0.20 m/s`
- Maximum yaw speed: `0.60 rad/s`
- XY gains: `Kp=1.5`, `Ki=0.05`, `Kd=0.10`
- yaw gains: `Kp=2.0`, `Ki=0.05`, `Kd=0.10`
- XY integral limit: `0.10 m*s`
- yaw integral limit: `0.20 rad*s`
- derivative low-pass factor: `0.20`
- Translation command decreases continuously near the XY tolerance
- Yaw command decreases continuously near the yaw tolerance
- No forced minimum speed inside the terminal controller

The PID integrates and differentiates global-frame XY error, then rotates the
resulting translation command into the robot frame. This prevents PID state
from changing direction merely because the robot is rotating. The controller
uses the original user goal pose, not a shortened local-plan endpoint. Signed
robot-frame X and Y commands are allowed, so a small correction does not
require rotating toward the position error first.

When only one error remains outside tolerance, the corresponding axis continues
to converge while the aligned axes command zero.

PID state resets when terminal convergence is entered or exited, when a new
goal arrives, and when the controller cycle time is invalid. Cycle time is
clamped to `0.01-0.20 s`.

## Safety And Failure Behavior

- Normal path-tracking limits remain unchanged.
- Terminal speed caps are lower than the existing cruise speed.
- The TEB velocity output is not used during terminal convergence.
- Before publishing a PID command, check the current footprint and a projected
  `0.5 s` footprint pose against the local costmap. Publish zero if either pose
  is in collision or outside the costmap.
- Final velocity saturation remains active.
- A new goal clears the terminal latch and starts initial path alignment.
- A displacement beyond `0.30 m` clears the terminal latch and resumes path
  tracking.
- Goal success always publishes a zero velocity.

## Verification

Unit tests cover:

- entering terminal convergence at `0.15 m`
- remaining latched between `0.15 m` and `0.30 m`
- exiting only above `0.30 m`
- simultaneous signed XY and yaw output
- PID integral limiting, derivative filtering, and reset behavior
- velocity caps and zero output inside tolerances
- five-cycle settled requirement

After build and unit tests, restart only `move_base` and test the B goal
`(6.3, -0.3, 10 deg)`. Record odometry, command velocity, status, and paths.
Acceptance requires:

- no repeated terminal/path state switching
- no turn-toward-position and turn-back cycle
- final error within the configured XY and yaw tolerances
- goal success approximately five seconds after entering the final `0.6 m`
  approach region
