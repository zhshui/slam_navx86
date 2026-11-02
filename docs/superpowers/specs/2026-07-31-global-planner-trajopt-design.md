# Global Planner TrajOpt Integration Design

## Goal

Optimize the complete path produced by
`global_planner::GlobalPlanner::makePlan()` with the trajectory optimizer from
`1moule/bipedal_wheel` branch `VLP-16`, while preserving the existing
`nav_core::BaseGlobalPlanner` input, output, plugin name, and ROS plan publisher.

## Source

The optimizer source is taken from commit
`e96fa4c0db4143a9bb06be6c265bac1270b0d08a` of:

`https://github.com/1moule/bipedal_wheel/tree/VLP-16/bipedal_wheel_planner/include/bipedal_wheel_planner/trajectory_generator`

Only the header-only components required for optimization are vendored:

- `backend_tools/LBFGS.hpp`
- `backend_tools/SplineTrajectory.hpp`
- `backend_tools/TrajectoryOptimizer.hpp`
- `perception_tools/GridMap.hpp`

The original `Astar.hpp` is not used because `global_planner` already produces
the input global path.

## Data Flow

1. `GlobalPlanner::makePlan()` generates the original path normally.
2. A `TrajectoryOptimizerAdapter` converts `costmap_` to the optimizer's binary
   occupancy map and computes its ESDF.
3. The adapter converts each path pose to `Eigen::Vector2d`.
4. `TrajOpt::TrajectoryOptimizer` optimizes the complete path.
5. The adapter samples the optimized spline at a configurable time interval.
6. Every sampled point and every segment between samples is checked against the
   current costmap.
7. A valid optimized result replaces `plan`; an invalid or failed result leaves
   the original path unchanged.
8. The existing orientation filter and `publishPlan(plan)` process the selected
   path.

## Costmap Conversion

Cells at or above `costmap_2d::INSCRIBED_INFLATED_OBSTACLE` are occupied.
Unknown cells are occupied when `allow_unknown` is false and free when it is
true. Lower-cost inflated cells remain free in the binary map; the optimizer's
`safe_threshold` supplies metric clearance from inscribed and lethal cells.

The map origin, resolution, width, and height are copied directly from
`costmap_`.

## Parameters

The following private parameters are loaded under the existing
`~/<planner-name>` namespace:

- `trajectory_optimization_enabled` (default `true`)
- `trajectory_max_velocity` (default `3.0` m/s)
- `trajectory_safe_distance` (default `0.5` m)
- `trajectory_sample_dt` (default `0.1` s)
- `trajectory_max_iterations` (default `1000`)

Total path length is computed from the original plan. Initial total time is
`length / trajectory_max_velocity`, and the source algorithm's piece length is
set to `length / total_time`, matching the supplied generator.

## Safety and Failure Handling

- Paths with fewer than two points are not optimized.
- Non-finite output, output outside the costmap, lethal cells, disallowed
  unknown cells, or colliding output segments reject the result.
- The first and last output poses are forced to the original first and last
  poses so the planner contract and exact goal pose are preserved.
- Any optimizer failure logs a throttled warning and returns the original
  global path.
- The upstream optimizer's boolean return is corrected so non-negative L-BFGS
  status codes mean success; the supplied version incorrectly converts
  convergence code `0` to `false`.

## Testing

Unit tests cover costmap conversion, short-path rejection, endpoint
preservation, finite output, and collision rejection. The package is then built
and its tests run through Catkin.
