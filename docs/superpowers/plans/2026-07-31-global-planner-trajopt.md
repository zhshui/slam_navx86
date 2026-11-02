# Global Planner TrajOpt Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Optimize the complete ROS global path inside `GlobalPlanner::makePlan()` using the supplied TrajOpt implementation and safely fall back to the original plan.

**Architecture:** Vendor the four header-only optimizer components under the `global_planner` include tree. Isolate conversion, optimization, and output validation in `TrajectoryOptimizerAdapter`; `planner_core.cpp` only invokes the adapter between path extraction and orientation filtering.

**Tech Stack:** ROS Noetic, Catkin, C++14, Eigen3, costmap_2d, GoogleTest, L-BFGS, cubic splines

## Global Constraints

- Preserve `global_planner/GlobalPlanner` and both `makePlan()` signatures.
- Preserve the existing plan publisher and output message types.
- Never return an unvalidated optimized path.
- Fall back to the original path on every optimization failure.
- Preserve exact first and last poses.
- Do not modify unrelated worktree changes.

---

### Task 1: Add a tested adapter boundary

**Files:**
- Create: `lite_cog/nav/src/global_planner/include/global_planner/trajectory_optimizer_adapter.h`
- Create: `lite_cog/nav/src/global_planner/test/trajectory_optimizer_adapter_test.cpp`
- Modify: `lite_cog/nav/src/global_planner/CMakeLists.txt`

**Interfaces:**
- Consumes: `costmap_2d::Costmap2D`, `std::vector<geometry_msgs::PoseStamped>`
- Produces: `global_planner::TrajectoryOptimizerAdapter::optimize(...)`

- [ ] Write tests for short-path rejection, endpoint preservation, and collision validation.
- [ ] Add the test target and run it to confirm it fails because the adapter implementation is missing.

### Task 2: Vendor TrajOpt and implement the adapter

**Files:**
- Create: `lite_cog/nav/src/global_planner/include/global_planner/trajectory_optimizer/LBFGS.hpp`
- Create: `lite_cog/nav/src/global_planner/include/global_planner/trajectory_optimizer/SplineTrajectory.hpp`
- Create: `lite_cog/nav/src/global_planner/include/global_planner/trajectory_optimizer/GridMap.hpp`
- Create: `lite_cog/nav/src/global_planner/include/global_planner/trajectory_optimizer/TrajectoryOptimizer.hpp`
- Create: `lite_cog/nav/src/global_planner/src/trajectory_optimizer_adapter.cpp`
- Modify: `lite_cog/nav/src/global_planner/CMakeLists.txt`
- Modify: `lite_cog/nav/src/global_planner/package.xml`

**Interfaces:**
- Consumes: adapter configuration, costmap, source plan
- Produces: validated optimized pose sequence

- [ ] Copy the exact upstream optimizer components and rewrite only their local include paths.
- [ ] Correct `TrajectoryOptimizer::plan()` to return success for non-negative L-BFGS codes.
- [ ] Implement costmap conversion, path conversion, optimization, sampling, and collision validation.
- [ ] Build and run the adapter test until it passes.

### Task 3: Integrate the adapter into GlobalPlanner

**Files:**
- Modify: `lite_cog/nav/src/global_planner/include/global_planner/planner_core.h`
- Modify: `lite_cog/nav/src/global_planner/src/planner_core.cpp`
- Modify: `lite_cog/nav/src/navigation/config/global_planner_params.yaml`

**Interfaces:**
- Consumes: original `plan` produced by `getPlanFromPotential()`
- Produces: optimized `plan` through the existing `makePlan()` output and publisher

- [ ] Load the five trajectory optimization parameters during planner initialization.
- [ ] Invoke the adapter before orientation filtering.
- [ ] Replace `plan` only when optimization and validation succeed.
- [ ] Add the default parameters to the navigation configuration.
- [ ] Build `global_planner`, run its tests, and verify ROS resolves the workspace plugin.
