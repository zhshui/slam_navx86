# Vendor ROS Noetic Global Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the official ROS Navigation `global_planner` 1.17.3 source package to the local Catkin workspace and verify that it overrides the system-installed plugin.

**Architecture:** Vendor the unmodified upstream package under `lite_cog/nav/src/global_planner`. Build it as a normal Catkin package so the workspace produces its own `libglobal_planner.so`; preserve the existing plugin name and `move_base` configuration.

**Tech Stack:** ROS Noetic, Catkin, C++14, `nav_core`, `costmap_2d`, `pluginlib`

## Global Constraints

- Use exactly ROS Navigation Stack version `1.17.3`.
- Do not overwrite files under `/opt/ros/noetic`.
- Do not alter the plugin name `global_planner/GlobalPlanner`.
- Do not include unrelated existing worktree changes in any commit.
- Do not add TrajOpt integration in this phase because its buildable source is unavailable.

---

### Task 1: Add and verify the upstream package

**Files:**
- Create: `lite_cog/nav/src/global_planner/**`
- Verify: `lite_cog/nav/src/global_planner/package.xml`
- Verify: `lite_cog/nav/src/global_planner/src/planner_core.cpp`

**Interfaces:**
- Consumes: ROS Noetic libraries installed under `/opt/ros/noetic`
- Produces: Catkin package `global_planner` and plugin library `libglobal_planner.so`

- [ ] **Step 1: Download the official 1.17.3 source archive**

Download `https://github.com/ros-planning/navigation/archive/refs/tags/1.17.3.tar.gz`
to a temporary directory and verify that the archive contains
`navigation-1.17.3/global_planner/package.xml`.

- [ ] **Step 2: Install only the global_planner subdirectory**

Extract `navigation-1.17.3/global_planner` as
`lite_cog/nav/src/global_planner`; do not copy other upstream packages.

- [ ] **Step 3: Verify source identity**

Run:

```bash
grep -n '<version>1.17.3</version>' lite_cog/nav/src/global_planner/package.xml
test -f lite_cog/nav/src/global_planner/src/planner_core.cpp
```

Expected: `grep` prints the version line and `test` exits with status zero.

- [ ] **Step 4: Build the package and its workspace dependencies**

Run from `lite_cog/nav`:

```bash
catkin_make --only-pkg-with-deps global_planner
```

Expected: Catkin exits with status zero and produces
`devel/lib/libglobal_planner.so`.

- [ ] **Step 5: Verify ROS package precedence**

Run:

```bash
bash -c 'source /opt/ros/noetic/setup.bash && source lite_cog/nav/devel/setup.bash && rospack find global_planner'
```

Expected output:

```text
/home/unitree/go2_nav/lite_cog/nav/src/global_planner
```

- [ ] **Step 6: Inspect the scoped diff**

Run:

```bash
git status --short -- docs/superpowers/specs/2026-07-31-vendor-global-planner-design.md docs/superpowers/plans/2026-07-31-vendor-global-planner.md lite_cog/nav/src/global_planner
```

Expected: only the two documentation files and the new package appear.
