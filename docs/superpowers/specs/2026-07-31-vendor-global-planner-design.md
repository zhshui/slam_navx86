# 引入 ROS Noetic global_planner 源码设计

## 目标

将与本机二进制包版本一致的 ROS Navigation `global_planner` 1.17.3 官方源码加入
`lite_cog/nav/src/global_planner`，使后续可以在工作区内修改
`global_planner::GlobalPlanner::makePlan()`，而不覆盖 `/opt/ros/noetic` 中的系统文件。

## 边界

- 源码固定使用 ROS Navigation Stack 1.17.3 的 `global_planner` 包。
- 保持插件名称 `global_planner/GlobalPlanner` 不变。
- 保持 `nav_core::BaseGlobalPlanner` 的 `makePlan()` 接口不变。
- 不修改现有 `move_base_params.yaml`；工作区环境被加载后，同名工作区包应优先于系统包。
- 本阶段不接入 `TrajOpt`，因为当前机器没有其头文件、实现或链接库。
- 不修改或提交工作区内已有的地图、数据库及日志变化。

## 实现

从 ROS 官方 `ros-planning/navigation` 仓库的 `1.17.3` 标签取得
`global_planner` 子目录，放入 Catkin 工作空间的 `src` 下。保留上游包结构、插件描述、
动态参数和测试，不改动规划算法。

随后对 `global_planner` 包执行定向 Catkin 编译，并在加载
`lite_cog/nav/devel/setup.bash` 后检查 `rospack find global_planner`，确认解析到工作区源码。

## 验证

1. `package.xml` 的版本必须为 `1.17.3`。
2. `catkin_make --only-pkg-with-deps global_planner` 必须成功。
3. 加载工作区环境后，`rospack find global_planner` 必须输出
   `/home/unitree/go2_nav/lite_cog/nav/src/global_planner`。

