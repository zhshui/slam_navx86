#ifndef GLOBAL_PLANNER_TRAJECTORY_OPTIMIZER_ADAPTER_H
#define GLOBAL_PLANNER_TRAJECTORY_OPTIMIZER_ADAPTER_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>

namespace global_planner
{

struct TrajectoryOptimizerConfig
{
  double max_velocity = 3.0;
  double safe_distance = 0.5;
  double sample_dt = 0.1;
  int max_iterations = 1000;
};

class TrajectoryOptimizerAdapter
{
public:
  static bool optimize(
    const costmap_2d::Costmap2D & costmap, bool allow_unknown,
    const std::vector<geometry_msgs::PoseStamped> & input,
    const TrajectoryOptimizerConfig & config,
    std::vector<geometry_msgs::PoseStamped> & output);

  static bool isCollisionFree(
    const costmap_2d::Costmap2D & costmap, bool allow_unknown,
    const std::vector<geometry_msgs::PoseStamped> & path);
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_TRAJECTORY_OPTIMIZER_ADAPTER_H
