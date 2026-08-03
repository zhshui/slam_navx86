#ifndef TEB_LOCAL_PLANNER_PATH_HEADING_H_
#define TEB_LOCAL_PLANNER_PATH_HEADING_H_

#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace teb_local_planner
{

double estimatePathHeading(const std::vector<geometry_msgs::PoseStamped>& path,
                           double lookback_dist,
                           double goal_x,
                           double goal_y,
                           double robot_x,
                           double robot_y,
                           double robot_yaw);

geometry_msgs::Twist calculatePathAlignedApproach(double dx_global,
                                                  double dy_global,
                                                  double robot_yaw,
                                                  double path_yaw,
                                                  double deadband);

}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER_PATH_HEADING_H_
