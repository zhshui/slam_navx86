#include <teb_local_planner/path_heading.h>

#include <algorithm>
#include <cmath>

namespace teb_local_planner
{

double estimatePathHeading(const std::vector<geometry_msgs::PoseStamped>& path,
                           double lookback_dist,
                           double goal_x,
                           double goal_y,
                           double robot_x,
                           double robot_y,
                           double robot_yaw)
{
  const double min_segment_length = 1e-3;

  if (path.size() >= 2)
  {
    const geometry_msgs::Point& end = path.back().pose.position;
    std::size_t start_idx = path.size() - 1;
    double accumulated_dist = 0.0;
    bool found_distinct_pose = false;

    for (std::size_t i = path.size() - 1; i > 0; --i)
    {
      const geometry_msgs::Point& current = path[i].pose.position;
      const geometry_msgs::Point& previous = path[i - 1].pose.position;
      const double segment_length =
          std::hypot(current.x - previous.x, current.y - previous.y);

      if (segment_length <= min_segment_length)
        continue;

      accumulated_dist += segment_length;
      start_idx = i - 1;
      found_distinct_pose = true;
      if (accumulated_dist >= lookback_dist)
        break;
    }

    if (found_distinct_pose)
    {
      const geometry_msgs::Point& start = path[start_idx].pose.position;
      const double dx = end.x - start.x;
      const double dy = end.y - start.y;
      if (std::hypot(dx, dy) > min_segment_length)
        return std::atan2(dy, dx);
    }
  }

  const double robot_to_goal_dx = goal_x - robot_x;
  const double robot_to_goal_dy = goal_y - robot_y;
  if (std::hypot(robot_to_goal_dx, robot_to_goal_dy) > min_segment_length)
    return std::atan2(robot_to_goal_dy, robot_to_goal_dx);

  return robot_yaw;
}

geometry_msgs::Twist calculatePathAlignedApproach(double dx_global,
                                                  double dy_global,
                                                  double robot_yaw,
                                                  double path_yaw,
                                                  double deadband)
{
  geometry_msgs::Twist command;
  const double distance = std::hypot(dx_global, dy_global);
  if (distance <= deadband)
    return command;

  const double heading_error =
      std::atan2(std::sin(path_yaw - robot_yaw), std::cos(path_yaw - robot_yaw));
  const double max_angular_vel = 0.6;
  const double min_angular_vel = 0.25;
  const double angular_deadband = 0.03;
  const double rotate_before_translate_angle = 0.17453292519943295;  // 10 deg

  command.angular.z = std::max(
      -max_angular_vel, std::min(max_angular_vel, 2.0 * heading_error));
  if (std::fabs(heading_error) > angular_deadband &&
      std::fabs(command.angular.z) < min_angular_vel)
  {
    command.angular.z = std::copysign(min_angular_vel, heading_error);
  }
  else if (std::fabs(heading_error) <= angular_deadband)
  {
    command.angular.z = 0.0;
  }

  if (std::fabs(heading_error) > rotate_before_translate_angle)
    return command;

  const double error_scale = (distance - deadband) / distance;
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const double error_robot_x =
      (cos_yaw * dx_global + sin_yaw * dy_global) * error_scale;
  const double error_robot_y =
      (-sin_yaw * dx_global + cos_yaw * dy_global) * error_scale;

  const double max_forward_vel = 0.16;
  const double min_forward_vel = 0.08;
  const double max_lateral_vel = 0.1;
  command.linear.x =
      std::max(0.0, std::min(max_forward_vel, 0.9 * error_robot_x));
  command.linear.y = std::max(
      -max_lateral_vel, std::min(max_lateral_vel, 0.9 * error_robot_y));

  if (command.linear.x > 1e-4 && command.linear.x < min_forward_vel)
    command.linear.x = min_forward_vel;

  return command;
}

}  // namespace teb_local_planner
