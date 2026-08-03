#ifndef TEB_LOCAL_PLANNER_TERMINAL_PID_H_
#define TEB_LOCAL_PLANNER_TERMINAL_PID_H_

#include <geometry_msgs/Twist.h>

namespace teb_local_planner
{

constexpr double kTerminalPidEnterDistance = 0.45;
constexpr double kTerminalPidExitDistance = 0.65;

struct TerminalPidConfig
{
  TerminalPidConfig();

  double xy_kp;
  double xy_ki;
  double xy_kd;
  double yaw_kp;
  double yaw_ki;
  double yaw_kd;
  double xy_integral_limit;
  double yaw_integral_limit;
  double derivative_filter_factor;
  double min_translation_velocity;
  double max_translation_velocity;
  double max_lateral_velocity;
  double min_yaw_velocity;
  double max_yaw_velocity;
};

bool updateTerminalConvergence(bool active,
                               double distance,
                               double enter_distance,
                               double exit_distance);

class TerminalPidController
{
public:
  TerminalPidController();
  explicit TerminalPidController(const TerminalPidConfig& config);

  void reset();

  geometry_msgs::Twist calculate(double dx_global,
                                 double dy_global,
                                 double yaw_error,
                                 double robot_yaw,
                                 double dt,
                                 double xy_tolerance,
                                 double yaw_tolerance);

private:
  TerminalPidConfig config_;
  bool initialized_;
  double integral_x_;
  double integral_y_;
  double integral_yaw_;
  double previous_error_x_;
  double previous_error_y_;
  double previous_error_yaw_;
  double filtered_derivative_x_;
  double filtered_derivative_y_;
  double filtered_derivative_yaw_;
};

}  // namespace teb_local_planner

#endif  // TEB_LOCAL_PLANNER_TERMINAL_PID_H_
