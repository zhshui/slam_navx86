#include <teb_local_planner/terminal_pid.h>

#include <algorithm>
#include <cmath>

namespace teb_local_planner
{
namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void limitVector(double limit, double& x, double& y)
{
  const double magnitude = std::hypot(x, y);
  if (magnitude > limit && magnitude > 0.0)
  {
    const double scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

}  // namespace

TerminalPidConfig::TerminalPidConfig()
    : xy_kp(1.5),
      xy_ki(0.05),
      xy_kd(0.10),
      yaw_kp(2.0),
      yaw_ki(0.05),
      yaw_kd(0.10),
      xy_integral_limit(0.10),
      yaw_integral_limit(0.20),
      derivative_filter_factor(0.20),
      min_translation_velocity(0.25),
      max_translation_velocity(0.30),
      max_lateral_velocity(0.10),
      min_yaw_velocity(0.25),
      max_yaw_velocity(0.60)
{
}

bool updateTerminalConvergence(bool active,
                               double distance,
                               double enter_distance,
                               double exit_distance)
{
  const double effective_exit_distance =
      std::max(enter_distance, exit_distance);
  if (!active)
    return distance <= enter_distance;
  return distance <= effective_exit_distance;
}

TerminalPidController::TerminalPidController()
    : TerminalPidController(TerminalPidConfig())
{
}

TerminalPidController::TerminalPidController(const TerminalPidConfig& config)
    : config_(config)
{
  reset();
}

void TerminalPidController::reset()
{
  initialized_ = false;
  integral_x_ = 0.0;
  integral_y_ = 0.0;
  integral_yaw_ = 0.0;
  previous_error_x_ = 0.0;
  previous_error_y_ = 0.0;
  previous_error_yaw_ = 0.0;
  filtered_derivative_x_ = 0.0;
  filtered_derivative_y_ = 0.0;
  filtered_derivative_yaw_ = 0.0;
}

geometry_msgs::Twist TerminalPidController::calculate(
    double dx_global,
    double dy_global,
    double yaw_error,
    double robot_yaw,
    double dt,
    double xy_tolerance,
    double yaw_tolerance)
{
  geometry_msgs::Twist command;
  yaw_error = normalizeAngle(yaw_error);
  const double distance = std::hypot(dx_global, dy_global);
  const double position_deadband = std::min(0.02, 0.25 * xy_tolerance);

  if (!std::isfinite(dt) || dt <= 0.0)
  {
    reset();
    dt = 0.01;
  }
  dt = clamp(dt, 0.01, 0.20);

  if (!initialized_)
  {
    previous_error_x_ = dx_global;
    previous_error_y_ = dy_global;
    previous_error_yaw_ = yaw_error;
    initialized_ = true;
  }
  else
  {
    const double filter = clamp(config_.derivative_filter_factor, 0.0, 1.0);
    const double raw_derivative_x =
        (dx_global - previous_error_x_) / dt;
    const double raw_derivative_y =
        (dy_global - previous_error_y_) / dt;
    const double raw_derivative_yaw =
        normalizeAngle(yaw_error - previous_error_yaw_) / dt;
    filtered_derivative_x_ =
        filter * raw_derivative_x +
        (1.0 - filter) * filtered_derivative_x_;
    filtered_derivative_y_ =
        filter * raw_derivative_y +
        (1.0 - filter) * filtered_derivative_y_;
    filtered_derivative_yaw_ =
        filter * raw_derivative_yaw +
        (1.0 - filter) * filtered_derivative_yaw_;
  }

  if (distance <= position_deadband)
  {
    integral_x_ = 0.0;
    integral_y_ = 0.0;
    filtered_derivative_x_ = 0.0;
    filtered_derivative_y_ = 0.0;
  }
  else
  {
    integral_x_ += dx_global * dt;
    integral_y_ += dy_global * dt;
    limitVector(config_.xy_integral_limit, integral_x_, integral_y_);

    double velocity_global_x =
        config_.xy_kp * dx_global +
        config_.xy_ki * integral_x_ +
        config_.xy_kd * filtered_derivative_x_;
    double velocity_global_y =
        config_.xy_kp * dy_global +
        config_.xy_ki * integral_y_ +
        config_.xy_kd * filtered_derivative_y_;
    limitVector(
        config_.max_translation_velocity,
        velocity_global_x, velocity_global_y);

    const double translation_speed =
        std::hypot(velocity_global_x, velocity_global_y);
    if (distance > xy_tolerance &&
        translation_speed > 0.0 &&
        translation_speed < config_.min_translation_velocity)
    {
      const double scale =
          std::min(config_.min_translation_velocity,
                   config_.max_translation_velocity) /
          translation_speed;
      velocity_global_x *= scale;
      velocity_global_y *= scale;
    }

    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    command.linear.x =
        cos_yaw * velocity_global_x + sin_yaw * velocity_global_y;
    command.linear.y = clamp(
        -sin_yaw * velocity_global_x + cos_yaw * velocity_global_y,
        -config_.max_lateral_velocity,
        config_.max_lateral_velocity);
  }

  if (std::fabs(yaw_error) <= yaw_tolerance)
  {
    integral_yaw_ = 0.0;
    filtered_derivative_yaw_ = 0.0;
  }
  else
  {
    integral_yaw_ = clamp(
        integral_yaw_ + yaw_error * dt,
        -config_.yaw_integral_limit, config_.yaw_integral_limit);
    command.angular.z = clamp(
        config_.yaw_kp * yaw_error +
            config_.yaw_ki * integral_yaw_ +
            config_.yaw_kd * filtered_derivative_yaw_,
        -config_.max_yaw_velocity, config_.max_yaw_velocity);
    if (command.angular.z != 0.0)
    {
      command.angular.z = std::copysign(
          std::max(
              std::fabs(command.angular.z),
              std::min(config_.min_yaw_velocity,
                       config_.max_yaw_velocity)),
          command.angular.z);
    }
  }

  previous_error_x_ = dx_global;
  previous_error_y_ = dy_global;
  previous_error_yaw_ = yaw_error;
  return command;
}

}  // namespace teb_local_planner
