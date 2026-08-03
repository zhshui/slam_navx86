#include <cmath>

#include <gtest/gtest.h>

#include <teb_local_planner/terminal_pid.h>

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

TEST(TerminalConvergence, entersAtThreshold)
{
  EXPECT_DOUBLE_EQ(teb_local_planner::kTerminalPidEnterDistance, 0.45);
  EXPECT_DOUBLE_EQ(teb_local_planner::kTerminalPidExitDistance, 0.65);
  EXPECT_TRUE(teb_local_planner::updateTerminalConvergence(
      false, 0.45,
      teb_local_planner::kTerminalPidEnterDistance,
      teb_local_planner::kTerminalPidExitDistance));
}

TEST(TerminalConvergence, remainsLatchedDuringSmallPositionCorrection)
{
  EXPECT_TRUE(teb_local_planner::updateTerminalConvergence(
      true, 0.55,
      teb_local_planner::kTerminalPidEnterDistance,
      teb_local_planner::kTerminalPidExitDistance));
}

TEST(TerminalConvergence, exitsOnlyAfterLargeDisplacement)
{
  EXPECT_TRUE(teb_local_planner::updateTerminalConvergence(
      true, 0.65,
      teb_local_planner::kTerminalPidEnterDistance,
      teb_local_planner::kTerminalPidExitDistance));
  EXPECT_FALSE(teb_local_planner::updateTerminalConvergence(
      true, 0.66,
      teb_local_planner::kTerminalPidEnterDistance,
      teb_local_planner::kTerminalPidExitDistance));
}

TEST(TerminalPid, translatesWhileCorrectingPositiveYaw)
{
  teb_local_planner::TerminalPidController controller;
  const geometry_msgs::Twist command =
      controller.calculate(0.10, 0.05, 0.40, 0.0, 0.1, 0.08, 0.05);

  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_GT(command.linear.y, 0.0);
  EXPECT_GT(command.angular.z, 0.0);
}

TEST(TerminalPid, usesNegativeLateralVelocityForRightSideError)
{
  teb_local_planner::TerminalPidController controller;
  const geometry_msgs::Twist command =
      controller.calculate(0.10, -0.05, 0.0, 0.0, 0.1, 0.08, 0.05);

  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_LT(command.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST(TerminalPid, drivesForwardWhileCorrectingFinalYaw)
{
  teb_local_planner::TerminalPidController controller;
  const double position_heading = std::atan2(0.05, 0.10);
  const geometry_msgs::Twist command =
      controller.calculate(
          0.10, 0.05, -1.0, position_heading, 0.1, 0.08, 0.05);

  EXPECT_GE(command.linear.x + 1e-12, 0.25);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-12);
  EXPECT_LT(command.angular.z, 0.0);
}

TEST(TerminalPid, correctsPositionAndYawTogetherNearGoal)
{
  teb_local_planner::TerminalPidController controller;
  const geometry_msgs::Twist command =
      controller.calculate(0.03, 0.02, 0.40, 0.0, 0.1, 0.08, 0.05);

  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_GT(command.linear.y, 0.0);
  EXPECT_GT(command.angular.z, 0.0);
}

TEST(TerminalPid, keepsTranslatingThroughSmallPositionDrift)
{
  teb_local_planner::TerminalPidController controller;
  controller.calculate(0.07, 0.0, 0.40, 0.0, 0.1, 0.08, 0.05);
  const geometry_msgs::Twist command =
      controller.calculate(0.09, 0.0, 0.40, 0.0, 0.1, 0.08, 0.05);

  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-12);
  EXPECT_GT(command.angular.z, 0.0);
}

TEST(TerminalPid, usesMinimumEffectiveYawVelocity)
{
  teb_local_planner::TerminalPidController controller;
  const geometry_msgs::Twist command =
      controller.calculate(0.0, 0.0, 0.07, 0.0, 0.1, 0.08, 0.05);

  EXPECT_GE(command.angular.z, 0.25);
}

TEST(TerminalPid, capsTranslationAndYawVelocity)
{
  teb_local_planner::TerminalPidController translation_controller;
  const geometry_msgs::Twist translation_command =
      translation_controller.calculate(
          1.0, 1.0, 2.0, kPi / 4.0, 0.1, 0.08, 0.05);
  EXPECT_LE(translation_command.linear.x, 0.30 + 1e-9);
  EXPECT_LE(std::fabs(translation_command.linear.y), 0.10 + 1e-9);

  teb_local_planner::TerminalPidController yaw_controller;
  const geometry_msgs::Twist yaw_command =
      yaw_controller.calculate(0.0, 0.0, 2.0, 0.0, 0.1, 0.08, 0.05);
  EXPECT_LE(std::fabs(yaw_command.angular.z), 0.60 + 1e-9);
}

TEST(TerminalPid, zerosAxesInsideControllerDeadbands)
{
  teb_local_planner::TerminalPidController controller;
  const geometry_msgs::Twist command =
      controller.calculate(0.005, 0.005, 0.03, 0.0, 0.1, 0.08, 0.05);

  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST(TerminalPid, clampsIntegralAccumulation)
{
  teb_local_planner::TerminalPidConfig config;
  config.xy_kp = 0.0;
  config.xy_ki = 1.0;
  config.xy_kd = 0.0;
  config.xy_integral_limit = 0.10;
  config.min_translation_velocity = 0.0;
  config.max_translation_velocity = 1.0;
  teb_local_planner::TerminalPidController controller(config);

  geometry_msgs::Twist command;
  for (int i = 0; i < 20; ++i)
    command = controller.calculate(0.20, 0.0, 0.0, 0.0, 0.20, 0.08, 0.05);

  EXPECT_NEAR(command.linear.x, 0.10, 1e-9);
}

TEST(TerminalPid, filtersDerivativeAndResetPreventsKick)
{
  teb_local_planner::TerminalPidConfig config;
  config.xy_kp = 0.0;
  config.xy_ki = 0.0;
  config.xy_kd = 1.0;
  config.derivative_filter_factor = 0.20;
  config.min_translation_velocity = 0.0;
  config.max_translation_velocity = 2.0;
  teb_local_planner::TerminalPidController controller(config);

  EXPECT_DOUBLE_EQ(
      controller.calculate(0.10, 0.0, 0.0, 0.0, 0.10, 0.01, 0.05)
          .linear.x,
      0.0);
  EXPECT_NEAR(
      controller.calculate(0.20, 0.0, 0.0, 0.0, 0.10, 0.01, 0.05)
          .linear.x,
      0.20, 1e-9);

  controller.reset();
  EXPECT_DOUBLE_EQ(
      controller.calculate(0.20, 0.0, 0.0, 0.0, 0.10, 0.01, 0.05)
          .linear.x,
      0.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
