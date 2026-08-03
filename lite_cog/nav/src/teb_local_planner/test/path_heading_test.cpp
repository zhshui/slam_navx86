#include <cmath>
#include <initializer_list>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <teb_local_planner/path_heading.h>

namespace
{

std::vector<geometry_msgs::PoseStamped> makePath(
    std::initializer_list<std::pair<double, double> > points)
{
  std::vector<geometry_msgs::PoseStamped> path;
  for (const std::pair<double, double>& point : points)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    path.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(PathHeading, followsTwoPointTestRoute)
{
  const std::vector<geometry_msgs::PoseStamped> path =
      makePath({{-1.1, -1.6}, {6.3, -0.3}});

  EXPECT_NEAR(teb_local_planner::estimatePathHeading(
                  path, 0.8, 6.3, -0.3, -1.1, -1.6, 0.0),
              0.17390118913788663, 1e-5);
}

TEST(PathHeading, ignoresNoisyFinalEdge)
{
  const std::vector<geometry_msgs::PoseStamped> path =
      makePath({{0.0, 0.0}, {0.4, 0.0}, {0.8, 0.0},
                {1.2, 0.0}, {1.2, 0.05}});

  EXPECT_NEAR(teb_local_planner::estimatePathHeading(
                  path, 0.8, 1.2, 0.05, 0.0, 0.0, 0.0),
              std::atan2(0.05, 0.8), 1e-5);
}

TEST(PathHeading, usesAvailableShortPath)
{
  const std::vector<geometry_msgs::PoseStamped> path =
      makePath({{1.0, 2.0}, {1.3, 2.3}});

  EXPECT_NEAR(teb_local_planner::estimatePathHeading(
                  path, 0.8, 1.3, 2.3, 1.0, 2.0, -0.5),
              0.7853981633974483, 1e-5);
}

TEST(PathHeading, fallsBackToRobotToGoalForDuplicatePath)
{
  const std::vector<geometry_msgs::PoseStamped> path =
      makePath({{1.0, 1.0}, {1.0, 1.0}});

  EXPECT_NEAR(teb_local_planner::estimatePathHeading(
                  path, 0.8, 2.0, 2.0, 1.0, 1.0, -0.5),
              0.7853981633974483, 1e-5);
}

TEST(PathHeading, fallsBackToRobotYawAtGoal)
{
  const std::vector<geometry_msgs::PoseStamped> path;

  EXPECT_DOUBLE_EQ(teb_local_planner::estimatePathHeading(
                       path, 0.8, 1.0, 1.0, 1.0, 1.0, -0.5),
                   -0.5);
}

TEST(PathAlignedApproach, rotatesBeforeTranslatingWhenHeadingIsFarOff)
{
  const geometry_msgs::Twist command =
      teb_local_planner::calculatePathAlignedApproach(
          0.042, 0.163, -0.4188790204786391, 0.7853981633974483, 0.06);

  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
  EXPECT_GT(command.angular.z, 0.0);
  EXPECT_LE(command.angular.z, 0.6);
}

TEST(PathAlignedApproach, rotatesBeforeTranslatingForModerateHeadingError)
{
  const geometry_msgs::Twist command =
      teb_local_planner::calculatePathAlignedApproach(
          0.18, 0.0, 0.0, 0.3490658503988659, 0.06);

  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
  EXPECT_GT(command.angular.z, 0.0);
}

TEST(PathAlignedApproach, drivesForwardAfterPathHeadingAlignment)
{
  const geometry_msgs::Twist command =
      teb_local_planner::calculatePathAlignedApproach(
          0.12, 0.08, 0.0, 0.0, 0.06);

  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_GE(command.linear.x, 0.08);
  EXPECT_GT(command.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST(PathAlignedApproach, stopsInsidePositionDeadband)
{
  const geometry_msgs::Twist command =
      teb_local_planner::calculatePathAlignedApproach(
          0.02, 0.01, 0.0, 0.5, 0.06);

  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
