#include <global_planner/trajectory_optimizer_adapter.h>

#include <costmap_2d/cost_values.h>
#include <gtest/gtest.h>

namespace
{

geometry_msgs::PoseStamped pose(double x, double y)
{
  geometry_msgs::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.w = 1.0;
  return result;
}

costmap_2d::Costmap2D freeMap()
{
  return costmap_2d::Costmap2D(40, 40, 0.1, 0.0, 0.0, costmap_2d::FREE_SPACE);
}

TEST(TrajectoryOptimizerAdapter, RejectsPathsWithFewerThanTwoPoses)
{
  auto costmap = freeMap();
  const std::vector<geometry_msgs::PoseStamped> input{pose(0.5, 0.5)};
  std::vector<geometry_msgs::PoseStamped> output{pose(9.0, 9.0)};

  EXPECT_FALSE(global_planner::TrajectoryOptimizerAdapter::optimize(
    costmap, false, input, global_planner::TrajectoryOptimizerConfig(), output));
  ASSERT_EQ(1u, output.size());
  EXPECT_DOUBLE_EQ(9.0, output.front().pose.position.x);
}

TEST(TrajectoryOptimizerAdapter, CollisionValidationRejectsABlockedSegment)
{
  auto costmap = freeMap();
  costmap.setCost(20, 20, costmap_2d::LETHAL_OBSTACLE);
  const std::vector<geometry_msgs::PoseStamped> path{pose(1.0, 2.05), pose(3.0, 2.05)};

  EXPECT_FALSE(global_planner::TrajectoryOptimizerAdapter::isCollisionFree(
    costmap, false, path));
}

TEST(TrajectoryOptimizerAdapter, CollisionValidationHonorsUnknownPolicy)
{
  auto costmap = freeMap();
  costmap.setCost(20, 20, costmap_2d::NO_INFORMATION);
  const std::vector<geometry_msgs::PoseStamped> path{pose(1.0, 2.05), pose(3.0, 2.05)};

  EXPECT_FALSE(global_planner::TrajectoryOptimizerAdapter::isCollisionFree(
    costmap, false, path));
  EXPECT_TRUE(global_planner::TrajectoryOptimizerAdapter::isCollisionFree(
    costmap, true, path));
}

TEST(TrajectoryOptimizerAdapter, OptimizedPathPreservesExactEndpoints)
{
  auto costmap = freeMap();
  std::vector<geometry_msgs::PoseStamped> input{
    pose(0.5, 0.5), pose(1.2, 0.8), pose(2.0, 0.5), pose(3.0, 0.5)};
  input.front().header.stamp = ros::Time(10.0);
  input.back().header.stamp = ros::Time(20.0);

  global_planner::TrajectoryOptimizerConfig config;
  config.safe_distance = 0.2;
  config.max_iterations = 100;

  std::vector<geometry_msgs::PoseStamped> output;
  ASSERT_TRUE(global_planner::TrajectoryOptimizerAdapter::optimize(
    costmap, false, input, config, output));
  ASSERT_GE(output.size(), 2u);
  EXPECT_EQ(input.front().header, output.front().header);
  EXPECT_EQ(input.back().header, output.back().header);
  EXPECT_DOUBLE_EQ(input.front().pose.position.x, output.front().pose.position.x);
  EXPECT_DOUBLE_EQ(input.front().pose.position.y, output.front().pose.position.y);
  EXPECT_DOUBLE_EQ(input.back().pose.position.x, output.back().pose.position.x);
  EXPECT_DOUBLE_EQ(input.back().pose.position.y, output.back().pose.position.y);
  EXPECT_TRUE(global_planner::TrajectoryOptimizerAdapter::isCollisionFree(
    costmap, false, output));
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
