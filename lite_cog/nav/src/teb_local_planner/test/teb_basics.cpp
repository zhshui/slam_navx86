#include <gtest/gtest.h>

#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/timed_elastic_band.h>

namespace
{
class TestableTebOptimalPlanner : public teb_local_planner::TebOptimalPlanner
{
public:
  using teb_local_planner::TebOptimalPlanner::TebOptimalPlanner;
  using teb_local_planner::TebOptimalPlanner::AddEdgesViaPoints;
  using teb_local_planner::TebOptimalPlanner::AddTEBVertices;
  using teb_local_planner::TebOptimalPlanner::clearGraph;
};
}  // namespace

TEST(TEBBasic, autoResizeLargeValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a large timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt + 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResizeSmallValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a small timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResize)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // modify the timediff in the middle and add a pose with a smaller timediff as the last one
  teb.TimeDiff(5) = dt + 2*dt_hysteresis;
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, acceptsContinuousWarmStart)
{
  teb_local_planner::TimedElasticBand teb;
  teb.addPose(teb_local_planner::PoseSE2(0.0, 0.0, 0.0));
  teb.addPoseAndTimeDiff(
      teb_local_planner::PoseSE2(0.25, 0.0, 0.0), 0.5);
  teb.addPoseAndTimeDiff(
      teb_local_planner::PoseSE2(0.50, 0.0, 0.0), 0.5);

  EXPECT_FALSE(teb.hasLargeInitialGap(1.0));
}

TEST(TEBBasic, rejectsWarmStartWithLargeInitialJump)
{
  teb_local_planner::TimedElasticBand teb;
  teb.addPose(teb_local_planner::PoseSE2(2.24, -1.46, 0.0));
  teb.addPoseAndTimeDiff(
      teb_local_planner::PoseSE2(5.07, -0.99, 0.0), 0.5);
  teb.addPoseAndTimeDiff(
      teb_local_planner::PoseSE2(5.04, -1.00, 0.0), 0.5);

  EXPECT_TRUE(teb.hasLargeInitialGap(1.0));
}

TEST(TEBBasic, estimatesIntermediateYawFromPathGeometry)
{
  std::vector<geometry_msgs::PoseStamped> plan(3);
  for (geometry_msgs::PoseStamped& pose : plan)
    pose.pose.orientation.w = 1.0;
  plan[0].pose.position.x = 0.0;
  plan[1].pose.position.x = 0.5;
  plan[2].pose.position.x = 1.0;
  plan[1].pose.orientation.z = std::sin(M_PI / 4.0);
  plan[1].pose.orientation.w = std::cos(M_PI / 4.0);

  teb_local_planner::TimedElasticBand teb;
  ASSERT_TRUE(teb.initTrajectoryToGoal(
      plan, 0.5, 1.0, true, 3, false));

  EXPECT_NEAR(teb.Pose(1).theta(), 0.0, 1e-9);
}

TEST(TEBBasic, stopsOrderedViaPointAssociationWhenTrajectoryIsExhausted)
{
  teb_local_planner::TebConfig cfg;
  cfg.optim.weight_viapoint = 1.0;
  cfg.trajectory.via_points_ordered = true;

  teb_local_planner::ViaPointContainer via_points;
  for (int i = 1; i <= 10; ++i)
    via_points.emplace_back(0.1 * i, 0.0);

  teb_local_planner::ObstContainer obstacles;
  TestableTebOptimalPlanner planner(
      cfg, &obstacles, teb_local_planner::TebVisualizationPtr(), &via_points);
  planner.teb().addPose(teb_local_planner::PoseSE2(0.0, 0.0, 0.0));
  planner.teb().addPoseAndTimeDiff(
      teb_local_planner::PoseSE2(1.0, 0.0, 0.0), 0.5);
  planner.teb().addPoseAndTimeDiff(
      teb_local_planner::PoseSE2(2.0, 0.0, 0.0), 0.5);
  planner.teb().addPoseAndTimeDiff(
      teb_local_planner::PoseSE2(3.0, 0.0, 0.0), 0.5);

  planner.AddTEBVertices();
  planner.AddEdgesViaPoints();

  EXPECT_EQ(planner.optimizer()->edges().size(), 2u);
  planner.clearGraph();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
