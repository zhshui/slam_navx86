#include <global_planner/trajectory_optimizer_adapter.h>

#include <global_planner/trajectory_optimizer/GridMap.hpp>
#include <global_planner/trajectory_optimizer/TrajectoryOptimizer.hpp>

#include <costmap_2d/cost_values.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace global_planner
{
namespace
{

bool isBlocked(
  const costmap_2d::Costmap2D & costmap, bool allow_unknown, double wx, double wy)
{
  unsigned int mx;
  unsigned int my;
  if (!costmap.worldToMap(wx, wy, mx, my)) {
    return true;
  }

  const unsigned char cost = costmap.getCost(mx, my);
  if (cost == costmap_2d::NO_INFORMATION) {
    return !allow_unknown;
  }
  return cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

std::shared_ptr<grid_map::GridMap> makeGridMap(
  const costmap_2d::Costmap2D & costmap, bool allow_unknown)
{
  const unsigned int width = costmap.getSizeInCellsX();
  const unsigned int height = costmap.getSizeInCellsY();
  const double resolution = costmap.getResolution();

  auto grid_map = std::make_shared<grid_map::GridMap>();
  grid_map->init(
    width * resolution, height * resolution, resolution,
    Eigen::Vector2d(costmap.getOriginX(), costmap.getOriginY()));

  grid_map::RowMatrixXi occupancy(width, height);
  for (unsigned int x = 0; x < width; ++x) {
    for (unsigned int y = 0; y < height; ++y) {
      const unsigned char cost = costmap.getCost(x, y);
      const bool unknown = cost == costmap_2d::NO_INFORMATION;
      occupancy(x, y) =
        ((!unknown && cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) ||
         (unknown && !allow_unknown))
          ? 1
          : 0;
    }
  }
  grid_map->setMap(occupancy);
  return grid_map;
}

double pathLength(const std::vector<Eigen::Vector2d> & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

}  // namespace

bool TrajectoryOptimizerAdapter::isCollisionFree(
  const costmap_2d::Costmap2D & costmap, bool allow_unknown,
  const std::vector<geometry_msgs::PoseStamped> & path)
{
  if (path.empty()) {
    return false;
  }

  const double step = std::max(0.5 * costmap.getResolution(), 1e-3);
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double x = path[i].pose.position.x;
    const double y = path[i].pose.position.y;
    if (!std::isfinite(x) || !std::isfinite(y) || isBlocked(costmap, allow_unknown, x, y)) {
      return false;
    }

    if (i == 0) {
      continue;
    }

    const Eigen::Vector2d p0(
      path[i - 1].pose.position.x, path[i - 1].pose.position.y);
    const Eigen::Vector2d p1(x, y);
    const Eigen::Vector2d delta = p1 - p0;
    const double distance = delta.norm();
    if (!std::isfinite(distance)) {
      return false;
    }
    if (distance <= 1e-9) {
      continue;
    }

    const Eigen::Vector2d direction = delta / distance;
    for (double offset = step; offset < distance; offset += step) {
      const Eigen::Vector2d sample = p0 + direction * offset;
      if (isBlocked(costmap, allow_unknown, sample.x(), sample.y())) {
        return false;
      }
    }
  }
  return true;
}

bool TrajectoryOptimizerAdapter::optimize(
  const costmap_2d::Costmap2D & costmap, bool allow_unknown,
  const std::vector<geometry_msgs::PoseStamped> & input,
  const TrajectoryOptimizerConfig & config,
  std::vector<geometry_msgs::PoseStamped> & output)
{
  if (
    input.size() < 2 || config.max_velocity <= 0.0 || config.safe_distance < 0.0 ||
    config.sample_dt <= 0.0 || config.max_iterations <= 0) {
    return false;
  }

  std::vector<Eigen::Vector2d> path;
  path.reserve(input.size());
  for (const auto & pose : input) {
    const Eigen::Vector2d point(pose.pose.position.x, pose.pose.position.y);
    if (!point.allFinite()) {
      return false;
    }
    if (path.empty() || (point - path.back()).norm() > 1e-6) {
      path.push_back(point);
    }
  }
  if (path.size() < 2) {
    return false;
  }

  const double total_length = pathLength(path);
  if (!std::isfinite(total_length) || total_length <= 1e-6) {
    return false;
  }

  TrajOpt::TrajectoryParams params;
  params.total_len = total_length;
  params.total_time = total_length / config.max_velocity;
  params.piece_len = total_length / params.total_time;
  params.max_v = config.max_velocity;
  params.safe_threshold = config.safe_distance;
  params.max_iter = config.max_iterations;

  auto grid_map = makeGridMap(costmap, allow_unknown);
  TrajOpt::TrajectoryOptimizer optimizer(grid_map, path, params);
  if (!optimizer.plan()) {
    return false;
  }

  const std::vector<Eigen::Vector2d> samples = optimizer.sampleTrajectory(config.sample_dt);
  if (samples.size() < 2) {
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> candidate;
  candidate.reserve(samples.size());
  for (const auto & point : samples) {
    if (!point.allFinite()) {
      return false;
    }
    geometry_msgs::PoseStamped pose = input.front();
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    candidate.push_back(pose);
  }

  candidate.front() = input.front();
  candidate.back() = input.back();
  if (!isCollisionFree(costmap, allow_unknown, candidate)) {
    return false;
  }

  output.swap(candidate);
  return true;
}

}  // namespace global_planner
