#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <fstream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcd2grid");
  ros::NodeHandle nh("~");

  std::string file_path, output_dir;
  double resolution, filter_leaf, min_z, max_z;
  bool enable_voxel_filter, enable_z_filter;
  nh.param("file_path", file_path, std::string(""));
  nh.param("resolution", resolution, 0.05);
  nh.param("filter_leaf_size_m", filter_leaf, 0.1);
  nh.param("enable_voxel_filter", enable_voxel_filter, true);
  nh.param("pointcloud_min_z", min_z, -0.1);
  nh.param("pointcloud_max_z", max_z, 0.6);
  nh.param("enable_z_filter", enable_z_filter, true);
  nh.param("output_dir", output_dir, std::string(""));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(file_path, *cloud) == -1) {
    ROS_ERROR("Failed to load PCD file: %s", file_path.c_str());
    return 1;
  }
  ROS_INFO("Loaded %zu points", cloud->size());

  // Voxel filter (only if enabled and leaf_size > 0)
  if (enable_voxel_filter && filter_leaf > 0.0) {
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filter_leaf, filter_leaf, filter_leaf);
    vg.filter(*cloud);
    ROS_INFO("After voxel filter (leaf=%.3f): %zu points", filter_leaf, cloud->size());
  } else {
    ROS_INFO("Voxel filter disabled");
  }

  // Z filter (only if enabled)
  if (enable_z_filter) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*cloud);
    ROS_INFO("After z filter [%.2f, %.2f]: %zu points", min_z, max_z, cloud->size());
  } else {
    ROS_INFO("Z filter disabled");
  }

  // Find XY bounds
  double min_x = INFINITY, max_x = -INFINITY;
  double min_y = INFINITY, max_y = -INFINITY;
  for (const auto& pt : cloud->points) {
    if (pt.x < min_x) min_x = pt.x;
    if (pt.x > max_x) max_x = pt.x;
    if (pt.y < min_y) min_y = pt.y;
    if (pt.y > max_y) max_y = pt.y;
  }

  int width  = static_cast<int>((max_x - min_x) / resolution) + 1;
  int height = static_cast<int>((max_y - min_y) / resolution) + 1;

  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.header.stamp = ros::Time::now();
  map.info.resolution = resolution;
  map.info.width = width;
  map.info.height = height;
  map.info.origin.position.x = min_x - resolution * 0.5;
  map.info.origin.position.y = min_y - resolution * 0.5;
  map.info.origin.orientation.w = 1.0;
  map.data.resize(width * height, -1);

  // Project to grid
  for (const auto& pt : cloud->points) {
    int i = static_cast<int>((pt.x - min_x) / resolution);
    int j = static_cast<int>((pt.y - min_y) / resolution);
    if (i >= 0 && i < width && j >= 0 && j < height)
      map.data[j * width + i] = 100;
  }

  // Mark rest as free
  for (auto& v : map.data)
    if (v != 100) v = 0;

  int occ = 0;
  for (auto v : map.data) if (v == 100) occ++;
  ROS_INFO("Map: %dx%d, %d occupied, %zu free cells", width, height, occ, map.data.size() - occ);

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 1, true);
  pub.publish(map);
  ROS_INFO("Published /projected_map");

  // Save PGM + YAML
  std::string base_name = file_path;
  size_t last_slash = base_name.find_last_of("/\\");
  if (last_slash != std::string::npos)
    base_name = base_name.substr(last_slash + 1);
  if (last_slash != std::string::npos) {
    size_t last_dot = base_name.find_last_of(".");
    if (last_dot != std::string::npos)
      base_name = base_name.substr(0, last_dot);
  }

  std::string out_dir = output_dir.empty() ?
    file_path.substr(0, file_path.find_last_of("/\\") + 1) : output_dir;
  if (!out_dir.empty() && out_dir.back() != '/') out_dir += '/';

  std::string pgm_path = out_dir + base_name + ".pgm";
  std::string yaml_path = out_dir + base_name + ".yaml";

  std::ofstream pgm(pgm_path, std::ios::binary);
  pgm << "P5\n" << width << " " << height << "\n255\n";
  for (unsigned int j = map.info.height; j > 0; j--) {
    for (unsigned int i = 0; i < map.info.width; i++) {
      int8_t v = map.data[(j - 1) * width + i];
      unsigned char pixel = (v == 100 ? 0 : 254);
      pgm.write(reinterpret_cast<const char*>(&pixel), 1);
    }
  }
  pgm.close();

  std::ofstream yaml(yaml_path);
  yaml << "image: " << base_name << ".pgm\n";
  yaml << "resolution: " << resolution << "\n";
  yaml << "origin: [" << map.info.origin.position.x << ", "
       << map.info.origin.position.y << ", 0.0]\n";
  yaml << "negate: 0\n";
  yaml << "occupied_thresh: 0.65\n";
  yaml << "free_thresh: 0.196\n";
  yaml.close();

  ROS_INFO("Saved %s and %s", pgm_path.c_str(), yaml_path.c_str());

  // Give time for the publisher to send the message, then exit
  ros::Duration(1.0).sleep();
  ros::shutdown();
  return 0;
}
