cmake_minimum_required(VERSION 2.8.3)
project(pcd2grid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  pcl_ros
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pcd2grid_node src/pcd2grid_node.cpp)
target_link_libraries(pcd2grid_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS pcd2grid_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ./launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ./rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
