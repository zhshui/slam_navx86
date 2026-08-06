cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS scripts/path_smoother.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# install(DIRECTORY map/  ...) — directory not present, skipped
# install(DIRECTORY rviz/ ...) — directory not present, skipped
