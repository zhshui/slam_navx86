#!/bin/bash
# SLAM start script for faster-lio mapping_c16 (MID360)

export DISPLAY=${DISPLAY:-:0}

source /opt/ros/noetic/setup.bash
source /home/unitree/go2_nav/lite_cog/drivers/mid360_ws/devel/setup.bash
source /home/unitree/go2_nav/lite_cog/slam/devel/setup.bash

roslaunch faster_lio mapping_c16.launch "$@"
