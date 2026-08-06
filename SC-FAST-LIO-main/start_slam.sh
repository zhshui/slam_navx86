#!/bin/bash
# SLAM start script for SC-FAST-LIO mapping_mid360

source /opt/ros/noetic/setup.bash
source $HOME/go2_nav/SC-FAST-LIO-main/devel/setup.bash

roslaunch fast_lio mapping_mid360.launch "$@"
