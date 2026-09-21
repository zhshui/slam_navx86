#!/bin/bash
# SLAM start script for SC-FAST-LIO mapping_mid360

source /opt/ros/noetic/setup.bash
source /home/unitree/go2_nav/SC-FAST-LIO-main/devel/setup.bash

# 清理上次建图遗留的未合并分段 (map_stop_hook 合并失败时会保留分段,
# 不清的话旧分段会混入本次新地图)
rm -f /home/unitree/go2_nav/lite_cog/system/map/scans_seg_*.pcd

roslaunch fast_lio mapping_mid360.launch "$@"
