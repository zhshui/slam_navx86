#!/bin/bash
# Kill any existing lidar processes first
pkill -f "[l]ivox_lidar\|[l]ivox_ros_driver\|msg_MID360" 2>/dev/null
sleep 1

LOG=/home/robot/go2_nav/lite_cog/system/scripts/lidar/logs
mkdir -p "$LOG"

bash -c "source /opt/ros/noetic/setup.bash && source /home/robot/go2_nav/lite_cog/drivers/mid360_ws/devel/setup.bash && roslaunch livox_ros_driver2 msg_MID360.launch" > "$LOG/lidar.log" 2>&1 &
echo "[lidar] started PID=$!"
wait
