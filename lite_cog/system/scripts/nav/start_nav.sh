#!/bin/bash
LOG_DIR="/home/unitree/go2_nav/lite_cog/system/scripts/nav/logs"
mkdir -p "$LOG_DIR"

pkill -f "[r]oslaunch hdl_localization local_rslidar_imu.launch" 2>/dev/null
sleep 1
pkill -f "[v]elodyne_nodelet_manager" 2>/dev/null
pkill -f "[m]ove_base" 2>/dev/null
pkill -f "[l]idar_range_filter.py" 2>/dev/null
sleep 1

bash -c "source /opt/ros/noetic/setup.bash && source /home/unitree/go2_nav/lite_cog/nav/devel/setup.bash && roslaunch hdl_localization local_rslidar_imu.launch" > "$LOG_DIR/nav.log" 2>&1 &
PID1=$!

echo "[nav] started: hdl_localization=$PID1"
wait
