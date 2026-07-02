#!/bin/bash
# ============================================
# Restart move_base only (keep hdl_localization alive)
# Called by gateway when STVL params change
# ============================================
LOG_DIR="/home/robot/go2_nav/lite_cog/system/scripts/nav/logs"
mkdir -p "$LOG_DIR"

source /opt/ros/noetic/setup.bash
source /home/robot/go2_nav/lite_cog/nav/devel/setup.bash

echo "[restart_mb] $(date) Killing old move_base ..."
rosnode kill /move_base 2>/dev/null
pkill -f "[m]ove_base" 2>/dev/null
sleep 2

echo "[restart_mb] $(date) Launching new move_base ..."
roslaunch navigation navigation.launch >> "$LOG_DIR/move_base.log" 2>&1 &
PID=$!
echo "[restart_mb] $(date) move_base started (pid=$PID)"

# Wait for move_base to appear (wait_for_deps.sh can take up to 30s)
echo "[restart_mb] $(date) Waiting for /move_base node..."
for i in $(seq 1 40); do
    if rosnode list 2>/dev/null | grep -q /move_base; then
        echo "[restart_mb] $(date) move_base restart OK (${i}s)"
        exit 0
    fi
    sleep 1
done
echo "[restart_mb] $(date) move_base restart FAILED (timeout)"
exit 1
