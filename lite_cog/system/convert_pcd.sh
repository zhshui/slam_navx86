#!/bin/bash
PCD_FILE="$1"
if [ -z "$PCD_FILE" ] || [ ! -f "$PCD_FILE" ]; then
    echo "[convert_pcd] File not found: $PCD_FILE"
    exit 1
fi

source /opt/ros/noetic/setup.bash
source /home/unitree/go2_nav/lite_cog/slam/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11312
roscore -p 11312 &
ROSCORE_PID=$!
sleep 2

# Step 1: Level the PCD (correct ground tilt via RANSAC plane detection)
# Overwrites the original with the leveled version — tilted PCD is not kept.
LEVEL_PCD_BIN="/home/unitree/go2_nav/lite_cog/system/tools/level_pcd"
LEVELED_TMP="${PCD_FILE%.pcd}_tmp.pcd"

if [ -x "$LEVEL_PCD_BIN" ]; then
    echo "[convert_pcd] Leveling: $PCD_FILE"
    $LEVEL_PCD_BIN "$PCD_FILE" "$LEVELED_TMP" \
        --max-iterations 2000 --distance-threshold 0.03 --voxel-leaf 0.05 \
    && mv "$LEVELED_TMP" "$PCD_FILE" \
    && echo "[convert_pcd] Leveled PCD saved to: $PCD_FILE"
else
    echo "[convert_pcd] WARNING: level_pcd not found, using original PCD"
fi

echo "[convert_pcd] Converting: $PCD_FILE"
roslaunch pcd2grid direct_pcd2grid.launch file_path:="$PCD_FILE"

kill $ROSCORE_PID 2>/dev/null
wait $ROSCORE_PID 2>/dev/null

# Sync new map to nav_web
echo "[convert_pcd] Syncing maps to web..."
bash /home/unitree/go2_nav/lite_cog/system/scripts/slam/sync_maps_to_web.sh

# Clean up flat intermediate PGM/YAML — final archive is done by map_stop_hook.sh
PGM_FILE="${PCD_FILE%.pcd}.pgm"
YAML_FILE="${PCD_FILE%.pcd}.yaml"
rm -f "$PGM_FILE" "$YAML_FILE"
echo "[convert_pcd] Cleaned up intermediate: $(basename "$PGM_FILE") $(basename "$YAML_FILE")"

echo "[convert_pcd] Done"
