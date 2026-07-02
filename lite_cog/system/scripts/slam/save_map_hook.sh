#!/bin/bash
# MAP_SAVE_HOOK: 保存当前 /map 话题到 system/map/<名称>/ 子文件夹
# 用法: save_map_hook.sh <name>

NAME="$1"
MAP_DIR="/home/robot/go2_nav/lite_cog/system/map"
MAP_FOLDER="$MAP_DIR/$NAME"

source /opt/ros/noetic/setup.bash 2>/dev/null
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

mkdir -p "$MAP_FOLDER"

echo "[save_map] Saving /map to $MAP_FOLDER/ (master=$ROS_MASTER_URI)"

timeout 15 rosrun map_server map_saver -f "$MAP_FOLDER/${NAME}" map:=/map 2>/dev/null

if [ -f "$MAP_FOLDER/${NAME}.yaml" ]; then
    sed -i "s|^image:.*|image: ${NAME}.pgm|" "$MAP_FOLDER/${NAME}.yaml"
    echo "[save_map] Saved: $MAP_FOLDER/${NAME}.yaml + ${NAME}.pgm"
else
    echo "[save_map] WARNING: save may have failed"
fi
