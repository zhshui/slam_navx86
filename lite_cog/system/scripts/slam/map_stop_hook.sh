#!/bin/bash
# ============================================================
# MAP_STOP_HOOK — web端「停止建图」后自动调用
#   1. 停止 SLAM 进程（SIGINT 触发保存 PCD）
#   2. 等待 PCD 写入完成
#   3. 用 direct_pcd2grid.launch 将 PCD 转换为二维栅格地图
#   4. 同步结果到 nav_web 数据库
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MAP_DIR="/home/robot/go2_nav/lite_cog/system/map"
ACTIVE_DIR="$MAP_DIR/active"
GATEWAY_DIR="/home/robot/go2_nav/ros_web_gui_app/gateway"
MAPS_DIR="$GATEWAY_DIR/data/maps"
DB="$GATEWAY_DIR/data/nav_web.sqlite"
SLAM_PROC_PATTERN="${MAP_STOP_PROC_PATTERN:-fastlio_mapping}"
TAG="[map_stop_hook]"

echo "$TAG ====== 停止建图并转换栅格地图 ======"

LEVEL_PCD="/home/robot/go2_nav/lite_cog/system/tools/level_pcd"

# ---- 1. 停止 SLAM 进程 ----
echo "$TAG 1/6 停止 SLAM 进程 (pattern: $SLAM_PROC_PATTERN) ..."
PID=$(pgrep -f "$SLAM_PROC_PATTERN" | head -1 2>/dev/null || echo "")

if [ -n "$PID" ]; then
    echo "$TAG   找到进程 PID=$PID, 发送 SIGINT ..."
    kill -2 "$PID" 2>/dev/null || true

    # 等待进程退出 (最多 30s)
    for i in $(seq 1 60); do
        if ! kill -0 "$PID" 2>/dev/null; then
            echo "$TAG   进程已退出 (${i}s)"
            break
        fi
        sleep 0.5
    done

    if kill -0 "$PID" 2>/dev/null; then
        echo "$TAG   进程超时未退出, 发送 SIGKILL ..."
        kill -9 "$PID" 2>/dev/null || true
        sleep 1
    fi
else
    echo "$TAG   未找到匹配进程, SLAM 可能已停止"
fi

# ---- 2. 找到最新 PCD 文件 ----
echo "$TAG 2/6 查找 PCD 文件 ..."
PCD_FILE=""
LATEST_TS=0
for f in "$MAP_DIR"/*/*.pcd "$ACTIVE_DIR"/*.pcd; do
    [ -f "$f" ] || continue
    base=$(basename "$f")
    # 跳过衍生文件
    case "$base" in
        *leveled*|*colored*|*_z_range*) continue ;;
    esac
    ts=$(stat -c %Y "$f" 2>/dev/null || echo 0)
    if [ "$ts" -gt "$LATEST_TS" ]; then
        LATEST_TS=$ts; PCD_FILE="$f"
    fi
done

if [ -z "$PCD_FILE" ] || [ ! -f "$PCD_FILE" ]; then
    echo "$TAG   ERROR: 未找到 PCD 文件!"
    exit 1
fi
echo "$TAG   最新 PCD: $PCD_FILE ($(du -h "$PCD_FILE" | cut -f1))"

# 等待文件大小稳定 (防止 SLAM 还在写)
echo "$TAG   等待文件写入完成 ..."
for i in $(seq 1 15); do
    s1=$(stat -c %s "$PCD_FILE" 2>/dev/null || echo 0)
    sleep 1
    s2=$(stat -c %s "$PCD_FILE" 2>/dev/null || echo 0)
    if [ "$s1" -eq "$s2" ] && [ "$s1" -gt 0 ]; then
        echo "$TAG   文件大小稳定: $s1 bytes"
        break
    fi
done

# ---- 3. Z 轴水平修正 (level PCD) ----
echo "$TAG 3/6 Z 轴水平修正 (level_pcd) ..."
PCD_LEVELED="${PCD_FILE%.pcd}_leveled.pcd"
if [ -x "$LEVEL_PCD" ]; then
    echo "$TAG   执行: $LEVEL_PCD \"$PCD_FILE\" \"$PCD_LEVELED\""
    if "$LEVEL_PCD" "$PCD_FILE" "$PCD_LEVELED"; then
        echo "$TAG   Z 轴修正完成: $PCD_LEVELED"
        PCD_FOR_GRID="$PCD_LEVELED"
    else
        echo "$TAG   WARNING: level_pcd 失败, 使用原始 PCD"
        PCD_FOR_GRID="$PCD_FILE"
    fi
else
    echo "$TAG   WARNING: level_pcd 不可用, 跳过 Z 轴修正"
    PCD_FOR_GRID="$PCD_FILE"
fi

# ---- 4. 用 direct_pcd2grid 转换 PCD → PGM+YAML ----
echo "$TAG 4/6 启动 pcd2grid 转换 ..."

source /opt/ros/noetic/setup.bash
source /home/robot/go2_nav/lite_cog/slam/devel/setup.bash

# 优先用现有 ROS master, 没有则临时起一个
ROS_MASTER_PORT=11311
if rosnode list &>/dev/null 2>&1; then
    echo "$TAG   使用现有 ROS master"
else
    echo "$TAG   启动临时 roscore ..."
    ROS_MASTER_PORT=11312
    export ROS_MASTER_URI=http://localhost:$ROS_MASTER_PORT
    roscore -p $ROS_MASTER_PORT &
    ROSCORE_PID=$!
    sleep 2
fi

echo "$TAG   执行: roslaunch pcd2grid direct_pcd2grid.launch file_path:=$PCD_FOR_GRID"
roslaunch pcd2grid direct_pcd2grid.launch file_path:="$PCD_FOR_GRID"

# 如果是临时起的 roscore 则杀掉
[ -n "${ROSCORE_PID:-}" ] && kill $ROSCORE_PID 2>/dev/null && wait $ROSCORE_PID 2>/dev/null
echo "$TAG   pcd2grid 转换完成"

# ---- 5. 组织到地图文件夹 + 同步 active 目录 ----
echo "$TAG 5/6 组织到地图文件夹并同步 active 目录 ..."
mkdir -p "$ACTIVE_DIR"

# 命名: 参数传入优先, 否则自动递增 scans_N（不覆盖已有文件夹）
if [ -n "${1:-}" ]; then
    MAP_NAME="$1"
else
    N=1
    while [ -d "$MAP_DIR/scans_$N" ]; do N=$((N+1)); done
    MAP_NAME="scans_$N"
fi
echo "$TAG   地图名称: $MAP_NAME"
MAP_FOLDER="$MAP_DIR/$MAP_NAME"
mkdir -p "$MAP_FOLDER"

# pcd2grid 输出的 pgm/yaml 与转换用的 PCD 同目录同名
PCD_BASE=$(basename "$PCD_FOR_GRID" .pcd)
PGM_FILE="${PCD_FOR_GRID%.pcd}.pgm"
YAML_FILE="${PCD_FOR_GRID%.pcd}.yaml"

# 使用新名称归档到地图文件夹
mkdir -p "$MAP_FOLDER"

if [ -f "$PGM_FILE" ] && [ -f "$YAML_FILE" ]; then
    echo "$TAG   生成文件: $(basename "$PGM_FILE") / $(basename "$YAML_FILE")"

    # 1) 移动文件到地图文件夹，使用新名称
    # 使用修正后的 PCD 作为主文件
    if [ "$PCD_FOR_GRID" != "$PCD_FILE" ] && [ -f "$PCD_FOR_GRID" ]; then
        mv "$PCD_FOR_GRID" "$MAP_FOLDER/${MAP_NAME}.pcd"
        rm -f "$PCD_FILE"
        echo "$TAG   已保存: ${MAP_NAME}.pcd (Z轴修正)"
    else
        mv "$PCD_FILE"  "$MAP_FOLDER/${MAP_NAME}.pcd"
    fi
    mv "$PGM_FILE"  "$MAP_FOLDER/${MAP_NAME}.pgm"
    mv "$YAML_FILE" "$MAP_FOLDER/${MAP_NAME}.yaml"
    # 更新 YAML 中 image 为同文件夹内的 PGM
    sed -i "s|^image:.*|image: ${MAP_NAME}.pgm|" "$MAP_FOLDER/${MAP_NAME}.yaml"
    echo "$TAG   已归档到: $MAP_FOLDER/"

    # 2) 复制到 active 目录（供导航实时使用）
    cp "$MAP_FOLDER/${MAP_NAME}.pgm"  "$ACTIVE_DIR/current.pgm"
    cp "$MAP_FOLDER/${MAP_NAME}.yaml" "$ACTIVE_DIR/current.yaml"
    cp "$MAP_FOLDER/${MAP_NAME}.pcd"  "$ACTIVE_DIR/current.pcd"
    echo "NAME=$MAP_NAME" > "$ACTIVE_DIR/current.txt"
    sed -i 's|^image:.*|image: current.pgm|' "$ACTIVE_DIR/current.yaml"
    echo "$TAG   active 目录已更新"

    # 3) 复制到 system/map/current.* (供 hdl_localization 和 launch 文件读取)
    cp "$MAP_FOLDER/${MAP_NAME}.pgm"  "$MAP_DIR/current.pgm"
    cp "$MAP_FOLDER/${MAP_NAME}.yaml" "$MAP_DIR/current.yaml"
    cp "$MAP_FOLDER/${MAP_NAME}.pcd"  "$MAP_DIR/current.pcd"
    sed -i 's|^image:.*|image: current.pgm|' "$MAP_DIR/current.yaml"
    echo "$TAG   system/map/current.* 已更新"
else
    echo "$TAG   WARNING: 未找到 PGM/YAML 输出文件"
fi

# ---- 6. 注册到数据库 ----
echo "$TAG 6/6 注册到数据库 ..."

if [ -f "$MAP_FOLDER/${MAP_NAME}.pgm" ] && [ -f "$MAP_FOLDER/${MAP_NAME}.yaml" ]; then
    # 路径指向地图文件夹内的文件
    YAML_PATH="$MAP_FOLDER/${MAP_NAME}.yaml"
    PCD_PATH="$MAP_FOLDER/${MAP_NAME}.pcd"

    if command -v sqlite3 &>/dev/null; then
        EXISTS=$(sqlite3 "$DB" "SELECT id FROM maps WHERE name='$MAP_NAME';" 2>/dev/null || echo "")
        if [ -z "$EXISTS" ]; then
            ID=$(cat /proc/sys/kernel/random/uuid 2>/dev/null || echo "$(date +%s)-$RANDOM")
            sqlite3 "$DB" "INSERT INTO maps (id, name, yaml_path, pcd_path, created_at, active) VALUES ('$ID', '$MAP_NAME', '$YAML_PATH', '$PCD_PATH', datetime('now'), 0);" 2>/dev/null || true
            echo "$TAG   DB 新增: $MAP_NAME (folder: $MAP_FOLDER/)"
        else
            sqlite3 "$DB" "UPDATE maps SET yaml_path='$YAML_PATH', pcd_path='$PCD_PATH' WHERE id='$EXISTS';" 2>/dev/null || true
            echo "$TAG   DB 更新: $MAP_NAME (folder: $MAP_FOLDER/)"
        fi
    else
        echo "$TAG   WARNING: sqlite3 未安装, 跳过 DB 注册"
    fi
fi

echo "$TAG ====== 完成 ====="
