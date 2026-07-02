#!/bin/bash
# MAP_SWITCH_HOOK: web端切换地图时调用，同时更换 2D 栅格地图(PGM+YAML) 和 点云(PCD)
# 用法: switch_map_hook.sh <yaml_rel_path> <pcd_rel_path> <map_name>

set -e

YAML_REL="$1"
PCD_REL="$2"
MAP_NAME="${3:-unknown}"

GATEWAY_DIR="/home/robot/go2_nav/ros_web_gui_app/gateway"
MAP_DIR="/home/robot/go2_nav/lite_cog/system/map"
ACTIVE_DIR="/home/robot/go2_nav/lite_cog/system/map/active"
MAPS_DIR="$GATEWAY_DIR/data/maps"
LOG_TAG="[switch_map]"

echo "$LOG_TAG Activating: $MAP_NAME"
echo "$LOG_TAG   yaml: $YAML_REL"
echo "$LOG_TAG   pcd:  $PCD_REL"

mkdir -p "$ACTIVE_DIR" "$MAPS_DIR"

# ---- 解析路径 (支持绝对路径和相对路径) ----
if [[ "$YAML_REL" == /* ]]; then
    YAML_ABS="$YAML_REL"
else
    YAML_ABS="$GATEWAY_DIR/$YAML_REL"
fi
PGM_ABS="${YAML_ABS%.yaml}.pgm"

if [[ -n "$PCD_REL" ]]; then
    if [[ "$PCD_REL" == /* ]]; then
        PCD_ABS="$PCD_REL"
    else
        PCD_ABS="$GATEWAY_DIR/$PCD_REL"
    fi
else
    # 尝试从地图文件夹按名称找 PCD
    PCD_ABS="$MAP_DIR/${MAP_NAME}/${MAP_NAME}.pcd"
    if [ ! -f "$PCD_ABS" ]; then
        # 回退: 旧版扁平目录
        PCD_ABS="$MAP_DIR/${MAP_NAME}.pcd"
    fi
fi

echo "$LOG_TAG   yaml_abs: $YAML_ABS"
echo "$LOG_TAG   pgm_abs:  $PGM_ABS"
echo "$LOG_TAG   pcd_abs:  $PCD_ABS"

# ---- 1. 切换 2D 栅格地图 ----
if [ -f "$YAML_ABS" ]; then
    cp "$YAML_ABS" "$ACTIVE_DIR/current.yaml"
    sed -i 's|^image:.*|image: current.pgm|' "$ACTIVE_DIR/current.yaml"
    echo "$LOG_TAG   YAML -> $ACTIVE_DIR/current.yaml"
else
    echo "$LOG_TAG   WARNING: YAML not found: $YAML_ABS"
fi

if [ -f "$PGM_ABS" ]; then
    cp "$PGM_ABS" "$ACTIVE_DIR/current.pgm"
    echo "$LOG_TAG   PGM -> $ACTIVE_DIR/current.pgm"
else
    echo "$LOG_TAG   WARNING: PGM not found: $PGM_ABS"
fi

# ---- 2. 切换 PCD 点云 ----
if [ -n "$PCD_ABS" ] && [ -f "$PCD_ABS" ]; then
    cp "$PCD_ABS" "$ACTIVE_DIR/current.pcd"
    echo "$LOG_TAG   PCD -> $ACTIVE_DIR/current.pcd"
else
    echo "$LOG_TAG   WARNING: PCD not found: $PCD_ABS"
fi

# ---- 3. 同步到 system/map/current.* (供 hdl_localization 和 launch 文件读取) ----
if [ -f "$YAML_ABS" ]; then
    cp "$YAML_ABS" "$MAP_DIR/current.yaml"
    sed -i 's|^image:.*|image: current.pgm|' "$MAP_DIR/current.yaml"
    echo "$LOG_TAG   YAML -> $MAP_DIR/current.yaml"
fi
if [ -f "$PGM_ABS" ]; then
    cp "$PGM_ABS" "$MAP_DIR/current.pgm"
    echo "$LOG_TAG   PGM -> $MAP_DIR/current.pgm"
fi
if [ -n "$PCD_ABS" ] && [ -f "$PCD_ABS" ]; then
    cp "$PCD_ABS" "$MAP_DIR/current.pcd"
    echo "$LOG_TAG   PCD -> $MAP_DIR/current.pcd"
fi

# ---- 4. 写 active 记录 ----
echo "NAME=$MAP_NAME" > "$ACTIVE_DIR/current.txt"

# ---- 5. 通知 map_server 重载 2D 栅格地图 ----
if rosnode list 2>/dev/null | grep -qi "/MapServer"; then
    rosservice call /change_map "$ACTIVE_DIR/current.yaml" 2>/dev/null \
        && echo "$LOG_TAG   map_server reloaded ($ACTIVE_DIR/current.yaml)" \
        || echo "$LOG_TAG   map_server reload failed (may be OK)"
fi

# ---- 6. 提示：PCD切换需重启定位 ----
echo "$LOG_TAG 注意: PCD点云切换后需重启导航(hdl_localization)才能生效"

echo "$LOG_TAG Done: $MAP_NAME activated"
