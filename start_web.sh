#!/bin/bash
# ============================================
# go2_nav Web 自启动脚本
# 启动顺序: 等待WiFi → roscore → rosbridge → gateway → vite
# ============================================
LOG_DIR="/home/robot/go2_nav/logs"
mkdir -p "$LOG_DIR"

echo "[startup] $(date) ====== 启动 go2_nav web 服务 ======"

# 0. 等待 WiFi 连接（获取不到 IP 就一直等）
echo "[startup] 0/5 等待 WiFi 连接 (wlp44s0) ..."
WIFI_IP=""
while [ -z "$WIFI_IP" ]; do
    WIFI_IP=$(ip -4 addr show wlp44s0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
    if [ -n "$WIFI_IP" ]; then
        echo "[startup]   WiFi 已连接: $WIFI_IP"
        break
    fi
    echo "[startup]   WiFi 未连接，5秒后重试..."
    sleep 5
done

# 1. 清理残留端口（防止上次异常退出后端口被占用）
echo "[startup] 1/5 清理残留端口 ..."
for port in 11311 9090 8080 5173; do
    PID=$(ss -tlnp | grep ":${port} " | grep -oP 'pid=\K\d+' 2>/dev/null)
    if [ -n "$PID" ]; then
        echo "[startup]   清理端口 ${port} (pid=${PID})"
        kill -9 "$PID" 2>/dev/null
    fi
done
sleep 1

# 2. roscore
echo "[startup] 2/5 启动 roscore ..."
source /opt/ros/noetic/setup.bash
pkill -f rosmaster 2>/dev/null
sleep 1
roscore &>"$LOG_DIR/roscore.log" &
sleep 3

# 检查 roscore (11311)
for i in $(seq 1 10); do
    if ss -tlnp | grep -q 11311; then
        echo "[startup]   roscore OK"
        break
    fi
    sleep 1
done
if ! ss -tlnp | grep -q 11311; then
    echo "[startup]   roscore FAILED!"
    exit 1
fi

# 3. rosbridge
echo "[startup] 3/5 启动 rosbridge ..."
roslaunch rosbridge_server rosbridge_websocket.launch &>"$LOG_DIR/rosbridge.log" &
sleep 2

for i in $(seq 1 15); do
    if ss -tlnp | grep -q 9090; then
        echo "[startup]   rosbridge OK (0.0.0.0:9090)"
        break
    fi
    sleep 2
done
if ! ss -tlnp | grep -q 9090; then
    echo "[startup]   rosbridge FAILED!"
    exit 1
fi

# 4. gateway
echo "[startup] 4/5 启动 gateway ..."
cd /home/robot/go2_nav/ros_web_gui_app/gateway
npm run dev &>"$LOG_DIR/gateway.log" &
sleep 2

for i in $(seq 1 15); do
    if ss -tlnp | grep -q 8080; then
        echo "[startup]   gateway OK (0.0.0.0:8080)"
        break
    fi
    sleep 2
done
if ! ss -tlnp | grep -q 8080; then
    echo "[startup]   gateway FAILED!"
    exit 1
fi

# 5. vite
echo "[startup] 5/5 启动 vite ..."
cd /home/robot/go2_nav/ros_web_gui_app
npm run dev &>"$LOG_DIR/vite.log" &
sleep 2

for i in $(seq 1 15); do
    if ss -tlnp | grep -q 5173; then
        echo "[startup]   vite OK (0.0.0.0:5173)"
        break
    fi
    sleep 2
done
if ! ss -tlnp | grep -q 5173; then
    echo "[startup]   vite FAILED!"
    exit 1
fi

# WiFi IP 已在步骤 0 获取
echo "[startup] $(date) ====== 全部启动完成 ====="
echo "[startup] 其他设备访问: http://${WIFI_IP}:8080"
