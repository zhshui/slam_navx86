#!/bin/bash
# ============================================
# go2_nav Web 自启动脚本
# 启动顺序: roscore → rosbridge → gateway → vite
# ============================================
LOG_DIR="$HOME/go2_nav/logs"
LOCK_FILE="$LOG_DIR/start_web.lock"
mkdir -p "$LOG_DIR"

echo "[startup] $(date) ====== 启动 go2_nav web 服务 ======"

# 0. 防并发：有旧实例则先杀掉再启动
PID_FILE="$LOG_DIR/start_web.pid"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "[startup] 检测到旧实例，强制终止后重新启动 ..."
    # 优先通过 PID 文件找到旧进程（比 fuser 更可靠）
    if [ -f "$PID_FILE" ]; then
        OLD_PID=$(cat "$PID_FILE" 2>/dev/null)
        if [ -n "$OLD_PID" ] && [ "$OLD_PID" != "$$" ] && kill -0 "$OLD_PID" 2>/dev/null; then
            OLD_PGID=$(ps -o pgid= -p "$OLD_PID" 2>/dev/null | tr -d ' ')
            if [ -n "$OLD_PGID" ]; then
                echo "[startup]   终止进程组 PGID=$OLD_PGID (来自 PID 文件)"
                kill -9 -"$OLD_PGID" 2>/dev/null || true
            fi
            kill -9 "$OLD_PID" 2>/dev/null || true
        fi
    fi
    # 备用：通过 fuser 找到所有持有锁文件的进程（修复 tr -d ' ' 黏连多 PID 的 bug）
    for OLD_PID in $(fuser "$LOCK_FILE" 2>/dev/null); do
        [ "$OLD_PID" = "$$" ] && continue
        OLD_PGID=$(ps -o pgid= -p "$OLD_PID" 2>/dev/null | tr -d ' ')
        if [ -n "$OLD_PGID" ]; then
            echo "[startup]   终止进程组 PGID=$OLD_PGID (pid=$OLD_PID, 持有锁文件)"
            kill -9 -"$OLD_PGID" 2>/dev/null || true
        fi
        kill -9 "$OLD_PID" 2>/dev/null || true
    done
    sleep 1
    # 清理端口残留
    for port in 11311 9091 8080 5173; do
        for PID in $(ss -tlnp | grep ":${port} " | grep -oP 'pid=\K\d+' 2>/dev/null); do
            echo "[startup]   清理端口 ${port} (pid=${PID})"
            kill -9 "$PID" 2>/dev/null || true
        done
    done
    sleep 1
    # 重新获取锁
    exec 9>"$LOCK_FILE"
    flock -w 5 9 || { echo "[startup] 无法获取锁，退出"; exit 1; }
fi
# 写入当前 PID 供后续检测用，退出时自动清理
echo $$ > "$PID_FILE"
trap "rm -f '$PID_FILE'" EXIT

# 0.1 彻底清理所有相关进程（比端口清理更可靠）
echo "[startup] 0/4 清理残留进程 ..."
# 清理之前由本脚本启动的后台进程（roscore/rosbridge/gateway/vite + roslaunch）
for name in rosmaster rosbridge rosbridge_websocket roslaunch; do
    pkill -f "$name" 2>/dev/null || true
done
# 清理占用的端口（npm/node 进程）
for port in 11311 9091 8080 5173; do
    for PID in $(ss -tlnp | grep ":${port} " | grep -oP 'pid=\K\d+' 2>/dev/null); do
        echo "[startup]   清理端口 ${port} (pid=${PID})"
        kill -9 "$PID" 2>/dev/null || true
    done
done
sleep 2

# 0.2 统一 Node 版本为 v22（better-sqlite3 按 NODE_MODULE_VERSION 127 编译，
#      与 Node 26 不兼容会导致 gateway 崩溃）
echo "[startup] 0/5 切换 Node 版本 ..."
if [ -s "$HOME/.nvm/nvm.sh" ]; then
    export NVM_DIR="$HOME/.nvm"
    \. "$HOME/.nvm/nvm.sh"
    nvm use 22 >/dev/null 2>&1 || nvm use default >/dev/null 2>&1
fi
NODE_VER=$(node -v 2>/dev/null || echo "unknown")
echo "[startup]   使用 Node $NODE_VER"

# 1. roscore
echo "[startup] 1/5 启动 roscore ..."
source /opt/ros/noetic/setup.bash
pkill -f rosmaster 2>/dev/null
sleep 1
( exec 9>&- ; roscore ) &>"$LOG_DIR/roscore.log" &
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

# 2. rosbridge
echo "[startup] 2/5 启动 rosbridge ..."
( exec 9>&- ; roslaunch rosbridge_server rosbridge_websocket.launch port:=9091 ) &>"$LOG_DIR/rosbridge.log" &
sleep 2

for i in $(seq 1 15); do
    if ss -tlnp | grep -q 9091; then
        echo "[startup]   rosbridge OK (0.0.0.0:9091)"
        break
    fi
    sleep 2
done
if ! ss -tlnp | grep -q 9091; then
    echo "[startup]   rosbridge FAILED!"
    exit 1
fi

# 3. gateway (tsx without watch — avoids EADDRINUSE on restart)
echo "[startup] 3/5 启动 gateway ..."
cd /home/unitree/go2_nav/ros_web_gui_app/gateway
( exec 9>&- ; ROS_MODE=rosbridge npx tsx src/index.ts ) &>"$LOG_DIR/gateway.log" &
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

# 4. vite
echo "[startup] 4/5 启动 vite ..."
cd /home/unitree/go2_nav/ros_web_gui_app
( exec 9>&- ; npm run dev ) &>"$LOG_DIR/vite.log" &
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

# 获取 WiFi IP（优先无线网卡，否则取第一个 IP）
WIFI_IP=$(ip -o -4 addr show | grep -E 'wl[apn]' | awk '{print $4}' | cut -d/ -f1 | head -1)
if [ -z "$WIFI_IP" ]; then
    WIFI_IP=$(hostname -I | awk '{print $1}')
fi

echo "[startup] $(date) ====== 全部启动完成 ====="
echo "[startup] 其他设备访问: http://${WIFI_IP}:8080"

# 保持前台运行，等待所有后台子进程（systemd Type=simple 用）
# 任何子进程退出 → 脚本也退出 → systemd 触发 Restart
wait
