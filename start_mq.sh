#!/bin/bash
# ============================================
# go2_nav MQ Adapter 启动脚本
# 等待 roscore 就绪后启动 MQ 适配器
# ============================================
LOG_DIR="$HOME/go2_nav/logs"
mkdir -p "$LOG_DIR"

echo "[mq-startup] $(date) ====== 启动 MQ Adapter ======"

# 0. 设置 ROS 环境
source /opt/ros/noetic/setup.bash

# 1. 等待 roscore 就绪 (最多等 60 秒)
echo "[mq-startup] 等待 roscore 就绪 (127.0.0.1:11311) ..."
for i in $(seq 1 30); do
    if ss -tlnp | grep -q 11311; then
        echo "[mq-startup] roscore 已就绪"
        break
    fi
    sleep 2
done
if ! ss -tlnp | grep -q 11311; then
    echo "[mq-startup] ⚠ roscore 未检测到，尝试继续启动 (mq_adapter 会自动重连)"
fi

# 2. 启动 MQ Adapter
echo "[mq-startup] 启动 mq_adapter ..."
exec python3 $HOME/go2_nav/lite_cog/system/scripts/mq/mq_adapter.py
