#!/bin/bash
# mq_adapter 自启动脚本 — 在 gnome-terminal 中运行
# 由 ~/.config/autostart/mq_adapter.desktop 调用

echo "=========================================="
echo "  MQ Adapter — 导航桥接启动中..."
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "=========================================="
echo ""

# ==== ROS 环境 ====
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
    echo "✅ ROS noetic 环境已加载"
else
    echo "❌ 未找到 /opt/ros/noetic/setup.bash"
    read -p '按回车关闭...'
    exit 1
fi

# ==== ROS master ====
# 等待 roscore 就绪（如果尚未启动）
echo "⏳ 等待 ROS Master (127.0.0.1:11311)..."
for i in $(seq 1 30); do
    if rostopic list &>/dev/null; then
        echo "✅ ROS Master 已就绪"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "❌ ROS Master 在 30 秒内未就绪，尝试启动 roscore..."
        roscore &
        sleep 5
    fi
    sleep 1
done

# ==== 启动 mq_adapter ====
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo "🚀 启动 mq_adapter.py ..."
echo "   MQ_TYPE = ${MQ_TYPE:-mqtt}"
echo "   MQ_HOST = ${MQ_HOST:-49.233.183.203}"
echo ""

python3 mq_adapter.py

# 脚本退出后保持终端打开，方便查看错误
echo ""
echo "=========================================="
echo "  mq_adapter.py 已退出 (exit code: $?)"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "=========================================="
read -p '按回车关闭此窗口...'
