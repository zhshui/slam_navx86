#!/bin/bash

set -e

echo "=== 更新 apt 源 ==="
sudo apt update

echo "=== 安装系统库依赖 ==="
sudo apt install -y \
    libsdl1.2-dev \
    libsdl-image1.2-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    cmake \
    build-essential \
    git \
    python3-pip

echo "=== 安装 ROS Noetic 官方包 ==="
sudo apt install -y \
    ros-noetic-navigation \
    ros-noetic-octomap-ros \
    ros-noetic-move-base-flex \
    ros-noetic-libg2o

echo "=== 安装源码依赖 ==="
WORKSPACE=~/catkin_ws/src

# costmap_converter
if [ ! -d "$WORKSPACE/costmap_converter" ]; then
    echo "Cloning costmap_converter..."
    cd $WORKSPACE
    git clone https://github.com/rst-tu-dortmund/costmap_converter.git
fi

# g2o 源码（可选，如果 apt 安装失败）
if [ ! -d "$WORKSPACE/g2o" ]; then
    echo "Cloning g2o from GitHub..."
    cd $WORKSPACE
    git clone https://github.com/RainerKuemmerle/g2o.git
    mkdir -p g2o/build && cd g2o/build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install
fi

echo "=== 安装完成 ==="
echo "请回到你的 catkin 工作空间根目录执行："
echo "  catkin_make 或 catkin build"

