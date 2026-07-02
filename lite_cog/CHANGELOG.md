2026-05-19: 新增

  slam/src/faster-lio/launch/mapping_c16_graph.launch

  Faster-LIO + hdl_graph_slam 联合建图 launch 文件
  ├── Prefiltering: /cloud_registered_body → /filtered_points
  (降采样+离群点去除)
  ├── Faster-LIO: 前端里程计 → /Odometry
  ├── hdl_graph_slam: 回环检测 + g2o 位姿图优化 → TF: map→camera_init +
  /map_points
  └── map2odom_publisher: 发布优化后的 TF 修正

  关键配置：
  - 里程计源：Faster-LIO /Odometry（帧 camera_init → body）
  - 点云源：/cloud_registered_body（body 帧，已去畸变）
  - 回环：FAST_GICP，搜索半径 10m，累积距离 15m 后触发
  - IMU 重力约束开启（抑制 z 漂移）
  - 关闭 GPS、关闭地面检测（室外不适用）

  修改

  system/scripts/slam/start_slam.sh — mapping_c16.launch →
  mapping_c16_graph.launch

  使用方式

  1. 安装依赖（如果你还没做）：
  sudo apt-get install -y ros-noetic-geodesy ros-noetic-nmea-msgs
  2. 编译 hdl_graph_slam：
  cd /home/shui/LY/lite_cog/slam
  catkin build hdl_graph_slam
  3. 建图：跟之前一样 start_slam.sh，回环检测在后台自动运行
  4. 保存优化后的地图：
  rosservice call /hdl_graph_slam/save_map "resolution: 0.05
  destination: '/home/shui/LY/lite_cog/system/map/lite3_optimized.pcd'"
  5. 定位：hdl_localization 加载 lite3.pcd
