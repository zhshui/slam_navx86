/**********************************************************************
 * 改动说明：
 * 1. 引入了 ros/ros.h 和 geometry_msgs/Twist.h
 * 2. 删除了固定的 TEST_MODE 循环逻辑，改为由回调函数驱动
 * 3. 增加了对 cmd_vel 话题的订阅
 * 4. 添加 EMA 低通滤波平滑速度命令，避免突变
 * 5. 添加角速度缩放，提升转向速度
 ***********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

// EMA (指数移动平均) 低通滤波器
// filtered = alpha * raw + (1-alpha) * filtered_prev
class EmaFilter
{
public:
  EmaFilter() : initialized_(false), value_(0.0) {}

  void setAlpha(double alpha) { alpha_ = std::max(0.0, std::min(1.0, alpha)); }

  double filter(double raw)
  {
    if (!initialized_)
    {
      value_ = raw;
      initialized_ = true;
    }
    else
    {
      value_ = alpha_ * raw + (1.0 - alpha_) * value_;
    }
    return value_;
  }

  void reset()
  {
    initialized_ = false;
    value_ = 0.0;
  }

private:
  double alpha_;
  double value_;
  bool initialized_;
};

class Custom
{
public:
  Custom()
  {
    // 初始化 Unitree SDK 客户端
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    // 订阅机器人内部 DDS 状态（可选，用于获取反馈）
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  }

  void initFilters(ros::NodeHandle& nh)
  {
    // 从 ROS param 读取滤波参数，可运行时动态调参
    nh.param<double>("smoothing_alpha_linear", alpha_linear_, 0.3);
    nh.param<double>("smoothing_alpha_angular", alpha_angular_, 0.3);
    nh.param<double>("angular_scale", angular_scale_, 1.5);

    filter_vx_.setAlpha(alpha_linear_);
    filter_vy_.setAlpha(alpha_linear_);
    filter_vyaw_.setAlpha(alpha_angular_);

    ROS_INFO("Velocity filter: alpha_linear=%.2f, alpha_angular=%.2f, angular_scale=%.2f",
             alpha_linear_, alpha_angular_, angular_scale_);
  }

  // ROS cmd_vel 回调函数
  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    // 重新读取参数，支持运行时动态调整（rosparam set）
    ros::NodeHandle nh("~");
    double alpha_lin, alpha_ang, ang_scale;
    nh.param<double>("smoothing_alpha_linear", alpha_lin, alpha_linear_);
    nh.param<double>("smoothing_alpha_angular", alpha_ang, alpha_angular_);
    nh.param<double>("angular_scale", ang_scale, angular_scale_);

    // 如果参数变化，更新滤波器
    if (alpha_lin != alpha_linear_) {
      alpha_linear_ = alpha_lin;
      filter_vx_.setAlpha(alpha_linear_);
      filter_vy_.setAlpha(alpha_linear_);
    }
    if (alpha_ang != alpha_angular_) {
      alpha_angular_ = alpha_ang;
      filter_vyaw_.setAlpha(alpha_angular_);
    }
    angular_scale_ = ang_scale;

    double raw_vx = msg->linear.x;
    double raw_vy = msg->linear.y;
    double raw_vyaw = msg->angular.z;

    // EMA 低通滤波
    double vx = filter_vx_.filter(raw_vx);
    double vy = filter_vy_.filter(raw_vy);
    double vyaw = filter_vyaw_.filter(raw_vyaw);

    // 角速度缩放（加大转向速度）
    vyaw *= angular_scale_;

    // 调用 SDK 的 Move 接口
    sport_client.Move(vx, vy, vyaw);

    ROS_INFO("CmdVel raw(vx:%.2f vy:%.2f vyaw:%.2f) -> filtered(vx:%.2f vy:%.2f vyaw:%.2f)",
             raw_vx, raw_vy, raw_vyaw, vx, vy, vyaw);
  }

  // ROS sport_cmd 回调: 接收 String 指令 (stand_up / sit / damp / stand_down / recovery_stand / stop_move / balance_stand / classic_walk)
  void SportCmdCallback(const std_msgs::String::ConstPtr& msg)
  {
    std::string cmd = msg->data;
    ROS_INFO("Received sport_cmd: [%s]", cmd.c_str());

    if (cmd == "stand_up" || cmd == "stand") {
      sport_client.StandUp();
      ROS_INFO(" -> StandUp");
    } else if (cmd == "sit") {
      sport_client.Sit();
      ROS_INFO(" -> Sit");
    } else if (cmd == "damp") {
      sport_client.Damp();
      ROS_INFO(" -> Damp");
    } else if (cmd == "stand_down") {
      sport_client.StandDown();
      ROS_INFO(" -> StandDown");
    } else if (cmd == "recovery_stand") {
      sport_client.RecoveryStand();
      ROS_INFO(" -> RecoveryStand");
    } else if (cmd == "balance_stand") {
      sport_client.BalanceStand();
      ROS_INFO(" -> BalanceStand");
    } else if (cmd == "stop_move") {
      sport_client.StopMove();
      // 停止时重置滤波器，避免历史值影响下次启动
      filter_vx_.reset();
      filter_vy_.reset();
      filter_vyaw_.reset();
      ROS_INFO(" -> StopMove (filters reset)");
    } else if (cmd == "rise_sit") {
      sport_client.RiseSit();
      ROS_INFO(" -> RiseSit");
    } else if (cmd == "hello") {
      sport_client.Hello();
      ROS_INFO(" -> Hello");
    } else if (cmd == "classic_walk") {
      sport_client.ClassicWalk();
      ROS_INFO(" -> ClassicWalk");
    } else {
      ROS_WARN("Unknown sport_cmd: [%s]", cmd.c_str());
    }
  }

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
  }

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  // 速度滤波器
  EmaFilter filter_vx_;
  EmaFilter filter_vy_;
  EmaFilter filter_vyaw_;

  // 滤波参数（可运行时动态调整）
  double alpha_linear_;
  double alpha_angular_;
  double angular_scale_;
};

int main(int argc, char **argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "go2_cmd_vel_bridge");
  ros::NodeHandle nh;

  // 获取网卡参数 (从 launch 文件或命令行传入)
  std::string net_interface = "eth0";
  if (argc > 1) {
    net_interface = argv[1];
  }
  nh.param<std::string>("nic", net_interface, net_interface);

  // 初始化 Unitree 通道
  unitree::robot::ChannelFactory::Instance()->Init(0, net_interface);
  
  Custom custom;

  // 初始化速度滤波器（从 ROS param 读取参数）
  custom.initFilters(nh);

  // 创建 ROS 订阅者，监听 /cmd_vel 话题（速度控制）
  ros::Subscriber sub = nh.subscribe("cmd_vel", 10, &Custom::CmdVelCallback, &custom);

  // 创建 ROS 订阅者，监听 /go2/sport_cmd 话题（姿态指令）
  ros::Subscriber sport_sub = nh.subscribe("/go2/sport_cmd", 10, &Custom::SportCmdCallback, &custom);

  ROS_INFO("Go2 CMD_VEL Bridge Started. Listening to /cmd_vel and /go2/sport_cmd...");

  // 让机器站起来，准备接受运动指令
  sleep(1); 
  custom.sport_client.StandUp();
  ROS_INFO("Robot standing up...");

  // 循环等待 ROS 回调
  ros::spin();

  return 0;
}