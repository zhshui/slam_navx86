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
#include <algorithm>
#include <cmath>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include "classic_gait_initializer.h"
#include "classic_gait_lock.h"

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
      : command_active_(false),
        command_timeout_(0.5),
        classic_gait_lock_(2.0)
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
    nh.param<double>("smoothing_alpha_angular", alpha_angular_, 0.7);
    nh.param<double>("angular_scale", angular_scale_, 1.0);
    nh.param<double>("command_timeout", command_timeout_, 0.5);
    double classic_gait_lock_interval = 2.0;
    nh.param<double>("classic_gait_lock_interval", classic_gait_lock_interval, 2.0);
    classic_gait_lock_.setEnforcementInterval(classic_gait_lock_interval);

    filter_vx_.setAlpha(alpha_linear_);
    filter_vy_.setAlpha(alpha_linear_);
    filter_vyaw_.setAlpha(alpha_angular_);

    ROS_INFO("Velocity filter: alpha_linear=%.2f, alpha_angular=%.2f, angular_scale=%.2f, command_timeout=%.2fs",
             alpha_linear_, alpha_angular_, angular_scale_, command_timeout_);
  }

  void NavigationStateCallback(const std_msgs::String::ConstPtr& msg)
  {
    if (!classic_gait_lock_.updateNavigationState(msg->data))
      return;

    if (classic_gait_lock_.isLocked())
      ROS_INFO("Classic gait lock enabled for navigation state %s", msg->data.c_str());
    else
      ROS_INFO("Classic gait lock released for navigation state %s", msg->data.c_str());
  }

  void ClassicGaitLockCallback(const ros::WallTimerEvent&)
  {
    const double now = ros::SteadyTime::now().toSec();
    if (!classic_gait_lock_.isEnforcementDue(now))
      return;

    const int32_t code = sport_client.ClassicWalk(true);
    classic_gait_lock_.recordEnforcement(now);
    if (code == 0)
      ROS_INFO_THROTTLE(10.0, "Classic gait lock active");
    else
      ROS_WARN_THROTTLE(10.0, "Classic gait lock request returned code=%d", code);
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

    const bool stop_command =
        std::fabs(raw_vx) < 1e-4 &&
        std::fabs(raw_vy) < 1e-4 &&
        std::fabs(raw_vyaw) < 1e-4;
    if (stop_command)
    {
      sport_client.Move(0.0, 0.0, 0.0);
      resetMotionState();
      ROS_INFO_THROTTLE(1.0, "CmdVel zero command -> Move(0,0,0), gait preserved");
      return;
    }

    last_cmd_time_ = ros::SteadyTime::now();
    command_active_ = true;

    // EMA 低通滤波
    double vx = filter_vx_.filter(raw_vx);
    double vy = filter_vy_.filter(raw_vy);
    double vyaw = filter_vyaw_.filter(raw_vyaw);

    // 角速度缩放（加大转向速度）
    vyaw *= angular_scale_;

    // 调用 SDK 的 Move 接口
    int32_t move_code = sport_client.Move(vx, vy, vyaw);

    if (move_code != 0)
    {
      ROS_WARN("SportClient.Move returned %d for filtered(vx:%.2f vy:%.2f vyaw:%.2f)",
               move_code, vx, vy, vyaw);
    }
    ROS_INFO("CmdVel raw(vx:%.2f vy:%.2f vyaw:%.2f) -> filtered(vx:%.2f vy:%.2f vyaw:%.2f) code=%d",
             raw_vx, raw_vy, raw_vyaw, vx, vy, vyaw, move_code);
  }

  void WatchdogCallback(const ros::WallTimerEvent&)
  {
    if (!command_active_)
      return;

    const double age = (ros::SteadyTime::now() - last_cmd_time_).toSec();
    if (age <= command_timeout_)
      return;

    sport_client.Move(0.0, 0.0, 0.0);
    resetMotionState();
    ROS_WARN("CmdVel timeout after %.3fs -> Move(0,0,0), gait preserved", age);
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
      resetMotionState();
      ROS_INFO(" -> StopMove (filters reset)");
    } else if (cmd == "rise_sit") {
      sport_client.RiseSit();
      ROS_INFO(" -> RiseSit");
    } else if (cmd == "hello") {
      sport_client.Hello();
      ROS_INFO(" -> Hello");
    } else if (cmd == "classic_walk") {
      sport_client.ClassicWalk(true);
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
  bool command_active_;
  double command_timeout_;
  ros::SteadyTime last_cmd_time_;
  ClassicGaitLock classic_gait_lock_;

private:
  void resetMotionState()
  {
    filter_vx_.reset();
    filter_vy_.reset();
    filter_vyaw_.reset();
    command_active_ = false;
  }
};

int main(int argc, char **argv)
{
  // 初始化 ROS 节点
  ros::init(argc, argv, "go2_cmd_vel_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // 获取网卡参数 (从 launch 文件或命令行传入)
  std::string net_interface = "eth0";
  if (argc > 1) {
    net_interface = argv[1];
  }
  nh.param<std::string>("nic", net_interface, net_interface);
  int classic_gait_attempts = 5;
  double classic_gait_retry_delay = 1.0;
  private_nh.param<int>("classic_gait_attempts", classic_gait_attempts, 5);
  private_nh.param<double>("classic_gait_retry_delay", classic_gait_retry_delay, 1.0);

  // 初始化 Unitree 通道
  unitree::robot::ChannelFactory::Instance()->Init(0, net_interface);
  
  Custom custom;

  // 初始化速度滤波器（从 ROS param 读取参数）
  custom.initFilters(private_nh);

  // 创建 ROS 订阅者，监听 /cmd_vel 话题（速度控制）
  ros::Subscriber sub = nh.subscribe("cmd_vel", 10, &Custom::CmdVelCallback, &custom);

  // 创建 ROS 订阅者，监听 /go2/sport_cmd 话题（姿态指令）
  ros::Subscriber sport_sub = nh.subscribe("/go2/sport_cmd", 10, &Custom::SportCmdCallback, &custom);
  ros::Subscriber navigation_state_sub = nh.subscribe(
      "/move_base/TebLocalPlannerROS/navigation_state", 10,
      &Custom::NavigationStateCallback, &custom);
  ros::WallTimer watchdog_timer = nh.createWallTimer(
      ros::WallDuration(0.1), &Custom::WatchdogCallback, &custom);
  ros::WallTimer classic_gait_lock_timer = nh.createWallTimer(
      ros::WallDuration(0.1), &Custom::ClassicGaitLockCallback, &custom);

  ROS_INFO("Go2 CMD_VEL Bridge Started. Listening to /cmd_vel and /go2/sport_cmd...");

  // 初始化时选择一次经典步态；后续不再干预遥控器的步态选择。
  sleep(1);
  int32_t stand_code = custom.sport_client.StandUp();
  sleep(1);
  int gait_attempt = 0;
  int32_t classic_code = initializeClassicGait(
      [&custom, &gait_attempt]() {
        ++gait_attempt;
        const int32_t code = custom.sport_client.ClassicWalk(true);
        if (code != 0)
          ROS_WARN("ClassicWalk attempt %d failed with code=%d", gait_attempt, code);
        return code;
      },
      [classic_gait_retry_delay]() {
        ros::WallDuration(classic_gait_retry_delay).sleep();
      },
      std::max(1, classic_gait_attempts));

  if (classic_code == 0)
    ROS_INFO("Robot motion mode init: StandUp=%d ClassicWalk=0 attempts=%d",
             stand_code, gait_attempt);
  else
    ROS_ERROR("Robot motion mode init failed: StandUp=%d ClassicWalk=%d attempts=%d",
              stand_code, classic_code, gait_attempt);

  // 循环等待 ROS 回调
  ros::spin();

  return 0;
}
