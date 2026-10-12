#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"

namespace yhs
{

class CmdVelToCtrlCmdBridge : public rclcpp::Node
{
public:
  CmdVelToCtrlCmdBridge()
  : Node("cmd_vel_to_ctrl_cmd_bridge")
  {
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto ctrl_cmd_topic = declare_parameter<std::string>("ctrl_cmd_topic", "/ctrl_cmd");
    gear_ = declare_parameter<int64_t>("gear", 6);
    auto_gear_ = declare_parameter<bool>("auto_gear", true);
    lateral_threshold_ = declare_parameter<double>("lateral_threshold", 0.02);
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.2);

    if (gear_ != 6 && gear_ != 8) {
      throw std::invalid_argument("gear must be 6 (4T4D) or 8 (lateral mode)");
    }
    if (publish_rate_ < 50.0) {
      throw std::invalid_argument("publish_rate must be at least 50 Hz");
    }
    if (cmd_timeout_ <= 0.0) {
      throw std::invalid_argument("cmd_timeout must be greater than zero");
    }
    if (lateral_threshold_ < 0.0) {
      throw std::invalid_argument("lateral_threshold must be non-negative");
    }

    ctrl_cmd_publisher_ =
      create_publisher<yhs_can_interfaces::msg::CtrlCmd>(ctrl_cmd_topic, 10);
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      10,
      std::bind(&CmdVelToCtrlCmdBridge::cmd_vel_callback, this, std::placeholders::_1));

    const std::chrono::duration<double> period(1.0 / publish_rate_);
    publish_timer_ = create_wall_timer(
      period, std::bind(&CmdVelToCtrlCmdBridge::publish_ctrl_cmd, this));

    RCLCPP_INFO(
      get_logger(),
      "Bridging %s to %s at %.1f Hz "
      "(auto_gear=%s, fixed_gear=%ld, lateral_threshold=%.3f, timeout=%.3f s)",
      cmd_vel_topic.c_str(), ctrl_cmd_topic.c_str(), publish_rate_,
      auto_gear_ ? "true" : "false",
      static_cast<long>(gear_), lateral_threshold_, cmd_timeout_);

    if (auto_gear_) {
      RCLCPP_INFO(
        get_logger(),
        "  auto_gear 已启用：|linear.y| > %.3f → gear 8（横移），否则 → gear 6（旋转）",
        lateral_threshold_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "  auto_gear 已禁用：始终使用 gear %ld",
        static_cast<long>(gear_));
    }
  }

private:
  static constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  static constexpr int64_t kGear4T4D = 6;
  static constexpr int64_t kGearLateral = 8;

  static float finite_protocol_value(double value, double minimum, double maximum)
  {
    if (!std::isfinite(value)) {
      return 0.0F;
    }
    return static_cast<float>(std::clamp(value, minimum, maximum));
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_cmd_vel_ = *msg;
    last_command_time_ = std::chrono::steady_clock::now();
    command_received_ = true;
  }

  /// 根据 cmd_vel 内容决定使用哪个档位
  int64_t select_gear(const geometry_msgs::msg::Twist & cmd_vel) const
  {
    if (!auto_gear_) {
      return gear_;
    }

    const double lateral_abs = std::abs(cmd_vel.linear.y);
    const double rotation_abs = std::abs(cmd_vel.angular.z);

    // 横移优先：
    //   - 有横向速度需求（|vy| > threshold）→ gear 8
    //   - 否则 → gear 6（即使有旋转需求）
    //
    // 原因：FW-mid 的 gear 6 和 gear 8 互斥，不能同时做旋转和横移。
    //       如果你希望"旋转优先"，把这里的判断反过来即可。
    if (lateral_abs > lateral_threshold_) {
      return kGearLateral;
    }

    // 纯旋转或纯前进 → gear 6
    // rotation_abs 只用于调试，不影响选择
    (void)rotation_abs;
    return kGear4T4D;
  }

  void publish_ctrl_cmd()
  {
    geometry_msgs::msg::Twist cmd_vel;
    bool command_is_fresh = false;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_received_) {
        const double command_age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_command_time_).count();
        command_is_fresh = command_age <= cmd_timeout_;
        if (command_is_fresh) {
          cmd_vel = latest_cmd_vel_;
        }
      }
    }

    yhs_can_interfaces::msg::CtrlCmd ctrl_cmd;

    // ---------- 动态选择档位 ----------
    int64_t active_gear = kGear4T4D;
    if (command_is_fresh) {
      active_gear = select_gear(cmd_vel);
    } else {
      // 命令超时：默认 gear 6（安全），速度全零
      active_gear = kGear4T4D;
    }

    ctrl_cmd.ctrl_cmd_gear = static_cast<uint8_t>(active_gear);

    if (command_is_fresh) {
      ctrl_cmd.ctrl_cmd_x_linear =
        finite_protocol_value(cmd_vel.linear.x, -32.768, 32.767);

      if (active_gear == kGear4T4D) {
        // 4T4D 档：X 线速度 + Z 角速度
        ctrl_cmd.ctrl_cmd_z_angular = finite_protocol_value(
          cmd_vel.angular.z * kRadiansToDegrees, -327.68, 327.67);
        ctrl_cmd.ctrl_cmd_y_linear = 0.0F;
      } else {
        // 横移档：X + Y 线速度，无旋转
        ctrl_cmd.ctrl_cmd_z_angular = 0.0F;
        ctrl_cmd.ctrl_cmd_y_linear =
          finite_protocol_value(cmd_vel.linear.y, -32.768, 32.767);
      }
    } else {
      // 命令过期：显式清零
      ctrl_cmd.ctrl_cmd_x_linear = 0.0F;
      ctrl_cmd.ctrl_cmd_z_angular = 0.0F;
      ctrl_cmd.ctrl_cmd_y_linear = 0.0F;
    }

    ctrl_cmd_publisher_->publish(ctrl_cmd);
  }

  int64_t gear_;                  // 固定档位（auto_gear=false 时使用）
  bool auto_gear_;                // true=根据 cmd_vel 自动选档
  double lateral_threshold_;      // 判定"有横移需求"的 |vy| 阈值
  double publish_rate_;
  double cmd_timeout_;

  std::mutex command_mutex_;
  geometry_msgs::msg::Twist latest_cmd_vel_;
  std::chrono::steady_clock::time_point last_command_time_;
  bool command_received_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace yhs

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<yhs::CmdVelToCtrlCmdBridge>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("cmd_vel_to_ctrl_cmd_bridge"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}