// Copyright (c) 2026, b-robotized Group

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace
{
template <typename T>
T clamp_value(T value, T minimum, T maximum)
{
  return std::max(minimum, std::min(value, maximum));
}
}  // namespace

class AlliePwmDriver : public rclcpp::Node
{
public:
  AlliePwmDriver()
  : Node("allie_pwm_driver"),
    pwm_min_(declare_parameter<int>("pwm_min", 1000)),
    pwm_neutral_(declare_parameter<int>("pwm_neutral", 1500)),
    pwm_max_(declare_parameter<int>("pwm_max", 2000)),
    wheel_separation_(declare_parameter<double>("wheel_separation", 0.51)),
    max_linear_speed_(declare_parameter<double>("max_linear_speed_mps", 2.8)),
    max_angular_speed_(declare_parameter<double>("max_angular_speed_rad_s", 4.0)),
    max_wheel_speed_(declare_parameter<double>("max_wheel_speed_mps", 2.8)),
    timeout_seconds_(declare_parameter<double>("cmd_timeout_sec", 0.5)),
    invert_left_(declare_parameter<bool>("invert_left", false)),
    invert_right_(declare_parameter<bool>("invert_right", false)),
    last_pwm_(
      {std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::min(),
       std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::min()})
  {
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto cmd_vel_stamped_topic =
      declare_parameter<std::string>("cmd_vel_stamped_topic", "/diff_drive_controller/cmd_vel");
    const auto combined_topic = declare_parameter<std::string>("pwm_topic", "/allie/pwm");
    const auto front_left_topic =
      declare_parameter<std::string>("front_left_pwm_topic", "/allie/pwm/front_left");
    const auto rear_left_topic =
      declare_parameter<std::string>("rear_left_pwm_topic", "/allie/pwm/rear_left");
    const auto front_right_topic =
      declare_parameter<std::string>("front_right_pwm_topic", "/allie/pwm/front_right");
    const auto rear_right_topic =
      declare_parameter<std::string>("rear_right_pwm_topic", "/allie/pwm/rear_right");

    if (!(pwm_min_ < pwm_neutral_ && pwm_neutral_ < pwm_max_))
    {
      throw std::runtime_error("Expected pwm_min < pwm_neutral < pwm_max.");
    }
    if (max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0 || max_wheel_speed_ <= 0.0)
    {
      throw std::runtime_error("Speed limits must be positive.");
    }
    if (wheel_separation_ <= 0.0 || timeout_seconds_ <= 0.0)
    {
      throw std::runtime_error("wheel_separation and cmd_timeout_sec must be positive.");
    }

    combined_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(combined_topic, 10);
    front_left_pub_ = create_publisher<std_msgs::msg::Int16>(front_left_topic, 10);
    rear_left_pub_ = create_publisher<std_msgs::msg::Int16>(rear_left_topic, 10);
    front_right_pub_ = create_publisher<std_msgs::msg::Int16>(front_right_topic, 10);
    rear_right_pub_ = create_publisher<std_msgs::msg::Int16>(rear_right_topic, 10);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10, std::bind(&AlliePwmDriver::handle_twist, this, std::placeholders::_1));
    cmd_vel_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_stamped_topic, 10,
      std::bind(&AlliePwmDriver::handle_twist_stamped, this, std::placeholders::_1));

    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&AlliePwmDriver::enforce_timeout, this));

    publish_pwm(pwm_neutral_, pwm_neutral_, pwm_neutral_, pwm_neutral_);
    last_command_time_ = now();

    RCLCPP_INFO(
      get_logger(), "Allie PWM driver listening on '%s' and '%s', publishing PWM on '%s'.",
      cmd_vel_topic.c_str(), cmd_vel_stamped_topic.c_str(), combined_topic.c_str());
  }

private:
  void handle_twist(const geometry_msgs::msg::Twist & twist) { apply_twist(twist); }

  void handle_twist_stamped(const geometry_msgs::msg::TwistStamped & twist)
  {
    apply_twist(twist.twist);
  }

  void apply_twist(const geometry_msgs::msg::Twist & twist)
  {
    const double linear = clamp_value(twist.linear.x, -max_linear_speed_, max_linear_speed_);
    const double angular = clamp_value(twist.angular.z, -max_angular_speed_, max_angular_speed_);
    const double wheel_term = 0.5 * wheel_separation_ * angular;

    const int16_t left_pwm = speed_to_pwm(linear - wheel_term, invert_left_);
    const int16_t right_pwm = speed_to_pwm(linear + wheel_term, invert_right_);

    publish_pwm(left_pwm, left_pwm, right_pwm, right_pwm);
    last_command_time_ = now();
  }

  int16_t speed_to_pwm(double wheel_speed, bool invert) const
  {
    if (invert)
    {
      wheel_speed = -wheel_speed;
    }

    const double normalized = clamp_value(wheel_speed / max_wheel_speed_, -1.0, 1.0);

    if (normalized >= 0.0)
    {
      const double span = static_cast<double>(pwm_max_ - pwm_neutral_);
      return static_cast<int16_t>(std::lround(pwm_neutral_ + normalized * span));
    }

    const double span = static_cast<double>(pwm_neutral_ - pwm_min_);
    return static_cast<int16_t>(std::lround(pwm_neutral_ + normalized * span));
  }

  void publish_pwm(
    int16_t front_left_pwm, int16_t rear_left_pwm, int16_t front_right_pwm, int16_t rear_right_pwm)
  {
    std_msgs::msg::Int16 front_left_msg;
    front_left_msg.data = front_left_pwm;
    front_left_pub_->publish(front_left_msg);

    std_msgs::msg::Int16 rear_left_msg;
    rear_left_msg.data = rear_left_pwm;
    rear_left_pub_->publish(rear_left_msg);

    std_msgs::msg::Int16 front_right_msg;
    front_right_msg.data = front_right_pwm;
    front_right_pub_->publish(front_right_msg);

    std_msgs::msg::Int16 rear_right_msg;
    rear_right_msg.data = rear_right_pwm;
    rear_right_pub_->publish(rear_right_msg);

    std_msgs::msg::Int16MultiArray combined_msg;
    combined_msg.data =
      std::vector<int16_t>{front_left_pwm, rear_left_pwm, front_right_pwm, rear_right_pwm};
    combined_pub_->publish(combined_msg);

    last_pwm_ = {front_left_pwm, rear_left_pwm, front_right_pwm, rear_right_pwm};
  }

  void enforce_timeout()
  {
    const auto elapsed = (now() - last_command_time_).seconds();
    if (elapsed >= timeout_seconds_)
    {
      publish_pwm(pwm_neutral_, pwm_neutral_, pwm_neutral_, pwm_neutral_);
    }
  }

  int pwm_min_;
  int pwm_neutral_;
  int pwm_max_;
  double wheel_separation_;
  double max_linear_speed_;
  double max_angular_speed_;
  double max_wheel_speed_;
  double timeout_seconds_;
  bool invert_left_;
  bool invert_right_;

  std::array<int16_t, 4> last_pwm_;
  rclcpp::Time last_command_time_;

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr combined_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr front_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rear_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr front_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rear_right_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlliePwmDriver>());
  rclcpp::shutdown();
  return 0;
}
