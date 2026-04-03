// Copyright (c) 2026, b-robotized Group

#include "pwm_hardware_interface/pwm_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{
template <typename T>
T clamp_value(T value, T minimum, T maximum)
{
  return std::max(minimum, std::min(value, maximum));
}
}  // namespace

namespace pwm_hardware_interface
{

hardware_interface::CallbackReturn PwmHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 4)
  {
    RCLCPP_ERROR(
      get_logger(), "pwm_hardware_interface expects exactly 4 wheel joints, got %zu",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hardware_parameters = info_.hardware_parameters;

  pwm_topic_ = hardware_parameters.count("pwm_topic") != 0 ? hardware_parameters.at("pwm_topic")
                                                           : "/allie/pwm";
  front_left_pwm_topic_ = hardware_parameters.count("front_left_pwm_topic") != 0
                            ? hardware_parameters.at("front_left_pwm_topic")
                            : "/allie/pwm/front_left";
  rear_left_pwm_topic_ = hardware_parameters.count("rear_left_pwm_topic") != 0
                           ? hardware_parameters.at("rear_left_pwm_topic")
                           : "/allie/pwm/rear_left";
  front_right_pwm_topic_ = hardware_parameters.count("front_right_pwm_topic") != 0
                             ? hardware_parameters.at("front_right_pwm_topic")
                             : "/allie/pwm/front_right";
  rear_right_pwm_topic_ = hardware_parameters.count("rear_right_pwm_topic") != 0
                            ? hardware_parameters.at("rear_right_pwm_topic")
                            : "/allie/pwm/rear_right";
  pwm_min_ =
    hardware_parameters.count("pwm_min") != 0 ? std::stoi(hardware_parameters.at("pwm_min")) : 1000;
  pwm_neutral_ = hardware_parameters.count("pwm_neutral") != 0
                   ? std::stoi(hardware_parameters.at("pwm_neutral"))
                   : 1500;
  pwm_max_ =
    hardware_parameters.count("pwm_max") != 0 ? std::stoi(hardware_parameters.at("pwm_max")) : 2000;
  wheel_radius_ = hardware_parameters.count("wheel_radius") != 0
                    ? std::stod(hardware_parameters.at("wheel_radius"))
                    : 0.205;
  max_wheel_speed_mps_ = hardware_parameters.count("max_wheel_speed_mps") != 0
                           ? std::stod(hardware_parameters.at("max_wheel_speed_mps"))
                           : 2.8;
  invert_left_ = hardware_parameters.count("invert_left") != 0
                   ? hardware_parameters.at("invert_left") == "true"
                   : false;
  invert_right_ = hardware_parameters.count("invert_right") != 0
                    ? hardware_parameters.at("invert_right") == "true"
                    : false;

  if (!(pwm_min_ < pwm_neutral_ && pwm_neutral_ < pwm_max_))
  {
    RCLCPP_ERROR(get_logger(), "Expected pwm_min < pwm_neutral < pwm_max.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (wheel_radius_ <= 0.0 || max_wheel_speed_mps_ <= 0.0)
  {
    RCLCPP_ERROR(get_logger(), "wheel_radius and max_wheel_speed_mps must be positive.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  last_pwm_.fill(static_cast<int16_t>(pwm_neutral_));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PwmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PwmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (!ensure_publishers())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  publish_pwm(pwm_neutral_, pwm_neutral_, pwm_neutral_, pwm_neutral_);
  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (combined_pub_ != nullptr)
  {
    publish_pwm(pwm_neutral_, pwm_neutral_, pwm_neutral_, pwm_neutral_);
  }

  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PwmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    hw_velocities_[i] = hw_commands_[i];
    hw_positions_[i] += hw_velocities_[i] * dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PwmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!ensure_publishers())
  {
    return hardware_interface::return_type::ERROR;
  }

  const int16_t front_left_pwm = speed_to_pwm(hw_commands_[0], invert_left_);
  const int16_t front_right_pwm = speed_to_pwm(hw_commands_[1], invert_right_);
  const int16_t rear_left_pwm = speed_to_pwm(hw_commands_[2], invert_left_);
  const int16_t rear_right_pwm = speed_to_pwm(hw_commands_[3], invert_right_);

  publish_pwm(front_left_pwm, rear_left_pwm, front_right_pwm, rear_right_pwm);
  return hardware_interface::return_type::OK;
}

int16_t PwmHardwareInterface::speed_to_pwm(double wheel_velocity_rad_s, bool invert) const
{
  double wheel_speed_mps = wheel_velocity_rad_s * wheel_radius_;
  if (invert)
  {
    wheel_speed_mps = -wheel_speed_mps;
  }

  const double normalized = clamp_value(wheel_speed_mps / max_wheel_speed_mps_, -1.0, 1.0);

  if (normalized >= 0.0)
  {
    const double span = static_cast<double>(pwm_max_ - pwm_neutral_);
    return static_cast<int16_t>(std::lround(pwm_neutral_ + normalized * span));
  }

  const double span = static_cast<double>(pwm_neutral_ - pwm_min_);
  return static_cast<int16_t>(std::lround(pwm_neutral_ + normalized * span));
}

bool PwmHardwareInterface::ensure_publishers()
{
  if (
    combined_pub_ != nullptr && front_left_pub_ != nullptr && rear_left_pub_ != nullptr &&
    front_right_pub_ != nullptr && rear_right_pub_ != nullptr)
  {
    return true;
  }

  const auto node = get_node();
  if (node == nullptr)
  {
    RCLCPP_ERROR(get_logger(), "Failed to access the default hardware component node.");
    return false;
  }

  combined_pub_ = node->create_publisher<std_msgs::msg::Int16MultiArray>(pwm_topic_, 10);
  front_left_pub_ = node->create_publisher<std_msgs::msg::Int16>(front_left_pwm_topic_, 10);
  rear_left_pub_ = node->create_publisher<std_msgs::msg::Int16>(rear_left_pwm_topic_, 10);
  front_right_pub_ = node->create_publisher<std_msgs::msg::Int16>(front_right_pwm_topic_, 10);
  rear_right_pub_ = node->create_publisher<std_msgs::msg::Int16>(rear_right_pwm_topic_, 10);

  return true;
}

void PwmHardwareInterface::publish_pwm(
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
  combined_msg.data = {front_left_pwm, rear_left_pwm, front_right_pwm, rear_right_pwm};
  combined_pub_->publish(combined_msg);

  last_pwm_ = {front_left_pwm, rear_left_pwm, front_right_pwm, rear_right_pwm};
}

}  // namespace pwm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  pwm_hardware_interface::PwmHardwareInterface, hardware_interface::SystemInterface)
