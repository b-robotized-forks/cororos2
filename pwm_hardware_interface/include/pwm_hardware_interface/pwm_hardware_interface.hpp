// Copyright (c) 2026, b-robotized Group

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace pwm_hardware_interface
{

class PwmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PwmHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int16_t speed_to_pwm(double wheel_velocity_rad_s, bool invert) const;
  bool ensure_publishers();
  void publish_pwm(
    int16_t front_left_pwm, int16_t rear_left_pwm, int16_t front_right_pwm, int16_t rear_right_pwm);

  std::string pwm_topic_;
  std::string front_left_pwm_topic_;
  std::string rear_left_pwm_topic_;
  std::string front_right_pwm_topic_;
  std::string rear_right_pwm_topic_;
  int pwm_min_{1000};
  int pwm_neutral_{1500};
  int pwm_max_{2000};
  double wheel_radius_{0.205};
  double max_wheel_speed_mps_{2.8};
  bool invert_left_{false};
  bool invert_right_{false};

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  std::array<int16_t, 4> last_pwm_{
    {static_cast<int16_t>(0), static_cast<int16_t>(0), static_cast<int16_t>(0),
     static_cast<int16_t>(0)}};

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr combined_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr front_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rear_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr front_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rear_right_pub_;
};

}  // namespace pwm_hardware_interface
