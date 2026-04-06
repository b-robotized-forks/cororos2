// Copyright (c) 2026, b-robotized Group

#pragma once

#include <sys/types.h>
#include <cstdio>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace roboclaw_hardware_interface
{

class RoboclawHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoboclawHardwareInterface)
  ~RoboclawHardwareInterface() override;

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
  bool start_backend();
  void stop_backend();
  bool send_command(const std::string & command, std::string & response);
  bool expect_ok(const std::string & command);
  bool parse_state_response(const std::string & response);

  std::string python_executable_;
  std::string device_;
  int baud_{115200};
  int address_{128};
  bool use_encoder_{false};
  double max_speed_{1.2};
  double ticks_at_max_speed_{32760.0};
  int acceleration_{32000};
  double ticks_per_meter_{4342.2};
  bool m1_invert_{false};
  bool m2_invert_{false};
  int m1_encoder_sign_{1};
  int m2_encoder_sign_{1};
  double wheel_radius_{0.129};

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  pid_t backend_pid_{-1};
  FILE * backend_in_{nullptr};
  FILE * backend_out_{nullptr};
  bool backend_running_{false};
};

}  // namespace roboclaw_hardware_interface
