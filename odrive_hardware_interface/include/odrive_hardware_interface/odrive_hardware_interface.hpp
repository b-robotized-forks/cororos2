// Copyright (c) 2026, b-robotized Group

#pragma once

#include <sys/types.h>
#include <cstdio>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace odrive_hardware_interface
{

class ODriveHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ODriveHardwareInterface)
  ~ODriveHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
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
  bool read_state_from_backend();
  bool parse_state_response(const std::string & response);
  bool validate_joint_configuration() const;
  void reset_command_and_state_buffers();
  void reset_runtime_state();
  bool is_active_lifecycle_state() const;
  bool can_read_in_current_state() const;

  std::string python_executable_;
  std::string front_serial_number_;
  std::string rear_serial_number_;
  int front_right_axis_{0};
  int rear_right_axis_{1};
  double connect_timeout_{30.0};

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  pid_t backend_pid_{-1};
  FILE * backend_in_{nullptr};
  FILE * backend_out_{nullptr};
  bool backend_running_{false};
};

}  // namespace odrive_hardware_interface
