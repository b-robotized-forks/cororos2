// Copyright (c) 2026, b-robotized Group

#pragma once

#include <sys/types.h>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float32.hpp"

namespace roboclaw_hardware_interface
{

struct RoboclawTelemetry
{
  double main_battery_voltage{0.0};
  double logic_battery_voltage{0.0};
  double m1_current{0.0};
  double m2_current{0.0};
  double temp1_c{0.0};
  double temp2_c{0.0};
  int error_word{-1};
};

class RoboclawHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoboclawHardwareInterface)
  ~RoboclawHardwareInterface() override;

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
  bool validate_joint_configuration() const;
  bool start_backend();
  void stop_backend();
  bool send_command(const std::string & command, std::string & response);
  bool expect_ok(const std::string & command);
  bool read_state_from_backend();
  bool parse_state_response(const std::string & response);
  bool parse_status_response(const std::string & response, RoboclawTelemetry & telemetry);
  bool ensure_publishers();
  void release_publishers();
  void reset_command_and_state_buffers();
  void reset_runtime_state();
  bool is_active_lifecycle_state() const;
  bool can_read_in_current_state() const;
  bool poll_and_publish_status();
  void publish_status(const RoboclawTelemetry & telemetry);
  void reset_status_publish_state();

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
  double status_interval_sec_{2.0};

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  pid_t backend_pid_{-1};
  FILE * backend_in_{nullptr};
  FILE * backend_out_{nullptr};
  bool backend_running_{false};

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr logic_battery_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m1_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m2_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp1_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp2_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::chrono::steady_clock::time_point last_status_publish_{};
  bool has_published_status_{false};
  int last_error_word_{-2};
};

}  // namespace roboclaw_hardware_interface
