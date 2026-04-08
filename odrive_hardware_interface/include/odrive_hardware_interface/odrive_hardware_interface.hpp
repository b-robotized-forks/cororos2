// Copyright (c) 2026, b-robotized Group

#pragma once

#include <sys/types.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace odrive_hardware_interface
{

struct ODriveAxisTelemetry
{
  double current_a{0.0};
  double fet_temp_c{std::numeric_limits<double>::quiet_NaN()};
  double i2t{0.0};
  int state{0};
  std::uint64_t axis_error{0};
  std::uint64_t motor_error{0};
  std::uint64_t encoder_error{0};
  std::uint64_t controller_error{0};
};

struct ODriveBoardTelemetry
{
  double vbus_voltage{std::numeric_limits<double>::quiet_NaN()};
  double estop_voltage{std::numeric_limits<double>::quiet_NaN()};
};

struct ODriveTelemetry
{
  bool active{false};
  bool i2t_latched{false};
  ODriveBoardTelemetry front;
  ODriveBoardTelemetry rear;
  std::array<ODriveAxisTelemetry, 4> axes{};
};

class ODriveHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ODriveHardwareInterface)
  ~ODriveHardwareInterface() override;

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
  bool parse_status_response(const std::string & response, ODriveTelemetry & telemetry);
  bool ensure_publishers();
  void poll_and_publish_status();
  void publish_status(const ODriveTelemetry & telemetry);
  void reset_status_publish_state();
  bool response_has_error(const std::string & response) const;
  void set_backend_error(const std::string & error);
  void clear_backend_error();
  bool run_service_command(
    const std::string & command, const std::string & success_message, bool zero_commands,
    bool allow_backend_start, std::string & message);

  std::string python_executable_;
  std::string front_serial_number_;
  std::string rear_serial_number_;
  int front_right_axis_{0};
  int rear_right_axis_{1};
  double connect_timeout_{30.0};
  bool clear_errors_on_startup_{true};
  double status_interval_sec_{0.5};
  int estop_adc_channel_{5};
  double i2t_current_nominal_{2.0};
  double i2t_update_rate_{0.01};
  double i2t_resume_threshold_{2220.0};
  double i2t_warning_threshold_{3330.0};
  double i2t_error_threshold_{6660.0};

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  pid_t backend_pid_{-1};
  FILE * backend_in_{nullptr};
  FILE * backend_out_{nullptr};
  bool backend_running_{false};
  std::recursive_mutex backend_mutex_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_vbus_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rear_vbus_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_estop_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rear_estop_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> current_pubs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> temp_pubs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> i2t_pubs_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr preroll_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr engage_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
  std::chrono::steady_clock::time_point last_status_publish_{};
  bool has_published_status_{false};
  std::string last_status_summary_;
  std::string last_backend_error_;
};

}  // namespace odrive_hardware_interface
