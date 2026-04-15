// Copyright (c) 2026, b-robotized Group

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
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
  bool open_roboclaw();
  void close_roboclaw();
  bool configure_roboclaw_serial();
  bool ping_roboclaw();
  bool activate_backend();
  bool deactivate_backend();
  bool drive_backend(double left_mps, double right_mps);
  bool stop_motors();
  bool read_state_from_backend();
  bool read_status_from_backend(RoboclawTelemetry & telemetry);
  bool ensure_publishers();
  void release_publishers();
  void reset_command_and_state_buffers();
  void reset_runtime_state();
  bool is_active_lifecycle_state() const;
  bool can_read_in_current_state() const;
  bool poll_and_publish_status();
  void publish_status(const RoboclawTelemetry & telemetry);
  void reset_status_publish_state();

  bool flush_serial_io();
  bool write_all(const uint8_t * data, size_t size);
  bool read_exact(uint8_t * data, size_t size);
  void crc_clear();
  void crc_update(uint8_t data);
  bool send_packet_command(uint8_t command);
  bool write_byte(uint8_t value);
  bool write_word(uint16_t value);
  bool write_long(uint32_t value);
  bool write_signed_word(int16_t value);
  bool write_signed_long(int32_t value);
  bool write_checksum_and_ack();
  bool read_byte(uint8_t & value);
  bool read_word(uint16_t & value);
  bool read_long(uint32_t & value);
  bool read_signed_long(int32_t & value);
  bool read_checksum();
  bool read_uint16_command(uint8_t command, uint16_t & value);
  bool read_uint32_command(uint8_t command, uint32_t & value);
  bool read_encoder_command(uint8_t command, int32_t & ticks);
  bool write_no_arg_command(uint8_t command);
  bool write_byte_command(uint8_t command, uint8_t value);
  bool write_signed_word_uint32_command(uint8_t command, int16_t value1, uint32_t value2);
  bool write_signed_long_pair_command(uint8_t command, int32_t value1, int32_t value2);

  std::string device_;
  int roboclaw_fd_{-1};
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

  bool backend_running_{false};
  uint16_t crc_{0};
  std::optional<int32_t> last_left_ticks_;
  std::optional<int32_t> last_right_ticks_;
  std::chrono::steady_clock::time_point last_encoder_read_time_{};

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
