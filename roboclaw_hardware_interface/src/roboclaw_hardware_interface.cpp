// Copyright (c) 2026, b-robotized Group

#include "roboclaw_hardware_interface/roboclaw_hardware_interface.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{
const char * bool_to_string(bool value) { return value ? "true" : "false"; }

std::string format_double(double value)
{
  if (std::isfinite(value))
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << value;
    return stream.str();
  }
  return "nan";
}

const std::vector<std::pair<int, std::pair<uint8_t, const char *>>> & error_table()
{
  static const std::vector<std::pair<int, std::pair<uint8_t, const char *>>> table = {
    {0x0001, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "M1 over current"}},
    {0x0002, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "M2 over current"}},
    {0x0004, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Emergency Stop"}},
    {0x0008, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Temperature1"}},
    {0x0010, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Temperature2"}},
    {0x0020, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Main batt voltage high"}},
    {0x0040, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Logic batt voltage high"}},
    {0x0080, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Logic batt voltage low"}},
    {0x0100, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "M1 driver fault"}},
    {0x0200, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "M2 driver fault"}},
    {0x0400, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "Main batt voltage high"}},
    {0x0800, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "Main batt voltage low"}},
    {0x1000, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "Temperature1"}},
    {0x2000, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "Temperature2"}},
    {0x4000, {diagnostic_msgs::msg::DiagnosticStatus::OK, "M1 home"}},
    {0x8000, {diagnostic_msgs::msg::DiagnosticStatus::OK, "M2 home"}},
  };
  return table;
}

uint8_t diagnostic_level_from_error_word(int error_word)
{
  if (error_word < 0)
  {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  for (const auto & entry : error_table())
  {
    if ((error_word & entry.first) != 0)
    {
      level = std::max(level, entry.second.first);
    }
  }
  return level;
}

std::string diagnostic_message_from_error_word(int error_word)
{
  if (error_word < 0)
  {
    return "Roboclaw error register unavailable";
  }
  if (error_word == 0)
  {
    return "Normal";
  }

  std::ostringstream stream;
  bool first = true;
  for (const auto & entry : error_table())
  {
    if ((error_word & entry.first) == 0)
    {
      continue;
    }
    if (!first)
    {
      stream << "; ";
    }
    stream << entry.second.second;
    first = false;
  }

  if (first)
  {
    stream << "Roboclaw reported an unrecognized status code";
  }
  return stream.str();
}

diagnostic_msgs::msg::KeyValue make_key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
}  // namespace

namespace roboclaw_hardware_interface
{

RoboclawHardwareInterface::~RoboclawHardwareInterface() { stop_backend(); }

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::signal(SIGPIPE, SIG_IGN);

  if (info_.joints.size() != 2)
  {
    fprintf(
      stderr, "roboclaw_hardware_interface expects exactly 2 track joints, got %zu\n",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  python_executable_ = info_.hardware_parameters.count("python_executable") != 0
                         ? info_.hardware_parameters.at("python_executable")
                         : "python3";
  device_ = info_.hardware_parameters.count("device") != 0
              ? info_.hardware_parameters.at("device")
              : "/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x60A-if00";
  baud_ = info_.hardware_parameters.count("baud") != 0
            ? std::stoi(info_.hardware_parameters.at("baud"))
            : 115200;
  address_ = info_.hardware_parameters.count("address") != 0
               ? std::stoi(info_.hardware_parameters.at("address"))
               : 128;
  use_encoder_ = info_.hardware_parameters.count("use_encoder") != 0
                   ? info_.hardware_parameters.at("use_encoder") == "true"
                   : false;
  max_speed_ = info_.hardware_parameters.count("max_speed") != 0
                 ? std::stod(info_.hardware_parameters.at("max_speed"))
                 : 1.2;
  ticks_at_max_speed_ = info_.hardware_parameters.count("ticks_at_max_speed") != 0
                          ? std::stod(info_.hardware_parameters.at("ticks_at_max_speed"))
                          : 32760.0;
  acceleration_ = info_.hardware_parameters.count("acceleration") != 0
                    ? std::stoi(info_.hardware_parameters.at("acceleration"))
                    : 32000;
  ticks_per_meter_ = info_.hardware_parameters.count("ticks_per_meter") != 0
                       ? std::stod(info_.hardware_parameters.at("ticks_per_meter"))
                       : 4342.2;
  m1_invert_ = info_.hardware_parameters.count("m1_invert") != 0
                 ? info_.hardware_parameters.at("m1_invert") == "true"
                 : false;
  m2_invert_ = info_.hardware_parameters.count("m2_invert") != 0
                 ? info_.hardware_parameters.at("m2_invert") == "true"
                 : false;
  m1_encoder_sign_ = info_.hardware_parameters.count("m1_encoder_sign") != 0
                       ? std::stoi(info_.hardware_parameters.at("m1_encoder_sign"))
                       : 1;
  m2_encoder_sign_ = info_.hardware_parameters.count("m2_encoder_sign") != 0
                       ? std::stoi(info_.hardware_parameters.at("m2_encoder_sign"))
                       : 1;
  wheel_radius_ = info_.hardware_parameters.count("wheel_radius") != 0
                    ? std::stod(info_.hardware_parameters.at("wheel_radius"))
                    : 0.129;
  status_interval_sec_ = info_.hardware_parameters.count("status_interval_sec") != 0
                           ? std::stod(info_.hardware_parameters.at("status_interval_sec"))
                           : 2.0;

  if (device_.empty())
  {
    fprintf(stderr, "roboclaw_hardware_interface requires a non-empty serial device\n");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (
    max_speed_ <= 0.0 || ticks_at_max_speed_ <= 0.0 || ticks_per_meter_ <= 0.0 ||
    wheel_radius_ <= 0.0 || status_interval_sec_ < 0.0)
  {
    fprintf(stderr, "roboclaw_hardware_interface requires positive scale parameters\n");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboclawHardwareInterface::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface>
RoboclawHardwareInterface::export_command_interfaces()
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

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!start_backend())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!expect_ok("ACTIVATE"))
  {
    stop_backend();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!ensure_publishers())
  {
    stop_backend();
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  reset_status_publish_state();

  if (use_encoder_)
  {
    if (
      read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.0)) !=
      hardware_interface::return_type::OK)
    {
      stop_backend();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  poll_and_publish_status();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    expect_ok("DEACTIVATE");
    stop_backend();
  }
  reset_status_publish_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoboclawHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!use_encoder_)
  {
    const double dt = period.seconds();
    for (size_t i = 0; i < hw_commands_.size(); ++i)
    {
      hw_velocities_[i] = hw_commands_[i];
      hw_positions_[i] += hw_velocities_[i] * dt;
    }
    poll_and_publish_status();
    return hardware_interface::return_type::OK;
  }

  std::string response;
  if (!send_command("READ", response))
  {
    return hardware_interface::return_type::ERROR;
  }
  const bool ok = parse_state_response(response);
  poll_and_publish_status();
  return ok ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type RoboclawHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const double left_mps = hw_commands_[0] * wheel_radius_;
  const double right_mps = hw_commands_[1] * wheel_radius_;

  std::ostringstream command;
  command << "WRITE " << left_mps << ' ' << right_mps;

  return expect_ok(command.str()) ? hardware_interface::return_type::OK
                                  : hardware_interface::return_type::ERROR;
}

bool RoboclawHardwareInterface::start_backend()
{
  if (backend_running_)
  {
    return true;
  }

  const auto share_dir =
    ament_index_cpp::get_package_share_directory("roboclaw_hardware_interface");
  const auto script_path = share_dir + "/python/roboclaw_backend.py";

  int child_stdin[2];
  int child_stdout[2];
  if (pipe(child_stdin) != 0 || pipe(child_stdout) != 0)
  {
    perror("pipe");
    return false;
  }

  backend_pid_ = fork();
  if (backend_pid_ < 0)
  {
    perror("fork");
    return false;
  }

  if (backend_pid_ == 0)
  {
    dup2(child_stdin[0], STDIN_FILENO);
    dup2(child_stdout[1], STDOUT_FILENO);
    close(child_stdin[1]);
    close(child_stdout[0]);

    execlp(
      python_executable_.c_str(), python_executable_.c_str(), "-u", script_path.c_str(), "--device",
      device_.c_str(), "--baud", std::to_string(baud_).c_str(), "--address",
      std::to_string(address_).c_str(), "--use-encoder", bool_to_string(use_encoder_),
      "--max-speed", std::to_string(max_speed_).c_str(), "--ticks-at-max-speed",
      std::to_string(ticks_at_max_speed_).c_str(), "--acceleration",
      std::to_string(acceleration_).c_str(), "--ticks-per-meter",
      std::to_string(ticks_per_meter_).c_str(), "--m1-invert", bool_to_string(m1_invert_),
      "--m2-invert", bool_to_string(m2_invert_), "--m1-encoder-sign",
      std::to_string(m1_encoder_sign_).c_str(), "--m2-encoder-sign",
      std::to_string(m2_encoder_sign_).c_str(), static_cast<char *>(nullptr));
    perror("execlp");
    _exit(127);
  }

  close(child_stdin[0]);
  close(child_stdout[1]);

  backend_in_ = fdopen(child_stdin[1], "w");
  backend_out_ = fdopen(child_stdout[0], "r");
  if (backend_in_ == nullptr || backend_out_ == nullptr)
  {
    perror("fdopen");
    stop_backend();
    return false;
  }

  backend_running_ = true;

  std::string response;
  if (!send_command("PING", response) || response != "PONG")
  {
    fprintf(stderr, "Failed to start Roboclaw backend, got response '%s'\n", response.c_str());
    stop_backend();
    return false;
  }

  return true;
}

void RoboclawHardwareInterface::stop_backend()
{
  if (backend_running_)
  {
    int status = 0;
    const pid_t child_state = backend_pid_ > 0 ? waitpid(backend_pid_, &status, WNOHANG) : -1;
    if (child_state == 0)
    {
      std::string response;
      send_command("EXIT", response);
    }
  }

  if (backend_in_ != nullptr)
  {
    fclose(backend_in_);
    backend_in_ = nullptr;
  }
  if (backend_out_ != nullptr)
  {
    fclose(backend_out_);
    backend_out_ = nullptr;
  }
  if (backend_pid_ > 0)
  {
    int status = 0;
    waitpid(backend_pid_, &status, 0);
    backend_pid_ = -1;
  }
  backend_running_ = false;
}

bool RoboclawHardwareInterface::send_command(const std::string & command, std::string & response)
{
  if (!backend_running_ || backend_in_ == nullptr || backend_out_ == nullptr)
  {
    return false;
  }

  if (fprintf(backend_in_, "%s\n", command.c_str()) < 0)
  {
    perror("fprintf");
    return false;
  }
  fflush(backend_in_);

  char buffer[1024];
  if (fgets(buffer, sizeof(buffer), backend_out_) == nullptr)
  {
    if (ferror(backend_out_) != 0)
    {
      perror("fgets");
    }
    return false;
  }

  response = buffer;
  while (!response.empty() && (response.back() == '\n' || response.back() == '\r'))
  {
    response.pop_back();
  }

  return true;
}

bool RoboclawHardwareInterface::expect_ok(const std::string & command)
{
  std::string response;
  if (!send_command(command, response))
  {
    return false;
  }
  if (response != "OK")
  {
    fprintf(stderr, "Command '%s' failed with response '%s'\n", command.c_str(), response.c_str());
    return false;
  }
  return true;
}

bool RoboclawHardwareInterface::parse_state_response(const std::string & response)
{
  std::istringstream stream(response);
  std::string token;
  stream >> token;
  if (token != "STATE")
  {
    fprintf(stderr, "Unexpected backend state response: '%s'\n", response.c_str());
    return false;
  }

  double left_position_m = 0.0;
  double left_velocity_mps = 0.0;
  double right_position_m = 0.0;
  double right_velocity_mps = 0.0;
  if (!(stream >> left_position_m >> left_velocity_mps >> right_position_m >> right_velocity_mps))
  {
    fprintf(stderr, "Failed to parse backend state response: '%s'\n", response.c_str());
    return false;
  }

  hw_positions_[0] = left_position_m / wheel_radius_;
  hw_velocities_[0] = left_velocity_mps / wheel_radius_;
  hw_positions_[1] = right_position_m / wheel_radius_;
  hw_velocities_[1] = right_velocity_mps / wheel_radius_;

  return true;
}

bool RoboclawHardwareInterface::parse_status_response(
  const std::string & response, RoboclawTelemetry & telemetry)
{
  std::istringstream stream(response);
  std::string token;
  stream >> token;
  if (token != "STATUS")
  {
    fprintf(stderr, "Unexpected backend status response: '%s'\n", response.c_str());
    return false;
  }

  if (!(stream >> telemetry.main_battery_voltage >> telemetry.logic_battery_voltage >>
        telemetry.m1_current >> telemetry.m2_current >> telemetry.temp1_c >> telemetry.temp2_c >>
        telemetry.error_word))
  {
    fprintf(stderr, "Failed to parse backend status response: '%s'\n", response.c_str());
    return false;
  }

  return true;
}

bool RoboclawHardwareInterface::ensure_publishers()
{
  if (
    battery_pub_ != nullptr && m1_current_pub_ != nullptr && m2_current_pub_ != nullptr &&
    diagnostics_pub_ != nullptr)
  {
    return true;
  }

  const auto node = get_node();
  if (node == nullptr)
  {
    RCLCPP_ERROR(get_logger(), "Failed to access the default hardware component node.");
    return false;
  }

  battery_pub_ =
    node->create_publisher<sensor_msgs::msg::BatteryState>("roboclaw/battery_state", 10);
  m1_current_pub_ = node->create_publisher<std_msgs::msg::Float32>("roboclaw/m1_current", 10);
  m2_current_pub_ = node->create_publisher<std_msgs::msg::Float32>("roboclaw/m2_current", 10);
  diagnostics_pub_ =
    node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  return true;
}

void RoboclawHardwareInterface::poll_and_publish_status()
{
  if (!ensure_publishers())
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (
    has_published_status_ &&
    std::chrono::duration<double>(now - last_status_publish_).count() < status_interval_sec_)
  {
    return;
  }

  std::string response;
  if (!send_command("STATUS", response))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to read Roboclaw status from backend.");
    return;
  }

  RoboclawTelemetry telemetry;
  if (!parse_status_response(response, telemetry))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to parse Roboclaw status response.");
    return;
  }

  publish_status(telemetry);
  last_status_publish_ = now;
  has_published_status_ = true;
}

void RoboclawHardwareInterface::publish_status(const RoboclawTelemetry & telemetry)
{
  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = get_clock()->now();
  battery_msg.voltage = telemetry.main_battery_voltage;
  battery_msg.current = telemetry.m1_current + telemetry.m2_current;
  battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  battery_pub_->publish(battery_msg);

  std_msgs::msg::Float32 m1_current_msg;
  m1_current_msg.data = static_cast<float>(telemetry.m1_current);
  m1_current_pub_->publish(m1_current_msg);

  std_msgs::msg::Float32 m2_current_msg;
  m2_current_msg.data = static_cast<float>(telemetry.m2_current);
  m2_current_pub_->publish(m2_current_msg);

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Roboclaw";
  status.hardware_id = "Roboclaw@" + device_ + "#" + std::to_string(address_);
  status.level = diagnostic_level_from_error_word(telemetry.error_word);
  status.message = diagnostic_message_from_error_word(telemetry.error_word);
  status.values.push_back(make_key_value("device", device_));
  status.values.push_back(make_key_value("address", std::to_string(address_)));
  status.values.push_back(
    make_key_value("main_battery_voltage_v", format_double(telemetry.main_battery_voltage)));
  status.values.push_back(
    make_key_value("logic_battery_voltage_v", format_double(telemetry.logic_battery_voltage)));
  status.values.push_back(make_key_value("m1_current_a", format_double(telemetry.m1_current)));
  status.values.push_back(make_key_value("m2_current_a", format_double(telemetry.m2_current)));
  status.values.push_back(make_key_value("temp1_c", format_double(telemetry.temp1_c)));
  status.values.push_back(make_key_value("temp2_c", format_double(telemetry.temp2_c)));
  {
    std::ostringstream stream;
    if (telemetry.error_word >= 0)
    {
      stream << "0x" << std::hex << std::uppercase << telemetry.error_word;
    }
    else
    {
      stream << "unavailable";
    }
    status.values.push_back(make_key_value("error_word", stream.str()));
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = battery_msg.header.stamp;
  diagnostics.status.push_back(status);
  diagnostics_pub_->publish(diagnostics);

  if (telemetry.error_word != last_error_word_)
  {
    if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR)
    {
      RCLCPP_ERROR(get_logger(), "Roboclaw status changed: %s", status.message.c_str());
    }
    else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN)
    {
      RCLCPP_WARN(get_logger(), "Roboclaw status changed: %s", status.message.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Roboclaw status changed: %s", status.message.c_str());
    }
    last_error_word_ = telemetry.error_word;
  }
}

void RoboclawHardwareInterface::reset_status_publish_state()
{
  has_published_status_ = false;
  last_status_publish_ = std::chrono::steady_clock::time_point{};
  last_error_word_ = -2;
}

}  // namespace roboclaw_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  roboclaw_hardware_interface::RoboclawHardwareInterface, hardware_interface::SystemInterface)
