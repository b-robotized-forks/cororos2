// Copyright (c) 2026, b-robotized Group

#include "odrive_hardware_interface/odrive_hardware_interface.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr int kAxisStateIdle = 1;
constexpr int kAxisStateClosedLoopControl = 8;

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

std::string format_hex(std::uint64_t value)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase << value;
  return stream.str();
}

const char * axis_state_name(int state)
{
  switch (state)
  {
    case 0:
      return "UNDEFINED";
    case 1:
      return "IDLE";
    case 2:
      return "STARTUP_SEQUENCE";
    case 3:
      return "FULL_CALIBRATION_SEQUENCE";
    case 4:
      return "MOTOR_CALIBRATION";
    case 5:
      return "SENSORLESS_CONTROL";
    case 6:
      return "ENCODER_INDEX_SEARCH";
    case 7:
      return "ENCODER_OFFSET_CALIBRATION";
    case 8:
      return "CLOSED_LOOP_CONTROL";
    case 9:
      return "LOCKIN_SPIN";
    case 10:
      return "ENCODER_DIR_FIND";
    default:
      return "UNKNOWN";
  }
}

diagnostic_msgs::msg::KeyValue make_key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
}  // namespace

namespace odrive_hardware_interface
{

ODriveHardwareInterface::~ODriveHardwareInterface() { stop_backend(); }

hardware_interface::CallbackReturn ODriveHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::signal(SIGPIPE, SIG_IGN);

  if (info_.joints.size() != 4)
  {
    fprintf(
      stderr, "odrive_hardware_interface expects exactly 4 wheel joints, got %zu\n",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  python_executable_ = info_.hardware_parameters.count("python_executable") != 0
                         ? info_.hardware_parameters.at("python_executable")
                         : "python3";
  front_serial_number_ = info_.hardware_parameters.count("front_serial_number") != 0
                           ? info_.hardware_parameters.at("front_serial_number")
                           : "";
  rear_serial_number_ = info_.hardware_parameters.count("rear_serial_number") != 0
                          ? info_.hardware_parameters.at("rear_serial_number")
                          : "";
  front_right_axis_ = info_.hardware_parameters.count("front_right_axis") != 0
                        ? std::stoi(info_.hardware_parameters.at("front_right_axis"))
                        : 0;
  rear_right_axis_ = info_.hardware_parameters.count("rear_right_axis") != 0
                       ? std::stoi(info_.hardware_parameters.at("rear_right_axis"))
                       : 1;
  connect_timeout_ = info_.hardware_parameters.count("connect_timeout") != 0
                       ? std::stod(info_.hardware_parameters.at("connect_timeout"))
                       : 30.0;
  clear_errors_on_startup_ = info_.hardware_parameters.count("clear_errors_on_startup") != 0
                               ? info_.hardware_parameters.at("clear_errors_on_startup") == "true"
                               : true;
  status_interval_sec_ = info_.hardware_parameters.count("status_interval_sec") != 0
                           ? std::stod(info_.hardware_parameters.at("status_interval_sec"))
                           : 0.5;
  estop_adc_channel_ = info_.hardware_parameters.count("estop_adc_channel") != 0
                         ? std::stoi(info_.hardware_parameters.at("estop_adc_channel"))
                         : 5;
  i2t_current_nominal_ = info_.hardware_parameters.count("i2t_current_nominal") != 0
                           ? std::stod(info_.hardware_parameters.at("i2t_current_nominal"))
                           : 2.0;
  i2t_update_rate_ = info_.hardware_parameters.count("i2t_update_rate") != 0
                       ? std::stod(info_.hardware_parameters.at("i2t_update_rate"))
                       : 0.01;
  i2t_resume_threshold_ = info_.hardware_parameters.count("i2t_resume_threshold") != 0
                            ? std::stod(info_.hardware_parameters.at("i2t_resume_threshold"))
                            : 2220.0;
  i2t_warning_threshold_ = info_.hardware_parameters.count("i2t_warning_threshold") != 0
                             ? std::stod(info_.hardware_parameters.at("i2t_warning_threshold"))
                             : 3330.0;
  i2t_error_threshold_ = info_.hardware_parameters.count("i2t_error_threshold") != 0
                           ? std::stod(info_.hardware_parameters.at("i2t_error_threshold"))
                           : 6660.0;

  if (front_serial_number_.empty() || rear_serial_number_.empty())
  {
    fprintf(
      stderr,
      "odrive_hardware_interface requires front_serial_number and rear_serial_number when mock "
      "hardware is disabled\n");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
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
ODriveHardwareInterface::export_command_interfaces()
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

hardware_interface::CallbackReturn ODriveHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  if (!start_backend())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!ensure_publishers())
  {
    stop_backend();
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  reset_status_publish_state();

  if (!expect_ok("ACTIVATE"))
  {
    stop_backend();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.0)) !=
    hardware_interface::return_type::OK)
  {
    stop_backend();
    return hardware_interface::CallbackReturn::ERROR;
  }

  poll_and_publish_status();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  if (backend_running_)
  {
    expect_ok("DEACTIVATE");
    stop_backend();
  }
  reset_status_publish_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ODriveHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  std::string response;
  if (!send_command("READ", response))
  {
    set_backend_error("READ transport failure");
    return hardware_interface::return_type::ERROR;
  }
  const bool ok = parse_state_response(response);
  if (ok)
  {
    poll_and_publish_status();
    clear_backend_error();
  }
  return ok ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type ODriveHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  std::ostringstream command;
  command << "WRITE";
  for (double cmd_rad_s : hw_commands_)
  {
    command << ' ' << (cmd_rad_s / kTwoPi);
  }

  return expect_ok(command.str()) ? hardware_interface::return_type::OK
                                  : hardware_interface::return_type::ERROR;
}

bool ODriveHardwareInterface::start_backend()
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  if (backend_running_)
  {
    return true;
  }

  const auto share_dir = ament_index_cpp::get_package_share_directory("odrive_hardware_interface");
  const auto script_path = share_dir + "/python/odrive_backend.py";

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
      python_executable_.c_str(), python_executable_.c_str(), "-u", script_path.c_str(),
      "--front-serial", front_serial_number_.c_str(), "--rear-serial", rear_serial_number_.c_str(),
      "--front-right-axis", std::to_string(front_right_axis_).c_str(), "--rear-right-axis",
      std::to_string(rear_right_axis_).c_str(), "--connect-timeout",
      std::to_string(connect_timeout_).c_str(), "--clear-errors-on-startup",
      clear_errors_on_startup_ ? "true" : "false", "--estop-adc-channel",
      std::to_string(estop_adc_channel_).c_str(), "--i2t-current-nominal",
      std::to_string(i2t_current_nominal_).c_str(), "--i2t-update-rate",
      std::to_string(i2t_update_rate_).c_str(), "--i2t-resume-threshold",
      std::to_string(i2t_resume_threshold_).c_str(), "--i2t-warning-threshold",
      std::to_string(i2t_warning_threshold_).c_str(), "--i2t-error-threshold",
      std::to_string(i2t_error_threshold_).c_str(), static_cast<char *>(nullptr));
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
    fprintf(stderr, "Failed to start ODrive backend, got response '%s'\n", response.c_str());
    stop_backend();
    return false;
  }

  return true;
}

void ODriveHardwareInterface::stop_backend()
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
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

bool ODriveHardwareInterface::send_command(const std::string & command, std::string & response)
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
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

  char buffer[4096];
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

bool ODriveHardwareInterface::expect_ok(const std::string & command)
{
  std::string response;
  if (!send_command(command, response))
  {
    set_backend_error(command + " transport failure");
    return false;
  }
  if (response != "OK")
  {
    set_backend_error(command + " failed: " + response);
    fprintf(stderr, "Command '%s' failed with response '%s'\n", command.c_str(), response.c_str());
    return false;
  }
  clear_backend_error();
  return true;
}

bool ODriveHardwareInterface::run_service_command(
  const std::string & command, const std::string & success_message, bool zero_commands,
  bool allow_backend_start, std::string & message)
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);

  if (zero_commands)
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  }

  if (command == "CONNECT")
  {
    if (!backend_running_)
    {
      if (!allow_backend_start || !start_backend())
      {
        message =
          last_backend_error_.empty() ? "Failed to start the ODrive backend." : last_backend_error_;
        return false;
      }
    }
    else if (!expect_ok(command))
    {
      message = last_backend_error_.empty() ? "Failed to reconnect the ODrive driver."
                                            : last_backend_error_;
      return false;
    }
  }
  else
  {
    if (!backend_running_)
    {
      if (command == "DISCONNECT")
      {
        clear_backend_error();
        message = "ODrive driver is already disconnected.";
        return true;
      }
      set_backend_error("ODrive backend is not running.");
      message = last_backend_error_;
      return false;
    }

    if (!expect_ok(command))
    {
      message = last_backend_error_.empty() ? command + " failed." : last_backend_error_;
      return false;
    }
  }

  poll_and_publish_status();
  clear_backend_error();
  message = success_message;
  return true;
}

bool ODriveHardwareInterface::parse_state_response(const std::string & response)
{
  if (response_has_error(response))
  {
    set_backend_error("READ failed: " + response);
    fprintf(stderr, "Backend read returned error '%s'\n", response.c_str());
    return false;
  }

  std::istringstream stream(response);
  std::string token;
  stream >> token;
  if (token != "STATE")
  {
    set_backend_error("Unexpected READ response: " + response);
    fprintf(stderr, "Unexpected backend state response: '%s'\n", response.c_str());
    return false;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    double rotations = 0.0;
    double rotations_per_second = 0.0;
    if (!(stream >> rotations >> rotations_per_second))
    {
      set_backend_error("Failed to parse READ response: " + response);
      fprintf(stderr, "Failed to parse backend state response: '%s'\n", response.c_str());
      return false;
    }
    hw_positions_[i] = rotations * kTwoPi;
    hw_velocities_[i] = rotations_per_second * kTwoPi;
  }

  return true;
}

bool ODriveHardwareInterface::parse_status_response(
  const std::string & response, ODriveTelemetry & telemetry)
{
  if (response_has_error(response))
  {
    set_backend_error("STATUS failed: " + response);
    fprintf(stderr, "Backend status returned error '%s'\n", response.c_str());
    return false;
  }

  std::istringstream stream(response);
  std::string token;
  stream >> token;
  if (token != "STATUS")
  {
    set_backend_error("Unexpected STATUS response: " + response);
    fprintf(stderr, "Unexpected backend status response: '%s'\n", response.c_str());
    return false;
  }

  double front_active = 0.0;
  double front_i2t_latched = 0.0;
  double rear_active = 0.0;
  double rear_i2t_latched = 0.0;
  if (!(stream >> telemetry.front.vbus_voltage >> telemetry.front.estop_voltage >> front_active >>
        front_i2t_latched))
  {
    set_backend_error("Failed to parse STATUS response: " + response);
    fprintf(stderr, "Failed to parse backend status response: '%s'\n", response.c_str());
    return false;
  }

  for (size_t i = 0; i < 2; ++i)
  {
    auto & axis = telemetry.axes[i];
    double state = 0.0;
    double axis_error = 0.0;
    double motor_error = 0.0;
    double encoder_error = 0.0;
    double controller_error = 0.0;
    if (!(stream >> axis.current_a >> axis.fet_temp_c >> axis.i2t >> state >> axis_error >>
          motor_error >> encoder_error >> controller_error))
    {
      set_backend_error("Failed to parse STATUS response: " + response);
      fprintf(stderr, "Failed to parse backend status response: '%s'\n", response.c_str());
      return false;
    }
    axis.state = static_cast<int>(state);
    axis.axis_error = static_cast<std::uint64_t>(axis_error);
    axis.motor_error = static_cast<std::uint64_t>(motor_error);
    axis.encoder_error = static_cast<std::uint64_t>(encoder_error);
    axis.controller_error = static_cast<std::uint64_t>(controller_error);
  }

  if (!(stream >> telemetry.rear.vbus_voltage >> telemetry.rear.estop_voltage >> rear_active >>
        rear_i2t_latched))
  {
    set_backend_error("Failed to parse STATUS response: " + response);
    fprintf(stderr, "Failed to parse backend status response: '%s'\n", response.c_str());
    return false;
  }

  for (size_t i = 2; i < telemetry.axes.size(); ++i)
  {
    auto & axis = telemetry.axes[i];
    double state = 0.0;
    double axis_error = 0.0;
    double motor_error = 0.0;
    double encoder_error = 0.0;
    double controller_error = 0.0;
    if (!(stream >> axis.current_a >> axis.fet_temp_c >> axis.i2t >> state >> axis_error >>
          motor_error >> encoder_error >> controller_error))
    {
      set_backend_error("Failed to parse STATUS response: " + response);
      fprintf(stderr, "Failed to parse backend status response: '%s'\n", response.c_str());
      return false;
    }
    axis.state = static_cast<int>(state);
    axis.axis_error = static_cast<std::uint64_t>(axis_error);
    axis.motor_error = static_cast<std::uint64_t>(motor_error);
    axis.encoder_error = static_cast<std::uint64_t>(encoder_error);
    axis.controller_error = static_cast<std::uint64_t>(controller_error);
  }

  telemetry.active = (front_active > 0.5) || (rear_active > 0.5);
  telemetry.i2t_latched = (front_i2t_latched > 0.5) || (rear_i2t_latched > 0.5);

  clear_backend_error();
  return true;
}

bool ODriveHardwareInterface::ensure_publishers()
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
  if (
    status_pub_ != nullptr && front_vbus_pub_ != nullptr && rear_vbus_pub_ != nullptr &&
    front_estop_pub_ != nullptr && rear_estop_pub_ != nullptr && diagnostics_pub_ != nullptr &&
    current_pubs_.size() == info_.joints.size() && temp_pubs_.size() == info_.joints.size() &&
    i2t_pubs_.size() == info_.joints.size() && connect_service_ != nullptr &&
    disconnect_service_ != nullptr && calibrate_service_ != nullptr &&
    preroll_service_ != nullptr && engage_service_ != nullptr && release_service_ != nullptr)
  {
    return true;
  }

  const auto node = get_node();
  if (node == nullptr)
  {
    RCLCPP_ERROR(get_logger(), "Failed to access the default hardware component node.");
    return false;
  }

  status_pub_ = node->create_publisher<std_msgs::msg::String>("odrive/status", 10);
  front_vbus_pub_ = node->create_publisher<std_msgs::msg::Float32>("odrive/front/vbus_voltage", 10);
  rear_vbus_pub_ = node->create_publisher<std_msgs::msg::Float32>("odrive/rear/vbus_voltage", 10);
  front_estop_pub_ =
    node->create_publisher<std_msgs::msg::Float32>("odrive/front/estop_voltage", 10);
  rear_estop_pub_ = node->create_publisher<std_msgs::msg::Float32>("odrive/rear/estop_voltage", 10);
  diagnostics_pub_ =
    node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

  current_pubs_.clear();
  temp_pubs_.clear();
  i2t_pubs_.clear();
  current_pubs_.reserve(info_.joints.size());
  temp_pubs_.reserve(info_.joints.size());
  i2t_pubs_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints)
  {
    const auto topic_base = std::string("odrive/") + joint.name;
    current_pubs_.push_back(
      node->create_publisher<std_msgs::msg::Float32>(topic_base + "/current", 10));
    temp_pubs_.push_back(
      node->create_publisher<std_msgs::msg::Float32>(topic_base + "/temperature", 10));
    i2t_pubs_.push_back(node->create_publisher<std_msgs::msg::Float32>(topic_base + "/i2t", 10));
  }

  const auto make_service_callback = [this](
                                       const std::string & command,
                                       const std::string & success_message, bool zero_commands,
                                       bool allow_backend_start)
  {
    return [this, command, success_message, zero_commands, allow_backend_start](
             const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      response->success = run_service_command(
        command, success_message, zero_commands, allow_backend_start, response->message);
    };
  };

  if (connect_service_ == nullptr)
  {
    connect_service_ = node->create_service<std_srvs::srv::Trigger>(
      "odrive/connect_driver",
      make_service_callback(
        "CONNECT", "ODrive driver connected. Motors remain released until engage_motors is called.",
        true, true));
  }
  if (disconnect_service_ == nullptr)
  {
    disconnect_service_ = node->create_service<std_srvs::srv::Trigger>(
      "odrive/disconnect_driver",
      make_service_callback(
        "DISCONNECT", "ODrive driver disconnected. Commands are ignored until reconnect.", true,
        false));
  }
  if (calibrate_service_ == nullptr)
  {
    calibrate_service_ = node->create_service<std_srvs::srv::Trigger>(
      "odrive/calibrate_motors",
      make_service_callback(
        "CALIBRATE", "ODrive calibration complete. Motors remain released.", true, false));
  }
  if (preroll_service_ == nullptr)
  {
    preroll_service_ = node->create_service<std_srvs::srv::Trigger>(
      "odrive/preroll_motors",
      make_service_callback(
        "PREROLL", "ODrive index search complete. Motors remain released.", true, false));
  }
  if (engage_service_ == nullptr)
  {
    engage_service_ = node->create_service<std_srvs::srv::Trigger>(
      "odrive/engage_motors",
      make_service_callback("ENGAGE", "ODrive motors engaged.", true, false));
  }
  if (release_service_ == nullptr)
  {
    release_service_ = node->create_service<std_srvs::srv::Trigger>(
      "odrive/release_motors",
      make_service_callback("RELEASE", "ODrive motors released.", true, false));
  }
  return true;
}

void ODriveHardwareInterface::poll_and_publish_status()
{
  std::lock_guard<std::recursive_mutex> lock(backend_mutex_);
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
    set_backend_error("STATUS transport failure");
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to read ODrive status from backend.");
    return;
  }

  ODriveTelemetry telemetry;
  if (!parse_status_response(response, telemetry))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to parse ODrive status response.");
    return;
  }

  publish_status(telemetry);
  last_status_publish_ = now;
  has_published_status_ = true;
}

void ODriveHardwareInterface::publish_status(const ODriveTelemetry & telemetry)
{
  const auto stamp = get_clock()->now();

  auto publish_float = [](const auto & publisher, double value)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(value);
    publisher->publish(msg);
  };

  publish_float(front_vbus_pub_, telemetry.front.vbus_voltage);
  publish_float(rear_vbus_pub_, telemetry.rear.vbus_voltage);
  publish_float(front_estop_pub_, telemetry.front.estop_voltage);
  publish_float(rear_estop_pub_, telemetry.rear.estop_voltage);

  for (size_t i = 0; i < telemetry.axes.size(); ++i)
  {
    publish_float(current_pubs_[i], telemetry.axes[i].current_a);
    publish_float(temp_pubs_[i], telemetry.axes[i].fet_temp_c);
    publish_float(i2t_pubs_[i], telemetry.axes[i].i2t);
  }

  std_msgs::msg::String status_msg;
  std::uint8_t overall_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string overall_message = telemetry.active ? "Running" : "Connected";
  if (telemetry.i2t_latched)
  {
    overall_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    overall_message = "ODrive commands disabled by i2t thermal protection";
  }
  if (!last_backend_error_.empty())
  {
    overall_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    overall_message = last_backend_error_;
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus overall_status;
  overall_status.name = "ODrive";
  overall_status.hardware_id = front_serial_number_ + "+" + rear_serial_number_;
  overall_status.values.push_back(make_key_value("front_serial_number", front_serial_number_));
  overall_status.values.push_back(make_key_value("rear_serial_number", rear_serial_number_));
  overall_status.values.push_back(make_key_value("active", telemetry.active ? "true" : "false"));
  overall_status.values.push_back(
    make_key_value("i2t_latched", telemetry.i2t_latched ? "true" : "false"));
  overall_status.values.push_back(
    make_key_value("front_vbus_voltage_v", format_double(telemetry.front.vbus_voltage)));
  overall_status.values.push_back(
    make_key_value("rear_vbus_voltage_v", format_double(telemetry.rear.vbus_voltage)));
  overall_status.values.push_back(
    make_key_value("front_estop_voltage_v", format_double(telemetry.front.estop_voltage)));
  overall_status.values.push_back(
    make_key_value("rear_estop_voltage_v", format_double(telemetry.rear.estop_voltage)));
  if (!last_backend_error_.empty())
  {
    overall_status.values.push_back(make_key_value("backend_error", last_backend_error_));
  }

  for (size_t i = 0; i < telemetry.axes.size(); ++i)
  {
    const auto & axis = telemetry.axes[i];
    std::uint8_t axis_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string axis_message = telemetry.active ? "Running" : "Connected";
    const bool has_errors = axis.axis_error != 0 || axis.motor_error != 0 ||
                            axis.encoder_error != 0 || axis.controller_error != 0;
    if (has_errors)
    {
      axis_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      axis_message = "ODrive errors present";
    }
    else if (telemetry.i2t_latched && axis.i2t >= i2t_resume_threshold_)
    {
      axis_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      axis_message = "i2t protection is disabling this axis";
    }
    else if (axis.i2t >= i2t_error_threshold_)
    {
      axis_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      axis_message = "i2t error threshold exceeded";
    }
    else if (axis.i2t >= i2t_warning_threshold_)
    {
      axis_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      axis_message = "i2t warning threshold exceeded";
    }
    else if (telemetry.active && axis.state != kAxisStateClosedLoopControl)
    {
      axis_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      axis_message = "Axis not in closed loop control";
    }
    else if (!telemetry.active && axis.state == kAxisStateIdle)
    {
      axis_message = "Idle";
    }

    overall_level = std::max(overall_level, axis_level);
    if (overall_message == "Running" && axis_level != diagnostic_msgs::msg::DiagnosticStatus::OK)
    {
      overall_message = axis_message;
    }

    diagnostic_msgs::msg::DiagnosticStatus axis_status;
    axis_status.name = "ODrive " + info_.joints[i].name;
    axis_status.hardware_id =
      (i < 2 ? front_serial_number_ : rear_serial_number_) + ":" + info_.joints[i].name;
    axis_status.level = axis_level;
    axis_status.message = axis_message;
    axis_status.values.push_back(make_key_value("joint", info_.joints[i].name));
    axis_status.values.push_back(make_key_value("state", axis_state_name(axis.state)));
    axis_status.values.push_back(make_key_value("current_a", format_double(axis.current_a)));
    axis_status.values.push_back(
      make_key_value("fet_temperature_c", format_double(axis.fet_temp_c)));
    axis_status.values.push_back(make_key_value("i2t", format_double(axis.i2t)));
    axis_status.values.push_back(make_key_value("axis_error", format_hex(axis.axis_error)));
    axis_status.values.push_back(make_key_value("motor_error", format_hex(axis.motor_error)));
    axis_status.values.push_back(make_key_value("encoder_error", format_hex(axis.encoder_error)));
    axis_status.values.push_back(
      make_key_value("controller_error", format_hex(axis.controller_error)));
    diagnostics.status.push_back(axis_status);
  }

  overall_status.level = overall_level;
  overall_status.message = overall_message;
  diagnostics.status.insert(diagnostics.status.begin(), overall_status);
  diagnostics_pub_->publish(diagnostics);

  status_msg.data = overall_message;
  status_pub_->publish(status_msg);

  if (overall_message != last_status_summary_)
  {
    if (overall_level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR)
    {
      RCLCPP_ERROR(get_logger(), "ODrive status changed: %s", overall_message.c_str());
    }
    else if (overall_level == diagnostic_msgs::msg::DiagnosticStatus::WARN)
    {
      RCLCPP_WARN(get_logger(), "ODrive status changed: %s", overall_message.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "ODrive status changed: %s", overall_message.c_str());
    }
    last_status_summary_ = overall_message;
  }
}

void ODriveHardwareInterface::reset_status_publish_state()
{
  has_published_status_ = false;
  last_status_publish_ = std::chrono::steady_clock::time_point{};
  last_status_summary_.clear();
  last_backend_error_.clear();
}

bool ODriveHardwareInterface::response_has_error(const std::string & response) const
{
  return response.rfind("ERROR", 0) == 0;
}

void ODriveHardwareInterface::set_backend_error(const std::string & error)
{
  last_backend_error_ = error;
}

void ODriveHardwareInterface::clear_backend_error() { last_backend_error_.clear(); }

}  // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::SystemInterface)
