// Copyright (c) 2026, b-robotized Group

#include "odrive_hardware_interface/odrive_hardware_interface.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cmath>
#include <csignal>
#include <sstream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
}

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
  if (!start_backend())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

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

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    expect_ok("DEACTIVATE");
    stop_backend();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ODriveHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::string response;
  if (!send_command("READ", response))
  {
    return hardware_interface::return_type::ERROR;
  }
  return parse_state_response(response) ? hardware_interface::return_type::OK
                                        : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type ODriveHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
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
      std::to_string(connect_timeout_).c_str(), static_cast<char *>(nullptr));
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
    perror("fgets");
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
    return false;
  }
  if (response != "OK")
  {
    fprintf(stderr, "Command '%s' failed with response '%s'\n", command.c_str(), response.c_str());
    return false;
  }
  return true;
}

bool ODriveHardwareInterface::parse_state_response(const std::string & response)
{
  std::istringstream stream(response);
  std::string token;
  stream >> token;
  if (token != "STATE")
  {
    fprintf(stderr, "Unexpected backend state response: '%s'\n", response.c_str());
    return false;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    double rotations = 0.0;
    double rotations_per_second = 0.0;
    if (!(stream >> rotations >> rotations_per_second))
    {
      fprintf(stderr, "Failed to parse backend state response: '%s'\n", response.c_str());
      return false;
    }
    hw_positions_[i] = rotations * kTwoPi;
    hw_velocities_[i] = rotations_per_second * kTwoPi;
  }

  return true;
}

}  // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::SystemInterface)
