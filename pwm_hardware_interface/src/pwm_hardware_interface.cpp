// Copyright (c) 2026, b-robotized Group

#include "pwm_hardware_interface/pwm_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
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

  device_path_ = hardware_parameters.count("device_path") != 0
                   ? hardware_parameters.at("device_path")
                   : "/dev/serial/by-id/"
                     "usb-Pololu_Corporation_Pololu_Micro_Maestro_6-Servo_Controller_00467337-if00";
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

  channels_[0] = hardware_parameters.count("channel_fl") != 0
                   ? static_cast<uint8_t>(std::stoi(hardware_parameters.at("channel_fl")))
                   : 0;
  channels_[1] = hardware_parameters.count("channel_fr") != 0
                   ? static_cast<uint8_t>(std::stoi(hardware_parameters.at("channel_fr")))
                   : 1;
  channels_[2] = hardware_parameters.count("channel_rl") != 0
                   ? static_cast<uint8_t>(std::stoi(hardware_parameters.at("channel_rl")))
                   : 2;
  channels_[3] = hardware_parameters.count("channel_rr") != 0
                   ? static_cast<uint8_t>(std::stoi(hardware_parameters.at("channel_rr")))
                   : 3;

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

hardware_interface::CallbackReturn PwmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (!start_backend())
  {
    RCLCPP_ERROR(get_logger(), "Failed to start PWM backend.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!write_neutral())
  {
    RCLCPP_ERROR(get_logger(), "Failed to write neutral PWM during configure.");
    stop_backend();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface configured.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (!backend_running_)
  {
    RCLCPP_ERROR(get_logger(), "PWM backend is not running.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!write_neutral())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (backend_running_)
  {
    write_neutral();
  }

  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    write_neutral();
  }
  stop_backend();
  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    write_neutral();
  }
  stop_backend();
  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface shut down.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    write_neutral();
  }
  stop_backend();
  RCLCPP_ERROR(get_logger(), "Allie PWM hardware interface entered error state.");
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
  if (!backend_running_ || maestro_fd_ < 0)
  {
    RCLCPP_ERROR(get_logger(), "PWM backend is not running.");
    return hardware_interface::return_type::ERROR;
  }

  const int16_t front_left_pwm = speed_to_pwm(hw_commands_[0], invert_left_);
  const int16_t front_right_pwm = speed_to_pwm(hw_commands_[1], invert_right_);
  const int16_t rear_left_pwm = speed_to_pwm(hw_commands_[2], invert_left_);
  const int16_t rear_right_pwm = speed_to_pwm(hw_commands_[3], invert_right_);

  last_pwm_ = {front_left_pwm, front_right_pwm, rear_left_pwm, rear_right_pwm};

  if (
    !write_maestro_target(channels_[0], static_cast<uint16_t>(front_left_pwm * 4)) ||
    !write_maestro_target(channels_[1], static_cast<uint16_t>(front_right_pwm * 4)) ||
    !write_maestro_target(channels_[2], static_cast<uint16_t>(rear_left_pwm * 4)) ||
    !write_maestro_target(channels_[3], static_cast<uint16_t>(rear_right_pwm * 4)))
  {
    return hardware_interface::return_type::ERROR;
  }

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

bool PwmHardwareInterface::start_backend()
{
  if (backend_running_)
  {
    return true;
  }

  if (!open_maestro())
  {
    return false;
  }

  if (!configure_maestro_serial())
  {
    close_maestro();
    return false;
  }

  backend_running_ = true;
  return true;
}

void PwmHardwareInterface::stop_backend()
{
  if (maestro_fd_ >= 0)
  {
    close_maestro();
  }
  backend_running_ = false;
}

bool PwmHardwareInterface::open_maestro()
{
  maestro_fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY);
  if (maestro_fd_ < 0)
  {
    RCLCPP_ERROR(
      get_logger(), "Failed to open Maestro device '%s': %s", device_path_.c_str(),
      std::strerror(errno));
    return false;
  }

  return true;
}

void PwmHardwareInterface::close_maestro()
{
  if (maestro_fd_ >= 0)
  {
    ::close(maestro_fd_);
    maestro_fd_ = -1;
  }
}

bool PwmHardwareInterface::configure_maestro_serial()
{
  struct termios options;
  if (tcgetattr(maestro_fd_, &options) < 0)
  {
    RCLCPP_ERROR(get_logger(), "Failed to get serial attributes for Maestro.");
    return false;
  }

  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  if (tcsetattr(maestro_fd_, TCSANOW, &options) < 0)
  {
    RCLCPP_ERROR(get_logger(), "Failed to set serial attributes for Maestro.");
    return false;
  }

  return true;
}

bool PwmHardwareInterface::write_maestro_target(uint8_t channel, uint16_t target)
{
  const uint8_t command[] = {
    0x84, channel, static_cast<uint8_t>(target & 0x7F), static_cast<uint8_t>((target >> 7) & 0x7F)};

  const auto bytes_written = ::write(maestro_fd_, command, sizeof(command));
  if (bytes_written != static_cast<ssize_t>(sizeof(command)))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write Maestro target for channel %u.", channel);
    return false;
  }

  return true;
}

bool PwmHardwareInterface::write_neutral()
{
  const uint16_t neutral_target = static_cast<uint16_t>(pwm_neutral_ * 4);

  return write_maestro_target(channels_[0], neutral_target) &&
         write_maestro_target(channels_[1], neutral_target) &&
         write_maestro_target(channels_[2], neutral_target) &&
         write_maestro_target(channels_[3], neutral_target);
}

}  // namespace pwm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  pwm_hardware_interface::PwmHardwareInterface, hardware_interface::SystemInterface)
