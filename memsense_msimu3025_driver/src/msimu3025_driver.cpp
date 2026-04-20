// Copyright (c) 2026, b-robotized Group
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr int kPacketBytes = 54;
constexpr speed_t kBaudRate = B460800;
constexpr double kGravityAcceleration = 9.80665;
constexpr double kMinVectorNorm = 1.0e-6;

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double normalize_angle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

double vector_norm(double x, double y, double z) { return std::sqrt((x * x) + (y * y) + (z * z)); }

bool fletcher_valid(const std::array<uint8_t, kPacketBytes> & msg)
{
  int f1 = 0;
  int f2 = 0;
  for (int i = 0; i < (kPacketBytes - 2); ++i)
  {
    f1 += msg[static_cast<size_t>(i)];
    f2 += f1;
  }
  f1 %= 256;
  f2 %= 256;
  return f1 == msg[kPacketBytes - 2] && f2 == msg[kPacketBytes - 1];
}

float read_be_float(const std::array<uint8_t, kPacketBytes> & msg, size_t offset)
{
  const uint32_t raw =
    (static_cast<uint32_t>(msg[offset]) << 24) | (static_cast<uint32_t>(msg[offset + 1]) << 16) |
    (static_cast<uint32_t>(msg[offset + 2]) << 8) | static_cast<uint32_t>(msg[offset + 3]);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(float));
  return value;
}
}  // namespace

class Msimu3025Driver : public rclcpp::Node
{
public:
  Msimu3025Driver() : Node("msimu3025_driver"), tty_fd_(-1), bytes_avail_(0)
  {
    serial_path_ = declare_parameter<std::string>(
      "serial_path_msimu3025",
      "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_sensor_link");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/msimu3025_raw");
    filtered_imu_topic_ = declare_parameter<std::string>("filtered_imu_topic", "data");
    magnetic_topic_ = declare_parameter<std::string>("magnetic_topic", "/msimu3025_magnetic");
    temperature_topic_ =
      declare_parameter<std::string>("temperature_topic", "/msimu3025_temperature");
    publish_orientation_ = declare_parameter<bool>("publish_orientation", true);
    magnetic_declination_radians_ = declare_parameter<double>("magnetic_declination_radians", 0.0);
    orientation_smoothing_gain_ = declare_parameter<double>("orientation_smoothing_gain", 0.2);
    acceleration_magnitude_tolerance_ =
      declare_parameter<double>("acceleration_magnitude_tolerance", 3.0);
    orientation_roll_pitch_variance_ =
      declare_parameter<double>("orientation_roll_pitch_variance", 0.05);
    orientation_yaw_variance_ = declare_parameter<double>("orientation_yaw_variance", 0.1);

    raw_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    if (publish_orientation_)
    {
      filtered_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(filtered_imu_topic_, 10);
    }
    magnetic_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(magnetic_topic_, 10);
    temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>(temperature_topic_, 10);

    initialize_messages();

    reconnect_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&Msimu3025Driver::ensure_serial_connection, this));
    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&Msimu3025Driver::read_serial, this));
  }

  ~Msimu3025Driver() override { close_serial(); }

private:
  static void set_quaternion_message(
    const tf2::Quaternion & orientation, geometry_msgs::msg::Quaternion & orientation_msg)
  {
    orientation_msg.x = orientation.x();
    orientation_msg.y = orientation.y();
    orientation_msg.z = orientation.z();
    orientation_msg.w = orientation.w();
  }

  void initialize_messages()
  {
    raw_imu_msg_.orientation_covariance[0] = -1.0;
    raw_imu_msg_.header.frame_id = frame_id_;
    raw_imu_msg_.angular_velocity_covariance[0] = 0.62 * 2.0 * M_PI / 360.0 / 3600.0;
    raw_imu_msg_.angular_velocity_covariance[4] = 0.56 * 2.0 * M_PI / 360.0 / 3600.0;
    raw_imu_msg_.angular_velocity_covariance[8] = 0.80 * 2.0 * M_PI / 360.0 / 3600.0;
    raw_imu_msg_.linear_acceleration_covariance[0] = 2.6 * (1e-6) * kGravityAcceleration;
    raw_imu_msg_.linear_acceleration_covariance[4] = 2.6 * (1e-6) * kGravityAcceleration;
    raw_imu_msg_.linear_acceleration_covariance[8] = 6.7 * (1e-6) * kGravityAcceleration;

    filtered_imu_msg_ = raw_imu_msg_;
    filtered_imu_msg_.orientation.x = 0.0;
    filtered_imu_msg_.orientation.y = 0.0;
    filtered_imu_msg_.orientation.z = 0.0;
    filtered_imu_msg_.orientation.w = 1.0;
    filtered_imu_msg_.orientation_covariance[0] = orientation_roll_pitch_variance_;
    filtered_imu_msg_.orientation_covariance[4] = orientation_roll_pitch_variance_;
    filtered_imu_msg_.orientation_covariance[8] = orientation_yaw_variance_;

    magnetic_msg_.header.frame_id = frame_id_;
    magnetic_msg_.magnetic_field_covariance[0] = 0.0016 / 10000.0;
    magnetic_msg_.magnetic_field_covariance[4] = 0.0016 / 10000.0;
    magnetic_msg_.magnetic_field_covariance[8] = 0.0016 / 10000.0;

    temperature_msg_.header.frame_id = frame_id_;
    temperature_msg_.variance = 1.5;
  }

  bool estimate_orientation_from_accel_and_mag(tf2::Quaternion & measured_orientation)
  {
    const double accel_x = raw_imu_msg_.linear_acceleration.x;
    const double accel_y = raw_imu_msg_.linear_acceleration.y;
    const double accel_z = raw_imu_msg_.linear_acceleration.z;
    const double accel_norm = vector_norm(accel_x, accel_y, accel_z);
    if (
      !std::isfinite(accel_norm) || accel_norm < kMinVectorNorm ||
      std::fabs(accel_norm - kGravityAcceleration) > acceleration_magnitude_tolerance_)
    {
      return false;
    }

    const double accel_x_unit = accel_x / accel_norm;
    const double accel_y_unit = accel_y / accel_norm;
    const double accel_z_unit = accel_z / accel_norm;

    const double roll = std::atan2(accel_y_unit, accel_z_unit);
    const double pitch = std::atan2(
      -accel_x_unit, std::sqrt((accel_y_unit * accel_y_unit) + (accel_z_unit * accel_z_unit)));

    double yaw = last_yaw_radians_;
    const double mag_x = magnetic_msg_.magnetic_field.x;
    const double mag_y = magnetic_msg_.magnetic_field.y;
    const double mag_z = magnetic_msg_.magnetic_field.z;
    const double mag_norm = vector_norm(mag_x, mag_y, mag_z);
    if (std::isfinite(mag_norm) && mag_norm > kMinVectorNorm)
    {
      const double mag_x_unit = mag_x / mag_norm;
      const double mag_y_unit = mag_y / mag_norm;
      const double mag_z_unit = mag_z / mag_norm;

      const double sin_roll = std::sin(roll);
      const double cos_roll = std::cos(roll);
      const double sin_pitch = std::sin(pitch);
      const double cos_pitch = std::cos(pitch);
      const double mag_x_horizontal = (mag_x_unit * cos_pitch) +
                                      (mag_y_unit * sin_roll * sin_pitch) +
                                      (mag_z_unit * cos_roll * sin_pitch);
      const double mag_y_horizontal = (mag_y_unit * cos_roll) - (mag_z_unit * sin_roll);

      if (
        std::fabs(mag_x_horizontal) > kMinVectorNorm ||
        std::fabs(mag_y_horizontal) > kMinVectorNorm)
      {
        yaw = normalize_angle(
          std::atan2(-mag_y_horizontal, mag_x_horizontal) + magnetic_declination_radians_);
        last_yaw_radians_ = yaw;
        yaw_initialized_ = true;
      }
    }
    else if (!yaw_initialized_)
    {
      yaw = 0.0;
    }

    measured_orientation.setRPY(roll, pitch, yaw);
    return true;
  }

  void update_filtered_imu_orientation()
  {
    filtered_imu_msg_ = raw_imu_msg_;
    filtered_imu_msg_.orientation_covariance[0] = -1.0;

    if (!publish_orientation_)
    {
      return;
    }

    tf2::Quaternion measured_orientation;
    if (estimate_orientation_from_accel_and_mag(measured_orientation))
    {
      if (!orientation_initialized_)
      {
        orientation_estimate_ = measured_orientation;
        orientation_initialized_ = true;
      }
      else
      {
        orientation_estimate_ = orientation_estimate_.slerp(
          measured_orientation, clamp(orientation_smoothing_gain_, 0.0, 1.0));
      }
      orientation_estimate_.normalize();
    }

    if (!orientation_initialized_)
    {
      return;
    }

    set_quaternion_message(orientation_estimate_, filtered_imu_msg_.orientation);
    filtered_imu_msg_.orientation_covariance[0] = orientation_roll_pitch_variance_;
    filtered_imu_msg_.orientation_covariance[4] = orientation_roll_pitch_variance_;
    filtered_imu_msg_.orientation_covariance[8] = orientation_yaw_variance_;
  }

  void ensure_serial_connection()
  {
    if (tty_fd_ >= 0)
    {
      return;
    }

    tty_fd_ = open(serial_path_.c_str(), O_RDWR | O_NOCTTY);
    if (tty_fd_ < 0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to open Memsense serial port '%s': %s",
        serial_path_.c_str(), std::strerror(errno));
      return;
    }

    struct serial_struct serial_cfg;
    if (ioctl(tty_fd_, TIOCGSERIAL, &serial_cfg) == 0)
    {
      serial_cfg.flags |= ASYNC_LOW_LATENCY;
      ioctl(tty_fd_, TIOCSSERIAL, &serial_cfg);
    }

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = CS8 | CREAD | CLOCAL;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;
    cfsetospeed(&tio, kBaudRate);
    cfsetispeed(&tio, kBaudRate);
    tcsetattr(tty_fd_, TCSANOW, &tio);

    bytes_avail_ = 0;
    RCLCPP_INFO(get_logger(), "Opened Memsense serial port: %s", serial_path_.c_str());
  }

  void close_serial()
  {
    if (tty_fd_ >= 0)
    {
      close(tty_fd_);
      tty_fd_ = -1;
    }
    bytes_avail_ = 0;
  }

  void read_serial()
  {
    if (tty_fd_ < 0)
    {
      return;
    }

    uint8_t c = 0;
    const auto read_count = ::read(tty_fd_, &c, 1);
    if (read_count < 0)
    {
      if (errno != EAGAIN && errno != EWOULDBLOCK)
      {
        RCLCPP_ERROR(get_logger(), "Read failed on Memsense serial port: %s", std::strerror(errno));
        close_serial();
      }
      return;
    }

    if (read_count == 0)
    {
      return;
    }

    bytes_[static_cast<size_t>(bytes_avail_)] = c;
    ++bytes_avail_;

    if (bytes_avail_ < kPacketBytes)
    {
      return;
    }

    if (
      bytes_[0] == 0xA5 && bytes_[1] == 0xA5 && bytes_[2] == 0xA2 && bytes_[3] == kPacketBytes - 6)
    {
      if (fletcher_valid(bytes_))
      {
        parse_packet();
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Invalid Memsense Fletcher checksum");
      }
      bytes_avail_ = 0;
      return;
    }

    --bytes_avail_;
    for (int i = 0; i < bytes_avail_; ++i)
    {
      bytes_[static_cast<size_t>(i)] = bytes_[static_cast<size_t>(i + 1)];
    }
  }

  void parse_packet()
  {
    for (int i = 4; i < kPacketBytes; i += bytes_[static_cast<size_t>(i + 1)] + 2)
    {
      const auto field_id = bytes_[static_cast<size_t>(i)];
      const auto field_len = bytes_[static_cast<size_t>(i + 1)];
      switch (field_id)
      {
        case 0x81:
          if (field_len == 0x0c)
          {
            raw_imu_msg_.linear_acceleration.x =
              kGravityAcceleration * read_be_float(bytes_, static_cast<size_t>(i + 2));
            raw_imu_msg_.linear_acceleration.y =
              kGravityAcceleration * read_be_float(bytes_, static_cast<size_t>(i + 6));
            raw_imu_msg_.linear_acceleration.z =
              kGravityAcceleration * read_be_float(bytes_, static_cast<size_t>(i + 10));
          }
          break;
        case 0x82:
          if (field_len == 0x0c)
          {
            raw_imu_msg_.angular_velocity.x =
              2.0 * M_PI * read_be_float(bytes_, static_cast<size_t>(i + 2)) / 360.0;
            raw_imu_msg_.angular_velocity.y =
              2.0 * M_PI * read_be_float(bytes_, static_cast<size_t>(i + 6)) / 360.0;
            raw_imu_msg_.angular_velocity.z =
              2.0 * M_PI * read_be_float(bytes_, static_cast<size_t>(i + 10)) / 360.0;
          }
          break;
        case 0x83:
          if (field_len == 0x0c)
          {
            magnetic_msg_.magnetic_field.x =
              read_be_float(bytes_, static_cast<size_t>(i + 2)) / 10000.0;
            magnetic_msg_.magnetic_field.y =
              read_be_float(bytes_, static_cast<size_t>(i + 6)) / 10000.0;
            magnetic_msg_.magnetic_field.z =
              read_be_float(bytes_, static_cast<size_t>(i + 10)) / 10000.0;
          }
          break;
        case 0x87:
          if (field_len == 0x04)
          {
            temperature_msg_.temperature = read_be_float(bytes_, static_cast<size_t>(i + 2));
          }
          break;
        default:
          break;
      }
    }

    if (
      std::fabs(raw_imu_msg_.angular_velocity.x) >= 50.0 ||
      std::fabs(raw_imu_msg_.angular_velocity.y) >= 50.0 ||
      std::fabs(raw_imu_msg_.angular_velocity.z) >= 50.0 ||
      std::fabs(raw_imu_msg_.linear_acceleration.x) >= 500.0 ||
      std::fabs(raw_imu_msg_.linear_acceleration.y) >= 500.0 ||
      std::fabs(raw_imu_msg_.linear_acceleration.z) >= 500.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Discarding implausible Memsense packet");
      return;
    }

    update_filtered_imu_orientation();

    const auto stamp = now();
    raw_imu_msg_.header.stamp = stamp;
    filtered_imu_msg_.header.stamp = stamp;
    magnetic_msg_.header.stamp = stamp;
    temperature_msg_.header.stamp = stamp;

    raw_imu_pub_->publish(raw_imu_msg_);
    if (publish_orientation_)
    {
      filtered_imu_pub_->publish(filtered_imu_msg_);
    }
    magnetic_pub_->publish(magnetic_msg_);
    temperature_pub_->publish(temperature_msg_);
  }

  std::string serial_path_;
  std::string frame_id_;
  std::string imu_topic_;
  std::string filtered_imu_topic_;
  std::string magnetic_topic_;
  std::string temperature_topic_;
  bool publish_orientation_;
  double magnetic_declination_radians_;
  double orientation_smoothing_gain_;
  double acceleration_magnitude_tolerance_;
  double orientation_roll_pitch_variance_;
  double orientation_yaw_variance_;

  int tty_fd_;
  int bytes_avail_;
  std::array<uint8_t, kPacketBytes> bytes_{};

  sensor_msgs::msg::Imu raw_imu_msg_;
  sensor_msgs::msg::Imu filtered_imu_msg_;
  sensor_msgs::msg::MagneticField magnetic_msg_;
  sensor_msgs::msg::Temperature temperature_msg_;
  tf2::Quaternion orientation_estimate_{0.0, 0.0, 0.0, 1.0};
  bool orientation_initialized_ = false;
  double last_yaw_radians_ = 0.0;
  bool yaw_initialized_ = false;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;

  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Msimu3025Driver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
