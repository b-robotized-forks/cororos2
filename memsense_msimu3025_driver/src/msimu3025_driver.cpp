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

namespace
{
constexpr int kPacketBytes = 54;
constexpr speed_t kBaudRate = B460800;

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
    magnetic_topic_ = declare_parameter<std::string>("magnetic_topic", "/msimu3025_magnetic");
    temperature_topic_ =
      declare_parameter<std::string>("temperature_topic", "/msimu3025_temperature");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
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
  void initialize_messages()
  {
    imu_msg_.orientation_covariance[0] = -1.0;
    imu_msg_.header.frame_id = frame_id_;
    imu_msg_.angular_velocity_covariance[0] = 0.62 * 2.0 * M_PI / 360.0 / 3600.0;
    imu_msg_.angular_velocity_covariance[4] = 0.56 * 2.0 * M_PI / 360.0 / 3600.0;
    imu_msg_.angular_velocity_covariance[8] = 0.80 * 2.0 * M_PI / 360.0 / 3600.0;
    imu_msg_.linear_acceleration_covariance[0] = 2.6 * (1e-6) * 9.80665;
    imu_msg_.linear_acceleration_covariance[4] = 2.6 * (1e-6) * 9.80665;
    imu_msg_.linear_acceleration_covariance[8] = 6.7 * (1e-6) * 9.80665;

    magnetic_msg_.header.frame_id = frame_id_;
    magnetic_msg_.magnetic_field_covariance[0] = 0.0016 / 10000.0;
    magnetic_msg_.magnetic_field_covariance[4] = 0.0016 / 10000.0;
    magnetic_msg_.magnetic_field_covariance[8] = 0.0016 / 10000.0;

    temperature_msg_.header.frame_id = frame_id_;
    temperature_msg_.variance = 1.5;
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
            imu_msg_.linear_acceleration.x =
              9.80665 * read_be_float(bytes_, static_cast<size_t>(i + 2));
            imu_msg_.linear_acceleration.y =
              9.80665 * read_be_float(bytes_, static_cast<size_t>(i + 6));
            imu_msg_.linear_acceleration.z =
              9.80665 * read_be_float(bytes_, static_cast<size_t>(i + 10));
          }
          break;
        case 0x82:
          if (field_len == 0x0c)
          {
            imu_msg_.angular_velocity.x =
              2.0 * M_PI * read_be_float(bytes_, static_cast<size_t>(i + 2)) / 360.0;
            imu_msg_.angular_velocity.y =
              2.0 * M_PI * read_be_float(bytes_, static_cast<size_t>(i + 6)) / 360.0;
            imu_msg_.angular_velocity.z =
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
      std::fabs(imu_msg_.angular_velocity.x) >= 50.0 ||
      std::fabs(imu_msg_.angular_velocity.y) >= 50.0 ||
      std::fabs(imu_msg_.angular_velocity.z) >= 50.0 ||
      std::fabs(imu_msg_.linear_acceleration.x) >= 500.0 ||
      std::fabs(imu_msg_.linear_acceleration.y) >= 500.0 ||
      std::fabs(imu_msg_.linear_acceleration.z) >= 500.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Discarding implausible Memsense packet");
      return;
    }

    const auto stamp = now();
    imu_msg_.header.stamp = stamp;
    magnetic_msg_.header.stamp = stamp;
    temperature_msg_.header.stamp = stamp;

    imu_pub_->publish(imu_msg_);
    magnetic_pub_->publish(magnetic_msg_);
    temperature_pub_->publish(temperature_msg_);
  }

  std::string serial_path_;
  std::string frame_id_;
  std::string imu_topic_;
  std::string magnetic_topic_;
  std::string temperature_topic_;

  int tty_fd_;
  int bytes_avail_;
  std::array<uint8_t, kPacketBytes> bytes_{};

  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::MagneticField magnetic_msg_;
  sensor_msgs::msg::Temperature temperature_msg_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
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
