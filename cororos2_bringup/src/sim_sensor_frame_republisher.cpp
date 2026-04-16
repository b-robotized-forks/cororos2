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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class SimSensorFrameRepublisher : public rclcpp::Node
{
public:
  SimSensorFrameRepublisher()
  : Node("sim_sensor_frame_republisher"),
    robot_name_(declare_parameter<std::string>("robot_name", "allie")),
    lidar_frame_(declare_parameter<std::string>("lidar_frame", "ouster_link"))
  {
    const std::string ns = "/" + robot_name_;

    add_republisher<sensor_msgs::msg::LaserScan>(
      ns + "/raw/lidar/scan", ns + "/lidar/scan", lidar_frame_);
    add_republisher<sensor_msgs::msg::PointCloud2>(
      ns + "/raw/lidar/points", ns + "/lidar/points", lidar_frame_);
    add_republisher<sensor_msgs::msg::Image>(
      ns + "/raw/rgbd_front/image_raw", ns + "/rgbd_front/image_raw", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::Image>(
      ns + "/raw/rgbd_front/depth_image", ns + "/rgbd_front/depth_image", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::PointCloud2>(
      ns + "/raw/rgbd_front/points", ns + "/rgbd_front/points", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::CameraInfo>(
      ns + "/raw/rgbd_front/camera_info", ns + "/rgbd_front/camera_info", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::Imu>(
      ns + "/raw/imu/data", ns + "/imu/data", "imu_sensor_link");
    add_republisher<sensor_msgs::msg::NavSatFix>(ns + "/raw/gps/fix", ns + "/gps/fix", "gps_link");
  }

private:
  template <typename MessageT>
  void add_republisher(
    const std::string & input_topic, const std::string & output_topic, const std::string & frame_id)
  {
    auto publisher = create_publisher<MessageT>(output_topic, 10);
    auto subscription = create_subscription<MessageT>(
      input_topic, 10,
      [publisher, frame_id](const typename MessageT::SharedPtr msg)
      {
        auto output = *msg;
        output.header.frame_id = frame_id;
        publisher->publish(output);
      });

    subscriptions_.push_back(subscription);
    publishers_.push_back(publisher);
  }

  std::string robot_name_;
  std::string lidar_frame_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSensorFrameRepublisher>());
  rclcpp::shutdown();
  return 0;
}
