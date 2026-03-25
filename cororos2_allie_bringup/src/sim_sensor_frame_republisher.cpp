// Copyright (c) 2026, b-robotized Group

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
  SimSensorFrameRepublisher() : Node("sim_sensor_frame_republisher")
  {
    add_republisher<sensor_msgs::msg::LaserScan>(
      "/allie/raw/lidar/scan", "/allie/lidar/scan", "ouster_link");
    add_republisher<sensor_msgs::msg::PointCloud2>(
      "/allie/raw/lidar/points", "/allie/lidar/points", "ouster_link");
    add_republisher<sensor_msgs::msg::Image>(
      "/allie/raw/rgbd_front/image_raw", "/allie/rgbd_front/image_raw", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::Image>(
      "/allie/raw/rgbd_front/depth_image", "/allie/rgbd_front/depth_image", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::PointCloud2>(
      "/allie/raw/rgbd_front/points", "/allie/rgbd_front/points", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::CameraInfo>(
      "/allie/raw/rgbd_front/camera_info", "/allie/rgbd_front/camera_info", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::Image>(
      "/allie/raw/camera_front/image_raw", "/allie/camera_front/image_raw", "camera_front_link");
    add_republisher<sensor_msgs::msg::CameraInfo>(
      "/allie/raw/camera_front/camera_info", "/allie/camera_front/camera_info",
      "camera_front_link");
    add_republisher<sensor_msgs::msg::Image>(
      "/allie/raw/camera_rear/image_raw", "/allie/camera_rear/image_raw", "camera_rear_link");
    add_republisher<sensor_msgs::msg::CameraInfo>(
      "/allie/raw/camera_rear/camera_info", "/allie/camera_rear/camera_info", "camera_rear_link");
    add_republisher<sensor_msgs::msg::Image>(
      "/allie/raw/camera_left/image_raw", "/allie/camera_left/image_raw", "camera_left_link");
    add_republisher<sensor_msgs::msg::CameraInfo>(
      "/allie/raw/camera_left/camera_info", "/allie/camera_left/camera_info", "camera_left_link");
    add_republisher<sensor_msgs::msg::Image>(
      "/allie/raw/camera_right/image_raw", "/allie/camera_right/image_raw", "camera_right_link");
    add_republisher<sensor_msgs::msg::CameraInfo>(
      "/allie/raw/camera_right/camera_info", "/allie/camera_right/camera_info",
      "camera_right_link");
    add_republisher<sensor_msgs::msg::Imu>(
      "/allie/raw/imu/data", "/allie/imu/data", "imu_sensor_link");
    add_republisher<sensor_msgs::msg::NavSatFix>(
      "/allie/raw/gps/fix", "/allie/gps/fix", "gps_link");
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
