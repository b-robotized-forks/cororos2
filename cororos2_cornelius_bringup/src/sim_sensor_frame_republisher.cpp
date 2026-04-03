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
      "/cornelius/raw/lidar/scan", "/cornelius/lidar/scan", "ouster_link");
    add_republisher<sensor_msgs::msg::PointCloud2>(
      "/cornelius/raw/lidar/points", "/cornelius/lidar/points", "ouster_link");
    add_republisher<sensor_msgs::msg::Image>(
      "/cornelius/raw/rgbd_front/image_raw", "/cornelius/rgbd_front/image_raw", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::Image>(
      "/cornelius/raw/rgbd_front/depth_image", "/cornelius/rgbd_front/depth_image",
      "rgbd_front_link");
    add_republisher<sensor_msgs::msg::PointCloud2>(
      "/cornelius/raw/rgbd_front/points", "/cornelius/rgbd_front/points", "rgbd_front_link");
    add_republisher<sensor_msgs::msg::CameraInfo>(
      "/cornelius/raw/rgbd_front/camera_info", "/cornelius/rgbd_front/camera_info",
      "rgbd_front_link");
    add_republisher<sensor_msgs::msg::Imu>(
      "/cornelius/raw/imu/data", "/cornelius/imu/data", "imu_sensor_link");
    add_republisher<sensor_msgs::msg::NavSatFix>(
      "/cornelius/raw/gps/fix", "/cornelius/gps/fix", "gps_link");
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
