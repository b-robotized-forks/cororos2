#!/usr/bin/env python3

from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node


class ScanRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("scan_republisher")
        input_topic = self.declare_parameter("input_topic", "/scan").value
        output_topic = self.declare_parameter("output_topic", "/laser_scan").value
        frame_id = self.declare_parameter("frame_id", "").value

        self._frame_id = str(frame_id)
        self._publisher = self.create_publisher(LaserScan, str(output_topic), 10)
        self._subscription = self.create_subscription(LaserScan, str(input_topic), self._handle_scan, 10)

    def _handle_scan(self, msg: LaserScan) -> None:
        output = LaserScan()
        output.header = msg.header
        if self._frame_id:
            output.header.frame_id = self._frame_id
        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment
        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time
        output.range_min = msg.range_min
        output.range_max = msg.range_max
        output.ranges = list(msg.ranges)
        output.intensities = list(msg.intensities)
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
