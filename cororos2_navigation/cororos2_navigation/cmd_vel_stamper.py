#!/usr/bin/env python3

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node


class CmdVelStamper(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_stamper")
        input_topic = self.declare_parameter("input_topic", "/cmd_vel").value
        output_topic = self.declare_parameter("output_topic", "/diff_drive_controller/cmd_vel").value
        frame_id = self.declare_parameter("frame_id", "").value

        self._frame_id = str(frame_id)
        self._publisher = self.create_publisher(TwistStamped, str(output_topic), 10)
        self._subscription = self.create_subscription(Twist, str(input_topic), self._handle_cmd_vel, 10)

    def _handle_cmd_vel(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = msg
        self._publisher.publish(stamped)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
