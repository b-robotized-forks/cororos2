from nav_msgs.msg import Odometry
import rclpy
from rclpy.logging import get_logger

from .roboclaw_common import DriveCommand, EncoderOdom, RoboclawBase
from . import roboclaw_protocol as roboclaw


class RoboclawEncoderNode(RoboclawBase):
    def __init__(self) -> None:
        super().__init__("roboclaw_driver_encoder")
        self.declare_parameter("ticks_per_meter", 4342.2)
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("encoder_poll_rate_hz", 10.0)
        self.declare_parameter("m1_encoder_sign", 1)
        self.declare_parameter("m2_encoder_sign", 1)

        self.ticks_per_meter = self.get_parameter("ticks_per_meter").get_parameter_value().double_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value
        self.encoder_poll_rate_hz = self.get_parameter("encoder_poll_rate_hz").get_parameter_value().double_value
        self.m1_encoder_sign = self.get_parameter("m1_encoder_sign").get_parameter_value().integer_value
        self.m2_encoder_sign = self.get_parameter("m2_encoder_sign").get_parameter_value().integer_value

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.encoder_odom = EncoderOdom(self.ticks_per_meter, self.base_width)
        self.timer = self.create_timer(1.0 / self.encoder_poll_rate_hz, self._timer_callback)

    def compute_command(self, twist):
        right, left = self.compute_side_velocities(twist)
        m1 = int(right * self.ticks_per_meter)
        m2 = int(left * self.ticks_per_meter)
        if self.m1_invert:
            m1 = -m1
        if self.m2_invert:
            m2 = -m2
        return DriveCommand(m1=m1, m2=m2)

    def send_command(self, command: DriveCommand) -> None:
        try:
            if command.m1 == 0 and command.m2 == 0:
                self.stop_motors()
            else:
                roboclaw.SpeedM1M2(self.address, command.m1, command.m2)
        except OSError as exc:
            self.get_logger().warning(f"Failed sending Roboclaw speed command: {exc}")

    def stop_motors(self) -> None:
        roboclaw.ForwardM1(self.address, 0)
        roboclaw.ForwardM2(self.address, 0)

    def _timer_callback(self) -> None:
        self.check_command_timeout()
        self.maybe_publish_status()

        try:
            enc1 = roboclaw.ReadEncM1(self.address)
            enc2 = roboclaw.ReadEncM2(self.address)
        except OSError as exc:
            self.get_logger().warning(f"Failed reading Roboclaw encoders: {exc}")
            return

        if not enc1[0] or not enc2[0]:
            return

        left_ticks = int(enc2[1]) * self.m2_encoder_sign
        right_ticks = int(enc1[1]) * self.m1_encoder_sign
        now = self.get_clock().now()
        x, y, theta, vx, vtheta, _ = self.encoder_odom.update(left_ticks, right_ticks, now.nanoseconds / 1e9)
        qx, qy, qz, qw = EncoderOdom.quaternion_from_yaw(theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999.0
        odom.pose.covariance[21] = 99999.0
        odom.pose.covariance[28] = 99999.0
        odom.pose.covariance[35] = 0.01
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vtheta
        odom.twist.covariance = odom.pose.covariance
        self.odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RoboclawEncoderNode()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        get_logger("roboclaw_driver_encoder").error(str(exc))
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()
