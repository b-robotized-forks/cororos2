import math
from dataclasses import dataclass

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

from . import roboclaw_protocol as roboclaw


@dataclass
class DriveCommand:
    m1: int
    m2: int


class RoboclawBase(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter("device", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("address", 128)
        self.declare_parameter("max_speed", 1.2)
        self.declare_parameter("base_width", 0.387)
        self.declare_parameter("command_timeout_sec", 1.0)
        self.declare_parameter("status_interval_sec", 2.0)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("use_stamped_cmd_vel", False)
        self.declare_parameter("m1_invert", False)
        self.declare_parameter("m2_invert", False)

        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.address = self.get_parameter("address").get_parameter_value().integer_value
        self.max_speed = self.get_parameter("max_speed").get_parameter_value().double_value
        self.base_width = self.get_parameter("base_width").get_parameter_value().double_value
        self.command_timeout = Duration(
            seconds=self.get_parameter("command_timeout_sec").get_parameter_value().double_value
        )
        self.status_interval = Duration(
            seconds=self.get_parameter("status_interval_sec").get_parameter_value().double_value
        )
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.use_stamped_cmd_vel = self.get_parameter("use_stamped_cmd_vel").get_parameter_value().bool_value
        self.m1_invert = self.get_parameter("m1_invert").get_parameter_value().bool_value
        self.m2_invert = self.get_parameter("m2_invert").get_parameter_value().bool_value

        if self.address > 0x87 or self.address < 0x80:
            raise ValueError("Roboclaw address must be between 128 and 135 inclusive")
        if self.max_speed <= 0.0:
            raise ValueError("max_speed must be > 0")
        if self.base_width <= 0.0:
            raise ValueError("base_width must be > 0")

        self.last_command_time = self.get_clock().now()
        self.last_status_time = self.get_clock().now()
        self.is_stopped = True

        self.battery_pub = self.create_publisher(BatteryState, "roboclaw/battery_state", 10)
        self.m1_current_pub = self.create_publisher(Float32, "roboclaw/m1_current", 10)
        self.m2_current_pub = self.create_publisher(Float32, "roboclaw/m2_current", 10)

        self._connect()
        self._post_connect_initialization()

        if self.use_stamped_cmd_vel:
            self.create_subscription(TwistStamped, self.cmd_vel_topic, self._cmd_vel_stamped_callback, 10)
        else:
            self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)

    def _connect(self) -> None:
        self.get_logger().info(f"Connecting to Roboclaw on {self.device} @ {self.baud} baud")
        try:
            roboclaw.Open(self.device, self.baud)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"Could not open Roboclaw device {self.device}: {exc}") from exc

        version = roboclaw.ReadVersion(self.address)
        if version[0]:
            self.get_logger().info(f"Connected to Roboclaw version: {version[1]}")
        else:
            self.get_logger().warning("Connected to Roboclaw but could not read controller version")

    def _post_connect_initialization(self) -> None:
        self.stop_motors()
        try:
            roboclaw.ResetEncoders(self.address)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"Could not reset encoders during initialization: {exc}")

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.handle_twist(msg)

    def _cmd_vel_stamped_callback(self, msg: TwistStamped) -> None:
        self.handle_twist(msg.twist)

    def handle_twist(self, twist: Twist) -> None:
        self.last_command_time = self.get_clock().now()
        self.is_stopped = False
        command = self.compute_command(twist)
        self.send_command(command)

    def compute_side_velocities(self, twist: Twist) -> tuple[float, float]:
        linear_x = twist.linear.x
        angular_z = twist.angular.z
        right = linear_x + angular_z * self.base_width / 2.0
        left = linear_x - angular_z * self.base_width / 2.0
        right = max(min(right, self.max_speed), -self.max_speed)
        left = max(min(left, self.max_speed), -self.max_speed)
        return right, left

    def maybe_publish_status(self) -> None:
        now = self.get_clock().now()
        if now - self.last_status_time < self.status_interval:
            return
        self.last_status_time = now

        try:
            voltage = roboclaw.ReadMainBatteryVoltage(self.address)
            currents = roboclaw.ReadCurrents(self.address)
        except OSError as exc:
            self.get_logger().warning(f"Failed reading Roboclaw status: {exc}")
            return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Unexpected error reading Roboclaw status: {exc}")
            return

        if voltage[0]:
            battery_msg = BatteryState()
            battery_msg.header.stamp = now.to_msg()
            battery_msg.voltage = float(voltage[1]) / 10.0
            if currents[0]:
                battery_msg.current = float(currents[1] + currents[2]) / 100.0
            self.battery_pub.publish(battery_msg)

        if currents[0]:
            m1_msg = Float32()
            m1_msg.data = float(currents[1]) / 100.0
            m2_msg = Float32()
            m2_msg.data = float(currents[2]) / 100.0
            self.m1_current_pub.publish(m1_msg)
            self.m2_current_pub.publish(m2_msg)

    def check_command_timeout(self) -> None:
        if self.is_stopped:
            return
        if self.get_clock().now() - self.last_command_time > self.command_timeout:
            self.get_logger().warn("Did not receive cmd_vel within timeout, stopping motors")
            self.stop_motors()
            self.is_stopped = True

    def compute_command(self, twist: Twist) -> DriveCommand:
        raise NotImplementedError

    def send_command(self, command: DriveCommand) -> None:
        raise NotImplementedError

    def stop_motors(self) -> None:
        raise NotImplementedError

    def shutdown(self) -> None:
        self.get_logger().info("Stopping Roboclaw motors on shutdown")
        try:
            self.stop_motors()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to stop Roboclaw motors during shutdown: {exc}")


class EncoderOdom:
    def __init__(self, ticks_per_meter: float, base_width: float) -> None:
        self.ticks_per_meter = ticks_per_meter
        self.base_width = base_width
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_theta = 0.0
        self.last_left = 0
        self.last_right = 0
        self.last_time = None

    @staticmethod
    def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def update(
        self, left_ticks: int, right_ticks: int, now_sec: float
    ) -> tuple[float, float, float, float, float, float]:
        if self.last_time is None:
            self.last_left = left_ticks
            self.last_right = right_ticks
            self.last_time = now_sec
            return self.cur_x, self.cur_y, self.cur_theta, 0.0, 0.0, 0.0

        delta_left = left_ticks - self.last_left
        delta_right = right_ticks - self.last_right
        self.last_left = left_ticks
        self.last_right = right_ticks

        dist_left = delta_left / self.ticks_per_meter
        dist_right = delta_right / self.ticks_per_meter
        dist = (dist_right + dist_left) / 2.0

        dt = now_sec - self.last_time
        self.last_time = now_sec

        if delta_left == delta_right:
            d_theta = 0.0
            self.cur_x += dist * math.cos(self.cur_theta)
            self.cur_y += dist * math.sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.base_width
            radius = dist / d_theta
            self.cur_x += radius * (math.sin(d_theta + self.cur_theta) - math.sin(self.cur_theta))
            self.cur_y -= radius * (math.cos(d_theta + self.cur_theta) - math.cos(self.cur_theta))
            self.cur_theta += d_theta
            while self.cur_theta > math.pi:
                self.cur_theta -= 2.0 * math.pi
            while self.cur_theta < -math.pi:
                self.cur_theta += 2.0 * math.pi

        if abs(dt) < 1e-6:
            return self.cur_x, self.cur_y, self.cur_theta, 0.0, 0.0, d_theta

        return self.cur_x, self.cur_y, self.cur_theta, dist / dt, d_theta / dt, d_theta
