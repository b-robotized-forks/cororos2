from geometry_msgs.msg import Twist
import rclpy
from rclpy.logging import get_logger

from .roboclaw_common import DriveCommand, RoboclawBase
from . import roboclaw_protocol as roboclaw


class RoboclawNoEncoderNode(RoboclawBase):
    def __init__(self) -> None:
        self.ticks_at_max_speed = 32760.0
        self.acceleration = 32000
        super().__init__("roboclaw_driver_no_encoder")
        self.declare_parameter("ticks_at_max_speed", 32760.0)
        self.declare_parameter("acceleration", 32000)
        self.ticks_at_max_speed = self.get_parameter("ticks_at_max_speed").get_parameter_value().double_value
        self.acceleration = self.get_parameter("acceleration").get_parameter_value().integer_value
        self.timer = self.create_timer(0.1, self._timer_callback)

    def compute_command(self, twist: Twist) -> DriveCommand:
        right, left = self.compute_side_velocities(twist)
        m1 = int((right / self.max_speed) * self.ticks_at_max_speed)
        m2 = int((left / self.max_speed) * self.ticks_at_max_speed)
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
                roboclaw.DutyAccelM1(self.address, self.acceleration, command.m1)
                roboclaw.DutyAccelM2(self.address, self.acceleration, command.m2)
        except OSError as exc:
            self.get_logger().warning(f"Failed sending Roboclaw duty command: {exc}")

    def stop_motors(self) -> None:
        roboclaw.DutyAccelM1(self.address, 0, 0)
        roboclaw.DutyAccelM2(self.address, 0, 0)

    def _timer_callback(self) -> None:
        self.check_command_timeout()
        self.maybe_publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RoboclawNoEncoderNode()
        rclpy.spin(node)
    except Exception as exc:  # noqa: BLE001
        get_logger("roboclaw_driver_no_encoder").error(str(exc))
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()
