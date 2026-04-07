#!/usr/bin/env python3
import argparse
import logging
import math
import sys
import time

IMPORT_ERROR = None

try:
    import odrive
    from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_VELOCITY_CONTROL
except ModuleNotFoundError as exc:  # pragma: no cover - hardware env dependent
    odrive = None
    AXIS_STATE_CLOSED_LOOP_CONTROL = None
    AXIS_STATE_IDLE = None
    CONTROL_MODE_VELOCITY_CONTROL = None
    IMPORT_ERROR = exc


class ODriveBoard:
    MOTOR_STATES = [
        "UNDEFINED",
        "IDLE",
        "STARTUP_SEQUENCE",
        "FULL_CALIBRATION_SEQUENCE",
        "MOTOR_CALIBRATION",
        "SENSORLESS_CONTROL",
        "ENCODER_INDEX_SEARCH",
        "ENCODER_OFFSET_CALIBRATION",
        "CLOSED_LOOP_CONTROL",
        "LOCKIN_SPIN",
        "ENCODER_DIR_FIND",
    ]

    def __init__(
        self,
        serial_number: str,
        right_axis: int,
        connect_timeout: float,
        clear_errors_on_startup: bool,
        estop_adc_channel: int,
        i2t_current_nominal: float,
        i2t_update_rate: float,
        i2t_resume_threshold: float,
        i2t_warning_threshold: float,
        i2t_error_threshold: float,
    ) -> None:
        self.serial_number = serial_number
        self.right_axis_index = right_axis
        self.connect_timeout = connect_timeout
        self.clear_errors_on_startup = clear_errors_on_startup
        self.estop_adc_channel = estop_adc_channel
        self.i2t_current_nominal = i2t_current_nominal
        self.i2t_update_rate = i2t_update_rate
        self.i2t_resume_threshold = i2t_resume_threshold
        self.i2t_warning_threshold = i2t_warning_threshold
        self.i2t_error_threshold = i2t_error_threshold
        self.driver = None
        self.left_axis = None
        self.right_axis = None
        self.active = False
        self.i2t_latched = False
        self.last_i2t_update = None
        self.left_i2t = 0.0
        self.right_i2t = 0.0
        self.last_vbus_voltage = float("nan")
        self.last_estop_voltage = float("nan")

    @staticmethod
    def _safe_float(value, default: float = float("nan")) -> float:
        try:
            return float(value)
        except Exception:
            return default

    @staticmethod
    def _axis_error_snapshot(axis) -> tuple[int, int, int, int]:
        return (
            int(getattr(axis, "error", 0)),
            int(getattr(getattr(axis, "motor", None), "error", 0)),
            int(getattr(getattr(axis, "encoder", None), "error", 0)),
            int(getattr(getattr(axis, "controller", None), "error", 0)),
        )

    @classmethod
    def _format_axis_error_snapshot(cls, label: str, axis) -> str:
        axis_error, motor_error, encoder_error, controller_error = cls._axis_error_snapshot(axis)
        return (
            f"{label}:axis=0x{axis_error:X},motor=0x{motor_error:X},"
            f"encoder=0x{encoder_error:X},controller=0x{controller_error:X}"
        )

    def _format_all_errors(self) -> str:
        return "; ".join(
            [
                self._format_axis_error_snapshot("left", self.left_axis),
                self._format_axis_error_snapshot("right", self.right_axis),
            ]
        )

    def _has_any_errors(self) -> bool:
        return any(self._axis_error_snapshot(axis) != (0, 0, 0, 0) for axis in (self.left_axis, self.right_axis))

    def _clear_errors(self) -> None:
        for axis in (self.left_axis, self.right_axis):
            axis.error = 0
            axis.motor.error = 0
            axis.encoder.error = 0
            axis.controller.error = 0

    def _axis_current(self, axis) -> float:
        try:
            return self._safe_float(axis.motor.current_control.Ibus, 0.0)
        except Exception:
            return 0.0

    def _axis_temperature(self, axis) -> float:
        try:
            return self._safe_float(axis.fet_thermistor.temperature)
        except Exception:
            return float("nan")

    def connect(self) -> None:
        self.driver = odrive.find_any(path="usb", serial_number=self.serial_number, timeout=self.connect_timeout)
        self.right_axis = self.driver.axis0 if self.right_axis_index == 0 else self.driver.axis1
        self.left_axis = self.driver.axis1 if self.right_axis_index == 0 else self.driver.axis0
        if self._has_any_errors():
            startup_errors = self._format_all_errors()
            if self.clear_errors_on_startup:
                self._clear_errors()
                if self._has_any_errors():
                    raise RuntimeError(
                        f"startup errors persisted after clear on {self.serial_number}: {startup_errors}; now {self._format_all_errors()}"
                    )
            else:
                raise RuntimeError(f"startup errors on {self.serial_number}: {startup_errors}")
        self.refresh_status()

    def activate(self) -> None:
        if self._has_any_errors():
            raise RuntimeError(f"cannot activate {self.serial_number} with active errors: {self._format_all_errors()}")
        for axis in (self.left_axis, self.right_axis):
            axis.config.watchdog_timeout = 1.0
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis.controller.input_vel = 0.0
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.config.enable_watchdog = True
            axis.watchdog_feed()
        self.active = True

    def deactivate(self) -> None:
        if not self.driver:
            return
        for axis in (self.left_axis, self.right_axis):
            axis.requested_state = AXIS_STATE_IDLE
            axis.config.enable_watchdog = False
        self.active = False

    def drive(self, left_turns_per_second: float, right_turns_per_second: float) -> None:
        self.refresh_status()
        self.left_axis.watchdog_feed()
        self.right_axis.watchdog_feed()
        if self.i2t_latched:
            self.left_axis.controller.input_vel = 0.0
            self.right_axis.controller.input_vel = 0.0
            return
        self.left_axis.controller.input_vel = left_turns_per_second
        self.right_axis.controller.input_vel = -right_turns_per_second

    def read(self) -> list[float]:
        self.refresh_status()
        return [
            float(self.left_axis.encoder.pos_estimate),
            float(self.left_axis.encoder.vel_estimate),
            float(-self.right_axis.encoder.pos_estimate),
            float(-self.right_axis.encoder.vel_estimate),
        ]

    def _read_axis_status(self, axis, i2t_value: float) -> list[float]:
        axis_error, motor_error, encoder_error, controller_error = self._axis_error_snapshot(axis)
        return [
            self._axis_current(axis),
            self._axis_temperature(axis),
            self._safe_float(i2t_value, 0.0),
            float(getattr(axis, "current_state", 0)),
            float(axis_error),
            float(motor_error),
            float(encoder_error),
            float(controller_error),
        ]

    def _update_i2t(self, now: float) -> None:
        if self.last_i2t_update is None:
            self.last_i2t_update = now
            return
        dt = max(0.0, now - self.last_i2t_update)
        self.last_i2t_update = now

        left_current = self._axis_current(self.left_axis)
        right_current = self._axis_current(self.right_axis)

        left_power = max(0.0, left_current**2 - self.i2t_current_nominal**2)
        right_power = max(0.0, right_current**2 - self.i2t_current_nominal**2)
        decay = max(0.0, 1.0 - self.i2t_update_rate * dt)

        self.left_i2t = (self.left_i2t * decay) + (left_power * dt)
        self.right_i2t = (self.right_i2t * decay) + (right_power * dt)

        if max(self.left_i2t, self.right_i2t) > self.i2t_error_threshold:
            self.i2t_latched = True
        elif self.i2t_latched and max(self.left_i2t, self.right_i2t) < self.i2t_resume_threshold:
            self.i2t_latched = False

    def refresh_status(self) -> None:
        if not self.driver:
            return
        now = time.monotonic()
        self._update_i2t(now)
        self.last_vbus_voltage = self._safe_float(getattr(self.driver, "vbus_voltage", float("nan")))
        try:
            self.last_estop_voltage = self._safe_float(self.driver.get_adc_voltage(self.estop_adc_channel))
        except Exception:
            self.last_estop_voltage = float("nan")
        if self.i2t_latched:
            try:
                self.left_axis.controller.input_vel = 0.0
                self.right_axis.controller.input_vel = 0.0
            except Exception:
                pass

    def status_fields(self) -> list[float]:
        self.refresh_status()
        return [
            self._safe_float(self.last_vbus_voltage),
            self._safe_float(self.last_estop_voltage),
            float(1 if self.active else 0),
            float(1 if self.i2t_latched else 0),
            *self._read_axis_status(self.left_axis, self.left_i2t),
            *self._read_axis_status(self.right_axis, self.right_i2t),
        ]

    def disconnect(self) -> None:
        try:
            self.deactivate()
        except Exception:
            pass
        self.active = False
        self.driver = None
        self.left_axis = None
        self.right_axis = None


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "on"}:
        return True
    if lowered in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--front-serial", required=True)
    parser.add_argument("--rear-serial", required=True)
    parser.add_argument("--front-right-axis", type=int, default=0)
    parser.add_argument("--rear-right-axis", type=int, default=1)
    parser.add_argument("--connect-timeout", type=float, default=30.0)
    parser.add_argument("--clear-errors-on-startup", type=parse_bool, default=True)
    parser.add_argument("--estop-adc-channel", type=int, default=5)
    parser.add_argument("--i2t-current-nominal", type=float, default=2.0)
    parser.add_argument("--i2t-update-rate", type=float, default=0.01)
    parser.add_argument("--i2t-resume-threshold", type=float, default=2220.0)
    parser.add_argument("--i2t-warning-threshold", type=float, default=3330.0)
    parser.add_argument("--i2t-error-threshold", type=float, default=6660.0)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, stream=sys.stderr)

    if IMPORT_ERROR is not None:
        print(f"ERROR missing python dependency: {IMPORT_ERROR}", flush=True)
        return 1

    front = ODriveBoard(
        args.front_serial,
        args.front_right_axis,
        args.connect_timeout,
        args.clear_errors_on_startup,
        args.estop_adc_channel,
        args.i2t_current_nominal,
        args.i2t_update_rate,
        args.i2t_resume_threshold,
        args.i2t_warning_threshold,
        args.i2t_error_threshold,
    )
    rear = ODriveBoard(
        args.rear_serial,
        args.rear_right_axis,
        args.connect_timeout,
        args.clear_errors_on_startup,
        args.estop_adc_channel,
        args.i2t_current_nominal,
        args.i2t_update_rate,
        args.i2t_resume_threshold,
        args.i2t_warning_threshold,
        args.i2t_error_threshold,
    )

    try:
        front.connect()
        rear.connect()
    except Exception as exc:
        front.disconnect()
        rear.disconnect()
        print(f"ERROR connect failed: {exc}", flush=True)
        return 1

    active = False
    try:
        for raw_line in sys.stdin:
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            command = parts[0]

            if command == "PING":
                print("PONG", flush=True)
            elif command == "ACTIVATE":
                try:
                    front.activate()
                    rear.activate()
                    active = True
                    print("OK", flush=True)
                except Exception as exc:
                    front.deactivate()
                    rear.deactivate()
                    active = False
                    print(f"ERROR activate failed: {exc}", flush=True)
            elif command == "DEACTIVATE":
                front.deactivate()
                rear.deactivate()
                active = False
                print("OK", flush=True)
            elif command == "WRITE":
                if len(parts) != 5:
                    print("ERROR bad write command", flush=True)
                    continue
                fl, fr, rl, rr = (float(value) for value in parts[1:5])
                try:
                    if active:
                        front.drive(fl, fr)
                        rear.drive(rl, rr)
                    print("OK", flush=True)
                except Exception as exc:
                    print(f"ERROR write failed: {exc}", flush=True)
            elif command == "READ":
                try:
                    values = front.read() + rear.read()
                    print("STATE " + " ".join(f"{value:.10f}" for value in values), flush=True)
                except Exception as exc:
                    print(f"ERROR read failed: {exc}", flush=True)
            elif command == "STATUS":
                values = front.status_fields() + rear.status_fields()
                print(
                    "STATUS " + " ".join("nan" if math.isnan(value) else f"{value:.10f}" for value in values),
                    flush=True,
                )
            elif command == "EXIT":
                front.disconnect()
                rear.disconnect()
                print("OK", flush=True)
                return 0
            else:
                print(f"ERROR unknown command: {command}", flush=True)
    finally:
        front.disconnect()
        rear.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
