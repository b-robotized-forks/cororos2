#!/usr/bin/env python3
# Copyright (c) 2026, b-robotized Group
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import math
import sys
import time

import roboclaw_protocol as roboclaw


class RoboclawBackend:
    def __init__(
        self,
        device: str,
        baud: int,
        address: int,
        use_encoder: bool,
        max_speed: float,
        ticks_at_max_speed: float,
        acceleration: int,
        ticks_per_meter: float,
        m1_invert: bool,
        m2_invert: bool,
        m1_encoder_sign: int,
        m2_encoder_sign: int,
    ) -> None:
        self.device = device
        self.baud = baud
        self.address = address
        self.use_encoder = use_encoder
        self.max_speed = max_speed
        self.ticks_at_max_speed = ticks_at_max_speed
        self.acceleration = acceleration
        self.ticks_per_meter = ticks_per_meter
        self.m1_invert = m1_invert
        self.m2_invert = m2_invert
        self.m1_encoder_sign = m1_encoder_sign
        self.m2_encoder_sign = m2_encoder_sign
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_read_time = None

    def connect(self) -> None:
        roboclaw.Open(self.device, self.baud)
        version = roboclaw.ReadVersion(self.address)
        if not version[0]:
            raise RuntimeError("connected to Roboclaw but could not read controller version")

    def activate(self) -> None:
        self.stop_motors()
        if self.use_encoder:
            roboclaw.ResetEncoders(self.address)
            self.last_left_ticks = None
            self.last_right_ticks = None
            self.last_read_time = None

    def deactivate(self) -> None:
        self.stop_motors()

    def drive(self, left_mps: float, right_mps: float) -> None:
        left_mps = max(min(left_mps, self.max_speed), -self.max_speed)
        right_mps = max(min(right_mps, self.max_speed), -self.max_speed)

        if self.use_encoder:
            m1 = int(right_mps * self.ticks_per_meter)
            m2 = int(left_mps * self.ticks_per_meter)
            if self.m1_invert:
                m1 = -m1
            if self.m2_invert:
                m2 = -m2
            if m1 == 0 and m2 == 0:
                self.stop_motors()
            else:
                roboclaw.SpeedM1M2(self.address, m1, m2)
            return

        m1 = int((right_mps / self.max_speed) * self.ticks_at_max_speed)
        m2 = int((left_mps / self.max_speed) * self.ticks_at_max_speed)
        if self.m1_invert:
            m1 = -m1
        if self.m2_invert:
            m2 = -m2
        if m1 == 0 and m2 == 0:
            self.stop_motors()
        else:
            roboclaw.DutyAccelM1(self.address, self.acceleration, m1)
            roboclaw.DutyAccelM2(self.address, self.acceleration, m2)

    def read_state(self) -> tuple[float, float, float, float]:
        enc1 = roboclaw.ReadEncM1(self.address)
        enc2 = roboclaw.ReadEncM2(self.address)
        if not enc1[0] or not enc2[0]:
            raise RuntimeError("failed to read Roboclaw encoders")

        right_ticks = int(enc1[1]) * self.m1_encoder_sign
        left_ticks = int(enc2[1]) * self.m2_encoder_sign
        now = time.monotonic()

        left_position_m = left_ticks / self.ticks_per_meter
        right_position_m = right_ticks / self.ticks_per_meter
        if self.last_read_time is None:
            left_velocity_mps = 0.0
            right_velocity_mps = 0.0
        else:
            dt = now - self.last_read_time
            if dt <= 0.0:
                left_velocity_mps = 0.0
                right_velocity_mps = 0.0
            else:
                left_velocity_mps = (left_ticks - self.last_left_ticks) / self.ticks_per_meter / dt
                right_velocity_mps = (right_ticks - self.last_right_ticks) / self.ticks_per_meter / dt

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_read_time = now
        return left_position_m, left_velocity_mps, right_position_m, right_velocity_mps

    def read_status(self) -> tuple[float, float, float, float, float, float, int]:
        main_battery_v = self._read_scaled_value(roboclaw.ReadMainBatteryVoltage, scale=10.0)
        logic_battery_v = self._read_scaled_value(roboclaw.ReadLogicBatteryVoltage, scale=10.0)
        m1_current_a, m2_current_a = self._read_scaled_pair(roboclaw.ReadCurrents, scale=100.0)
        temp1_c = self._read_scaled_value(roboclaw.ReadTemp, scale=10.0)
        temp2_c = self._read_scaled_value(roboclaw.ReadTemp2, scale=10.0)
        error_word = self._read_integer_value(roboclaw.ReadError, default=-1)
        return (
            main_battery_v,
            logic_battery_v,
            m1_current_a,
            m2_current_a,
            temp1_c,
            temp2_c,
            error_word,
        )

    def stop_motors(self) -> None:
        if self.use_encoder:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
        else:
            roboclaw.DutyAccelM1(self.address, 0, 0)
            roboclaw.DutyAccelM2(self.address, 0, 0)

    def _read_scaled_value(self, reader, scale: float) -> float:
        try:
            result = reader(self.address)
        except Exception:
            return math.nan
        if not result[0]:
            return math.nan
        return float(result[1]) / scale

    def _read_scaled_pair(self, reader, scale: float) -> tuple[float, float]:
        try:
            result = reader(self.address)
        except Exception:
            return math.nan, math.nan
        if not result[0]:
            return math.nan, math.nan
        return float(result[1]) / scale, float(result[2]) / scale

    def _read_integer_value(self, reader, default: int) -> int:
        try:
            result = reader(self.address)
        except Exception:
            return default
        if not result[0]:
            return default
        return int(result[1])


def parse_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--address", type=int, default=128)
    parser.add_argument("--use-encoder", type=parse_bool, default=False)
    parser.add_argument("--max-speed", type=float, default=1.2)
    parser.add_argument("--ticks-at-max-speed", type=float, default=32760.0)
    parser.add_argument("--acceleration", type=int, default=32000)
    parser.add_argument("--ticks-per-meter", type=float, default=4342.2)
    parser.add_argument("--m1-invert", type=parse_bool, default=False)
    parser.add_argument("--m2-invert", type=parse_bool, default=False)
    parser.add_argument("--m1-encoder-sign", type=int, default=1)
    parser.add_argument("--m2-encoder-sign", type=int, default=1)
    args = parser.parse_args()

    backend = RoboclawBackend(
        device=args.device,
        baud=args.baud,
        address=args.address,
        use_encoder=args.use_encoder,
        max_speed=args.max_speed,
        ticks_at_max_speed=args.ticks_at_max_speed,
        acceleration=args.acceleration,
        ticks_per_meter=args.ticks_per_meter,
        m1_invert=args.m1_invert,
        m2_invert=args.m2_invert,
        m1_encoder_sign=args.m1_encoder_sign,
        m2_encoder_sign=args.m2_encoder_sign,
    )

    try:
        backend.connect()
    except Exception as exc:  # noqa: BLE001
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
                backend.activate()
                active = True
                print("OK", flush=True)
            elif command == "DEACTIVATE":
                backend.deactivate()
                active = False
                print("OK", flush=True)
            elif command == "WRITE":
                if len(parts) != 3:
                    print("ERROR bad write command", flush=True)
                    continue
                left_mps, right_mps = (float(value) for value in parts[1:3])
                if active:
                    backend.drive(left_mps, right_mps)
                print("OK", flush=True)
            elif command == "READ":
                if not args.use_encoder:
                    print("ERROR encoder mode disabled", flush=True)
                    continue
                values = backend.read_state()
                print("STATE " + " ".join(f"{value:.10f}" for value in values), flush=True)
            elif command == "STATUS":
                values = backend.read_status()
                print(
                    "STATUS " + " ".join(f"{value:.10f}" for value in values[:-1]) + f" {values[-1]}",
                    flush=True,
                )
            elif command == "EXIT":
                backend.deactivate()
                active = False
                print("OK", flush=True)
                return 0
            else:
                print(f"ERROR unknown command: {command}", flush=True)
    finally:
        if active:
            try:
                backend.deactivate()
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
