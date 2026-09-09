from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from x5_gripper_sdk import (
    GripperCalibration,
    GripperConfig,
    GripperSafetyError,
    X5Gripper,
    load_calibration,
    save_calibration,
)
from x5_gripper_sdk.protocol import (
    DISABLE_COMMAND,
    ENABLE_COMMAND,
    SET_ZERO_COMMAND,
    DM_J4310_LIMITS,
    float_to_uint,
    is_system_command,
    unpack_mit_command,
)


def feedback_bytes(
    *,
    motor_id: int = 8,
    position: float = 2.48,
    velocity: float = 0.0,
    torque: float = 0.0,
) -> bytes:
    p = float_to_uint(position, -12.5, 12.5, 16)
    v = float_to_uint(velocity, -30.0, 30.0, 12)
    t = float_to_uint(torque, -10.0, 10.0, 12)
    return bytes(
        (
            0x10 | motor_id,
            p >> 8,
            p & 0xFF,
            v >> 4,
            ((v & 0xF) << 4) | (t >> 8),
            t & 0xFF,
            40,
            35,
        )
    )


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now


class FakeTransport:
    def __init__(self, clock: FakeClock, *, position: float = 2.48, velocity: float = 0.0):
        self.clock = clock
        self.position = position
        self.velocity = velocity
        self.sent: list[tuple[int, bytes]] = []
        self.inbox: list[tuple[int, bytes]] = []
        self.closed = False

    def send(self, can_id: int, data: bytes) -> None:
        payload = bytes(data)
        self.sent.append((can_id, payload))
        if not is_system_command(payload):
            command = unpack_mit_command(payload)
            if command["kp"] >= 0.01:
                self.position = command["position"]
            elif abs(command["torque"]) >= 0.01:
                self.position += math.copysign(0.05, command["torque"])
        self.inbox.append((0x18, feedback_bytes(position=self.position, velocity=self.velocity)))

    def recv(self, timeout: float) -> tuple[int, bytes] | None:
        if self.inbox:
            return self.inbox.pop(0)
        self.clock.now += max(0.0, timeout)
        return None

    def close(self) -> None:
        self.closed = True


def config(**overrides: object) -> GripperConfig:
    values: dict[str, object] = {
        "interface": "can2",
        "motor_can_id": 8,
        "kp": 5.0,
        "kd": 0.2,
        "claim_listen_s": 0.02,
        "acknowledge_unverified_hardware": True,
    }
    values.update(overrides)
    return GripperConfig(**values)  # type: ignore[arg-type]


class ProtocolTest(unittest.TestCase):
    def test_command_round_trip(self) -> None:
        from x5_gripper_sdk.protocol import pack_mit_command

        payload = pack_mit_command(1.2, -0.5, 5.0, 0.2, -0.1, DM_J4310_LIMITS)
        decoded = unpack_mit_command(payload)
        self.assertAlmostEqual(decoded["position"], 1.2, places=3)
        self.assertAlmostEqual(decoded["velocity"], -0.5, delta=0.01)
        self.assertAlmostEqual(decoded["kp"], 5.0, places=1)


class CalibrationTest(unittest.TestCase):
    def test_save_load_and_normalize(self) -> None:
        calibration = GripperCalibration(
            device_serial="unit-1",
            interface="can2",
            motor_can_id=8,
            feedback_can_id=None,
            closed_position_rad=2.0,
            open_position_rad=1.0,
            calibrated_at="2026-01-01T00:00:00+08:00",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = save_calibration(Path(directory) / "gripper.json", calibration)
            loaded = load_calibration(path, device_serial="unit-1", interface="can2", motor_can_id=8)
        self.assertEqual(loaded, calibration)
        self.assertEqual(loaded.normalized_opening(2.0), 0.0)
        self.assertEqual(loaded.normalized_opening(1.0), 1.0)
        self.assertEqual(loaded.position_for_opening(0.25), 1.75)

    def test_device_mismatch_is_rejected(self) -> None:
        payload = {
            "device_serial": "another-device",
            "interface": "can2",
            "motor_can_id": 8,
            "feedback_can_id": None,
            "closed_position_rad": 2.0,
            "open_position_rad": 1.0,
            "calibrated_at": "now",
            "schema_version": 1,
            "robot_model": "X5-2023",
            "motor_profile": "dm_j4310",
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gripper.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "序列号"):
                load_calibration(path, device_serial="unit-1")


class DriverTest(unittest.TestCase):
    @staticmethod
    def _write_calibration(directory: str) -> Path:
        path = Path(directory) / "gripper.json"
        path.write_text(
            json.dumps(
                {
                    "device_serial": "X5-2023-001",
                    "interface": "can2",
                    "motor_can_id": 8,
                    "feedback_can_id": None,
                    "closed_position_rad": 2.0,
                    "open_position_rad": 1.0,
                    "calibrated_at": "now",
                    "schema_version": 1,
                    "robot_model": "X5-2023",
                    "motor_profile": "dm_j4310",
                }
            ),
            encoding="utf-8",
        )
        return path

    def test_hold_never_sets_zero_and_always_disables(self) -> None:
        clock = FakeClock()
        transport = FakeTransport(clock)
        with X5Gripper(config(), transport=transport, monotonic=clock.monotonic) as gripper:
            result = gripper.hold(0.05)
        payloads = [payload for _, payload in transport.sent]
        self.assertEqual(payloads[-1], DISABLE_COMMAND)
        self.assertNotIn(SET_ZERO_COMMAND, payloads)
        self.assertAlmostEqual(result.start_position_rad, 2.48, places=2)
        self.assertEqual({can_id for can_id, _ in transport.sent}, {8})

    def test_open_uses_negative_torque(self) -> None:
        clock = FakeClock()
        transport = FakeTransport(clock, position=2.0)
        gripper = X5Gripper(config(), transport=transport, monotonic=clock.monotonic).connect()
        result = gripper.open_relative(0.10, torque_nm=-0.10, duration_after_s=0.05)
        commands = [
            unpack_mit_command(payload)
            for _, payload in transport.sent
            if payload not in (ENABLE_COMMAND, DISABLE_COMMAND)
        ]
        self.assertLess(result.final_state.position_rad, result.start_position_rad)
        self.assertTrue(any(command["torque"] < -0.05 for command in commands))
        self.assertEqual(transport.sent[-1][1], DISABLE_COMMAND)

    def test_close_can_use_explicit_quarter_nm_torque(self) -> None:
        clock = FakeClock()
        transport = FakeTransport(clock, position=1.0)
        gripper = X5Gripper(config(), transport=transport, monotonic=clock.monotonic).connect()
        result = gripper.close_relative(0.03, torque_nm=0.25, duration_after_s=0.05)
        commands = [
            unpack_mit_command(payload)
            for _, payload in transport.sent
            if payload not in (ENABLE_COMMAND, DISABLE_COMMAND)
        ]
        self.assertTrue(result.target_reached)
        self.assertTrue(any(command["torque"] > 0.24 for command in commands))
        self.assertTrue(all(command["kp"] < 0.01 for command in commands))
        self.assertEqual(transport.sent[-1][1], DISABLE_COMMAND)

    def test_move_to_opening_streams_linear_position_trajectory(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self._write_calibration(directory)
            clock = FakeClock()
            transport = FakeTransport(clock, position=1.8)
            gripper = X5Gripper(
                config(calibration_path=path),
                transport=transport,
                monotonic=clock.monotonic,
            ).connect()
            result = gripper.move_to_opening(
                0.5,
                speed_rad_s=0.25,
                duration_after_s=0.05,
            )

        commands = [
            unpack_mit_command(payload)
            for _, payload in transport.sent
            if payload not in (ENABLE_COMMAND, DISABLE_COMMAND)
        ]
        positions = [float(command["position"]) for command in commands]
        self.assertEqual(result.operation, "move_to_opening")
        self.assertAlmostEqual(result.target_position_rad, 1.5, places=6)
        self.assertAlmostEqual(result.final_state.opening or 0.0, 0.5, places=2)
        self.assertTrue(result.target_reached)
        self.assertGreater(len({round(position, 3) for position in positions}), 3)
        self.assertTrue(all(a >= b for a, b in zip(positions, positions[1:])))
        self.assertTrue(all(command["kp"] >= 4.9 for command in commands))
        self.assertTrue(all(abs(command["torque"]) < 0.01 for command in commands))
        self.assertEqual(transport.sent[-1][1], DISABLE_COMMAND)

    def test_linear_opening_requires_calibration(self) -> None:
        clock = FakeClock()
        transport = FakeTransport(clock)
        gripper = X5Gripper(
            config(), transport=transport, monotonic=clock.monotonic
        ).connect()
        with self.assertRaisesRegex(GripperSafetyError, "双端点标定"):
            gripper.move_to_opening(0.5)
        self.assertEqual(transport.sent, [])

    def test_linear_endpoint_helpers_use_calibrated_endpoints(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self._write_calibration(directory)

            open_clock = FakeClock()
            open_transport = FakeTransport(open_clock, position=1.5)
            opened = X5Gripper(
                config(calibration_path=path),
                transport=open_transport,
                monotonic=open_clock.monotonic,
            ).connect().open_linearly(speed_rad_s=0.25, duration_after_s=0.05)

            close_clock = FakeClock()
            close_transport = FakeTransport(close_clock, position=1.5)
            closed = X5Gripper(
                config(calibration_path=path),
                transport=close_transport,
                monotonic=close_clock.monotonic,
            ).connect().close_linearly(speed_rad_s=0.25, duration_after_s=0.05)

        self.assertAlmostEqual(opened.target_position_rad, 1.0)
        self.assertAlmostEqual(closed.target_position_rad, 2.0)
        self.assertTrue(opened.target_reached)
        self.assertTrue(closed.target_reached)

    def test_hold_to_move_callback_stops_linear_motion_and_disables(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self._write_calibration(directory)
            clock = FakeClock()
            transport = FakeTransport(clock, position=1.5)
            gripper = X5Gripper(
                config(calibration_path=path),
                transport=transport,
                monotonic=clock.monotonic,
            ).connect()
            callback_count = 0

            def continue_motion() -> bool:
                nonlocal callback_count
                callback_count += 1
                return callback_count <= 4

            result = gripper.close_linearly_while(
                continue_motion,
                speed_rad_s=0.25,
            )

        self.assertTrue(result.stopped_by_request)
        self.assertFalse(result.target_reached)
        self.assertLess(result.final_state.position_rad, result.target_position_rad)
        self.assertEqual(transport.sent[-1][1], DISABLE_COMMAND)

    def test_velocity_fault_disables(self) -> None:
        clock = FakeClock()
        transport = FakeTransport(clock, velocity=2.1)
        gripper = X5Gripper(config(), transport=transport, monotonic=clock.monotonic).connect()
        with self.assertRaisesRegex(GripperSafetyError, "尚未静止"):
            gripper.hold(0.05)
        self.assertEqual(transport.sent[-1][1], DISABLE_COMMAND)

    def test_acknowledgement_is_required(self) -> None:
        clock = FakeClock()
        transport = FakeTransport(clock)
        gripper = X5Gripper(
            config(acknowledge_unverified_hardware=False),
            transport=transport,
            monotonic=clock.monotonic,
        ).connect()
        with self.assertRaisesRegex(GripperSafetyError, "风险确认"):
            gripper.hold(0.05)
        self.assertEqual(transport.sent, [])

    def test_calibration_endpoint_prevents_overtravel(self) -> None:
        calibration_payload = {
            "device_serial": "X5-2023-001",
            "interface": "can2",
            "motor_can_id": 8,
            "feedback_can_id": None,
            "closed_position_rad": 2.0,
            "open_position_rad": 1.0,
            "calibrated_at": "now",
            "schema_version": 1,
            "robot_model": "X5-2023",
            "motor_profile": "dm_j4310",
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gripper.json"
            path.write_text(json.dumps(calibration_payload), encoding="utf-8")
            clock = FakeClock()
            transport = FakeTransport(clock, position=1.02)
            gripper = X5Gripper(
                config(calibration_path=path),
                transport=transport,
                monotonic=clock.monotonic,
            ).connect()
            with self.assertRaisesRegex(GripperSafetyError, "标定端点"):
                gripper.open_relative(0.05, duration_after_s=0.05)
        self.assertEqual(transport.sent[-1][1], DISABLE_COMMAND)


if __name__ == "__main__":
    unittest.main()
