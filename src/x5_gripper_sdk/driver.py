"""X5 夹爪同步控制 API。每个动作结束后都会失能电机。"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Callable

from .calibration import GripperCalibration, load_calibration
from .models import (
    GripperConfig,
    GripperError,
    GripperSafetyError,
    GripperState,
    MotionResult,
)
from .protocol import (
    DISABLE_COMMAND,
    ENABLE_COMMAND,
    MitFeedback,
    is_system_command,
    looks_like_feedback,
    pack_mit_command,
    unpack_mit_command,
    unpack_mit_feedback,
)
from .transport import CanTransport, SocketCanTransport


EventCallback = Callable[[str, dict[str, object]], None]


class X5Gripper:
    """独占控制一个 X5-2023 外置夹爪 ESC。

    这是同步、阻塞式 API。一次只允许运行一个动作；动作正常结束或异常退出时
    都会发送 DISABLE。SDK 永远不会发送电机 SET_ZERO 命令。
    """

    def __init__(
        self,
        config: GripperConfig,
        *,
        transport: CanTransport | None = None,
        event_callback: EventCallback | None = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        config.validate()
        self.config = config
        self._transport = transport
        self._owns_transport = transport is None
        self._event_callback = event_callback
        self._monotonic = monotonic
        self._operation_lock = threading.Lock()
        self._latest_state: GripperState | None = None
        self._calibration: GripperCalibration | None = None
        if config.calibration_path is not None:
            self._calibration = load_calibration(
                config.calibration_path,
                device_serial=config.device_serial,
                interface=config.interface,
                motor_can_id=config.motor_can_id,
            )

    @property
    def calibration(self) -> GripperCalibration | None:
        return self._calibration

    @property
    def latest_state(self) -> GripperState | None:
        return self._latest_state

    @property
    def connected(self) -> bool:
        return self._transport is not None

    def connect(self) -> "X5Gripper":
        if self._transport is None:
            try:
                self._transport = SocketCanTransport(self.config.interface)
            except OSError as exc:
                raise GripperError(
                    f"无法打开 SocketCAN {self.config.interface}: {exc}"
                ) from exc
        self._emit("connected", interface=self.config.interface)
        return self

    def close(self) -> None:
        transport = self._transport
        if transport is None:
            return
        try:
            transport.send(self.config.motor_can_id, DISABLE_COMMAND)
            self._emit("disabled", reason="close")
        finally:
            if self._owns_transport:
                transport.close()
                self._transport = None

    def __enter__(self) -> "X5Gripper":
        return self.connect()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def emergency_stop(self) -> None:
        """立即向目标 ESC 发送 DISABLE；可重复调用。"""
        transport = self._require_transport()
        transport.send(self.config.motor_can_id, DISABLE_COMMAND)
        self._emit("disabled", reason="emergency_stop")

    def reload_calibration(self, path: str | Path | None = None) -> GripperCalibration:
        source = path if path is not None else self.config.calibration_path
        if source is None:
            raise ValueError("没有指定 calibration_path。")
        self._calibration = load_calibration(
            source,
            device_serial=self.config.device_serial,
            interface=self.config.interface,
            motor_can_id=self.config.motor_can_id,
        )
        return self._calibration

    def capture_stationary_position(self) -> GripperState:
        """在 DISABLE 状态读取一个静止端点，供人工双点标定使用。"""
        self._require_acknowledgement()
        with self._operation_lock:
            transport = self._require_transport()
            claimed_feedback = self._claim_motor()
            try:
                feedback = claimed_feedback
                if feedback is None:
                    transport.send(self.config.motor_can_id, ENABLE_COMMAND)
                    feedback = self._wait_for_feedback(self.config.feedback_timeout_s)
                self._require_stationary(feedback)
                state = self._to_state(feedback)
                self._latest_state = state
                self._emit("calibration_point_captured", position_rad=state.position_rad)
                return state
            finally:
                transport.send(self.config.motor_can_id, DISABLE_COMMAND)

    def hold(self, duration_s: float = 0.5) -> MotionResult:
        """保持使能时测得的位置，随后自动失能。"""
        self._validate_duration(duration_s)
        return self._run_motion(
            operation="hold",
            delta_rad=0.0,
            speed_rad_s=0.15,
            ff_torque_nm=0.0,
            kp=self.config.kp,
            duration_after_s=duration_s,
        )

    def move_to_opening(
        self,
        opening: float,
        *,
        speed_rad_s: float = 0.15,
        duration_after_s: float = 0.2,
    ) -> MotionResult:
        """按时间线性轨迹运动到绝对归一化开度。

        ``opening=0`` 表示标定闭合端，``opening=1`` 表示标定张开端。
        该接口必须先加载双端点标定；轨迹为
        ``q(t) = q_start + sign(target-start) * speed_rad_s * t``，并在目标处截断。
        """
        calibration = self._require_calibration()
        value = float(opening)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError("opening 必须是 [0, 1] 内的有限数值。")
        self._validate_speed(speed_rad_s)
        self._validate_duration(duration_after_s)
        return self._run_motion(
            operation="move_to_opening",
            target_position_rad=calibration.position_for_opening(value),
            speed_rad_s=float(speed_rad_s),
            ff_torque_nm=0.0,
            kp=self._require_position_kp(),
            duration_after_s=duration_after_s,
        )

    def open_linearly(
        self,
        *,
        speed_rad_s: float = 0.15,
        duration_after_s: float = 0.2,
    ) -> MotionResult:
        """沿线性位置轨迹运动到标定张开端。"""
        return self.move_to_opening(
            1.0,
            speed_rad_s=speed_rad_s,
            duration_after_s=duration_after_s,
        )

    def open_linearly_while(
        self,
        continue_motion: Callable[[], bool],
        *,
        speed_rad_s: float = 0.15,
    ) -> MotionResult:
        """按线性位置轨迹张开，直到回调返回 False 或到达标定张开端。"""
        calibration = self._require_calibration()
        self._validate_speed(speed_rad_s)
        return self._run_motion(
            operation="open_linearly_while",
            target_position_rad=calibration.open_position_rad,
            speed_rad_s=float(speed_rad_s),
            ff_torque_nm=0.0,
            kp=self._require_position_kp(),
            duration_after_s=0.05,
            continue_motion=continue_motion,
        )

    def close_linearly(
        self,
        *,
        speed_rad_s: float = 0.15,
        duration_after_s: float = 0.2,
    ) -> MotionResult:
        """沿线性位置轨迹运动到标定闭合端。"""
        return self.move_to_opening(
            0.0,
            speed_rad_s=speed_rad_s,
            duration_after_s=duration_after_s,
        )

    def close_linearly_while(
        self,
        continue_motion: Callable[[], bool],
        *,
        speed_rad_s: float = 0.15,
    ) -> MotionResult:
        """按线性位置轨迹闭合，直到回调返回 False 或到达标定闭合端。"""
        calibration = self._require_calibration()
        self._validate_speed(speed_rad_s)
        return self._run_motion(
            operation="close_linearly_while",
            target_position_rad=calibration.closed_position_rad,
            speed_rad_s=float(speed_rad_s),
            ff_torque_nm=0.0,
            kp=self._require_position_kp(),
            duration_after_s=0.05,
            continue_motion=continue_motion,
        )

    def close_relative(
        self,
        distance_rad: float,
        *,
        speed_rad_s: float = 0.15,
        duration_after_s: float = 0.2,
        torque_nm: float | None = None,
        continue_motion: Callable[[], bool] | None = None,
    ) -> MotionResult:
        """微动接口：向已验证的正编码器方向相对闭合。

        默认使用限速位置斜坡。传入正 torque_nm 时使用 kp=0 的力矩微动。
        ``continue_motion`` 返回 False 时立即停止并失能，用于按住按键连续运动。
        """
        distance = self._validate_distance(distance_rad, self.config.max_close_nudge_rad, "闭合")
        self._validate_duration(duration_after_s)
        if torque_nm is None:
            self._validate_speed(speed_rad_s)
            if self.config.kp <= 0.0:
                raise ValueError("位置闭合要求配置 kp > 0；也可传入正 torque_nm。")
            kp = self.config.kp
            torque = 0.0
        else:
            torque = float(torque_nm)
            if not 0.0 < torque <= self.config.max_ff_torque_nm:
                raise ValueError("闭合 torque_nm 必须为允许范围内的正数。")
            kp = 0.0
        return self._run_motion(
            operation="close_relative",
            delta_rad=distance,
            speed_rad_s=float(speed_rad_s),
            ff_torque_nm=torque,
            kp=kp,
            duration_after_s=duration_after_s,
            continue_motion=continue_motion,
        )

    def open_relative(
        self,
        distance_rad: float,
        *,
        torque_nm: float = -0.10,
        duration_after_s: float = 0.2,
        continue_motion: Callable[[], bool] | None = None,
    ) -> MotionResult:
        """微动接口：用 kp=0 和负前馈力矩向负编码器方向相对张开。"""
        distance = self._validate_distance(distance_rad, self.config.max_open_nudge_rad, "张开")
        self._validate_duration(duration_after_s)
        torque = float(torque_nm)
        if not -self.config.max_ff_torque_nm <= torque < 0.0:
            raise ValueError("张开 torque_nm 必须为允许范围内的负数。")
        return self._run_motion(
            operation="open_relative",
            delta_rad=-distance,
            speed_rad_s=0.15,
            ff_torque_nm=torque,
            kp=0.0,
            duration_after_s=duration_after_s,
            continue_motion=continue_motion,
        )

    def _run_motion(
        self,
        *,
        operation: str,
        delta_rad: float | None = None,
        target_position_rad: float | None = None,
        speed_rad_s: float,
        ff_torque_nm: float,
        kp: float,
        duration_after_s: float,
        continue_motion: Callable[[], bool] | None = None,
    ) -> MotionResult:
        if (delta_rad is None) == (target_position_rad is None):
            raise ValueError("必须且只能指定 delta_rad 或 target_position_rad。")
        self._require_acknowledgement()
        with self._operation_lock:
            transport = self._require_transport()
            self._claim_motor()
            enabled = False
            try:
                transport.send(self.config.motor_can_id, ENABLE_COMMAND)
                enabled = True
                before = self._wait_for_feedback(self.config.feedback_timeout_s)
                self._require_stationary(before)
                start = float(before.position)
                target = (
                    start + float(delta_rad)
                    if delta_rad is not None
                    else float(target_position_rad)
                )
                delta = target - start
                calibration = self._calibration
                if calibration is not None:
                    if calibration.open_direction_sign >= 0.0 and delta != 0.0:
                        raise GripperSafetyError(
                            "本标定的开方向不是负编码器方向，与当前已验证控制模型不一致。"
                        )
                    if not calibration.contains(start, margin_rad=0.02):
                        raise GripperSafetyError("当前位置在标定行程之外。")
                    if not calibration.contains(target, margin_rad=0.005):
                        raise GripperSafetyError("动作目标将越过标定端点。")
                if kp > 0.0 and max(abs(start), abs(target)) > self.config.max_position_kp_abs_rad:
                    raise GripperSafetyError(
                        "线性轨迹离 MIT 零点过远，禁止使用位置 kp；请检查标定或使用低力矩微动。"
                    )

                torque_mode = kp == 0.0 and abs(ff_torque_nm) > 0.0
                slew_s = 0.0 if torque_mode or delta == 0.0 else abs(delta) / speed_rad_s
                total_s = slew_s + duration_after_s
                hold_to_move = continue_motion is not None and torque_mode
                hold_timeout_s = max(2.0, abs(delta) / 0.02) if hold_to_move else total_s
                period = 1.0 / self.config.refresh_hz
                command_position = start
                command_torque = ff_torque_nm if torque_mode else 0.0
                payload = pack_mit_command(
                    command_position,
                    0.0,
                    kp,
                    self.config.kd,
                    command_torque,
                )
                self._drain()
                transport.send(self.config.motor_can_id, payload)
                started = self._monotonic()
                last_sent = started
                last_feedback_at = started
                latest = before
                torque_reached = False
                torque_reached_at: float | None = None
                max_velocity = 0.0
                max_excursion = 0.0
                max_tracking = 0.0
                sample_count = 0
                stopped_by_request = False
                self._emit("motion_started", operation=operation, start=start, target=target)

                while True:
                    now = self._monotonic()
                    elapsed = now - started
                    if continue_motion is not None and not continue_motion():
                        stopped_by_request = True
                        break
                    if torque_mode:
                        measured_now = float(latest.position)
                        if not torque_reached and (
                            (delta < 0.0 and measured_now <= target)
                            or (delta > 0.0 and measured_now >= target)
                        ):
                            torque_reached = True
                            torque_reached_at = now
                        command_position = start
                        command_torque = 0.0 if torque_reached else ff_torque_nm
                    else:
                        command_position = self._slew(start, delta, speed_rad_s, elapsed)
                        command_torque = 0.0

                    if now - last_sent >= period:
                        payload = pack_mit_command(
                            command_position,
                            0.0,
                            kp,
                            self.config.kd,
                            command_torque,
                        )
                        transport.send(self.config.motor_can_id, payload)
                        last_sent = now

                    received = transport.recv(min(0.02, period))
                    received_at = self._monotonic()
                    if received is not None:
                        feedback = self._matching_feedback(*received)
                        if feedback is not None:
                            latest = feedback
                            last_feedback_at = received_at
                        elif self._is_target_mit_command(*received):
                            raise GripperSafetyError("检测到其他控制器正在向夹爪发送 MIT 命令。")
                    if received_at - last_feedback_at > self.config.feedback_watchdog_s:
                        raise GripperSafetyError("夹爪反馈超时，已失能。")

                    measured = float(latest.position)
                    velocity = abs(float(latest.velocity))
                    excursion = abs(measured - start)
                    tracking = abs(measured - command_position)
                    max_velocity = max(max_velocity, velocity)
                    max_excursion = max(max_excursion, excursion)
                    max_tracking = max(max_tracking, tracking)
                    sample_count += 1
                    velocity_limit = (
                        self.config.max_torque_velocity_rad_s
                        if torque_mode
                        else self.config.max_nudge_velocity_rad_s
                        if delta != 0.0
                        else self.config.max_hold_velocity_rad_s
                    )
                    if not math.isfinite(velocity) or velocity > velocity_limit:
                        raise GripperSafetyError("夹爪速度超过安全限制。")
                    if delta > 0.0:
                        if measured < start - self.config.max_wrong_way_rad:
                            raise GripperSafetyError("闭合动作向错误方向运动。")
                        if measured > target + self.config.max_overshoot_rad:
                            raise GripperSafetyError("闭合动作超过目标。")
                    elif delta < 0.0:
                        if measured > start + self.config.max_wrong_way_rad:
                            raise GripperSafetyError("张开动作向错误方向运动。")
                        if measured < target - self.config.max_overshoot_rad:
                            raise GripperSafetyError("张开动作超过目标。")
                    elif excursion > self.config.max_hold_excursion_rad:
                        raise GripperSafetyError("保持期间夹爪偏离初始位置。")
                    if delta != 0.0 and not torque_mode and tracking > self.config.max_tracking_error_rad:
                        raise GripperSafetyError("位置跟踪误差超过安全限制。")
                    if hold_to_move:
                        if (
                            torque_reached
                            and torque_reached_at is not None
                            and now - torque_reached_at >= duration_after_s
                        ):
                            break
                        if not torque_reached and elapsed >= hold_timeout_s:
                            raise GripperSafetyError("连续微动超时，已失能。")
                    elif received_at - started >= total_s:
                        break

                final_state = self._to_state(latest)
                self._latest_state = final_state
                target_tolerance = 0.02
                target_reached = (
                    delta == 0.0
                    or (
                        delta > 0.0
                        and final_state.position_rad >= target - target_tolerance
                    )
                    or (
                        delta < 0.0
                        and final_state.position_rad <= target + target_tolerance
                    )
                )
                result = MotionResult(
                    operation=operation,
                    start_position_rad=start,
                    target_position_rad=target,
                    final_state=final_state,
                    max_velocity_rad_s=max_velocity,
                    max_excursion_rad=max_excursion,
                    max_tracking_error_rad=max_tracking,
                    sample_count=sample_count,
                    target_reached=target_reached,
                    stopped_by_request=stopped_by_request,
                )
                self._emit("motion_finished", **result.as_dict())
                return result
            finally:
                if enabled:
                    transport.send(self.config.motor_can_id, DISABLE_COMMAND)
                    self._emit("disabled", reason="motion_finished_or_aborted")

    def _claim_motor(self) -> MitFeedback | None:
        transport = self._require_transport()
        self._drain()
        transport.send(self.config.motor_can_id, DISABLE_COMMAND)
        deadline = self._monotonic() + self.config.claim_listen_s
        foreign: list[bytes] = []
        latest: MitFeedback | None = None
        while self._monotonic() < deadline:
            received = transport.recv(min(0.02, deadline - self._monotonic()))
            if received is None:
                continue
            feedback = self._matching_feedback(*received)
            if feedback is not None:
                latest = feedback
            elif self._is_target_mit_command(*received):
                foreign.append(bytes(received[1]))
        if foreign:
            decoded = [unpack_mit_command(payload) for payload in foreign]
            low_kp_repeat = (
                len(set(foreign)) == 1
                and max(float(item["kp"]) for item in decoded) < 1.0
            )
            if not (self.config.allow_residual_low_kp_command and low_kp_repeat):
                raise GripperSafetyError("目标 CAN ID 上仍有其他控制器的 MIT 命令。")
        return latest

    def _wait_for_feedback(self, timeout_s: float) -> MitFeedback:
        transport = self._require_transport()
        deadline = self._monotonic() + timeout_s
        while self._monotonic() < deadline:
            received = transport.recv(min(0.02, deadline - self._monotonic()))
            if received is None:
                continue
            feedback = self._matching_feedback(*received)
            if feedback is not None:
                return feedback
        raise GripperSafetyError("未收到与夹爪电机 ID 匹配的 MIT 反馈。")

    def _matching_feedback(self, can_id: int, data: bytes) -> MitFeedback | None:
        if not looks_like_feedback(data):
            return None
        feedback = unpack_mit_feedback(can_id, data)
        if feedback.motor_id != (self.config.motor_can_id & 0xF):
            return None
        if self.config.feedback_can_id is not None and feedback.can_id != self.config.feedback_can_id:
            return None
        return feedback

    def _is_target_mit_command(self, can_id: int, data: bytes) -> bool:
        return (
            int(can_id) == self.config.motor_can_id
            and len(data) == 8
            and not is_system_command(data)
            and not looks_like_feedback(data)
        )

    def _to_state(self, feedback: MitFeedback) -> GripperState:
        opening = None
        if self._calibration is not None:
            opening = self._calibration.normalized_opening(feedback.position)
        return GripperState(
            can_id=feedback.can_id,
            motor_id=feedback.motor_id,
            status=feedback.status,
            position_rad=feedback.position,
            velocity_rad_s=feedback.velocity,
            torque_nm=feedback.torque,
            mos_temp_c=feedback.mos_temp_c,
            rotor_temp_c=feedback.rotor_temp_c,
            opening=opening,
        )

    def _require_stationary(self, feedback: MitFeedback) -> None:
        if not math.isfinite(feedback.position) or not math.isfinite(feedback.velocity):
            raise GripperSafetyError("夹爪反馈不是有限数值。")
        if abs(feedback.velocity) > self.config.stable_velocity_rad_s:
            raise GripperSafetyError("动作前夹爪尚未静止。")

    def _drain(self, maximum: int = 4096) -> None:
        transport = self._require_transport()
        for _ in range(maximum):
            if transport.recv(0.0) is None:
                break

    def _require_transport(self) -> CanTransport:
        if self._transport is None:
            raise GripperError("尚未连接夹爪；请先调用 connect()。")
        return self._transport

    def _require_acknowledgement(self) -> None:
        if not self.config.acknowledge_unverified_hardware:
            raise GripperSafetyError(
                "当前 CAN ID、增益和方向仍需逐机确认；请完成风险确认后设置 "
                "acknowledge_unverified_hardware=True。"
            )

    def _require_calibration(self) -> GripperCalibration:
        if self._calibration is None:
            raise GripperSafetyError("线性绝对开度控制要求先加载双端点标定文件。")
        return self._calibration

    def _require_position_kp(self) -> float:
        if self.config.kp <= 0.0:
            raise ValueError("线性位置控制要求配置 kp > 0。")
        return self.config.kp

    @staticmethod
    def _slew(start: float, delta: float, speed: float, elapsed: float) -> float:
        if delta == 0.0:
            return start
        traveled = min(max(0.0, elapsed) * speed, abs(delta))
        return start + math.copysign(traveled, delta)

    @staticmethod
    def _validate_duration(duration_s: float) -> None:
        if not math.isfinite(float(duration_s)) or not 0.05 <= float(duration_s) <= 10.0:
            raise ValueError("duration 必须在 [0.05, 10] 秒。")

    @staticmethod
    def _validate_distance(distance: float, maximum: float, name: str) -> float:
        value = float(distance)
        if not math.isfinite(value) or not 0.0 < value <= maximum:
            raise ValueError(f"{name}距离必须在 (0, {maximum}] rad。")
        return value

    def _validate_speed(self, speed: float) -> None:
        value = float(speed)
        if not math.isfinite(value) or not 0.0 < value <= self.config.max_nudge_speed_rad_s:
            raise ValueError(
                f"speed_rad_s 必须在 (0, {self.config.max_nudge_speed_rad_s}]。"
            )

    def _emit(self, event: str, **fields: object) -> None:
        if self._event_callback is not None:
            self._event_callback(event, fields)
