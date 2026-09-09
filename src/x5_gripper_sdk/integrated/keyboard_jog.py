"""Bounded Cartesian keyboard jogging for one X5-2023 arm.

Each motion key requests a 1 cm end-effector translation in the SDK base frame
while preserving the latest measured orientation. The target must remain within
80 cm of the startup position and pass the vendor IK and joint-limit checks. The
resulting motion is sent as a 50 Hz straight Cartesian target stream whose target
advances at no more than 1 cm/s. Each Cartesian sample is converted through the
vendor IK before motion begins; the vendor's direct Cartesian command is
intentionally not used. The tool never homes the arm. Gripper commands remain
disabled unless the separate effort0 experiment is explicitly enabled.

The source is organized by responsibility rather than execution order. See
``docs/arx_keyboard_jog_code_index.md`` for symbol, key-command, JSONL-event,
and future module-extraction indexes.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import signal
import sys
import termios
import threading
import time
import tty
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from statistics import median
from typing import Any, Callable, Iterator, Sequence, TextIO

import numpy as np

from .arx_wrench_model import (
    ARX_ARM_JOINT_NAMES,
    ARX_FINGER_JOINT_NAMES,
    ARXWrenchModel,
)
from . import arm_backend as backend
from .session_log import resolve_session_log_path
from .wrench_estimation import WrenchEstimateConfig, estimate_wrench


# =============================================================================
# 1. Safety limits, experiment parameters, and operator command constants
# =============================================================================

# 六个机械臂关节的软件允许范围，单位为 rad；IK 结果超出范围时拒绝运动。
JOINT_LIMITS_RAD = (
    (-2.617994, 3.1416),
    (0.0, 3.665),
    (0.0, 3.141593),
    (-1.571, 1.571),
    (-1.571, 1.571),
    (-2.094395, 2.094),
)
# 18:46 can2 rest pose: J2 feedback is −0.00248 rad. The first 1 cm R
# waypoint IK returned −0.001709. 18:50 H/S near startup IK returned
# −0.010 to −0.0125 (FK/IK of the rest pose is not an exact inverse).
# Clip milliradian encoder/IK bias onto the bound; still reject real
# limit violations.
IK_JOINT_LIMIT_SLACK_RAD = 0.020

# 每按一次 W/S/A/D/R/F 时，末端目标沿对应轴移动 0.01 m（1 cm）。
CARTESIAN_STEP_M = 0.01

# 直线目标轨迹的推进速度，单位为 m/s；这是发送目标的速度，不是厂家闭环保证值。
CARTESIAN_SPEED_M_S = 0.01

# 末端相对本次启动位置允许的最大三维直线位移，0.80 m 即 80 cm。
MAX_DISPLACEMENT_M = 0.80

# 单次 IK 解相对当前反馈允许的最大关节差，用于拒绝 IK 跳到另一解支。
MAX_IK_JOINT_STEP_RAD = 0.20

# 运动完成时允许的末端位置误差，0.002 m 即 2 mm。
TARGET_TOLERANCE_M = 0.002

# 关节位置目标的发送周期，0.02 s 对应 50 Hz。
CARTESIAN_COMMAND_PERIOD_S = 0.02

# 规划轨迹中任一关节目标允许的最大变化速率，单位为 rad/s。
MAX_JOINT_COMMAND_RATE_RAD_S = 0.20

# 每个20 ms周期允许的最大关节目标增量；由目标速率上限自动计算，目前为0.004 rad。
MAX_JOINT_COMMAND_STEP_RAD = (
    MAX_JOINT_COMMAND_RATE_RAD_S * CARTESIAN_COMMAND_PERIOD_S
)

# A feedback value may sit a few milliradians outside a software joint bound
# because of the encoder/IK zero bias accepted by IK_JOINT_LIMIT_SLACK_RAD.
# Before Cartesian planning, slew the position target back onto that bound and
# wait for measured velocity to settle. Keep this short because the correction
# is at most the existing 0.02 rad slack.
JOINT_LIMIT_NORMALIZATION_TIMEOUT_S = 1.0

# 正常运动阶段实测任一关节速度的安全停止阈值。
MAX_MEASURED_VELOCITY_RAD_S = 0.50

# 等待键盘输入时，速度反馈必须连续超限这么多帧才进入保护，用于过滤厂家诊断中的
# 单帧量化/通信尖峰。轨迹执行阶段仍保持单帧超限立即保护。
KEYBOARD_IDLE_OVERSPEED_CONSECUTIVE_SAMPLES = 3

# 到达目标后，需要连续多少个稳定样本才判定运动结束。
MOTION_SETTLE_STABLE_SAMPLES = 3

# 进入厂家位置模式后，至少观察这么长时间再允许判定稳定。
POSITION_MODE_WARMUP_S = 0.5

# 位置模式预热允许的最长时间，超时仍未稳定就停止。
POSITION_MODE_WARMUP_MAX_S = 2.0

# 位置模式预热结束前要求的连续稳定样本数。
POSITION_MODE_WARMUP_STABLE_SAMPLES = 5

# 位置模式切换期间任一关节相对切换起点允许的最大偏移，单位为 rad。
POSITION_MODE_WARMUP_MAX_EXCURSION_RAD = 0.040

# status=5 切换后的特殊瞬态观察窗口，0.300 s 后恢复正常速度保护。
POSITION_MODE_TRANSITION_OBSERVATION_S = 0.300

# 上述瞬态窗口内使用的临时关节速度阈值，单位为 rad/s。
POSITION_MODE_TRANSITION_MAX_VELOCITY_RAD_S = 0.6

# 瞬态窗口内必须连续这么多个样本超速，才触发安全停止。
POSITION_MODE_TRANSITION_CONSECUTIVE_SAMPLES = 3

# status=5初始化时夹爪相对切换前反馈允许的最大漂移；该限制不作用于后续明确按键运动。
POSITION_MODE_GRIPPER_MAX_HOLD_EXCURSION_RAW = 0.10

# 启动读取反馈时至少等待的时间，用于避开厂家 SDK 的延迟初值。
FEEDBACK_WARMUP_MIN_S = 2.0

# 反馈预热期间，若提供 idle_pump，按这个间隔刷新夹爪 MIT，避免 50 ms 一发盖不过 Catch。
FEEDBACK_WARMUP_PUMP_PERIOD_S = 0.01

# 启动反馈预热的最大等待时间，超过后仍不稳定就停止。
FEEDBACK_WARMUP_MAX_S = 5.0

# 判断机械臂“已静止”的关节速度阈值，单位为 rad/s。
FEEDBACK_STABLE_VELOCITY_RAD_S = 0.05

# 启动反馈必须连续达到的稳定样本数。
FEEDBACK_STABLE_SAMPLES = 5

# 切换位置模式前等待关节反馈恢复静止的最长时间。
POSITION_MODE_PRECHECK_MAX_S = 2.0

# 位置模式切换前稳定性检查的采样周期，0.02 s 即 50 Hz。
POSITION_MODE_PRECHECK_PERIOD_S = 0.02

# 厂家夹爪位置命令的原始最小值；当前实机方向中 0 表示闭合端。
GRIPPER_RAW_MIN = 0.0

# 厂家夹爪位置命令的原始最大值；当前实机方向中 5 表示张开端。
GRIPPER_RAW_MAX = 5.0

# O 键默认发送的完全张开目标，可由 --gripper-open-raw 覆盖。
GRIPPER_DEFAULT_OPEN_RAW = GRIPPER_RAW_MAX

# C 键允许到达的默认最终闭合目标，可由 --gripper-closed-raw 覆盖。
GRIPPER_DEFAULT_CLOSED_RAW = GRIPPER_RAW_MIN

# 连续闭合时每步的原始位置增量；它控制步长，不是夹爪闭合速度。
GRIPPER_RAW_STEP = 0.25

# 暂定单侧手指行程，与 robot.yml / MuJoCo 的 0.044 m 一致；对侧 equality 锁定，
# 开口变化约为两侧之和。夹爪尚未完成开合标定，1 mm 按该暂定开口换算。
GRIPPER_FINGER_STROKE_M = 0.044

# 检测到接触后再闭合的开口量，1 mm。
GRIPPER_CONTACT_EXTRA_CLOSE_M = 0.001

# 1 mm 开口对应的 raw 增量；raw 0–5 对应两侧合计 2 × 0.044 m。
GRIPPER_CONTACT_EXTRA_CLOSE_RAW = (
    GRIPPER_CONTACT_EXTRA_CLOSE_M
    / (2.0 * GRIPPER_FINGER_STROKE_M)
    * (GRIPPER_RAW_MAX - GRIPPER_RAW_MIN)
)

# 判断夹爪是否到达目标时允许的原始位置误差。闭合步使用该值，并叠加已测偏移。
GRIPPER_RAW_POSITION_TOLERANCE = 0.10

# 首次完全张开时尚未测到命令—反馈偏移；实机全开反馈约 4.90，相对命令 5
# 差约 0.10，0.10 容差会贴边失败。张开到位单独使用更宽容差。
GRIPPER_OPEN_POSITION_TOLERANCE = 0.20

# 判断夹爪是否停止时的反馈速度阈值；它不是发送给厂家的速度命令。
GRIPPER_RAW_STABLE_VELOCITY = 0.05

# O 键建立空载 effort0 基线前要求的连续稳定样本数。
GRIPPER_STABLE_SAMPLES = 10

# 夹爪位置、速度和 effort0 的监测周期，0.02 s 即 50 Hz；不是闭合速度。
GRIPPER_MONITOR_PERIOD_S = 0.02

# 一次张开或闭合命令等待反馈到位的最长时间。全开从 0 走到 5 后还需 10 个稳定样本。
GRIPPER_COMMAND_TIMEOUT_S = 8.0

# O 键张开期间的六轴关节速度反馈必须连续超限这么多次才停止，用于过滤单帧尖峰。
# 正常机械臂轨迹仍保持单样本超速立即停止。
GRIPPER_OPEN_ARM_OVERSPEED_CONSECUTIVE_SAMPLES = 3

# O 键张开会释放夹爪负载，六轴机械臂可能出现短时速度瞬态。仅在 O 阶段把连续
# 超速阈值放宽到 1.0 rad/s；正常机械臂轨迹仍使用 MAX_MEASURED_VELOCITY_RAD_S。
GRIPPER_OPEN_MAX_ARM_VELOCITY_RAD_S = 1.0

# 放宽 O 阶段速度阈值后，任一机械臂关节相对按 O 前位置的偏移仍不得超过该值。
# 该位置保护不等待连续样本，且不高于离线标定相对 tare 的默认 0.05 rad 姿态门槛。
GRIPPER_OPEN_MAX_ARM_EXCURSION_RAD = 0.050

# 只有夹爪速度低于该值时，普通过流才可能被判定为接触而非启动 effort0。
GRIPPER_CONTACT_MAX_VELOCITY_RAW_S = 0.05

# 普通过流需要连续达到的样本数，避免单次 effort0 尖峰误触发。
GRIPPER_CONTACT_CONSECUTIVE_SAMPLES = 3

# 不考虑夹爪速度的紧急 effort0 增量阈值，达到后立即回退并保护。
GRIPPER_EMERGENCY_CURRENT_DELTA_RAW = 1.5

# MuJoCo 模型中用于计算夹爪整体外力雅可比的末端 site 名称。
MCC_FORCE_SITE_NAME = "ee_site"

# MCC 雅可比反解的正则化系数；增大更稳定，但会压低外力代理值。
MCC_FORCE_REGULARIZATION = 1e-2

# 六轴 effort 指数平滑系数；越大响应越快，同时也越容易保留噪声。
MCC_FORCE_EFFORT_EMA_ALPHA = 0.1

# 由速度差分得到的关节加速度指数平滑系数。
MCC_FORCE_ACCELERATION_EMA_ALPHA = 0.2

# 加速度估算的数值限幅，单位为 rad/s²；这是估算保护，不是运动速度限制。
MCC_FORCE_MAX_ACCELERATION_RAD_S2 = 20.0

# 相邻观测超过该间隔则丢弃 effort/加速度平滑，避免空闲按 G 带上运动残留。
MCC_FORCE_SAMPLE_GAP_S = 0.2

# MCC 输出中区分 moving/stationary 的关节速度阈值，单位为 rad/s。
MCC_FORCE_STATIONARY_VELOCITY_RAD_S = 0.05

# 实机运动前要求操作者输入的确认文字；目前只需输入数字 1。
MOTION_CONFIRMATION = "1"

# 自动已知载荷标定每组默认保存的 G 样本数。
FORCE_CALIBRATION_DEFAULT_SAMPLES_PER_LOAD = 20

# 用户现有 100/200/500 g 砝码可组成的加载—卸载序列。
# 每项依次为：加载方向、盒内砝码总质量、给操作者的实物摆放提示。
FORCE_CALIBRATION_LOAD_PLAN = (
    ("up", 0.0, "保持盒子为空，不放砝码"),
    ("up", 100.0, "盒内只放 100 g 砝码"),
    ("up", 200.0, "取出 100 g，盒内只放 200 g 砝码"),
    ("up", 300.0, "盒内放 100 g + 200 g 砝码"),
    ("up", 500.0, "取出 100 g 和 200 g，盒内只放 500 g 砝码"),
    ("up", 600.0, "盒内放 500 g + 100 g 砝码"),
    ("up", 700.0, "取出 100 g，盒内放 500 g + 200 g 砝码"),
    ("up", 800.0, "盒内放 500 g + 200 g + 100 g 砝码"),
    ("down", 700.0, "取出 100 g，盒内保留 500 g + 200 g 砝码"),
    ("down", 600.0, "取出 200 g、放入 100 g，盒内为 500 g + 100 g"),
    ("down", 500.0, "取出 100 g，盒内只保留 500 g 砝码"),
    ("down", 300.0, "取出 500 g，盒内放 200 g + 100 g 砝码"),
    ("down", 200.0, "取出 100 g，盒内只保留 200 g 砝码"),
    ("down", 100.0, "取出 200 g、放入 100 g，盒内只放 100 g 砝码"),
    ("down", 0.0, "取出全部砝码，保持空盒"),
)

# 按键到“笛卡尔轴索引、方向”的映射：0/1/2 分别为 X/Y/Z，±1 为正反方向。
KEY_DIRECTIONS = {
    "w": (0, 1),
    "s": (0, -1),
    "a": (1, 1),
    "d": (1, -1),
    "r": (2, 1),
    "f": (2, -1),
}

# 厂家 SDK 会反复打印的横幅；日志过滤器只隐藏这一整行文字。
VENDOR_BANNER = "ARX方舟无限"


# =============================================================================
# 2. Shared errors and known-load force-calibration metadata
# =============================================================================

class JogSafetyError(RuntimeError):
    """Raised when a Cartesian target violates a jogging safety bound."""


class GripperOpenTimeoutError(JogSafetyError):
    """Raised when the gripper does not settle at the open target in time."""


@dataclass(frozen=True)
class ForceCalibrationLabel:
    """Operator-entered metadata attached to each subsequent G sample."""

    sample_label: str
    box_mass_g: float
    added_mass_g: float
    pose_label: str
    note: str = ""

    def as_log_record(self) -> dict[str, Any]:
        total_mass_g = self.box_mass_g + self.added_mass_g
        return {
            "sample_label": self.sample_label,
            "box_mass_g": self.box_mass_g,
            "added_mass_g": self.added_mass_g,
            "total_suspended_mass_g": total_mass_g,
            "expected_gravity_force_n": total_mass_g / 1000.0 * 9.80665,
            "pose_label": self.pose_label,
            "note": self.note,
            "gravity_constant_m_s2": 9.80665,
            "mass_convention": "box_plus_added_mass_suspended_by_gripper",
        }


@dataclass
class AutomaticForceCalibration:
    """State machine for a confirmed 20-sample known-load sequence."""

    box_mass_g: float
    pose_label: str
    samples_per_load: int = FORCE_CALIBRATION_DEFAULT_SAMPLES_PER_LOAD
    step_index: int = 0
    samples_in_step: int = 0
    awaiting_confirmation: bool = True
    completed: bool = False

    @property
    def step_count(self) -> int:
        return len(FORCE_CALIBRATION_LOAD_PLAN)

    @property
    def current_step(self) -> tuple[str, float, str] | None:
        if self.completed or self.step_index >= self.step_count:
            return None
        return FORCE_CALIBRATION_LOAD_PLAN[self.step_index]

    def instruction(self) -> str:
        step = self.current_step
        if step is None:
            return "自动标定序列已完成。"
        direction, added_mass_g, instruction = step
        direction_text = "加载" if direction == "up" else "卸载"
        return (
            f"第 {self.step_index + 1}/{self.step_count} 组（{direction_text}，"
            f"砝码总质量 {added_mass_g:g} g）：{instruction}。"
            "放稳并等待盒子静止后按 N 确认；确认前 G 不会保存。"
        )

    def confirm_current_step(self) -> ForceCalibrationLabel:
        step = self.current_step
        if step is None:
            raise ValueError("自动标定序列已经完成。")
        direction, added_mass_g, instruction = step
        self.awaiting_confirmation = False
        self.samples_in_step = 0
        pose_prefix = self.pose_label.split("_", 1)[0] or "pose"
        return ForceCalibrationLabel(
            sample_label=(
                f"{pose_prefix}_{direction}_w{int(round(added_mass_g)):03d}_r01"
            ),
            box_mass_g=self.box_mass_g,
            added_mass_g=added_mass_g,
            pose_label=self.pose_label,
            note=f"automatic_sequence: {instruction}",
        )

    def sample_metadata(self) -> dict[str, Any]:
        if self.completed or self.awaiting_confirmation:
            raise ValueError("当前自动标定组尚未由操作者确认。")
        return {
            "automatic": True,
            "group_index": self.step_index + 1,
            "group_count": self.step_count,
            "sample_index": self.samples_in_step + 1,
            "samples_per_group": self.samples_per_load,
        }

    def record_sample(self) -> bool:
        """Record one G sample; return True when the current group completes."""
        if self.completed or self.awaiting_confirmation:
            raise ValueError("当前自动标定组尚未由操作者确认。")
        self.samples_in_step += 1
        if self.samples_in_step < self.samples_per_load:
            return False
        self.step_index += 1
        self.samples_in_step = 0
        self.awaiting_confirmation = True
        if self.step_index >= self.step_count:
            self.completed = True
        return True


# =============================================================================
# 3. Read-only MCC end-effector force proxy
# =============================================================================

class MCCEndEffectorForceEstimator:
    """Read-only MCC wrench estimator for the complete gripper assembly.

    The first six SDK effort channels are treated as provisional joint torques.
    They are not calibrated on X5-2023 yet, so every result is deliberately
    labelled as a proxy rather than Newtons.  MuJoCo inverse dynamics removes
    the modelled gravity, velocity and acceleration torque before the MCC
    Jacobian solve.  Gripper equality, contact, joint-limit, actuator and
    Coulomb-friction forces are disabled for that inverse so finger internals
    and velocity-sign chatter cannot leak onto the arm.  Stationary or stale
    samples drop leftover acceleration and effort smoothing.  A stationary
    startup tare removes the remaining joint offsets.
    """

    ARM_JOINT_NAMES = ARX_ARM_JOINT_NAMES
    FINGER_JOINT_NAMES = ARX_FINGER_JOINT_NAMES

    def __init__(
        self,
        xml_path: Path | None = None,
        *,
        fk_solver: Callable[[np.ndarray], Sequence[float]] | None = None,
    ) -> None:
        self._wrench_model = ARXWrenchModel(
            xml_path=xml_path,
            site_name=MCC_FORCE_SITE_NAME,
        )
        self.xml_path = self._wrench_model.xml_path
        self.sim = self._wrench_model.sim
        self._qpos_indices = list(self._wrench_model.qpos_indices)
        # The shared model already returns a six-column arm Jacobian.
        self._dof_indices = list(range(6))
        self._estimate_config = WrenchEstimateConfig(
            force_reg=MCC_FORCE_REGULARIZATION,
            torque_reg=1e-1,
            force_only=True,
            axis_aligned=False,
        )
        self.fk_solver = fk_solver
        self._joint_residual_offset: np.ndarray | None = None
        self._effort_ema: np.ndarray | None = None
        self._acceleration_ema = np.zeros(6, dtype=np.float64)
        self._previous_velocity: np.ndarray | None = None
        self._previous_time: float | None = None

    @staticmethod
    def _arm_vectors(
        state: dict[str, Any],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        arm = state["arm"]
        position = np.asarray(
            _finite_values(
                arm["position_rad"], expected=6, label="joint position"
            ),
            dtype=np.float64,
        )
        velocity = np.asarray(
            _finite_values(
                arm["velocity_rad_s"], expected=6, label="joint velocity"
            ),
            dtype=np.float64,
        )
        effort = np.asarray(
            _finite_values(
                arm["effort_raw"], expected=6, label="joint effort"
            ),
            dtype=np.float64,
        )
        return position, velocity, effort

    def _model_state(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        acceleration: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        state = self._wrench_model.evaluate(position, velocity, acceleration)
        return (
            state.jacobian_position,
            state.jacobian_rotation,
            state.site_rotation_base,
            state.model_torque_nm,
        )

    def joint_sample(self, state: dict[str, Any]) -> dict[str, Any]:
        """Return the joint feedback used for the force calculation."""
        position, velocity, effort = self._arm_vectors(state)
        fk_pose = None
        if self.fk_solver is not None:
            fk_pose = _finite_values(
                self.fk_solver(position.copy()),
                expected=6,
                label="MCC force-sample FK pose",
            )
        return {
            "source": "same_sdk_diagnostic_sample_as_force",
            "position_rad": position.tolist(),
            "position_deg": [
                math.degrees(float(value)) for value in position
            ],
            "velocity_rad_s": velocity.tolist(),
            "effort_raw": effort.tolist(),
            "fk_ee_pose_xyzrpy": fk_pose,
        }

    def tare(self, state: dict[str, Any], *, now: float | None = None) -> None:
        """Set the current unloaded, stationary arm state as zero external force."""
        position, velocity, effort = self._arm_vectors(state)
        if max(abs(float(value)) for value in velocity) > (
            MCC_FORCE_STATIONARY_VELOCITY_RAD_S
        ):
            raise JogSafetyError(
                "MCC end-effector force tare requires joint velocity below "
                f"{MCC_FORCE_STATIONARY_VELOCITY_RAD_S:.3f} rad/s."
            )
        _, _, _, model_torque = self._model_state(
            position,
            np.zeros(6, dtype=np.float64),
            np.zeros(6, dtype=np.float64),
        )
        self._joint_residual_offset = effort - model_torque
        self._effort_ema = effort.copy()
        self._acceleration_ema[:] = 0.0
        self._previous_velocity = velocity.copy()
        self._previous_time = time.monotonic() if now is None else float(now)

    def observe(
        self, state: dict[str, Any], *, now: float | None = None
    ) -> dict[str, Any]:
        """Estimate the net force at the complete gripper and return metadata."""
        if self._joint_residual_offset is None:
            raise JogSafetyError("MCC end-effector force estimator has not been tared.")
        timestamp = time.monotonic() if now is None else float(now)
        position, velocity, effort = self._arm_vectors(state)
        max_velocity = float(np.max(np.abs(velocity)))
        stationary = max_velocity <= MCC_FORCE_STATIONARY_VELOCITY_RAD_S
        dt = (
            None
            if self._previous_time is None
            else timestamp - self._previous_time
        )
        sample_stale = (
            dt is None
            or dt < 0.002
            or dt > MCC_FORCE_SAMPLE_GAP_S
        )

        if self._effort_ema is None or sample_stale:
            self._effort_ema = effort.copy()
        else:
            alpha = MCC_FORCE_EFFORT_EMA_ALPHA
            self._effort_ema = alpha * effort + (1.0 - alpha) * self._effort_ema

        if sample_stale or stationary or self._previous_velocity is None:
            self._acceleration_ema[:] = 0.0
            model_acceleration = np.zeros(6, dtype=np.float64)
        else:
            acceleration = (velocity - self._previous_velocity) / float(dt)
            acceleration = np.clip(
                acceleration,
                -MCC_FORCE_MAX_ACCELERATION_RAD_S2,
                MCC_FORCE_MAX_ACCELERATION_RAD_S2,
            )
            acceleration_alpha = MCC_FORCE_ACCELERATION_EMA_ALPHA
            self._acceleration_ema = (
                acceleration_alpha * acceleration
                + (1.0 - acceleration_alpha) * self._acceleration_ema
            )
            model_acceleration = self._acceleration_ema
        model_velocity = (
            np.zeros(6, dtype=np.float64) if stationary else velocity
        )
        self._previous_velocity = velocity.copy()
        self._previous_time = timestamp

        jacp, jacr, site_rotation, model_torque = self._model_state(
            position, model_velocity, model_acceleration
        )
        residual = self._effort_ema - model_torque - self._joint_residual_offset
        external_joint_proxy = -residual
        wrench_base = estimate_wrench(
            np.asarray(jacp[:, self._dof_indices], dtype=np.float32),
            np.asarray(jacr[:, self._dof_indices], dtype=np.float32),
            np.asarray(external_joint_proxy, dtype=np.float32),
            np.asarray(site_rotation, dtype=np.float32),
            self._estimate_config,
        ).astype(np.float64)
        force_base = wrench_base[:3]
        force_gripper = site_rotation.T @ force_base
        joint_sample = self.joint_sample(state)
        joint_sample["captured_monotonic_s"] = timestamp
        return {
            "force_gripper_proxy": force_gripper.tolist(),
            "force_base_proxy": force_base.tolist(),
            "force_norm_proxy": float(np.linalg.norm(force_gripper)),
            "joint_external_effort_proxy": external_joint_proxy.tolist(),
            "joint_model_torque": model_torque.tolist(),
            "joint_acceleration_rad_s2": self._acceleration_ema.tolist(),
            "max_joint_velocity_rad_s": max_velocity,
            "motion_state": (
                "moving"
                if max_velocity > MCC_FORCE_STATIONARY_VELOCITY_RAD_S
                else "stationary"
            ),
            "dynamic_compensation": "mujoco_inverse_q_qdot_qddot",
            "calibrated": False,
            "unit": "uncalibrated_mcc_force_proxy_not_newton",
            "site": MCC_FORCE_SITE_NAME,
            "frame": "gripper",
            "joint_sample": joint_sample,
        }

    def close(self) -> None:
        self._wrench_model.close()


# =============================================================================
# 4. Experimental gripper effort0 feedback, contact guard, and calibration
# =============================================================================

def _gripper_raw_feedback(state: dict[str, Any]) -> dict[str, Any]:
    """Read the documented seventh X5-2023 vector entries as raw values."""
    lengths = state.get("raw_vector_lengths", {})
    if any(lengths.get(name) != 7 for name in ("position", "velocity", "effort")):
        raise JogSafetyError(
            "Gripper current limiting requires seven-value position, velocity, "
            "and effort0 feedback vectors."
        )
    extra = state.get("additional_sdk_feedback", {})
    values = {
        "position_raw": float(extra.get("position_raw")),
        "velocity_raw": float(extra.get("velocity_raw")),
        "effort0": float(extra.get("effort0")),
        "catch_status": state.get("gripper", {}).get("catch_status"),
    }
    if not all(
        math.isfinite(values[name])
        for name in ("position_raw", "velocity_raw", "effort0")
    ):
        raise JogSafetyError("Gripper raw feedback contains NaN or Inf.")
    return values


@dataclass
class GripperCurrentGuard:
    """Experimental effort0 limiter for the X5-2023 seventh actuator."""

    current_delta_limit_raw: float
    open_raw: float = GRIPPER_DEFAULT_OPEN_RAW
    closed_raw: float = GRIPPER_DEFAULT_CLOSED_RAW
    step_raw: float = GRIPPER_RAW_STEP
    baseline_current_raw: float | None = None
    contact_overcurrent_samples: int = 0
    commanded_position_raw: float | None = None
    command_feedback_offset_raw: float = 0.0
    contact_latched: bool = False
    contact_hold_position_raw: float | None = None
    no_load_curve_path: Path | None = None
    no_load_curve: list[tuple[float, float]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.no_load_curve_path is not None:
            self.no_load_curve_path = Path(self.no_load_curve_path).expanduser().resolve()
            self._load_no_load_curve()

    def _load_no_load_curve(self) -> None:
        path = self.no_load_curve_path
        if path is None or not path.exists():
            return
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            points = [
                (float(item["position_raw"]), float(item["current_delta_raw"]))
                for item in payload["points"]
            ]
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(f"Invalid gripper no-load curve {path}: {exc}") from exc
        if len(points) < 2 or not all(
            math.isfinite(position) and math.isfinite(delta)
            for position, delta in points
        ):
            raise ValueError(
                f"Gripper no-load curve {path} must contain at least two finite points."
            )
        self.no_load_curve = sorted(points)

    def _save_no_load_curve(self) -> None:
        path = self.no_load_curve_path
        if path is None:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "robot_model": "X5-2023",
            "semantics": "closing_no_load_current_delta_from_open_baseline_raw",
            "points": [
                {
                    "position_raw": position,
                    "current_delta_raw": delta,
                }
                for position, delta in self.no_load_curve
            ],
        }
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    @property
    def has_no_load_curve(self) -> bool:
        return len(self.no_load_curve) >= 2

    def expected_no_load_current(self, position_raw: float) -> float:
        if self.baseline_current_raw is None:
            raise JogSafetyError("Open-current baseline is unavailable.")
        if not self.has_no_load_curve:
            return float(self.baseline_current_raw)
        positions = np.asarray(
            [point[0] for point in self.no_load_curve], dtype=np.float64
        )
        deltas = np.asarray(
            [point[1] for point in self.no_load_curve], dtype=np.float64
        )
        expected_delta = float(np.interp(float(position_raw), positions, deltas))
        return float(self.baseline_current_raw) + expected_delta

    @staticmethod
    def _step_toward(current: float, target: float, step: float) -> float:
        delta = target - current
        if abs(delta) <= step:
            return target
        return current + math.copysign(step, delta)

    def next_close_target(self, current: float) -> float:
        return self._step_toward(float(current), self.closed_raw, self.step_raw)

    def contact_extra_close_target(self, current: float) -> float:
        return self._step_toward(
            float(current),
            self.closed_raw,
            GRIPPER_CONTACT_EXTRA_CLOSE_RAW,
        )

    def release_target(self, current: float) -> float:
        return self._step_toward(float(current), self.open_raw, self.step_raw)

    def enforce(
        self,
        controller: Any,
        state: dict[str, Any],
        log_stream: TextIO,
        *,
        context: str,
    ) -> dict[str, Any]:
        feedback = _gripper_raw_feedback(state)
        if self.baseline_current_raw is None:
            return feedback
        open_baseline_delta = abs(
            feedback["effort0"] - self.baseline_current_raw
        )
        expected_no_load_current = self.expected_no_load_current(
            feedback["position_raw"]
        )
        contact_residual = abs(
            feedback["effort0"] - expected_no_load_current
        )
        speed = abs(feedback["velocity_raw"])
        emergency = open_baseline_delta >= GRIPPER_EMERGENCY_CURRENT_DELTA_RAW
        if self.contact_latched and not emergency:
            return feedback
        contact_candidate = (
            contact_residual >= self.current_delta_limit_raw
            and speed <= GRIPPER_CONTACT_MAX_VELOCITY_RAW_S
        )
        if contact_candidate:
            self.contact_overcurrent_samples += 1
        else:
            self.contact_overcurrent_samples = 0
        if contact_residual >= self.current_delta_limit_raw or emergency:
            _write_log(
                log_stream,
                "gripper_current_observation",
                context=context,
                feedback=feedback,
                baseline_current_raw=self.baseline_current_raw,
                open_baseline_current_delta_raw=open_baseline_delta,
                expected_no_load_current_raw=expected_no_load_current,
                contact_current_residual_raw=contact_residual,
                no_load_curve_available=self.has_no_load_curve,
                current_delta_limit_raw=self.current_delta_limit_raw,
                contact_max_velocity_raw_s=(
                    GRIPPER_CONTACT_MAX_VELOCITY_RAW_S
                ),
                contact_candidate=contact_candidate,
                contact_overcurrent_samples=self.contact_overcurrent_samples,
                required_contact_samples=(
                    GRIPPER_CONTACT_CONSECUTIVE_SAMPLES
                ),
                emergency_current_delta_raw=(
                    GRIPPER_EMERGENCY_CURRENT_DELTA_RAW
                ),
                emergency=emergency,
            )
        if not emergency:
            if self.contact_overcurrent_samples < (
                GRIPPER_CONTACT_CONSECUTIVE_SAMPLES
            ):
                return feedback
            hold = min(
                GRIPPER_RAW_MAX,
                max(
                    GRIPPER_RAW_MIN,
                    feedback["position_raw"] - self.command_feedback_offset_raw,
                ),
            )
            squeeze = self.contact_extra_close_target(hold)
            controller.set_gripper_raw_position(squeeze)
            self.commanded_position_raw = squeeze
            self.contact_latched = True
            self.contact_hold_position_raw = squeeze
            _write_log(
                log_stream,
                "gripper_contact_latched",
                context=context,
                feedback=feedback,
                baseline_current_raw=self.baseline_current_raw,
                open_baseline_current_delta_raw=open_baseline_delta,
                expected_no_load_current_raw=expected_no_load_current,
                contact_current_residual_raw=contact_residual,
                no_load_curve_available=self.has_no_load_curve,
                current_delta_limit_raw=self.current_delta_limit_raw,
                contact_overcurrent_samples=self.contact_overcurrent_samples,
                contact_target_raw=hold,
                extra_close_m=GRIPPER_CONTACT_EXTRA_CLOSE_M,
                extra_close_raw=GRIPPER_CONTACT_EXTRA_CLOSE_RAW,
                hold_target_raw=squeeze,
                protection_requested=False,
            )
            self.contact_overcurrent_samples = 0
            _user_print(
                "\n检测到夹爪接触物体：再闭合 "
                f"{GRIPPER_CONTACT_EXTRA_CLOSE_M * 1000.0:.0f} mm "
                f"（{squeeze:.3f} raw）后停止；"
                f"残差={contact_residual:.4f} effort0。"
                "机械臂控制继续。按 O 张开后才能再次按 C。"
            )
            return feedback

        release_source = (
            feedback["position_raw"] - self.command_feedback_offset_raw
            if self.commanded_position_raw is None
            else self.commanded_position_raw
        )
        release = self.release_target(release_source)
        controller.set_gripper_raw_position(release)
        self.commanded_position_raw = release
        _write_log(
            log_stream,
            "gripper_current_limit",
            context=context,
            reason="emergency_current",
            feedback=feedback,
            baseline_current_raw=self.baseline_current_raw,
            open_baseline_current_delta_raw=open_baseline_delta,
            expected_no_load_current_raw=expected_no_load_current,
            contact_current_residual_raw=contact_residual,
            current_delta_limit_raw=self.current_delta_limit_raw,
            contact_overcurrent_samples=self.contact_overcurrent_samples,
            release_target_raw=release,
        )
        self.contact_overcurrent_samples = 0
        raise JogSafetyError(
            "Gripper emergency_current effort0 delta reached "
            f"{open_baseline_delta:.4f} (emergency limit "
            f"{GRIPPER_EMERGENCY_CURRENT_DELTA_RAW:.4f}); "
            f"released toward {release:.3f} and requested protection."
        )

    def open_and_calibrate(
        self,
        controller: Any,
        *,
        max_velocity: float,
        log_stream: TextIO,
        arm_reference_positions: Sequence[float] | None = None,
    ) -> dict[str, Any]:
        """Open fully, then establish a stationary no-load effort0 baseline."""
        open_velocity_limit = max(
            float(max_velocity), GRIPPER_OPEN_MAX_ARM_VELOCITY_RAD_S
        )
        reference_positions = (
            None
            if arm_reference_positions is None
            else _finite_values(
                arm_reference_positions,
                expected=6,
                label="gripper-open arm reference position",
            )
        )
        controller.set_gripper_raw_position(self.open_raw)
        self.commanded_position_raw = self.open_raw
        self.command_feedback_offset_raw = 0.0
        self.contact_latched = False
        self.contact_hold_position_raw = None
        self.contact_overcurrent_samples = 0
        started = time.monotonic()
        stable_currents: list[float] = []
        arm_overspeed_samples: list[dict[str, Any]] = []
        latest_state: dict[str, Any] | None = None
        last_feedback: dict[str, Any] | None = None
        while time.monotonic() - started < GRIPPER_COMMAND_TIMEOUT_S:
            latest_state = _diagnostic(controller)
            arm_velocities = _finite_values(
                latest_state["arm"]["velocity_rad_s"],
                expected=6,
                label="joint velocity",
            )
            arm_positions = _finite_values(
                latest_state["arm"]["position_rad"],
                expected=6,
                label="joint position",
            )
            if reference_positions is None:
                reference_positions = list(arm_positions)
            excursions = [
                abs(position - reference)
                for position, reference in zip(arm_positions, reference_positions)
            ]
            peak_excursion_index = max(
                range(len(excursions)), key=excursions.__getitem__
            )
            peak_excursion = excursions[peak_excursion_index]
            if peak_excursion > GRIPPER_OPEN_MAX_ARM_EXCURSION_RAD:
                _write_log(
                    log_stream,
                    "joint_excursion_limit_during_gripper_open",
                    limit_rad=GRIPPER_OPEN_MAX_ARM_EXCURSION_RAD,
                    peak_joint_index=peak_excursion_index + 1,
                    peak_joint_excursion_rad=peak_excursion,
                    reference_position_rad=reference_positions,
                    position_rad=arm_positions,
                    state=latest_state,
                )
                raise JogSafetyError(
                    "Measured joint excursion exceeded "
                    f"{GRIPPER_OPEN_MAX_ARM_EXCURSION_RAD:.3f} rad while opening "
                    "the gripper: "
                    f"J{peak_excursion_index + 1}={peak_excursion:+.3f} rad."
                )
            arm_velocity = max(abs(value) for value in arm_velocities)
            if arm_velocity > open_velocity_limit:
                peak_index = max(
                    range(len(arm_velocities)),
                    key=lambda index: abs(arm_velocities[index]),
                )
                arm_overspeed_samples.append(
                    {
                        "elapsed_s": time.monotonic() - started,
                        "peak_joint_index": peak_index + 1,
                        "peak_joint_velocity_rad_s": arm_velocities[peak_index],
                        "velocity_rad_s": arm_velocities,
                    }
                )
                stable_currents.clear()
            else:
                arm_overspeed_samples.clear()
            if (
                len(arm_overspeed_samples)
                >= GRIPPER_OPEN_ARM_OVERSPEED_CONSECUTIVE_SAMPLES
            ):
                peak_sample = max(
                    arm_overspeed_samples,
                    key=lambda sample: abs(sample["peak_joint_velocity_rad_s"]),
                )
                _write_log(
                    log_stream,
                    "joint_velocity_limit_during_gripper_open",
                    limit_rad_s=open_velocity_limit,
                    normal_motion_limit_rad_s=max_velocity,
                    consecutive_sample_limit=(
                        GRIPPER_OPEN_ARM_OVERSPEED_CONSECUTIVE_SAMPLES
                    ),
                    overspeed_samples=arm_overspeed_samples,
                    peak_joint_index=peak_sample["peak_joint_index"],
                    peak_joint_velocity_rad_s=(
                        peak_sample["peak_joint_velocity_rad_s"]
                    ),
                    velocity_rad_s=peak_sample["velocity_rad_s"],
                    state=latest_state,
                )
                raise JogSafetyError(
                    f"Measured velocity exceeded {open_velocity_limit:.3f} rad/s "
                    "for "
                    f"{GRIPPER_OPEN_ARM_OVERSPEED_CONSECUTIVE_SAMPLES} consecutive "
                    "samples while opening the gripper: "
                    f"J{peak_sample['peak_joint_index']}="
                    f"{peak_sample['peak_joint_velocity_rad_s']:+.3f} rad/s peak."
                )
            feedback = _gripper_raw_feedback(latest_state)
            last_feedback = feedback
            if (
                arm_velocity <= open_velocity_limit
                and abs(feedback["position_raw"] - self.open_raw)
                <= GRIPPER_OPEN_POSITION_TOLERANCE
                and abs(feedback["velocity_raw"])
                <= GRIPPER_RAW_STABLE_VELOCITY
            ):
                stable_currents.append(feedback["effort0"])
            else:
                stable_currents.clear()
            if len(stable_currents) >= GRIPPER_STABLE_SAMPLES:
                self.baseline_current_raw = float(median(stable_currents))
                self.contact_overcurrent_samples = 0
                self.command_feedback_offset_raw = (
                    feedback["position_raw"] - self.open_raw
                )
                _write_log(
                    log_stream,
                    "gripper_opened_and_calibrated",
                    feedback=feedback,
                    baseline_current_raw=self.baseline_current_raw,
                    current_delta_limit_raw=self.current_delta_limit_raw,
                    open_target_raw=self.open_raw,
                    command_feedback_offset_raw=self.command_feedback_offset_raw,
                    open_position_tolerance_raw=GRIPPER_OPEN_POSITION_TOLERANCE,
                    open_arm_velocity_limit_rad_s=open_velocity_limit,
                    open_arm_excursion_limit_rad=(
                        GRIPPER_OPEN_MAX_ARM_EXCURSION_RAD
                    ),
                )
                return latest_state
            time.sleep(GRIPPER_MONITOR_PERIOD_S)
        position = None if last_feedback is None else last_feedback["position_raw"]
        velocity = None if last_feedback is None else last_feedback["velocity_raw"]
        _write_log(
            log_stream,
            "gripper_open_timeout",
            open_target_raw=self.open_raw,
            last_feedback=last_feedback,
            open_position_tolerance_raw=GRIPPER_OPEN_POSITION_TOLERANCE,
            timeout_s=GRIPPER_COMMAND_TIMEOUT_S,
        )
        raise GripperOpenTimeoutError(
            "Gripper did not reach a stable open position within "
            f"{GRIPPER_COMMAND_TIMEOUT_S:.1f} seconds; "
            f"last position={position} raw, velocity={velocity} raw/s, "
            f"target={self.open_raw:.3f} (tolerance "
            f"{GRIPPER_OPEN_POSITION_TOLERANCE:.2f})."
        )

    def calibrate_no_load_curve(
        self,
        controller: Any,
        *,
        max_velocity: float,
        log_stream: TextIO,
    ) -> dict[str, Any]:
        """Sweep an empty gripper and store stationary effort0 versus position."""
        opened_state = self.open_and_calibrate(
            controller,
            max_velocity=max_velocity,
            log_stream=log_stream,
        )
        assert self.baseline_current_raw is not None
        opened_feedback = _gripper_raw_feedback(opened_state)
        points: list[tuple[float, float]] = [
            (float(opened_feedback["position_raw"]), 0.0)
        ]
        command_position = self.open_raw
        _write_log(
            log_stream,
            "gripper_no_load_curve_started",
            open_raw=self.open_raw,
            closed_raw=self.closed_raw,
            step_raw=self.step_raw,
            baseline_current_raw=self.baseline_current_raw,
            curve_path=(
                None
                if self.no_load_curve_path is None
                else str(self.no_load_curve_path)
            ),
        )
        latest_state = opened_state
        while not math.isclose(command_position, self.closed_raw, abs_tol=1e-9):
            target = self.next_close_target(command_position)
            expected_feedback_position = target + self.command_feedback_offset_raw
            controller.set_gripper_raw_position(target)
            self.commanded_position_raw = target
            started = time.monotonic()
            stable_currents: list[float] = []
            stable_positions: list[float] = []
            while time.monotonic() - started < GRIPPER_COMMAND_TIMEOUT_S:
                latest_state = _diagnostic(controller)
                arm_velocity = max(
                    abs(value)
                    for value in _finite_values(
                        latest_state["arm"]["velocity_rad_s"],
                        expected=6,
                        label="joint velocity",
                    )
                )
                if arm_velocity > max_velocity:
                    raise JogSafetyError(
                        f"Measured velocity exceeded {max_velocity:.3f} rad/s "
                        "during gripper no-load calibration."
                    )
                feedback = _gripper_raw_feedback(latest_state)
                emergency_delta = abs(
                    feedback["effort0"] - self.baseline_current_raw
                )
                if emergency_delta >= GRIPPER_EMERGENCY_CURRENT_DELTA_RAW:
                    controller.set_gripper_raw_position(self.open_raw)
                    self.commanded_position_raw = self.open_raw
                    raise JogSafetyError(
                        "Gripper no-load calibration reached the emergency "
                        f"effort0 delta {emergency_delta:.4f}; requested opening."
                    )
                if (
                    abs(
                        feedback["position_raw"] - expected_feedback_position
                    )
                    <= GRIPPER_RAW_POSITION_TOLERANCE
                    and abs(feedback["velocity_raw"])
                    <= GRIPPER_RAW_STABLE_VELOCITY
                ):
                    stable_currents.append(feedback["effort0"])
                    stable_positions.append(feedback["position_raw"])
                else:
                    stable_currents.clear()
                    stable_positions.clear()
                if len(stable_currents) >= GRIPPER_STABLE_SAMPLES:
                    measured_position = float(median(stable_positions))
                    measured_current = float(median(stable_currents))
                    current_delta = measured_current - self.baseline_current_raw
                    points.append((measured_position, current_delta))
                    _write_log(
                        log_stream,
                        "gripper_no_load_curve_point",
                        target_raw=target,
                        position_raw=measured_position,
                        effort0=measured_current,
                        current_delta_raw=current_delta,
                    )
                    _user_print(
                        "空载曲线采样："
                        f"位置={measured_position:.4f} raw，"
                        f"effort0偏移={current_delta:+.4f}。"
                    )
                    break
                time.sleep(GRIPPER_MONITOR_PERIOD_S)
            else:
                controller.set_gripper_raw_position(self.open_raw)
                self.commanded_position_raw = self.open_raw
                raise JogSafetyError(
                    "Gripper no-load calibration did not settle at target "
                    f"{target:.3f} raw within {GRIPPER_COMMAND_TIMEOUT_S:.1f} seconds; "
                    "requested opening."
                )
            command_position = target

        self.no_load_curve = sorted(points)
        self._save_no_load_curve()
        _write_log(
            log_stream,
            "gripper_no_load_curve_saved",
            point_count=len(self.no_load_curve),
            points=self.no_load_curve,
            curve_path=(
                None
                if self.no_load_curve_path is None
                else str(self.no_load_curve_path)
            ),
        )
        reopened_state = self.open_and_calibrate(
            controller,
            max_velocity=max_velocity,
            log_stream=log_stream,
        )
        _user_print(
            f"空载位置—effort0曲线完成，共 {len(self.no_load_curve)} 个点；"
            "夹爪已重新张开。"
        )
        return reopened_state

    def close_one_step(
        self,
        controller: Any,
        *,
        max_velocity: float,
        log_stream: TextIO,
    ) -> dict[str, Any]:
        """Close by one bounded raw step while continuously enforcing effort0."""
        if self.baseline_current_raw is None:
            raise JogSafetyError(
                "Press O to open and calibrate the gripper effort0 baseline before C."
            )
        before = _diagnostic(controller)
        feedback = self.enforce(
            controller, before, log_stream, context="before_gripper_close"
        )
        self.contact_overcurrent_samples = 0
        command_start = (
            feedback["position_raw"] - self.command_feedback_offset_raw
            if self.commanded_position_raw is None
            else self.commanded_position_raw
        )
        target = self.next_close_target(command_start)
        expected_feedback_position = target + self.command_feedback_offset_raw
        controller.set_gripper_raw_position(target)
        self.commanded_position_raw = target
        _write_log(
            log_stream,
            "gripper_close_requested",
            feedback=feedback,
            command_start_raw=command_start,
            target_raw=target,
            expected_feedback_position_raw=expected_feedback_position,
            command_feedback_offset_raw=self.command_feedback_offset_raw,
            step_raw=self.step_raw,
            baseline_current_raw=self.baseline_current_raw,
            current_delta_limit_raw=self.current_delta_limit_raw,
        )
        started = time.monotonic()
        stable_samples = 0
        latest_state = before
        while time.monotonic() - started < GRIPPER_COMMAND_TIMEOUT_S:
            latest_state = _diagnostic(controller)
            arm_velocity = max(
                abs(value)
                for value in _finite_values(
                    latest_state["arm"]["velocity_rad_s"],
                    expected=6,
                    label="joint velocity",
                )
            )
            if arm_velocity > max_velocity:
                raise JogSafetyError(
                    f"Measured velocity exceeded {max_velocity:.3f} rad/s "
                    "during gripper close."
                )
            feedback = self.enforce(
                controller, latest_state, log_stream, context="gripper_close"
            )
            if self.contact_latched:
                _write_log(
                    log_stream,
                    "gripper_close_stopped_on_contact",
                    feedback=feedback,
                    hold_target_raw=self.contact_hold_position_raw,
                )
                return latest_state
            if (
                abs(feedback["position_raw"] - expected_feedback_position)
                <= GRIPPER_RAW_POSITION_TOLERANCE
                and abs(feedback["velocity_raw"])
                <= GRIPPER_RAW_STABLE_VELOCITY
            ):
                stable_samples += 1
            else:
                stable_samples = 0
            if stable_samples >= 3:
                _write_log(
                    log_stream,
                    "gripper_close_step_reached",
                    feedback=feedback,
                    target_raw=target,
                    expected_feedback_position_raw=expected_feedback_position,
                )
                return latest_state
            time.sleep(GRIPPER_MONITOR_PERIOD_S)
        release = self.release_target(self.commanded_position_raw)
        controller.set_gripper_raw_position(release)
        self.commanded_position_raw = release
        _write_log(
            log_stream,
            "gripper_close_step_timeout",
            feedback=feedback,
            target_raw=target,
            expected_feedback_position_raw=expected_feedback_position,
            command_feedback_offset_raw=self.command_feedback_offset_raw,
            release_target_raw=release,
        )
        raise JogSafetyError(
            "Gripper close step timed out; released one step and requested protection."
        )

    def close_until_grasp(
        self,
        controller: Any,
        *,
        max_velocity: float,
        log_stream: TextIO,
    ) -> dict[str, Any]:
        """Close in bounded raw steps until contact, then extra 1 mm, then stop."""
        if self.baseline_current_raw is None:
            raise JogSafetyError(
                "Press O to open and calibrate the gripper effort0 baseline before C."
            )
        latest_state: dict[str, Any] | None = None
        while not self.contact_latched:
            command_start = self.commanded_position_raw
            if command_start is None:
                latest_state = _diagnostic(controller)
                feedback = _gripper_raw_feedback(latest_state)
                command_start = (
                    feedback["position_raw"] - self.command_feedback_offset_raw
                )
            if abs(float(command_start) - self.closed_raw) <= 1e-6:
                if latest_state is None:
                    latest_state = _diagnostic(controller)
                return latest_state
            latest_state = self.close_one_step(
                controller,
                max_velocity=max_velocity,
                log_stream=log_stream,
            )
        return latest_state


# =============================================================================
# 5. Numeric validation and Cartesian target construction
# =============================================================================

def _finite_values(
    values: Sequence[float], *, expected: int, label: str
) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != expected:
        raise JogSafetyError(f"{label} length {len(result)} != {expected}.")
    if not all(math.isfinite(value) for value in result):
        raise JogSafetyError(f"{label} contains NaN or Inf.")
    return result


def _distance(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(first, second)))


def build_cartesian_target(
    current_pose: Sequence[float],
    startup_pose: Sequence[float],
    *,
    axis: int,
    direction: int,
) -> list[float]:
    """Build a 1 cm XYZ target that preserves the latest measured orientation."""
    current = _finite_values(current_pose, expected=6, label="current EE pose")
    startup = _finite_values(startup_pose, expected=6, label="startup EE pose")
    if axis not in range(3):
        raise JogSafetyError("Cartesian axis must be X, Y, or Z.")
    if direction not in (-1, 1):
        raise JogSafetyError("Cartesian direction must be -1 or +1.")

    target = current.copy()
    target[axis] += direction * CARTESIAN_STEP_M
    displacement = _distance(target[:3], startup[:3])
    if displacement > MAX_DISPLACEMENT_M + 1e-9:
        raise JogSafetyError(
            "EE target would exceed the 80 cm displacement limit "
            f"({displacement * 100.0:.2f} cm requested)."
        )
    return target


def build_return_target(
    current_pose: Sequence[float], startup_pose: Sequence[float]
) -> list[float]:
    """Return startup XYZ while preserving the latest measured orientation."""
    current = _finite_values(current_pose, expected=6, label="current EE pose")
    startup = _finite_values(startup_pose, expected=6, label="startup EE pose")
    target = current.copy()
    target[:3] = startup[:3]
    return target


def validate_ik_solution(
    solution: Sequence[float], current_joints: Sequence[float]
) -> list[float]:
    """Reject non-finite, out-of-range, or discontinuous vendor IK results."""
    joints = _finite_values(solution, expected=6, label="IK solution")
    current = _finite_values(current_joints, expected=6, label="current joints")
    for index, (value, bounds) in enumerate(zip(joints, JOINT_LIMITS_RAD)):
        lower, upper = bounds
        if value < lower - IK_JOINT_LIMIT_SLACK_RAD or value > upper + IK_JOINT_LIMIT_SLACK_RAD:
            raise JogSafetyError(
                f"IK J{index + 1}={value:.6f} rad exceeds "
                f"[{lower:.6f}, {upper:.6f}]."
            )
        if value < lower:
            joints[index] = lower
        elif value > upper:
            joints[index] = upper
        value = joints[index]
        delta = abs(value - current[index])
        if delta > MAX_IK_JOINT_STEP_RAD + 1e-9:
            raise JogSafetyError(
                f"IK J{index + 1} jump {delta:.6f} rad exceeds "
                f"{MAX_IK_JOINT_STEP_RAD:.3f} rad."
            )
    return joints


def _joint_limit_normalization_target(
    current_joints: Sequence[float],
) -> list[float]:
    """Clamp only small, slack-approved feedback bias onto joint bounds."""
    current = _finite_values(
        current_joints,
        expected=6,
        label="joint-limit normalization feedback",
    )
    target = current.copy()
    for index, (value, (lower, upper)) in enumerate(
        zip(current, JOINT_LIMITS_RAD)
    ):
        if value < lower:
            if value < lower - IK_JOINT_LIMIT_SLACK_RAD:
                raise JogSafetyError(
                    f"Joint feedback J{index + 1}={value:.6f} rad is below "
                    f"the normalization allowance {lower:.6f}-"
                    f"{IK_JOINT_LIMIT_SLACK_RAD:.3f}."
                )
            target[index] = lower
        elif value > upper:
            if value > upper + IK_JOINT_LIMIT_SLACK_RAD:
                raise JogSafetyError(
                    f"Joint feedback J{index + 1}={value:.6f} rad is above "
                    f"the normalization allowance {upper:.6f}+"
                    f"{IK_JOINT_LIMIT_SLACK_RAD:.3f}."
                )
            target[index] = upper
    return target


def _execute_joint_limit_normalization(
    controller: Any,
    state: dict[str, Any],
    *,
    max_velocity: float,
    log_stream: TextIO,
    idle_pump: Callable[[], None] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[dict[str, Any], list[float], bool]:
    """Slew slack-approved joint feedback bias to the nearest legal bound."""
    start = _joint_positions(state)
    target = _joint_limit_normalization_target(start)
    maximum_delta = max(abs(a - b) for a, b in zip(start, target))
    if maximum_delta <= 1e-12:
        return state, start, False

    command_rate = min(MAX_JOINT_COMMAND_RATE_RAD_S, float(max_velocity))
    if command_rate <= 0.0:
        raise JogSafetyError("Joint-limit normalization rate must be positive.")
    waypoint_count = max(
        1,
        math.ceil(
            maximum_delta
            / (command_rate * CARTESIAN_COMMAND_PERIOD_S)
        ),
    )
    trajectory = [
        [
            start_value + (target_value - start_value) * index / waypoint_count
            for start_value, target_value in zip(start, target)
        ]
        for index in range(1, waypoint_count + 1)
    ]
    started = monotonic()
    deadline = started + JOINT_LIMIT_NORMALIZATION_TIMEOUT_S
    latest = state
    _write_log(
        log_stream,
        "joint_limit_normalization_started",
        start_joints_rad=start,
        target_joints_rad=target,
        command_rate_rad_s=command_rate,
        command_period_s=CARTESIAN_COMMAND_PERIOD_S,
        waypoint_count=waypoint_count,
    )

    next_command_at = started + CARTESIAN_COMMAND_PERIOD_S
    for waypoint_index, waypoint in enumerate(trajectory, start=1):
        while monotonic() < next_command_at:
            if idle_pump is not None:
                idle_pump()
            remaining = next_command_at - monotonic()
            if remaining > 0.0:
                sleep(min(CARTESIAN_COMMAND_PERIOD_S, remaining))
        if monotonic() >= deadline:
            raise JogSafetyError(
                "Joint-limit normalization exceeded "
                f"{JOINT_LIMIT_NORMALIZATION_TIMEOUT_S:.1f} s."
            )
        latest = _diagnostic(controller)
        velocities = _finite_values(
            latest["arm"]["velocity_rad_s"],
            expected=6,
            label="joint-limit normalization velocity",
        )
        if max(abs(value) for value in velocities) > max_velocity:
            raise JogSafetyError(
                "Measured velocity exceeded "
                f"{max_velocity:.3f} rad/s during joint-limit normalization."
            )
        controller.update_arm_position_target(waypoint)
        if idle_pump is not None:
            idle_pump()
        _write_log(
            log_stream,
            "joint_limit_normalization_target_sent",
            waypoint_index=waypoint_index,
            waypoint_count=waypoint_count,
            target_joints_rad=waypoint,
        )
        next_command_at = monotonic() + CARTESIAN_COMMAND_PERIOD_S

    stable_samples = 0
    while monotonic() < deadline:
        if idle_pump is not None:
            idle_pump()
        sleep(CARTESIAN_COMMAND_PERIOD_S)
        latest = _diagnostic(controller)
        velocities = _finite_values(
            latest["arm"]["velocity_rad_s"],
            expected=6,
            label="joint-limit normalization velocity",
        )
        maximum_velocity = max(abs(value) for value in velocities)
        if maximum_velocity > max_velocity:
            raise JogSafetyError(
                "Measured velocity exceeded "
                f"{max_velocity:.3f} rad/s during joint-limit normalization."
            )
        if maximum_velocity <= FEEDBACK_STABLE_VELOCITY_RAD_S:
            stable_samples += 1
        else:
            stable_samples = 0
        if stable_samples >= MOTION_SETTLE_STABLE_SAMPLES:
            _write_log(
                log_stream,
                "joint_limit_normalization_complete",
                elapsed_s=monotonic() - started,
                commanded_joints_rad=target,
                measured_joints_rad=_joint_positions(latest),
                stable_samples=stable_samples,
            )
            return latest, target, True
    raise JogSafetyError(
        "Joint-limit normalization did not settle within "
        f"{JOINT_LIMIT_NORMALIZATION_TIMEOUT_S:.1f} s."
    )


# =============================================================================
# 6. Command-line interface and experiment-scope validation
# =============================================================================

def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Jog one X5-2023 end effector by 1 cm per key, within 80 cm of "
            "its startup position."
        )
    )
    parser.add_argument("--interface", required=True, help="Exact interface: can2 or can4")
    parser.add_argument("--sdk-path", default=None)
    parser.add_argument("--motion-timeout", type=float, default=15.0)
    parser.add_argument(
        "--idle-timeout",
        type=float,
        default=None,
        help=(
            "Deprecated compatibility option; keyboard input now waits "
            "indefinitely and this value is ignored."
        ),
    )
    parser.add_argument(
        "--max-commands",
        type=int,
        default=None,
        help="Deprecated compatibility option; command count is no longer limited.",
    )
    parser.add_argument(
        "--max-velocity", type=float, default=MAX_MEASURED_VELOCITY_RAD_S
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=None,
        help="JSONL path. Default: app/logs/, keeping the 5 newest session logs.",
    )
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument(
        "--acknowledge-limited-feedback",
        action="store_true",
        help="Acknowledge that effort, fault and continuous gripper feedback are unverified.",
    )
    parser.add_argument(
        "--acknowledge-transition-grace-risk",
        action="store_true",
        help=(
            "Acknowledge the fixed 300 ms status-5 transition observation window and "
            "its measured-speed risk."
        ),
    )
    parser.add_argument("--enable-gripper", action="store_true")
    parser.add_argument(
        "--acknowledge-unverified-gripper-current",
        action="store_true",
        help=(
            "Acknowledge that the seventh effort0 value is uncalibrated "
            "and is not a force measurement."
        ),
    )
    parser.add_argument(
        "--gripper-current-delta-limit-raw",
        type=float,
        default=None,
        help=(
            "Required with --enable-gripper: maximum seventh effort0 residual "
            "from the interpolated position-dependent no-load curve."
        ),
    )
    parser.add_argument(
        "--gripper-no-load-curve",
        type=Path,
        default=None,
        help=(
            "Optional no-load position/effort0 curve JSON path. The default is "
            "app/x5_2023_gripper_no_load_curve_<interface>.json."
        ),
    )
    parser.add_argument(
        "--gripper-open-raw", type=float, default=GRIPPER_DEFAULT_OPEN_RAW
    )
    parser.add_argument(
        "--gripper-closed-raw", type=float, default=GRIPPER_DEFAULT_CLOSED_RAW
    )
    parser.add_argument("--gripper-step-raw", type=float, default=GRIPPER_RAW_STEP)
    parser.add_argument(
        "--auto-force-calibration-box-mass-g",
        type=float,
        default=None,
        help=(
            "Enable the confirmed 100/200/500 g automatic load sequence and "
            "provide the measured empty-box mass in grams."
        ),
    )
    parser.add_argument(
        "--force-calibration-pose-label",
        default="p01_z_down",
        help="Pose label used by the automatic force-calibration sequence.",
    )
    parser.add_argument(
        "--force-calibration-samples-per-load",
        type=int,
        default=FORCE_CALIBRATION_DEFAULT_SAMPLES_PER_LOAD,
        help="Number of stationary G samples required for each automatic load group.",
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if not args.interface or any(char in args.interface for char in "^$*+?{}[]|()\\"):
        raise ValueError("--interface must be one exact SocketCAN name, not a pattern.")
    if not 1.0 <= float(args.motion_timeout) <= 30.0:
        raise ValueError("--motion-timeout must be in [1.0, 30.0] seconds.")
    if not 0.0 < float(args.max_velocity) <= MAX_MEASURED_VELOCITY_RAD_S:
        raise ValueError(
            "--max-velocity must be in "
            f"(0, {MAX_MEASURED_VELOCITY_RAD_S:.1f}] rad/s."
        )
    gripper_values = (
        float(args.gripper_open_raw),
        float(args.gripper_closed_raw),
        float(args.gripper_step_raw),
    )
    if not all(math.isfinite(value) for value in gripper_values):
        raise ValueError("Gripper raw targets and step must be finite.")
    if not all(
        GRIPPER_RAW_MIN <= value <= GRIPPER_RAW_MAX
        for value in gripper_values[:2]
    ):
        raise ValueError("Gripper open/closed raw targets must be in [0, 5].")
    if gripper_values[0] == gripper_values[1]:
        raise ValueError("Gripper open and closed raw targets must differ.")
    if not 0.0 < gripper_values[2] <= abs(gripper_values[1] - gripper_values[0]):
        raise ValueError("Gripper raw step must be positive and no larger than its range.")
    if args.enable_gripper:
        if args.gripper_current_delta_limit_raw is None:
            raise ValueError(
                "--enable-gripper requires --gripper-current-delta-limit-raw."
            )
        limit = float(args.gripper_current_delta_limit_raw)
        if not math.isfinite(limit) or limit <= 0.0:
            raise ValueError("Gripper raw current delta limit must be finite and positive.")
    if args.auto_force_calibration_box_mass_g is not None:
        _calibration_mass_g(
            str(args.auto_force_calibration_box_mass_g), field_name="盒子质量"
        )
        _calibration_text(
            str(args.force_calibration_pose_label),
            field_name="姿态标签",
            allow_empty=False,
        )
        if not 1 <= int(args.force_calibration_samples_per_load) <= 1000:
            raise ValueError(
                "--force-calibration-samples-per-load must be in [1, 1000]."
            )


def _validate_motion_scope(args: argparse.Namespace) -> None:
    if args.interface != "can2":
        raise ValueError("Experimental transition-grace motion is restricted to can2.")
    if float(args.max_velocity) > MAX_MEASURED_VELOCITY_RAD_S:
        raise ValueError(
            "Motion-phase --max-velocity must not exceed "
            f"{MAX_MEASURED_VELOCITY_RAD_S:.3f} rad/s."
        )


# =============================================================================
# 7. SDK state, logging, terminal I/O, and operator-facing formatting
# =============================================================================

def _diagnostic(controller: Any) -> dict[str, Any]:
    state = controller.get_diagnostic_state()
    _finite_values(state["arm"]["position_rad"], expected=6, label="joint position")
    _finite_values(state["arm"]["velocity_rad_s"], expected=6, label="joint velocity")
    _finite_values(state["arm"]["effort_raw"], expected=6, label="joint effort")
    return state


def _joint_positions(state: dict[str, Any]) -> list[float]:
    return _finite_values(
        state["arm"]["position_rad"], expected=6, label="joint position"
    )


def _fk_pose(
    state: dict[str, Any],
    fk_solver: Callable[[np.ndarray], Sequence[float]],
) -> list[float]:
    """Compute XYZ/RPY from validated joint feedback, not vendor EE feedback."""
    result = _finite_values(
        fk_solver(np.asarray(_joint_positions(state), dtype=float)),
        expected=6,
        label="FK EE pose",
    )
    if any(abs(value) > 2.0 for value in result[:3]):
        raise JogSafetyError("FK EE position exceeds the 2 m sanity bound.")
    if any(abs(value) > 2.0 * math.pi for value in result[3:]):
        raise JogSafetyError("FK EE orientation exceeds the 2*pi sanity bound.")
    return result


def _write_log(stream: TextIO, event: str, **values: Any) -> None:
    record = {
        "time": datetime.now().astimezone().isoformat(),
        "event": event,
        **values,
    }
    stream.write(json.dumps(record, ensure_ascii=False) + "\n")
    stream.flush()


@contextmanager
def _terminal_keys(stream: TextIO) -> Iterator[list[Any]]:
    descriptor = stream.fileno()
    original = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        yield original
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, original)


def _calibration_text(value: str, *, field_name: str, allow_empty: bool) -> str:
    result = value.strip()
    if not result and not allow_empty:
        raise ValueError(f"{field_name}不能为空。")
    if len(result) > 120 or any(ord(char) < 32 for char in result):
        raise ValueError(f"{field_name}必须是不超过 120 个字符的单行文字。")
    return result


def _calibration_mass_g(value: str, *, field_name: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise ValueError(f"{field_name}必须是克数，例如 40 或 100。") from exc
    if not math.isfinite(result) or result < 0.0 or result > 5000.0:
        raise ValueError(f"{field_name}必须在 0～5000 g 之间。")
    return result


def _prompt_force_calibration_label(
    current: ForceCalibrationLabel | None,
    *,
    input_stream: TextIO,
    terminal_attributes: list[Any] | None = None,
) -> ForceCalibrationLabel | None:
    """Temporarily use cooked input and collect metadata for later G samples."""
    descriptor = input_stream.fileno() if terminal_attributes is not None else None
    if descriptor is not None:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, terminal_attributes)

    def read_value(prompt: str, previous: str | None = None) -> str | None:
        suffix = "" if previous is None else f"（回车保留 {previous}）"
        # Keep the prompt on its own terminated line. Otherwise a vendor C++
        # banner can be appended to the same pipe line before the output filter
        # sees it, making the banner appear inside the prompt.
        _user_print(f"{prompt}{suffix}：")
        line = input_stream.readline()
        if line == "":
            return None
        value = line.strip()
        return previous if not value and previous is not None else value

    try:
        _user_print("\n输入本组 G 样本的标定标签；质量单位均为 g。")
        sample = read_value(
            "样本标签，例如 empty_box、box_plus_100g",
            None if current is None else current.sample_label,
        )
        if sample is None:
            return None
        box = read_value(
            "盒子实测质量",
            None if current is None else f"{current.box_mass_g:g}",
        )
        if box is None:
            return None
        added = read_value(
            "盒内新增砝码总质量",
            None if current is None else f"{current.added_mass_g:g}",
        )
        if added is None:
            return None
        pose = read_value(
            "姿态标签，例如 pose_z_down_1",
            None if current is None else current.pose_label,
        )
        if pose is None:
            return None
        note = read_value(
            "备注（可留空）",
            "" if current is None else current.note,
        )
        if note is None:
            return None
        return ForceCalibrationLabel(
            sample_label=_calibration_text(
                sample, field_name="样本标签", allow_empty=False
            ),
            box_mass_g=_calibration_mass_g(box, field_name="盒子质量"),
            added_mass_g=_calibration_mass_g(added, field_name="砝码质量"),
            pose_label=_calibration_text(
                pose, field_name="姿态标签", allow_empty=False
            ),
            note=_calibration_text(note, field_name="备注", allow_empty=True),
        )
    finally:
        if descriptor is not None:
            tty.setcbreak(descriptor)


def _suppress_vendor_line(text: str) -> bool:
    return text.strip() == VENDOR_BANNER


@contextmanager
def _filtered_vendor_output() -> Iterator[None]:
    """Hide the repeated vendor banner on stdout/stderr; forward other lines."""
    sys.stdout.flush()
    sys.stderr.flush()
    original_stdout = os.dup(1)
    original_stderr = os.dup(2)
    read_fd, write_fd = os.pipe()
    os.dup2(write_fd, 1)
    os.dup2(write_fd, 2)
    os.close(write_fd)

    def forward_output() -> None:
        with os.fdopen(read_fd, "rb", closefd=True) as stream:
            for raw_line in iter(stream.readline, b""):
                text = raw_line.decode("utf-8", errors="replace")
                if not _suppress_vendor_line(text):
                    os.write(original_stderr, raw_line)

    reader = threading.Thread(target=forward_output, daemon=True)
    reader.start()
    try:
        yield
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os.dup2(original_stdout, 1)
        os.dup2(original_stderr, 2)
        reader.join(timeout=1.0)
        os.close(original_stdout)
        os.close(original_stderr)


def _read_key(timeout: float | None) -> str | None:
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if readable else None


def _read_key_with_gripper_monitor(
    _timeout: float | None,
    *,
    controller: Any,
    gripper_guard: GripperCurrentGuard | None,
    max_velocity: float,
    log_stream: TextIO,
) -> str | None:
    if gripper_guard is None or gripper_guard.baseline_current_raw is None:
        return _read_key(None)
    overspeed_samples = 0
    while True:
        key = _read_key(GRIPPER_MONITOR_PERIOD_S)
        if key is not None:
            return key
        state = _diagnostic(controller)
        velocities = _finite_values(
            state["arm"]["velocity_rad_s"],
            expected=6,
            label="joint velocity",
        )
        joint_index, joint_velocity = max(
            enumerate(velocities), key=lambda item: abs(item[1])
        )
        measured_max_velocity = abs(joint_velocity)
        if measured_max_velocity > max_velocity:
            overspeed_samples += 1
            _write_log(
                log_stream,
                "keyboard_idle_overspeed_observation",
                velocity_rad_s=velocities,
                measured_max_velocity_rad_s=measured_max_velocity,
                joint_index=joint_index + 1,
                joint_velocity_rad_s=joint_velocity,
                limit_rad_s=max_velocity,
                consecutive_samples=overspeed_samples,
                required_consecutive_samples=(
                    KEYBOARD_IDLE_OVERSPEED_CONSECUTIVE_SAMPLES
                ),
            )
        else:
            overspeed_samples = 0
        if overspeed_samples >= KEYBOARD_IDLE_OVERSPEED_CONSECUTIVE_SAMPLES:
            raise JogSafetyError(
                f"Measured velocity exceeded {max_velocity:.3f} rad/s "
                "for "
                f"{KEYBOARD_IDLE_OVERSPEED_CONSECUTIVE_SAMPLES} consecutive "
                "samples while waiting for a key."
            )
        gripper_guard.enforce(
            controller, state, log_stream, context="keyboard_idle"
        )


def _user_print(*values: Any, end: str = "\n") -> None:
    print(*values, file=sys.stderr, flush=True, end=end)


def _print_state(state: dict[str, Any], ee_pose: Sequence[float]) -> None:
    arm = state["arm"]
    xyz_cm = [round(float(value) * 100.0, 2) for value in ee_pose[:3]]
    positions_deg = [round(math.degrees(value), 2) for value in arm["position_rad"]]
    velocities = [round(float(value), 4) for value in arm["velocity_rad_s"]]
    _user_print(
        f"末端XYZ(cm): {xyz_cm}  关节(deg): {positions_deg}  "
        f"速度(rad/s): {velocities}"
    )


def _print_mcc_force(
    reading: dict[str, Any],
    calibration_label: ForceCalibrationLabel | None = None,
) -> None:
    force = [float(value) for value in reading["force_gripper_proxy"]]
    force_base = [float(value) for value in reading["force_base_proxy"]]
    _user_print(
        "夹爪整体外力（MCC 未标定代理值，不能当作 N）：\n"
        f"  夹爪坐标 Fx={force[0]:+.4f}  Fy={force[1]:+.4f}  "
        f"Fz={force[2]:+.4f}  合力={float(reading['force_norm_proxy']):.4f}\n"
        f"  基座坐标 Fx={force_base[0]:+.4f}  Fy={force_base[1]:+.4f}  "
        f"Fz={force_base[2]:+.4f}\n"
        f"  状态={reading['motion_state']}，"
        f"动力学补偿={reading['dynamic_compensation']}，"
        "标定=未完成"
    )
    joint_sample = reading.get("joint_sample")
    if isinstance(joint_sample, dict):
        positions_deg = joint_sample.get("position_deg")
        fk_pose = joint_sample.get("fk_ee_pose_xyzrpy")
        if positions_deg is not None:
            joints_text = [round(float(value), 2) for value in positions_deg]
            _user_print(f"  同步关节角(deg)={joints_text}")
        if fk_pose is not None:
            xyz_cm = [round(float(value) * 100.0, 2) for value in fk_pose[:3]]
            _user_print(
                f"  同步末端XYZ(cm)={xyz_cm}，"
                f"位姿来源={joint_sample.get('source')}"
            )
    if calibration_label is not None:
        label = calibration_label.as_log_record()
        _user_print(
            "  样本标签="
            f"{label['sample_label']}，姿态={label['pose_label']}，"
            f"总悬挂质量={label['total_suspended_mass_g']:.3f} g，"
            f"理论重力={label['expected_gravity_force_n']:.5f} N"
        )


def _looks_like_delayed_zero_arm_feedback(state: dict[str, Any]) -> bool:
    """Vendor SDK first frames are all-zero joints and velocities."""
    positions = _joint_positions(state)
    velocities = _finite_values(
        state["arm"]["velocity_rad_s"], expected=6, label="joint velocity"
    )
    return (
        max(abs(value) for value in positions) < 1e-12
        and max(abs(value) for value in velocities) < 1e-12
    )


def _warm_up_feedback(
    controller: Any,
    *,
    fk_solver: Callable[[np.ndarray], Sequence[float]],
    on_sample: Callable[[dict[str, Any]], None] | None = None,
    idle_pump: Callable[[], None] | None = None,
    min_s: float = FEEDBACK_WARMUP_MIN_S,
    max_s: float = FEEDBACK_WARMUP_MAX_S,
    ignore_delayed_zero: bool = False,
) -> tuple[dict[str, Any], list[float]]:
    """Wait for delayed vendor feedback and require several stationary samples."""
    started = time.monotonic()
    stable_samples = 0
    latest_state = _diagnostic(controller)
    latest_pose = _fk_pose(latest_state, fk_solver)
    if on_sample is not None:
        on_sample(latest_state)
    while True:
        now = time.monotonic()
        velocities = _finite_values(
            latest_state["arm"]["velocity_rad_s"],
            expected=6,
            label="joint velocity",
        )
        delayed_zero = (
            ignore_delayed_zero
            and _looks_like_delayed_zero_arm_feedback(latest_state)
        )
        if (
            not delayed_zero
            and now - started >= min_s
            and max(abs(value) for value in velocities)
            <= FEEDBACK_STABLE_VELOCITY_RAD_S
        ):
            stable_samples += 1
        else:
            stable_samples = 0
        if stable_samples >= FEEDBACK_STABLE_SAMPLES:
            return latest_state, latest_pose
        if now - started >= max_s:
            raise JogSafetyError(
                "Joint feedback did not become stationary during the "
                f"{max_s:.1f} s warmup."
            )
        if idle_pump is not None:
            until = time.monotonic() + 0.05
            while time.monotonic() < until:
                idle_pump()
                remaining = until - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(FEEDBACK_WARMUP_PUMP_PERIOD_S, remaining))
        else:
            time.sleep(0.05)
        latest_state = _diagnostic(controller)
        latest_pose = _fk_pose(latest_state, fk_solver)
        if on_sample is not None:
            on_sample(latest_state)


# =============================================================================
# 8. Cartesian trajectory planning, streaming, and runtime safety monitoring
# =============================================================================

def _cartesian_joint_trajectory(
    start_pose: Sequence[float],
    target_pose: Sequence[float],
    start_joints: Sequence[float],
    *,
    ik_solver: Callable[[np.ndarray], Sequence[float]],
    max_joint_command_rate: float = MAX_JOINT_COMMAND_RATE_RAD_S,
) -> list[tuple[list[float], list[float]]]:
    """Plan a straight 1 cm/s Cartesian target stream before moving."""
    start = _finite_values(start_pose, expected=6, label="trajectory start pose")
    target = _finite_values(target_pose, expected=6, label="trajectory target pose")
    previous_joints = _finite_values(
        start_joints, expected=6, label="trajectory start joints"
    )
    distance = _distance(start[:3], target[:3])
    count = max(
        1,
        math.ceil(
            distance
            / (CARTESIAN_SPEED_M_S * CARTESIAN_COMMAND_PERIOD_S)
        ),
    )
    trajectory: list[tuple[list[float], list[float]]] = []
    for index in range(1, count + 1):
        fraction = index / count
        pose = [
            start_value + (target_value - start_value) * fraction
            for start_value, target_value in zip(start, target)
        ]
        joints = validate_ik_solution(
            ik_solver(np.asarray(pose, dtype=float)), previous_joints
        )
        command_step = max(
            abs(value - previous)
            for value, previous in zip(joints, previous_joints)
        )
        command_rate = command_step / CARTESIAN_COMMAND_PERIOD_S
        if command_rate > max_joint_command_rate + 1e-9:
            raise JogSafetyError(
                "Cartesian 1 cm/s trajectory requires a "
                f"{command_rate:.3f} rad/s joint target rate; limit is "
                f"{max_joint_command_rate:.3f} rad/s."
            )
        trajectory.append((pose, joints))
        previous_joints = joints
    return trajectory


def _execute_interpolated_joint_move(
    controller: Any,
    *,
    start_joints: Sequence[float],
    target_joints: Sequence[float],
    start_pose: Sequence[float],
    target_pose: Sequence[float],
    fk_solver: Callable[[np.ndarray], Sequence[float]],
    ik_solver: Callable[[np.ndarray], Sequence[float]],
    motion_timeout: float,
    max_velocity: float,
    log_stream: TextIO,
    gripper_guard: GripperCurrentGuard | None = None,
    force_estimator: MCCEndEffectorForceEstimator | None = None,
    force_calibration_label: ForceCalibrationLabel | None = None,
    automatic_force_calibration: AutomaticForceCalibration | None = None,
    idle_pump: Callable[[], None] | None = None,
    on_key: Callable[[str], bool] | None = None,
) -> tuple[dict[str, Any], list[float], bool]:
    """Stream a 1 cm/s Cartesian target; discard motion keys but honor stops."""
    start = _finite_values(start_joints, expected=6, label="trajectory start")
    target = _finite_values(target_joints, expected=6, label="trajectory target")
    trajectory = _cartesian_joint_trajectory(
        start_pose,
        target_pose,
        start,
        ik_solver=ik_solver,
        max_joint_command_rate=min(MAX_JOINT_COMMAND_RATE_RAD_S, max_velocity),
    )
    planned_waypoint_count = len(trajectory)
    started = time.monotonic()
    deadline = started + motion_timeout
    latest_state = _diagnostic(controller)
    latest_pose = _fk_pose(latest_state, fk_solver)
    latest_force_reading = (
        None
        if force_estimator is None
        else force_estimator.observe(latest_state)
    )
    _write_log(
        log_stream,
        "trajectory_started",
        execution_mode="cartesian_target_stream",
        planned_waypoint_count=planned_waypoint_count,
        cartesian_speed_command_m_s=CARTESIAN_SPEED_M_S,
        command_period_s=CARTESIAN_COMMAND_PERIOD_S,
        max_joint_command_rate_rad_s=min(
            MAX_JOINT_COMMAND_RATE_RAD_S, max_velocity
        ),
        max_joint_command_step_rad=MAX_JOINT_COMMAND_STEP_RAD,
        target_pose_xyzrpy=list(target_pose),
        target_joints_rad=target,
    )
    next_command_at = started + CARTESIAN_COMMAND_PERIOD_S

    def poll_until(until: float, waypoint_index: int) -> bool:
        nonlocal latest_state, latest_pose, latest_force_reading
        while True:
            now = time.monotonic()
            if now >= until:
                return False
            if now >= deadline:
                return False
            if idle_pump is not None:
                idle_pump()
            readable, _, _ = select.select(
                [sys.stdin], [], [], min(CARTESIAN_COMMAND_PERIOD_S, until - now)
            )
            if readable:
                key = sys.stdin.read(1).lower()
                if on_key is not None:
                    if on_key(key):
                        _write_log(
                            log_stream,
                            "operator_stop_during_motion",
                            key=repr(key),
                            waypoint_index=waypoint_index,
                        )
                        return True
                elif key in ("q", "\x1b", " "):
                    _write_log(
                        log_stream,
                        "operator_stop_during_motion",
                        key=repr(key),
                        waypoint_index=waypoint_index,
                    )
                    return True
                if key == "p":
                    _user_print()
                    _print_state(latest_state, latest_pose)
                elif key == "g" and latest_force_reading is not None:
                    if automatic_force_calibration is not None:
                        _user_print(
                            "\n自动标定只接受机械臂静止时的 G；本次运动中按键未保存。"
                        )
                        _write_log(
                            log_stream,
                            "automatic_force_calibration_motion_sample_ignored",
                            waypoint_index=waypoint_index,
                        )
                        continue
                    # Read one coherent diagnostic sample at the G keypress so
                    # force, joints and FK pose always share the same state.
                    latest_state = _diagnostic(controller)
                    latest_pose = _fk_pose(latest_state, fk_solver)
                    latest_force_reading = force_estimator.observe(latest_state)
                    _user_print()
                    _print_mcc_force(
                        latest_force_reading, force_calibration_label
                    )
                    _write_log(
                        log_stream,
                        "mcc_end_effector_force_requested",
                        waypoint_index=waypoint_index,
                        reading=latest_force_reading,
                        calibration_label=(
                            None
                            if force_calibration_label is None
                            else force_calibration_label.as_log_record()
                        ),
                        state=latest_state,
                        fk_ee_pose_xyzrpy=latest_pose,
                    )
                elif key in KEY_DIRECTIONS:
                    _write_log(
                        log_stream,
                        "motion_key_ignored_while_busy",
                        key=key,
                        waypoint_index=waypoint_index,
                    )

            latest_state = _diagnostic(controller)
            latest_pose = _fk_pose(latest_state, fk_solver)
            if force_estimator is not None:
                latest_force_reading = force_estimator.observe(latest_state)
            velocities = _finite_values(
                latest_state["arm"]["velocity_rad_s"],
                expected=6,
                label="joint velocity",
            )
            if max(abs(value) for value in velocities) > max_velocity:
                _write_log(
                    log_stream,
                    "velocity_limit",
                    limit_rad_s=max_velocity,
                    velocity_rad_s=velocities,
                    waypoint_index=waypoint_index,
                    state=latest_state,
                    fk_ee_pose_xyzrpy=latest_pose,
                )
                raise JogSafetyError(
                    f"Measured velocity exceeded {max_velocity:.3f} rad/s."
                )
            if gripper_guard is not None:
                gripper_guard.enforce(
                    controller,
                    latest_state,
                    log_stream,
                    context="arm_trajectory",
                )

    for waypoint_index, (waypoint_pose, waypoint_joints) in enumerate(
        trajectory, start=1
    ):
        if poll_until(next_command_at, waypoint_index):
            return latest_state, latest_pose, True
        now = time.monotonic()
        if now >= deadline:
            raise JogSafetyError(
                f"Cartesian trajectory exceeded {motion_timeout:.1f} s."
            )
        controller.update_arm_position_target(waypoint_joints)
        if idle_pump is not None:
            idle_pump()
        _write_log(
            log_stream,
            "trajectory_target_sent",
            waypoint_index=waypoint_index,
            planned_waypoint_count=planned_waypoint_count,
            elapsed_s=now - started,
            target_pose_xyzrpy=waypoint_pose,
            target_joints_rad=waypoint_joints,
        )
        # Schedule from the actual send time so an overloaded process can only
        # slow the target stream down; it can never catch up by sending faster.
        next_command_at = now + CARTESIAN_COMMAND_PERIOD_S

    stable_samples = 0
    while True:
        now = time.monotonic()
        if now >= deadline:
            target_error = _distance(target_pose[:3], latest_pose[:3])
            _write_log(
                log_stream,
                "motion_timeout",
                planned_waypoint_count=planned_waypoint_count,
                state=latest_state,
                fk_ee_pose_xyzrpy=latest_pose,
                target_error_m=target_error,
            )
            raise JogSafetyError(
                f"Cartesian target was not reached within {motion_timeout:.1f} s; "
                f"position error {target_error * 100.0:.2f} cm."
            )
        if poll_until(
            min(deadline, now + CARTESIAN_COMMAND_PERIOD_S),
            planned_waypoint_count,
        ):
            return latest_state, latest_pose, True
        target_error = _distance(target_pose[:3], latest_pose[:3])
        velocities = _finite_values(
            latest_state["arm"]["velocity_rad_s"],
            expected=6,
            label="joint velocity",
        )
        if (
            target_error <= TARGET_TOLERANCE_M
            and max(abs(value) for value in velocities)
            <= FEEDBACK_STABLE_VELOCITY_RAD_S
        ):
            stable_samples += 1
        else:
            stable_samples = 0
        if stable_samples >= MOTION_SETTLE_STABLE_SAMPLES:
            _write_log(
                log_stream,
                "trajectory_reached",
                elapsed_s=time.monotonic() - started,
                target_error_m=target_error,
                state=latest_state,
                fk_ee_pose_xyzrpy=latest_pose,
            )
            return latest_state, latest_pose, False


# =============================================================================
# 9. Position-mode transition and interactive keyboard state machine
# =============================================================================

def _enter_and_warm_up_position_mode(
    controller: Any,
    *,
    fk_solver: Callable[[np.ndarray], Sequence[float]],
    max_velocity: float,
    log_stream: TextIO,
    gripper_guard: GripperCurrentGuard | None = None,
    gripper_command_min_raw: float = GRIPPER_RAW_MIN,
    gripper_command_max_raw: float = GRIPPER_RAW_MAX,
    gripper_prime_target_raw: float | None = None,
    enforce_gripper_hold_excursion: bool = True,
    position_mode_warmup_s: float = POSITION_MODE_WARMUP_S,
    idle_pump: Callable[[], None] | None = None,
) -> tuple[dict[str, Any], list[float]]:
    """Enter status 5 at zero error once and require a stable 0.5 s hold.

    ``gripper_prime_target_raw`` overrides the measured-position hold so a
    linear-gripper startup can pre-load the close endpoint. Set
    ``enforce_gripper_hold_excursion`` to False when that primed target is
    intentionally far from the current encoder reading.
    """
    before = _diagnostic(controller)
    before_velocities = _finite_values(
        before["arm"]["velocity_rad_s"], expected=6, label="joint velocity"
    )
    if max(abs(value) for value in before_velocities) > (
        FEEDBACK_STABLE_VELOCITY_RAD_S
    ):
        precheck_started = time.monotonic()
        stable_samples = 0
        _write_log(
            log_stream,
            "position_mode_precheck_wait_started",
            velocity_rad_s=before_velocities,
            stable_velocity_limit_rad_s=FEEDBACK_STABLE_VELOCITY_RAD_S,
        )
        while stable_samples < FEEDBACK_STABLE_SAMPLES:
            before = _diagnostic(controller)
            if gripper_guard is not None:
                gripper_guard.enforce(
                    controller,
                    before,
                    log_stream,
                    context="position_mode_precheck",
                )
                if gripper_prime_target_raw is not None:
                    controller.set_gripper_raw_position(
                        float(gripper_prime_target_raw)
                    )
                    gripper_guard.commanded_position_raw = float(
                        gripper_prime_target_raw
                    )
            before_velocities = _finite_values(
                before["arm"]["velocity_rad_s"],
                expected=6,
                label="joint velocity",
            )
            measured_max_velocity = max(
                abs(value) for value in before_velocities
            )
            if measured_max_velocity <= FEEDBACK_STABLE_VELOCITY_RAD_S:
                stable_samples += 1
            else:
                stable_samples = 0
            elapsed = time.monotonic() - precheck_started
            _write_log(
                log_stream,
                "position_mode_precheck_sample",
                elapsed_s=elapsed,
                velocity_rad_s=before_velocities,
                measured_max_velocity_rad_s=measured_max_velocity,
                stable_samples=stable_samples,
                required_stable_samples=FEEDBACK_STABLE_SAMPLES,
            )
            if stable_samples >= FEEDBACK_STABLE_SAMPLES:
                break
            if elapsed >= POSITION_MODE_PRECHECK_MAX_S:
                raise JogSafetyError(
                    "Joint feedback did not remain below "
                    f"{FEEDBACK_STABLE_VELOCITY_RAD_S:.3f} rad/s for "
                    f"{FEEDBACK_STABLE_SAMPLES} consecutive samples during "
                    "the 2 s position-mode precheck."
                )
            if idle_pump is not None:
                idle_pump()
            time.sleep(POSITION_MODE_PRECHECK_PERIOD_S)

    target = _joint_positions(before)

    gripper_start_position_raw: float | None = None
    gripper_hold_target: float | None = None
    if gripper_guard is not None:
        # Vendor status=5 enables the seventh-axis position cache together with
        # the six arm joints. Prime that cache from the latest measured gripper
        # position so an uninitialized/default target cannot drive it fully
        # open during the mode transition.
        gripper_feedback = _gripper_raw_feedback(before)
        gripper_start_position_raw = float(gripper_feedback["position_raw"])
        command_min = min(
            float(gripper_command_min_raw), float(gripper_command_max_raw)
        )
        command_max = max(
            float(gripper_command_min_raw), float(gripper_command_max_raw)
        )
        if gripper_prime_target_raw is not None:
            gripper_hold_target = min(
                command_max,
                max(command_min, float(gripper_prime_target_raw)),
            )
        else:
            gripper_hold_target = min(
                command_max,
                max(
                    command_min,
                    gripper_feedback["position_raw"]
                    - gripper_guard.command_feedback_offset_raw,
                ),
            )
        gripper_guard.commanded_position_raw = gripper_hold_target
        _write_log(
            log_stream,
            "gripper_target_primed_before_position_mode",
            measured_position_raw=gripper_feedback["position_raw"],
            measured_velocity_raw_s=gripper_feedback["velocity_raw"],
            command_feedback_offset_raw=(
                gripper_guard.command_feedback_offset_raw
            ),
            hold_target_raw=gripper_hold_target,
            primed_to_requested_target=gripper_prime_target_raw is not None,
            command_bounds_raw=[command_min, command_max],
        )

    if gripper_hold_target is None:
        controller.enter_arm_position_mode(target)
    else:
        controller.enter_arm_position_mode(target, gripper_raw=gripper_hold_target)
    _write_log(
        log_stream,
        "position_mode_warmup_started",
        status=5,
        zero_error_target_joints_rad=target,
    )
    started = time.monotonic()
    stable_samples = 0
    transition_overspeed_samples = 0
    latest_state = before
    latest_pose = _fk_pose(before, fk_solver)
    while True:
        latest_state = _diagnostic(controller)
        latest_pose = _fk_pose(latest_state, fk_solver)
        if gripper_guard is not None:
            gripper_guard.enforce(
                controller,
                latest_state,
                log_stream,
                context="position_mode_warmup",
            )
            latest_gripper_feedback = _gripper_raw_feedback(latest_state)
            assert gripper_start_position_raw is not None
            assert gripper_hold_target is not None
            gripper_excursion = abs(
                latest_gripper_feedback["position_raw"]
                - gripper_start_position_raw
            )
            if (
                enforce_gripper_hold_excursion
                and gripper_excursion
                > POSITION_MODE_GRIPPER_MAX_HOLD_EXCURSION_RAW
            ):
                controller.set_gripper_raw_position(gripper_hold_target)
                _write_log(
                    log_stream,
                    "gripper_excursion_during_position_mode_entry",
                    start_position_raw=gripper_start_position_raw,
                    hold_target_raw=gripper_hold_target,
                    feedback=latest_gripper_feedback,
                    excursion_raw=gripper_excursion,
                    excursion_limit_raw=(
                        POSITION_MODE_GRIPPER_MAX_HOLD_EXCURSION_RAW
                    ),
                    protection_requested=True,
                )
                raise JogSafetyError(
                    "Gripper moved "
                    f"{gripper_excursion:.4f} raw while entering status=5; "
                    "the initialization hold limit is "
                    f"{POSITION_MODE_GRIPPER_MAX_HOLD_EXCURSION_RAW:.3f} raw."
                )
            controller.set_gripper_raw_position(gripper_hold_target)
            gripper_guard.commanded_position_raw = gripper_hold_target
        velocities = _finite_values(
            latest_state["arm"]["velocity_rad_s"],
            expected=6,
            label="joint velocity",
        )
        elapsed = time.monotonic() - started
        excursion = max(
            abs(value - start)
            for value, start in zip(_joint_positions(latest_state), target)
        )
        measured_max_velocity = max(abs(value) for value in velocities)
        transition_observation_active = (
            elapsed < POSITION_MODE_TRANSITION_OBSERVATION_S
        )
        if transition_observation_active:
            if measured_max_velocity > POSITION_MODE_TRANSITION_MAX_VELOCITY_RAD_S:
                transition_overspeed_samples += 1
            else:
                transition_overspeed_samples = 0
        _write_log(
            log_stream,
            "position_mode_warmup_sample",
            elapsed_s=elapsed,
            velocity_rad_s=velocities,
            measured_max_velocity_rad_s=measured_max_velocity,
            max_excursion_rad=excursion,
            transition_observation_active=transition_observation_active,
            transition_overspeed_samples=transition_overspeed_samples,
            gripper_feedback=(
                None if gripper_guard is None else latest_gripper_feedback
            ),
            gripper_hold_target_raw=gripper_hold_target,
        )
        if excursion > POSITION_MODE_WARMUP_MAX_EXCURSION_RAD:
            raise JogSafetyError(
                "Position-mode warmup exceeded the "
                f"{POSITION_MODE_WARMUP_MAX_EXCURSION_RAD:.3f} rad excursion limit."
            )
        if transition_observation_active:
            if (
                transition_overspeed_samples
                >= POSITION_MODE_TRANSITION_CONSECUTIVE_SAMPLES
            ):
                raise JogSafetyError(
                    "Position-mode transition exceeded 0.600 rad/s for "
                    "3 consecutive samples."
                )
        elif measured_max_velocity > max_velocity:
            raise JogSafetyError(
                f"Position-mode warmup exceeded {max_velocity:.3f} rad/s "
                "after the 300 ms transition observation window."
            )
        if elapsed >= position_mode_warmup_s and max(
            abs(value) for value in velocities
        ) <= FEEDBACK_STABLE_VELOCITY_RAD_S:
            stable_samples += 1
        elif elapsed >= position_mode_warmup_s:
            stable_samples = 0
        if stable_samples >= POSITION_MODE_WARMUP_STABLE_SAMPLES:
            _write_log(
                log_stream,
                "position_mode_warmup_passed",
                elapsed_s=elapsed,
                state=latest_state,
                fk_ee_pose_xyzrpy=latest_pose,
            )
            return latest_state, latest_pose
        if elapsed >= POSITION_MODE_WARMUP_MAX_S:
            raise JogSafetyError(
                "Position-mode warmup did not settle below 0.050 rad/s."
            )
        if idle_pump is not None:
            idle_pump()
        time.sleep(0.005)


def _interactive_loop(
    controller: Any,
    *,
    startup_pose: Sequence[float],
    fk_solver: Callable[[np.ndarray], Sequence[float]],
    ik_solver: Callable[[np.ndarray], Sequence[float]],
    motion_timeout: float,
    idle_timeout: float | None,
    max_velocity: float,
    log_stream: TextIO,
    gripper_guard: GripperCurrentGuard | None = None,
    force_estimator: MCCEndEffectorForceEstimator | None = None,
    automatic_force_calibration: AutomaticForceCalibration | None = None,
) -> None:
    commands = 0
    force_calibration_label: ForceCalibrationLabel | None = None
    _user_print(
        "按键：W/S = X+/X-，A/D = Y+/Y-，R/F = Z+/Z-；"
        "每次 1 cm，目标轨迹速度 1 cm/s。"
    )
    _user_print(
        "H = 以 1 cm/s 回到本次启动零点；p 查看状态；"
        "空格/Esc/q 保护并退出；离启动位置最远 80 cm。"
    )
    if force_estimator is not None:
        if automatic_force_calibration is None:
            _user_print(
                "G = 打印夹爪整体受到的三轴外力；"
                "当前为 MCC 未标定代理值，不是牛顿。运动过程中也可按 G。"
            )
            _user_print(
                "L = 输入/更新本组标定标签（盒子质量、砝码质量和姿态）；"
                "此后每次 G 都会立即把标签和完整六轴状态写入 JSONL。"
            )
        else:
            _user_print(
                "G = 保存一个静止标定样本；机械臂未静止或正在运动时不保存。"
                "当前仍为 MCC 未标定代理值，不是牛顿。"
            )
            _user_print(
                "自动20样本标定已启用：按提示摆放砝码，等待静止后按 N 确认，"
                "再按 G 采样；每组完成后程序自动提示下一组。"
            )
            _user_print(automatic_force_calibration.instruction())
            _write_log(
                log_stream,
                "automatic_force_calibration_started",
                box_mass_g=automatic_force_calibration.box_mass_g,
                pose_label=automatic_force_calibration.pose_label,
                samples_per_load=automatic_force_calibration.samples_per_load,
                load_plan=[
                    {
                        "direction": direction,
                        "added_mass_g": mass,
                        "instruction": instruction,
                    }
                    for direction, mass, instruction in FORCE_CALIBRATION_LOAD_PLAN
                ],
            )
    if gripper_guard is not None:
        _user_print(
            "O = 完全张开并采集空载effort0基线；"
            "C = 向闭合方向连续闭合，夹到物体后再闭合 1 mm 并停止。"
            "闭合前必须先按 O；低速连续 3 次effort0增量超限会再闭合 1 mm 后锁定，"
            "不会退出机械臂控制。"
        )
        _user_print(
            "按 O 前必须托住盒子：张开阶段连续 3 次超过 1.000 rad/s，或任一关节"
            "相对按键前偏移超过 0.050 rad，会进入整臂保护；正常运动仍为 "
            "0.500 rad/s 单样本保护，等待按键时为连续 3 样本保护。"
        )
        _user_print(
            "K = 在夹爪完全空载时自动扫描并保存位置—effort0曲线；"
            "首次使用必须先做 K 标定。"
        )
    with _terminal_keys(sys.stdin) as terminal_attributes:
        while True:
            key = _read_key_with_gripper_monitor(
                idle_timeout,
                controller=controller,
                gripper_guard=gripper_guard,
                max_velocity=max_velocity,
                log_stream=log_stream,
            )
            if key is None:
                continue
            key = key.lower()
            if key in ("q", "\x1b", " "):
                _user_print("\n操作员停止，进入保护模式。")
                _write_log(log_stream, "operator_stop", key=repr(key))
                return
            if key == "p":
                state = _diagnostic(controller)
                pose = _fk_pose(state, fk_solver)
                _user_print()
                _print_state(state, pose)
                if gripper_guard is not None:
                    feedback = _gripper_raw_feedback(state)
                    baseline = gripper_guard.baseline_current_raw
                    expected_current = (
                        None
                        if baseline is None
                        else gripper_guard.expected_no_load_current(
                            feedback["position_raw"]
                        )
                    )
                    residual = (
                        None
                        if expected_current is None
                        else abs(feedback["effort0"] - expected_current)
                    )
                    _user_print(
                        "夹爪原始反馈："
                        f"位置={feedback['position_raw']:.4f}，"
                        f"速度={feedback['velocity_raw']:.4f}，"
                        f"effort0={feedback['effort0']:.4f}，"
                        f"全开基线={baseline}，"
                        f"该位置空载预期={expected_current}，"
                        f"接触effort0残差={residual}，"
                        f"接触锁定={gripper_guard.contact_latched}，"
                        f"空载曲线点数={len(gripper_guard.no_load_curve)}"
                    )
                _write_log(log_stream, "state", state=state, fk_ee_pose_xyzrpy=pose)
                continue
            if key == "n" and automatic_force_calibration is not None:
                if automatic_force_calibration.completed:
                    _user_print("\n自动标定序列已经完成，不需要再次确认。")
                    continue
                if not automatic_force_calibration.awaiting_confirmation:
                    _user_print(
                        "\n当前组已经确认，请继续按 G；"
                        f"已保存 {automatic_force_calibration.samples_in_step}/"
                        f"{automatic_force_calibration.samples_per_load} 次。"
                    )
                    continue
                force_calibration_label = (
                    automatic_force_calibration.confirm_current_step()
                )
                label_record = force_calibration_label.as_log_record()
                _write_log(
                    log_stream,
                    "force_calibration_label_updated",
                    source="automatic_confirmed_load_sequence",
                    calibration_label=label_record,
                    automatic_group=(
                        automatic_force_calibration.sample_metadata()
                    ),
                )
                _user_print(
                    "\n本组已确认："
                    f"{label_record['sample_label']}，"
                    f"砝码={label_record['added_mass_g']:.0f} g，"
                    f"盒子+砝码={label_record['total_suspended_mass_g']:.3f} g。"
                    f"现在按 G 保存 {automatic_force_calibration.samples_per_load} 次。"
                )
                continue
            if key == "g" and force_estimator is not None:
                automatic_sample = None
                if automatic_force_calibration is not None:
                    if automatic_force_calibration.completed:
                        _user_print("\n自动标定序列已完成；本次 G 未继续追加。")
                        continue
                    if automatic_force_calibration.awaiting_confirmation:
                        _user_print("\n尚未确认本组砝码，G 未保存。")
                        _user_print(automatic_force_calibration.instruction())
                        continue
                    automatic_sample = (
                        automatic_force_calibration.sample_metadata()
                    )
                state = _diagnostic(controller)
                pose = _fk_pose(state, fk_solver)
                if automatic_force_calibration is not None:
                    calibration_velocity = max(
                        abs(value)
                        for value in _finite_values(
                            state["arm"]["velocity_rad_s"],
                            expected=6,
                            label="joint velocity",
                        )
                    )
                    if calibration_velocity > MCC_FORCE_STATIONARY_VELOCITY_RAD_S:
                        _user_print(
                            "\n机械臂尚未静止，本次 G 未保存："
                            f"最大关节速度 {calibration_velocity:.4f} rad/s，"
                            f"需要不超过 {MCC_FORCE_STATIONARY_VELOCITY_RAD_S:.3f} rad/s。"
                        )
                        _write_log(
                            log_stream,
                            "automatic_force_calibration_moving_sample_rejected",
                            automatic_calibration=automatic_sample,
                            max_joint_velocity_rad_s=calibration_velocity,
                            limit_rad_s=MCC_FORCE_STATIONARY_VELOCITY_RAD_S,
                            state=state,
                            fk_ee_pose_xyzrpy=pose,
                        )
                        continue
                reading = force_estimator.observe(state)
                _user_print()
                _print_mcc_force(reading, force_calibration_label)
                _write_log(
                    log_stream,
                    "mcc_end_effector_force_requested",
                    reading=reading,
                    calibration_label=(
                        None
                        if force_calibration_label is None
                        else force_calibration_label.as_log_record()
                    ),
                    automatic_calibration=automatic_sample,
                    state=state,
                    fk_ee_pose_xyzrpy=pose,
                )
                if automatic_force_calibration is not None:
                    group_finished = automatic_force_calibration.record_sample()
                    if automatic_force_calibration.completed:
                        force_calibration_label = None
                        _user_print(
                            "自动标定全部完成：15 组数据均已保存。"
                            "请托住盒子后按 O 松开，或按 q 退出。"
                        )
                        _write_log(
                            log_stream,
                            "automatic_force_calibration_completed",
                        )
                    elif group_finished:
                        force_calibration_label = None
                        _user_print(
                            "本组"
                            f"{automatic_force_calibration.samples_per_load}"
                            "次采样已完成。"
                        )
                        _user_print(automatic_force_calibration.instruction())
                        _write_log(
                            log_stream,
                            "automatic_force_calibration_group_completed",
                            completed_group_index=automatic_sample["group_index"],
                            next_group_index=(
                                automatic_force_calibration.step_index + 1
                            ),
                        )
                    else:
                        _user_print(
                            "自动标定进度："
                            f"{automatic_force_calibration.samples_in_step}/"
                            f"{automatic_force_calibration.samples_per_load}。"
                        )
                continue
            if key == "l" and force_estimator is not None:
                if automatic_force_calibration is not None:
                    _user_print(
                        "\n自动标定已启用，L 手动标签已锁定；"
                        "请按当前提示摆放砝码并按 N。"
                    )
                    continue
                try:
                    updated_label = _prompt_force_calibration_label(
                        force_calibration_label,
                        input_stream=sys.stdin,
                        terminal_attributes=terminal_attributes,
                    )
                except ValueError as exc:
                    _user_print(f"\n标签输入无效，保留原标签：{exc}")
                    continue
                if updated_label is None:
                    _user_print("\n标签输入已取消，保留原标签。")
                    continue
                force_calibration_label = updated_label
                label_record = updated_label.as_log_record()
                _write_log(
                    log_stream,
                    "force_calibration_label_updated",
                    calibration_label=label_record,
                )
                _user_print(
                    "\n标定标签已更新："
                    f"{label_record['sample_label']}，"
                    f"总悬挂质量={label_record['total_suspended_mass_g']:.3f} g，"
                    f"理论重力={label_record['expected_gravity_force_n']:.5f} N，"
                    f"姿态={label_record['pose_label']}。"
                )
                continue
            if key == "o" and gripper_guard is not None:
                try:
                    open_reference_state = _diagnostic(controller)
                    state = gripper_guard.open_and_calibrate(
                        controller,
                        max_velocity=max_velocity,
                        log_stream=log_stream,
                        arm_reference_positions=open_reference_state["arm"][
                            "position_rad"
                        ],
                    )
                except GripperOpenTimeoutError as exc:
                    _user_print(f"\n夹爪张开未确认稳定，机械臂控制继续：{exc}")
                    continue
                feedback = _gripper_raw_feedback(state)
                _user_print(
                    "\n夹爪已张开；空载effort0基线="
                    f"{gripper_guard.baseline_current_raw:.4f}，"
                    f"当前位置={feedback['position_raw']:.4f} raw。"
                )
                continue
            if key == "k" and gripper_guard is not None:
                _user_print(
                    "\n开始空载曲线标定：确认两个夹爪之间没有任何物体，"
                    "扫描过程中不要触碰夹爪。"
                )
                try:
                    gripper_guard.calibrate_no_load_curve(
                        controller,
                        max_velocity=max_velocity,
                        log_stream=log_stream,
                    )
                except GripperOpenTimeoutError as exc:
                    _user_print(f"\n空载标定因张开未稳定而中止，机械臂控制继续：{exc}")
                continue
            if key == "c" and gripper_guard is not None:
                if gripper_guard.baseline_current_raw is None:
                    _user_print("\n请先按 O，张开夹爪并采集空载effort0基线。")
                    continue
                if not gripper_guard.has_no_load_curve:
                    _user_print(
                        "\n尚无空载位置—effort0曲线；请清空夹爪后按 K 完成标定。"
                    )
                    continue
                if gripper_guard.contact_latched:
                    _user_print(
                        "\n夹爪已因接触物体停止闭合；请先按 O 张开并重新采集基线。"
                    )
                    continue
                state = gripper_guard.close_until_grasp(
                    controller,
                    max_velocity=max_velocity,
                    log_stream=log_stream,
                )
                if gripper_guard.contact_latched:
                    continue
                feedback = _gripper_raw_feedback(state)
                expected_current = gripper_guard.expected_no_load_current(
                    feedback["position_raw"]
                )
                residual = abs(
                    feedback["effort0"]
                    - expected_current
                )
                _user_print(
                    "\n夹爪已到达闭合端，未检测到接触："
                    f"位置={feedback['position_raw']:.4f} raw，"
                    f"空载预期effort0={expected_current:.4f}，"
                    f"接触effort0残差={residual:.4f}/"
                    f"{gripper_guard.current_delta_limit_raw:.4f}。"
                )
                continue
            if key not in KEY_DIRECTIONS and key != "h":
                continue

            before = _diagnostic(controller)
            before_pose = _fk_pose(before, fk_solver)
            if key == "h":
                target_pose = build_return_target(before_pose, startup_pose)
                command_type = "return_to_start"
                if _distance(before_pose[:3], target_pose[:3]) <= TARGET_TOLERANCE_M:
                    _user_print("\n末端已经位于本次启动零点附近。")
                    _write_log(
                        log_stream,
                        "return_to_start_already_reached",
                        state=before,
                        fk_ee_pose_xyzrpy=before_pose,
                    )
                    continue
                # The complete return may exceed the normal one-key 0.20 rad
                # final-jump check. The preplanned 50 Hz path below still checks
                # every adjacent joint command against the 0.20 rad/s target-rate bound.
                raw_solution = _finite_values(
                    ik_solver(np.asarray(target_pose, dtype=float)),
                    expected=6,
                    label="IK solution",
                )
                ik_solution = validate_ik_solution(raw_solution, raw_solution)
            else:
                axis, direction = KEY_DIRECTIONS[key]
                target_pose = build_cartesian_target(
                    before_pose,
                    startup_pose,
                    axis=axis,
                    direction=direction,
                )
                command_type = "cartesian_step"
                ik_solution = validate_ik_solution(
                    ik_solver(np.asarray(target_pose, dtype=float)),
                    _joint_positions(before),
                )
            commands += 1
            _write_log(
                log_stream,
                "command_requested",
                key=key,
                command_type=command_type,
                command_index=commands,
                target_pose_xyzrpy=target_pose,
                predicted_joints_rad=ik_solution,
                before=before,
                before_fk_ee_pose_xyzrpy=before_pose,
            )
            after, after_pose, operator_stopped = _execute_interpolated_joint_move(
                controller,
                start_joints=_joint_positions(before),
                target_joints=ik_solution,
                start_pose=before_pose,
                target_pose=target_pose,
                fk_solver=fk_solver,
                ik_solver=ik_solver,
                motion_timeout=motion_timeout,
                max_velocity=max_velocity,
                log_stream=log_stream,
                gripper_guard=gripper_guard,
                force_estimator=force_estimator,
                force_calibration_label=force_calibration_label,
                automatic_force_calibration=automatic_force_calibration,
            )
            actual_step = _distance(before_pose[:3], after_pose[:3])
            actual_excursion = _distance(startup_pose[:3], after_pose[:3])
            target_error = _distance(target_pose[:3], after_pose[:3])
            _write_log(
                log_stream,
                "command",
                key=key,
                command_type=command_type,
                command_index=commands,
                target_pose_xyzrpy=target_pose,
                predicted_joints_rad=ik_solution,
                before=before,
                before_fk_ee_pose_xyzrpy=before_pose,
                after=after,
                after_fk_ee_pose_xyzrpy=after_pose,
                actual_step_m=actual_step,
                actual_excursion_m=actual_excursion,
                target_error_m=target_error,
            )
            _user_print()
            _print_state(after, after_pose)
            requested_step = _distance(before_pose[:3], target_pose[:3])
            if actual_step > requested_step + TARGET_TOLERANCE_M:
                raise JogSafetyError(
                    f"Measured EE step {actual_step * 100.0:.2f} cm is too large."
                )
            if target_error > TARGET_TOLERANCE_M:
                raise JogSafetyError(
                    f"Measured EE target error {target_error * 100.0:.2f} cm "
                    "exceeds 0.20 cm."
                )
            if actual_excursion > MAX_DISPLACEMENT_M + 0.002:
                raise JogSafetyError("Measured EE displacement exceeded 80 cm.")
            if operator_stopped:
                _user_print("操作员在运动等待期间停止，进入保护模式。")
                return


# =============================================================================
# 10. Process lifecycle and protection-mode cleanup
# =============================================================================

def main(argv: Sequence[str] | None = None) -> None:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        _validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    if args.enable_motion and not args.acknowledge_limited_feedback:
        parser.error(
            "--enable-motion also requires --acknowledge-limited-feedback."
        )
    if args.enable_motion and not args.acknowledge_transition_grace_risk:
        parser.error(
            "--enable-motion also requires "
            "--acknowledge-transition-grace-risk."
        )
    if args.enable_gripper and not args.enable_motion:
        parser.error("--enable-gripper requires --enable-motion.")
    if args.enable_gripper and not args.acknowledge_unverified_gripper_current:
        parser.error(
            "--enable-gripper also requires "
            "--acknowledge-unverified-gripper-current."
        )
    if (
        args.auto_force_calibration_box_mass_g is not None
        and not args.enable_gripper
    ):
        parser.error(
            "--auto-force-calibration-box-mass-g requires --enable-gripper."
        )
    if args.enable_motion:
        try:
            _validate_motion_scope(args)
        except ValueError as exc:
            parser.error(str(exc))
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    log_path = resolve_session_log_path(
        args.log,
        f"x5_2023_cartesian_jog_{args.interface}_{timestamp}.jsonl",
    )

    controllers: list[Any] = []
    force_estimator: MCCEndEffectorForceEstimator | None = None
    with log_path.open("a", encoding="utf-8") as log_stream:
        _write_log(
            log_stream,
            "start",
            interface=args.interface,
            motion_enabled=bool(args.enable_motion),
            mcc_end_effector_force_enabled=bool(args.enable_motion),
            mcc_end_effector_force_calibrated=False,
            gripper_enabled=bool(args.enable_gripper),
            cartesian_step_m=CARTESIAN_STEP_M,
            cartesian_speed_command_m_s=CARTESIAN_SPEED_M_S,
            max_displacement_m=MAX_DISPLACEMENT_M,
            execution_mode="cartesian_target_stream",
            command_period_s=CARTESIAN_COMMAND_PERIOD_S,
            max_joint_command_rate_rad_s=MAX_JOINT_COMMAND_RATE_RAD_S,
            max_joint_command_step_rad=MAX_JOINT_COMMAND_STEP_RAD,
            max_measured_velocity_rad_s=float(args.max_velocity),
            transition_observation_s=POSITION_MODE_TRANSITION_OBSERVATION_S,
            transition_max_velocity_rad_s=(
                POSITION_MODE_TRANSITION_MAX_VELOCITY_RAD_S
            ),
            transition_consecutive_samples=(
                POSITION_MODE_TRANSITION_CONSECUTIVE_SAMPLES
            ),
            transition_max_excursion_rad=(
                POSITION_MODE_WARMUP_MAX_EXCURSION_RAD
            ),
            gripper_open_raw=float(args.gripper_open_raw),
            gripper_closed_raw=float(args.gripper_closed_raw),
            gripper_step_raw=float(args.gripper_step_raw),
            gripper_current_delta_limit_raw=(
                None
                if args.gripper_current_delta_limit_raw is None
                else float(args.gripper_current_delta_limit_raw)
            ),
            automatic_force_calibration_enabled=(
                args.auto_force_calibration_box_mass_g is not None
            ),
            automatic_force_calibration_box_mass_g=(
                None
                if args.auto_force_calibration_box_mass_g is None
                else float(args.auto_force_calibration_box_mass_g)
            ),
            force_calibration_pose_label=str(args.force_calibration_pose_label),
            force_calibration_samples_per_load=int(
                args.force_calibration_samples_per_load
            ),
        )
        try:
            sdk = backend.load_bimanual_sdk(sdk_path=args.sdk_path)
            fk_solver = getattr(sdk, "forward_kinematics", None)
            ik_solver = getattr(sdk, "inverse_kinematics", None)
            if not callable(fk_solver):
                raise JogSafetyError("Vendor SDK does not expose forward_kinematics().")
            if not callable(ik_solver):
                raise JogSafetyError("Vendor SDK does not expose inverse_kinematics().")
            controllers = backend.create_controllers(
                args.interface,
                [],
                [],
                [],
                [],
                [],
                0,
                1,
                sdk_path=args.sdk_path,
            )
            if len(controllers) != 1:
                raise JogSafetyError(
                    f"Expected exactly one controller, got {len(controllers)}."
                )
            controller = controllers[0]
            gripper_guard = None
            if args.enable_gripper:
                gripper_curve_path = (
                    Path(__file__).resolve().parents[1]
                    / f"x5_2023_gripper_no_load_curve_{args.interface}.json"
                    if args.gripper_no_load_curve is None
                    else args.gripper_no_load_curve.expanduser().resolve()
                )
                gripper_guard = GripperCurrentGuard(
                    current_delta_limit_raw=float(
                        args.gripper_current_delta_limit_raw
                    ),
                    open_raw=float(args.gripper_open_raw),
                    closed_raw=float(args.gripper_closed_raw),
                    step_raw=float(args.gripper_step_raw),
                    no_load_curve_path=gripper_curve_path,
                )
                _write_log(
                    log_stream,
                    "gripper_no_load_curve_status",
                    curve_path=str(gripper_curve_path),
                    loaded=gripper_guard.has_no_load_curve,
                    point_count=len(gripper_guard.no_load_curve),
                )
            backend.disable_motors(controllers)
            backend.initialize(controllers)
            with _filtered_vendor_output():
                _user_print("等待厂家关节反馈稳定（最多 5 秒）……")
                initial_state, startup_pose = _warm_up_feedback(
                    controller,
                    fk_solver=fk_solver,
                )
                if gripper_guard is not None:
                    feedback = controller.get_experimental_gripper_raw_feedback()
                    _write_log(
                        log_stream,
                        "gripper_raw_feedback_available",
                        feedback=feedback,
                    )
                _write_log(
                    log_stream,
                    "initial_state",
                    state=initial_state,
                    fk_ee_pose_xyzrpy=startup_pose,
                )
                _user_print(f"接口：{args.interface}；日志：{log_path}")
                _print_state(initial_state, startup_pose)

                if not args.enable_motion:
                    _user_print(
                        "预览完成：未发送运动命令。实验运动还需要两个风险确认参数，"
                        "并且只允许 can2。"
                    )
                    return
                if not sys.stdin.isatty():
                    raise JogSafetyError("Motion mode requires an interactive terminal.")

                _user_print(
                    "\n确认工作区清空、机械臂固定、急停可触达，"
                    "夹爪整体未受外力，并且只操作这一台机械臂。"
                )
                _user_print(f"输入 {MOTION_CONFIRMATION} 后按回车：")
                answer = sys.stdin.readline().strip()
                if answer != MOTION_CONFIRMATION:
                    _user_print("确认文字不匹配，未发送运动命令。")
                    _write_log(log_stream, "confirmation_rejected")
                    return

                def stop_on_signal(signum: int, _frame: Any) -> None:
                    raise KeyboardInterrupt(f"signal {signum}")

                previous_sigterm = signal.signal(signal.SIGTERM, stop_on_signal)
                try:
                    _user_print(
                        "零误差进入 status=5：前 300 ms 使用 0.6 rad/s/连续 3 样本规则，"
                        "位置偏移限制为 0.040 rad；随后恢复 "
                        f"{float(args.max_velocity):.3f} rad/s……"
                    )
                    warm_state, warm_pose = _enter_and_warm_up_position_mode(
                        controller,
                        fk_solver=fk_solver,
                        max_velocity=float(args.max_velocity),
                        log_stream=log_stream,
                        gripper_guard=gripper_guard,
                    )
                    force_estimator = MCCEndEffectorForceEstimator(
                        fk_solver=fk_solver,
                    )
                    force_estimator.tare(warm_state)
                    tare_reading = force_estimator.observe(warm_state)
                    _user_print(
                        "MCC 夹爪整体外力已在当前静止姿态校零；"
                        "六轴 effort 尚未标定，G 键结果不是牛顿。"
                    )
                    _write_log(
                        log_stream,
                        "mcc_end_effector_force_tared",
                        reading=tare_reading,
                    )
                    automatic_force_calibration = None
                    if args.auto_force_calibration_box_mass_g is not None:
                        automatic_force_calibration = AutomaticForceCalibration(
                            box_mass_g=float(
                                args.auto_force_calibration_box_mass_g
                            ),
                            pose_label=str(args.force_calibration_pose_label),
                            samples_per_load=int(
                                args.force_calibration_samples_per_load
                            ),
                        )
                    _print_state(warm_state, warm_pose)
                    _interactive_loop(
                        controller,
                        startup_pose=warm_pose,
                        fk_solver=fk_solver,
                        ik_solver=ik_solver,
                        motion_timeout=float(args.motion_timeout),
                        idle_timeout=args.idle_timeout,
                        max_velocity=float(args.max_velocity),
                        log_stream=log_stream,
                        gripper_guard=gripper_guard,
                        force_estimator=force_estimator,
                        automatic_force_calibration=(
                            automatic_force_calibration
                        ),
                    )
                finally:
                    signal.signal(signal.SIGTERM, previous_sigterm)
        except KeyboardInterrupt as exc:
            _write_log(log_stream, "error", error=repr(exc))
            _user_print("\n收到中断，进入保护模式。")
        except JogSafetyError as exc:
            _write_log(log_stream, "safety_stop", error=repr(exc))
            _user_print(f"\n安全停止：{exc}")
            raise SystemExit(2) from None
        except Exception as exc:
            _write_log(log_stream, "error", error=repr(exc))
            _user_print(f"\n未处理异常，进入保护模式：{exc}")
            raise
        finally:
            if force_estimator is not None:
                force_estimator.close()
            if controllers:
                try:
                    backend.disable_motors(controllers)
                    _write_log(log_stream, "protection_requested")
                finally:
                    backend.close(controllers)
            _write_log(log_stream, "closed")


if __name__ == "__main__":
    main()
