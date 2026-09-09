"""Safety-focused adapter for the vendor X5 ``bimanual`` SDK.

The vendor SDK is loaded lazily so importing the rest of this repository does
not require its platform-specific CPython extension.  In the bundled SDK,
``type=0`` explicitly selects X5-2023.  The SDK has no named continuous gripper
feedback methods, but its installed C++ header documents the joint position,
velocity, and current vectors as seven-dimensional.  General policy execution
still treats the seventh value as uncalibrated ``effort0``; the bounded
keyboard tool may use it only through the explicit experimental surface below.
"""

from __future__ import annotations

import importlib
import inspect
import math
import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any, Dict, Iterable, List, Sequence

ARM_DOF = 6
TOTAL_ACTUATORS = 7
X5_2023_TYPE = 0
X5_2023_MODEL = "X5-2023"
X5_2023_GRIPPER_RAW_MIN = 0.0
X5_2023_GRIPPER_RAW_MAX = 5.0

REQUIRED_X5_2023_METHODS = (
    "get_joint_positions",
    "get_joint_velocities",
    "get_joint_currents",
    "get_catch_status",
    "set_joint_positions",
    "set_gripper_pos",
    "protect_mode",
)
CONTINUOUS_GRIPPER_METHODS = (
    "get_gripper_pos",
    "get_gripper_vel",
    "get_gripper_current",
)


class ARXBetaSafetyError(RuntimeError):
    """Raised when the vendor SDK reports a state that must block motion."""


def inspect_x5_2023_sdk(module: ModuleType) -> Dict[str, Any]:
    """Check the non-hardware API contract of a vendor ``bimanual`` module."""
    single_arm = getattr(module, "SingleArm", None)
    if single_arm is None:
        raise ImportError("Vendor 'bimanual' module does not expose SingleArm.")

    missing = [
        name
        for name in REQUIRED_X5_2023_METHODS
        if not callable(getattr(single_arm, name, None))
    ]
    if missing:
        raise ImportError(
            "Vendor X5 SDK is missing required SingleArm methods: "
            + ", ".join(missing)
        )
    if not callable(getattr(module, "forward_kinematics", None)):
        raise ImportError("Vendor X5 SDK does not expose forward_kinematics().")

    return {
        "robot_model": X5_2023_MODEL,
        "sdk_type": X5_2023_TYPE,
        "continuous_gripper_feedback": all(
            callable(getattr(single_arm, name, None))
            for name in CONTINUOUS_GRIPPER_METHODS
        ),
        "fault_reporting": hasattr(single_arm, "fault"),
        "offline_joint_reporting": hasattr(single_arm, "offline_joints"),
        "explicit_close": callable(getattr(single_arm, "close", None)),
    }


def _finite_float_list(value: Any, *, expected: int, label: str) -> List[float]:
    values = _finite_float_sequence(value, label=label)
    if len(values) != expected:
        raise ValueError(f"{label} length {len(values)} != {expected}.")
    return values


def _finite_float_sequence(value: Any, *, label: str) -> List[float]:
    try:
        values = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a numeric sequence.") from exc
    if not all(math.isfinite(item) for item in values):
        raise ValueError(f"{label} contains NaN or Inf.")
    return values


def _split_x5_feedback(
    value: Any, *, label: str
) -> tuple[List[float], float | None, int]:
    """Split documented six-axis data from an optional unverified 7th value."""
    values = _finite_float_sequence(value, label=label)
    if len(values) not in (ARM_DOF, TOTAL_ACTUATORS):
        raise ValueError(
            f"{label} length {len(values)} is unsupported; expected 6 or 7."
        )
    extra = values[ARM_DOF] if len(values) == TOTAL_ACTUATORS else None
    return values[:ARM_DOF], extra, len(values)


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name, "").strip()
    value = default if not raw else float(raw)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite.")
    return value


@dataclass(frozen=True)
class GripperMapping:
    """Affine map between vendor raw gripper radians and model coordinates."""

    raw_min: float = 0.0
    raw_max: float = 5.0
    model_min: float = 0.0
    model_max: float = 0.044

    def __post_init__(self) -> None:
        values = (self.raw_min, self.raw_max, self.model_min, self.model_max)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Gripper mapping bounds must be finite.")
        if self.raw_max <= self.raw_min:
            raise ValueError("raw_max must be greater than raw_min.")
        if self.model_max <= self.model_min:
            raise ValueError("model_max must be greater than model_min.")

    @classmethod
    def from_env(cls) -> "GripperMapping":
        return cls(
            raw_min=_env_float("ARX5_BETA_GRIPPER_RAW_MIN", 0.0),
            raw_max=_env_float("ARX5_BETA_GRIPPER_RAW_MAX", 5.0),
            model_min=_env_float("ARX5_BETA_GRIPPER_MODEL_MIN", 0.0),
            model_max=_env_float("ARX5_BETA_GRIPPER_MODEL_MAX", 0.044),
        )

    def raw_to_model(self, raw_position: float) -> float:
        raw = float(raw_position)
        if not math.isfinite(raw):
            raise ValueError("Raw gripper position must be finite.")
        tolerance = 1e-6
        if raw < self.raw_min - tolerance or raw > self.raw_max + tolerance:
            raise ValueError(
                f"Raw gripper position {raw} is outside "
                f"[{self.raw_min}, {self.raw_max}]."
            )
        raw = min(max(raw, self.raw_min), self.raw_max)
        alpha = (raw - self.raw_min) / (self.raw_max - self.raw_min)
        return self.model_min + alpha * (self.model_max - self.model_min)

    def model_to_raw(self, model_position: float) -> float:
        model = float(model_position)
        if not math.isfinite(model):
            raise ValueError("Model gripper position must be finite.")
        tolerance = 1e-6
        if model < self.model_min - tolerance or model > self.model_max + tolerance:
            raise ValueError(
                f"Model gripper position {model} is outside "
                f"[{self.model_min}, {self.model_max}]."
            )
        model = min(max(model, self.model_min), self.model_max)
        alpha = (model - self.model_min) / (self.model_max - self.model_min)
        return self.raw_min + alpha * (self.raw_max - self.raw_min)


def load_bimanual_sdk(sdk_path: str | None = None) -> ModuleType:
    """Load the vendor SDK from an installed package or external bundle."""
    path_value = (
        sdk_path
        or os.getenv("ARX5_VENDOR_SDK_PATH", "")
        or os.getenv("ARX5_BETA_SDK_PATH", "")
    ).strip()
    if path_value:
        sdk_root = Path(path_value).expanduser().resolve()
        if not sdk_root.is_dir():
            raise ImportError(f"Vendor X5 SDK path does not exist: {sdk_root}")
        root_str = str(sdk_root)
        if root_str not in sys.path:
            sys.path.insert(0, root_str)

    try:
        module = importlib.import_module("bimanual")
    except ImportError as exc:
        raise ImportError(
            "Could not import the vendor X5 'bimanual' SDK. The bundled extension "
            "is built for CPython 3.12 on x86_64. Use the bundled launcher or "
            "set ARX5_VENDOR_SDK_PATH to its parent directory."
        ) from exc
    inspect_x5_2023_sdk(module)
    return module


def _discover_interfaces(port_pattern: str) -> List[str]:
    """Resolve a literal SocketCAN interface or a regular-expression pattern."""
    if not port_pattern:
        raise ValueError("ARX5 interface must not be empty.")
    has_regex_meta = bool(re.search(r"[.^$*+?{}\[\]|()\\]", port_pattern))
    if not has_regex_meta:
        return [port_pattern]

    try:
        pattern = re.compile(port_pattern)
    except re.error as exc:
        raise ValueError(f"Invalid ARX5 interface pattern: {port_pattern!r}") from exc
    net_dir = Path("/sys/class/net")
    interfaces = (
        sorted(entry.name for entry in net_dir.iterdir()) if net_dir.is_dir() else []
    )
    matched = [name for name in interfaces if pattern.fullmatch(name)]
    if not matched:
        raise RuntimeError(
            f"No SocketCAN interface matched {port_pattern!r}; detected {interfaces}."
        )
    return matched


class ARXBetaControl:
    """Dynamixel-like surface backed by ``bimanual.SingleArm``."""

    def __init__(
        self,
        interface: str,
        arm: Any,
        gripper_mapping: GripperMapping | None = None,
        command_duration: float = 0.02,
    ) -> None:
        self.interface = str(interface)
        self.arm = arm
        self.gripper_mapping = gripper_mapping or GripperMapping.from_env()
        self.command_duration = float(command_duration)
        if not math.isfinite(self.command_duration) or self.command_duration <= 0.0:
            raise ValueError("command_duration must be finite and positive.")

    def capabilities(self) -> Dict[str, Any]:
        return {
            "robot_model": X5_2023_MODEL,
            "sdk_type": X5_2023_TYPE,
            "continuous_gripper_feedback": all(
                callable(getattr(self.arm, name, None))
                for name in CONTINUOUS_GRIPPER_METHODS
            ),
            "catch_status_available": callable(
                getattr(self.arm, "get_catch_status", None)
            ),
            "fault_reporting": hasattr(self.arm, "fault"),
            "offline_joint_reporting": hasattr(self.arm, "offline_joints"),
            "explicit_close": callable(getattr(self.arm, "close", None)),
        }

    def _raise_if_faulted(self) -> None:
        fault = getattr(self.arm, "fault", None)
        if fault:
            raise ARXBetaSafetyError(f"Vendor X5 controller fault: {fault}")
        offline = getattr(self.arm, "offline_joints", None)
        if offline is not None:
            try:
                offline_values = list(offline)
            except TypeError:
                offline_values = [offline] if offline else []
            if offline_values:
                raise ARXBetaSafetyError(
                    f"Vendor X5 offline joints: {offline_values}"
                )

    def initialize_motors(self) -> None:
        """Validate six-axis feedback without moving or automatically homing."""
        self._raise_if_faulted()
        self.get_diagnostic_state()

    def get_motor_ids(self) -> List[int]:
        return list(range(TOTAL_ACTUATORS))

    def get_diagnostic_state(self) -> Dict[str, Any]:
        """Read only feedback that the installed X5-2023 SDK really provides."""
        self._raise_if_faulted()
        positions, extra_position, position_length = _split_x5_feedback(
            self.arm.get_joint_positions(), label="joint positions"
        )
        velocities, extra_velocity, velocity_length = _split_x5_feedback(
            self.arm.get_joint_velocities(), label="joint velocities"
        )
        effort, extra_effort, effort_length = _split_x5_feedback(
            self.arm.get_joint_currents(), label="joint effort"
        )
        catch_status = None
        get_catch_status = getattr(self.arm, "get_catch_status", None)
        if callable(get_catch_status):
            catch_status = bool(get_catch_status())
        return {
            "arm": {
                "position_rad": positions,
                "velocity_rad_s": velocities,
                "effort_raw": effort,
            },
            "gripper": {
                "catch_status": catch_status,
                "continuous_feedback_available": self.capabilities()[
                    "continuous_gripper_feedback"
                ],
            },
            "additional_sdk_feedback": {
                "present": any(
                    value is not None
                    for value in (extra_position, extra_velocity, extra_effort)
                ),
                "index": ARM_DOF,
                "position_raw": extra_position,
                "velocity_raw": extra_velocity,
                "effort0": extra_effort,
                "semantics": (
                    "unverified; do not treat as gripper feedback until checked "
                    "against X5-2023 hardware"
                ),
            },
            "raw_vector_lengths": {
                "position": position_length,
                "velocity": velocity_length,
                "effort": effort_length,
            },
            "capabilities": self.capabilities(),
        }

    def get_experimental_gripper_raw_feedback(self) -> Dict[str, Any]:
        """Return the documented seventh vector entries without unit claims."""
        state = self.get_diagnostic_state()
        lengths = state["raw_vector_lengths"]
        if any(lengths[name] != TOTAL_ACTUATORS for name in ("position", "velocity", "effort")):
            raise ARXBetaSafetyError(
                "X5-2023 gripper limiting requires seven-value position, "
                "velocity, and effort0 feedback vectors."
            )
        extra = state["additional_sdk_feedback"]
        position = float(extra["position_raw"])
        velocity = float(extra["velocity_raw"])
        effort0 = float(extra["effort0"])
        if not all(math.isfinite(value) for value in (position, velocity, effort0)):
            raise ARXBetaSafetyError("X5-2023 gripper raw feedback contains NaN or Inf.")
        return {
            "position_raw": position,
            "velocity_raw": velocity,
            "effort0": effort0,
            "catch_status": state["gripper"]["catch_status"],
            "semantics": (
                "experimental X5-2023 seventh-vector feedback; effort0 is "
                "uncalibrated and is not amperes or newtons"
            ),
        }

    def set_gripper_raw_position(self, position: float) -> None:
        """Set only the X5-2023 gripper through low-level ``set_catch``."""
        self._raise_if_faulted()
        target = float(position)
        if not math.isfinite(target):
            raise ValueError("Gripper raw target must be finite.")
        if not X5_2023_GRIPPER_RAW_MIN <= target <= X5_2023_GRIPPER_RAW_MAX:
            raise ValueError(
                f"Gripper raw target {target} is outside "
                f"[{X5_2023_GRIPPER_RAW_MIN}, {X5_2023_GRIPPER_RAW_MAX}]."
            )
        raw = getattr(self.arm, "arm", None)
        setter = getattr(raw, "set_catch", None)
        if not callable(setter):
            raise ARXBetaSafetyError(
                "Vendor low-level gripper set_catch method is unavailable."
            )
        setter(target)

    def _read_once(self) -> Dict[str, List[float]]:
        self._raise_if_faulted()
        capabilities = self.capabilities()
        if not capabilities["continuous_gripper_feedback"]:
            raise ARXBetaSafetyError(
                "The installed X5-2023 SDK has no continuous gripper position, "
                "velocity, or current feedback. Full 7-actuator policy execution "
                "is blocked; use mcc-arx-diagnose for six-axis read-only checks."
            )

        positions, _, _ = _split_x5_feedback(
            self.arm.get_joint_positions(), label="joint positions"
        )
        velocities, _, _ = _split_x5_feedback(
            self.arm.get_joint_velocities(), label="joint velocities"
        )
        effort, _, _ = _split_x5_feedback(
            self.arm.get_joint_currents(), label="joint effort"
        )
        raw_gripper = float(self.arm.get_gripper_pos())
        raw_gripper_vel = float(self.arm.get_gripper_vel())
        gripper_effort = float(self.arm.get_gripper_current())
        extras = (raw_gripper, raw_gripper_vel, gripper_effort)
        if not all(math.isfinite(value) for value in extras):
            raise ValueError("Gripper state contains NaN or Inf.")

        gripper_model = self.gripper_mapping.raw_to_model(raw_gripper)
        gripper_velocity_model = raw_gripper_vel * (
            (self.gripper_mapping.model_max - self.gripper_mapping.model_min)
            / (self.gripper_mapping.raw_max - self.gripper_mapping.raw_min)
        )
        zeros = [0.0] * TOTAL_ACTUATORS
        return {
            "pos": [*positions, gripper_model],
            "vel": [*velocities, gripper_velocity_model],
            # The vendor API calls this method get_joint_currents while its
            # README describes torque feedback. Preserve the raw values until
            # a per-device effort calibration establishes the actual units.
            "cur": [*effort, gripper_effort],
            "pwm": zeros.copy(),
            "vin": zeros.copy(),
            "temp": zeros.copy(),
        }

    def get_state(self, retries: int = 0) -> Dict[str, List[float]]:
        attempts_left: int | None = None if retries < 0 else int(retries) + 1
        last_error: Exception | None = None
        while attempts_left is None or attempts_left > 0:
            try:
                return self._read_once()
            except (ValueError, ARXBetaSafetyError):
                raise
            except Exception as exc:  # pragma: no cover - hardware dependent
                last_error = exc
                if attempts_left is not None:
                    attempts_left -= 1
                if attempts_left == 0:
                    break
                time.sleep(0.01)
        raise RuntimeError("Failed to read vendor X5 state.") from last_error

    def set_pos(self, pos_vec: Sequence[float]) -> None:
        self._raise_if_faulted()
        target = _finite_float_list(
            pos_vec, expected=TOTAL_ACTUATORS, label="motor target"
        )
        raw_gripper = self.gripper_mapping.model_to_raw(target[-1])
        self.set_arm_pos(target[:ARM_DOF])
        self.arm.set_gripper_pos(raw_gripper)

    def set_arm_pos(self, pos_vec: Sequence[float]) -> None:
        """Send a six-axis target without issuing any gripper command."""
        self._raise_if_faulted()
        target = _finite_float_list(
            pos_vec, expected=ARM_DOF, label="arm target"
        )
        try:
            parameters = inspect.signature(
                self.arm.set_joint_positions
            ).parameters
        except (TypeError, ValueError):
            parameters = {}
        if "duration" in parameters:
            self.arm.set_joint_positions(
                positions=target[:ARM_DOF], duration=self.command_duration
            )
        else:
            # The bundled X5 SDK accepts only the target vector.  The 50 Hz
            # application loop and per-cycle step limits remain authoritative;
            # do not claim that this SDK supports a duration argument.
            self.arm.set_joint_positions(target)

    def enter_arm_position_mode(
        self,
        pos_vec: Sequence[float],
        gripper_raw: float | None = None,
    ) -> None:
        """Prime an exact target, then enter vendor POSITION_CONTROL once.

        If ``gripper_raw`` is set, write it with ``set_catch`` after the six
        arm targets and before ``set_arm_status(5)``. Status 5 otherwise keeps
        the vendor seventh-axis default, which on this gripper is raw 0.
        """
        self._raise_if_faulted()
        target = _finite_float_list(pos_vec, expected=ARM_DOF, label="arm target")
        raw = getattr(self.arm, "arm", None)
        if raw is None:
            raise ARXBetaSafetyError("Vendor low-level InterfacesPy is unavailable.")
        target_setter = getattr(raw, "set_joint_positions", None)
        status_setter = getattr(raw, "set_arm_status", None)
        if not callable(target_setter) or not callable(status_setter):
            raise ARXBetaSafetyError(
                "Vendor low-level position target/status methods are unavailable."
            )
        target_setter(target)
        if gripper_raw is not None:
            self.set_gripper_raw_position(gripper_raw)
        status_setter(5)

    def update_arm_position_target(self, pos_vec: Sequence[float]) -> None:
        """Update the low-level target without re-entering position mode."""
        self._raise_if_faulted()
        target = _finite_float_list(pos_vec, expected=ARM_DOF, label="arm target")
        raw = getattr(self.arm, "arm", None)
        target_setter = getattr(raw, "set_joint_positions", None)
        if not callable(target_setter):
            raise ARXBetaSafetyError(
                "Vendor low-level position target method is unavailable."
            )
        target_setter(target)

    def set_vel(self, vel_vec: Sequence[float]) -> None:
        del vel_vec
        raise NotImplementedError("Vendor X5 adapter does not expose velocity commands.")

    def set_pd(self, kp_vec: Sequence[float], kd_vec: Sequence[float]) -> None:
        del kp_vec, kd_vec
        raise NotImplementedError("Vendor X5 adapter does not expose PD gain updates.")

    def disable_motors(self) -> None:
        self.arm.protect_mode()

    def close_motors(self) -> None:
        close_method = getattr(self.arm, "close", None)
        if callable(close_method):
            close_method()


def create_controllers(
    port_pattern: str,
    kp: Sequence[float],
    kd: Sequence[float],
    ki: Sequence[float],
    zero_pos: Sequence[float],
    control_mode: Sequence[str],
    baudrate: int,
    return_delay: int,
    *,
    sdk_path: str | None = None,
    gripper_mapping: GripperMapping | None = None,
    command_duration: float = 0.02,
    robot_type: int = X5_2023_TYPE,
) -> List[ARXBetaControl]:
    """Create X5-2023 controllers without issuing a motion command."""
    del kp, kd, ki, zero_pos, control_mode, baudrate, return_delay
    if int(robot_type) != X5_2023_TYPE:
        raise ValueError(
            f"This adapter only supports X5-2023 type {X5_2023_TYPE}, "
            f"got {robot_type}."
        )
    module = load_bimanual_sdk(sdk_path=sdk_path)
    interfaces = _discover_interfaces(port_pattern)
    controllers: List[ARXBetaControl] = []
    for interface in interfaces:
        arm = module.SingleArm({"can_port": interface, "type": int(robot_type)})
        controllers.append(
            ARXBetaControl(
                interface=interface,
                arm=arm,
                gripper_mapping=gripper_mapping,
                command_duration=command_duration,
            )
        )
    return controllers


def initialize(controllers: Iterable[ARXBetaControl]) -> None:
    for controller in controllers:
        controller.initialize_motors()


def get_motor_ids(
    controllers: Sequence[ARXBetaControl],
) -> Dict[str, List[int]]:
    return {
        f"controller_{index}": controller.get_motor_ids()
        for index, controller in enumerate(controllers)
    }


def get_motor_states(
    controllers: Sequence[ARXBetaControl], retries: int = 0
) -> Dict[str, Dict[str, List[float]]]:
    return {
        f"controller_{index}": controller.get_state(retries=retries)
        for index, controller in enumerate(controllers)
    }


def get_diagnostic_states(
    controllers: Sequence[ARXBetaControl],
) -> Dict[str, Dict[str, Any]]:
    return {
        f"controller_{index}": controller.get_diagnostic_state()
        for index, controller in enumerate(controllers)
    }


def validate_policy_capabilities(
    controllers: Sequence[ARXBetaControl],
) -> None:
    missing = [
        controller.interface
        for controller in controllers
        if not controller.capabilities()["continuous_gripper_feedback"]
    ]
    if missing:
        raise ARXBetaSafetyError(
            "Full X5-2023 policy requires continuous gripper feedback, which is "
            "not provided by the installed SDK on: " + ", ".join(missing)
        )


def set_motor_pos(
    controllers: Sequence[ARXBetaControl], pos_vecs: Sequence[Sequence[float]]
) -> None:
    if len(controllers) != len(pos_vecs):
        raise ValueError("Controller and position vector counts must match.")
    for controller, positions in zip(controllers, pos_vecs):
        controller.set_pos(positions)


def set_motor_vel(
    controllers: Sequence[ARXBetaControl], vel_vecs: Sequence[Sequence[float]]
) -> None:
    if len(controllers) != len(vel_vecs):
        raise ValueError("Controller and velocity vector counts must match.")
    for controller, velocities in zip(controllers, vel_vecs):
        controller.set_vel(velocities)


def set_motor_pd(
    controllers: Sequence[ARXBetaControl],
    kp_vecs: Sequence[Sequence[float]],
    kd_vecs: Sequence[Sequence[float]],
) -> None:
    if len(controllers) != len(kp_vecs) or len(controllers) != len(kd_vecs):
        raise ValueError("Controller and gain vector counts must match.")
    for controller, kp_vec, kd_vec in zip(controllers, kp_vecs, kd_vecs):
        controller.set_pd(kp_vec, kd_vec)


def disable_motors(controllers: Sequence[ARXBetaControl]) -> None:
    for controller in controllers:
        controller.disable_motors()


def close(controllers: Sequence[ARXBetaControl]) -> None:
    for controller in controllers:
        try:
            controller.disable_motors()
        finally:
            controller.close_motors()
