"""已标定夹爪控制，以及可选的 X5 机械臂笛卡尔按键控制。"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import tempfile
import time
import tty
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Iterator, TextIO

# 允许从源码目录直接运行本示例，不依赖 editable 安装或 PYTHONPATH。
SDK_SRC = Path(__file__).resolve().parents[1] / "src"
if SDK_SRC.is_dir():
    sys.path.insert(0, str(SDK_SRC))

import x5_gripper_sdk as x5_sdk_package
from x5_gripper_sdk import (
    GripperConfig,
    GripperError,
    MotionResult,
    X5Gripper,
    load_calibration,
)


INITIAL_KEY_REPEAT_GRACE_S = 0.65
KEY_RELEASE_TIMEOUT_S = 0.18
ARM_MOVE_KEYS = frozenset("wsadrf")


def run_arm_keyboard_control(
    arm: Any,
    *,
    initial_target_raw: float,
    open_target_raw: float,
    close_target_raw: float,
    open_rate_raw_s: float,
    close_rate_raw_s: float,
    refresh_hz: float,
    max_seconds: float,
    log_stream: TextIO,
    poll_key: Callable[[], str | None],
    calibration_path: Path | None = None,
    physical_origin_raw: float | None = None,
    command_feedback_offset_raw: float = 0.0,
    runtime_torque_slope_nm_per_raw: float | None = None,
    travel_calibration: Any = None,
    safety_check: Callable[[], None] | None = None,
    gripper_key_layout: str = "c-close",
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, Any]:
    """本示例自己的机械臂/夹爪按键循环。

    初始化、ESC 8 交接和 CAN 安全监控由 SDK 运行层完成；这里直接解释
    W/S/A/D/R/F/H 等机械臂按键并执行对应动作。
    """
    from x5_gripper_sdk.integrated import combined_control as control

    session = control.VendorCatchGripperSession(
        arm.controller,
        initial_target_raw=initial_target_raw,
        open_target_raw=open_target_raw,
        close_target_raw=close_target_raw,
        open_rate_raw_s=open_rate_raw_s,
        close_rate_raw_s=close_rate_raw_s,
        refresh_hz=refresh_hz,
        max_arm_velocity=arm.max_velocity,
        log_stream=log_stream,
        physical_origin_raw=physical_origin_raw,
        travel_calibration=travel_calibration,
        command_feedback_offset_raw=command_feedback_offset_raw,
        runtime_torque_slope_nm_per_raw=runtime_torque_slope_nm_per_raw,
        monotonic=monotonic,
        sleep=sleep,
    )
    control._user_print(
        "按键：W/S=X+/X-，A/D=Y+/Y-，R/F=Z+/Z-；"
        "H=回到启动位置；按住 c=闭合，按住 o=张开；"
        "]/[=闭合/张开微动；p=状态；q/Esc=保护退出。"
    )
    try:
        while monotonic() - session.started_at < float(max_seconds):
            if safety_check is not None:
                safety_check()
            key = poll_key()
            if key is not None:
                lowered = key.lower()
                if key in control.QUIT_KEYS:
                    break
                if lowered == "p":
                    state = session.observe()
                    control._print_state(
                        state, control._fk_pose(state, arm.fk_solver)
                    )
                    control._print_gripper(session)
                elif lowered in ARM_MOVE_KEYS or lowered == "h":
                    if control._run_cartesian_command(
                        session,
                        arm,
                        key=lowered,
                        log_stream=log_stream,
                    ):
                        break
                elif session.apply_key(
                    control.map_gripper_key(key, gripper_key_layout),
                    monotonic(),
                ):
                    break
            session.pump(monotonic(), wait=True)
            if safety_check is not None:
                safety_check()
        result = session.result()
        result["calibration_path"] = (
            None
            if calibration_path is None
            else str(calibration_path.resolve())
        )
        result["calibration_saved"] = False
        control._write_log(log_stream, "keyboard_control_passed", **result)
        return result
    finally:
        try:
            session.observe(force_log=True)
        except Exception as exc:
            control._write_log(
                log_stream,
                "keyboard_final_snapshot_failed",
                error=repr(exc),
            )
        try:
            control.backend.disable_motors(arm.controllers)
            control._write_log(log_stream, "protection_requested")
        except Exception as exc:
            control._write_log(
                log_stream,
                "protection_failed",
                error=repr(exc),
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="X5 已标定线性夹爪控制")
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--interface", default="can2")
    parser.add_argument("--motor-can-id", type=lambda value: int(value, 0), default=8)
    parser.add_argument("--feedback-can-id", type=lambda value: int(value, 0), default=None)
    parser.add_argument("--device-serial", default="X5-2023-001")
    parser.add_argument("--kp", type=float, default=5.0)
    parser.add_argument("--kd", type=float, default=0.2)
    parser.add_argument("--linear-speed-rad-s", type=float, default=0.25)
    parser.add_argument("--nudge-step-rad", type=float, default=0.02)
    parser.add_argument("--nudge-torque-nm", type=float, default=0.25)
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    parser.add_argument(
        "--enable-arm",
        action="store_true",
        help="启用同一 CAN 接口上的 X5 机械臂笛卡尔按键控制",
    )
    parser.add_argument("--arm-sdk-path", type=Path, default=None)
    parser.add_argument("--arm-motion-timeout", type=float, default=15.0)
    parser.add_argument("--arm-max-velocity", type=float, default=0.5)
    parser.add_argument("--arm-session-seconds", type=float, default=120.0)
    parser.add_argument(
        "--startup-speed-rad-s",
        type=float,
        default=0.35,
        help="厂家初始化前，夹爪移动到标定参考端点的目标速度",
    )
    parser.add_argument(
        "--startup-timeout-s",
        type=float,
        default=90.0,
        help="厂家初始化前夹爪参考端点运动的总超时",
    )
    return parser


def run_arm_and_gripper(args: argparse.Namespace, calibration_path: Path) -> None:
    """Use the SDK's guarded combined controller on the shared CAN bus.

    Vendor status 5 owns all seven actuator slots, so the direct-ID-8 gripper
    loop and ``SingleArm`` must not run concurrently. The combined controller
    suppresses ESC 8 during vendor Init, validates the handoff, and then uses
    the vendor Catch loop for the gripper while retaining independent CAN
    torque/velocity monitoring.
    """
    if args.interface != "can2" or args.motor_can_id != 8:
        raise GripperError("机械臂联合模式目前只允许 interface=can2、motor_can_id=8。")
    if args.feedback_can_id not in (None, 0):
        raise GripperError("机械臂联合模式的反馈 CAN ID 只允许留空或设为 0。")

    from x5_gripper_sdk.integrated import combined_control as control

    calibration = load_calibration(
        calibration_path,
        device_serial=args.device_serial,
        interface=args.interface,
        motor_can_id=args.motor_can_id,
    )
    sdk_path = (
        args.arm_sdk_path.expanduser().resolve()
        if args.arm_sdk_path is not None
        else Path(x5_sdk_package.__file__).resolve().parent
        / "vendor_runtime"
        / "arx_x5_python"
    )
    if not sdk_path.is_dir():
        raise GripperError(f"找不到厂家机械臂 SDK：{sdk_path}")

    # 两个控制器的端点都是 DM-J4310 的绝对电机坐标，但 JSON 字段名不同。
    # 只在临时目录生成兼容文件，不修改用户的原始标定文件。
    with tempfile.TemporaryDirectory(prefix="x5-arm-gripper-") as directory:
        bridge_path = Path(directory) / "gripper-calibration.json"
        control.save_gripper_calibration(
            bridge_path,
            control.GripperTravelCalibration(
                interface=calibration.interface,
                closed_zero_physical_raw=calibration.closed_position_rad,
                open_max_physical_raw=calibration.open_position_rad,
                calibrated_at=calibration.calibrated_at,
            ),
        )
        control.run_integrated_session(
            [
                "--interface",
                args.interface,
                "--sdk-path",
                str(sdk_path),
                "--gripper-calibration",
                str(bridge_path),
                "--acknowledge-startup-esc-disable",
                "--acknowledge-limited-feedback",
                "--acknowledge-transition-grace-risk",
                "--gripper-key-layout",
                "c-close",
                "--read-only-gripper-calibration",
                "--open-rate-raw-s",
                str(args.linear_speed_rad_s),
                "--close-rate-raw-s",
                str(args.linear_speed_rad_s),
                "--motion-timeout",
                str(args.arm_motion_timeout),
                "--max-velocity",
                str(args.arm_max_velocity),
                "--max-seconds",
                str(args.arm_session_seconds),
                "--startup-speed-rad-s",
                str(args.startup_speed_rad_s),
                "--startup-timeout-s",
                str(args.startup_timeout_s),
            ],
            jog_runner=run_arm_keyboard_control,
            enable_control=True,
        )


@contextmanager
def immediate_keys() -> Iterator[int]:
    descriptor = sys.stdin.fileno()
    if not os.isatty(descriptor):
        raise GripperError("本控制器必须在交互式终端中运行。")
    previous = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        yield descriptor
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)


def read_key(descriptor: int) -> str | None:
    readable, _, _ = select.select([descriptor], [], [], 0.10)
    if not readable:
        return None
    raw = os.read(descriptor, 1)
    return raw.decode(errors="ignore") if raw else None


def drain_keys(descriptor: int) -> None:
    while select.select([descriptor], [], [], 0.0)[0]:
        if not os.read(descriptor, 1):
            return


def print_result(result: MotionResult, *, allow_requested_stop: bool = False) -> None:
    state = result.final_state
    opening = "未知" if state.opening is None else f"{state.opening:.1%}"
    print(
        "\r\n"
        f"{result.operation}：起点={result.start_position_rad:.4f}，"
        f"目标={result.target_position_rad:.4f}，"
        f"终点={state.position_rad:.4f} rad，开度={opening}，"
        f"速度={state.velocity_rad_s:.4f} rad/s，"
        f"力矩={state.torque_nm:.4f} Nm，到达={result.target_reached}"
    )
    if result.stopped_by_request:
        print("松开按键，线性运动已停止并失能。")
    if not result.target_reached and not (
        allow_requested_stop and result.stopped_by_request
    ):
        raise GripperError("线性动作未到达目标，已停止；请检查端点、阻挡和驱动力。")


class HeldKeyTracker:
    """根据终端键盘自动重复字符推断按键是否仍按住。"""

    def __init__(self, descriptor: int, active_key: str):
        self.descriptor = descriptor
        self.active_key = active_key
        self.started_at = time.monotonic()
        self.last_seen_at = self.started_at
        self.repeat_seen = False
        self.pending_key: str | None = None

    def continue_motion(self) -> bool:
        now = time.monotonic()
        while select.select([self.descriptor], [], [], 0.0)[0]:
            raw = os.read(self.descriptor, 1)
            if not raw:
                return False
            key = raw.decode(errors="ignore").lower()
            if key == self.active_key:
                self.repeat_seen = True
                self.last_seen_at = now
            else:
                self.pending_key = key
                return False
        if not self.repeat_seen:
            return now - self.started_at < INITIAL_KEY_REPEAT_GRACE_S
        return now - self.last_seen_at < KEY_RELEASE_TIMEOUT_S

    def wait_for_release_after_endpoint(self) -> None:
        """到达端点后吞掉当前键的重复字符，避免松手前再次启动。"""
        last_active = time.monotonic()
        while time.monotonic() - last_active < KEY_RELEASE_TIMEOUT_S:
            readable, _, _ = select.select(
                [self.descriptor],
                [],
                [],
                KEY_RELEASE_TIMEOUT_S,
            )
            if not readable:
                return
            raw = os.read(self.descriptor, 1)
            if not raw:
                return
            key = raw.decode(errors="ignore").lower()
            if key == self.active_key:
                last_active = time.monotonic()
            else:
                self.pending_key = key
                return


def print_help() -> None:
    print(
        "\r\n已标定线性控制（按键无需 Enter）：\r\n"
        "  按住 o  向全开方向线性运动，松手停止\r\n"
        "  按住 c  向全闭方向线性运动，松手停止\r\n"
        "  1..9    线性运动到 10%..90% 开度\r\n"
        "  0       线性运动到全闭端\r\n"
        "  [       -0.25 Nm 张开微动\r\n"
        "  ]       +0.25 Nm 闭合微动\r\n"
        "  p       读取当前位置和标定开度\r\n"
        "  h       保持当前位置\r\n"
        "  空格    发送 DISABLE（动作进行中请用 Ctrl+C 或物理急停）\r\n"
        "  ?       显示帮助\r\n"
        "  q/Esc   DISABLE 并退出"
    )


def run(gripper: X5Gripper, args: argparse.Namespace) -> None:
    print_help()
    with immediate_keys() as descriptor:
        pending_key: str | None = None
        while True:
            key = pending_key
            pending_key = None
            if key is None:
                key = read_key(descriptor)
            if key is None:
                continue
            key = key.lower()
            if key in {"q", "\x1b"}:
                return
            if key == " ":
                gripper.emergency_stop()
                print("\r\n已发送 DISABLE。")
                continue
            try:
                if key == "?":
                    print_help()
                elif key == "o":
                    tracker = HeldKeyTracker(descriptor, "o")
                    result = gripper.open_linearly_while(
                        tracker.continue_motion,
                        speed_rad_s=args.linear_speed_rad_s,
                    )
                    print_result(result, allow_requested_stop=True)
                    if result.target_reached:
                        tracker.wait_for_release_after_endpoint()
                    pending_key = tracker.pending_key
                elif key in {"c", "0"}:
                    if key == "0":
                        print_result(
                            gripper.close_linearly(
                                speed_rad_s=args.linear_speed_rad_s
                            )
                        )
                    else:
                        tracker = HeldKeyTracker(descriptor, "c")
                        result = gripper.close_linearly_while(
                            tracker.continue_motion,
                            speed_rad_s=args.linear_speed_rad_s,
                        )
                        print_result(result, allow_requested_stop=True)
                        if result.target_reached:
                            tracker.wait_for_release_after_endpoint()
                        pending_key = tracker.pending_key
                elif key in "123456789":
                    print_result(
                        gripper.move_to_opening(
                            int(key) / 10.0,
                            speed_rad_s=args.linear_speed_rad_s,
                        )
                    )
                elif key == "[":
                    print_result(
                        gripper.open_relative(
                            args.nudge_step_rad,
                            torque_nm=-abs(args.nudge_torque_nm),
                        )
                    )
                elif key == "]":
                    print_result(
                        gripper.close_relative(
                            args.nudge_step_rad,
                            torque_nm=abs(args.nudge_torque_nm),
                        )
                    )
                elif key == "p":
                    state = gripper.capture_stationary_position()
                    opening = "未知" if state.opening is None else f"{state.opening:.1%}"
                    print(
                        f"\r\n位置={state.position_rad:.4f} rad，"
                        f"速度={state.velocity_rad_s:.4f} rad/s，开度={opening}"
                    )
                elif key == "h":
                    print_result(gripper.hold(args.hold_seconds))
                else:
                    continue
            except (ValueError, GripperError) as exc:
                gripper.emergency_stop()
                print(f"\r\n动作中止并已失能：{exc}")
            finally:
                if key not in {"c", "o"}:
                    drain_keys(descriptor)


def main() -> None:
    args = build_parser().parse_args()
    calibration_path = args.calibration.expanduser().resolve()
    if args.enable_arm:
        print("警告：本程序将同时控制 can2 上的机械臂和外置夹爪。")
        print(f"标定文件：{calibration_path}")
        print(
            "机械臂按键：W/S=X+/X-，A/D=Y+/Y-，R/F=Z+/Z-，"
            "每次移动末端 1 cm；H 返回启动位置。"
        )
        print("夹爪按键：按住 c 闭合，按住 o 张开，松开后保持。")
        try:
            run_arm_and_gripper(args, calibration_path)
        except (ImportError, OSError, RuntimeError, ValueError, GripperError) as exc:
            raise SystemExit(f"机械臂/夹爪联合控制启动失败：{exc}") from exc
        return

    config = GripperConfig(
        interface=args.interface,
        motor_can_id=args.motor_can_id,
        feedback_can_id=args.feedback_can_id,
        kp=args.kp,
        kd=args.kd,
        calibration_path=calibration_path,
        device_serial=args.device_serial,
        acknowledge_unverified_hardware=True,
    )

    print("警告：本程序会按照已标定端点驱动真实夹爪。")
    print(f"标定文件：{calibration_path}")
    if input("固定机械臂、清空夹爪、停止其他 can2 程序后输入大写 RUN：").strip() != "RUN":
        raise SystemExit("未确认，启动取消。")
    try:
        with X5Gripper(config) as gripper:
            calibration = gripper.calibration
            assert calibration is not None
            print(
                f"已加载：闭合={calibration.closed_position_rad:.7f} rad，"
                f"张开={calibration.open_position_rad:.7f} rad，"
                f"行程={calibration.travel_rad:.7f} rad"
            )
            run(gripper, args)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，终端已恢复，夹爪已失能。")
    except (OSError, ValueError, GripperError) as exc:
        raise SystemExit(f"启动失败或动作中止，夹爪已失能：{exc}") from exc


if __name__ == "__main__":
    main()
