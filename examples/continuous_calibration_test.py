"""直接调用 x5_gripper_sdk 的持续按键与双端点标定实机测试。

按住 C/O 期间持续发送运动命令；松开后停止并失能。普通终端没有真实
KeyUp，因此用键盘自动重复推断按键是否仍按住。
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import tty
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
from typing import Iterator

# 允许从任意工作目录直接执行本文件，不依赖 editable 安装。
SDK_SRC = Path(__file__).resolve().parents[1] / "src"
if SDK_SRC.is_dir():
    sys.path.insert(0, str(SDK_SRC))

from x5_gripper_sdk import (
    GripperCalibration,
    GripperConfig,
    GripperError,
    GripperSafetyError,
    GripperState,
    MotionResult,
    X5Gripper,
    save_calibration,
)
from x5_gripper_sdk.held_key import HeldKeyTracker


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="X5 夹爪持续按键、双端点标定与开合实机测试"
    )
    parser.add_argument("--interface", default="can2")
    parser.add_argument("--motor-can-id", type=lambda value: int(value, 0), default=8)
    parser.add_argument(
        "--feedback-can-id",
        type=lambda value: int(value, 0),
        default=None,
    )
    parser.add_argument("--kp", type=float, default=5.0)
    parser.add_argument("--kd", type=float, default=0.2)
    parser.add_argument("--device-serial", default="X5-2023-001")
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path("./gripper_X5-2023-001.json"),
    )
    parser.add_argument(
        "--step-rad",
        type=float,
        default=0.50,
        help="按住一次连续运动的行程上限；闭合仍受 SDK 0.20 rad 限制",
    )
    parser.add_argument("--close-torque-nm", type=float, default=0.25)
    parser.add_argument("--open-torque-nm", type=float, default=-0.25)
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    parser.add_argument(
        "--ignore-existing-calibration",
        action="store_true",
        help="重新标定时不加载旧端点保护，但仍原子覆盖指定输出文件",
    )
    return parser


@contextmanager
def immediate_keys() -> Iterator[int]:
    """临时切换到单字符输入，并确保退出时恢复终端。"""
    descriptor = sys.stdin.fileno()
    if not os.isatty(descriptor):
        raise RuntimeError("持续按键测试必须在交互式终端中运行。")
    previous = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        yield descriptor
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)


def read_key(descriptor: int, timeout_s: float = 0.10) -> str | None:
    readable, _, _ = select.select([descriptor], [], [], timeout_s)
    if not readable:
        return None
    raw = os.read(descriptor, 1)
    if not raw:
        return None
    return raw.decode(errors="ignore")


def drain_queued_keys(descriptor: int) -> int:
    """丢弃动作执行期间积压的按键，防止松手后继续运动。"""
    drained = 0
    while True:
        readable, _, _ = select.select([descriptor], [], [], 0.0)
        if not readable:
            return drained
        if not os.read(descriptor, 1):
            return drained
        drained += 1


def state_text(state: GripperState) -> str:
    opening = "未标定" if state.opening is None else f"{state.opening:.1%}"
    return (
        f"位置={state.position_rad:.4f} rad，"
        f"速度={state.velocity_rad_s:.4f} rad/s，"
        f"力矩={state.torque_nm:.4f} Nm，开度={opening}"
    )


def print_motion(result: MotionResult) -> None:
    print(
        "\r\n"
        f"{result.operation}: 起点={result.start_position_rad:.4f}，"
        f"目标={result.target_position_rad:.4f}，"
        f"终点={result.final_state.position_rad:.4f} rad，"
        f"反馈力矩={result.final_state.torque_nm:.4f} Nm，"
        f"到达目标={result.target_reached}"
    )
    if result.stopped_by_request:
        print("松开按键，连续运动已停止并失能。")


def capture_closed(gripper: X5Gripper) -> GripperState:
    state = gripper.capture_stationary_position()
    print(f"\r\n已采集闭合端：{state_text(state)}")
    return state


def capture_open_and_save(
    gripper: X5Gripper,
    *,
    closed: GripperState | None,
    path: Path,
    config: GripperConfig,
) -> GripperCalibration:
    if closed is None:
        raise ValueError("尚未采集闭合端；请先移动到完全闭合位置并按 z。")
    opened = gripper.capture_stationary_position()
    if opened.position_rad >= closed.position_rad:
        raise GripperSafetyError(
            "张开端编码器值没有小于闭合端，与当前已验证方向不一致；未保存标定。"
        )
    calibration = GripperCalibration(
        device_serial=config.device_serial,
        interface=config.interface,
        motor_can_id=config.motor_can_id,
        feedback_can_id=config.feedback_can_id,
        closed_position_rad=closed.position_rad,
        open_position_rad=opened.position_rad,
        calibrated_at=datetime.now().astimezone().isoformat(timespec="milliseconds"),
    )
    destination = save_calibration(path, calibration)
    gripper.reload_calibration(destination)
    print(
        f"\r\n已采集张开端：{state_text(opened)}\r\n"
        f"标定已保存并立即加载：{destination}\r\n"
        f"标定行程={calibration.travel_rad:.4f} rad"
    )
    return calibration


def print_help() -> None:
    print(
        "\r\n按键无需 Enter：\r\n"
        "  按住 c  连续闭合（+0.25 Nm），松手停止\r\n"
        "  按住 o  连续张开（-0.25 Nm），松手停止\r\n"
        "  空格    立即 DISABLE\r\n"
        "  h       保持当前位置\r\n"
        "  z       采集完全闭合端\r\n"
        "  m       采集最大张开端、保存并加载标定\r\n"
        "  ?       重新显示帮助\r\n"
        "  q/Esc   DISABLE 并退出\r\n"
        "标定顺序：按住移动到闭合端 → 松键并按 z → 按住移动到张开端 → 松键并按 m。"
    )


def _run_held_relative(
    gripper: X5Gripper,
    args: argparse.Namespace,
    *,
    descriptor: int,
    key: str,
    blocked_direction: str | None,
) -> tuple[str | None, str | None]:
    """按住期间连续发送相对运动命令；返回 (pending_key, blocked_direction)。"""
    if blocked_direction == key:
        label = "闭合" if key == "c" else "张开"
        print(f"\r\n{label}已锁定；先检查端点/阻挡，按反向键解除。")
        return None, blocked_direction

    tracker = HeldKeyTracker(descriptor, key)
    if key == "c":
        distance = min(args.step_rad, gripper.config.max_close_nudge_rad)
        move = lambda: gripper.close_relative(
            distance,
            torque_nm=args.close_torque_nm,
            continue_motion=tracker.continue_motion,
            duration_after_s=0.05,
        )
        label = "闭合"
    else:
        distance = min(args.step_rad, gripper.config.max_open_nudge_rad)
        move = lambda: gripper.open_relative(
            distance,
            torque_nm=args.open_torque_nm,
            continue_motion=tracker.continue_motion,
            duration_after_s=0.05,
        )
        label = "张开"

    blocked = None

    while True:
        result = move()
        print_motion(result)
        if result.stopped_by_request:
            return tracker.pending_key, blocked
        if not result.target_reached:
            print(f"{label}未到目标；已锁定 {key}。先检查端点/阻挡，按反向键解除。")
            tracker.wait_for_release_after_endpoint()
            return tracker.pending_key, key
        if not tracker.continue_motion():
            return tracker.pending_key, blocked


def run(gripper: X5Gripper, args: argparse.Namespace) -> None:
    closed_state: GripperState | None = None
    blocked_direction: str | None = None
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
                blocked_direction = None
                print("\r\n已发送 DISABLE。")
                continue
            if key == "?":
                print_help()
                continue
            try:
                if key == "h":
                    result = gripper.hold(args.hold_seconds)
                    print_motion(result)
                elif key in {"c", "o"}:
                    pending_key, blocked_direction = _run_held_relative(
                        gripper,
                        args,
                        descriptor=descriptor,
                        key=key,
                        blocked_direction=blocked_direction,
                    )
                elif key == "z":
                    closed_state = capture_closed(gripper)
                    blocked_direction = "c"
                elif key == "m":
                    capture_open_and_save(
                        gripper,
                        closed=closed_state,
                        path=args.calibration,
                        config=gripper.config,
                    )
                    blocked_direction = "o"
                else:
                    continue
            except (ValueError, GripperError) as exc:
                gripper.emergency_stop()
                print(f"\r\n动作被拒绝并已失能：{exc}")
            finally:
                if key not in {"c", "o"}:
                    drain_queued_keys(descriptor)


def main() -> None:
    args = build_parser().parse_args()
    calibration_path = args.calibration.expanduser().resolve()
    existing_calibration = (
        None
        if args.ignore_existing_calibration
        else calibration_path if calibration_path.exists() else None
    )
    config = GripperConfig(
        interface=args.interface,
        motor_can_id=args.motor_can_id,
        feedback_can_id=args.feedback_can_id,
        kp=args.kp,
        kd=args.kd,
        device_serial=args.device_serial,
        calibration_path=existing_calibration,
        acknowledge_unverified_hardware=True,
    )

    print("警告：本脚本会直接调用 SDK 驱动真实夹爪。")
    print("请固定机械臂、清空夹爪、停止其他 can 写入程序并准备好急停。")
    print(
        f"按住时连续运动，单段上限={args.step_rad:.3f} rad"
        f"（闭合≤{config.max_close_nudge_rad:.2f}，张开≤{config.max_open_nudge_rad:.2f}），"
        f"闭合={args.close_torque_nm:+.2f} Nm，张开={args.open_torque_nm:+.2f} Nm"
    )
    print(f"标定文件：{calibration_path}（{'已加载' if existing_calibration else '尚不存在'}）")
    if input("确认以上条件后输入大写 RUN：").strip() != "RUN":
        raise SystemExit("未确认，测试取消。")

    try:
        with X5Gripper(config) as gripper:
            run(gripper, args)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，终端已恢复，夹爪已失能。")
    except (OSError, ValueError, GripperError) as exc:
        raise SystemExit(f"测试中止，夹爪已失能：{exc}") from exc


if __name__ == "__main__":
    main()
