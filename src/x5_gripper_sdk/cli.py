"""x5-gripper 命令行入口。"""

from __future__ import annotations

import argparse
import json
import os
import select
import sys
import termios
import tty
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
from typing import Iterator

from .calibration import GripperCalibration, save_calibration
from .driver import X5Gripper
from .models import GripperConfig, GripperError, GripperSafetyError


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="X5-2023 外置夹爪独立 CAN SDK")
    parser.add_argument("--interface", default="can2")
    parser.add_argument("--motor-can-id", type=lambda value: int(value, 0), default=8)
    parser.add_argument("--feedback-can-id", type=lambda value: int(value, 0), default=None)
    parser.add_argument("--kp", type=float, default=5.0)
    parser.add_argument("--kd", type=float, default=0.2)
    parser.add_argument("--device-serial", default="X5-2023-001")
    parser.add_argument("--calibration", type=Path, default=None)
    parser.add_argument("--acknowledge-unverified-hardware", action="store_true")
    subparsers = parser.add_subparsers(dest="command", required=True)

    hold = subparsers.add_parser("hold", help="保持当前位置")
    hold.add_argument("--seconds", type=float, default=0.5)

    move = subparsers.add_parser("move", help="线性运动到标定归一化开度")
    move.add_argument("--opening", type=float, required=True, help="0=全闭，1=全开")
    move.add_argument("--speed-rad-s", type=float, default=0.15)
    move.add_argument("--duration-after-s", type=float, default=0.2)

    linear_open = subparsers.add_parser("open-linear", help="线性运动到标定张开端")
    linear_open.add_argument("--speed-rad-s", type=float, default=0.15)
    linear_open.add_argument("--duration-after-s", type=float, default=0.2)

    linear_close = subparsers.add_parser("close-linear", help="线性运动到标定闭合端")
    linear_close.add_argument("--speed-rad-s", type=float, default=0.15)
    linear_close.add_argument("--duration-after-s", type=float, default=0.2)

    close = subparsers.add_parser("close", help="相对闭合微动（兼容接口）")
    close.add_argument("--distance-rad", type=float, required=True)
    close.add_argument("--speed-rad-s", type=float, default=0.15)
    close.add_argument("--torque-nm", type=float, default=None)

    opened = subparsers.add_parser("open", help="相对张开微动（兼容接口）")
    opened.add_argument("--distance-rad", type=float, required=True)
    opened.add_argument("--torque-nm", type=float, default=-0.10)

    calibrate = subparsers.add_parser("calibrate", help="人工采集闭合/张开端点")
    calibrate.add_argument("--output", type=Path, required=True)

    interactive = subparsers.add_parser("interactive", help="交互式保持和开合实机测试")
    interactive.add_argument("--hold-seconds", type=float, default=1.0)
    interactive.add_argument("--close-step-rad", type=float, default=0.03)
    interactive.add_argument("--close-torque-nm", type=float, default=0.25)
    interactive.add_argument("--open-step-rad", type=float, default=0.03)
    interactive.add_argument("--open-torque-nm", type=float, default=-0.25)

    live = subparsers.add_parser("live", help="无需 Enter 的持续按键与双端点标定")
    live.add_argument("--calibration-output", type=Path, required=True)
    live.add_argument("--step-rad", type=float, default=0.02)
    live.add_argument("--close-torque-nm", type=float, default=0.25)
    live.add_argument("--open-torque-nm", type=float, default=-0.25)
    live.add_argument("--hold-seconds", type=float, default=1.0)
    return parser


def _print_motion_result(result: object) -> None:
    state = result.final_state  # type: ignore[attr-defined]
    opening = "未标定" if state.opening is None else f"{state.opening:.1%}"
    print(
        f"完成：start={result.start_position_rad:.4f} rad，"  # type: ignore[attr-defined]
        f"target={result.target_position_rad:.4f} rad，"  # type: ignore[attr-defined]
        f"final={state.position_rad:.4f} rad，"
        f"velocity={state.velocity_rad_s:.4f} rad/s，"
        f"torque={state.torque_nm:.4f} Nm，开度={opening}，"
        f"到达目标={result.target_reached}"  # type: ignore[attr-defined]
    )
    if not result.target_reached:  # type: ignore[attr-defined]
        raise GripperSafetyError(
            "动作未到达目标，可能已到机械端点、受到阻挡或驱动力不足；"
            "禁止继续重复同方向命令，请检查实物。"
        )


def _run_interactive(gripper: X5Gripper, args: argparse.Namespace) -> None:
    print("警告：该模式会实际驱动夹爪。")
    print("请确认机械臂已固定、夹爪内无物体、其他 can2 控制程序已停止、急停可用。")
    if input("确认后输入大写 RUN：").strip() != "RUN":
        raise GripperError("未确认，测试取消。")
    print("\n每次输入一个字母并按 Enter：h=保持，c=小步闭合，o=小步张开，s=失能，q=退出")
    while True:
        command = input("命令（单个字母 + Enter）> ").strip().lower()
        if len(command) != 1:
            print("每次只能输入一个命令字母，然后按 Enter。")
            continue
        if command == "q":
            return
        if command == "s":
            gripper.emergency_stop()
            print("已发送 DISABLE。")
        elif command == "h":
            _print_motion_result(gripper.hold(args.hold_seconds))
        elif command == "c":
            _print_motion_result(
                gripper.close_relative(
                    args.close_step_rad,
                    torque_nm=args.close_torque_nm,
                )
            )
        elif command == "o":
            _print_motion_result(
                gripper.open_relative(
                    args.open_step_rad,
                    torque_nm=args.open_torque_nm,
                )
            )
        else:
            print("未知命令；请输入 h、c、o、s 或 q。")


@contextmanager
def _immediate_keys() -> Iterator[int]:
    descriptor = sys.stdin.fileno()
    if not os.isatty(descriptor):
        raise GripperError("live 模式必须在交互式终端中运行。")
    previous = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        yield descriptor
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)


def _read_immediate_key(descriptor: int) -> str | None:
    readable, _, _ = select.select([descriptor], [], [], 0.10)
    if not readable:
        return None
    raw = os.read(descriptor, 1)
    return raw.decode(errors="ignore") if raw else None


def _drain_immediate_keys(descriptor: int) -> None:
    while select.select([descriptor], [], [], 0.0)[0]:
        if not os.read(descriptor, 1):
            return


def _run_live(gripper: X5Gripper, args: argparse.Namespace) -> None:
    output = args.calibration_output.expanduser().resolve()
    closed_state = None
    blocked_direction = None
    print(
        "\n按键无需 Enter：按住 c=闭合，按住 o=张开，空格=失能，h=保持，"
        "z=采集闭合端，m=采集张开端并保存标定，q/Esc=退出。\n"
        "标定顺序：移动到闭合端后按 z，再移动到最大安全张开端后按 m。"
    )
    with _immediate_keys() as descriptor:
        while True:
            key = _read_immediate_key(descriptor)
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
            try:
                if key == "h":
                    _print_motion_result(gripper.hold(args.hold_seconds))
                elif key == "c":
                    if blocked_direction == "c":
                        continue
                    result = gripper.close_relative(
                        args.step_rad,
                        torque_nm=args.close_torque_nm,
                    )
                    print(
                        f"\r\n闭合：start={result.start_position_rad:.4f}，"
                        f"target={result.target_position_rad:.4f}，"
                        f"final={result.final_state.position_rad:.4f}，"
                        f"reached={result.target_reached}"
                    )
                    blocked_direction = None if result.target_reached else "c"
                    if blocked_direction:
                        print("闭合未到目标，已锁定 c；检查端点/阻挡，按 o 解除。")
                elif key == "o":
                    if blocked_direction == "o":
                        continue
                    result = gripper.open_relative(
                        args.step_rad,
                        torque_nm=args.open_torque_nm,
                    )
                    print(
                        f"\r\n张开：start={result.start_position_rad:.4f}，"
                        f"target={result.target_position_rad:.4f}，"
                        f"final={result.final_state.position_rad:.4f}，"
                        f"reached={result.target_reached}"
                    )
                    blocked_direction = None if result.target_reached else "o"
                    if blocked_direction:
                        print("张开未到目标，已锁定 o；检查端点/阻挡，按 c 解除。")
                elif key == "z":
                    closed_state = gripper.capture_stationary_position()
                    blocked_direction = "c"
                    print(f"\r\n闭合端已采集：{closed_state.position_rad:.4f} rad")
                elif key == "m":
                    if closed_state is None:
                        raise GripperSafetyError("请先到完全闭合端并按 z。")
                    opened_state = gripper.capture_stationary_position()
                    if opened_state.position_rad >= closed_state.position_rad:
                        raise GripperSafetyError(
                            "张开端编码器值未小于闭合端，方向不匹配，未保存。"
                        )
                    calibration = GripperCalibration(
                        device_serial=gripper.config.device_serial,
                        interface=gripper.config.interface,
                        motor_can_id=gripper.config.motor_can_id,
                        feedback_can_id=gripper.config.feedback_can_id,
                        closed_position_rad=closed_state.position_rad,
                        open_position_rad=opened_state.position_rad,
                        calibrated_at=datetime.now().astimezone().isoformat(
                            timespec="milliseconds"
                        ),
                    )
                    destination = save_calibration(output, calibration)
                    gripper.reload_calibration(destination)
                    blocked_direction = "o"
                    print(
                        f"\r\n张开端已采集：{opened_state.position_rad:.4f} rad；"
                        f"标定已保存并加载：{destination}"
                    )
                else:
                    continue
            except (ValueError, GripperError) as exc:
                gripper.emergency_stop()
                print(f"\r\n动作被拒绝并已失能：{exc}")
            finally:
                _drain_immediate_keys(descriptor)
            if key == "c" and blocked_direction == "o":
                blocked_direction = None
            elif key == "o" and blocked_direction == "c":
                blocked_direction = None


def main(argv: list[str] | None = None) -> None:
    args = _parser().parse_args(argv)
    calibration_path = args.calibration
    if args.command == "live":
        live_output = args.calibration_output.expanduser().resolve()
        if live_output.exists():
            calibration_path = live_output
    config = GripperConfig(
        interface=args.interface,
        motor_can_id=args.motor_can_id,
        feedback_can_id=args.feedback_can_id,
        kp=args.kp,
        kd=args.kd,
        device_serial=args.device_serial,
        calibration_path=calibration_path,
        acknowledge_unverified_hardware=args.acknowledge_unverified_hardware,
    )
    try:
        with X5Gripper(config) as gripper:
            if args.command == "hold":
                result = gripper.hold(args.seconds)
            elif args.command == "move":
                result = gripper.move_to_opening(
                    args.opening,
                    speed_rad_s=args.speed_rad_s,
                    duration_after_s=args.duration_after_s,
                )
            elif args.command == "open-linear":
                result = gripper.open_linearly(
                    speed_rad_s=args.speed_rad_s,
                    duration_after_s=args.duration_after_s,
                )
            elif args.command == "close-linear":
                result = gripper.close_linearly(
                    speed_rad_s=args.speed_rad_s,
                    duration_after_s=args.duration_after_s,
                )
            elif args.command == "close":
                result = gripper.close_relative(
                    args.distance_rad,
                    speed_rad_s=args.speed_rad_s,
                    torque_nm=args.torque_nm,
                )
            elif args.command == "open":
                result = gripper.open_relative(args.distance_rad, torque_nm=args.torque_nm)
            elif args.command == "interactive":
                _run_interactive(gripper, args)
                return
            elif args.command == "live":
                print("警告：live 模式会直接驱动真实夹爪。")
                print("请固定机械臂、清空夹爪、停止其他 can2 写入程序并准备急停。")
                if input("确认后输入大写 RUN：").strip() != "RUN":
                    raise GripperError("未确认，测试取消。")
                _run_live(gripper, args)
                return
            else:
                print("确保夹爪内无物体、机械臂固定且急停可用。")
                confirmation = input("手动将夹爪移到完全闭合端，确认静止后输入 CLOSED：")
                if confirmation != "CLOSED":
                    raise GripperError("未确认闭合端，标定已取消。")
                closed = gripper.capture_stationary_position()
                confirmation = input("手动将夹爪移到最大安全张开端，确认静止后输入 OPEN：")
                if confirmation != "OPEN":
                    raise GripperError("未确认张开端，标定已取消。")
                opened = gripper.capture_stationary_position()
                if opened.position_rad >= closed.position_rad:
                    raise GripperError(
                        "当前 SDK 只允许已验证的负编码器张开方向；采集结果方向不匹配。"
                    )
                calibration = GripperCalibration(
                    device_serial=args.device_serial,
                    interface=args.interface,
                    motor_can_id=args.motor_can_id,
                    feedback_can_id=args.feedback_can_id,
                    closed_position_rad=closed.position_rad,
                    open_position_rad=opened.position_rad,
                    calibrated_at=datetime.now().astimezone().isoformat(timespec="milliseconds"),
                )
                path = save_calibration(args.output, calibration)
                print(f"标定已保存：{path}")
                return
        print(json.dumps(result.as_dict(), ensure_ascii=False, indent=2))
    except KeyboardInterrupt:
        raise SystemExit("\n收到 Ctrl+C，夹爪已失能。") from None
    except (OSError, ValueError, GripperError) as exc:
        raise SystemExit(f"错误：{exc}") from exc


if __name__ == "__main__":
    main()
