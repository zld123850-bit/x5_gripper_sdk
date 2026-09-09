"""X5 外置夹爪实机交互测试。

运行前必须固定机械臂、清空夹爪、停止其他 can2 控制程序并准备好急停。
"""

from __future__ import annotations

import argparse
from pathlib import Path

from x5_gripper_sdk import GripperConfig, GripperError, GripperSafetyError, X5Gripper


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="X5 夹爪实机交互测试")
    parser.add_argument("--interface", default="can2")
    parser.add_argument("--motor-can-id", type=lambda value: int(value, 0), default=8)
    parser.add_argument("--feedback-can-id", type=lambda value: int(value, 0), default=None)
    parser.add_argument("--kp", type=float, required=True)
    parser.add_argument("--kd", type=float, required=True)
    parser.add_argument("--device-serial", default="X5-2023-001")
    parser.add_argument("--calibration", type=Path, default=None)
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    parser.add_argument("--close-step-rad", type=float, default=0.03)
    parser.add_argument("--close-torque-nm", type=float, default=0.25)
    parser.add_argument("--open-step-rad", type=float, default=0.03)
    parser.add_argument("--open-torque-nm", type=float, default=-0.25)
    return parser


def print_result(result: object) -> None:
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


def main() -> None:
    args = build_parser().parse_args()
    print("警告：该脚本会实际驱动夹爪。")
    print("请确认机械臂已固定、夹爪内无物体、其他 can2 控制程序已停止、急停可用。")
    if input("确认后输入大写 RUN：").strip() != "RUN":
        raise SystemExit("未确认，测试取消。")

    config = GripperConfig(
        interface=args.interface,
        motor_can_id=args.motor_can_id,
        feedback_can_id=args.feedback_can_id,
        kp=args.kp,
        kd=args.kd,
        device_serial=args.device_serial,
        calibration_path=args.calibration,
        acknowledge_unverified_hardware=True,
    )

    try:
        with X5Gripper(config) as gripper:
            print("\n每次输入一个字母并按 Enter：h=保持，c=小步闭合，o=小步张开，s=失能，q=退出")
            while True:
                command = input("命令（单个字母 + Enter）> ").strip().lower()
                if len(command) != 1:
                    print("每次只能输入一个命令字母，然后按 Enter。")
                    continue
                if command == "q":
                    break
                if command == "s":
                    gripper.emergency_stop()
                    print("已发送 DISABLE。")
                    continue
                if command == "h":
                    print_result(gripper.hold(args.hold_seconds))
                    continue
                if command == "c":
                    print_result(
                        gripper.close_relative(
                            args.close_step_rad,
                            torque_nm=args.close_torque_nm,
                        )
                    )
                    continue
                if command == "o":
                    print_result(
                        gripper.open_relative(
                            args.open_step_rad,
                            torque_nm=args.open_torque_nm,
                        )
                    )
                    continue
                print("未知命令；请输入 h、c、o、s 或 q。")
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，已退出并发送 DISABLE。")
    except (OSError, ValueError, GripperError) as exc:
        raise SystemExit(f"测试中止，夹爪已失能：{exc}") from exc


if __name__ == "__main__":
    main()
