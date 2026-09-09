"""最小 Python 调用示例。运行前请先完成 README 中的实机检查。"""

from pathlib import Path

from x5_gripper_sdk import GripperConfig, X5Gripper


config = GripperConfig(
    interface="can2",
    motor_can_id=8,
    feedback_can_id=None,
    kp=5.0,
    kd=0.2,
    calibration_path=Path("./gripper_X5-2023-001.json"),
    device_serial="X5-2023-001",
    acknowledge_unverified_hardware=True,
)

with X5Gripper(config) as gripper:
    hold = gripper.hold(0.5)
    print("当前状态：", hold.final_state)

    half_open = gripper.move_to_opening(0.5, speed_rad_s=0.15)
    print("线性移动后开度：", half_open.final_state.opening)

    opened = gripper.open_linearly(speed_rad_s=0.15)
    print("全开后开度：", opened.final_state.opening)

    # 原相对接口继续保留，用于小距离、低力矩微动。
    nudged = gripper.close_relative(0.03, torque_nm=0.25)
    print("闭合微动后开度：", nudged.final_state.opening)
