#!/usr/bin/env bash
set -euo pipefail

SDK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# ===== 独立标定参数区：需要长期修改时只改这里 =====
GRIPPER_INTERFACE="${X5_GRIPPER_INTERFACE:-can2}"
MOTOR_CAN_ID="${X5_GRIPPER_MOTOR_CAN_ID:-8}"
FEEDBACK_CAN_ID="${X5_GRIPPER_FEEDBACK_CAN_ID:-}"
DEVICE_SERIAL="${X5_GRIPPER_DEVICE_SERIAL:-X5-2023-001}"
KP="${X5_GRIPPER_KP:-5.0}"
KD="${X5_GRIPPER_KD:-0.2}"
# 按住时单段连续行程上限；闭合仍受 SDK 0.20 rad 限制，张开受 0.50 rad 限制。
STEP_RAD="${X5_GRIPPER_CALIBRATION_STEP_RAD:-0.50}"
CLOSE_TORQUE_NM="${X5_GRIPPER_CALIBRATION_CLOSE_TORQUE_NM:-0.25}"
OPEN_TORQUE_NM="${X5_GRIPPER_CALIBRATION_OPEN_TORQUE_NM:--0.25}"
HOLD_SECONDS="${X5_GRIPPER_HOLD_SECONDS:-1.0}"
CALIBRATION_FILE="${X5_GRIPPER_CALIBRATION_FILE:-${SDK_DIR}/gripper-calibration.json}"
PYTHON_BIN="${X5_GRIPPER_PYTHON_BIN:-python}"
# ===== 参数区结束 =====

export PYTHONPATH="${SDK_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

echo "独立夹爪标定，不会初始化机械臂。"
echo "标定输出文件：${CALIBRATION_FILE}"

COMMAND=(
    "${PYTHON_BIN}"
    "${SDK_DIR}/examples/continuous_calibration_test.py"
    --interface "${GRIPPER_INTERFACE}"
    --motor-can-id "${MOTOR_CAN_ID}"
    --device-serial "${DEVICE_SERIAL}"
    --kp "${KP}"
    --kd "${KD}"
    --step-rad "${STEP_RAD}"
    --close-torque-nm "${CLOSE_TORQUE_NM}"
    --open-torque-nm "${OPEN_TORQUE_NM}"
    --hold-seconds "${HOLD_SECONDS}"
    --calibration "${CALIBRATION_FILE}"
    --ignore-existing-calibration
)

if [[ -n "${FEEDBACK_CAN_ID}" ]]; then
    COMMAND+=(--feedback-can-id "${FEEDBACK_CAN_ID}")
fi

exec "${COMMAND[@]}" "$@"
