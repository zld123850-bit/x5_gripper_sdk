#!/usr/bin/env bash
# 使用已标定参数启动 X5 夹爪；默认同时启用机械臂笛卡尔按键控制。
#
# 日常启动命令：
#   bash /home/lida/lida-ws/minimalist_compliance_control_X5_merged/x5_gripper_sdk/start-calibrated-gripper.sh
#
# 临时覆盖参数示例：
#   X5_GRIPPER_LINEAR_SPEED_RAD_S=0.30 bash start-calibrated-gripper.sh
#   bash start-calibrated-gripper.sh --linear-speed-rad-s 0.30
#
# 严格模式：命令失败、变量未定义或管道失败时立即退出。
set -euo pipefail

# 获取脚本所在目录的绝对路径，因此启动不受当前工作目录影响。
SDK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# ===== 常用参数区：需要长期修改时只改这里 =====
# SocketCAN 接口；机械臂联合模式目前只允许 can2。
GRIPPER_INTERFACE="${X5_GRIPPER_INTERFACE:-can2}"
# 外置夹爪电机 CAN ID；联合模式目前只允许 ID 8。
MOTOR_CAN_ID="${X5_GRIPPER_MOTOR_CAN_ID:-8}"
# 反馈帧 CAN ID；留空时按照反馈 payload 中的电机 ID 匹配。
FEEDBACK_CAN_ID="${X5_GRIPPER_FEEDBACK_CAN_ID:-}"
# 设备序列号，用于检查标定文件是否属于当前设备。
DEVICE_SERIAL="${X5_GRIPPER_DEVICE_SERIAL:-X5-2023-001}"
# 独立直连夹爪模式使用的比例增益和微分增益。
KP="${X5_GRIPPER_KP:-5.0}"
KD="${X5_GRIPPER_KD:-0.2}"
# 按住 c/o 时夹爪的线性目标速度，单位 rad/s。
LINEAR_SPEED_RAD_S="${X5_GRIPPER_LINEAR_SPEED_RAD_S:-0.25}"
# 厂家初始化前，夹爪自动移动到标定参考端点的目标速度，单位 rad/s。
STARTUP_SPEED_RAD_S="${X5_GRIPPER_STARTUP_SPEED_RAD_S:-0.35}"
# 上述初始化参考端点运动的总超时，单位 s。
STARTUP_TIMEOUT_SECONDS="${X5_GRIPPER_STARTUP_TIMEOUT_SECONDS:-90.0}"
# 独立夹爪模式按一次 [ 或 ] 的微动距离，单位 rad。
NUDGE_STEP_RAD="${X5_GRIPPER_NUDGE_STEP_RAD:-0.02}"
# 独立夹爪模式的微动力矩绝对值，单位 Nm。
NUDGE_TORQUE_NM="${X5_GRIPPER_NUDGE_TORQUE_NM:-0.25}"
# 独立夹爪模式按 h 后的保持时间，单位 s。
HOLD_SECONDS="${X5_GRIPPER_HOLD_SECONDS:-1.0}"
# 1：机械臂 + 夹爪联合控制；0：仅运行独立夹爪控制。
ENABLE_ARM="${X5_GRIPPER_ENABLE_ARM:-1}"
# Python 包内置的厂家 X5 Python 运行库；扩展模块需要兼容的 Python 3.12。
# 正常使用无需修改；只有替换厂家运行库时才设置 X5_GRIPPER_ARM_SDK_PATH。
ARM_SDK_PATH="${X5_GRIPPER_ARM_SDK_PATH:-${SDK_DIR}/src/x5_gripper_sdk/vendor_runtime/arx_x5_python}"
# 单次机械臂笛卡尔动作超时时间，单位 s。
ARM_MOTION_TIMEOUT="${X5_GRIPPER_ARM_MOTION_TIMEOUT:-15.0}"
# 机械臂任一关节反馈速度的安全上限，单位 rad/s。
ARM_MAX_VELOCITY="${X5_GRIPPER_ARM_MAX_VELOCITY:-0.5}"
# 一次联合按键控制会话允许持续的时间，单位 s。
ARM_SESSION_SECONDS="${X5_GRIPPER_ARM_SESSION_SECONDS:-120.0}"
# 双端点标定文件；默认使用本脚本同目录下的最新标定。
CALIBRATION_FILE="${X5_GRIPPER_CALIBRATION_FILE:-${SDK_DIR}/gripper-calibration.json}"
# 默认使用当前终端/Conda 环境中的 python，也可通过环境变量指定绝对路径。
PYTHON_BIN="${X5_GRIPPER_PYTHON_BIN:-python}"
# ===== 参数区结束 =====

# 线性控制必须先有有效标定文件；不存在时立即退出，不创建控制器。
if [[ ! -f "${CALIBRATION_FILE}" ]]; then
    echo "错误：找不到夹爪标定文件：${CALIBRATION_FILE}" >&2
    exit 2
fi

echo "使用最新标定文件：${CALIBRATION_FILE}"
# 直接加入本 SDK 的 src，避免依赖 editable 安装状态。
export PYTHONPATH="${SDK_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"
echo "使用 SDK 源码：${SDK_DIR}/src"

# 使用 Bash 数组组装命令，确保路径或参数中包含空格时不会被错误拆分。
COMMAND=(
    "${PYTHON_BIN}"
    "${SDK_DIR}/examples/calibrated_linear_control.py"
    --calibration "${CALIBRATION_FILE}"
    --interface "${GRIPPER_INTERFACE}"
    --motor-can-id "${MOTOR_CAN_ID}"
    --device-serial "${DEVICE_SERIAL}"
    --kp "${KP}"
    --kd "${KD}"
    --linear-speed-rad-s "${LINEAR_SPEED_RAD_S}"
    --startup-speed-rad-s "${STARTUP_SPEED_RAD_S}"
    --startup-timeout-s "${STARTUP_TIMEOUT_SECONDS}"
    --nudge-step-rad "${NUDGE_STEP_RAD}"
    --nudge-torque-nm "${NUDGE_TORQUE_NM}"
    --hold-seconds "${HOLD_SECONDS}"
)

# 只有设置了反馈 CAN ID 才传给 Python；空字符串不能解析成整数。
if [[ -n "${FEEDBACK_CAN_ID}" ]]; then
    COMMAND+=(--feedback-can-id "${FEEDBACK_CAN_ID}")
fi

# ENABLE_ARM=1 时增加机械臂联合控制参数；设为 0 即保留原独立夹爪模式。
if [[ "${ENABLE_ARM}" == "1" ]]; then
    # 厂家扩展依赖这两个同目录共享库；在启动 Python 前加入动态库搜索路径。
    ARM_LIBRARY_PATH="${ARM_SDK_PATH}/bimanual/api:${ARM_SDK_PATH}/bimanual/api/arx_x5_src"
    export LD_LIBRARY_PATH="${ARM_LIBRARY_PATH}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    COMMAND+=(
        --enable-arm
        --arm-sdk-path "${ARM_SDK_PATH}"
        --arm-motion-timeout "${ARM_MOTION_TIMEOUT}"
        --arm-max-velocity "${ARM_MAX_VELOCITY}"
        --arm-session-seconds "${ARM_SESSION_SECONDS}"
    )
fi

# "$@" 表示用户在启动命令末尾追加的参数。重复参数以最后一个值为准，
# 因此可用于临时覆盖上面的默认配置。
# exec 让 Python 接管当前进程，使 Ctrl+C 和程序退出状态能够直接传递。
exec "${COMMAND[@]}" "$@"
