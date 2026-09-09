# X5 外置夹爪 SDK 中文使用说明

本 SDK 用于控制 X5-2023 外置夹爪的 DaMiao DM-J4310 电机，提供两种运行方式：

- 独立夹爪控制：通过 SocketCAN 直接控制一个明确配置的夹爪 ESC，不初始化机械臂；
- 机械臂与夹爪联合控制：使用 SDK 内置的厂家运行库，在同一个 `can2` 接口上控制
  X5 机械臂和外置夹爪。

主要功能包括当前位置保持、相对微动、双端点软件标定、归一化开度控制、全开/全闭线性
控制、交互式键盘控制、反馈读取，以及超时、超速、越界、反向运动和控制器冲突保护。

> **重要：**SDK 永远不会发送电机 `SET_ZERO` 命令。标定只记录软件端点，不修改
> 电机内部零点。独立夹爪动作正常结束或异常退出时都会发送 `DISABLE`。

## 1. 运行环境与实机检查

软件运行环境需要：

- Linux x86_64；
- Python `3.12`（项目限制为 `>=3.12,<3.13`）；
- 已配置并启用的 SocketCAN 接口，默认名称为 `can2`。

实机运行还需要外置 DM-J4310 夹爪；联合模式同时需要 X5-2023 机械臂。

每次连接实机前必须完成以下检查：

1. 固定机械臂，清空夹爪内部，并确保急停可立即触达。
2. 确认 USB-CAN 设备对应的 SocketCAN 接口确实是 `can2`。
3. 停止厂家 `SingleArm` 及其他所有会向 `can2` 写入命令的程序。
4. 核对夹爪电机 CAN ID。仓库默认使用 `8`，不能在未确认实机 ID 时直接运行。
5. 确认 `kp=5.0`、`kd=0.2` 适用于当前设备；它们是本项目使用的受限参数，不是
   所有设备的通用值。

查看 CAN 接口状态：

```bash
ip -details link show can2
```

本仓库不负责创建或配置 SocketCAN 接口；请先使用所配 USB-CAN 适配器的配置流程将接口
设置为 `UP`。

## 2. 安装

进入 SDK 根目录。如果使用项目现有 Conda 环境，可先执行：

```bash
conda activate arx5-py312
cd /home/lida/lida-ws/x5_gripper_sdk
python -m pip install -e .
```

若 SDK 位于其他位置，请将 `cd` 后的路径替换为实际路径。目录名和 Python 包名中的
下划线 `_` 不需要转义，不能写成 `x5\_gripper\_sdk`。

验证安装：

```bash
x5-gripper --help
python -c "import x5_gripper_sdk; print(x5_gripper_sdk.__all__)"
```

只有独立夹爪控制使用纯 Python SocketCAN；机械臂联合模式还会加载包内的 CPython 3.12
x86_64 厂家二进制扩展，因此必须使用兼容的 Python 和平台。

## 3. 第一次低风险验证

首先只保持当前位置 `0.2 s`：

```bash
x5-gripper \
  --interface can2 \
  --motor-can-id 8 \
  --kp 5.0 \
  --kd 0.2 \
  --acknowledge-unverified-hardware \
  hold --seconds 0.2
```

`--acknowledge-unverified-hardware` 表示操作者已经核对 CAN ID、固定机械臂并准备好急停，
不是跳过 SDK 内部安全检查。程序会尝试独占目标电机、确认反馈静止、短暂保持，然后自动
失能。若运动方向、速度、声音或温度异常，请立即按物理急停，不要继续标定。

需要逐步测试开合时，可使用需要按 Enter 的交互模式：

```bash
x5-gripper \
  --interface can2 \
  --motor-can-id 8 \
  --kp 5.0 \
  --kd 0.2 \
  --acknowledge-unverified-hardware \
  interactive
```

输入大写 `RUN` 后，使用：

- `h`：保持当前位置；
- `c`：小步闭合；
- `o`：小步张开；
- `s`：发送 `DISABLE`；
- `q`：失能并退出。

每个命令输入一个字母后按 Enter。动作返回 `target_reached=false` 时，不要继续重复同方向
命令；先检查机械端点、障碍物、传动卡滞和反馈力矩。

## 4. 双端点标定

标定文件记录以下对应关系：

- `opening=0.0`：完全闭合端；
- `opening=1.0`：最大安全张开端；
- 中间值：按电机编码器行程线性归一化，不代表夹指的毫米距离。

当前控制模型要求张开时编码器值减小，即
`open_position_rad < closed_position_rad`。方向不符合时程序会拒绝保存或拒绝运动。

### 4.1 推荐：持续按键标定

在 SDK 根目录运行：

```bash
bash ./start-gripper-calibration.sh
```

脚本使用自身位置定位源码，默认将结果写入同目录的 `gripper-calibration.json`，不会初始化
机械臂。它会传入 `--ignore-existing-calibration`，因此重新标定时不会使用旧端点保护；只能
用默认 `0.02 rad` 小步谨慎靠近机械端点。

确认安全条件并输入大写 `RUN` 后，按键无需 Enter：

- 按住 `c`：连续小步闭合；
- 按住 `o`：连续小步张开；
- `h`：保持当前位置；
- `z`：夹爪完全闭合且静止后采集闭合端；
- `m`：夹爪最大安全张开且静止后采集张开端，保存并立即加载标定；
- 空格：立即发送 `DISABLE`；
- `?`：重新显示帮助；
- `q` 或 `Esc`：失能并退出。

完整顺序是：

1. 按住 `c` 小步移动到闭合端，松键后按 `z`。
2. 按住 `o` 小步移动到最大安全张开端，松键后按 `m`。
3. 确认终端显示标定已保存，并检查打印出的闭合端、张开端和行程。

若某方向未到达目标，该方向会被锁定。检查端点或阻挡后，按反方向键解除锁定。

### 4.2 直接使用 CLI 标定

无需持续按键时，也可以先用 `close`、`open` 小距离靠近端点，再采集两个静止位置：

```bash
# 小距离闭合：位置斜坡
x5-gripper \
  --interface can2 --motor-can-id 8 --kp 5.0 --kd 0.2 \
  --acknowledge-unverified-hardware \
  close --distance-rad 0.03 --speed-rad-s 0.05

# 小距离张开：kp=0，负前馈力矩
x5-gripper \
  --interface can2 --motor-can-id 8 --kp 5.0 --kd 0.2 \
  --acknowledge-unverified-hardware \
  open --distance-rad 0.03 --torque-nm -0.08

# 依次采集闭合端和张开端
x5-gripper \
  --interface can2 --motor-can-id 8 --kp 5.0 --kd 0.2 \
  --device-serial X5-2023-001 \
  --acknowledge-unverified-hardware \
  calibrate --output ./gripper-calibration.json
```

`calibrate` 本身不会发送位置或力矩运动目标，只采集静止反馈；若失能后没有反馈，SDK 可能
短暂发送 `ENABLE` 以读取状态，并在采集后再次发送 `DISABLE`。程序会先要求在闭合端输入
大写 `CLOSED`，再要求在张开端输入大写 `OPEN`。

默认安全硬限为单次闭合不超过 `0.20 rad`、单次张开不超过 `0.50 rad`、前馈力矩绝对值
不超过 `0.50 Nm`。标定前没有软件端点保护，不要使用一次命令尝试完整行程。

### 4.3 标定文件注意事项

仓库自带的 `gripper-calibration.json` 是序列号 `X5-2023-001` 的设备标定，不是通用出厂
参数。更换夹爪、电机、机械传动、CAN 接口、设备序列号，或编码器坐标发生变化后，必须
重新标定，不能直接复用该文件或交换两个端点绕过方向检查。

标定文件会校验模型、电机类型、设备序列号、CAN 接口、CAN ID、编码范围和行程。保存采用
原子替换，不会发送 `SET_ZERO`。

## 5. 使用已标定夹爪

### 5.1 一键启动机械臂与夹爪联合控制

默认启动脚本会开启联合控制：

```bash
bash ./start-calibrated-gripper.sh
```

脚本默认加载同目录的 `gripper-calibration.json`，并自动设置源码路径和厂家动态库路径。
联合模式只允许 `interface=can2`、`motor-can-id=8`，机械臂与夹爪共用 CAN 总线，运行期间
不能再启动其他机械臂或夹爪控制程序。

阅读启动提示并输入 `1` 后，程序会在厂家初始化前将夹爪低速移动到标定参考位置，然后完成
受监控的控制权交接。按键无需 Enter：

- `W` / `S`：末端沿基坐标系 `X+` / `X-` 移动；
- `A` / `D`：末端沿基坐标系 `Y+` / `Y-` 移动；
- `R` / `F`：末端沿基坐标系 `Z+` / `Z-` 移动；
- `H`：机械臂以受限轨迹返回本次程序启动位置；
- 按住 `c` / `o`：夹爪连续闭合 / 张开，松键后保持；
- `]` / `[`：夹爪闭合 / 张开微动；
- `p`：显示机械臂、末端位姿和夹爪状态；
- 空格：夹爪保持当前位置；
- `q` 或 `Esc`：机械臂进入保护状态并退出。

每次机械臂方向键请求末端移动 `1 cm`，目标轨迹速度为 `1 cm/s`。普通终端没有真实 KeyUp
事件，夹爪的持续开合依赖键盘自动重复；若桌面环境关闭了自动重复，按住按键只会产生一次
很小的目标增量。

联合控制不会同时运行直连 `X5Gripper` 和厂家 `SingleArm`。原始标定 JSON 以只读方式
加载，联合模式中的 `Z/M` 标定写入被禁用；需要重新标定时请使用第 4 节的独立流程。

### 5.2 只控制夹爪

不初始化机械臂时：

```bash
X5_GRIPPER_ENABLE_ARM=0 bash ./start-calibrated-gripper.sh
```

输入大写 `RUN` 后，按键为：

- 按住 `o` / `c`：向全开 / 全闭方向线性运动，松手停止；
- `1`～`9`：移动到 `10%`～`90%` 开度；
- `0`：移动到完全闭合端；
- `[` / `]`：张开 / 闭合微动；
- `p`：读取位置、速度和归一化开度；
- `h`：保持当前位置；
- 空格：发送 `DISABLE`；
- `?`：显示帮助；
- `q` 或 `Esc`：失能并退出。

### 5.3 启动参数

长期默认值集中在 `start-calibrated-gripper.sh` 顶部的“常用参数区”。常用环境变量如下：

| 环境变量 | 默认值 | 含义 |
| --- | ---: | --- |
| `X5_GRIPPER_INTERFACE` | `can2` | SocketCAN 接口 |
| `X5_GRIPPER_MOTOR_CAN_ID` | `8` | 夹爪电机 CAN ID |
| `X5_GRIPPER_DEVICE_SERIAL` | `X5-2023-001` | 标定文件设备序列号 |
| `X5_GRIPPER_CALIBRATION_FILE` | 脚本同目录 JSON | 标定文件路径 |
| `X5_GRIPPER_LINEAR_SPEED_RAD_S` | `0.25` | 按住 `c/o` 时的目标速度 |
| `X5_GRIPPER_STARTUP_SPEED_RAD_S` | `0.35` | 联合模式初始化前参考运动速度 |
| `X5_GRIPPER_STARTUP_TIMEOUT_SECONDS` | `90.0` | 参考运动总超时 |
| `X5_GRIPPER_ENABLE_ARM` | `1` | `1`=联合控制，`0`=仅夹爪 |
| `X5_GRIPPER_ARM_SESSION_SECONDS` | `120.0` | 联合控制单次会话时限 |
| `X5_GRIPPER_PYTHON_BIN` | `python` | 启动使用的 Python |

还可设置 `X5_GRIPPER_FEEDBACK_CAN_ID`、`X5_GRIPPER_KP`、`X5_GRIPPER_KD`、
`X5_GRIPPER_NUDGE_STEP_RAD`、`X5_GRIPPER_NUDGE_TORQUE_NM`、
`X5_GRIPPER_HOLD_SECONDS`、`X5_GRIPPER_ARM_SDK_PATH`、
`X5_GRIPPER_ARM_MOTION_TIMEOUT` 和 `X5_GRIPPER_ARM_MAX_VELOCITY`。

临时参数也可以追加在脚本命令后，重复参数以最后一个值为准：

```bash
bash ./start-calibrated-gripper.sh \
  --linear-speed-rad-s 0.10 \
  --arm-session-seconds 180
```

## 6. 命令行控制

`x5-gripper` 的接口、CAN ID、增益、标定文件等全局参数必须写在子命令之前。

移动到 `50%` 开度：

```bash
x5-gripper \
  --interface can2 \
  --motor-can-id 8 \
  --kp 5.0 --kd 0.2 \
  --device-serial X5-2023-001 \
  --calibration ./gripper-calibration.json \
  --acknowledge-unverified-hardware \
  move --opening 0.50 --speed-rad-s 0.10
```

线性全开和全闭：

```bash
x5-gripper \
  --calibration ./gripper-calibration.json \
  --acknowledge-unverified-hardware \
  open-linear --speed-rad-s 0.10

x5-gripper \
  --calibration ./gripper-calibration.json \
  --acknowledge-unverified-hardware \
  close-linear --speed-rad-s 0.10
```

可用子命令：

| 子命令 | 用途 | 是否要求标定 |
| --- | --- | --- |
| `hold` | 保持当前位置 | 否 |
| `move` | 移动到 `[0,1]` 归一化开度 | 是 |
| `open-linear` | 移动到标定张开端 | 是 |
| `close-linear` | 移动到标定闭合端 | 是 |
| `close` | 相对闭合微动 | 否 |
| `open` | 相对张开微动 | 否 |
| `interactive` | 需要 Enter 的交互实机测试 | 否 |
| `live` | 无需 Enter 的持续按键与双端点标定 | 输出文件必填 |
| `calibrate` | 手动摆放后采集两个静止端点 | 输出文件必填 |

查看某个子命令的完整参数：

```bash
x5-gripper move --help
x5-gripper live --help
```

## 7. Python API

```python
from pathlib import Path

from x5_gripper_sdk import GripperConfig, X5Gripper

config = GripperConfig(
    interface="can2",
    motor_can_id=8,
    feedback_can_id=None,  # 未确认反馈帧 CAN ID 时，按 payload 内的 motor ID 匹配
    kp=5.0,
    kd=0.2,
    device_serial="X5-2023-001",
    calibration_path=Path("./gripper-calibration.json"),
    acknowledge_unverified_hardware=True,
)

with X5Gripper(config) as gripper:
    hold = gripper.hold(0.2)
    print(hold.final_state)

    half_open = gripper.move_to_opening(0.5, speed_rad_s=0.10)
    print("到达 50%：", half_open.target_reached)

    opened = gripper.open_linearly(speed_rad_s=0.10)
    print("到达全开端：", opened.target_reached)

    nudged = gripper.close_relative(0.03, torque_nm=0.25)
    print("闭合微动到达目标：", nudged.target_reached)
```

推荐始终使用 `with`，使上下文退出时再次发送 `DISABLE`。业务程序也可显式调用：

```python
gripper.emergency_stop()
```

主要公共方法：

- `connect()` / `close()`：连接或关闭 SocketCAN；
- `hold()`：保持当前位置；
- `move_to_opening()`：移动到绝对归一化开度；
- `open_linearly()` / `close_linearly()`：线性全开 / 全闭；
- `open_linearly_while()` / `close_linearly_while()`：由回调决定是否继续运动；
- `open_relative()` / `close_relative()`：小距离相对微动；
- `capture_stationary_position()`：读取静止位置，完成后发送 `DISABLE`；
- `reload_calibration()`：重新加载标定文件；
- `emergency_stop()`：立即向目标 ESC 发送 `DISABLE`。

`MotionResult` 包含起点、目标、最终状态、最大速度、最大位移、最大跟踪误差、采样数、
`target_reached` 和 `stopped_by_request`。完整最小示例见 `examples/basic_control.py`。

## 8. 安全边界与常见问题

- SDK 是同步阻塞接口，同一个 `X5Gripper` 实例一次只执行一个动作。
- 独立模式每个动作完成后都会失能，不会跨 API 调用维持持续夹持力。
- 力矩微动遇到物体时可能无法到达编码器目标，必须检查 `target_reached`。
- 标定后的绝对位置控制要求 `kp > 0`，且目标必须位于双端点范围内。
- `opening` 是编码器归一化值，不是夹指间距，也不是夹持力。
- SDK 未将电机反馈力矩换算为夹持力；夹持力控制需要另做机构和测力标定。
- `feedback_can_id=None` 会按反馈 payload 内的电机 ID 匹配；确认反馈 CAN ID 后，建议配置
  精确值。
- 出现“其他控制器正在发送 MIT 命令”时，停止所有共享 CAN 总线的控制程序后再试。
- 出现标定设备、接口或 CAN ID 不匹配时，应使用当前设备重新标定，不要编辑 JSON 绕过。
- 联合模式导入厂家扩展失败时，先确认系统为 x86_64、Python 为 3.12，并通过启动脚本运行。

## 9. 代码位置与开发测试

```text
src/x5_gripper_sdk/driver.py                 独立夹爪同步 API
src/x5_gripper_sdk/cli.py                    x5-gripper 命令行
src/x5_gripper_sdk/calibration.py            双端点标定读写与校验
src/x5_gripper_sdk/integrated/               联合控制、安全交接与力估计
src/x5_gripper_sdk/vendor_runtime/            内置厂家机械臂运行库
examples/basic_control.py                    最小 Python 示例
examples/continuous_calibration_test.py      持续按键标定
examples/calibrated_linear_control.py        已标定控制与联合控制入口
```

单元测试不连接实机：

```bash
cd /home/lida/lida-ws/x5_gripper_sdk
PYTHONPATH=src python -m unittest discover -s tests -v
```
