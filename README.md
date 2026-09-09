# X5 Gripper SDK

Standalone SocketCAN SDK for the X5-2023 external DaMiao DM-J4310 gripper, with optional X5 arm + gripper combined control.

**完整中文使用说明（安装、实机检查、标定、CLI、Python API）：** [README.zh-CN.md](README.zh-CN.md)

## Quick start

```bash
git clone https://github.com/zld123850-bit/x5_gripper_sdk.git
cd x5_gripper_sdk

python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e .

x5-gripper --help
```

Requirements: Linux x86_64, Python 3.12, and a configured SocketCAN interface (default `can2`). Combined arm mode also needs the bundled CPython 3.12 vendor extension.

This SDK never sends motor `SET_ZERO`. Independent gripper motions send `DISABLE` on both normal completion and abnormal exit. Always confirm the motor CAN ID, clear the gripper, and keep e-stop reachable before connecting hardware.
