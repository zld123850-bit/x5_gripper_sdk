"""Linux SocketCAN 传输层。"""

from __future__ import annotations

import errno
import socket
import struct
from typing import Protocol


CAN_ERR_FLAG = 0x20000000
CAN_RTR_FLAG = 0x40000000
CAN_EFF_FLAG = 0x80000000
CAN_SFF_MASK = 0x7FF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


class CanTransport(Protocol):
    def send(self, can_id: int, data: bytes) -> None: ...
    def recv(self, timeout: float) -> tuple[int, bytes] | None: ...
    def close(self) -> None: ...


class SocketCanTransport:
    """一个关闭自身帧回环的原始 SocketCAN socket。"""

    def __init__(self, interface: str):
        if not hasattr(socket, "AF_CAN"):
            raise OSError("当前 Python/系统不支持 Linux AF_CAN。")
        self.interface = interface
        self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            if hasattr(socket, "CAN_RAW_RECV_OWN_MSGS"):
                self._socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
            self._socket.bind((interface,))
        except BaseException:
            self._socket.close()
            raise

    def send(self, can_id: int, data: bytes) -> None:
        payload = bytes(data)
        if len(payload) > 8:
            raise ValueError("CAN 数据不能超过 8 字节。")
        frame = struct.pack(
            CAN_FRAME_FMT,
            int(can_id) & CAN_SFF_MASK,
            len(payload),
            payload.ljust(8, b"\x00"),
        )
        try:
            self._socket.send(frame)
        except OSError as exc:
            if exc.errno != errno.ENOBUFS:
                raise

    def recv(self, timeout: float) -> tuple[int, bytes] | None:
        self._socket.settimeout(max(0.0, float(timeout)))
        try:
            raw = self._socket.recv(CAN_FRAME_SIZE)
        except (TimeoutError, socket.timeout, BlockingIOError):
            return None
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                return None
            raise
        if len(raw) < CAN_FRAME_SIZE:
            return None
        can_id, dlc, payload = struct.unpack(CAN_FRAME_FMT, raw)
        if can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG):
            return None
        return int(can_id) & CAN_SFF_MASK, bytes(payload[: int(dlc)])

    def close(self) -> None:
        self._socket.close()

