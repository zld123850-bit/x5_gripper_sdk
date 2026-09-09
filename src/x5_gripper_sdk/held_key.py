"""根据终端键盘自动重复推断按键是否仍按住。"""

from __future__ import annotations

import os
import select
import time
from typing import Callable


INITIAL_KEY_REPEAT_GRACE_S = 0.65
KEY_RELEASE_TIMEOUT_S = 0.18


class HeldKeyTracker:
    """普通终端没有真实 KeyUp，用自动重复字符判断按住状态。"""

    def __init__(
        self,
        descriptor: int,
        active_key: str,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ):
        self.descriptor = descriptor
        self.active_key = active_key
        self._monotonic = monotonic
        self.started_at = monotonic()
        self.last_seen_at = self.started_at
        self.repeat_seen = False
        self.pending_key: str | None = None

    def continue_motion(self) -> bool:
        now = self._monotonic()
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
        """到达行程上限后吞掉当前键的重复字符，避免松手前再次启动。"""
        last_active = self._monotonic()
        while self._monotonic() - last_active < KEY_RELEASE_TIMEOUT_S:
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
                last_active = self._monotonic()
            else:
                self.pending_key = key
                return
