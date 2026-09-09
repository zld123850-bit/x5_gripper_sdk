"""联合控制会话日志，默认保存在独立 SDK 的 logs 目录。"""

from __future__ import annotations

import re
from pathlib import Path

SESSION_LOG_DIR = Path(__file__).resolve().parents[3] / "logs"
SESSION_LOG_KEEP = 5
SESSION_LOG_GLOB = "x5_2023_*.jsonl"
_TIMESTAMP = re.compile(r"_(\d{8})_(\d{6})\.jsonl$")


def resolve_session_log_path(
    explicit: Path | None,
    filename: str,
    *,
    log_dir: Path | None = None,
    keep: int = SESSION_LOG_KEEP,
) -> Path:
    """Return the log path for this run.

    Default files go in ``x5_gripper_sdk/logs`` and older ``x5_2023_*.jsonl`` files in
    that directory are deleted so at most ``keep`` remain after this run.
    An explicit ``--log`` path is left as-is and is not pruned.
    """
    if explicit is not None:
        path = Path(explicit).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        return path
    directory = Path(log_dir or SESSION_LOG_DIR).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    path = (directory / filename).resolve()
    prune_session_logs(directory, keep=keep, incoming=path)
    return path


def prune_session_logs(
    directory: Path,
    *,
    keep: int = SESSION_LOG_KEEP,
    incoming: Path | None = None,
) -> list[Path]:
    """Delete older ``x5_2023_*.jsonl`` files so ``keep`` newest remain."""
    if keep < 1:
        raise ValueError("keep must be >= 1")
    directory = directory.resolve()
    files = {
        path.resolve()
        for path in directory.glob(SESSION_LOG_GLOB)
        if path.is_file()
    }
    if incoming is not None:
        files.add(Path(incoming).resolve())
    ranked = sorted(files, key=_recency_key, reverse=True)
    deleted: list[Path] = []
    incoming_resolved = Path(incoming).resolve() if incoming is not None else None
    for path in ranked[keep:]:
        if incoming_resolved is not None and path == incoming_resolved:
            continue
        if path.exists():
            path.unlink()
            deleted.append(path)
    return deleted


def _recency_key(path: Path) -> tuple[str, str]:
    match = _TIMESTAMP.search(path.name)
    if match is not None:
        return (match.group(1) + match.group(2), path.name)
    if path.exists():
        return (f"{path.stat().st_mtime_ns:020d}", path.name)
    return ("99999999999999", path.name)
