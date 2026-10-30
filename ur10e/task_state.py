"""Crash-safe single-task journal for Pallet robot commands."""

from __future__ import annotations

import json
import os
import tempfile
import time
import uuid
from pathlib import Path


TASK_STATES = {
    "queued", "running", "succeeded", "failed", "recovery_required",
}
ACTIVE_STATES = {"queued", "running", "recovery_required"}


class TaskJournal:
    def __init__(self, path: str | Path, *, now=time.time) -> None:
        self.path = Path(path).expanduser()
        self._now = now
        self.record = self._read()
        if self.record and self.record["state"] == "running":
            self._transition(
                "recovery_required",
                error="previous process stopped while robot execution was running",
            )

    def _read(self) -> dict | None:
        if not self.path.exists():
            return None
        try:
            record = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"cannot read task journal {self.path}: {error}") from error
        if not isinstance(record, dict) or record.get("state") not in TASK_STATES:
            raise ValueError(f"invalid task journal {self.path}")
        return record

    def _write(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(
            prefix=f".{self.path.name}.", dir=self.path.parent, text=True,
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(self.record, stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _transition(self, state: str, **updates) -> dict:
        if self.record is None or state not in TASK_STATES:
            raise ValueError("task transition requires a valid record and state")
        timestamp = float(self._now())
        self.record.update(updates)
        self.record["state"] = state
        self.record["updated_at"] = timestamp
        self.record.setdefault("history", []).append({"state": state, "at": timestamp})
        self._write()
        return self.record.copy()

    def enqueue(self, command: dict, *, task_id: str | None = None, metadata=None) -> dict:
        if self.record and self.record["state"] in ACTIVE_STATES:
            raise ValueError(
                f"task {self.record.get('task_id')} is {self.record['state']}; "
                "finish or acknowledge recovery before queueing another command"
            )
        timestamp = float(self._now())
        self.record = {
            "version": 1,
            "task_id": task_id or str(uuid.uuid4()),
            "state": "queued",
            "attempt": 0,
            "command": command,
            "metadata": metadata or {},
            "created_at": timestamp,
            "updated_at": timestamp,
            "history": [{"state": "queued", "at": timestamp}],
            "result": None,
            "error": None,
        }
        self._write()
        return self.record.copy()

    def start(self) -> dict:
        if not self.record or self.record["state"] != "queued":
            raise ValueError("only a queued task can start")
        self.record["attempt"] += 1
        return self._transition("running", error=None)

    def succeed(self, result=None) -> dict:
        if not self.record or self.record["state"] != "running":
            raise ValueError("only a running task can succeed")
        return self._transition("succeeded", result=result or {}, error=None)

    def fail(self, error: str, result=None) -> dict:
        if not self.record or self.record["state"] not in {"queued", "running"}:
            raise ValueError("only a queued or running task can fail")
        return self._transition("failed", error=str(error), result=result or {})

    def require_recovery(self, error: str, result=None) -> dict:
        if not self.record or self.record["state"] != "running":
            raise ValueError("only a running task can require recovery")
        return self._transition(
            "recovery_required", error=str(error), result=result or {},
        )

    def acknowledge_recovery(self, note: str = "operator acknowledged recovery") -> dict:
        if not self.record or self.record["state"] != "recovery_required":
            raise ValueError("no recovery_required task to acknowledge")
        return self._transition("failed", recovery_acknowledgement=str(note))
