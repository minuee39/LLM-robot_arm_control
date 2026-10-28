import json

import pytest

from task_state import TaskJournal


class Clock:
    def __init__(self):
        self.value = 100.0

    def __call__(self):
        self.value += 1.0
        return self.value


def test_task_journal_records_success_atomically(tmp_path):
    path = tmp_path / "task.json"
    journal = TaskJournal(path, now=Clock())
    queued = journal.enqueue({"action": "pick_place"}, task_id="task-1")
    assert queued["state"] == "queued"
    assert journal.start()["attempt"] == 1
    assert journal.succeed({"final_xy_error_m": 0.004})["state"] == "succeeded"

    saved = json.loads(path.read_text(encoding="utf-8"))
    assert [item["state"] for item in saved["history"]] == [
        "queued", "running", "succeeded",
    ]
    assert saved["result"]["final_xy_error_m"] == 0.004


def test_running_task_becomes_recovery_required_after_restart(tmp_path):
    path = tmp_path / "task.json"
    journal = TaskJournal(path, now=Clock())
    journal.enqueue({"action": "pick_place"}, task_id="task-2")
    journal.start()

    restarted = TaskJournal(path, now=Clock())
    assert restarted.record["state"] == "recovery_required"
    with pytest.raises(ValueError, match="recovery_required"):
        restarted.enqueue({"action": "pick_place"})
    assert restarted.acknowledge_recovery("scene inspected")["state"] == "failed"
    assert restarted.enqueue({"action": "pick_place"})["state"] == "queued"


def test_active_task_cannot_be_overwritten(tmp_path):
    journal = TaskJournal(tmp_path / "task.json", now=Clock())
    journal.enqueue({"action": "pick_place"}, task_id="task-3")
    with pytest.raises(ValueError, match="queued"):
        journal.enqueue({"action": "pick_place"})
