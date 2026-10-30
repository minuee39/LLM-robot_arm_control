import json
from types import SimpleNamespace

import pallet_llm_console


def test_execution_dynamics_requests_fresh_lower_speed_restart(tmp_path, monkeypatch):
    spec = {
        "version": 1, "frame": "base_footprint", "unit": "m",
        "tasks": [{
            "action": "pick_place", "pick_object": "red_block",
            "place_xyz": [0.24, -0.28, 0.025],
        }],
    }
    retry_spec = tmp_path / "retry.json"
    retry_spec.write_text(json.dumps(spec), encoding="utf-8")
    pending = tmp_path / "pending.json"
    args = SimpleNamespace(
        output_root=tmp_path / "out", retry_spec=retry_spec,
        speed_scaling=1.0, pending_retry_spec=pending,
    )
    monkeypatch.setattr(
        pallet_llm_console, "execute_spec",
        lambda unused_args, unused_path, unused_sequence: (
            1, {"passed": False, "failure_class": "execution_dynamics"}),
    )

    assert pallet_llm_console.run(args) == 75
    assert json.loads(pending.read_text(encoding="utf-8")) == spec


def test_non_speed_failure_does_not_request_speed_fallback(tmp_path, monkeypatch):
    spec = {
        "version": 1, "frame": "base_footprint", "unit": "m",
        "tasks": [{
            "action": "pick_place", "pick_object": "blue_block",
            "place_xyz": [0.28, -0.28, 0.025],
        }],
    }
    retry_spec = tmp_path / "retry.json"
    retry_spec.write_text(json.dumps(spec), encoding="utf-8")
    args = SimpleNamespace(
        output_root=tmp_path / "out", retry_spec=retry_spec,
        speed_scaling=1.0, pending_retry_spec=tmp_path / "pending.json",
    )
    monkeypatch.setattr(
        pallet_llm_console, "execute_spec",
        lambda unused_args, unused_path, unused_sequence: (
            1, {"passed": False, "failure_class": "vision"}),
    )
    monkeypatch.setattr("builtins.input", lambda unused_prompt="": "exit")

    assert pallet_llm_console.run(args) == 0
    assert not args.pending_retry_spec.exists()
