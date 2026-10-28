#!/usr/bin/env python3
"""Interactive Gemini TaskSpec console for an already running Pallet simulator."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

from pallet_task_planner import compile_task_spec_with_gemini
from pallet_task_spec import load_task_spec
from mtc_command_runner import load_pallet_scene


SPEED_RESTART_RETURN_CODE = 75


def write_spec(path: Path, spec: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(spec, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def execute_spec(args, spec_path: Path, sequence: int) -> tuple[int, dict | None]:
    job_dir = args.output_root / f"job-{sequence:03d}-speed-{args.speed_scaling:.1f}"
    command = [
        sys.executable, str(args.palletizing_job),
        "--scene-file", str(args.scene_file),
        "--output-dir", str(job_dir),
        "--task-spec", str(spec_path),
        "--job-id", f"llm-{int(time.time())}-{sequence:03d}",
        "--detection-timeout", str(args.detection_timeout),
        "--arm-velocity-scaling", str(args.speed_scaling),
        "--arm-acceleration-scaling", str(args.speed_scaling),
        "--cartesian-velocity-scaling", str(args.speed_scaling),
        "--cartesian-acceleration-scaling", str(args.speed_scaling),
    ]
    completed = subprocess.run(command, cwd=args.repository_dir, check=False)
    report_path = job_dir / "report.json"
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        report = None
    return completed.returncode, report


def run(args) -> int:
    args.output_root.mkdir(parents=True, exist_ok=True)
    sequence = 0
    queued_spec = load_task_spec(args.retry_spec) if args.retry_spec else None
    while True:
        if queued_spec is None:
            print("\n[READY] Gemini 팔레타이징 명령을 입력하세요. (종료: exit)", flush=True)
            try:
                user_text = input("> ").strip()
            except EOFError:
                return 0
            if not user_text:
                continue
            if user_text.lower() in {"exit", "quit", "종료"}:
                return 0
            try:
                scene_manager = load_pallet_scene(
                    args.scene_file,
                    max_age=args.detection_timeout,
                )
                spec = compile_task_spec_with_gemini(
                    user_text,
                    scene_manager=scene_manager,
                )
            except ValueError as error:
                print(f"[REJECTED] {error}", flush=True)
                continue
            print("[TASKSPEC]", flush=True)
            print(json.dumps(spec, ensure_ascii=False, indent=2), flush=True)
            if input("실행할까요? [y/N] ").strip().lower() not in {"y", "yes", "예"}:
                print("[CANCELLED] 명령을 실행하지 않았습니다.", flush=True)
                continue
        else:
            spec = queued_spec
            queued_spec = None
            print(
                f"[RETRY] 새 시뮬레이션에서 속도/가속도 {args.speed_scaling:.1f}로 재실행합니다.",
                flush=True,
            )

        sequence += 1
        spec_path = args.output_root / f"task-{sequence:03d}.json"
        write_spec(spec_path, spec)
        returncode, report = execute_spec(args, spec_path, sequence)
        if returncode == 0 and report and report.get("passed") is True:
            print("[PASS] 작업 명세 실행이 완료됐습니다.", flush=True)
            continue
        failure_class = report.get("failure_class") if isinstance(report, dict) else "unknown"
        print(f"[FAIL] 작업 실패: {failure_class}", flush=True)
        if failure_class == "execution_dynamics":
            shutil.copyfile(spec_path, args.pending_retry_spec)
            print(
                "[SPEED] 동역학 실패로 전체 시뮬레이션을 초기화한 뒤 속도를 낮춥니다.",
                flush=True,
            )
            return SPEED_RESTART_RETURN_CODE


def main() -> int:
    repository_dir = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-file", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--speed-scaling", type=float, required=True)
    parser.add_argument("--retry-spec", type=Path)
    parser.add_argument("--pending-retry-spec", type=Path, required=True)
    parser.add_argument("--detection-timeout", type=float, default=45.0)
    parser.add_argument(
        "--palletizing-job", type=Path,
        default=repository_dir / "ur10e/palletizing_job.py")
    args = parser.parse_args()
    args.repository_dir = repository_dir
    if not 0.0 < args.speed_scaling <= 1.0:
        parser.error("--speed-scaling must be within (0, 1]")
    try:
        return run(args)
    except (OSError, ValueError) as error:
        print(f"[ERROR] {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
