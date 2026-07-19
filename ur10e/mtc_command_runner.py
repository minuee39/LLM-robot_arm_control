import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np

from command_parser import parse_user_command_with_memory, validate_command
from llm_to_json import parse_user_command_with_llm
from scene_config import BLOCK_SIZE
from scene_manager import SceneManager


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_RUN_SCRIPT = PROJECT_DIR / "scripts" / "run_mtc_isaac_pick_place.sh"
DEFAULT_SCENE_STATE_FILE = Path("/tmp/ur10e_isaac_scene_objects.json")
DEFAULT_TOUCH_COLLISION_COMMAND = [
    "ros2",
    "run",
    "isaac_moveit_bridge",
    "allow_touch_collisions",
]


def select_provider(provider: str) -> str | None:
    if provider == "local":
        return None
    if provider in {"gemini", "chatgpt"}:
        return provider

    if os.environ.get("GEMINI_API_KEY"):
        return "gemini"
    if os.environ.get("OPENAI_API_KEY") or os.environ.get("CHATGPT_API_KEY"):
        return "chatgpt"
    return None


def parse_command(user_text: str, scene_manager: SceneManager, provider: str | None) -> dict:
    scene_objects = scene_manager.as_command_scene()
    if provider is None:
        return parse_user_command_with_memory(user_text, scene_manager.memory)

    try:
        print(f"[Parser] {provider} LLM 사용", flush=True)
        return parse_user_command_with_llm(user_text, scene_objects, provider=provider)
    except ValueError as error:
        print(f"[Parser] LLM 파싱 실패, 로컬 파서로 전환: {error}", flush=True)
        return parse_user_command_with_memory(user_text, scene_manager.memory)


def clamp_center_z(position: np.ndarray, object_height: float) -> np.ndarray:
    adjusted = np.array(position, dtype=float).copy()
    adjusted[2] = max(float(adjusted[2]), object_height / 2.0)
    return adjusted


def read_isaac_scene_objects(path: Path = DEFAULT_SCENE_STATE_FILE) -> dict[str, dict]:
    if not path.exists():
        return {}

    data = json.loads(path.read_text(encoding="utf-8"))
    objects = data.get("objects", data)
    if not isinstance(objects, list):
        return {}

    scene_objects = {}
    for item in objects:
        if not isinstance(item, dict) or "name" not in item or "position" not in item:
            continue
        scene_objects[str(item["name"])] = item
    return scene_objects


def sync_scene_manager_from_isaac(scene_manager: SceneManager, path: Path = DEFAULT_SCENE_STATE_FILE) -> bool:
    scene_objects = read_isaac_scene_objects(path)
    if not scene_objects:
        return False

    for name, item in scene_objects.items():
        if name not in scene_manager.names():
            continue
        scene_manager.update_object(
            name,
            position=item["position"],
            size=item.get("size"),
            confidence=1.0,
            status="on_table",
        )
    return True


def scene_state_mtime(path: Path = DEFAULT_SCENE_STATE_FILE) -> int | None:
    if not path.exists():
        return None
    return path.stat().st_mtime_ns


def wait_for_scene_state_update(
    previous_mtime_ns: int | None,
    path: Path = DEFAULT_SCENE_STATE_FILE,
    timeout: float = 2.0,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current_mtime_ns = scene_state_mtime(path)
        if current_mtime_ns is not None and current_mtime_ns != previous_mtime_ns:
            return
        time.sleep(0.05)


def command_to_mtc_args(command: dict, scene_manager: SceneManager) -> list[str]:
    pick_object = scene_manager.get_object(command["pick_object"])
    target_position = scene_manager.resolve_target_position(command)

    object_size = BLOCK_SIZE if pick_object.size is None else pick_object.size
    object_size = np.asarray(object_size, dtype=float)
    if object_size.shape != (3,) or np.any(object_size <= 0.0):
        raise ValueError(f"invalid object size for {pick_object.name}: {object_size}")

    object_height = float(object_size[2])
    object_position = clamp_center_z(pick_object.position, object_height)
    place_position = clamp_center_z(target_position, object_height)

    return [
        f"object_id:={command['pick_object']}",
        f"object_x:={object_position[0]:.6f}",
        f"object_y:={object_position[1]:.6f}",
        f"object_z:={object_position[2]:.6f}",
        f"object_size_x:={object_size[0]:.6f}",
        f"object_size_y:={object_size[1]:.6f}",
        f"object_size_z:={object_size[2]:.6f}",
        f"place_x:={place_position[0]:.6f}",
        f"place_y:={place_position[1]:.6f}",
        f"place_z:={place_position[2]:.6f}",
    ]


def execution_tuning_args(
    max_solutions: int,
    early_execute_cost_threshold: float,
    move_to_pick_timeout: float,
    move_to_pick_max_path_length: float,
    move_to_place_timeout: float,
    return_home_timeout: float,
    gripper_close_min: float,
    gripper_close_max: float,
    gripper_close_step: float,
) -> list[str]:
    return [
        f"max_solutions:={max_solutions}",
        f"early_execute_cost_threshold:={early_execute_cost_threshold:.3f}",
        f"move_to_pick_timeout:={move_to_pick_timeout:.3f}",
        f"move_to_pick_max_path_length:={move_to_pick_max_path_length:.3f}",
        f"move_to_place_timeout:={move_to_place_timeout:.3f}",
        f"return_home_timeout:={return_home_timeout:.3f}",
        f"gripper_close_min:={gripper_close_min:.3f}",
        f"gripper_close_max:={gripper_close_max:.3f}",
        f"gripper_close_step:={gripper_close_step:.3f}",
    ]


def build_mtc_invocation(
    user_text: str,
    provider: str = "auto",
    scene_manager: SceneManager | None = None,
) -> tuple[dict, list[str]]:
    if scene_manager is None:
        scene_manager = SceneManager.from_defaults()

    selected_provider = select_provider(provider)
    command = parse_command(user_text, scene_manager, selected_provider)
    validate_command(command, scene_manager.as_command_scene())
    return command, command_to_mtc_args(command, scene_manager)


def ensure_touch_collisions() -> int:
    completed = subprocess.run(DEFAULT_TOUCH_COLLISION_COMMAND, check=False)
    if completed.returncode != 0:
        print(
            f"[ERROR] Failed to apply MoveIt touch collision ACM with return code {completed.returncode}.",
            flush=True,
        )
    return completed.returncode


def execute_user_command(
    user_text: str,
    scene_manager: SceneManager,
    provider: str,
    run_script: str,
    dry_run: bool = False,
    max_solutions: int = 5,
    early_execute_cost_threshold: float = 40.0,
    move_to_pick_timeout: float = 5.0,
    move_to_pick_max_path_length: float = 8.0,
    move_to_place_timeout: float = 6.0,
    return_home_timeout: float = 1.0,
    gripper_close_min: float = 0.02,
    gripper_close_max: float = 0.45,
    gripper_close_step: float = 0.08,
) -> int:
    if sync_scene_manager_from_isaac(scene_manager):
        print("[Scene] synced object positions from Isaac scene state.", flush=True)

    command, mtc_args = build_mtc_invocation(user_text, provider=provider, scene_manager=scene_manager)
    mtc_args = [
        *mtc_args,
        *execution_tuning_args(
            max_solutions,
            early_execute_cost_threshold,
            move_to_pick_timeout,
            move_to_pick_max_path_length,
            move_to_place_timeout,
            return_home_timeout,
            gripper_close_min,
            gripper_close_max,
            gripper_close_step,
        ),
    ]

    print("[Command]", command, flush=True)
    print("[MTC args]", " ".join(mtc_args), flush=True)

    if dry_run:
        return 0

    scene_state_before_execution = scene_state_mtime()
    touch_collision_returncode = ensure_touch_collisions()
    if touch_collision_returncode != 0:
        print("[ERROR] MTC execution skipped because touch collision ACM was not applied.", flush=True)
        return touch_collision_returncode

    completed = subprocess.run([run_script, *mtc_args], check=False)
    if completed.returncode == 0:
        wait_for_scene_state_update(scene_state_before_execution)
        if sync_scene_manager_from_isaac(scene_manager):
            actual_position = scene_manager.get_object(command["pick_object"]).position
            scene_manager.apply_pick_place_result(command, actual_position)
            print("[Scene] updated command memory from Isaac actual object position.", flush=True)
        else:
            print("[WARN] Isaac scene state was unavailable. Scene memory was not updated.", flush=True)
    else:
        print(f"[ERROR] MTC execution failed with return code {completed.returncode}. Scene memory was not updated.", flush=True)

    return completed.returncode


def run_interactive(
    provider: str,
    run_script: str,
    dry_run: bool = False,
    initial_command: str | None = None,
    max_solutions: int = 5,
    early_execute_cost_threshold: float = 40.0,
    move_to_pick_timeout: float = 5.0,
    move_to_pick_max_path_length: float = 8.0,
    move_to_place_timeout: float = 6.0,
    return_home_timeout: float = 1.0,
    gripper_close_min: float = 0.02,
    gripper_close_max: float = 0.45,
    gripper_close_step: float = 0.08,
) -> int:
    scene_manager = SceneManager.from_defaults()

    last_returncode = 0
    if initial_command:
        try:
            last_returncode = execute_user_command(
                initial_command,
                scene_manager=scene_manager,
                provider=provider,
                run_script=run_script,
                dry_run=dry_run,
                max_solutions=max_solutions,
                early_execute_cost_threshold=early_execute_cost_threshold,
                move_to_pick_timeout=move_to_pick_timeout,
                move_to_pick_max_path_length=move_to_pick_max_path_length,
                move_to_place_timeout=move_to_place_timeout,
                return_home_timeout=return_home_timeout,
                gripper_close_min=gripper_close_min,
                gripper_close_max=gripper_close_max,
                gripper_close_step=gripper_close_step,
            )
        except ValueError as error:
            last_returncode = 1
            print(f"[ERROR] {error}", flush=True)

    while True:
        print("[READY] 다음 pick/place 명령을 입력하세요. (종료: exit, quit, 종료)", flush=True)
        try:
            user_text = input("> ").strip()
        except EOFError:
            print("", flush=True)
            return last_returncode

        if not user_text:
            continue

        if user_text.lower() in {"exit", "quit", "종료"}:
            return last_returncode

        try:
            last_returncode = execute_user_command(
                user_text,
                scene_manager=scene_manager,
                provider=provider,
                run_script=run_script,
                dry_run=dry_run,
                max_solutions=max_solutions,
                early_execute_cost_threshold=early_execute_cost_threshold,
                move_to_pick_timeout=move_to_pick_timeout,
                move_to_pick_max_path_length=move_to_pick_max_path_length,
                move_to_place_timeout=move_to_place_timeout,
                return_home_timeout=return_home_timeout,
                gripper_close_min=gripper_close_min,
                gripper_close_max=gripper_close_max,
                gripper_close_step=gripper_close_step,
            )
        except ValueError as error:
            last_returncode = 1
            print(f"[ERROR] {error}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run MTC pick-place from a natural-language command.")
    parser.add_argument("command", nargs="*", help="Natural-language pick-place command. If omitted, starts interactive mode.")
    parser.add_argument(
        "--provider",
        choices=["auto", "gemini", "chatgpt", "local"],
        default="auto",
        help="Parser provider. auto uses Gemini, then ChatGPT, then local fallback.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print parsed command and launch args only.")
    parser.add_argument("--run-script", default=str(DEFAULT_RUN_SCRIPT), help="MTC coordinate launch wrapper.")
    parser.add_argument("--once", action="store_true", help="Run the provided command once and exit.")
    parser.add_argument("--max-solutions", type=int, default=5)
    parser.add_argument("--early-execute-cost-threshold", type=float, default=40.0)
    parser.add_argument("--move-to-pick-timeout", type=float, default=5.0)
    parser.add_argument("--move-to-pick-max-path-length", type=float, default=8.0)
    parser.add_argument("--move-to-place-timeout", type=float, default=6.0)
    parser.add_argument("--return-home-timeout", type=float, default=1.0)
    parser.add_argument("--gripper-close-min", type=float, default=0.02)
    parser.add_argument("--gripper-close-max", type=float, default=0.45)
    parser.add_argument("--gripper-close-step", type=float, default=0.08)
    parser.add_argument(
        "--gripper-close-position",
        type=float,
        default=None,
        help="Compatibility option: use one fixed close value instead of the automatic close range.",
    )
    args = parser.parse_args()
    if args.gripper_close_position is not None:
        args.gripper_close_min = args.gripper_close_position
        args.gripper_close_max = args.gripper_close_position
        args.gripper_close_step = 1.0

    if not args.command:
        return run_interactive(
            args.provider,
            args.run_script,
            dry_run=args.dry_run,
            max_solutions=args.max_solutions,
            early_execute_cost_threshold=args.early_execute_cost_threshold,
            move_to_pick_timeout=args.move_to_pick_timeout,
            move_to_pick_max_path_length=args.move_to_pick_max_path_length,
            move_to_place_timeout=args.move_to_place_timeout,
            return_home_timeout=args.return_home_timeout,
            gripper_close_min=args.gripper_close_min,
            gripper_close_max=args.gripper_close_max,
            gripper_close_step=args.gripper_close_step,
        )

    user_text = " ".join(args.command)
    if args.once:
        scene_manager = SceneManager.from_defaults()
        return execute_user_command(
            user_text,
            scene_manager=scene_manager,
            provider=args.provider,
            run_script=args.run_script,
            dry_run=args.dry_run,
            max_solutions=args.max_solutions,
            early_execute_cost_threshold=args.early_execute_cost_threshold,
            move_to_pick_timeout=args.move_to_pick_timeout,
            move_to_pick_max_path_length=args.move_to_pick_max_path_length,
            move_to_place_timeout=args.move_to_place_timeout,
            return_home_timeout=args.return_home_timeout,
            gripper_close_min=args.gripper_close_min,
            gripper_close_max=args.gripper_close_max,
            gripper_close_step=args.gripper_close_step,
        )

    return run_interactive(
        args.provider,
        args.run_script,
        dry_run=args.dry_run,
        initial_command=user_text,
        max_solutions=args.max_solutions,
        early_execute_cost_threshold=args.early_execute_cost_threshold,
        move_to_pick_timeout=args.move_to_pick_timeout,
        move_to_pick_max_path_length=args.move_to_pick_max_path_length,
        move_to_place_timeout=args.move_to_place_timeout,
        return_home_timeout=args.return_home_timeout,
        gripper_close_min=args.gripper_close_min,
        gripper_close_max=args.gripper_close_max,
        gripper_close_step=args.gripper_close_step,
    )


if __name__ == "__main__":
    raise SystemExit(main())
