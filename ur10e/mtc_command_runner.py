import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np

from command_parser import parse_user_command_with_memory, validate_command
from grasp_verifier import GraspMonitor
from llm_to_json import parse_user_command_with_llm
from scene_config import BLOCK_SIZE
from scene_manager import SceneManager
from vision.scene_objects import EXPECTED_BLOCK_NAMES


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_RUN_SCRIPT = PROJECT_DIR / "scripts" / "run_mtc_isaac_pick_place.sh"
DEFAULT_SCENE_STATE_FILE = Path("/tmp/ur10e_isaac_scene_objects.json")
DEFAULT_SCENE_MEMORY_FILE = Path("/tmp/ur10e_command_scene_memory.json")
DEFAULT_VISION_SCENE_FILE = Path("/tmp/ur10e_vision_scene.json")
DEFAULT_VISION_MAX_AGE = 2.0
DEFAULT_VISION_MIN_CONFIDENCE = 0.6
DEFAULT_GRASP_MIN_LIFT = 0.03
DEFAULT_GRASP_RELATIVE_TOLERANCE = 0.03
DEFAULT_GRASP_MIN_LIFTED_SAMPLES = 2
DEFAULT_TOUCH_COLLISION_COMMAND = [
    "ros2",
    "run",
    "isaac_moveit_bridge",
    "allow_touch_collisions",
]
SUPPORTED_PLANNER_IDS = (
    "RRTkConfigDefault",
    "RRTConnectkConfigDefault",
    "RRTstarkConfigDefault",
)


def format_xyz(position) -> str:
    values = np.asarray(position, dtype=float)
    return f"({values[0]:+.3f}, {values[1]:+.3f}, {values[2]:+.3f}) m"


def format_path_limit(value: float) -> str:
    return "off" if value < 0.0 else f"{value:.1f}"


def print_execution_summary(
    command: dict,
    scene_manager: SceneManager,
    *,
    planner_id: str,
    max_solutions: int,
    max_solution_cost: float,
    move_to_pick_timeout: float,
    move_to_pick_max_path_length: float,
    move_to_place_timeout: float,
    move_to_place_max_path_length: float,
    return_home_timeout: float,
    return_home_max_path_length: float,
    gripper_close_min: float,
    gripper_close_max: float,
) -> None:
    pick_object = scene_manager.get_object(command["pick_object"])
    object_size = BLOCK_SIZE if pick_object.size is None else np.asarray(pick_object.size, dtype=float)
    pick_position = clamp_center_z(pick_object.position, float(object_size[2]))
    target_position = clamp_center_z(
        scene_manager.resolve_target_position(command),
        float(object_size[2]),
    )
    rows = (
        ("object", command["pick_object"]),
        ("pick XYZ", format_xyz(pick_position)),
        ("target", command["target_object"]),
        ("place XYZ", format_xyz(target_position)),
        ("planner", planner_id),
        ("solutions", str(max_solutions)),
        (
            "quality limits",
            f"cost <= {max_solution_cost:.1f} | paths {format_path_limit(move_to_pick_max_path_length)} / "
            f"{format_path_limit(move_to_place_max_path_length)} / "
            f"{format_path_limit(return_home_max_path_length)}",
        ),
        (
            "timeouts",
            f"pick {move_to_pick_timeout:.1f}s | place {move_to_place_timeout:.1f}s | home {return_home_timeout:.1f}s",
        ),
        ("gripper", f"{gripper_close_min:.3f} .. {gripper_close_max:.3f} rad"),
    )
    width = max(len(label) for label, _ in rows)
    print("\n" + "=" * 68, flush=True)
    print(" PICK & PLACE", flush=True)
    print("-" * 68, flush=True)
    for label, value in rows:
        print(f" {label:<{width}} : {value}", flush=True)
    print("=" * 68, flush=True)


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


def save_scene_memory(
    scene_manager: SceneManager,
    path: Path = DEFAULT_SCENE_MEMORY_FILE,
) -> None:
    objects = {}
    for name, item in scene_manager.as_command_scene().items():
        objects[name] = {
            "position": np.asarray(item["position"], dtype=float).tolist(),
            "confidence": float(item.get("confidence", 1.0)),
            "status": str(item.get("status", "on_table")),
        }
        if "size" in item:
            objects[name]["size"] = np.asarray(item["size"], dtype=float).tolist()
        if "orientation" in item:
            objects[name]["orientation"] = np.asarray(item["orientation"], dtype=float).tolist()

    path = Path(path)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(
        json.dumps({"updated_at": time.time(), "objects": objects}),
        encoding="utf-8",
    )
    os.replace(tmp_path, path)


def restore_scene_memory(
    scene_manager: SceneManager,
    path: Path = DEFAULT_SCENE_MEMORY_FILE,
) -> bool:
    path = Path(path)
    if not path.exists():
        return False
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False

    objects = data.get("objects")
    if not isinstance(objects, dict):
        return False

    restored = False
    for name, item in objects.items():
        if name not in scene_manager.names() or not isinstance(item, dict):
            continue
        position = np.asarray(item.get("position"), dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            continue
        scene_manager.update_object(
            name,
            position,
            orientation=item.get("orientation"),
            size=item.get("size"),
            confidence=float(item.get("confidence", 1.0)),
            status=str(item.get("status", "on_table")),
        )
        restored = True
    return restored


def create_runtime_scene_manager() -> SceneManager:
    scene_manager = SceneManager.from_defaults()
    if restore_scene_memory(scene_manager):
        print("[Scene] restored last-known object positions from local memory.", flush=True)
    if sync_scene_manager_from_isaac(scene_manager):
        print("[Scene] refreshed hidden object positions from the current Isaac scene.", flush=True)
    save_scene_memory(scene_manager)
    return scene_manager


def read_vision_scene_objects(
    path: Path = DEFAULT_VISION_SCENE_FILE,
    *,
    max_age: float = DEFAULT_VISION_MAX_AGE,
    min_confidence: float = DEFAULT_VISION_MIN_CONFIDENCE,
    now: float | None = None,
) -> dict[str, dict]:
    path = Path(path).expanduser()
    if max_age <= 0.0:
        raise ValueError("vision max age must be positive")
    if not 0.0 <= min_confidence <= 1.0:
        raise ValueError("vision minimum confidence must be between 0 and 1")
    if not path.exists():
        raise ValueError(f"vision scene file not found: {path}. Start the YOLO camera node first")

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"failed to read vision scene: {error}") from error

    if data.get("frame") != "world":
        raise ValueError("vision scene frame must be 'world'")

    updated_at = data.get("updated_at")
    if not isinstance(updated_at, (int, float)) or not np.isfinite(updated_at):
        raise ValueError("vision scene updated_at must be a finite timestamp")
    if now is None:
        now = time.time()
    age = float(now) - float(updated_at)
    if age < -1.0:
        raise ValueError(f"vision scene timestamp is {abs(age):.2f}s in the future")
    if age > max_age:
        raise ValueError(f"vision scene is stale: age={age:.2f}s, limit={max_age:.2f}s")

    objects = data.get("objects")
    if not isinstance(objects, dict):
        raise ValueError("vision scene objects must be an object keyed by block name")
    if not objects:
        raise ValueError("vision scene does not contain any detected blocks")

    unknown_names = sorted(set(objects) - set(EXPECTED_BLOCK_NAMES))
    if unknown_names:
        raise ValueError(f"vision scene contains unknown blocks: {', '.join(unknown_names)}")

    validated = {}
    for name, item in objects.items():
        if not isinstance(item, dict):
            raise ValueError(f"invalid vision detection for {name}")
        position = np.asarray(item.get("position"), dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError(f"invalid world position for {name}: {item.get('position')}")
        confidence = item.get("confidence")
        if not isinstance(confidence, (int, float)) or not np.isfinite(confidence):
            raise ValueError(f"invalid confidence for {name}: {confidence}")
        if confidence < min_confidence:
            raise ValueError(
                f"vision confidence too low for {name}: {confidence:.3f} < {min_confidence:.3f}"
            )
        validated[name] = {
            "position": position,
            "confidence": float(confidence),
        }
    return validated


def sync_scene_manager_from_vision(
    scene_manager: SceneManager,
    path: Path = DEFAULT_VISION_SCENE_FILE,
    *,
    max_age: float = DEFAULT_VISION_MAX_AGE,
    min_confidence: float = DEFAULT_VISION_MIN_CONFIDENCE,
) -> set[str]:
    scene_objects = read_vision_scene_objects(
        path,
        max_age=max_age,
        min_confidence=min_confidence,
    )
    for name, item in scene_objects.items():
        if name not in scene_manager.names():
            continue
        scene_manager.update_object(
            name,
            position=item["position"],
            confidence=item["confidence"],
            status="on_table",
        )
    return set(scene_objects)


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


def vision_scene_mtime(path: Path = DEFAULT_VISION_SCENE_FILE) -> int | None:
    path = Path(path).expanduser()
    if not path.exists():
        return None
    return path.stat().st_mtime_ns


def wait_for_vision_scene_update(
    previous_mtime_ns: int | None,
    path: Path = DEFAULT_VISION_SCENE_FILE,
    timeout: float = 3.0,
) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current_mtime_ns = vision_scene_mtime(path)
        if current_mtime_ns is not None and current_mtime_ns != previous_mtime_ns:
            return True
        time.sleep(0.05)
    return False


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
    planner_id: str,
    max_solutions: int,
    max_solution_cost: float,
    move_to_pick_timeout: float,
    move_to_pick_max_path_length: float,
    move_to_place_timeout: float,
    move_to_place_max_path_length: float,
    return_home_timeout: float,
    return_home_max_path_length: float,
    gripper_close_min: float,
    gripper_close_max: float,
    gripper_close_step: float,
) -> list[str]:
    return [
        f"planner_id:={planner_id}",
        f"max_solutions:={max_solutions}",
        f"max_solution_cost:={max_solution_cost:.3f}",
        f"move_to_pick_timeout:={move_to_pick_timeout:.3f}",
        f"move_to_pick_max_path_length:={move_to_pick_max_path_length:.3f}",
        f"move_to_place_timeout:={move_to_place_timeout:.3f}",
        f"move_to_place_max_path_length:={move_to_place_max_path_length:.3f}",
        f"return_home_timeout:={return_home_timeout:.3f}",
        f"return_home_max_path_length:={return_home_max_path_length:.3f}",
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
    planner_id: str = "RRTConnectkConfigDefault",
    max_solutions: int = 2,
    max_solution_cost: float = 60.0,
    move_to_pick_timeout: float = 5.0,
    move_to_pick_max_path_length: float = -1.0,
    move_to_place_timeout: float = 4.0,
    move_to_place_max_path_length: float = -1.0,
    return_home_timeout: float = 1.0,
    return_home_max_path_length: float = -1.0,
    gripper_close_min: float = 0.02,
    gripper_close_max: float = 0.50,
    gripper_close_step: float = 0.08,
    vision_scene_file: str | Path = DEFAULT_VISION_SCENE_FILE,
    vision_max_age: float = DEFAULT_VISION_MAX_AGE,
    vision_min_confidence: float = DEFAULT_VISION_MIN_CONFIDENCE,
    vision_verification_timeout: float = 3.0,
) -> int:
    vision_scene_file = Path(vision_scene_file).expanduser()
    detected_names = sync_scene_manager_from_vision(
        scene_manager,
        vision_scene_file,
        max_age=vision_max_age,
        min_confidence=vision_min_confidence,
    )
    print(
        "[Scene] synced YOLO detections: " + ", ".join(sorted(detected_names)),
        flush=True,
    )

    command, mtc_args = build_mtc_invocation(user_text, provider=provider, scene_manager=scene_manager)
    required_names = {command["pick_object"], command["target_object"]}
    missing_required = sorted(required_names - detected_names)
    if missing_required:
        remembered_positions = ", ".join(
            f"{name}={format_xyz(scene_manager.get_object(name).position)}"
            for name in missing_required
        )
        print(
            "[Scene] camera occluded; using remembered positions: " + remembered_positions,
            flush=True,
        )
    mtc_args = [
        *mtc_args,
        *execution_tuning_args(
            planner_id,
            max_solutions,
            max_solution_cost,
            move_to_pick_timeout,
            move_to_pick_max_path_length,
            move_to_place_timeout,
            move_to_place_max_path_length,
            return_home_timeout,
            return_home_max_path_length,
            gripper_close_min,
            gripper_close_max,
            gripper_close_step,
        ),
    ]

    print_execution_summary(
        command,
        scene_manager,
        planner_id=planner_id,
        max_solutions=max_solutions,
        max_solution_cost=max_solution_cost,
        move_to_pick_timeout=move_to_pick_timeout,
        move_to_pick_max_path_length=move_to_pick_max_path_length,
        move_to_place_timeout=move_to_place_timeout,
        move_to_place_max_path_length=move_to_place_max_path_length,
        return_home_timeout=return_home_timeout,
        return_home_max_path_length=return_home_max_path_length,
        gripper_close_min=gripper_close_min,
        gripper_close_max=gripper_close_max,
    )

    if dry_run:
        return 0

    vision_scene_before_execution = vision_scene_mtime(vision_scene_file)
    touch_collision_returncode = ensure_touch_collisions()
    if touch_collision_returncode != 0:
        print("[ERROR] MTC execution skipped because touch collision ACM was not applied.", flush=True)
        return touch_collision_returncode

    initial_object_position = scene_manager.get_object(command["pick_object"]).position
    grasp_monitor = GraspMonitor(
        DEFAULT_SCENE_STATE_FILE,
        command["pick_object"],
        initial_object_position,
        min_lift=DEFAULT_GRASP_MIN_LIFT,
        relative_tolerance=DEFAULT_GRASP_RELATIVE_TOLERANCE,
        min_lifted_samples=DEFAULT_GRASP_MIN_LIFTED_SAMPLES,
    )
    grasp_monitor.start()
    try:
        completed = subprocess.run([run_script, *mtc_args], check=False)
    finally:
        grasp_verification = grasp_monitor.stop()

    if completed.returncode == 0:
        relative_deviation = grasp_verification.max_relative_deviation
        relative_deviation_text = (
            "n/a" if relative_deviation is None else f"{relative_deviation:.4f} m"
        )
        if not grasp_verification.success:
            print("\n[RESULT] GRASP FAILED", flush=True)
            print(f" reason             : {grasp_verification.reason}", flush=True)
            print(f" samples            : {grasp_verification.sample_count}", flush=True)
            print(f" lifted samples     : {grasp_verification.lifted_sample_count}", flush=True)
            print(f" maximum lift       : {grasp_verification.max_lift:.4f} m", flush=True)
            print(f" relative deviation : {relative_deviation_text}", flush=True)
            print(
                "[ERROR] Controller execution completed, but physical grasp was not verified. "
                "Command memory was not updated.",
                flush=True,
            )
            return 1
        print("\n[RESULT] GRASP VERIFIED", flush=True)
        print(f" object             : {command['pick_object']}", flush=True)
        print(f" maximum lift       : {grasp_verification.max_lift:.4f} m", flush=True)
        print(f" relative deviation : {relative_deviation_text}", flush=True)
        vision_updated = wait_for_vision_scene_update(
            vision_scene_before_execution,
            vision_scene_file,
            timeout=vision_verification_timeout,
        )
        post_detected_names: set[str] = set()
        if vision_updated:
            try:
                post_detected_names = sync_scene_manager_from_vision(
                    scene_manager,
                    vision_scene_file,
                    max_age=vision_max_age,
                    min_confidence=vision_min_confidence,
                )
            except ValueError as error:
                print(
                    f"[Scene] post-execution vision unavailable; using planned position: {error}",
                    flush=True,
                )
        else:
            print(
                "[Scene] no new YOLO scene after execution; using planned position.",
                flush=True,
            )

        if command["pick_object"] in post_detected_names:
            actual_position = scene_manager.get_object(command["pick_object"]).position
            position_source = "YOLO-observed"
        else:
            pick_object = scene_manager.get_object(command["pick_object"])
            object_size = BLOCK_SIZE if pick_object.size is None else pick_object.size
            actual_position = clamp_center_z(
                scene_manager.resolve_target_position(command),
                float(np.asarray(object_size, dtype=float)[2]),
            )
            position_source = "remembered/planned (camera occluded)"
        scene_manager.apply_pick_place_result(command, actual_position)
        print(
            f"[Scene] updated command memory from {position_source} object position: "
            f"{format_xyz(actual_position)}",
            flush=True,
        )
    else:
        print(f"[ERROR] MTC execution failed with return code {completed.returncode}. Scene memory was not updated.", flush=True)

    return completed.returncode


def run_interactive(
    provider: str,
    run_script: str,
    dry_run: bool = False,
    initial_command: str | None = None,
    planner_id: str = "RRTConnectkConfigDefault",
    max_solutions: int = 2,
    max_solution_cost: float = 60.0,
    move_to_pick_timeout: float = 5.0,
    move_to_pick_max_path_length: float = -1.0,
    move_to_place_timeout: float = 4.0,
    move_to_place_max_path_length: float = -1.0,
    return_home_timeout: float = 1.0,
    return_home_max_path_length: float = -1.0,
    gripper_close_min: float = 0.02,
    gripper_close_max: float = 0.50,
    gripper_close_step: float = 0.08,
    vision_scene_file: str | Path = DEFAULT_VISION_SCENE_FILE,
    vision_max_age: float = DEFAULT_VISION_MAX_AGE,
    vision_min_confidence: float = DEFAULT_VISION_MIN_CONFIDENCE,
    vision_verification_timeout: float = 3.0,
) -> int:
    scene_manager = create_runtime_scene_manager()

    last_returncode = 0
    if initial_command:
        try:
            last_returncode = execute_user_command(
                initial_command,
                scene_manager=scene_manager,
                provider=provider,
                run_script=run_script,
                dry_run=dry_run,
                planner_id=planner_id,
                max_solutions=max_solutions,
                max_solution_cost=max_solution_cost,
                move_to_pick_timeout=move_to_pick_timeout,
                move_to_pick_max_path_length=move_to_pick_max_path_length,
                move_to_place_timeout=move_to_place_timeout,
                move_to_place_max_path_length=move_to_place_max_path_length,
                return_home_timeout=return_home_timeout,
                return_home_max_path_length=return_home_max_path_length,
                gripper_close_min=gripper_close_min,
                gripper_close_max=gripper_close_max,
                gripper_close_step=gripper_close_step,
                vision_scene_file=vision_scene_file,
                vision_max_age=vision_max_age,
                vision_min_confidence=vision_min_confidence,
                vision_verification_timeout=vision_verification_timeout,
            )
            save_scene_memory(scene_manager)
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
                planner_id=planner_id,
                max_solutions=max_solutions,
                max_solution_cost=max_solution_cost,
                move_to_pick_timeout=move_to_pick_timeout,
                move_to_pick_max_path_length=move_to_pick_max_path_length,
                move_to_place_timeout=move_to_place_timeout,
                move_to_place_max_path_length=move_to_place_max_path_length,
                return_home_timeout=return_home_timeout,
                return_home_max_path_length=return_home_max_path_length,
                gripper_close_min=gripper_close_min,
                gripper_close_max=gripper_close_max,
                gripper_close_step=gripper_close_step,
                vision_scene_file=vision_scene_file,
                vision_max_age=vision_max_age,
                vision_min_confidence=vision_min_confidence,
                vision_verification_timeout=vision_verification_timeout,
            )
            save_scene_memory(scene_manager)
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
    parser.add_argument(
        "--planner-id",
        choices=SUPPORTED_PLANNER_IDS,
        default="RRTConnectkConfigDefault",
        help="OMPL planner used for sampled arm motions.",
    )
    parser.add_argument(
        "--max-solutions",
        type=int,
        default=2,
        help="Number of MTC task solutions required before execution.",
    )
    parser.add_argument("--max-solution-cost", type=float, default=60.0)
    parser.add_argument("--move-to-pick-timeout", type=float, default=5.0)
    parser.add_argument("--move-to-pick-max-path-length", type=float, default=-1.0)
    parser.add_argument("--move-to-place-timeout", type=float, default=4.0)
    parser.add_argument("--move-to-place-max-path-length", type=float, default=-1.0)
    parser.add_argument("--return-home-timeout", type=float, default=1.0)
    parser.add_argument("--return-home-max-path-length", type=float, default=-1.0)
    parser.add_argument("--gripper-close-min", type=float, default=0.02)
    parser.add_argument("--gripper-close-max", type=float, default=0.50)
    parser.add_argument("--gripper-close-step", type=float, default=0.08)
    parser.add_argument("--vision-scene-file", default=str(DEFAULT_VISION_SCENE_FILE))
    parser.add_argument("--vision-max-age", type=float, default=DEFAULT_VISION_MAX_AGE)
    parser.add_argument("--vision-min-confidence", type=float, default=DEFAULT_VISION_MIN_CONFIDENCE)
    parser.add_argument("--vision-verification-timeout", type=float, default=3.0)
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
            planner_id=args.planner_id,
            max_solutions=args.max_solutions,
            max_solution_cost=args.max_solution_cost,
            move_to_pick_timeout=args.move_to_pick_timeout,
            move_to_pick_max_path_length=args.move_to_pick_max_path_length,
            move_to_place_timeout=args.move_to_place_timeout,
            move_to_place_max_path_length=args.move_to_place_max_path_length,
            return_home_timeout=args.return_home_timeout,
            return_home_max_path_length=args.return_home_max_path_length,
            gripper_close_min=args.gripper_close_min,
            gripper_close_max=args.gripper_close_max,
            gripper_close_step=args.gripper_close_step,
            vision_scene_file=args.vision_scene_file,
            vision_max_age=args.vision_max_age,
            vision_min_confidence=args.vision_min_confidence,
            vision_verification_timeout=args.vision_verification_timeout,
        )

    user_text = " ".join(args.command)
    if args.once:
        scene_manager = create_runtime_scene_manager()
        try:
            result = execute_user_command(
                user_text,
                scene_manager=scene_manager,
                provider=args.provider,
                run_script=args.run_script,
                dry_run=args.dry_run,
                planner_id=args.planner_id,
                max_solutions=args.max_solutions,
                max_solution_cost=args.max_solution_cost,
                move_to_pick_timeout=args.move_to_pick_timeout,
                move_to_pick_max_path_length=args.move_to_pick_max_path_length,
                move_to_place_timeout=args.move_to_place_timeout,
                move_to_place_max_path_length=args.move_to_place_max_path_length,
                return_home_timeout=args.return_home_timeout,
                return_home_max_path_length=args.return_home_max_path_length,
                gripper_close_min=args.gripper_close_min,
                gripper_close_max=args.gripper_close_max,
                gripper_close_step=args.gripper_close_step,
                vision_scene_file=args.vision_scene_file,
                vision_max_age=args.vision_max_age,
                vision_min_confidence=args.vision_min_confidence,
                vision_verification_timeout=args.vision_verification_timeout,
            )
            save_scene_memory(scene_manager)
            return result
        except ValueError as error:
            print(f"[ERROR] {error}", flush=True)
            return 1

    return run_interactive(
        args.provider,
        args.run_script,
        dry_run=args.dry_run,
        initial_command=user_text,
        planner_id=args.planner_id,
        max_solutions=args.max_solutions,
        max_solution_cost=args.max_solution_cost,
        move_to_pick_timeout=args.move_to_pick_timeout,
        move_to_pick_max_path_length=args.move_to_pick_max_path_length,
        move_to_place_timeout=args.move_to_place_timeout,
        move_to_place_max_path_length=args.move_to_place_max_path_length,
        return_home_timeout=args.return_home_timeout,
        return_home_max_path_length=args.return_home_max_path_length,
        gripper_close_min=args.gripper_close_min,
        gripper_close_max=args.gripper_close_max,
        gripper_close_step=args.gripper_close_step,
        vision_scene_file=args.vision_scene_file,
        vision_max_age=args.vision_max_age,
        vision_min_confidence=args.vision_min_confidence,
        vision_verification_timeout=args.vision_verification_timeout,
    )


if __name__ == "__main__":
    raise SystemExit(main())
