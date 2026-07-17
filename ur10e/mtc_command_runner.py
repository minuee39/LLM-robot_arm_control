import argparse
import os
import subprocess
from pathlib import Path

import numpy as np

from command_parser import parse_user_command_with_memory, validate_command
from llm_to_json import parse_user_command_with_llm
from scene_config import MTC_OBJECT_SIZE
from scene_manager import SceneManager


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_RUN_SCRIPT = PROJECT_DIR / "scripts" / "run_mtc_isaac_pick_place.sh"


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


def command_to_mtc_args(command: dict, scene_manager: SceneManager) -> list[str]:
    pick_object = scene_manager.get_object(command["pick_object"])
    target_position = scene_manager.resolve_target_position(command)

    object_height = float(MTC_OBJECT_SIZE[2])
    object_radius = float(max(MTC_OBJECT_SIZE[0], MTC_OBJECT_SIZE[1]) / 2.0)
    object_position = clamp_center_z(pick_object.position, object_height)
    place_position = clamp_center_z(target_position, object_height)

    return [
        f"object_x:={object_position[0]:.6f}",
        f"object_y:={object_position[1]:.6f}",
        f"object_z:={object_position[2]:.6f}",
        f"object_height:={object_height:.6f}",
        f"object_radius:={object_radius:.6f}",
        f"place_x:={place_position[0]:.6f}",
        f"place_y:={place_position[1]:.6f}",
        f"place_z:={place_position[2]:.6f}",
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


def execute_user_command(
    user_text: str,
    scene_manager: SceneManager,
    provider: str,
    run_script: str,
    dry_run: bool = False,
) -> int:
    command, mtc_args = build_mtc_invocation(user_text, provider=provider, scene_manager=scene_manager)

    print("[Command]", command, flush=True)
    print("[MTC args]", " ".join(mtc_args), flush=True)

    if dry_run:
        return 0

    completed = subprocess.run([run_script, *mtc_args], check=False)
    if completed.returncode == 0:
        final_position = clamp_center_z(scene_manager.resolve_target_position(command), float(MTC_OBJECT_SIZE[2]))
        scene_manager.apply_pick_place_result(command, final_position)
        print("[Scene] updated command memory and object position.", flush=True)

    return completed.returncode


def run_interactive(provider: str, run_script: str, dry_run: bool = False) -> int:
    scene_manager = SceneManager.from_defaults()

    print("[READY] Enter natural-language pick/place commands.", flush=True)
    print("[READY] Type 'exit', 'quit', or '종료' to stop.", flush=True)

    last_returncode = 0
    while True:
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
    args = parser.parse_args()

    if not args.command:
        return run_interactive(args.provider, args.run_script, dry_run=args.dry_run)

    user_text = " ".join(args.command)
    scene_manager = SceneManager.from_defaults()
    return execute_user_command(
        user_text,
        scene_manager=scene_manager,
        provider=args.provider,
        run_script=args.run_script,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    raise SystemExit(main())
