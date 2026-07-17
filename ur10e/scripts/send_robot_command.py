#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from command_server import DEFAULT_COMMAND_HOST, DEFAULT_COMMAND_PORT, send_command


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a natural-language robot command to Isaac Sim.")
    parser.add_argument("command", nargs="*", help="Command text to send. If omitted, starts interactive mode.")
    parser.add_argument("--host", default=DEFAULT_COMMAND_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_COMMAND_PORT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command:
        command = " ".join(args.command)
        print(send_command(command, args.host, args.port))
        return 0

    print("로봇 명령을 입력하세요. 종료하려면 exit 또는 quit.")
    while True:
        try:
            command = input("> ").strip()
        except EOFError:
            return 0

        if not command:
            continue

        print(send_command(command, args.host, args.port))
        if command.lower() in {"exit", "quit", "종료"}:
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
