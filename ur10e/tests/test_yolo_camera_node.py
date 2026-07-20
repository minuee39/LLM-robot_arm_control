"""Compatibility launcher for the original YOLO camera command.

The runtime node now lives in apps/yolo_camera_node.py. Keeping this wrapper
avoids breaking the previously used command while keeping runtime code out of
the test suite.
"""

from pathlib import Path
import sys


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))


def main() -> None:
    from apps.yolo_camera_node import main as run_yolo_camera_node

    run_yolo_camera_node()


if __name__ == "__main__":
    main()
