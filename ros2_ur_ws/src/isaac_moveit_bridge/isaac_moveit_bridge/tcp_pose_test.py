import argparse
from typing import Optional

from geometry_msgs.msg import Pose

from isaac_moveit_bridge.tcp_pose import tcp_target_to_tool0_target


def make_pose(args: argparse.Namespace) -> Pose:    
    pose = Pose()
    pose.position.x = args.x
    pose.position.y = args.y
    pose.position.z = args.z
    pose.orientation.x = args.qx
    pose.orientation.y = args.qy
    pose.orientation.z = args.qz
    pose.orientation.w = args.qw
    return pose


def print_pose(label: str, pose: Pose) -> None:
    print(label)
    print(
        "  position: "
        f"x={pose.position.x:.6f}, "
        f"y={pose.position.y:.6f}, "
        f"z={pose.position.z:.6f}"
    )
    print(
        "  orientation: "
        f"x={pose.orientation.x:.6f}, "
        f"y={pose.orientation.y:.6f}, "
        f"z={pose.orientation.z:.6f}, "
        f"w={pose.orientation.w:.6f}"
    )


def parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a desired TCP target pose into a MoveIt tool0 target pose."
    )
    parser.add_argument("--x", type=float, default=0.5, help="TCP target x in base_link.")
    parser.add_argument("--y", type=float, default=0.0, help="TCP target y in base_link.")
    parser.add_argument("--z", type=float, default=0.3, help="TCP target z in base_link.")
    parser.add_argument("--qx", type=float, default=0.0, help="TCP target quaternion x.")
    parser.add_argument("--qy", type=float, default=0.0, help="TCP target quaternion y.")
    parser.add_argument("--qz", type=float, default=0.0, help="TCP target quaternion z.")
    parser.add_argument("--qw", type=float, default=1.0, help="TCP target quaternion w.")
    return parser.parse_args(args)


def main(args: Optional[list[str]] = None) -> None:
    parsed_args = parse_args(args)
    tcp_target = make_pose(parsed_args)
    tool0_target = tcp_target_to_tool0_target(tcp_target)

    print_pose("TCP target pose in base_link:", tcp_target)
    print_pose("MoveIt tool0 target pose in base_link:", tool0_target)


if __name__ == "__main__":
    main()
