import argparse
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateLogger(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("joint_state_logger")
        self.topic = topic
        self.row_count = 0

        self.subscription = self.create_subscription(JointState, topic, self.on_joint_state, 10)
        self.get_logger().info(f"Subscribing: {topic}")

    def on_joint_state(self, msg: JointState) -> None:
        velocities = list(msg.velocity)
        efforts = list(msg.effort)
        lines = [
            f"JointState #{self.row_count + 1} "
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        ]

        for index, name in enumerate(msg.name):
            position = msg.position[index] if index < len(msg.position) else ""
            velocity = velocities[index] if index < len(velocities) else ""
            effort = efforts[index] if index < len(efforts) else ""
            lines.append(f"  {name}: position={position} velocity={velocity} effort={effort}")

        self.row_count += 1
        self.get_logger().info("\n" + "\n".join(lines))


def parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print sensor_msgs/JointState messages to the terminal.")
    parser.add_argument(
        "--topic",
        default="/isaac_joint_states",
        help="JointState topic to subscribe to.",
    )
    return parser.parse_args(args)


def main(args: Optional[list[str]] = None) -> None:
    parsed_args = parse_args(args)
    rclpy.init(args=args)
    node = JointStateLogger(topic=parsed_args.topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
