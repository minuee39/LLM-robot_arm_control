#!/usr/bin/python3
"""Bounded ROS safety stop used when a supervised motion monitor exits."""
import json
import math
import time
import uuid


def main():
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('pallet_monitor_supervisor_stop')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(String, '/pallet/safety_events', qos)
    acknowledged = False
    event_id = f'monitor-supervisor-{uuid.uuid4().hex}'

    def status(msg):
        nonlocal acknowledged
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        # An existing fresh latch also prevents all new goals.
        if not isinstance(payload, dict):
            return
        stamp = payload.get('published_at_unix')
        if (payload.get('state') == 'STOPPED'
                and payload.get('safety_cancel_enabled') is True
                and isinstance(stamp, (int, float)) and math.isfinite(stamp)
                and 0 <= time.time() - stamp < 2.0):
            acknowledged = True

    sub = node.create_subscription(String, '/pallet/safety_status', status, qos)
    deadline = time.monotonic() + 2.0
    try:
        while rclpy.ok() and time.monotonic() < deadline and not acknowledged:
            pub.publish(String(data=json.dumps({
                'event_id': event_id, 'code': 'MONITOR_LOST', 'severity': 'critical',
                'source': 'palletizing_job', 'frame': 'base_footprint',
                'published_at_unix': time.time(),
            })))
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps({'monitor_safety_stop_acknowledged': acknowledged}), flush=True)
    return 0 if acknowledged else 1


if __name__ == '__main__':
    raise SystemExit(main())
