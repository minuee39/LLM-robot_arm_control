#!/usr/bin/env python3
"""Visual-only adaptive gripper joint states, with no hardware transport."""
import argparse
import json
from pathlib import Path

from adaptive_gripper_core import AdaptiveGripper


def create_slider_window(model, initial_angle, on_angle):
    from python_qt_binding.QtCore import Qt
    from python_qt_binding.QtWidgets import (
        QLabel, QPushButton, QSlider, QVBoxLayout, QHBoxLayout, QWidget)

    class SliderWindow(QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle('Adaptive gripper preview')
            self.setMinimumWidth(430)
            layout = QVBoxLayout(self)
            layout.addWidget(QLabel('Motor angle (rad)'))
            self.slider = QSlider(Qt.Horizontal)
            self.slider.setRange(round(model.angles[0]*100), round(model.angles[-1]*100))
            self.slider.setSingleStep(1)
            self.slider.setPageStep(100)
            self.slider.setTickPosition(QSlider.TicksBelow)
            self.slider.setTickInterval(600)
            layout.addWidget(self.slider)
            self.values = QLabel()
            layout.addWidget(self.values)
            buttons = QHBoxLayout()
            for angle in (model.angles[0], 0., model.angles[-1]):
                button = QPushButton(f'{angle:+g} rad')
                button.clicked.connect(lambda checked=False, value=angle: self.slider.setValue(round(value*100)))
                buttons.addWidget(button)
            layout.addLayout(buttons)
            layout.addWidget(QLabel('Visual preview only'))
            self.show_pose(model.pose(initial_angle))
            self.slider.valueChanged.connect(lambda value: on_angle(value/100.))

        def show_pose(self, pose):
            self.slider.blockSignals(True)
            self.slider.setValue(round(pose['motor_rad']*100))
            self.slider.blockSignals(False)
            self.values.setText(
                f"Motor: {pose['motor_rad']:+.2f} rad\n"
                f"Opening: {pose['opening_mm']/10:.2f} cm\n"
                f"Nut depth: {pose['nut_depth_mm']/10:.2f} cm")

    return SliderWindow()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path)
    parser.add_argument('--motor-rad', type=float, default=0.0)
    parser.add_argument('--offline', action='store_true')
    parser.add_argument('--gui', action='store_true', help='Show the coupled motor-angle slider')
    args, ros_args = parser.parse_known_args()
    if args.geometry is None:
        from ament_index_python.packages import get_package_share_directory
        args.geometry = Path(get_package_share_directory('pallet'))/'config/adaptive_gripper/geometry.json'
    model = AdaptiveGripper.load(args.geometry)
    pose = model.pose(args.motor_rad)
    if args.offline:
        print(json.dumps({k: v for k, v in pose.items() if k != 'sides'}, indent=2))
        return
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64

    class Preview(Node):
        def __init__(self):
            super().__init__('adaptive_gripper_preview')
            self.pose = pose
            self.window = None
            self.publisher = self.create_publisher(JointState, '/adaptive_gripper_preview/joint_states', 10)
            self.subscription = self.create_subscription(Float64, '/adaptive_gripper_preview/motor_angle', self.update_pose, 10)
            self.timer = self.create_timer(0.05, self.publish_pose)
            self.get_logger().info('Visual preview only; motor_angle range [-12,12] rad. No actuator interface.')

        def update_pose(self, message):
            try:
                next_pose = model.pose(message.data)
            except ValueError as error:
                self.get_logger().error(f'Rejected preview command; retaining previous pose: {error}')
                return
            self.pose = next_pose
            if self.window is not None:
                self.window.show_pose(next_pose)

        def publish_pose(self):
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(self.pose['joints'])
            message.position = list(self.pose['joints'].values())
            self.publisher.publish(message)

    rclpy.init(args=ros_args)
    node = Preview()
    try:
        if args.gui:
            from python_qt_binding.QtCore import QTimer
            from python_qt_binding.QtWidgets import QApplication
            app = QApplication([])
            import signal
            signal.signal(signal.SIGINT, lambda *_: app.quit())
            signal.signal(signal.SIGTERM, lambda *_: app.quit())

            def change_angle(value):
                message = Float64()
                message.data = value
                node.update_pose(message)

            node.window = create_slider_window(model, args.motor_rad, change_angle)
            node.window.show()
            timer = QTimer()

            def tick():
                if rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0)
                else:
                    app.quit()

            timer.timeout.connect(tick)
            timer.start(10)
            try:
                app.exec_()
            finally:
                timer.stop()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
