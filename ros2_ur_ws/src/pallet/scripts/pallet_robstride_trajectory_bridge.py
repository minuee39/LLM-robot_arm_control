#!/usr/bin/env python3
"""Execute MoveIt FollowJointTrajectory goals on RobStride motors over CAN."""

from __future__ import annotations

import math
import threading
import time

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from robstride_trajectory_core import (
    ConfigurationError,
    HardwareConfig,
    RobStrideBus,
    TrajectoryError, 
    load_hardware_config,
    validate_trajectory,
)


ACTION_NAME = "/pallet_arm_controller/follow_joint_trajectory"


class PalletRobStrideTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("pallet_robstride_trajectory_bridge")
        default_config = get_package_share_directory("pallet") + "/config/robstride_hardware_legacy_5axis.yaml" # hardware YAML 경로 생성
        self.declare_parameter("config_file", default_config) # ROS2 Parameter 선언. 
        self.declare_parameter("can_interface", "") # Can 인터페이스를 parameter로 
        self.declare_parameter("dry_run", True)
        self.declare_parameter("allow_live_direct_goal", False)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._allow_live_direct_goal = bool(self.get_parameter("allow_live_direct_goal").value) 
        # ros parameter를 가져와 python bool로 변환 / _dry_run처럼 _로 시작하는 이름 -> 클래스 내부에서 사용하는 변수
        self._config: HardwareConfig = load_hardware_config(  # : HardwareConfig -> type hint
            str(self.get_parameter("config_file").value),
            require_commissioned=not self._dry_run,
        )
        interface_override = str(self.get_parameter("can_interface").value) # 별도 CAN interface 지정했는지 
        interface = interface_override or self._config.interface
        # interface overide에서 다른 설정을 헀다면 그 설정을 아니면 self._config.interface에서의 설정을 사용.
        self._lock = threading.Lock() # 관절 위치, 속도 변수 보호용 lock 
        self._goal_lock = threading.Lock() # trajectory goal 실행을 하나로 제한하기 위해 별도 lock. trajectory A가 실행중일 때 B 들어오는 상황 막기
        self._positions = {joint.name: 0.0 for joint in self._config.joints} # 관절 상태 초기화
        self._velocities = {joint.name: 0.0 for joint in self._config.joints} # 관절 속도 초기화
        self._last_feedback = 0.0 # encoder feedback 받은 시간 저장. -> 속도를 계산할 때 사용
        self._bus: RobStrideBus | None = None
        if not self._dry_run:
            try:
                self._bus = RobStrideBus(interface, self._config.host_id) # interface, host_id 전달
                self._read_feedback(require_all=True) # 연결 이후 모터 실제 위치 읽기
            except Exception: # 예외 발생 시
                if self._bus is not None: # can bus 생성되었는지
                    self._bus.close() # Can 연결 닫기
                    self._bus = None # 객체를 비어있는 상태로
                raise # 현재 예외를 상위로 전달. 

        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10) # (Message type, topic, queue depth(저장 최대 메시지 수)
        self._state_timer = self.create_timer(0.05, self._publish_joint_state) # 0.05초(20hz)마다 publish joint state 실행
        self._server = ActionServer( # Action 서버 생성
            self, # action 서버 등록. 
            FollowJointTrajectory, # Action Message type
            ACTION_NAME, 
            execute_callback=self._execute, # Trajectory goal 승인 후 실행할 함수
            goal_callback=self._goal_callback, # 새로운 goal을 받을지 선택. 
            cancel_callback=lambda _: CancelResponse.ACCEPT, # cancel 들어오면 항상 cancel. lamda _ 는 이름없는 짧은 함수. 
        )
        mode = "DRY RUN (CAN disabled)" if self._dry_run else f"LIVE CAN ({interface})"
        self.get_logger().warn(f"RobStride trajectory bridge ready: {mode}") # warning 메시지 출력
        self.get_logger().info(f"Action ready: {ACTION_NAME}")  # action ready 메시지 출력. 

    @property # 메서드를 변수처럼 사용 가능. 
    def _joint_names(self) -> list[str]:
        return [joint.name for joint in self._config.joints] # 설정된 관절 이름을 list로 반환

    def _goal_callback(self, _request) -> GoalResponse: # 새로운 trajectory goal이 들어왔을 떄 실행
        if self._goal_lock.locked():  # trajectory가 이미 실행중이면 reject
            self.get_logger().error("Rejected trajectory: another goal is active")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _read_feedback(self, *, require_all: bool) -> list[float]: # *의 용도는 require_all= 까지 적어라는 뜻. read_feedback(True) 무슨 뜻인지 모르는것 방지
        if self._dry_run:
            with self._lock:
                return [self._positions[name] for name in self._joint_names] #내부 simulation positino 반환
        assert self._bus is not None # Live Can통신이므로 없으면 안됨. assert -> 조건식이 참인지 확인 
        motor_positions = self._bus.read_motor_positions(
            (joint.motor_id for joint in self._config.joints),
            self._config.feedback_timeout_s,
        )
        missing = [joint.name for joint in self._config.joints if joint.motor_id not in motor_positions] # feedback dict에 응답하지 않는 모터 id 찾기. 
        if missing and require_all:
            raise RuntimeError(f"missing motor feedback for {missing}")
        now = time.monotonic() # 프로그램이 실행되는 동안 경과한 시간을 반환. ( 시스템 시간이 변경되어도 항상 증가 )
        with self._lock:
            elapsed = now - self._last_feedback
            for joint in self._config.joints:
                if joint.motor_id in motor_positions:
                    before = self._positions[joint.name]
                    position = joint.joint_from_motor(motor_positions[joint.motor_id]) # motor 값과 moveit 값 변환
                    self._positions[joint.name] = position # 최신 관절 위치 저장 
                    self._velocities[joint.name] = ( # 관절 속도 계산 
                        (position - before) / elapsed if self._last_feedback and elapsed > 0.0 else 0.0
                    )
            if not missing:
                self._last_feedback = now # 모든 motor feedback이 정상적으로 들어왔을 떄 마지막 feedback 시간을 갱신. 
            return [self._positions[name] for name in self._joint_names] # configuration의 순서에 맞는 joint position 갱신

    def _publish_joint_state(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg() # ros2 시간을 message timestamp로 기록
        message.name = self._joint_names # 관절 이름을 넣음. 
        with self._lock:  # 읽는 중에 다른 곳에서 값이 변경되면 서로 다른 시점의 데이터가 섞임. 이 블록이 끝날때까지 다른건 lock을 가짐
            message.position = [self._positions[name] for name in message.name]
            message.velocity = [self._velocities[name] for name in message.name]
        self._joint_state_pub.publish(message) # joint_state로 보냄. 

    @staticmethod # self를 사용할 필요 없어서 static method
    def _result(code: int, message: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    def _safe_stop(self) -> None:
        if self._bus is None:
            return
        for joint in self._config.joints:
            try:
                self._bus.stop(joint.motor_id)
            except Exception as error: # 개별 motor stop 실패 판독 
                self.get_logger().error(f"Failed to stop {joint.name}: {error}") # 오류 기록하고 다음 motor stop

    # 원하는 관절각을 robstride에 전달
    def _command(self, names: list[str], positions: list[float]) -> None:  
        position_by_name = dict(zip(names, positions))
        ordered = [position_by_name[joint.name] for joint in self._config.joints]
        if self._dry_run:
            now = time.monotonic()
            with self._lock:
                elapsed = now - self._last_feedback
                for joint, position in zip(self._config.joints, ordered):
                    before = self._positions[joint.name]
                    self._positions[joint.name] = position
                    self._velocities[joint.name] = (
                        (position - before) / elapsed if self._last_feedback and elapsed > 0.0 else 0.0
                    )
                self._last_feedback = now
        else:
            assert self._bus is not None
            self._bus.command_joint_positions(self._config.joints, ordered) # robstride 모터에 position command 보냄.


    # 현재 Trajectory 실행상태를 moveit에 돌려주기
    def _action_feedback(self, goal_handle, names: list[str], desired: list[float], actual: list[float]) -> None: 
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = names
        feedback.desired = JointTrajectoryPoint(positions=desired)
        feedback.actual = JointTrajectoryPoint(positions=actual)
        feedback.error = JointTrajectoryPoint(positions=[target - observed for target, observed in zip(desired, actual)])
        goal_handle.publish_feedback(feedback) # moveit에 feedback 전달

    # encoder 값을 moveit이 요구한 joint 순서대로 재배열 
    def _actual_in_goal_order(self, names: list[str]) -> list[float]:
        configured = self._read_feedback(require_all=True)
        by_name = dict(zip(self._joint_names, configured))
        return [by_name[name] for name in names]

    # moveit이 보내면 실제로 생성.
    def _execute(self, goal_handle):
        with self._goal_lock:
            names = list(goal_handle.request.trajectory.joint_names)
            try:
                # trajectory 검증
                points = validate_trajectory(names, goal_handle.request.trajectory.points, self._config)
                # 현재 실제 위치 확인
                actual = self._actual_in_goal_order(names)
                # 각 관절 중 오차가 가장 큰 값을 선택 (trajectory 첫번째 점과 현재 로봇 상태의 차이)
                start_error = max(abs(target - observed) for target, observed in zip(points[0][1], actual))
                # 너무 멀면 trajectory 중지
                if start_error > self._config.max_start_error_rad:
                    raise TrajectoryError(
                        f"first point is {start_error:.6f} rad from measured state; limit is {self._config.max_start_error_rad:.6f} rad"
                    )
                if not self._dry_run and not self._allow_live_direct_goal:
                    raise TrajectoryError(
                        "Direct final-target execution on live hardware requires "
                        "allow_live_direct_goal:=true after workspace and collision review"
                    )
                if self._bus is not None:
                    actual_by_name = dict(zip(names, actual))
                    for joint in self._config.joints:
                        self._bus.configure_position_mode(joint, actual_by_name[joint.name])

                # PP(position profile) mode: send one final position command
                # per joint. The motor's VEL_MAX / ACC_SET generate motion.
                # This intentionally does not stream MoveIt waypoints.
                # Intermediate path collision guarantees do not carry over.
                final_target = points[-1][1]
                self._command(names, final_target)
                started = time.monotonic()
                # Allow at least the planned trajectory duration plus settle time;
                # check feedback independently of how often commands are issued.
                deadline = started + points[-1][0] + self._config.settle_timeout_s
                max_error = math.inf
                while time.monotonic() < deadline:
                    if goal_handle.is_cancel_requested:
                        self._safe_stop()
                        goal_handle.canceled()
                        return self._result(
                            FollowJointTrajectory.Result.SUCCESSFUL,
                            "Canceled; motors stopped",
                        )
                    actual = self._actual_in_goal_order(names)
                    max_error = max(abs(target - observed) for target, observed in zip(final_target, actual))
                    self._action_feedback(goal_handle, names, final_target, actual)
                    elapsed = time.monotonic() - started
                    if max_error <= self._config.goal_tolerance_rad:
                        goal_handle.succeed()
                        return self._result(FollowJointTrajectory.Result.SUCCESSFUL, f"Goal reached; max error={max_error:.6f} rad")
                    time.sleep(self._config.feedback_period_s)
                raise RuntimeError(f"goal tolerance violated; max error={max_error:.6f} rad")
            except TrajectoryError as error:
                goal_handle.abort()
                return self._result(FollowJointTrajectory.Result.INVALID_GOAL, str(error))
            except Exception as error:
                self._safe_stop()
                self.get_logger().error(f"Trajectory aborted and motors stopped: {error}")
                goal_handle.abort()
                return self._result(FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED, str(error))

    def destroy_node(self) -> None:
        self._safe_stop()
        self._server.destroy()
        if self._bus is not None:
            self._bus.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PalletRobStrideTrajectoryBridge()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except ConfigurationError as error:
        print(f"Unsafe RobStride configuration: {error}")
        raise SystemExit(2) from error
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
