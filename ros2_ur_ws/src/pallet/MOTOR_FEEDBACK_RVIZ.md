# 모터 위치 → RViz 읽기 전용 연동

실행 파일: scripts/pallet_motor_joint_state.py. 변환/통신: scripts/motor_joint_state_core.py.

모터 값을 q=direction*(motor-zero_rad)/ratio로 변환해 /joint_states에 발행한다. 모델은 현재 재정의한 영점과 J4 반전이 적용된 pallet_with_gripper.urdf다. 현재 hand_guiding.yaml의 방향/영점/전달비는 미확정이므로 manual/can 모드는 보정 완료 전 시작을 거부한다. 중력보상 commissioned나 토크 설정은 읽기 전용 표시의 요구사항이 아니다.

기존 joint_state_publisher_gui, trajectory bridge 등 /joint_states 발행자는 먼저 종료한다. 아래 launch는 기본 슬라이더를 실행하지 않는다. 다른 ROS_DOMAIN_ID를 사용하면 모든 관련 터미널에 동일하게 설정한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/minwoo/Development/LLM/ros2_ur_ws/install/setup.bash
# 보정된 팔 6축의 CAN 읽기 연동:
ros2 launch pallet display_motor_feedback.launch.py mode:=can
# 보정된 모터 위치를 직접 입력:
ros2 launch pallet display_motor_feedback.launch.py mode:=manual
```

manual 입력은 /pallet_motor_positions의 std_msgs/msg/Float64MultiArray다. 기본 데이터 길이는 6, 순서는 설정의 J1..J6에 대응하는 ID 1..6, 단위는 모터 출력축 rad다. 실제 읽은 값으로 data를 채운다. 예시 명령의 숫자는 반드시 측정값으로 바꾼다:

```bash
ros2 topic pub --once /pallet_motor_positions std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

그리퍼 보정까지 끝나면 include_gripper:=true로 실행하고 7개 값을 입력한다. 그리퍼의 다회전 값은 그대로 손가락 g_r에 넣지 않으며 원점과 기구학 전달비가 필요하다. 기본 그리퍼는 표시상 q=0(열림)으로 유지된다. g_l=-g_r로 발행한다.

보정 전에도 소프트웨어 연결을 확인하려면 URDF 관절각 직접 입력 모드를 사용한다. 이 모드는 모터 좌표를 변환하는 실물 보정 시험이 아니다:

```bash
ros2 launch pallet display_motor_feedback.launch.py mode:=joint_preview
# 별도 터미널, 동일한 ROS 환경:
ros2 topic pub --once /pallet_preview_positions std_msgs/msg/Float64MultiArray '{data: [0.0, 0.1, -0.2, 0.3, 0.0, 0.0, -0.05]}'
```

CAN 모드는 통신 type 17, mechPos(0x7019) 읽기 프레임만 보낸다. enable/stop/zero/mode/current 명령은 제공하지 않는다. 기본 10Hz로 전체 응답을 받은 경우에만 새 상태를 발행한다. 누락/비유한/URDF 범위 초과는 거부하고 /pallet_motor_state/status로 알린다. 0.5초 이상 유효 입력이 없으면 STALE을 표시하며 이전 값을 새 타임스탬프로 다시 보내지 않는다. RViz에는 마지막 자세가 남을 수 있으므로 status를 함께 확인한다. 모터 회전 수의 전원 후 복원이나 wrap 자동 보정은 수행하지 않는다. joint_preview는 유한한 7개 URDF 각도만 검사하며 실제 충돌/관절 한계 검증 도구가 아니다.

검증: 변환/방향/전달비, 보정 누락, 팔 단독, 읽기 프레임과 host 필터 테스트. ROS 토픽 입력→JointState 출력 및 mimic 연동 확인. 실제 CAN 연결·모터 응답은 검증하지 않았다.

## RobStride CH340 AT 시리얼 어댑터

Motor Studio 포트를 먼저 닫는다. raw 위치 읽기(보정 불필요):

```bash
python3 /home/minwoo/Development/LLM/robstride_read_angles.py --transport serial --port /dev/ttyUSB0 --motor 1 2 3 4 5 6 7 --once
```

영점/방향/전달비 보정 후 RViz 연동:

```bash
ros2 launch pallet display_motor_feedback.launch.py mode:=can transport:=serial serial_port:=/dev/ttyUSB0
```

시리얼 기본 속도는 921600 baud다. CAN 버스 속도와 별개이며 어댑터 설정을 임의 변경하지 않는다. AT + big-endian ((CAN ID<<3)|4) + DLC + payload + CRLF 프레임이다. pyserial을 사용하고 exclusive open을 요청한다. type 17 읽기 이외의 송신은 거부한다. 통신 분할/노이즈/꼬리 손상/제어명령 차단 및 기존 읽기 로직 8개 테스트 통과. 실제 모터 응답은 아직 검증하지 않았다. 중력보상 출력 코드는 여전히 SocketCAN 기반이다.

사용자 실기 결과: 최초 일괄 요청에서 3/4/5가 누락됐지만 개별 재요청으로 ID 1..7 모두 위치 응답을 받았다. 이를 반영해 시리얼 transport를 응답 대기 후 다음 모터를 조회하는 순차 방식으로 변경했다. 전체 주기 timeout을 공유하며, 완료되지 않은 표본은 실시간 RViz로 보내지 않는다. 변경한 순차 방식의 실기 재확인은 남아 있다. 보고된 위치값은 motor_position_observation_20261022.json에 관측으로만 저장했으며 영점으로 적용하지 않았다.
