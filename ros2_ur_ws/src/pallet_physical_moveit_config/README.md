# 최신 6축 Pallet MoveIt 설정

종료 오류 후속: 2026-10-27 로컬 플러그인 코드 수명 보정 적용.
현재 상태와 적용 범위는 [SHUTDOWN_FIX.md](SHUTDOWN_FIX.md) 참고.
아래 초기 SIGSEGV 기록은 수정 전 이력이다.

기준: pallet/urdf/pallet_with_gripper.urdf, 팔 J1~J6, 그리퍼 g_r (g_l mimic).
기존 pallet_moveit_config는 5축 시뮬레이션용으로 보존한다.

## 가상 실행

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pallet_physical_moveit_config demo.launch.py
```

이 demo는 GenericSystem을 사용하며 CAN/USB 모터 명령을 전송하지 않는다.
그리퍼 제어기도 가상 FollowJointTrajectory이다. 실물 드라이버는 미연결이다.
기본 0 자세는 가상 초기값이며 안전한 실물 home이나 무충돌 자세를 보장하지 않는다.

## 아직 필요한 검증

- 계획 tip은 CAD의 gripper 프레임. 실제 파지 TCP 오프셋은 실측 후 추가한다.
- 속도/가속도는 기존 브리지의 수치에 계획 배율 0.1을 적용한 가상 설정이다.
  실물 위치 유지, 전류 제한, 추종 성능이 검증됐다는 뜻이 아니다.
- 자기 충돌은 인접 링크만 제외했다. 나머지 간섭은 실제 모델 기준으로 조사한다.
- 피드백과 영점은 기존 pallet/config/hand_guiding.yaml 및 motor_zero_reference.json을 보존한다.
  방향/기구비 검증과 실물 자세 대조가 남아 있다.
- USB 직렬 위치제어 action bridge와 그리퍼 보정은 별도 단계이다.
- 기존 robstride_demo.launch.py를 이 6축 모델의 실물 실행 명령으로 사용하지 않는다.

## 2026-10-27 검증 결과

- ROS 2 Jazzy 환경에서 pallet 및 본 패키지 colcon build 통과.
- 모델 참조·6축 chain·컨트롤러·관절 제한 일치 pytest 2개 통과.
- MoveItConfigsBuilder의 Xacro 전개에서 J1~J6, g_r command/state interface 확인.
- demo.launch.py --show-args 통과.
- GUI 없는 모의 기동에서 팔/그리퍼 컨트롤러와 joint_state_broadcaster 활성화 확인.
- /check_state_validity: 0 자세 valid=true, contacts=[] (등록된 테이블/외부 장애물 없음).
- /plan_kinematic_path: J1=0.02rad, 나머지=0 목표 계획 성공(error_code=1), 궤적 14점.
- 실제 trajectory 실행과 실물 모터 구동은 수행하지 않음.
- 종료 SIGINT 처리에서 설치된 move_group의 CallbackGroup 소멸 중 SIGSEGV 관측.
  계획 요청 완료 후 종료 단계의 오류이며 원인 미확정. 정상 종료 검증은 남아 있음.
- 카메라/Octomap 센서는 미설정이므로 해당 센서 경고/오류 로그가 나타남.
- 샌드박스 검증에서는 ROS_HOME=/tmp/pallet_physical_ros 및
  ROS_LOG_DIR=/tmp/pallet_physical_moveit_logs 사용.
- 상세 로컬 검증 로그: /tmp/pallet_physical_plan_smoke.log (임시 파일).

## 모의 실행·취소 검증 도구

```bash
source /opt/ros/jazzy/setup.bash
source /home/minwoo/Development/LLM/ros2_ur_ws/install/setup.bash
ros2 run pallet_physical_moveit_config verify_mock_execution.py
```

도구는 ROS domain 87과 localhost discovery에서 자체 GenericSystem demo를 기동한다.
다른 domain 87 작업을 실행하지 않는 상태에서 사용한다. 실행 중인 실물 제어기에
연결하는 도구가 아니며 CAN/USB 장치를 열지 않는다.

확인 순서: MoveIt 계획 → ExecuteTrajectory → joint_states 목표 도달 →
g_r=-0.02rad 가상 이동 → 팔 FollowJointTrajectory 취소 → 0.5초 위치 유지.
모든 대기에 timeout을 적용한다. g_l은 독립 피드백 관절이 아닌 URDF mimic이다.
취소 검증은 팔 컨트롤러 action 대상으로, MoveIt action의 취소 전파 검증과 구분한다.
종료 오류도 result.json에 기록하고 비정상 종료 코드로 보고한다.

2026-10-27 결과:

- 계획·팔 실행·그리퍼 실행·컨트롤러 취소/유지 모두 2회 통과.
- 1회차 종료 정상: /tmp/pallet_mock_verify_6zartfuk/result.json
- 2회차 종료 SIGSEGV 재현: /tmp/pallet_mock_verify_wcnyw1gl/result.json
- 설치된 MoveIt 2.12.4의 TrajectoryExecutionManager/CallbackGroup 소멸 경로에서
  간헐적으로 발생한다. 동작 검증 성공과 정상 종료 실패를 구분한다.
- 같은 Jazzy 2.12.4의 종료 오류가 upstream에도 보고돼 있다:
  https://github.com/moveit/moveit2/issues/3680
  동일 원인이라는 확정은 아니며, 시스템 패키지 교체나 예외 숨김은 하지 않았다.
- pytest 2개, Python 문법 검사, 패키지 재빌드 통과.
- 실물 위치 유지·부하 추종·정지 성능을 입증한 결과는 아니다.

다음 실물 준비: 기존 읽기 전용 피드백과 최신 MoveIt 모델 자세 대조,
TCP 실측, USB 직렬 위치제어기 구현 및 축별 검증.

## 실물 피드백 표시와 계획 전용 launch

기존 모의 demo를 종료한 뒤 다음을 실행한다. 다른 joint_states 발행기와 동시에
사용하지 않는다. hand_guiding.yaml의 방향·영점 매핑을 기존 값 그대로 사용한다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 장치 없이 수동 모터각 입력을 기다리는 기본 모드
ros2 launch pallet_physical_moveit_config feedback.launch.py
```

실물 USB 직렬 피드백을 읽을 때 (연결된 실제 포트 확인 후 지정):

```bash
ros2 launch pallet_physical_moveit_config feedback.launch.py \
  mode:=can transport:=serial serial_port:=/dev/ttyUSB0 serial_baud:=921600
```

mode=can은 기존 피드백 노드의 하드웨어 읽기 모드 이름이며,
transport=serial이면 USB 직렬 AT 어댑터를 사용한다.
읽기 요청만 전송하며 enable/목표 위치/영점 쓰기는 하지 않는다.
controller_manager와 trajectory bridge는 기동하지 않는다.
allow_trajectory_execution=false를 고정하고 ExecuteTrajectory capability를 제거한다.
RViz의 계획은 가능하지만 실행용 launch가 아니다.

- 그리퍼 ID7은 미보정이므로 읽지 않는다. 표시된 개구 상태는 실제 측정값이 아니다.
- 원시 모터각 → direction*(raw-zero)/ratio → J1~J6 → joint_states → MoveIt 상태.
- 입력은 rad. 기본 수동 입력 topic은 /pallet_motor_positions, 순서는 ID1~6이다.
- 상태 topic /pallet_motor_state/status의 STALE은 유효한 새 피드백이 없다는 뜻이다.
  마지막 화면 자세를 최신 실측으로 해석하지 않는다. 오래된 상태에서 계획을 실행할 수 없다.
- TCP는 아직 CAD gripper 프레임이다. 실측 파지 중심 보정은 남아 있다.
- 실제 실행 경로와 모터 정지 성능은 이 launch에서 검증하지 않는다.

2026-10-27 피드백 경로 검증:

- 별도 ROS domain 88에서 하드웨어 없이 manual 모드 기동.
- 원시 모터각 [0.01,0.02,0.03,0.04,0.05,0.06] rad 입력.
- joint_states가 [-0.01,-0.02,0.03,0.04,-0.05,-0.06] rad로 변환됨.
- /get_planning_scene의 로봇 상태가 위 6축 값과 일치.
- ExecuteTrajectory action 없음, controller_manager 없음 확인.
- 입력 중단 후 STALE 상태 감지 확인.
- 모델 및 직렬 읽기 전용 테스트 8개, 문법 검사, 빌드 통과.
- /dev/ttyUSB0와 /dev/serial/by-id가 검증 환경에 없어 실물 읽기는 미실시.
- 로그: /tmp/pallet_feedback_validation.log. 기존 MoveIt 종료 문제는 별도 미해결 항목이다.

실물 검증 순서와 명령은 [HARDWARE_VALIDATION.md](HARDWARE_VALIDATION.md)를 참고한다.

4단계 CAD 기준 치수와 TCP 측정 절차: [TCP_MEASUREMENT.md](TCP_MEASUREMENT.md).

## 그리퍼 실측 보정 (2026-10-27)

사용자가 Motor 앱의 실제 현재각으로 확인: -12rad=0mm, 0rad=20mm, +12rad=40mm.
config/gripper_calibration.json에 원시각 기준으로 기록했다. 중간값은 선형 보간이며
측정 범위 밖은 거부한다. 기존 URDF g_r 값이나 ID7 영점/모터 제한은 변경하지 않았다.

```bash
# 10mm 개구 목표의 변환 결과: -6rad. 계산만 하며 움직이지 않는다.
ros2 run pallet_physical_moveit_config gripper_calibration.py --opening-mm 10
# 현재각 0rad의 자유공간 개구폭 추정 결과: 20mm
ros2 run pallet_physical_moveit_config gripper_calibration.py --motor-rad 0
```

파지 접촉 후의 실제 개구폭/파지 성공은 모터각만으로 확정하지 않는다.
-12rad는 닫힘 기준이며 물체가 있을 때 강제로 도달시킬 목표가 아니다.
현재 영점이 바뀌면 대응표도 재보정한다. 실물 전류/속도/정지 성능은 미검증이다.
다음은 실제 닫힘 상태에서 손끝 중심의 TCP 측정이다.

팔 단축 PP 드라이런 도구: pallet/SINGLE_AXIS_POSITION.md. 실물 제한값과 PP watchdog 확인 전 LIVE 차단.

6축 MoveIt 실행 브리지와 드라이런 실행 방법: [SERIAL_BRIDGE.md](SERIAL_BRIDGE.md).
