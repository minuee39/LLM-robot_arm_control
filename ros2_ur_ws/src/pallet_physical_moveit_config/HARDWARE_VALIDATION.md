# Pallet 실물 검증 체크리스트와 코드

작성일: 2026-10-27. 범위: 중력보상·강화학습 제외, 위치제어 기반 pick and place 준비.
기준 workspace: /home/minwoo/Development/LLM/ros2_ur_ws
실물 결과는 아래 기록란을 채우기 전까지 미확인이다.

## 현재 상태

- 완료: 최신 6축 모델, 가상 팔/그리퍼 실행과 팔 컨트롤러 취소 검증.
- 완료: 수동 각도 입력이 MoveIt 현재 상태로 반영되는 소프트웨어 검증.
- 기존 기록: 2026-10-22 hand_guiding.yaml의 실물/RViz 방향·대략각 일치. 정밀 계측은 아님.
- 사용자 확인 완료: ID1~6 응답, 실물/RViz 축·방향·각도 일치, 약10Hz 및 OK 상태.
- 진행 중: 실제 치수/TCP 확인. 기준은 닫힌 상태 양쪽 손끝 중심(TCP_MEASUREMENT.md).
- 사용자 확인: ID7 실제각 -12/0/+12rad에서 개구폭 0/20/40mm.
- 완료: 그리퍼 원시각/개구폭 변환과 범위 검사(구동 없음).
- 남음: TCP 실측, 그리퍼 중간/양방향 재현성 및 위치제어 실물 검증.
- 미구현: USB 직렬 팔 위치제어 action bridge, adaptive 그리퍼 실물 제어/변환.
- 후속 수정: MoveIt 2.12.4 종료 SIGSEGV에 로컬 코드 수명 보정 적용. SHUTDOWN_FIX.md 참고.

## 1. 준비 및 읽기 전용 통신

팔을 지지하고 기존 제어 프로그램 및 demo/display launch를 종료한다.
동일 USB 포트는 한 프로그램만 사용한다. 아래 읽기는 모터 enable·영점 변경을 하지 않는다.
각 터미널에서:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

장치 확인 후 예제의 /dev/ttyUSB0를 실제 포트로 바꾼다.

```bash
ls -l /dev/serial/by-id/ /dev/ttyUSB*
python3 /home/minwoo/Development/LLM/robstride_read_angles.py \
  --transport serial --port /dev/ttyUSB0 --baud 921600 \
  --motor 1 2 3 4 5 6 --once
```

합격: 모든 ID가 유한한 각도 응답. NO RESPONSE/timeout이면 다음 단계 진행 전 원인 확인.
이 명령은 원시 모터각을 출력하며 URDF 방향 변환이 적용된 관절각과 구분한다.

## 2. 실제 팔 → MoveIt 상태

위 단독 읽기가 끝난 뒤 터미널 A:

```bash
ros2 launch pallet_physical_moveit_config feedback.launch.py \
  mode:=can transport:=serial serial_port:=/dev/ttyUSB0 serial_baud:=921600
```

터미널 B에서 각각 실행해 확인:

```bash
ros2 topic echo /pallet_motor_state/status
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
```

필요하면 별도 터미널에서 기록 (출력 경로는 새 폴더여야 함):

```bash
ros2 bag record -o /tmp/pallet_feedback_check_01 \
  /joint_states /pallet_motor_state/status /tf /tf_static
```

- [ ] J1~J6이 모두 반영되고 실제 축과 일치한다.
- [ ] 지지 상태에서 허용된 수동 자세 변화에 대해 회전 방향이 일치한다.
- [ ] 몇 가지 자세에서 실물과 RViz의 대략적인 모양/각도가 일치한다.
- [ ] 각도 점프와 반복 timeout/STALE이 없다. 발생하면 시간/축/자세를 기록한다.
- [ ] 알려진 기준 자세에서 각도 오차를 측정·기록한다. 수치 합격 한계는 작업 정밀도에 맞춰 결정한다.

주기 기본값은 10Hz이며 실제 수신률은 측정해 기록한다. 달성했다고 가정하지 않는다.
그리퍼 g_r/g_l=0은 표시용 값이며 ID7 실측이 아니다.
팔을 움직이기 위해 위치 유지가 풀린다고 가정하거나 강제로 역구동하지 않는다.
STALE은 피드백 문제이며 실제 하드웨어 정지 시험을 대신하지 않는다.

## 3. 모델 치수 및 TCP

- [ ] CAD 회전축 사이 거리/오프셋과 조립 상태에 큰 차이가 없는지 확인.
- [ ] gripper 프레임 기준 실제 파지 중심 위치·방향을 측정.
- [ ] 다른 자세에서도 예측 손끝 위치와 실측 위치를 비교.

현재 TCP는 CAD gripper 프레임이다. 실측 TCP 설정 코드는 아직 추가하지 않았다.
그리퍼 개방/접촉에 따라 파지 중심이 변하면 해당 상태도 함께 기록한다.

## 4. 그리퍼 측정

사용자 제공 과거 기록: 리드 2mm/rev, -42.07~500도, 약3.012mm 너트 변위,
최대 개구폭67mm 및 특정 구간 변화량18mm. 현 영점/전체 스트로크와 일치하는지는 미확인.

- [ ] 모터 출력각과 스크루가 1:1인지 확인.
- [ ] 현재 ID7 영점과 과거 각도 기준의 관계 확인. 영점을 덮어쓰지 않는다.
- [ ] 개방·폐쇄 끝점에서 현재 모터각, 너트 상대 위치, 개구폭 측정.
- [ ] 중간 3~5곳 이상 측정. 가능하면 개방/폐쇄 양방향으로 측정해 유격 확인.
- [ ] 스크루 방향, 링크 접촉/간섭, 케이블 제한 기록.

ID7 원시각 읽기만 할 때, feedback launch 종료 후:

```bash
python3 /home/minwoo/Development/LLM/robstride_read_angles.py \
  --transport serial --port /dev/ttyUSB0 --baud 921600 --motor 7 --once
```

이 명령은 그리퍼를 이동시키지 않는다. 각 위치로 움직이는 실물 제어 코드는 미구현이다.
과거 각도 범위를 현 위치 명령이나 안전 한계로 바로 사용하지 않는다.

측정표: motor_angle_rad, approach_direction(open/close), nut_relative_mm,
opening_mm, contact_state, zero_reference, note.
연속 다회전 각도를 보존하고 500도를 140도로 축약하지 않는다.

## 5. 드라이버 구현 후 수행할 구동 검증 (현재 실행 코드 없음)

1. 팔 한 축: 현재각으로 초기 목표 설정 → 작은 변위 → 도달/유지/추종 오차 기록.
2. 취소, 통신 단절, 비상정지 및 낙하 방지 상태 확인. 중단 방식에 따른 실제 거동 기록.
3. 검증된 한계로 팔 6축 FollowJointTrajectory 연결, 무부하 짧은 경로 실행.
4. 그리퍼: 실측표 변환 → 지정폭 → 파지/실패/정지 구분. 물체 접촉 시 제한 확인.
5. 작업대/물체 충돌 모델과 TCP 반영 후 고정 좌표 집기·이송·놓기.

속도·전류·가속도·허용 오차는 확인된 모터/기구 조건 및 실측값으로 정한다.
현재 가상 설정 수치를 실물 검증 완료 값으로 사용하지 않는다.

## 코드 위치와 역할

모든 경로는 기준 workspace 기준이며, 아래는 실제 파일이다.

- src/pallet_physical_moveit_config/launch/feedback.launch.py: 최신 모델 + 피드백 + 계획 전용.
- src/pallet/scripts/pallet_motor_joint_state.py: 읽기/변환/상태 발행, stale 감지.
- src/pallet/scripts/motor_joint_state_core.py: 방향*(raw-zero)/ratio, 관절 한계 검사.
- src/pallet/scripts/robstride_serial_readonly.py: 파라미터 읽기 전용 직렬 전송.
- src/pallet/config/hand_guiding.yaml: 기존 방향/영점/비율. 전체 commissioned=false 유지.
- src/pallet/config/motor_zero_reference.json: 2026-10-22 영점 기록.
- src/pallet/urdf/pallet_with_gripper.urdf: 최신 모델, 그리퍼 기구 수정은 남음.
- src/pallet_physical_moveit_config/scripts/verify_mock_execution.py: 가상 실행 검증 전용.

기존 pallet_robstride_trajectory_bridge.py는 SocketCAN 경로이다.
기존 single_axis_current/gravity 도구는 위치제어 도구가 아니며 이번 절차에서 실행하지 않는다.

## 결과 기록

날짜 / 포트 / 설정파일 / 로봇 자세 / 확인한 축 / 측정 오차 / 상태 로그 /
사진 또는 영상 경로 / 통과·실패 / 다음 조치를 남긴다.
이 문서 작성 시점에는 새 launch의 실제 장치 수신·구동을 실행하지 않았다.
