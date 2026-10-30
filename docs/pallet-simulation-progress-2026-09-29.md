# Pallet 실기 제외 시뮬레이션 통합 작업 기록

> 이 문서는 prismatic 그리퍼 적용 전의 중간 기록이다. 현재 결과·실행 명령·인수인계는 [후속 작업 기록](pallet-simulation-handoff-2026-09-29.md)을 참조한다.

## 작성일과 작업 목적

- 작성일: 2026-09-29
- 목적: 실물 로봇 없이 가능한 RGB-D 인식, 명령 변환, Isaac/MoveIt/MTC 계획 경로를 검증하고 미완료 지점을 분명히 한다.
- 실제 로봇, 모터, CAN 장치는 사용하지 않았다. 커밋·푸시·Notion 기록도 하지 않았다.

## 최종 기준 workspace

```bash
cd /home/minwoo/Development/LLM
```

작업 트리는 기존 변경과 이번 변경이 섞여 있으므로 전체 diff를 하나의 기능으로 간주하거나 무분별하게 스테이징하지 않는다.

## 현재 주요 패키지와 파일

- `ros2_ur_ws/src/mtc_tutorial/`: Pallet MTC 프로파일, 계획 전용 실행, 실패 단계 로그.
- `ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/`: Isaac RGB-D 카메라와 액션 브리지.
- `ur10e/apps/yolo_camera_node.py`, `ur10e/vision/`: 색상 기반 블록 검출, 깊이·좌표 변환, 장면 JSON 출력.
- `ur10e/mtc_command_runner.py`, `ur10e/command_parser.py`: Pallet 명령을 MTC 계획 인자로 변환.

## 문제 → 원인 → 해결 → 검증

1. **카메라 출력 확인 필요:** Isaac headless 통합 launch로 RGB, Depth, CameraInfo 토픽과 `/tmp/pallet_camera_calibration.json`을 생성했다. HSV 색상 검출 노드의 `/tmp/pallet_vision_scene_resume.json`에는 실제 촬영한 빨간 블록만 기록됐다. 출력 계약은 `frame=base_footprint`, `unit=m`, `position_type=box_center`, 크기 `(0.03, 0.03, 0.05) m`다. 관측 중심은 약 `(-0.300, 0.297, 0.028) m`였으며, 예상 XY `(-0.30, 0.30) m`와 몇 mm 차이였다. 초록색·파란색은 존재하지 않아 출력 객체에 없었다.
2. **색상 후보 경고 과다:** 작업대 밖의 색상 후보가 매 프레임 일반 카메라 오류로 기록됐다. `OffTableCandidateError`를 도입해 작업대 외 후보만 디버그 로그로 거절하도록 변경했다. 8초 재실행에서 빨간 블록은 계속 출력됐고 해당 반복 경고는 사라졌다. `timeout`으로 노드를 종료할 때 ROS publisher context 예외가 한 번 출력됐으므로 종료 처리 안정성은 별도 확인이 필요하다.
3. **그리퍼 명령 한계 불일치:** 브리지의 기존 허용 상한 `0.229 rad`는 MoveIt `joint_limits.yaml`의 `J8` 상한 `0.103 rad`와 달랐다. 브리지가 설정 파일의 상·하한을 읽도록 변경하고 목표 허용 오차를 `0.002 rad`로 줄였다. 실행 중인 Isaac 액션 서버에 `0.2 rad` 명령을 보내 목표 거절을 확인했다. 블록 파지 시 접촉으로 목표각에 못 미치는 경우의 성공·stall 판정은 아직 검증되지 않았다.
4. **MTC Pick & Place 계획 미완료:** 90초 및 30초 제한의 `planning_only:=true` 실행 모두 해를 만들지 못했다. grasp pose 후보와 일부 접근/배치 IK 해는 생성됐지만 `move above pick/place` OMPL 요청이 5~10초에 시간 초과됐고, 배치 후보 일부는 `Collision between J3 - J5` 또는 `no IK found`였다. `J8` 실측 시작값 약 `0.0002515 rad`와 MTC open pose `0 rad`의 차이도 Connect 단계에서 보고됐다. 접근·하강 단계 실패 콜백을 추가했으나 이 실행에서 별도 실패 사유 문구는 출력되지 않았다. 단일 원인은 아직 확정하지 않는다.

## 최종 구조와 설계 결정

- 색상 검출은 YOLO weight 없이 실행 가능하며, 작업대 높이 검사를 통과한 후보만 Pallet 장면으로 내보낸다.
- 작업대 Z 기준은 `base_footprint`에서 `0 m`; 블록 중심 Z는 높이 절반 `0.025 m`에 Isaac 초기 여유 `0.003 m`를 더한 `0.028 m`다. MTC 정적 프로파일의 기본 Z `0.025 m`와는 차이가 있으므로 카메라 장면을 사용할 때에는 검출 좌표를 명령 인자로 전달해야 한다.
- 계획 전용 MTC 실행은 궤적을 컨트롤러에 보내지 않는다. 계획 성공 전에는 실행 단계 성공을 주장하지 않는다.

## 실행 순서

터미널 1에서 시뮬레이션 작업 셀을 시작한다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mtc_tutorial pallet_mtc_isaac_demo.launch.py headless:=true use_rviz:=false startup_timeout:=180
```

`Pallet workcell ready for direct-pose MTC execution.` 메시지 뒤 터미널 2에서 카메라 장면을 생성한다.

```bash
cd /home/minwoo/Development/LLM
source /opt/ros/jazzy/setup.bash
python3 ur10e/apps/yolo_camera_node.py --detector color --coordinate-mode pallet --scene-state-file /tmp/pallet_camera_calibration.json --vision-scene-file /tmp/pallet_vision_scene.json
```

터미널 3에서 MTC 계획만 확인한다. **현재 계획 실패가 재현된다.**

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py robot_profile:=pallet_gripper planning_only:=true task_timeout:=30
```

## 검증 명령과 결과

```bash
cd /home/minwoo/Development/LLM
PYTHONPATH=ur10e PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q ur10e/tests/test_depth_utils.py ur10e/tests/test_hsv_block_detector.py ur10e/tests/test_mtc_command_runner.py ur10e/tests/test_calibration.py ur10e/tests/test_detection_store.py
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s ros2_ur_ws/src/mtc_tutorial/test -p 'test_*.py'
python3 -m py_compile ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/isaac_action_bridge.py ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/isaac_sim.py ros2_ur_ws/src/mtc_tutorial/launch/pallet_mtc_isaac_demo.launch.py ur10e/apps/yolo_camera_node.py
```

- Python 관련 테스트 65개, MTC 프로파일 테스트 5개, 문법 검사, `mtc_tutorial` 빌드, 해당 파일 `git diff --check`: 통과.
- Isaac 카메라 → 색상 검출 → 장면 JSON: 빨간 블록 1개에 대해 확인.
- `0.2 rad` 그리퍼 명령: 설정 상한 초과로 거절됨.
- MTC 30초·90초 계획: 실패. Pick & Place 실행과 물체 이동은 검증하지 못했다.
- 종료 후 관련 Isaac/MoveGroup/MTC 프로세스가 남지 않은 것을 확인했다.

## 남은 주의사항과 다음 작업 로드맵

- [x] 실기 없는 로봇 관절 계약 점검 및 시뮬레이터 기동
- [x] Isaac RGB-D 출력 및 빨간 블록 위치 계약 검증
- [x] Pallet 명령 변환·비전 단위 테스트와 MTC 계획 전용 경로 검증
- [ ] MTC의 접근·하강 경로, `J3–J5` 충돌, J8 시작값 차이를 각각 격리하고 전체 계획 1회 성공
- [ ] 계획 성공 후 Isaac에서 파지·배치 실행 및 물체 최종 위치 검증
- [ ] 접촉 시 그리퍼 action의 `reached_goal`/`stalled` 의미를 검증하고 반복 실행 회귀 기준 수립
- [ ] 실제 로봇 완성 후에만 장비 관절·CAN·안전 범위를 실측하여 실기 연동
