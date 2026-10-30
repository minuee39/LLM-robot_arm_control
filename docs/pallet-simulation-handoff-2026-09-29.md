# Pallet 실기 제외 시뮬레이션 통합 작업 기록과 인수인계

## 작성일과 작업 목적

- 작성일: 2026-09-29
- 목적: 실측 손가락 최대 개방 폭 67 mm를 모델에 적용하고, Isaac Sim RGB-D 카메라 → 장면 JSON → 명령 → MoveIt Task Constructor → 물리 파지·배치까지 확인한다.
- 실제 로봇·모터·CAN은 사용하지 않았다. 이번 작업을 커밋하거나 푸시하지 않았다.

## 최종 기준 workspace

```bash
cd /home/minwoo/Development/LLM
```

기존 혼합 변경이 있는 작업 트리를 유지했다. 이전의 [진행 기록](pallet-simulation-progress-2026-09-29.md)은 prismatic 전 상태의 중간 기록이며, 현재 결과는 이 문서를 기준으로 한다.

후속 시작 위치·질량·폭 변경 및 0.065~0.08 kg 반복 시험의 실제 결과는 [조건별 시험 기록](pallet-simulation-scenario-study-2026-09-29.md)에 정리했다. 0.0725 kg에서 성공과 실패가 모두 나왔고 0.075~0.08 kg은 관측된 두 번씩 모두 실패했다. 아래 단일 블록 성공 결과를 범용 성공률이나 실물 하중 기준으로 해석하지 않는다.
이후 Pallet `standard` 명령은 Isaac의 최종 물리 배치까지 자동 검사하도록 바뀌었다. 검사 실패·피드백 부재는 종료 코드 3이며, 계획 전용 모드는 기존처럼 물리 검사를 하지 않는다.

## 현재 주요 패키지와 파일

- `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper/urdf/pallet_gripper.urdf`: 양쪽 prismatic 손가락과 67 mm 개방 운동학.
- `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper_moveit_config/config/`: SRDF, 열림 자세, 관절 제한, Isaac 물리 게인·작업 셀.
- `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/isaac_sim.py`: RGB-D, 블록 실제 위치 토픽, 양쪽 손가락 피드백 및 시뮬레이션용 평행 패드 충돌체.
- `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/isaac_action_bridge.py`: 양측 실측 위치·힘에 따른 그리퍼 액션 판정.
- `/home/minwoo/Development/LLM/ros2_ur_ws/src/mtc_tutorial/config/robot_profiles.yaml`: 파지 목표 J8=0.012 m, 손끝을 블록 중심보다 15 mm 낮추는 자세.
- `/home/minwoo/Development/LLM/ur10e/apps/yolo_camera_node.py`, `/home/minwoo/Development/LLM/ur10e/mtc_command_runner.py`: 색상 블록 인식과 단일 인식 블록의 명시적 배치 좌표 명령.
- `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/watch_pallet_block.py`: 실제 블록 XY·높이·수직 자세를 확인하는 유한 시간 관측기.

## 문제 → 원인 → 해결 → 검증

1. **개폐 모델 불일치:** 구형 J8 회전 모델은 실측 최대 손가락 간격 67 mm를 표현하지 못했다. J8/J9를 `prismatic`으로 바꾸고 각각 `0..0.0335 m` 이동하도록 했다. J9는 J8을 `multiplier=1`로 따라가고 두 손가락 간격은 `2 × J8 = 0..0.067 m`이다. URDF·MoveIt·초기 열림 자세를 맞추고 손가락끼리의 충돌은 SRDF/PhysX에서 제외했다. `check_urdf`, 관절 계약 감사, 보정 도구의 0/67 mm 끝점 테스트가 통과했다.
2. **검사 도구의 구형 계약:** 관절 감사기가 J9 `mimic=-1`을 기대하고 보정 도구가 반올림 때문에 67 mm를 거부했다. prismatic/+1/양쪽 33.5 mm 계약으로 감사기를 갱신하고 1 µm 끝점 계산 오차만 허용했다. 보정 테스트 3개와 `--object-width-mm 67`이 통과했다.
3. **컨트롤러 성공과 물리 배치가 달랐음:** 초기 MTC 9구간 실행은 성공했지만 블록은 `(-0.3913, -0.1571, 0.0150) m`에서 멈춰 목표와 약 706 mm 떨어졌다. 당시 양쪽 손가락은 약 0.021 m에서 접촉했고, 블록은 lift 중 약 0.055 m까지 올라간 뒤 이동 중 미끄러졌다. 양쪽 실측 J8/J9와 힘을 발행하고, 그리퍼 액션의 목표 도달 또는 양측 정지·양측 힘 1 N 이상을 서로 구분해 판정하도록 했다. ROS `/joint_states`에는 명령 관절 J8만 발행하고 원시 `/isaac_joint_states`에는 J8/J9 모두 발행한다.
4. **접촉면과 배치 자세:** CAD 원본 충돌 메시에는 연결부 형상이 포함돼 평행 집게 접촉을 제대로 재현하지 못했다. Isaac에 가져오는 *임시 URDF*에서만 두 손끝 충돌체를 크기 `(0.012, 0.022, 0.030) m` 평행 박스로 바꿨다. 공개 URDF의 시각 메시·MoveIt 충돌 메시와 실측 67 mm 개폐 운동학은 유지했다. 30 mm 폭 블록에 대해 닫힘 명령 `J8=0.012 m`를 사용한다. 첫 평행 패드 실행은 목표 XY 근처였지만 블록이 옆으로 누워 중심 Z가 0.015 m였다. 손끝 목표를 블록 중심에서 15 mm 아래로 낮추자 수직 배치가 확인됐다. 이 패드는 시뮬레이션 접촉 프록시이며 실제 손가락 패드의 크기·마찰·힘을 실측한 결과가 아니다.
5. **카메라 장면에 블록이 하나뿐임:** 기존 Pallet 명령은 두 번째 인식 물체를 목적지로 요구했다. 카메라가 실제 검출한 빨간 블록 하나와 명시적인 `place_xyz` 좌표를 받도록 명령 JSON을 확장했다. 장면의 `frame=base_footprint`, `unit=m`, 시간·신뢰도 0.6 이상을 검증하고, 실행 모드는 `--pallet-execution-mode standard`로 선택한다. 카메라 장면에서 빨간 블록만 검출했고 신뢰도는 약 0.978, 좌표는 약 `(-0.2995, 0.2971, 0.0280) m`였다. 이 좌표로 실행한 MTC 9구간은 약 62.8초에 완료됐다.

## 최종 구조와 설계 결정

```text
Isaac RGB-D → HSV 블록 검출 → /tmp/pallet_vision_scene_e2e.json
→ Pallet JSON 명령(place_xyz는 base_footprint, 단위 m)
→ MTC 계획·실행 → J8/J9 양측 실측 액션 판정
→ /isaac_calibration_block_pose로 실제 물체 위치·자세 검증
```

검증 목표 블록은 `0.030 × 0.030 × 0.050 m`, 시작 중심 `(-0.30, 0.30, 0.025) m`, 배치 목표 중심 `(0.30, -0.30, 0.025) m`다. 이 좌표는 모두 `base_footprint` 기준이며 팔 관절은 rad, 그리퍼 관절은 m다. 배치 검증은 최종 XY 오차 20 mm 이하, 높이 오차 5 mm 이하, 블록 긴 축의 수직 기울기 15° 이하와 최근 피드백을 요구한다.

시뮬레이션 직접 좌표 실행의 최종 중심은 `(0.2853, -0.2997, 0.0250) m`로 XY 오차 약 14.7 mm였다. 카메라 장면을 사용한 최종 중심은 `(0.29194, -0.29959, 0.02500) m`로 XY 오차 약 8.1 mm였고 긴 축은 수직이었다. 실행 종료 후 ROS 블록 위치 토픽으로 안정 상태를 다시 확인했다. 이 결과는 이번 시뮬레이션 작업 셀의 결과이며, 실제 하드웨어 정확도나 반복정밀도 수치는 아니다.

## 실행 순서

터미널 1에서 시뮬레이터와 MoveGroup을 시작하고 `Pallet workcell ready for direct-pose MTC execution.` 메시지를 기다린다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mtc_tutorial pallet_mtc_isaac_demo.launch.py headless:=true use_rviz:=false startup_timeout:=180
```

터미널 2에서 카메라 검출을 실행한다. `pallet_vision_scene_e2e.json`이 생성되고 계속 갱신되는지 확인한다.

```bash
cd /home/minwoo/Development/LLM
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 ur10e/apps/yolo_camera_node.py --detector color --coordinate-mode pallet --scene-state-file /tmp/pallet_camera_calibration.json --vision-scene-file /tmp/pallet_vision_scene_e2e.json --stability-min-samples 3
```

터미널 3에서 실제 블록의 최종 위치와 자세를 측정한다. 한 번의 실행마다 새 파일 이름을 사용한다.

```bash
cd /home/minwoo/Development/LLM
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 ros2_ur_ws/scripts/watch_pallet_block.py --seconds 130 --output /tmp/pallet_block_motion_camera_command.json
```

터미널 4에서 카메라 장면 기반 명령을 실행한다. 우선 `--dry-run`으로 인자만 확인할 수 있고, 아래 명령은 Isaac 시뮬레이터를 실제로 움직인다.

```bash
cd /home/minwoo/Development/LLM
/usr/bin/python3 ur10e/mtc_command_runner.py --robot-profile pallet_gripper --pallet-scene-file /tmp/pallet_vision_scene_e2e.json --pallet-execution-mode standard --once '{"action":"pick_place","pick_object":"red_block","place_xyz":[0.3,-0.3,0.025]}'
```

## 검증 명령과 결과

```bash
cd /home/minwoo/Development/LLM
PYTHONPATH=ur10e PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q ur10e/tests/test_depth_utils.py ur10e/tests/test_hsv_block_detector.py ur10e/tests/test_mtc_command_runner.py ur10e/tests/test_calibration.py ur10e/tests/test_detection_store.py
PYTHONPATH=ros2_ur_ws/src/pallet_gripper/tools PYTHONDONTWRITEBYTECODE=1 python3 -m unittest ros2_ur_ws/src/pallet_gripper/tools/test_gripper_calibration.py
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s ros2_ur_ws/src/mtc_tutorial/test -p 'test_*.py'
python3 ros2_ur_ws/scripts/audit_pallet_joint_contract.py
check_urdf ros2_ur_ws/src/pallet_gripper/urdf/pallet_gripper.urdf
```

- 카메라·명령 관련 pytest 67개, 그리퍼 보정 unittest 3개, MTC 프로파일 unittest 5개 통과. URDF 파싱과 관절 계약 감사 통과. `colcon build --packages-select pallet_gripper pallet_gripper_moveit_config mtc_tutorial --symlink-install` 완료.
- 양측 접촉 판정의 중복 샘플·단측 힘·위치 변화 사례를 ROS Python 환경에서 확인했다.
- 첫 CAD 충돌체 물리 실행: MTC 종료는 성공, 실제 배치 실패. 수정 후 직접 좌표 실행과 카메라 명령 실행: MTC 완료 및 엄격한 블록 XY·높이·수직 자세 검사 통과.
- 카메라 명령 실행 결과 파일: `/tmp/pallet_block_motion_camera_command.json`. 이 파일은 측정 출력으로 저장소 커밋 대상이 아니다.

## 남은 주의사항

- Isaac의 손끝 패드는 임시 충돌 프록시다. 공개 MoveIt URDF는 여전히 CAD 충돌 메시이므로 두 계획 모델의 접촉 형상이 정확히 일치하지 않는다. 실제 손가락 접촉면과 힘·속도는 장비 완성 후 측정해야 한다.
- 배치 성공은 현재 30 mm 블록 한 종류와 두 번의 성공 실행에서 확인됐다. 물체 크기·질량, 다른 시작 위치, 반복 시도에 대한 안정성은 별도 검증이 필요하다.
- MoveGroup과 일부 ROS 노드는 시뮬레이터 종료 시 SIGINT 후 SIGTERM으로 종료될 수 있다. 실행 중 동작 결과와 종료 로그는 구분해서 확인한다.
- 기존 `pallet/config/robstride_hardware.yaml`은 5축 미시운전 설정이다. 현재 6축 팔 + J8 모델과 장비 대응이 확인되지 않아 실기 구동에 사용하지 않는다.
- 계정 사용량의 잔여 퍼센트는 이 작업 환경에서 직접 제공되지 않았다. Codex 공식 문서는 사용자 대시보드 또는 활성 CLI의 `/status`를 안내한다.

## 다음 작업 로드맵

- [x] 실측 0..67 mm 대칭 prismatic 모델 및 URDF/MoveIt/Isaac 계약 검증
- [x] 양측 실제 손가락 위치·접촉 판정과 블록 위치·자세 계측
- [x] 30 mm 블록의 직접 좌표 MTC 파지·배치 시뮬레이션 성공
- [x] Isaac 카메라 검출 → 단일 블록 JSON 명령 → MTC 실행 → 실제 배치 계측 성공
- [ ] 여러 출발 위치·물체 크기·질량·계획 경로로 반복 안정성 검증
- [ ] 시뮬레이터 접촉 프록시와 MoveIt 충돌 모델의 차이 축소
- [ ] 그리퍼 개폐력·마찰·모터 연결은 실물 완성 후에만 실측·적용
