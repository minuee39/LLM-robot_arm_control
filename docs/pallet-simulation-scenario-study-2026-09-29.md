# Pallet 블록 조건별 Isaac 시뮬레이션 시험

## 작성일과 목적

- 2026-09-29. 실물 로봇 없이 기존 30×30×50 mm, 0.05 kg 단일 성공 사례를 다른 시작 위치·질량·폭으로 확장해 확인했다.
- 각 조건은 한 번씩 실행했다. 아래 2/3은 이 세 조건의 관측값이지 반복 성공률이나 실기 성능이 아니다.

## 최종 기준 workspace와 주요 파일

- `/home/minwoo/Development/LLM`
- `ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/workcell_scenario.py`: 실행별 블록 중심·크기·질량 입력의 유한값, 양수, 테이블 내 배치, 67 mm 개방 폭 계약 검사.
- `ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/isaac_sim.py`: 검증한 입력을 Isaac 동적 블록에 적용. 공용 `workcell.yaml`은 수정하지 않는다.
- `ros2_ur_ws/src/mtc_tutorial/launch/pallet_mtc_isaac_demo.launch.py`: `block_x`, `block_y`, `block_size_x/y/z`, `block_mass_kg` 실행 인자 전달.
- `ros2_ur_ws/src/pallet_gripper_moveit_config/test/test_workcell_scenario.py`: 정상 입력·경계값 테스트 2개.
- `ros2_ur_ws/scripts/watch_pallet_block.py`: Isaac 실제 블록 중심·자세 판정. 원시 궤적은 `/tmp/pallet_block_variant_0{1,2,3}.json`에 저장됐다.

## 문제 → 원인 → 해결 → 검증

1. **고정 작업 셀:** 이전에는 `workcell.yaml`을 고쳐야 다른 블록 조건을 실행할 수 있었다. 시뮬레이터 시작 인자와 순수 검증 함수를 추가하고 불가능한 크기·비정상 질량·테이블 밖 중심을 거부한다. 단위 테스트 2개와 관련 ROS 패키지 2개 빌드가 통과했다.
2. **MTC 성공과 실제 파지 실패:** 0.08 kg 조건에서 MTC 9구간은 58.3초에 정상 종료했으나, 실제 블록은 이동 중 이탈했다. 블록 Z가 0 아래로 처음 떨어진 시점은 관측 시작 후 71.49초였고, 최종 중심은 `(0.14210,-0.14764,-0.74000)` m로 배치 검사에 실패했다. MTC 완료만으로 성공 처리하지 않고 블록 계측 결과를 별도로 확인했다. 추락 후 블록이 테이블을 관통해 바닥까지 떨어진 상태라 현재 Isaac 접촉 프록시·테이블 물리 모델의 한계도 조사해야 한다.
3. **원인 분리:** 시작 위치를 `(-0.27,0.28)` m로 유지하고 질량만 0.05 kg으로 되돌린 결과 최종 XY 오차 9.57 mm·수직 자세로 성공했다. 따라서 변경 위치만으로 실패했다고 볼 근거는 없고, 0.08 kg에서의 파지 여유 부족 또는 접촉 물리 변화가 유력하다. 단 한 번씩의 비교이므로 질량 임계값은 확정하지 않았다.
4. **폭 변경:** 기본 시작 위치·0.05 kg에서 블록 Y 폭을 30→25 mm로 바꾸고 카메라의 `--pallet-block-size`도 같이 맞췄다. MTC 실행은 61.97초에 끝났고 실제 최종 XY 오차 3.84 mm·수직 자세로 성공했다.

## 조건과 결과

| 시험 | 시작 중심 XY (m) | 블록 크기 (m) | 질량 (kg) | MTC | 실제 최종 중심 (m) | XY 오차 | 물리 판정 |
| --- | --- | --- | ---: | --- | --- | ---: | --- |
| 01 | (-0.27, 0.28) | 0.030×0.030×0.050 | 0.08 | 완료 | (0.14210, -0.14764, -0.74000) | 219.43 mm | 실패 |
| 02 | (-0.27, 0.28) | 0.030×0.030×0.050 | 0.05 | 완료 | (0.29044, -0.29975, 0.02500) | 9.57 mm | 성공 |
| 03 | (-0.30, 0.30) | 0.030×0.025×0.050 | 0.05 | 완료 | (0.29890, -0.30367, 0.02500) | 3.84 mm | 성공 |

목표 중심은 모두 `(0.30,-0.30,0.025)` m, 좌표계는 `base_footprint`다. `watch_pallet_block.py`는 XY 20 mm, 높이 5 mm, 블록 긴 축 수직 기울기 15° 이내와 최신 피드백을 동시에 요구한다.

## 재현 순서

터미널 1: 아래에서 조건 인자만 바꿔 실행하고 `Pallet workcell ready for direct-pose MTC execution.`을 기다린다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mtc_tutorial pallet_mtc_isaac_demo.launch.py headless:=true use_rviz:=false startup_timeout:=180 block_x:=-0.27 block_y:=0.28 block_mass_kg:=0.05
```

터미널 2: 카메라 장면을 생성한다. 시험 03은 `--pallet-block-size 0.030,0.025,0.050`을 추가해야 Isaac 블록과 MTC 객체 크기가 일치한다. 시험별 장면·측정 JSON 파일명은 재사용하지 않는다.

```bash
cd /home/minwoo/Development/LLM
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 ur10e/apps/yolo_camera_node.py --detector color --coordinate-mode pallet --scene-state-file /tmp/pallet_camera_calibration.json --vision-scene-file /tmp/pallet_vision_scene_variant_02.json --stability-min-samples 3
```

터미널 3: 실행 직전에 측정을 시작한다.

```bash
cd /home/minwoo/Development/LLM
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 ros2_ur_ws/scripts/watch_pallet_block.py --seconds 115 --output /tmp/pallet_block_variant_02.json
```

터미널 4: 카메라가 `red_block`을 검출한 뒤 명령한다. 현재 `standard` 실행은 MTC 완료 뒤 Isaac 블록의 최종 위치·높이·수직 자세를 자동 확인한다. 종료 코드 0은 물리 배치 검사까지 통과, 종료 코드 3은 물리 배치 실패 또는 피드백 부재다. 터미널 3의 장시간 측정은 상세 궤적 분석이 필요할 때 사용한다.

```bash
cd /home/minwoo/Development/LLM
/usr/bin/python3 ur10e/mtc_command_runner.py --robot-profile pallet_gripper --pallet-scene-file /tmp/pallet_vision_scene_variant_02.json --pallet-execution-mode standard --once '{"action":"pick_place","pick_object":"red_block","place_xyz":[0.3,-0.3,0.025]}'
```

## 검증과 남은 주의사항

- `python3 -m unittest discover -s ros2_ur_ws/src/pallet_gripper_moveit_config/test -p 'test_*.py'`: 2개 통과.
- `colcon build --symlink-install --packages-select pallet_gripper_moveit_config mtc_tutorial`: 성공.
- 현재 접촉 패드는 Isaac 임시 충돌 프록시다. 물리 실패를 실물 그리퍼의 허용 질량으로 일반화하지 않는다.
- 그리퍼 여유·질량 경계는 동일 조건 반복과 중간 질량 시험으로 확인해야 한다.
- 시뮬레이터 종료 시 MoveGroup이 SIGINT 후 SIGTERM으로 종료되는 기존 현상이 재현됐다. 실행 결과와 종료 로그는 구분해 해석한다.

## 후속 수정: 명령 종료 코드에 Isaac 물리 판정 연결

- `ur10e/mtc_command_runner.py`의 Pallet `standard` 모드에서 MTC가 성공한 뒤 `ros2_ur_ws/scripts/watch_pallet_block.py`를 3초 실행한다. `base_footprint` 기준 목표 중심과 실제 블록 XY 20 mm·높이 5 mm·수직 15°·신선한 토픽을 검증한다. 토픽 부재, 검사 시간 초과, 검사 실패는 종료 코드 3이다. MTC 자체가 실패하면 그 종료 코드를 유지하며, 계획 전용 모드는 물리 검사를 하지 않는다. Isaac은 현재 `red_block`의 물리 위치만 발행하므로 다른 블록의 `standard` 실행은 거부한다.
- 기존 실패 조건 `(-0.27,0.28)` m·0.08 kg을 다시 실행했다. MTC 9구간은 약 58.45초에 완료됐지만 블록 최종 중심은 `(-0.06222,-1.13127,-0.74000)` m였고, 명령은 물리 검사 실패 코드 3으로 끝났다. 이 실행의 목표 XY 오차는 약 906.8 mm다. 앞선 0.08 kg 시험과 결과 위치가 다르므로 이탈 궤적은 결정적이지 않다.
- 기본 조건 `(-0.30,0.30)` m·0.05 kg을 다시 실행했다. MTC 약 62.79초 후 자동 검사의 실제 최종 중심은 `(0.28616,-0.29969,0.02500)` m, XY 오차 약 13.84 mm, 수직 자세였고 명령은 코드 0으로 끝났다.
- `PYTHONPATH=ur10e PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q ur10e/tests/test_mtc_command_runner.py`: 30개 통과. 실제 Isaac 성공·실패 양쪽의 종료 코드를 확인했다.

## 다음 작업 로드맵

- [x] 시작 위치·크기·질량을 실행 인자로 분리하고 유효성 검사
- [x] 변경 시작점 0.05/0.08 kg 비교 및 25 mm 폭 블록 시험
- [x] 0.065·0.0725·0.075 kg 중간 질량과 경계 근처 반복 시험
- [x] MTC 성공과 Isaac 물리 배치 판정을 하나의 실행 결과로 묶기
- [ ] 0.065 kg 초과 경계 조건의 계획 경로·카메라 좌표·접촉력과 배치 오차 상관관계 분석
- [x] 조건당 3회 반복과 실행별 시뮬레이터 재시작 절차 자동화
- [ ] Isaac 접촉 프록시·테이블 충돌과 MoveIt CAD 충돌 모델의 차이 분석
- [x] 500 g 조건에서 파지력·경로 속도를 분리해 반복 시험하고 Isaac 전용 안정 설정 기록

## 후속 시험: 질량 경계 부근 반복 (2026-09-29)

시작 중심 `(-0.27,0.28)` m, 블록 크기 `(0.030,0.030,0.050)` m, 목표 `(0.30,-0.30,0.025)` m, J8 닫힘 목표 0.012 m를 유지하고 질량만 변경했다. 매 시험은 Isaac을 새로 시작했으며 카메라 검출 좌표는 약 `(-0.269,0.277,0.028)` m 범위였다. 모든 MTC 명령은 9구간 계획·실행을 완료했지만 실제 물리 판정은 아래처럼 달랐다. `standard` 명령의 종료 코드 0은 물리 배치 성공, 3은 물리 배치 실패다.

| 질량 | 이 조건의 관측 성공/실행 | 이번 추가 시험의 실제 최종 XY 오차 | 비고 |
| ---: | ---: | --- | --- |
| 0.050 kg | 1/1 | 이전 시험 9.57 mm | 기준 조건. 기본 시작점 시험과 합산하지 않음 |
| 0.065 kg | 1/1 | 4.29 mm | 수직 배치 성공 |
| 0.0725 kg | 1/2 | 1.35 mm 성공; 43.06 mm 실패 | 동일 질량에서 결과가 갈림 |
| 0.075 kg | 0/2 | 219.22 mm 실패; 69.12 mm 실패 | 첫 시도는 테이블 아래로 추락, 둘째는 테이블 위에 놓였지만 목표 밖 |
| 0.080 kg | 0/2 | 앞선 시험 219.43 mm; 906.76 mm 실패 | 두 번 모두 파지 후 목표 배치 실패 |

원시 장시간 궤적은 `/tmp/pallet_block_mass_065.json`, `/tmp/pallet_block_mass_0725_execution.json`, `/tmp/pallet_block_mass_0725_repeat.json`, `/tmp/pallet_block_mass_075.json`, `/tmp/pallet_block_mass_075_repeat.json`에 있다. 0.0725 kg 첫 시도의 **초기** 관측기(`/tmp/pallet_block_mass_0725.json`)는 계획 시간이 길어 실제 동작 전에 종료됐으므로 그 `placement_pass=false`는 물리 실패 횟수에 넣지 않았다. 실행 도중 새로 시작한 관측기와 명령의 자동 최종 판정은 통과했다.

이 데이터로 일정한 허용 질량이나 성공률을 산출할 수 없다. 특히 0.0725 kg에서 한 번 성공하고 한 번 실패했으며, 작은 카메라 좌표 차이와 계획 경로·접촉 동역학 차이를 아직 통제하지 못했다. 현재 데모에는 반복 성공을 확인한 기본 0.05 kg 설정을 유지하고, 더 무거운 블록은 물리 배치 검사 실패 시 즉시 실패로 보고하도록 한다. 이는 시뮬레이션 운용 판단이지 실물 안전 하중 기준이 아니다.

중간 질량 재현 시 터미널 1의 launch 인자에서 `block_mass_kg:=0.065`, `0.0725`, `0.075` 중 하나를 사용한다. 계획 시간이 최대 90초까지 늘 수 있으므로 장시간 궤적이 필요하면 터미널 3의 관측 시간을 `--seconds 210`으로 하고 질량·시도별 새 출력 파일을 지정한다. 명령의 자동 3초 최종 판정은 장시간 관측기와 별개로 실행된다.
