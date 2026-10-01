# Pallet Isaac 운반 높이 점검

## 작성일과 목적

- 2026-09-29 작성, 2026-09-30 갱신. 파지 후 블록이 책상에 가까이 운반되는 원인을 MTC 명령과 Isaac 물리 반응으로 분리하고, 기본 0.05 kg 블록의 높이를 개선한다.
- 기준 workspace: `/home/minwoo/Development/LLM`. 실제 로봇과 모터는 구동하지 않았다.

## 현재 주요 파일

- `ros2_ur_ws/src/mtc_tutorial/config/robot_profiles.yaml`: Pallet 상승·배치 접근·후퇴 거리.
- `ros2_ur_ws/src/mtc_tutorial/src/mtc_tutorial.cpp`: attach 후 `lift object`, OMPL `move above place`, `lower object` 구성.
- `ros2_ur_ws/scripts/watch_pallet_block.py`: Isaac 블록 위치·자세와 운반 중 모서리의 테이블 간격 계측.
- `ros2_ur_ws/scripts/run_pallet_scenario_matrix.py`: 조건별 Isaac 재시작, MTC 실행, 궤적 계측, 판정과 정리를 자동화.

## 문제 → 원인 → 해결 → 검증

1. **파지 직후 탁상에 가까움:** 기존 Pallet 프로파일은 `lift_min_distance=0.01`, `lift_max_distance=0.03` m였다. 높이 0.05 m 블록의 바닥이 탁상에서 명령상 1~3 cm만 떨어지는 MTC 설정이다. 배치 접근도 중심 `place_z+0.01 m`였다. 기본 0.05 kg에서 팔/그리퍼가 무게 때문에 처졌다는 실측 근거는 없다. 기존 0.075~0.08 kg 실패 기록은 블록 미끄러짐·추락에 해당한다.
2. **높이 조정:** 상승·배치 전 하강·후퇴를 각각 고정 `0.04 m`로 맞췄다. `0.10 m`는 lift Cartesian 경로가 약 54.5%에서 중단되고 배치 IK가 없었다. `0.05 m`는 lift 자체는 계획됐지만 배치 IK가 없어 전체 계획에 실패했다. `0.04 m`는 9구간 계획과 Isaac 실행을 완료했다.
3. **중간 경로 계측:** OMPL `move above place`에는 탁상 높이 하한이 없으므로 양 끝의 4 cm만으로 전 구간 간격을 보장할 수 없다. 관측기에 시작점→목표점 XY 투영의 10~90% 구간에서 회전한 블록의 가장 낮은 모서리 Z를 계산하는 `transfer_min_bottom_clearance`와 샘플 수를 추가했다. 최종 상태만 짧게 관측하면 이 값은 `null`이다. 기존 최종 배치 판정은 그대로 둔다.
4. **계측만으로 안전 높이를 보장하지 못함:** Pallet 프로파일에 `transfer_min_clearance=0.020 m`를 추가하고, `move above place`가 만든 궤적을 실행 전에 검사하도록 했다. 파지 변환으로 블록 바닥 모서리 4개를 TCP 좌표로 옮긴 뒤, 각 관절 궤적 구간을 최대 관절 거리 `0.02` 간격으로 보간하고 FK로 모든 모서리의 탁상 간격을 계산한다. 하나라도 기준보다 낮으면 해당 해의 비용을 무한대로 만들어 MTC가 다른 후보를 찾게 하며, 안전한 후보가 없으면 실행하지 않는다. OMPL에 네 모서리 `PositionConstraint`를 직접 넣는 방식은 정상 조건에서도 constrained sampling이 106.113초 걸린 뒤 실패해 사용하지 않았다.
5. **조건별 반복 실행이 수동이었음:** 시뮬레이터와 MoveGroup 준비, 관측기, 동일 치수의 MTC 객체, 최종 판정, 프로세스 종료를 하나의 실행기로 묶었다. 기본 조건은 기준, 변경 시작점, 폭 25 mm, 0.065 kg이다. 각 실행은 새 Isaac 세션을 사용하고 MTC 성공·물리 배치·운반 중 최소 20 mm 간격을 모두 통과해야 성공이다. 시간 초과나 예외에도 SIGINT→SIGTERM→SIGKILL 순으로 해당 프로세스 그룹만 정리한다. `ros2 launch`가 자식 MTC 노드 실패에도 0을 반환할 수 있어, 종료 코드와 함께 `Task execution completed` 로그가 있는지도 검사한다.
6. **자동화 첫 실행의 환경 제한:** 제한된 실행 환경에서는 ROS 로그 디렉터리가 읽기 전용이었고 GPU·원격 Isaac 기본 에셋도 보이지 않았다. 실행별 `ROS_LOG_DIR`을 결과 디렉터리로 지정해 로그 문제를 해결했다. 실제 GPU 통합 시험은 제한 밖에서 실행했으며 일반 사용자 터미널에는 이 샌드박스 제약이 없다.

## 최종 구조와 설계 결정

기본 블록 `(0.030, 0.030, 0.050) m`, 질량 `0.05 kg`, 시작 `(-0.30, 0.30, 0.025) m`, 목표 `(0.30, -0.30, 0.025) m`, 탁상 상면 Z `0 m`를 사용했다. 블록 중심의 최고 Z는 `0.064895 m`였다. 운반 구간 샘플 104개의 회전된 블록 모서리 기준 최소 탁상 간격은 `0.023389 m`였다. 최종 중심은 `(0.287186, -0.299066, 0.025000) m`, 목표 XY 오차 `0.012848 m`, 수직 자세 판정 통과였다. 원시 관측 파일은 `/tmp/pallet_block_carry_10cm.json`이며 파일 이름은 첫 시험 당시 지정된 것으로, 최종 프로파일은 4 cm다.

추가 자동 시험도 모두 통과했다. 변경 시작점 `(-0.27,0.28)`·0.05 kg은 XY 오차 `11.31 mm`, 최소 간격 `26.90 mm`였다. 폭 25 mm·0.05 kg은 XY 오차 `0.78 mm`, 최소 간격 `25.11 mm`였다. 변경 시작점·0.065 kg은 XY 오차 `2.40 mm`, 최소 간격 `27.75 mm`였다. 조건당 한 번의 결과이며 반복 성공률은 아니다.

2026-09-30의 경로 검사 통합 시험에서 기본 20 mm 조건은 2개 해를 약 0.934초에 계획했다. 50 mm로 올린 부정 시험은 후보 5개를 모두 거부했고, 가장 낮은 후보의 진단은 `0.0251299 < 0.05 m`였다. 따라서 안전하지 않은 계획은 실행 전에 중단된다. 이어서 기본 조건을 Isaac에서 실제 실행한 결과, MTC 9구간 실행을 완료했고 블록 바닥의 운반 중 최소 간격은 `25.1109 mm`, 최종 XY 오차는 `6.1010 mm`, 높이 오차는 약 `0.0001 mm`였으며 수직 자세와 배치 판정을 통과했다. 원시 관측 파일은 `/tmp/pallet_block_clearance_guard.json`이다.

조건별 3회 반복과 시험 중 발견한 운반 미끄러짐·계획 시간 초과 수정 결과는 [반복 시험 보고서](pallet-scenario-repeatability-2026-09-30.md)에 정리했다. 최종 설정은 네 조건에서 12/12회 통과했고 전체 최소 간격은 24.85 mm, 최대 XY 오차는 4.98 mm였다.

## 실행 순서

터미널 1:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mtc_tutorial pallet_mtc_isaac_demo.launch.py headless:=true use_rviz:=false startup_timeout:=180
```

터미널 2에서 Isaac 준비 메시지 뒤 관측을 시작한다:

```bash
cd /home/minwoo/Development/LLM
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 ros2_ur_ws/scripts/watch_pallet_block.py --seconds 180 --output /tmp/pallet_block_carry.json
```

터미널 3:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py robot_profile:=pallet_gripper execution_mode:=standard
```

조건별 자동 시험은 ROS 환경을 source한 뒤 다음처럼 실행한다. `--repeat 3`을 추가하면 각 조건을 새 Isaac 세션에서 세 번씩 실행한다. 결과 JSON과 단계별 로그는 출력된 `/tmp/pallet-scenario-matrix-*` 디렉터리에 남는다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_pallet_scenario_matrix.py --scenario offset --scenario narrow --scenario mass_065
```

## 검증 명령과 결과

- `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s ros2_ur_ws/src/mtc_tutorial/test -p 'test_*.py'`: 5개 통과.
- `python3 -m py_compile ros2_ur_ws/scripts/watch_pallet_block.py`: 통과.
- 회전한 블록 모서리 계산의 수평·90° 회전 예제와 실제 관측 JSON 재계산: 통과.
- Isaac 통합: 4 cm 설정의 MTC 9구간 실행 완료, 최종 블록 위치·높이·수직 자세 검사 통과. 10 cm와 5 cm 계획 실패도 확인.
- 시나리오 실행기·입력 검증 unittest 4개와 dry-run: 통과.
- 자동 Isaac 시험: 변경 시작점, 폭 25 mm, 0.065 kg의 MTC·배치·최소 20 mm 운반 간격 판정 통과. 실행 후 관련 프로세스가 남지 않음을 확인.
- 최소 간격 경로 검사: 기본 20 mm planning-only 통과, 50 mm 부정 시험은 최소 `25.1299 mm` 후보를 실행 전에 거부, 기본 Isaac 실행은 실측 `25.1109 mm`로 통과.
- 최종 회귀 검사: MTC 프로파일 unittest 5개, 시나리오 실행기 unittest 3개, Python 구문 검사, `git diff --check`, `mtc_tutorial` 빌드 통과.

## 남은 주의사항과 다음 작업 로드맵

- [x] 낮은 파지 후 높이의 MTC 원인 확인과 도달 가능한 설정 적용
- [x] 실제 물리 블록의 운반 중 탁상 간격 계측
- [x] 다른 시작점·크기·질량에서 4 cm 경로의 계획 가능성과 물리 배치를 각 1회 확인
- [x] 각 조건을 3회 반복해 성공률과 최소 간격 분산 확인
- [x] 운반 구간 전체의 최소 간격을 실행 전에 검사하고 실패 시 다른 후보 탐색 또는 중단 처리
- [ ] 0.065 kg 초과 질량의 접촉력·경로·배치 오차를 같은 조건으로 반복 분석
