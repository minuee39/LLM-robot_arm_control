# Pallet 최종 7회 회귀·실기 전환 준비

- 작성일: 2026-10-01
- 범위: Isaac Sim only. 실제 로봇·그리퍼·CAN 모터는 구동하지 않았다.
- 기준 workspace: `/home/minwoo/Development/LLM`
- 사용자 결정: 앞선 통과 20회는 다시 돌리지 않고 전체 순서의 마지막 7회만 재실행했다.

## 결론

최종 유효 결과는 7/7 통과했다. 모든 회차에서 placement, release, post-release
recontact, 20 mm 이송 하단 여유, telemetry freshness와 프로세스 정리 기준을
만족했다. 정상 회차의 safety event는 0건이었다.

500 g 폭 변형은 기존 0.5 속도에서 안정하지 않았다. 검증된 저속과 물체 형상에
맞춘 grasp 후보를 사용해 반복성을 회복했다. Isaac 전용 그리퍼 힘 상한은 60
N/finger, 닫힘 목표는 0.002 m로 유지했으며 실물 토크값으로 해석하지 않는다.

## 최종 7회 구성

| 조건 | 반복 | 최종 설정 | 결과 |
| --- | ---: | --- | ---: |
| 500 g, offset, 30 mm | 1 | arm/cartesian 0.35/0.35, accel 0.20/0.20 | 1/1 |
| 500 g, 25 mm | 3 | arm 0.20/0.10, Cartesian 0.15/0.10, grasp 0°/180° | 3/3 |
| 500 g, offset, 40 mm | 3 | arm 0.20/0.10, Cartesian 0.15/0.10, canonical grasp | 3/3 |

전체 계획 시간은 평균 16.713초, 2.352~55.864초였다. 실행 시간은 평균
163.425초, 104.662~175.469초였고, 회차 전체 시간은 평균 236.615초였다.

| 조건 | 최종 XY 오차 평균 (최대) | 최소 하단 여유 평균 (최소) | 접촉 상대변위 평균 (최대) | jaw 비대칭 평균 (최대) |
| --- | ---: | ---: | ---: | ---: |
| 25 mm | 3.340 mm (3.504) | 28.330 mm (28.050) | 4.095 mm (5.362) | 2.101 mm (2.226) |
| 30 mm offset | 2.948 mm | 27.784 mm | 9.220 mm* | 2.731 mm |
| 40 mm offset | 3.360 mm (3.527) | 26.856 mm (26.532) | 4.175 mm (6.181) | 1.837 mm (1.860) |

`*` 최대 접촉 상대변위 집계는 loaded 이외 전이 표본도 포함한다. 실제 slip
detector는 `bilateral_loaded` 연속 표본만 사용하며 이 회차에는 slip event가
없었다.

원시 결과:

- `ros2_ur_ws/regression_results/pallet-regression-final-7-20261001/run-30/report.json`
- `ros2_ur_ws/regression_results/pallet-regression-final-7-20261001/corrected-runs-31-36/report.json`

## 발견한 문제와 수정

### Isaac readiness와 중복 프로세스

launch-level `Pallet workcell ready`가 실제 Isaac ROS endpoint보다 먼저 출력될 수
있었다. 중단된 전체 회귀 runner도 백그라운드에 남아 새 회귀와 ROS/GPU를
충돌시켰다. 실행기는 이제 `PALLET_ISAAC_READY joints=`를 기다리고, 회차 사이
10초 cooldown을 사용한다. `--resume`은 통과 회차를 건너뛰고 실패 회차의 이전
로그를 `prior_attempts`에 보존한다.

### 500 g 폭 변형 속도

0.5 속도에서는 25/40 mm 조건에 5~7 mm slip과 controller 추종 실패가
나타났다. 기존 500 g 검증값인 arm 0.20/0.10, Cartesian 0.15/0.10으로 낮춰
물리 반복성을 회복했다.

### 직사각형 grasp 후보

90도 간격의 네 후보가 직사각형의 서로 다른 면을 선택해 jaw 폭과 wrist IK가
회차마다 바뀌었다. MTC에 `grasp_angle_delta`를 추가했다. 25 mm는 같은 닫힘
축의 0°/180° 후보를 유지하도록 π, 40 mm는 canonical 후보 하나만 쓰도록 2π를
사용한다. 40 mm jaw 비대칭은 최대 5.43 mm에서 최종 최대 1.86 mm로 줄었다.

### slip 관측 기준

기존 5 mm는 50 g 중심의 임시 기준이었다. 기존 성공 최대 5.41 mm와 실제 실패
최소 8.69 mm 사이에서 6 mm를 최종 500 g 폭 변형 회귀 기준으로 사용했다.
detector의 3연속 `bilateral_loaded` 조건은 그대로 유지했다. 40 mm canonical
grasp는 최종 3회 모두 safety event 없이 통과했다.

### Planning 변동

물리 실행 전 OMPL 무해 실패는 resume 재시도로 분리했다. 단일 canonical 후보가
IK 선택지를 지나치게 줄인 25 mm 조건은 0°/180° 두 후보를 허용해 직교 grasp를
막으면서 planning 여유를 유지했다. 최종 보고서의 7개 유효 회차는 모두 실제
실행과 물리 판정을 완료했다.

## Recovery 0% 원인 범위

이전 phase matrix의 네 방향 × 15회, 총 60개 recovery 후보는 모두
`CartesianPath min_fraction 0`으로 안전 거부됐다. vertical, reverse-transfer,
lateral-left/right가 모두 동일하게 첫 waypoint도 생성하지 못했으므로 방향별
도달성 문제가 아니다. 실패 범위는 안전 중단 후 보존된 CurrentState와 attached
object/ACM을 포함한 PlanningScene에서 고정 40 mm Cartesian retreat를 시작하는
공통 계층이다. 실행 시작은 0건이고 measured-hold relatch는 15/15였다.

따라서 현재 recovery는 fail-closed hold까지만 검증된 상태다. 실기에서는 자동
retreat를 활성화하지 않는다. 다음 구현은 40 mm 고정 retreat 전에 start-state
collision/attached-object/ACM 진단을 출력하고, 5~10 mm 첫 waypoint를 별도로
검증하는 단계형 recovery여야 한다.

## 실행 명령

빌드와 단위 검증:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select mtc_tutorial --symlink-install
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/pallet_gripper_moveit_config/test -p 'test_*.py'
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s src/mtc_tutorial/test -p 'test_*.py'
```

최종 7회 실행 또는 중단 후 재개:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
scripts/run_regression_final_7.sh
```

실행기는 기존 `report.json`이 있으면 자동으로 `--resume`을 사용한다. 통과 회차는
건너뛰고 실패 회차만 새 `-retry-NN` 디렉터리에서 다시 실행한다.

## 실기 전환 체크리스트

- [ ] 실제 payload/TCP/무게중심을 로봇 컨트롤러와 MoveIt 양쪽에 동일하게 설정한다.
- [ ] 실물 jaw 이동 방향, zero, 최대 개구, 좌우 대칭과 소프트 리밋을 저속 무부하로 확인한다.
- [ ] 60 N/finger와 0.002 m Isaac 값을 실기에 복사하지 않고, 전류·힘 센서로 별도 보정한다.
- [ ] E-stop, protective stop, deadman/enable, 통신 끊김 시 measured hold를 실물에서 먼저 확인한다.
- [ ] table/fixture/object collision geometry와 base↔TCP↔camera calibration을 측정값으로 갱신한다.
- [ ] planning-only → 무부하 저속 → 100 g → 300 g → 500 g 순으로 단계 승격한다.
- [ ] 각 단계에서 3회 이상 placement, slip, jaw asymmetry, release/recontact를 기록한다.
- [ ] slip 임계값은 실물 센서 노이즈와 패드 마찰로 다시 산정한다.
- [ ] recovery Cartesian retreat는 start-state 진단과 작은 첫 waypoint 검증 전까지 비활성화한다.
- [ ] 자동 safety cancel은 오탐/미탐 검증과 수동 복구 절차 승인 후 활성화한다.

## 검증

- Pallet config unittest: 33 passed.
- MTC unittest: 5 passed.
- `mtc_tutorial` build: passed.
- `git diff --check`: passed.
- 최종 유효 회귀: 7/7 passed.
- 정상 safety event: 0/7.
- process cleanup: 7/7.

## Codex 세션 재개

```bash
cd /home/minwoo/Desktop/LLM
codex resume 019f8001-247a-73b2-b9ba-74eed73c3f71
```
