# Pallet 1단계 접촉 조건 비교 실험

작성일: 2026-10-09. 목적: 운반 슬립의 회전·접촉 원인 후보를 구분한다.

## 최종 기준 workspace와 주요 파일

`/home/minwoo/Development/LLM`, ROS 2 Jazzy, Isaac Sim 5.1.
주요 파일은 `ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/isaac_sim.py`, `ros2_ur_ws/src/mtc_tutorial/launch/pallet_mtc_isaac_demo.launch.py`, `ros2_ur_ws/scripts/run_palletizing_gui.sh`다.

## 비교 기준

- 블록: 0.5 kg, 30 × 30 × 50 mm.
- 초기 red 중심: base_footprint (-0.32, 0.10, 0.025) m.
- 목표: (0.24, -0.28, 0.025) m.
- arm/Cartesian velocity 0.3, acceleration 0.05.
- close target 0.002 m, grasp angular spacing 2π, place yaw 0.
- upright transfer 활성, lift/lower 0.04 m, 안전 정지 활성.
- 슬립 threshold 0.006 m, 3 consecutive samples.
- 기준 결과: `ros2_ur_ws/regression_results/red-v030-a005-upright-safety-r2-20261009`.
- 변경 변수: block contact offset 0.100 m → 0.002 m. 재질·마찰·힘 제한은 변경하지 않는다.

## 문제 → 원인 후보 → 조사

기준 실험의 첫 6 mm 초과 시각 84.349초에서 변위 X -0.058 mm, Y -4.417 mm, Z -4.959 mm, TCP tilt 0.75°, block tilt 13.41°, J8/J9 effort 11.95/18.71 N이었다. 이전 v0.5/a0.2 upright 실험도 TCP tilt 0.66°, block tilt 11.63°였다. 따라서 TCP가 수직에 가까워도 블록은 손 안에서 회전했다. 이 상관관계만으로 마찰 부족을 확정하지 않는다.

8초 정지 시뮬레이션에서 composed USD를 읽었다. 세 블록의 static friction 0.2, dynamic friction 1.0, restitution 0, contact offset 0.1 m, rest offset 0 m가 확인됐다. 조 source collider의 명시적 재질 및 offset은 null이었다. null은 마찰 0을 의미하지 않으며 런타임 fallback이 사용될 수 있다. instance proxy까지 포함해 접촉 진단을 확장했다.

설치된 Isaac `DynamicCuboid`가 `FixedCuboid`의 기본 재질 및 offset 설정을 상속하는 것은 코드로 확인했다. 이 기본 접촉 생성 거리가 30 mm 블록에 비해 크므로, 단일 변수 비교 대상으로 선택했다. contact offset은 접촉 힘이 항상 해당 거리에서 생긴다는 의미가 아니다.

## 설계와 검증

`--block-contact-offset-m`는 기본 -1로 기존 설정을 유지하며, 실험 override는 (0, 0.01] m만 허용한다. collider의 실제 재질·마찰·offset을 `PALLET_CONTACT_AUDIT`로 기록한다. 설정 변경은 시뮬레이션 블록에 한정한다.

`--block-static-friction` 역시 기본 -1로 기존 재질을 유지한다. 실험 override는 [0, 2]만 허용하며, physics material binding이 없으면 오류로 중단한다. 동마찰·조 힘 제한은 변경하지 않는다.

Python 문법, 셸 문법, diff check 통과. 기존 시뮬레이션 테스트 41개 통과. 첫 8초 진단 실행 exit code 0.

## 실행 순서

```bash
cd /home/minwoo/Development/LLM
ros2_ur_ws/scripts/run_palletizing_gui.sh --headless --task-spec ros2_ur_ws/scripts/pallet_task_stage1_red.json --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/stage1-offset002-20261009 --velocity-scaling 0.3 --acceleration-scaling 0.05 --gripper-close-position 0.002 --grasp-angle-delta 6.283185307179586 --place-yaw 0.0 --upright-transfer --block-contact-offset-m 0.002
```

정지 마찰 비교는 같은 명령에 `--block-static-friction 1.0`을 추가하고 output-dir를 `stage1-offset002-static100-20261009`로 바꾼다.

## 실제 실행 결과

| 조건 | 계획 | 감지 순간 상대 변위 | 최대 상대 변위 | 판정 |
| --- | --- | --- | --- | --- |
| 기준 offset 100 mm / static 0.2 | 성공 | 7.835 mm | 8.217 mm | SLIP, 안전 중단 |
| offset 2 mm / static 0.2 | 첫 계획 실패, 재계획 성공 | 6.212 mm | 6.745 mm | SLIP, 안전 중단 |
| offset 2 mm / static 1.0 | 첫 계획 성공 | 7.747 mm | 8.311 mm | SLIP, 안전 중단 |

마지막 실험은 런타임 audit에서 block static/dynamic friction 1.0/1.0, contact offset 0.002 m를 확인했다. 세 실험 모두 `execution_dynamics`, MTC return code 2이며 목표 배치에는 도달하지 못했다. OMPL 경로가 실행마다 달라 수치 차이만으로 개선·악화를 확정하지 않는다.

offset 2 mm / static 1.0 실험의 첫 6 mm 초과 시각 76.327초에서 TCP 기준 변위는 X -0.242 mm, Y -3.993 mm, Z -5.071 mm, 블록 tilt 9.79°, J8/J9 effort 17.26/18.52 N이었다. 정지 마찰 증가만으로는 운반 안정화를 달성하지 못했다.

결과 원본: `ros2_ur_ws/regression_results/stage1-offset002-20261009/job/report.json`, `ros2_ur_ws/regression_results/stage1-offset002-static100-20261009/job/report.json` 및 각각의 motion.json. 접촉 audit은 Isaac kit log에도 저장된다. 대용량 regression 산출물은 커밋하지 않는다. 실제 장비는 구동하지 않았으며 변경한 실험값을 실기 마찰값으로 간주하지 않는다.

## 다음 작업 로드맵

- [x] 재사용 가능한 red TaskSpec 저장
- [x] 기존 두 upright 실험의 첫 threshold 초과 비교
- [x] 런타임 physics 재질·offset 확인
- [x] 단일 변수 override 구현과 문법/기존 테스트 검증
- [x] offset 비교 결과 판정: 최초 계획 실패 후 두 번째 실행에서 6.212 mm slip, return code 2. 성공 아님.
- [x] offset 2 mm 고정 후 block static friction 0.2 → 1.0 비교: 7.747 mm slip, 성공 아님
- [x] 다음 후보 선택: 조의 명시적 physics material binding (현재 null), 접촉 형상 확인 병행
- [x] 조 재질 binding 명시 및 collider audit 확인 후 동일 red 실험: slip 7.670 mm, 실패
- [x] 실제 패드 크기·평행 정렬 확인 및 성공/실패 상대 회전 비교 (원인 확정은 아님)
- [ ] 경로 차이를 통제한 반복 비교: 단일 실행 수치만으로 개선 주장 금지
- [ ] 단독 3회 연속 및 다중 작업 검증

## 집게 재질 명시 실험

`--jaw-friction` 실험 옵션을 추가했다. 기본 -1은 재질 binding을 변경하지 않는다. [0, 2]의 유한한 값만 허용하며 양쪽 조의 정·동마찰을 같은 값으로 명시하고 restitution은 0으로 설정한다. collider에 재질이 적용되지 않거나 양쪽 collider를 확인하지 못하면 동작 전에 중단한다. 이는 실기 측정값이 아닌 시뮬레이션 민감도 실험이다.

첫 실행에서 importer 반환 경로는 `/pallet_gripper/root_joint`로 확인됐다. 이를 링크 부모로 사용한 것이 시작 실패의 원인이었고, 실제 부모 prim에서 J8/J9 링크를 찾아 binding하도록 수정했다. 최초 실패 로그는 `stage1-offset002-static100-jaw100-20261009/simulation.log`에 보존했다. 로봇 동작 전 실패였다.

재실행 `stage1-offset002-static100-jaw100-r2-20261009`에서 실제 `/pallet_gripper/J8/collisions/mesh_0/box`와 J9 collider의 bound material이 `/World/Physics_Materials/jaw_experiment`, 정·동마찰 1.0/1.0으로 확인됐다. 원본 `/colliders/...` template은 null이지만 실제 robot collider의 binding을 확인했다.

잘못된 CLI 값이 Isaac import 전에 거부되는 테스트와 help 무기동 테스트를 추가했다. 관련 테스트 43개 통과. 블록 offset 2 mm, 정지 마찰 1.0 및 기존 속도·가속도·조 힘 제한을 유지했다.

최종 결과: 첫 계획·실행 후 47.951초에 SLIP 7.670 mm가 확인됐고 최대 상대 변위 7.686 mm, 최대 jaw asymmetry 0.908 mm였다. MTC return code 2, `execution_dynamics`, 완료 0/1로 안전 중단됐다. 재질 명시만으로 운반 안정화를 달성하지 못했으므로 기본 설정으로 채택하지 않는다. 다음은 패드 방향·위치와 이송 회전의 접촉 관계 조사이며, 성공 조건을 완화하지 않는다.

```bash
cd /home/minwoo/Development/LLM
ros2_ur_ws/scripts/run_palletizing_gui.sh --headless --task-spec ros2_ur_ws/scripts/pallet_task_stage1_red.json --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/stage1-offset002-static100-jaw100-r2-20261009 --velocity-scaling 0.3 --acceleration-scaling 0.05 --gripper-close-position 0.002 --grasp-angle-delta 6.283185307179586 --place-yaw 0.0 --upright-transfer --block-contact-offset-m 0.002 --block-static-friction 1.0 --jaw-friction 1.0
```

## Codex 세션 재개

```bash
cd /home/minwoo/Desktop/LLM
codex resume 019f8001-247a-73b2-b9ba-74eed73c3f71
```

## 접촉 기하 런타임 검증

작성일: 2026-10-09. 기준 workspace는 동일하다. 패드 방향 오류 여부와 운반 회전을 비교하기 위해 선택형 `--contact-geometry-log`를 추가했다. 기본 false이며, wrapper에서는 옵션을 추가하면 활성화된다. 매초 실제 USD collider 중심·세 축·크기, 같은 프레임의 블록 중심·방향·조 위치를 `PALLET_CONTACT_GEOMETRY` JSON으로 기록한다. 좌표는 base_footprint, 길이는 m, 블록 quaternion은 wxyz다. 두 Cube collider가 아니거나 변환이 유효하지 않으면 오류로 중단한다. 실제 접촉점이나 접촉 충격량을 측정하는 기능은 아니다.

URDF 순기구학으로 계산한 pad 중심은 TCP 기준 J8 (-0.006, 0, -0.014545) m, J9 (0.006, 0, -0.014545) m이며 각각 개방량에 따라 ±X로 이동한다. 법선은 ±X로 평행하다. 실제 런타임 크기는 12×22×30 mm, 관찰한 전체 구간의 법선 평행 오차는 최대 0.000034°였다. importer의 크기·정렬 오류는 이번 증거에서 발견되지 않아 패드 형상을 변경하지 않았다.

기하 로그를 켠 첫 실행 `stage1-pad-geometry-20261009`는 첫 시도 1/1 통과했다. 최종 XY 오차 2.966 mm, release 감지, 재접촉·runtime fault·safety event 없음. 최대 적재 상대 변위는 6.026 mm였지만 6 mm를 넘은 적재 샘플이 하나여서 기존 3회 연속 기준의 SLIP은 발생하지 않았다. '6 mm를 한 번도 넘지 않음'으로 표현하면 안 된다.

동일한 물리 설정의 직전 실패 실행은 최대 적재 상대 변위 7.686 mm, 초과 샘플 7개, 파지 기준 대비 상대 회전 변화 9.54°였다. 이번 성공 실행의 최대 변위 시점 상대 회전 변화는 2.22°였다. 전자는 Y/Z 방향 변위가 각각 약 -5.44/-5.43 mm였다. 실행마다 경로가 달라 기하 로그 활성화나 재질 변경의 인과 효과로 확정할 수 없다. 패드 기준의 블록 위치가 큰 yaw 변화 동안 일정한 구간도 있어 전체 yaw 변화량만으로 실패를 설명하지 않는다.

재현 명령은 집게 재질 실험 명령에 `--contact-geometry-log`를 추가하고 output-dir를 `stage1-pad-geometry-20261009`로 바꾼다. 동일 조건의 두 번째 반복은 `stage1-pad-geometry-repeat2-20261009`에 별도 보존한다. 테스트 44개, Python·셸 문법 및 diff check 통과. 실기 구동이나 공개 URDF 수정은 수행하지 않았다.

두 번째 반복 결과는 SLIP 6.888 mm, 최대 상대 변위 7.636 mm, return code 2, `execution_dynamics`, 완료 0/1이다. 이때도 패드 법선 평행 오차는 최대 0.000030°였다. 안전 중단 전 J9 패드 기준 블록 중심은 약 (-20.98, -5.59, -5.16) mm로, 압착 법선 방향 거리는 유지되지만 면 내 Y/Z 방향으로 이동했다. 형상 정렬을 바꾸는 근거는 확보되지 않았고, 동일 조건 반복 성공은 달성하지 못했다.

다음 작업은 성공/실패의 loaded-transfer 진입 자세, IK 분기와 시간별 선형·각속도/가감속 비교다. 경로의 영향과 물리 접촉의 영향을 구분한 뒤 한 가지 변경을 선택한다. 6 mm / 3회 연속 실패 기준은 유지하며 다중 블록 검증은 보류한다.
