# Pallet TCP–블록 동기 계측과 slip 판정 기준

## 목적

Isaac Sim의 Pallet pick-place에서 MTC 완료와 최종 배치 결과만으로는 운반 중 미끄러짐 시작 시점을 알 수 없었다. `watch_pallet_block.py`가 블록 pose, TCP TF, J8/J9 위치·속도·effort를 같은 timestamp 기준으로 결합하고, 안정된 양측 접촉이 처음 확인된 시점부터 TCP 좌표계의 블록 상대변위를 계산하도록 했다.

이 작업은 실물 로봇·모터·CAN을 사용하지 않았다. 아래 수치는 Isaac 평면 접촉 패드와 현재 drive 모델에서만 유효하다.

## 계측 계약

- 블록 기준 토픽: `/isaac_calibration_block_pose`, frame `base_footprint`.
- TCP: TF `base_footprint → tcp`를 블록 pose의 timestamp로 조회한다.
- 턱 피드백: `/isaac_joint_states`에서 블록 pose timestamp와 가장 가까운 J8/J9 샘플을 선택한다.
- 150 ms 지연 큐를 사용해 해당 시각의 TF와 관절 피드백이 도착할 시간을 확보한다.
- TF와 관절 정렬 오차를 각 샘플에 기록한다. 없거나 0.5초를 초과하면 `unavailable` 또는 `stale`로 남기며 성공값처럼 사용하지 않는다.
- 최초 `bilateral_stationary_loaded` 샘플을 접촉 기준점으로 사용한다.
- 턱이 움직이는 `bilateral_loaded` 구간은 transient가 크므로 slip 판정에서 제외한다.

## timestamp 오류와 수정

초기 구현은 블록 pose callback에서 최신 TCP TF를 읽었다. 첫 baseline 성공 실행에서 접촉 중 최대 상대변위가 14.20 mm로 나타났지만, 최대 지점의 TF가 블록보다 약 94 ms 오래돼 팔 이동량이 상대변위로 섞인 값이었다.

블록 시각 기준 TF 조회와 최근접 관절 샘플 선택으로 수정한 뒤 baseline의 안정 접촉 최대 상대변위는 3.20 mm로 줄었다. 최대 지점의 TF 정렬 오차는 0 ms, 관절 정렬 오차는 0.36 ms였다.

## 동기 계측 비교 결과

| 조건 | 결과 | 안정 접촉 최대 상대변위 | 최종 XY 오차 | 운반 최소 하단 간격 |
| --- | --- | ---: | ---: | ---: |
| 50 g baseline, 기본 프로파일 | 성공 | 3.20 mm | 1.13 mm | 25.09 mm |
| 500 g, 강한 파지+저속 | 성공 | 1.75 mm | 2.40 mm | 27.49 mm |
| 500 g, 강한 파지+기본 속도 #1 | 실패 | 8.69 mm | 590.42 mm | 약 0 mm |
| 500 g, 강한 파지+기본 속도 #2 | 실패 | 25.30 mm | 25.11 mm | 25.15 mm |
| 500 g, 강한 파지+기본 속도 #3 | 실패 | 33.60 mm | 637.51 mm | 약 0 mm |

기본 속도 500 g은 3/3 실패했다. #1과 #3은 크게 이탈했고, #2는 테이블 위에 놓였지만 XY 20 mm 허용값을 넘었다. MTC 자식 노드는 세 실행 모두 완료 메시지를 남겼으므로 최종 제어 성공만으로 물리 성공을 판단할 수 없다.

## 임시 관측 임계값

동기 계측 성공 실행의 최대값은 3.20 mm, 실패 실행의 최소값은 8.69 mm였다. 두 집합 사이의 5 mm를 **관측용 임시 slip threshold**로 사용한다.

- 성공 2회: 5 mm 초과 없음.
- 실패 3회: 안정 접촉 상태에서 모두 5 mm 초과 검출.
- 최초 검출 시각: 실패 #1 125.86 s, #2 64.00 s, #3 26.13 s.

이 임계값은 아직 실행 취소용 안전 한계가 아니다. 성공 표본이 50 g 1회와 500 g 저속 1회뿐이며 위치·경로·블록 폭 변동을 충분히 포함하지 않았다. 우선 보고서에 `slip_threshold_m`, `slip_detected`, `slip_event`를 기록하고 추가 성공 회귀에서 오탐이 없는지 확인한다.

## 구현과 검증

- `ros2_ur_ws/scripts/watch_pallet_block.py`: timestamp 동기화, TCP 상대변위, 접촉 상태, slip 이벤트 기록.
- `ros2_ur_ws/scripts/run_pallet_scenario_matrix.py`: `--slip-threshold` 전달과 상대변위 통계 집계.
- `ros2_ur_ws/scripts/pallet_scenarios_500g.json`: 강한 파지·기본 속도 `mass_500_fast` 재현 조건 추가.
- `ros2_ur_ws/src/pallet_gripper_moveit_config/test/test_watch_pallet_block.py`: 좌표 변환, 최근접 샘플, 접촉 상태, slip 판정 단위 테스트.
- 계산·회귀 unittest 10개, Python 구문 검사와 `git diff --check`를 통과했다.

원시 결과:

- 정상 baseline: `/tmp/pallet-telemetry-baseline-synced`
- 500 g 저속 성공: `/tmp/pallet-telemetry-mass500-slow`
- 500 g 기본 속도 3회 실패: `/tmp/pallet-telemetry-mass500-fast`

`/tmp` 원본은 재부팅이나 시스템 정리로 사라질 수 있다. 재현 조건과 핵심 수치는 이 문서에 보존했다.

## 다음 작업

## 3샘플 debounce 구현과 성공 회귀

단일 5 mm 초과만 사용하면 50 g 성공 baseline에서도 턱이 움직이는 동안 한 샘플이 8.01 mm에 도달해 오탐이 발생했다. 반대로 `bilateral_stationary_loaded`만 연속으로 요구하면 실제 slip 중 턱 속도 변화로 상태가 흔들려 실패를 놓쳤다.

다음 규칙을 채택했다.

1. 접촉 기준점은 최초 `bilateral_stationary_loaded`에서만 생성한다.
2. 이후 `bilateral_loaded` 또는 `bilateral_stationary_loaded` 상태에서 상대변위 5 mm 초과가 3개 샘플 연속일 때 확정한다.
3. 현재 블록 관측 주기는 약 10 Hz이므로 확인 지연은 약 0.2초다.
4. effort가 사라지거나 변위가 임계값 이하로 돌아오면 연속 횟수를 초기화한다.

기존 원시 데이터 재평가에서 성공 2회는 무검출, 실패 3회는 모두 검출됐다. 실패의 첫 초과부터 확정까지 약 0.21초가 걸렸다.

신규 성공 회귀는 다음 명령으로 실행했다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_pallet_scenario_matrix.py \
  --scenario baseline --scenario offset --scenario narrow --scenario mass_065 \
  --repeat 1 --slip-threshold 0.005 --slip-confirm-samples 3 \
  --task-timeout 300 --monitor-seconds 360 \
  --output-dir /tmp/pallet-slip-debounce-success-matrix
```

| 조건 | 배치 | slip 검출 | 안정 접촉 최대 상대변위 | 최종 XY 오차 | 최소 하단 간격 |
| --- | --- | --- | ---: | ---: | ---: |
| baseline | 성공 | 없음 | 5.28 mm | 1.18 mm | 25.10 mm |
| offset | 성공 | 없음 | 3.21 mm | 1.41 mm | 28.14 mm |
| narrow | 성공 | 없음 | 5.41 mm | 0.82 mm | 25.31 mm |
| mass 0.065 kg | 성공 | 없음 | 2.69 mm | 3.49 mm | 27.72 mm |

baseline과 narrow는 최대값이 5 mm를 잠깐 넘었지만 연속 3샘플 조건을 충족하지 않아 오탐 없이 통과했다. 신규 회귀 결과는 `/tmp/pallet-slip-debounce-success-matrix/report.json`에 있다.

## 다음 작업

1. 검증된 debounce 이벤트를 우선 관측 전용 safety event 토픽으로 발행한다.
2. MTC/arm action 취소 경로는 별도 opt-in 옵션으로 추가하고 실패 주입 시험에서만 활성화한다.
3. 취소 후 gripper hold, 수직 retreat 또는 즉시 정지 중 어떤 복구가 현재 상태에서 안전한지 별도 상태 머신으로 정의한다.
## Isaac gripper effort calibration candidate

The Notion mechanism-ratio note proposes 60 N/finger as an initial Isaac
sensitivity candidate. J8/J9 are prismatic simulation joints, so this is a
linear effort ceiling in newtons, not motor torque in N·m. The source URDF
remains at 10 N for both jaws; `isaac_physics.yaml` overrides only the imported
Isaac articulation.

Validation passed: Python compilation, 14 MoveIt-config unit tests, the Pallet
joint-contract audit, and `git diff --check`. A 500 g fast scenario was started
and its Isaac log confirmed
`PALLET_SIM_GRIPPER_EFFORT_LIMIT_N_PER_FINGER 60.0`, but the host reported
`NVML_ERROR_DRIVER_NOT_LOADED`. No `/isaac_joint_states` arrived before the
180-second startup timeout, so no contact-force or slip result was produced.

Re-run after the NVIDIA driver is available:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_pallet_scenario_matrix.py \
  --scenario-file scripts/pallet_scenarios_500g.json \
  --scenario mass_500_fast --repeat 1 \
  --slip-threshold 0.005 --slip-confirm-samples 3 \
  --task-timeout 300 --monitor-seconds 360 \
  --output-dir /tmp/pallet-gripper-60n-mass500-fast-rerun
```
