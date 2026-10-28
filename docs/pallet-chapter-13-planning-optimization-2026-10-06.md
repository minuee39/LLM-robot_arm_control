# Pallet 챕터 13 — 500 g 계획 시간 및 위치 스트림 감시 검증

작성일: 2026-10-06. 목적: 500 g·30 mm pick/place의 MTC 계획 제한을 비교하고, 최종 안전 감시 조건으로 10회 반복 및 지속적인 위치 데이터 중단 시 정지를 검증한다.

기준 workspace: `/home/minwoo/Development/LLM`.

## 현재 주요 패키지와 파일

- 시나리오 실행기: `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/run_pallet_scenario_matrix.py`
- 비교 설정: `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/pallet_scenarios_planning_compare.json`
- 물리 계측·안전 감시: `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/watch_pallet_block.py`
- 최종 10회 보고서: `/home/minwoo/Development/LLM/ros2_ur_ws/regression_results/planning-bounded-final-watchdog-20261006/report.json`
- 폭별 비교 설정: `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/pallet_scenarios_planning_widths.json`
- 폭별 3회 보고서: `/home/minwoo/Development/LLM/ros2_ur_ws/regression_results/planning-widths-pilot-20261006/report.json`
- 지속 중단 시험: `/home/minwoo/Development/LLM/ros2_ur_ws/regression_results/planning-stale-watchdog-time-20261006/report.json`

## 작업 중 문제 → 원인 → 해결 → 검증

1. 첫 비교 보고서에서 기존 설정 3/3, 단계별 계획 제한 3/2/2 s의 단축안 3/3을 얻었다. 추가 반복 중 한 번은 제한된 프로세스가 GPU 장치에 접근하지 못해 Isaac가 시작되지 않았다. 드라이버 장애로 오판했으나 동일 호스트의 장치 접근 환경에서 `nvidia-smi -L`로 RTX 5070 Laptop GPU를 확인하고 재실행이 통과했다. 해당 실패는 `planning-compare-20261006/report.json`의 `prior_attempts`에 남겼다.
2. 기존 위치 스트림 감시는 로드 상태에서 블록 pose 메시지 나이가 0.75 s를 한 번 넘으면 즉시 `TELEMETRY_STALE`를 발행했다. 8회차에서 0.778 s의 일회성 간격으로 작업이 취소됐다. 감시기의 샘플 횟수만 추가했을 때도 세 번의 감시 호출이 0.026 s에 몰릴 수 있음을 물리 중단 시험에서 확인했다. 최종 구조는 `fault_confirm_samples=3`과 경계 초과 0.15 s 지속을 모두 요구하며, 새 pose가 오면 카운터를 초기화한다.
3. 2 s 위치 메시지 중단 주입에서는 감시 이벤트가 스트림 나이 약 0.917 s에 발생했다. `TELEMETRY_STALE`가 발행되고 진행 중인 작업이 취소됐으며 단계형 복구가 완료됐다. 따라서 최종 감시가 지속 중단을 무시하지 않는 것을 물리 시뮬레이션에서 확인했다.
4. 중단 직후 보고서 전체 판정·통계가 오래된 값으로 남던 문제는 회차별 원자적 집계로 해결했다. `--resume`은 기록된 시나리오 설정과 현재 설정이 다르면 이전 성공 회차를 재사용하지 않는다. Isaac 실행 전 GPU 접근도 확인한다.

## 최종 구조와 설계 결정

기존 설정은 500 g·30 mm, arm 0.5, Cartesian 0.35, 가속도 0.2, 계획 제한 집기/놓기/복귀 10/5/5 s다. 단축안은 같은 형상·질량·속도·가속도에서 계획 제한만 3/2/2 s로 바꾼다. 제한 시간은 경로 계산에 관한 값이지 로봇 운동 속도 배율이 아니다.

초기 비교의 기존 설정 3회는 모두 통과했고 평균 계획 12.6277 s, 실행 80.6553 s였다. 최종 감시 조건으로 별도 실행한 단축안 10회는 **첫 시도 10/10 통과**, 평균 계획 5.5337 s(범위 0.541~11.182 s), 평균 실행 79.0054 s였다. 최종 XY 오차 평균은 1.156 mm, 운반 중 최소 바닥 여유 평균은 26.787 mm, 안전 오탐 및 정리 실패는 각각 0건이다. 표본 수 3회와 10회가 다르므로 5.5337 s 대 12.6277 s는 관측 평균 비교이지 보편적 성능 보장은 아니다.

동일한 계획 제한을 기존 장시간 회귀의 다른 폭에도 적용했다. 25 mm는 첫 시도 3/3, 평균 계획 1.2883 s·실행 157.4073 s였다. 40 mm는 첫 시도 3/3, 평균 계획 4.8873 s·실행 166.6867 s였다. 두 형상 모두 안전 오탐·정리 실패 0건이다. 이전 보수 설정의 각 3회 평균 계획 시간은 25 mm 12.0263 s, 40 mm 10.7960 s였다. 계획 시간이 단축된 관측 결과지만 계획기의 확률적 변동이 있으므로 이 작은 표본으로 속도 향상률을 보장하지 않는다. 폭별 형상·그립·운동 속도는 장시간 회귀와 같고 계획 제한만 3/2/2 s로 바꿨다.

## 터미널별 실행 순서

터미널 하나에서 ROS 환경을 준비하고, GPU가 보이는 세션에서 최종 반복을 실행한다.

```bash
cd /home/minwoo/Development/LLM
source /opt/ros/jazzy/setup.bash
source ros2_ur_ws/install/setup.bash
nvidia-smi -L
python3 ros2_ur_ws/scripts/run_pallet_scenario_matrix.py \
  --scenario-file ros2_ur_ws/scripts/pallet_scenarios_planning_compare.json \
  --scenario planning_bounded_500g --repeat 10 \
  --task-timeout 300 --monitor-seconds 330 --startup-timeout 240 \
  --inter-run-cooldown 10 --slip-threshold 0.006 \
  --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/planning-bounded-final-watchdog-20261006
```

위 출력 디렉터리가 이미 있으면 새 디렉터리를 지정한다. 실패 회차만 다시 실행할 때에만 동일 디렉터리와 `--resume`을 사용한다. 최종 10/10 첫 시도 보고서는 재실행하지 않는다.

폭별 반복도 별도 디렉터리에서 수행한다.

```bash
python3 ros2_ur_ws/scripts/run_pallet_scenario_matrix.py \
  --scenario-file ros2_ur_ws/scripts/pallet_scenarios_planning_widths.json \
  --repeat 3 --task-timeout 300 --monitor-seconds 330 \
  --startup-timeout 240 --inter-run-cooldown 10 --slip-threshold 0.006 \
  --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/planning-widths-pilot-20261006
```

지속 중단 안전 시험은 별도 디렉터리에서 수행한다.

```bash
python3 ros2_ur_ws/scripts/run_pallet_scenario_matrix.py \
  --scenario-file ros2_ur_ws/scripts/pallet_scenarios_500g.json \
  --scenario mass_500_release_stable --repeat 1 \
  --safety-cancel-enabled --expect-safety-stop \
  --inject-sim-fault-phase transfer --inject-sim-fault-code TELEMETRY_STALE \
  --pose-stream-timeout 0.75 --startup-timeout 240 \
  --task-timeout 420 --monitor-seconds 420 --recovery-timeout 120 \
  --settle-seconds 3 --min-clearance 0.020 \
  --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/planning-stale-watchdog-time-20261006
```

## 검증 명령과 결과

```bash
cd /home/minwoo/Development/LLM
python3 -m unittest \
  ros2_ur_ws/src/pallet_gripper_moveit_config/test/test_watch_pallet_block.py \
  ros2_ur_ws/src/pallet_gripper_moveit_config/test/test_pallet_scenario_matrix.py
git diff --check
```

단위 테스트 27개 통과. 최종 10회 보고서 `passed=true`, `success_count=10`, `failure_count=0`, `safety_false_positive_count=0`, `process_cleanup_failure_count=0`. 폭별 보고서 `passed=true`, 25 mm 3/3·40 mm 3/3. 지속 중단 시험 `passed=true`, 안전 이벤트 `TELEMETRY_STALE`, 취소·복구 완료.

## 남은 주의사항과 다음 작업 로드맵

- [x] 동일 500 g·30 mm에서 기존/단축 계획 제한 비교
- [x] 최종 위치 스트림 감시 조건으로 첫 시도 10/10 반복 통과
- [x] 2 s 스트림 중단 시 안전 정지와 단계형 복구 확인
- [x] 500 g의 25 mm·40 mm 폭에서도 단축 계획 제한을 각 3회 검증
- [ ] 100 g·300 g 및 실기 적용 여부는 별도 검증
- [ ] 자연어/LLM → 검증된 작업 명세 연결; OpenAI 키 선택은 사용자 응답 대기
- [ ] 실기 5축 보정·그리퍼·좌표계 실측 및 단계별 구동 검증
- [ ] Notion 기록은 저장소 `AGENTS.md`의 금지 지침이 명시적으로 해제된 경우에만 작성
