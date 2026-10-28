# Pallet 실제 Fault Injection·다중 복구·Planning 최적화

- 작성일: 2026-10-01
- 목적: 장기 시나리오 매트릭스 전에 watchdog, 실제 simulation fault, 복구 진단, 다중 복구 후보, 중단 시점 반복성과 planning 속도를 검증한다.
- 기준 workspace: `/home/minwoo/Development/LLM`
- 범위: Isaac Sim only. 실제 로봇과 CAN 모터는 구동하지 않았다.

## 주요 변경 파일

- `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/watch_pallet_block.py`
- `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/run_pallet_scenario_matrix.py`
- `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/isaac_sim.py`
- `/home/minwoo/Development/LLM/ros2_ur_ws/src/mtc_tutorial/src/mtc_tutorial.cpp`
- `/home/minwoo/Development/LLM/ros2_ur_ws/src/mtc_tutorial/config/robot_profiles.yaml`

## 문제 → 원인 → 해결 → 검증

### Block pose stream 완전 단절

- 문제: pose callback이 완전히 멈추면 sample 기반 stale 판정도 실행되지 않았다.
- 원인: 기존 검사는 새 pose sample 내부에서만 joint/TF freshness를 확인했다.
- 해결: load로 arm된 뒤 마지막 block pose 수신 시각이 0.75초를 넘으면 wall-clock watchdog이 `TELEMETRY_STALE`을 발생시킨다. 정상 release 뒤에는 disarm한다.
- 검증: Isaac pose publication을 2초간 실제 차단하여 0.764초 age에서 `injected=false` stale 이벤트와 안전 중단을 확인했다.

### 실제 drop·tilt·stale fault 검증

- 문제: 이전 시험은 safety event를 직접 주입했으므로 detector 자체를 검증하지 못했다.
- 해결: 기본 비활성인 `/pallet/sim_fault_commands` test endpoint를 추가했다. drop은 block pose/velocity, tilt는 30도 pose/angular velocity, stale은 pose publication을 변경한다.
- 검증: 실제 상태 변경에서 `BLOCK_DROP_DETECTED`, `TILT_DETECTED`, `TELEMETRY_STALE`이 각각 올바르게 발생했으며 세 run 모두 통과했다.

### 복구 실패 원인과 다중 후보

- 문제: recovery 실패가 문자열 하나로만 보고되고 +Z 후보 하나만 존재했다.
- 해결: vertical, reverse-transfer, lateral-left, lateral-right 네 bounded candidate를 순서대로 planning한다. planning-only safe refusal일 때만 다음 후보를 시도하며 실행 시작 후 실패하면 중단한다. 후보별 시간, 방향, 실행 여부, 실패 분류와 근거를 `recovery_attempts`에 기록한다.
- 검증: phase matrix 15회에서 60개 후보가 모두 `CartesianPath min_fraction 0`으로 안전 거부되었고 recovery trajectory/open 실행은 0건이었다. 마지막에 measured hold를 재래치했다.

### MTC planning 시간 편차

- 문제: 15회 기준 planning 0.829~83.839초, 평균 24.783초였다.
- 원인: self-collision `J3-J5`인 place IK branch에 긴 OMPL 연결 시간이 반복 사용되었다.
- 해결: Pallet profile의 `place_max_ik_solutions`를 8→2, `move_to_place_timeout`을 20→5초로 제한했다.
- 검증: 정상 500g 3/3 성공. planning 0.743/5.595/15.532초, 평균 7.290초. 기준 대비 평균 70.6%, 최악값 81.5% 감소했다.

## 실행 명령

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

실제 fault 1회:

```bash
python3 scripts/run_pallet_scenario_matrix.py \
  --scenario-file scripts/pallet_scenarios_500g.json \
  --scenario mass_500_release_stable --repeat 1 \
  --safety-cancel-enabled --expect-safety-stop \
  --inject-sim-fault-phase transfer \
  --inject-sim-fault-code TELEMETRY_STALE \
  --pose-stream-timeout 0.75 \
  --startup-timeout 240 --task-timeout 420 --monitor-seconds 420 \
  --recovery-timeout 60 --settle-seconds 3 --min-clearance 0.020 \
  --output-dir /tmp/pallet-physical-stale
```

phase별 3회 반복:

```bash
for phase in after_contact after_lift transfer lower pre_release; do
  python3 scripts/run_pallet_scenario_matrix.py \
    --scenario-file scripts/pallet_scenarios_500g.json \
    --scenario mass_500_release_stable --repeat 3 \
    --safety-cancel-enabled --expect-safety-stop \
    --inject-sim-fault-phase "$phase" \
    --inject-sim-fault-code TELEMETRY_STALE \
    --pose-stream-timeout 0.75 \
    --startup-timeout 240 --task-timeout 420 --monitor-seconds 420 \
    --recovery-timeout 60 --settle-seconds 3 --min-clearance 0.020 \
    --output-dir "/tmp/pallet-phase-${phase}-repeat3"
done
```

최적화 정상 회귀:

```bash
python3 scripts/run_pallet_scenario_matrix.py \
  --scenario-file scripts/pallet_scenarios_500g.json \
  --scenario mass_500_release_stable --repeat 3 \
  --safety-cancel-enabled --pose-stream-timeout 0.75 \
  --startup-timeout 240 --task-timeout 300 --monitor-seconds 300 \
  --settle-seconds 3 --min-clearance 0.020 \
  --output-dir /tmp/pallet-planning-optimized-v2-repeat3
```

## 검증 결과

- Python/pallet tests: 32 passed.
- MTC profile tests: 5 passed.
- MTC package build: passed.
- Python compile, `git diff --check`: passed.
- 실제 detector fault: drop/tilt/stale 3/3 통과.
- 중단 시점: 5 phases × 3회 = 15/15 통과.
- recovery 후보: 60/60 safe refusal, 실행 시작 0건, fallback hold 15/15.
- 최적화 정상 회귀: 3/3, safety event 0건, release 후 재접촉 0건.
- 최적화 후 최대 XY 오차 2.156 mm, 최소 운반 여유 27.482 mm.

주요 report:

- `/tmp/pallet-physical-drop-final-20261001/report.json`
- `/tmp/pallet-physical-tilt-20261001/report.json`
- `/tmp/pallet-physical-stale-20261001/report.json`
- `/tmp/pallet-phase-{phase}-repeat3-20261001/report.json`
- `/tmp/pallet-planning-optimized-v2-repeat3-20261001/report.json`

## Codex 세션 재개

```bash
cd /home/minwoo/Desktop/LLM
codex resume 019f8001-247a-73b2-b9ba-74eed73c3f71
```

## 남은 주의사항

- 네 recovery candidate가 시험한 모든 중단 자세에서 0% Cartesian path로 거부됐다. fail-closed 안전성은 확인했지만 자동 복구 동작 성공은 아니다.
- simulator fault endpoint는 명시적으로 enable된 시험 run에서만 활성화된다.
- 실제 장비 임계값은 gripper current/force와 센서 지연을 측정한 뒤 다시 보정해야 한다.

## 다음 작업 로드맵

- [x] Pose stream watchdog
- [x] 실제 simulation drop/tilt/stale fault
- [x] 구조화 recovery diagnostics
- [x] 다중 bounded recovery candidates
- [x] 중단 시점별 3회 반복
- [x] MTC planning 시간 단축
- [ ] 장시간·다중 시나리오 매트릭스 회귀
- [ ] 실제 장비 전환 준비
