# Pallet 다중 이상 감지·Fault Coverage 검증

- 작성일: 2026-10-01
- 작업 목적: 500 g 블록 운반 중 낙하, 기울기, telemetry stale를 debounce 방식으로 감지하고 기존 fail-closed 안전 중단 경로와 연결한다.
- 검증 범위: Isaac Sim only. 실제 로봇 및 CAN 모터는 구동하지 않았다.

## 최종 기준 workspace

`/home/minwoo/Development/LLM`

물리는 기존 기준인 240 Hz를 유지했다. 500 g 시나리오는 arm/cartesian velocity scaling 0.5, acceleration scaling 0.25를 사용한다.

## 현재 주요 파일

- `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/watch_pallet_block.py`
- `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/run_pallet_scenario_matrix.py`
- `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper_moveit_config/test/test_watch_pallet_block.py`
- `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper_moveit_config/test/test_pallet_scenario_matrix.py`

## 문제 → 원인 → 해결 → 검증

### 1. 소스 위치의 정상 접촉을 낙하로 오인할 수 있음

- 문제: grasp 직후 블록 바닥 높이는 테이블과 거의 같으므로 단순 높이 임계값은 즉시 낙하 이벤트를 만든다.
- 원인: 운반 시작 여부와 바닥 접촉 여부를 구분하지 않은 절대 높이 판정이다.
- 해결: 양쪽 jaw load가 확인된 뒤 블록 바닥이 20 mm 이상 올라간 경우에만 `lift_confirmed`를 설정한다. 이후 목표점에서 50 mm 이상 떨어진 위치에서 바닥 여유가 5 mm 미만인 상태가 3샘플 연속일 때 `BLOCK_DROP_DETECTED`를 발생시킨다.
- 검증: 사전 lift 없는 저고도 샘플은 이벤트가 없고, lift 후 저고도 3샘플에서만 이벤트가 발생하는 단위 테스트를 통과했다.

### 2. 순간적인 자세·telemetry 흔들림을 안전 중단으로 오인할 수 있음

- 문제: 한 프레임의 TF/joint 지연 또는 자세 노이즈로 작업이 불필요하게 중단될 수 있다.
- 원인: 단일 샘플 판정에는 시간적 지속성 조건이 없다.
- 해결: 기울기 20도 초과와 joint/TCP telemetry 비정상을 각각 3샘플 연속 확인한다. 감지는 양쪽 jaw load로 arm되며, 목표 근처에서 `not_loaded` 3샘플이 확인되면 release로 판정하고 disarm한다.
- 검증: tilt, stale, release-disarm 경계조건 단위 테스트와 정상 500 g 3회 회귀를 통과했다.

### 3. 안전 코드별 bridge 처리 경로 확인 필요

- 문제: bridge가 선언한 네 코드가 실제 실행 중 모두 같은 fail-closed 계약을 따르는지 확인해야 했다.
- 원인: 기존 통합 시험은 `SLIP_DETECTED` 중심이었다.
- 해결: scenario runner에 `--inject-safety-code`를 추가하여 `SLIP_DETECTED`, `BLOCK_DROP_DETECTED`, `TILT_DETECTED`, `TELEMETRY_STALE`를 선택적으로 주입할 수 있게 했다.
- 검증: 네 코드 모두 active task를 중단했다. 현재 중단 자세에서는 recovery MTC 경로가 planning 단계에서 거부되었고, trajectory/open을 실행하지 않은 채 measured hold를 다시 latch하여 `refused_relatched`로 종료했다.

## 최종 구조와 설계 결정

- `SLIP_DETECTED`: TCP 대비 블록 상대 변위 5 mm 초과, 3샘플 연속.
- `BLOCK_DROP_DETECTED`: load arm + lift 확인 후, 목표점 밖에서 블록 바닥 여유 5 mm 미만, 3샘플 연속.
- `TILT_DETECTED`: load arm + lift 확인 후, 블록 장축 기울기 20도 초과, 3샘플 연속.
- `TELEMETRY_STALE`: load arm 이후 joint 또는 TCP measurement가 fresh가 아닌 상태, 3샘플 연속.
- release: 목표 XY 50 mm 이내에서 `not_loaded` 3샘플 연속이면 감지기를 disarm한다.
- 이벤트 publisher는 최초 이벤트만 발행한다. bridge도 최초 유효 이벤트를 latch한다.
- recovery plan이 유효하지 않으면 open/detach/home을 강행하지 않고 현재 measured hold로 돌아간다.

## 실행 순서

### 터미널 1: 환경 준비

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 터미널 1: 정상 500 g 반복 회귀

```bash
python3 scripts/run_pallet_scenario_matrix.py \
  --scenario-file scripts/pallet_scenarios_500g.json \
  --scenario mass_500_release_stable --repeat 3 \
  --safety-cancel-enabled \
  --startup-timeout 240 --task-timeout 420 --monitor-seconds 420 \
  --settle-seconds 3 --min-clearance 0.020 \
  --slip-threshold 0.005 --slip-confirm-samples 3 \
  --drop-clearance-threshold 0.005 \
  --tilt-threshold-deg 20 --fault-confirm-samples 3 \
  --output-dir /tmp/pallet-multifault-normal-repeat3
```

### 터미널 1: 안전 코드별 주입 시험

아래의 `CODE`를 네 값 중 하나로 바꾸고, 매번 새로운 output directory를 지정한다.

```bash
CODE=BLOCK_DROP_DETECTED
python3 scripts/run_pallet_scenario_matrix.py \
  --scenario-file scripts/pallet_scenarios_500g.json \
  --scenario mass_500_release_stable --repeat 1 \
  --safety-cancel-enabled --expect-safety-stop \
  --inject-safety-event-after-contact 0.5 \
  --inject-safety-code "$CODE" \
  --startup-timeout 240 --task-timeout 420 --monitor-seconds 420 \
  --recovery-timeout 180 --settle-seconds 3 --min-clearance 0.020 \
  --output-dir "/tmp/pallet-fault-${CODE}"
```

허용값: `SLIP_DETECTED`, `BLOCK_DROP_DETECTED`, `TILT_DETECTED`, `TELEMETRY_STALE`.

## 검증 명령과 결과

```bash
cd /home/minwoo/Development/LLM
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s ros2_ur_ws/src/pallet_gripper_moveit_config/test -p 'test_*.py'
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s ros2_ur_ws/src/mtc_tutorial/test -p 'test_*.py'
python3 -m py_compile \
  ros2_ur_ws/scripts/watch_pallet_block.py \
  ros2_ur_ws/scripts/run_pallet_scenario_matrix.py
```

- pallet_gripper_moveit_config: 28 tests passed.
- mtc_tutorial: 5 tests passed.
- Python compile 및 `git diff --check`: 통과.
- 안전 코드 주입: 4/4 중단 경로 통과. 세 신규 코드의 결과는 모두 `refused_relatched`; 기존 slip 통합 시험도 통과.
- 정상 500 g: 3/3 성공, safety event 0/3, post-release re-contact 0/3.
- 최소 운반 바닥 여유: 27.417 mm.
- 최대 최종 XY 오차: 1.966 mm.
- 최대 loaded jaw 비대칭: 1.722 mm.
- 최대 release 후 블록 변위: 0.000054 mm.

결과 파일:

- `/tmp/pallet-safety-failclosed-20261001/report.json`
- `/tmp/pallet-fault-drop-20261001/report.json`
- `/tmp/pallet-fault-tilt-20261001/report.json`
- `/tmp/pallet-fault-stale-20261001/report.json`
- `/tmp/pallet-multifault-normal-repeat3-20261001/report.json`

## Codex 세션 재개

```bash
cd /home/minwoo/Desktop/LLM
codex resume 019f8001-247a-73b2-b9ba-74eed73c3f71
```

## 남은 주의사항

- 현재 telemetry stale 판정은 block pose 샘플이 계속 들어오는 상황에서 joint/TF freshness를 검사한다. block pose stream 자체가 완전히 끊기는 경우를 위한 wall-clock watchdog은 다음 단계에서 추가해야 한다.
- 주입 시험은 이벤트 계약과 bridge 반응을 검증한다. 실제 물체를 의도적으로 떨어뜨리거나 기울인 physics fault injection은 아직 수행하지 않았다.
- recovery planner는 시험한 중단 자세에서 유효 경로를 찾지 못했다. 안전하게 hold하지만 자동 복구 동작이 완료된 것은 아니다.
- 실제 장비 임계값은 gripper current/force와 pose noise를 측정한 뒤 별도로 보정해야 한다.

## 다음 작업 로드맵

- [ ] block pose stream 자체의 wall-clock stale watchdog 추가.
- [ ] recovery planning 거부 원인과 충돌 pair를 구조화해 report에 기록.
- [ ] +Z, source retreat, 목표 반대 방향 등 bounded recovery candidate를 순서대로 계획하고 collision-valid 후보만 실행.
- [ ] physics fault injector로 실제 drop/tilt/stale 조건에서 자동 감지 이벤트 검증.
- [ ] 실제 gripper telemetry 계약과 안전 한계 정의 후 hardware-in-the-loop 단계로 승격.
