# Pallet 로드맵 검증 기록

작성일: 2026-10-06. 목적: 현재 완료 증거와 미완료 범위를 분리해 후속 작업을 재현한다.

기준 workspace: `/home/minwoo/Development/LLM`.

## 장시간 회귀

`ros2_ur_ws/regression_results/chapter-10-soak-final-20261005/report.json`의 최종 유효 9회는 모두 통과했다. 30 mm segmented, 25 mm conservative, 40 mm conservative 각 3회다. 최초 40 mm 계획 실패 1회는 prior_attempts로 보존됐고 재실행에서 통과했다. 따라서 최초 시도 무실패 9/9라는 의미는 아니다.

최종 성공 회차의 계획 시간은 평균 14.7109 s, 0.710~36.747 s였다. 실행 시간은 평균 137.6632 s, 78.051~178.552 s였다. 정상 안전 오탐 및 프로세스 정리 실패는 각각 0건이다.

동일한 500 g·30 mm 조건에서 MTC 단계별 계획 제한을 10/5/5 s(집기/놓기/복귀)에서 3/2/2 s로 줄였다. `ros2_ur_ws/regression_results/planning-compare-20261006/report.json`에서 기존 설정은 3/3 통과, 평균 계획 12.6277 s·실행 80.6553 s였다. 단축안은 최종 유효 10/10 통과, 평균 계획 3.7439 s·실행 79.4195 s이며 최종 XY 오차 평균 1.462 mm, 안전 오탐·정리 실패 각각 0건이다. 서로 다른 3회/10회 표본의 관측 평균 비교이며, 모든 물체 크기에서 동일한 속도 향상을 보장하지 않는다.

첫 시도 실패도 보존했다. 4회차는 제한된 실행 환경의 GPU 장치 접근 실패였으며 드라이버 장애가 아니었다. 8회차는 위치 메시지 간격이 0.778 s로 0.75 s 경계를 한 번 넘어 `TELEMETRY_STALE` 안전 정지가 발생했다. 위치 스트림 감시에 `fault_confirm_samples=3` 및 0.15 s 연속 확인을 적용했다. 8회차는 샘플 횟수 조건 적용 후 재실행해 통과했으며 `prior_attempts`에서 실패를 확인할 수 있다. 최종 시간 조건의 결과는 아래 별도 10회 보고서를 기준으로 판단한다.

`ros2_ur_ws/regression_results/planning-stale-watchdog-time-20261006/report.json`에서는 Isaac가 위치 메시지를 2초간 중단했을 때 약 0.917 s의 스트림 나이에서 `TELEMETRY_STALE`가 확인되고, 작업 취소 및 단계형 복구가 완료됐다. 따라서 짧은 일회성 지연을 거르면서 지속 중단에는 안전 정지하는 두 경로를 각각 검증했다. 모니터/시나리오 단위 테스트 27개가 통과했다.

최종 0.15 s 시간 확인까지 적용한 별도 보고서 `ros2_ur_ws/regression_results/planning-bounded-final-watchdog-20261006/report.json`은 첫 시도 10/10 통과했다. 평균 계획 5.5337 s(0.541~11.182 s), 실행 79.0054 s, 최종 XY 오차 1.156 mm이며 안전 오탐·정리 실패는 각각 0건이다. 자세한 문제 이력, 실행 순서와 검증은 `docs/pallet-chapter-13-planning-optimization-2026-10-06.md`에 기록했다.

동일한 제한을 기존 장시간 회귀의 25 mm·40 mm 조건에도 적용했다. `ros2_ur_ws/regression_results/planning-widths-pilot-20261006/report.json`에서 각각 첫 시도 3/3 통과, 평균 계획 1.2883 s·4.8873 s, 안전 오탐·정리 실패 0건이다. 이는 500 g 세 가지 폭에 대한 증거이며 다른 질량이나 실기 속도로 확장한 결과는 아니다.

문제 → 원인 → 해결 → 검증: GPU 접근 오류 뒤 반복 실행을 중단했을 때 `report.json`의 전체 통과 표시와 통계가 직전 회차로 남았다. 집계가 루프 종료 시에만 갱신되던 것이 원인이었다. 실행기는 이제 회차마다 집계를 원자적으로 갱신하고, 다음 Isaac 시작 전 `nvidia-smi -L`로 해당 프로세스의 GPU 접근을 확인한다. 또한 `--resume`은 기록된 시나리오 파라미터가 현재 설정과 다르면 이전 성공 회차를 재사용하지 않는다. 관련 단위 테스트 12개와 문법 검사가 통과했고, 현재 보고서의 bounded 설정이 기록된 실행과 일치함을 확인했다. 제한된 실행 환경에서는 다음 회차를 시작하지 않고, 장치 접근 환경에서는 GPU가 확인된다. 과거 보고서에는 실행기의 모든 CLI 제한값이 보존되지 않아, 시나리오 이외 조건의 동일성은 이 검증만으로 보장할 수 없다.

```bash
cd /home/minwoo/Development/LLM
source /opt/ros/jazzy/setup.bash
source ros2_ur_ws/install/setup.bash
python3 ros2_ur_ws/scripts/run_pallet_scenario_matrix.py \
  --scenario-file ros2_ur_ws/scripts/pallet_scenarios_planning_compare.json \
  --scenario planning_bounded_500g --repeat 10 --resume \
  --task-timeout 300 --monitor-seconds 330 --startup-timeout 240 \
  --inter-run-cooldown 10 --slip-threshold 0.006 \
  --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/planning-compare-20261006
```

```bash
cd /home/minwoo/Development/LLM
bash ros2_ur_ws/scripts/run_500g_soak_regression.sh \
  --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/chapter-10-soak-final-20261005
```

통과 회차는 건너뛰고 실패 회차를 재실행한다.

## 500 g GUI 실행 프로파일 정정

문제: `run_500g_pick_place_gui.sh`는 v35/v50을 선택했지만 GUI 시나리오 파일에는 v35가 없고 v50/v100 모두 속도·가속도 배율 1.0으로 기록돼 있었다. 원인은 실행 스크립트와 시나리오 파일의 프로파일 불일치다. `ros2_ur_ws/scripts/pallet_scenarios_500g_gui.json`을 기존 속도 실험의 v35(관절·직선 속도 0.35, 가속도 0.2), v50(속도 0.5, 가속도 0.2)과 같게 정정했다. JSON 문법, 속도 실험 설정과의 값 일치, GUI 셸 문법을 검증했다. GUI 실험 보고서 `ros2_ur_ws/regression_results/500g-gui-v35-fixed-20261006/report.json`과 `ros2_ur_ws/regression_results/500g-gui-v50-fixed-20261006/report.json`에서 각각 1/1 배치 통과를 확인했다. 후자는 미끄럼 없음, release 감지 및 재접촉 없음, XY 오차 1.648 mm였다. 실행 로그에서 Isaac 준비 완료와 RViz 프로세스 시작을 확인했다. 이는 GUI 프로세스·물리 결과의 확인이며 화면의 시각적 품질 평가를 뜻하지 않는다.

```bash
cd /home/minwoo/Development/LLM
bash ros2_ur_ws/scripts/run_500g_pick_place_gui.sh --speed-profile validated-fast
# 비교 실험을 원할 때만: --speed-profile experimental-05
```

## 작업 명세 계층

`ur10e/pallet_task_spec.py`는 version/frame/unit/tasks 계약, 유한 좌표, 작업대 경계, 물체 중복, 목적지 40 mm 축 간격을 검증한다. 명세의 좌표 범위 검증은 IK 도달성 보장이 아니며 MTC가 실제 경로를 검증한다.

`ur10e/palletizing_job.py --task-spec`은 검증된 명세를 기존 카메라 기반 실행 경로에 연결한다. GUI wrapper는 Isaac 시작 전에 명세를 검사한다. 관련 unit tests 39개가 통과했고, `ros2_ur_ws/regression_results/task-spec-e2e-20261006/job/report.json`의 명세 기반 3물체 물리 e2e가 첫 시도 3/3 통과했다.

```bash
cd /home/minwoo/Development/LLM
python3 ur10e/pallet_task_spec.py ros2_ur_ws/scripts/pallet_task_three_blocks.json
bash ros2_ur_ws/scripts/run_palletizing_gui.sh \
  --task-spec ros2_ur_ws/scripts/pallet_task_three_blocks.json
```

## 실패 동작

계획/시작 실패 code 1만 제한 재시도한다. 동작 후 물리 배치 검증 실패는 code 2와 recovery_required로 기록하며 작업을 중단한다. 복구 확인 없이 같은 저널의 새 작업을 시작할 수 없다.

## 다음 작업 로드맵

- [x] 500 g 장시간 회귀 최종 유효 9회 확인 및 실패 이력 보존
- [x] 선언형 작업 명세 → 카메라/MTC 실행기 연결
- [x] 명세 경로의 실제 3물체 물리 e2e 완료 확인: task-spec-e2e-20261006/job/report.json, 3/3 첫 시도 통과
- [x] Gemini 자연어 → 검증된 작업 명세 생성 및 통합 실행 연결
- [x] MTC 계획 시간 단축 후보를 동일 500 g·30 mm 조건으로 비교하고 최종 유효 10회 검증
- [ ] 실기 5축과 시뮬레이션 6축/그리퍼 모델 매핑 및 보정
- [ ] 속도·가속도 0.5의 2물체 통합 동역학 실패 수정 및 재검증
- [ ] Notion 기록 재허용 후 미기록 챕터 업로드

Notion 작성은 현재 저장소 AGENTS.md의 금지 지침에 따라 수행하지 않는다. 전체 로드맵은 아직 완료되지 않았다.
