# Pallet Gemini 통합 시뮬레이션 및 적응형 속도 검증

작성일: 2026-10-06. 목적: 자연어 명령, Isaac Sim, RGB-D 인식, RViz/MTC와 물리 감시를 하나의 실행 흐름으로 연결하고 속도·가속도 `1.0 → 0.7 → 0.5` 정책을 검증한다.

기준 workspace: `/home/minwoo/Development/LLM`.

## 현재 주요 파일

- 통합 실행기: `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/run_pallet_llm_gui.sh`
- Gemini 명령 콘솔: `/home/minwoo/Development/LLM/ur10e/pallet_llm_console.py`
- 자연어 TaskSpec 컴파일러: `/home/minwoo/Development/LLM/ur10e/pallet_task_planner.py`
- TaskSpec 검증기: `/home/minwoo/Development/LLM/ur10e/pallet_task_spec.py`
- 순차 작업 및 물리 감시: `/home/minwoo/Development/LLM/ur10e/palletizing_job.py`
- RViz 설정: `/home/minwoo/Development/LLM/ros2_ur_ws/src/mtc_tutorial/launch/mtc.rviz`
- 적응형 속도 시나리오: `/home/minwoo/Development/LLM/ros2_ur_ws/scripts/pallet_scenarios_adaptive_speed.json`

## 최종 구조와 설계 결정

Gemini는 UR10e+Robotiq 경로와 동일하게 자연어를 `pick_object/target_object/relation`으로만 구조화한다. `옆에`, `왼쪽에`, `위에` 같은 관계는 TaskSpec 안에서 유지되고, 실행 직전에 최신 카메라 장면을 읽어 목표 좌표로 변환한다. 고정 팔레트 배치 명령은 기존 `place_xyz` TaskSpec도 지원한다. 속도, 가속도, 셸 명령, ROS 파라미터는 LLM 출력 계약에서 거부한다.

통합 실행기는 Isaac Sim GUI, MoveIt, RViz, RGB-D YOLO+HSV 인식과 Gemini 명령 콘솔을 시작한다. 전용 YOLO 가중치가 블록 영역을 먼저 검출하고, 각 bounding box 내부의 HSV 색상 분할 결과로 `red_block`, `green_block`, `blue_block`을 결정한다. RViz에는 MTC/PlanningScene과 `/yolo/annotated` Image display가 함께 설정된다.

각 물체 실행 중 `watch_pallet_block.py`가 slip, drop, release와 recontact를 감시한다. `execution_dynamics` 실패만 전체 시뮬레이션을 새로 시작해 `1.0 → 0.7 → 0.5`로 낮춘다. 비전·LLM·계획 실패는 속도 강등 원인으로 사용하지 않는다. `0.5` 실패 후에는 추가 자동 반복 없이 중단한다.

## 문제 → 원인 → 해결 → 검증

1. Pallet 실행 경로가 `provider="local"`로 고정돼 Gemini 선택을 무시했다. 선택된 provider를 전달하고 Pallet scene에서도 Gemini 파서를 사용하도록 수정했다. 이후 슬롯 전용 TaskSpec이 `옆에` 관계를 두 개의 고정 좌표 작업으로 오해하는 문제가 확인되어, UR10e+Robotiq의 관계형 LLM JSON 방식을 통합 실행기에도 적용했다.
2. 일반 Python에 `pydantic`이 없어 LLM 모듈을 import할 수 없었다. 외부 LLM JSON은 작은 명시적 검증기로 교체해 선택 의존성을 제거했다. 추가 실행 필드와 비유한 수치도 거부한다.
3. 최종 위치 확인만으로는 MTC 성공 코드 뒤의 물리 slip을 놓쳤다. 순차 작업마다 read-only 물리 모니터를 동시 실행하고 보고서의 `slip_detected`, `release_detected`, `post_release_recontact_detected`를 판정에 포함했다.
4. 상대 output 경로가 ROS workspace의 자식 프로세스에서 이중 경로를 만들 수 있었다. 통합 실행기와 시나리오 실행기가 output 경로를 절대 경로로 정규화하도록 수정했다.

## 물리 검증 결과

단일 500 g·30 mm 기준 회귀:

| 속도·가속도 | 결과 | 근거 |
| ---: | --- | --- |
| 1.0 | 실패 | 운반 중 slip 6.178 mm |
| 0.7 | 실패 | 운반 중 slip 6.738 mm, 최종 XY 오차 10.530 mm |
| 0.5 | 통과 | slip 없음, XY 오차 3.058 mm, 하단 여유 28.458 mm, release 확인, 재접촉 없음 |

Gemini 2물체 통합 E2E:

| 속도·가속도 | 결과 | 근거 |
| ---: | --- | --- |
| 1.0 | 실패 후 전체 재시작 | 첫 red block slip 6.934 mm |
| 0.7 | 실패 후 전체 재시작 | red 통과, blue slip 6.999 mm 및 drop |
| 0.5 | 안전 중단 | 첫 red block release 직전 slip 6.035 mm |

통합 E2E는 자동 강등과 fail-closed 중단을 검증했지만 2물체 작업의 최종 성공 기준은 충족하지 못했다. 기존 검증 프로파일의 가속도는 주로 0.2였으나 이번 정책은 속도와 가속도를 함께 0.5 이상으로 사용했다. 임계값이나 최저 속도를 임의로 완화하지 않았다.

결과 경로:

- `/home/minwoo/Development/LLM/ros2_ur_ws/regression_results/adaptive-speed-100-20261006/report.json`
- `/home/minwoo/Development/LLM/ros2_ur_ws/regression_results/adaptive-speed-070-20261006/report.json`
- `/home/minwoo/Development/LLM/ros2_ur_ws/regression_results/adaptive-speed-050-20261006/report.json`
- `/home/minwoo/Development/LLM/ros2_ur_ws/regression_results/llm-integrated-e2e-20261006/`

## 실행 순서

```bash
cd /home/minwoo/Development/LLM
source ~/.bashrc
gemini_api
bash ros2_ur_ws/scripts/run_pallet_llm_gui.sh
```

명령 콘솔에서 자연어를 입력하고 생성된 TaskSpec을 확인한 뒤 `y`로 승인한다. `exit`로 전체 세션을 종료한다.

## 검증 명령과 결과

```bash
cd /home/minwoo/Development/LLM
PYTHONPATH=/home/minwoo/Development/LLM/ur10e \
  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  ur10e/tests/test_llm_to_json.py \
  ur10e/tests/test_pallet_task_spec.py \
  ur10e/tests/test_pallet_task_planner.py \
  ur10e/tests/test_pallet_llm_console.py \
  ur10e/tests/test_palletizing_job.py \
  ur10e/tests/test_mtc_command_runner.py
```

관련 테스트 53개 통과. Pallet 시뮬레이션 unittest 40개, RobStride 게이트 13개, MTC 프로파일 5개 통과. `mtc_tutorial` 빌드, launch `--show-args`, 셸/Python 문법 검사와 `git diff --check` 통과. 전체 `ur10e/tests` 수집은 선택 의존성 `ultralytics`가 일반 Python에 없어 `test_yolo_webcam.py`에서 중단됐으며 이번 변경 관련 테스트에는 영향이 없다.

## 다음 작업 로드맵

- [x] Gemini 한국어 다중 작업 → 검증된 TaskSpec 실호출
- [x] Isaac Sim + RViz/MTC + 인식 영상 + Gemini 콘솔 통합
- [x] 작업 중 물리 감시와 `1.0 → 0.7 → 0.5` 전체 재시작 정책
- [x] 단일 블록 적응형 속도 물리 회귀
- [ ] 2물체 0.5 동역학 실패의 grasp/contact 구간 진단
- [ ] 사용자가 정한 속도 하한을 유지하면서 acceleration 별도 정책 여부 결정
- [ ] 수정 후 2물체 및 3물체 통합 E2E 통과

Notion에는 저장소 지침에 따라 기록하지 않았다.
