# Pallet 500 g 운반 재계획 및 안전 정지 검증

작성일: 2026-10-09

## 작업 목적과 기준 workspace

중단된 저가속 실험을 점검하고, 안전 이벤트가 실제 실행 정지로 연결되도록 수정한다. 기준은 `/home/minwoo/Development/LLM` 및 `ros2_ur_ws`, ROS 2 Jazzy다. 실기 구동은 수행하지 않았다.

## 확인된 문제와 해결

1. 이전 저가속 실험은 중단되어 `report.json`이 없다. 해당 실험은 완료 결과로 취급하지 않는다.
2. `run_palletizing_gui.sh`는 safety cancel을 켜지 않아 SLIP 이벤트 이후에도 움직였다. 시뮬레이션 launch에 `safety_cancel_enabled:=true`를 명시했다.
3. `palletizing_job.py`는 활성화된 모니터의 결과가 없어도 return code 0이면 성공할 수 있었다. 결과 누락 시 실패와 `monitoring` 분류를 적용하고 회귀 테스트를 추가했다.
4. Cartesian 경로 fraction 0.352941만으로 IK 분기 문제를 확정할 수 없다. 40 mm와 100 mm에서 동일한 실패는 관찰됐으나 충돌/IK/목표 관절 분기 검증이 더 필요하다.

## 실행과 검증

관련 Python 테스트 61개, 셸 문법 검사와 전체 `git diff --check`가 통과했다.

```bash
cd /home/minwoo/Development/LLM/ur10e
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q tests/test_palletizing_job.py tests/test_mtc_command_runner.py tests/test_depth_utils.py
```

```bash
cd /home/minwoo/Development/LLM
ros2_ur_ws/scripts/run_palletizing_gui.sh --headless --task-spec /tmp/pallet-red-block-latest.json --output-dir ros2_ur_ws/regression_results/red-v030-a005-upright-safety-r2-20261009 --velocity-scaling 0.3 --acceleration-scaling 0.05 --gripper-close-position 0.002 --grasp-angle-delta 6.283185307179586 --place-yaw 0.0 --upright-transfer
```

결과 디렉터리: `/home/minwoo/Development/LLM/ros2_ur_ws/regression_results/red-v030-a005-upright-safety-r2-20261009`.

- 계획 성공, 적재 이송 계획 시간 29.125초.
- SLIP 이벤트: 7.8346 mm, 최대 TCP 상대 변위 8.2175 mm.
- 모니터 이벤트 발행 Unix 시각 1791549522.7324955.
- bridge measured hold 로그 시각 1791549522.737737523 (약 5.2 ms 뒤).
- MTC 실행 실패 시각 1791549522.758992150 (약 26.5 ms 뒤).
- return code 2, `execution_dynamics`, 완료 0/1. 배치 성공으로 보고하지 않는다.
- 이 시간 차이는 프로세스 로그 시각 비교이며 모터 물리 정지 시간을 의미하지 않는다.

## 동역학 분석

파지 기준 시각 75.592초에 조 힘은 17.32/18.42 N이었다. 2초 뒤 lift 중 TCP 상대 변위는 0.154 mm, 5초 뒤 0.122 mm였다. 84.585초 이벤트에서 TCP 상대 변위는 X -0.065 mm, Y -4.887 mm, Z -6.123 mm였고 조 힘은 13.87/18.74 N이었다. 따라서 이 실험에서는 파지 직후나 lift 자체보다 적재 이송 중 변위가 급증했다. 실제 접촉 마찰/접촉 면/회전 경로 중 어느 요소가 원인인지는 아직 확정하지 않았다.

## 다음 작업 로드맵

- [x] 중단 실험과 완료 실험 구분
- [x] safety event → bridge measured hold → MTC 실행 실패 확인
- [x] 활성 모니터 결과 누락 시 성공 차단
- [x] lift 중 안정성과 transfer 중 상대 변위 비교
- [ ] 실제 블록·조의 physics material binding과 마찰 계수 확인
- [ ] 이송의 yaw 변화와 블록 상대 회전/변위 비교, 가능한 IK 후보 선택 개선
- [ ] 확인된 원인 한 가지에 대응하는 변경 후 red 단독 재검증
- [ ] red 단독이 모든 기준을 통과하면 red+blue 연속 검증

실패 기준 6 mm 및 현재 시뮬레이션 힘 제한은 유지한다. 성공하지 않은 운반 옵션은 기본값으로 채택하지 않는다.

## Codex 세션 재개

```bash
cd /home/minwoo/Desktop/LLM
codex resume 019f8001-247a-73b2-b9ba-74eed73c3f71
```
