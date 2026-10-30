# Pallet 반복 작업 계획

작성일: 2026-10-09. 기준 workspace: `/home/minwoo/Development/LLM`.

## 운영 방식과 완료 기준

사용자 지시에 따라 계획 → 첫 작업 실행·검증 → 계획 업데이트 → 다음 작업을 반복한다. 사용자가 중단을 요청하기 전까지 안전한 소프트웨어 작업을 이어간다. 실기 구동, 커밋·푸시와 새 외부 권한 확장은 포함하지 않는다. 사용자 입력 또는 승인 없이는 할 수 없는 작업은 필요한 최소 정보와 함께 보고한다. 실패 기준 6 mm / 3회 연속은 완화하지 않는다.

목표 검증은 red 단독 첫 시도 3회 연속 통과 후 2개·3개 순차 작업으로 확대한다. 단일 성공이나 재계획을 숨긴 최종 성공을 안정화 완료로 처리하지 않는다.

## 현재 상태 — 2026-10-11 사용자 요청으로 일시중지

반복46의 조건부 형상 민감도 분석과 관련 검증까지 완료했다. 반복47은 아래 계획만 작성했으며 구현·실행하지 않았다. 사용자의 중단 요청으로 자동 반복 작업을 멈췄고, 명시적인 재개 요청 전까지 개발·검증·시뮬레이션을 진행하지 않는다. 진단 리더와 조건부 오차 정책은 구현됐지만 실행용 형상 승인과 전체 gripper 경로 검증은 미완료다.

새1280/system 입력 두 번째 반복은 job/전체장면 통과지만 red 첫 계획 실패 후 두 번째 시도 성공으로 첫시도 회귀는 실패했다. opening1/plane0.05 mm 후보의독립원래target0/90/0 관측은29.849초/113events/92three-stable, green45° 진단은29.872초/113events/58three-stable이다. 후자는blue 크기검증29회거부이며기준을유지했다. 모든측정은진단용·불확실도미인증이고모션에적용하지않았다. 다음은joint diagnostic reader·명시적오차계약/consumer·보수적bridge/전체gripper검증후원래40 mm3블록E2E이다. 안전기준·기본vision/planner·최종목표좌표변경없음.

## 최초 계획 이력 — 반복 1

1. [x] 성공·실패 적재 구간의 상대 회전 및 시간별 속도·가속도 비교 도구 구현과 검증
2. [x] 팔 J1~J6의 동기화된 위치·속도·effort 기록 추가 및 실제 기록 확인
3. [x] 최초 접촉 팔 자세가 유사함을 확인, 모니터 기준점 지연을 발견·수정
4. [ ] 저속 정착 후보를 기본값과 분리해 단독 실행 및 반복 검증
5. [ ] 결과에 따라 후보 유지·철회하고 다음 계획 갱신

## 첫 작업 결과 → 계획 갱신

`ros2_ur_ws/scripts/analyze_pallet_transfer.py`는 quaternion 부호 불연속을 제거한 세계 좌표 각속도와 선속도, 0.5초 구간 유한차분 가속도를 비교한다. 0.25초보다 큰 데이터 간격은 연결하지 않는다. 적재 구간에는 lift/lower도 포함하므로 정확한 계획 단계나 순간 물리 가속도와 같다고 표현하지 않는다. 관련 테스트 47개 통과.

| 적재 기준 초기 5초 | 성공 실행 | 실패 반복 |
| --- | --- | --- |
| 최대 선속도 | 0.0388 m/s | 0.0408 m/s |
| 최대 선가속도 | 0.0461 m/s² | 0.0483 m/s² |
| 최대 각속도 | 11.39 °/s | 4.26 °/s |
| 최대 각가속도 | 8.69 °/s² | 7.16 °/s² |
| 블록 상대 회전 변화 | 2.29° | 12.36° |

출처: `stage1-pad-geometry-20261009/job/01-red_block-motion.json`, `stage1-pad-geometry-repeat2-20261009/job/01-red_block-motion.json`. 각 1회 관측이므로 통계적 원인 확정은 아니다. 실패가 단순히 더 빠른 회전으로 발생했다는 근거는 없다. 기존 motion log에는 arm joint 기록이 없어 IK 분기 판정을 할 수 없다. 따라서 속도를 다시 낮추는 대신 동기화된 팔 관절 기록을 우선한다.

## 반복 2 — 기록 검증 → 안전 기준점 수정 → 정착 비교

- 팔 기록 실행 `stage1-arm-record1-20261009`는 기존 기준에서 1/1 배치 통과, XY 오차 4.876 mm였다. J1~J6 위치·속도·effort 모두 기록됐다. 첫 접촉 관절 위치는 후속 실패와 모두 반올림 0.001 rad 수준에서 동일했다: (-1.254, 0.880, -1.548, 2.539, -0.518, 1.294) rad. 초기 IK 분기 차이로 확정할 근거는 없다.
- 문제 → 원인: 기준점이 `bilateral_stationary_loaded` 한 샘플에서만 생성되어 첫 실제 bilateral_loaded와 약 11초 차이가 있었다. 이 시점에는 블록이 이미 들리고 이송 중이었다. 초기 grasp/lift 변위를 제외하는 감시 공백이다.
- 해결: 3회 연속 유효한 양측 적재 측정에서 최초 샘플을 기준점으로 고정한다. 중복 시간·누락·비유한 상대좌표·0.5초 초과 간격은 확인으로 세지 않는다. 이후 재설정하지 않는다. 6 mm 및 SLIP 3회 연속 기준은 그대로 유지한다.
- 과거 기록 재계산: 첫 geometry 성공은 6 mm 초과 지속 이벤트 없음, geometry 실패는 이벤트 발생. arm-record1은 새로운 접촉 기준에서 6.756 mm 지속 이벤트가 확인되므로 새 안전 기준 통과의 증거로 재사용하지 않는다. 과거 원본 report는 수정하지 않았다.
- 실제 수정 검증 `stage1-early-contact-reference-20261009`: 기준점 51.103초, SLIP 7.208 mm·최대 7.486 mm로 안전 중단, return code 2. 초기 접촉 감시부터 실행 정지까지 연결 확인.
- 순수 테스트 50개 통과 후 다음 실험을 준비했다. 조 effort는 직접 접촉 정상력 측정과 다르므로 마찰력 계산에 그대로 사용하지 않는다.

### 갱신한 다음 계획

1. [x] `--arm-settle-seconds` 선택형 구현, 기본 0.0으로 기존 동작 유지, 유한한 [0, 2]초 허용
2. [x] 1.0초 정착 비교 실행: 리프트 후 저속 조건 미충족으로 5초 제한 실패
3. [x] 초기 접촉 기준으로 정지 구간 기록 분석: 최대 상대 변위 1.391 mm, SLIP 없음
4. [ ] 결과에 따라 정착 후보 채택/철회 또는 동일 경로 재생·추종 진단 계획 갱신

정착 조건은 5초 제한의 기존 goal settle loop 내에서 확인하며 무제한 대기하지 않는다. 한 stage 실패 시 성공으로 넘어가지 않는다. MoveIt 공개 모델·실기 설정은 변경하지 않는다. 정착 관련 테스트 포함 51개 통과.

```bash
cd /home/minwoo/Development/LLM
ros2_ur_ws/scripts/run_palletizing_gui.sh --headless --task-spec ros2_ur_ws/scripts/pallet_task_stage1_red.json --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/stage1-arm-settle100-20261009 --velocity-scaling 0.3 --acceleration-scaling 0.05 --gripper-close-position 0.002 --grasp-angle-delta 6.283185307179586 --place-yaw 0.0 --upright-transfer --block-contact-offset-m 0.002 --block-static-friction 1.0 --jaw-friction 1.0 --contact-geometry-log --arm-settle-seconds 1.0
```

## 반복 3 — 정착 실패 진단 → solver 단일 변수 비교

정착 실험은 접근·집기 전 팔 목표에서는 각각 1초 저속 정착을 확인했다. 리프트 후 `Isaac target not reached within 5.0 seconds`로 중단됐다. 블록 위치는 (-0.32009, 0.09988, 0.06404) m, 최대 상대 이동 1.391 mm로 안정적이었지만 적재 이송은 시작하지 못했으므로 운반 성공은 아니다.

53초 이후 정지 구간의 J6 위치 폭은 0.0000036 rad, 순 위치 변화 속도 약 0.00000063 rad/s인데 마지막 reported velocity는 -0.02794 rad/s였다. J2의 위치 폭 0.00163 rad/순 변화 약 0.00039 rad/s와 reported -0.01362 rad/s도 차이가 크다. 위치 유한차분과 순간 속도는 같은 값일 필요가 없지만, 정지 상태에서 지속되는 차이를 조사할 필요가 있다. 기준을 완화하거나 오류를 숨기지 않는다.

8초 무명령 GPU 진단에서 articulation solver position/velocity iterations=32/1을 확인했다 (exit code 0). 설치된 SingleArticulation API의 getter/setter를 읽고 `--solver-velocity-iterations` 선택형 override를 연결했다. 기본 -1은 원래 설정 유지, [1, 64]의 정수만 허용한다. 적용 후 getter 검증과 `PALLET_SOLVER_AUDIT` 기록을 추가했다. 테스트 51개, 문법 및 diff check 통과.

### 갱신한 다음 계획

1. [x] 정착 1초 조건 유지 + velocity iteration 1 → 8 단일 변수 비교 (`stage1-arm-settle100-veliter8-20261009`)
2. [x] 실제 32/8 적용 확인, 리프트 뒤 저속 정착 실패 지속
3. [ ] 정착 및 배치가 통과하면 같은 설정의 단독 반복 검증, 실패하면 기준을 유지한 채 실패 근거로 다음 후보 선택

재현은 위 정착 명령에 `--solver-velocity-iterations 8`을 추가하고 output-dir를 해당 결과 경로로 바꾼다. 이 설정은 아직 검증 중인 시뮬레이션 후보이며 실기 파라미터가 아니다.

8회 실험 결과: return code 2, 리프트 뒤 동일한 5초 settle 제한 실패. 최대 블록 상대 변위 0.608 mm, SLIP 없음, transfer 미시작. 마지막 약 2.2초의 J6 위치 폭 0.0000024 rad / 순 변화 약 0.0000010 rad/s에 비해 reported velocity -0.0720 rad/s였다. iteration 증가가 속도 불일치를 해결하지 못했으므로 채택하지 않는다.

### 갱신한 다음 계획 — 반복 4

1. [x] 저속 정착 게이트와 별개의 timed goal-dwell 실험 옵션 구현. 기존 위치 오차 조건을 유지하고 목표를 1초간 유지하되 저속 정착 성공이라고 표현하지 않는다.
2. [x] solver override는 원래 1회로 돌리고, 초기 접촉 safety 기준으로 dwell 중 상대 회전·이송 진입 비교
3. [ ] 별도로 reported velocity와 실제 위치 변화 차이를 기록한다. 안전 실패를 무시하거나 threshold를 완화하지 않는다.

이 실험은 '일정 시간 목표 유지'가 경계 동역학에 미치는 영향을 분리하는 것이며, 저속 정착 실패를 성공으로 포장하는 fallback이 아니다. 정착 옵션 자체는 기본 비활성이며 검사 실패 기록을 보존한다.

`--arm-goal-dwell-seconds 1.0`로 실험을 시작했다. 두 정착 모드를 동시에 활성화하면 오류로 거부한다. 목표 위치 오차가 범위를 벗어나면 dwell 확인은 초기화하고, 반복·역행 피드백 시각을 완료로 세지 않는다. 시간은 기존 5초 goal-loop 범위에 한정한다. 실제 로그도 velocity-settle assertion이 아님을 명시한다. 관련 테스트 52개와 작업 실행기 Python 회귀 61개 통과.

```bash
cd /home/minwoo/Development/LLM
ros2_ur_ws/scripts/run_palletizing_gui.sh --headless --task-spec ros2_ur_ws/scripts/pallet_task_stage1_red.json --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/stage1-goal-dwell100-20261009 --velocity-scaling 0.3 --acceleration-scaling 0.05 --gripper-close-position 0.002 --grasp-angle-delta 6.283185307179586 --place-yaw 0.0 --upright-transfer --block-contact-offset-m 0.002 --block-static-friction 1.0 --jaw-friction 1.0 --contact-geometry-log --arm-goal-dwell-seconds 1.0
```

목표 유지 실험은 리프트 뒤 dwell까지 수행했으나 이송 중 SLIP 6.308 mm, 최대 6.333 mm로 안전 중단됐다. return code 2, 완료 0/1. 대기 시간만으로 안정화되지 않았으며 기본값으로 채택하지 않는다. 검증 중 NaN/Inf 속도·effort가 정착/접촉 성공으로 처리되지 않도록 명시 거부를 추가했다. 실패 메시지도 위치 미도달과 velocity-settle 실패 및 goal-dwell 실패를 구분하도록 수정했다. 테스트 53개 통과.

## 반복 5 — 공식 solver 거동 확인 → 외력 적용 방식 비교

[PhysX 공식 문서](https://nvidia-omniverse.github.io/PhysX/physx/5.8.0/docs/Simulation.html#tgs-steady-state-velocity-and-position-discrepancy)는 TGS의 timestep 초기에 외력을 적용하고 내부 position iteration마다 spring을 적용할 경우 정지 위치와 nonzero reported velocity가 공존할 수 있음을 설명한다. 두 위치의 차분과 reported velocity의 불일치도 solver 단계 차이 때문에 생길 수 있다. 현재 관찰은 이 거동과 일치하지만 프로젝트의 미끄럼 원인까지 확정하지 않는다. articulation Coriolis 계산은 문서가 예외로 명시한다.

설치 `schema.usda`와 NVIDIA `ChairStackingDemo.py`에서 `PhysxSceneAPI.CreateEnableExternalForcesEveryIterationAttr().Set(True)` 지원을 확인했다. 실제 scene의 solver type/원래 flag/적용 flag를 `PALLET_SCENE_SOLVER_AUDIT`로 기록하고, world.reset 전에 선택형 override를 적용한다. `--external-forces-every-iteration`를 추가하지 않으면 기존 값을 그대로 유지한다. CLI 허용값 검증 포함 테스트 54개, 문법·diff check 통과.

### 갱신한 다음 계획

1. [x] 기존 solver 32/1, 저속 정착 1초 조건 유지 + external forces every iteration 활성화 실험
2. [x] 실제 TGS/flag=true/원래 false 확인. 초기 정지 상태 속도는 작아졌지만 경로 계획 실패로 적재 비교 불가
3. [ ] 정착 및 운반 성공이면 동일 설정 3회 연속 검증, 실패면 증거에 맞춰 다음 후보 선택

결과 경로 `stage1-arm-settle100-externalforces-20261009`. 재현은 반복 2 정착 명령에 `--external-forces-every-iteration`를 추가하고 해당 output-dir로 바꾼다. timed goal-dwell이나 velocity iterations override는 활성화하지 않는다. 미끄럼·기울어짐·fresh feedback·최종 배치 실패 기준은 그대로 유지한다.

## 반복 6 — 계획 실패 뒤 카메라 재검증 → 같은 물리 조건 재실험

외력 실험은 약 180초 후 `move above pick` 계획 실패(code 1), 이후 두 시도는 `pick_object is not in camera scene`으로 즉시 실패했다. 모션 미시작, 0/1. 초기 정지 상태 J6 reported velocity 약 -0.00000063 rad/s를 확인했으나 적재 상태의 정착/미끄럼 개선 증거는 아직 없다.

문제 → 원인 → 해결 → 검증: 긴 계획 실패 뒤에는 카메라 장면이 바뀔 수 있는데 실행기가 재시도 전 검출을 재확인하지 않았다. `/home/minwoo/Development/LLM/ur10e/palletizing_job.py`에서 재시도마다 필수 객체의 fresh/stable 검출을 제한 시간 안에 다시 확인한다. 실패하면 추가 planner를 실행하지 않고 마지막 실제 시도와 별도 `retry_blocked` 원인을 보존한다. code 2 실행 실패는 여전히 재시도하지 않는다. 관련 pytest 62개 및 diff check 통과.

1. [x] 재시도 전 bounded camera 검증 구현 및 정상 재시도/검출 누락 회귀 검증
2. [x] 새 시뮬레이션에서 external-forces=true + low-velocity settle 1초 재실험 (`stage1-arm-settle100-externalforces-r2-20261009`)
3. [x] 집기/리프트 도달, 저속 정착 제한 실패. 최대 블록 상대 이동 0.975 mm, SLIP 없음, transfer 0, code 2. 마지막 J4/J6 reported velocity -0.0131/-0.0334 rad/s. 외력 flag만으로 적재 후 속도 불일치가 해결되지 않았다.

## 반복 7 — 감시 수명·결과 출처 안전 보완

검토에서 monitor 기본 300초가 누적 계획 시간보다 먼저 끝날 수 있음을 확인했다. job 완료 시점보다 먼저 종료된 summary나 재사용 output의 이전 결과를 합격에 사용할 위험이 있었다. monitor를 감독 모드로 실행하고 deadline은 안전 실패로 처리한다. runner 대기 중 0.1초 간격 생존 확인, 조기 종료 시 bounded ROS 안전 정지 요청 및 runner process group 중단. 새 MONITOR_TIMEOUT/MONITOR_LOST 코드는 bridge의 measured hold와 후속 goal 거부를 재사용한다.

실행별 UUID와 종료 이유를 검증하고, 정상 supervisor_stop 및 종료 코드 0/1이 아닌 결과는 거부한다. 최종 placement_pass/fresh_feedback도 필수로 검사한다. standalone duration 측정의 정상 종료 의미는 유지한다. 실기 제어 경로는 변경하지 않는다.

1. [x] 감시 만료/조기 종료/이전 JSON 출처 및 runner 감독 구현
2. [x] Python 작업 회귀 66개, 시뮬레이션 관련 단위 57개 및 diff check 통과. 실기/Isaac 없이 실제 ROS bridge에서 MONITOR_LOST와 MONITOR_TIMEOUT DDS 전달 및 STOPPED latch 확인. 정상 SIGINT는 supervisor_stop로 분류되고 safety event를 발행하지 않음.
3. [x] 외력 단일 변수의 실제 운반 비교: 기준 조건(선택형 저속 정착 진단 off) + external forces true. 기존 6 mm/3회 연속·tilt·배치 기준은 그대로 유지

결과 경로 `stage1-externalforces-transfer-20261009`. 저속 정착은 원래 기본 off인 선택형 진단이며 실패 결과는 그대로 보존한다. 이번에는 정착 성공 여부가 아니라 외력 적용 방식이 운반 중 SLIP에 미치는 영향을 기준 명령과 비교한다.

최초 실험 첫 시도 1/1 통과(code 0), XY 오차 4.437 mm, release=true/recontact=false, supervisor_stop 정상·fresh feedback 확인. 최대 loaded 상대 변위 6.418 mm였으나 기존 연속 3회 SLIP는 발생하지 않았다. 단일 성공으로 기본값 채택하지 않으며 같은 조건 반복 검증이 필요하다.

## 반복 8 — 종료 경로 보완 → 동일 조건 반복 검증

감독 코드 리뷰에서 job 예외 시 별도 runner 세션이 남고, leader 종료만 보고 자식 그룹 정리를 끝낼 위험을 확인했다. bounded 전체 process group 정리와 예외 시 안전 정지·정리를 먼저 보완한다.

1. [x] 감독 예외/자식 그룹 잔류/monitor kill fallback 보완 및 회귀. 예외를 숨기지 않고 안전 정지·그룹 정리 후 재발생시킨다. 그룹 leader가 끝나도 전체 그룹을 확인해 INT→TERM→KILL을 제한 시간 안에 적용하며, 정리 확인 불가를 실패 기록한다.
2. [x] 동일 운반 조건 2번째 독립 시뮬레이션: 최초 접근 계획 실패 후 재시도에서 운반/배치 도달. 누적 monitor 300초가 복귀 중 만료되어 MONITOR_TIMEOUT으로 measured hold, code 2, monitoring failure. 완료 0/1이며 반복 성공으로 세지 않는다.
3. [x] 연속 성공 기준 미충족. 다음 계획을 감시 결과 저장 race 및 계획 실패 원인 분리로 변경

### 카메라 재검증의 계약 보완

두 번째 반복을 기다리면서 reader 검토로 top-level list의 AttributeError와 NaN updated_at 수용을 재현했다. 잘못된 root, bool/비유한 시각은 빈 검출로 거부하고, detection timeout/vision age/monitor duration은 유한한 양수만 허용하여 무한 대기를 막는다. runner도 별도로 장면 검증을 수행하므로 이것이 실제 잘못된 로봇 동작을 만들었다고 단정하지 않는다. `wait_for_objects`는 연속 polling 검증이고 새로운 카메라 프레임 3회를 의미하지 않는다. 실제 카메라 생산자의 별도 최소 측정·분산·confidence·TTL 안정화가 선행된다. confidence 기준을 낮추거나 YOLO를 임의 대체하지 않았다.

## 반복 9 — 실제 시간 만료 안전 정지 → 종료 기록 보존·재시도 수명 분리

실제 r2 로그에서 MONITOR_TIMEOUT 전달과 measured hold 및 controller ABORT를 확인했다. 종료 직후 job가 보낸 SIGINT가 monitor의 대형 JSON 직렬화 중 KeyboardInterrupt를 일으켜 motion JSON이 남지 않았다. 안전 중단은 성공했지만 분석 기록 보존 실패이며 별도 수정한다.

감시 300초를 임의로 늘리는 대신, code 1(모션 이전) 재시도에 한해 각 시도마다 새로운 감독 감시기를 시작하는 구조로 수명을 분리한다. 이전 시도 감시 결과는 별도 경로에 보존하고 실제 움직임 이후 code 2 재시도는 여전히 금지한다. 같은 시도의 deadline 만료는 안전 실패로 유지한다.

1. [x] 감독 종료 기록을 반복 SIGINT로부터 보호하고 회귀. 직렬화 안에서 실제 SIGINT를 발생시키는 격리 subprocess 검사(정상 실패/기존 예외 모두 결과 보존) 통과. 시뮬레이션 단위 총 58개 통과.
2. [x] 모션 이전 재시도의 monitor 수명/UUID/결과 파일 분리 및 회귀. 첫 시도 경로는 보존, 이후 `attempt-02-motion.json` 등으로 분리. 매 시도 결과를 attempts에 보존하며 최종 report에 motion_output 경로 추가. missing monitor는 code 1이어도 재시도 금지. 관련 Python 회귀 73개 통과.
3. [ ] 접근 계획 실패의 OMPL 원경로/시간 파라미터화 후 충돌 구분. 충돌 검사와 ACM은 완화하지 않음

### 2026-10-10 이어서 갱신한 계획

1. [x] 동일 외력 운반 조건의 세 번째 독립 실험으로 정상 감독 종료/시도별 결과 및 최종 배치 검증 (`stage1-externalforces-transfer-r3-20261010`). job 첫 시도 1/1/code 0, XY 2.446 mm, 최대 loaded 상대 변위 4.324 mm, SLIP/runtime fault/safety event 없음, release 확인/recontact 없음. JSON 보존·supervisor_stop/fresh placement 확인.
2. [ ] 계획 실패는 별도 분석: 실패/성공 모두 TOTG 후 ValidateSolution의 self-collision 거부가 있고 일부 OMPL PathSimplifier는 TOTG 전에도 invalid 영역 경고. 따라서 TOTG만 원인으로 단정하지 않는다.
3. [ ] 실제 비교 결과와 원경로/재시간화 증거를 확보한 뒤 충돌 검사를 유지하는 단일 변수 개선 선택

주의: r3 job는 통과했으나 wrapper exit 127 (`out: command not found`) 발생. 실행 도중 wrapper 파일 앞부분을 변경한 작업이 Bash의 이어 읽기 위치에 영향을 준 것으로 판단한다. 현재 파일 bash -n 통과/해당 line 정상/관련 프로세스 종료 확인. 완료된 물리 결과는 보존하지만 wrapper 전체 성공으로 세지 않으며 다음부터 실행 중 shell 파일을 수정하지 않는다.

## 반복 10 — 경로 재시간화 허용량 단일 변수 비교

설치 MoveIt 2.12.4와 동일 tag의 [AddTimeOptimalParameterization 구현](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/planning/planning_response_adapter_plugins/src/add_time_optimal_parameterization.cpp), [파라미터 정의](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/planning/planning_response_adapter_plugins/res/default_response_adapter_params.yaml)를 확인했다. 기본 path_tolerance는 0.1, adapter는 pipeline namespace 아래 totg.path_tolerance를 사용한다. 원래 경로에서 관절별 변형을 허용하는 양이다. 이것을 줄여 충돌 거부 빈도가 바뀌는지 비교하되, TOTG가 원인이라고 미리 결론 내리지 않는다.

`--totg-path-tolerance` 선택 옵션을 wrapper → job → command runner → MTC launch에 연결했다. 미지정 시 기존 config 그대로 유지한다. Pallet에서만 유한한 (0, 0.1] rad override 허용, ValidateSolution이 없는 구성은 거부한다. PALLET_PLANNING_AUDIT로 실제 launch dictionary 적용을 기록한다. 단위 회귀 73개, launch 순수 helper 3개, 셸 문법·diff check 통과. C++/ACM/충돌 검사·물리 설정 기본값은 변경하지 않았다.

1. [x] 공식 설치 버전 API 확인 및 선택형 허용량 비교 구현·회귀
2. [x] 외력 후보 물리 조건 유지 + TOTG 0.1→0.01 단일 변수 실제 비교 (`stage1-externalforces-totg001-20261010`)
3. [x] 실제 ROS `/mtc_node_47233` ompl.totg.path_tolerance=0.01 확인. 계획 5.995초, ValidateSolution 거부 0회, 첫 시도 job 1/1/code 0, wrapper exit 0. XY 4.545 mm, 최대 loaded 상대 이동 5.792 mm, SLIP/tilt/runtime fault/safety event 없음, release=true/recontact=false. 같은 조건 반복 필요.

비교 기준 r3는 계획 34.178초/ValidateSolution 거부 2회였다. 각각 한 번 관측이고 OMPL은 확률적이므로 파라미터만으로 성능 차이가 발생했다고 확정하지 않는다. 원경로 전후 직접 collision 분리는 아직 미완료이다.

## 반복 11 — 전체 ROS 로그 보존 → 동일 후보 반복

지난 날짜 `/tmp` launch log가 사라져 원경로 분석 근거가 유지되지 않는 문제를 확인했다. 성공 r3와 TOTG 후보 현재 full log를 각 job/01-red_block-ros.log로 복사·보존했고 후보 로그 SHA-256 일치를 확인했다. 이후 job는 `MTC_LAUNCH_LOG_DIR=<output_dir>/ros`를 자식에 전달하여 full log를 결과 폴더에 직접 생성한다. standalone 호출의 기본 /tmp는 유지한다. report에 명시적 planning_tuning 값을 추가한다. 실행 중 shell 파일은 수정하지 않는다.

1. [x] 로그 경로 전달 회귀 및 셸 문법 검사. 관련 Python 회귀 74개 및 diff check 통과.
2. [x] 동일 후보 두 번째 독립 시뮬레이션 실패: 계획 6.395초, 2개 계획 중 cost 23.5614 선택. 이송 초기 SLIP 7.335 mm(최대 7.498 mm), code 2/0/1. safety measured hold/ABORT 및 정상 감독 summary 보존 확인. 전체 ROS log는 job/ros에 실제 생성됨.
3. [x] 연속 성공 미충족으로 다중 물체 전환 보류, 세 번째 동일 실패 후보 반복 대신 성공/실패 시간 정렬 비교 후 새 후보 선택

## 반복 12 — 이송 선속도 비교 → 속도 단일 변수 실험

두 TOTG 0.01 실험은 초기 파지 관절 위치·TCP 방향이 거의 동일했고 초기 loaded 5초 최대 상대 이동은 성공 0.561 mm/실패 0.595 mm였다. 전체 loaded 측정의 선속도 최대는 성공 0.0290 m/s/실패 0.0430 m/s, 상대 회전 3.118°/8.677°였다. angular speed는 오히려 성공 10.82°/s/실패 4.48°/s여서 각속도 크기만으로 원인을 설명할 수 없다. 서로 다른 경로와 성공/실패 길이이므로 같은 초기 loaded 시간 구간을 별도로 비교한다.

후보는 global arm/Cartesian velocity scaling 0.3→0.2 하나다. acceleration 0.05, 마찰/접촉 geometry, force limit, 외력 flag, TOTG 0.01, SLIP 6 mm/3회 연속 및 tilt/placement 기준은 유지한다. 선속도 저감 효과를 검증하는 실험이지 다른 IK 경로 또는 힘 원인을 이미 확정한 조치가 아니다.

1. [x] 같은 초기 loaded 14.022초 비교: 성공 상대 이동 1.894 mm/선속도 0.0290 m/s/선가속도 0.0460 m/s², 실패 7.335 mm/0.0430/0.0591. 초기 파지 분기 차이라는 근거는 없으며 관측 경로의 선속도 차이를 다음 후보로 선택한다.
2. [x] 새 시뮬레이션 velocity 0.2 단일 변수 비교 (`stage1-externalforces-totg001-v020-20261010`): 첫 시도 1/1/code0, wrapper 정상. 계획70.354초/실행135.911초, XY7.014 mm, 최대 loaded 변위5.936 mm, SLIP/fault/safety event 없음, release=true/recontact=false. 전 구간 최대 TCP 선속도0.0303 m/s/선가속도0.0428 m/s², 각속도3.149°/s. 다른 경로 때문에 nominal scaling에 정확히 비례했다고 표현하지 않는다.
3. [x] 통과했으나 threshold까지 최대 변위 여유0.064 mm에 불과하고 한 번 관측이다. 후보 유지 및 같은 조건 독립 반복으로 계획 갱신

## 반복 13 — velocity 0.2 재현성 검증

1. [x] 같은 후보 두 번째 독립 실험 (`stage1-externalforces-totg001-v020-r2-20261010`): 첫 시도1/1/code0, wrapper 정상, XY4.852 mm, 최대 loaded 변위6.357 mm. 연속3회 SLIP 없음, release=true/recontact=false, fault/safety event 없음. 감독 정상 종료 및 full log 보존. 현재2연속 첫 시도 성공.
2. [x] 세 번째 독립 실험 첫 시도1/1/code0, wrapper exit0 (`stage1-externalforces-totg001-v020-r3-20261010`). XY2.389 mm, 최대 loaded 변위4.535 mm. SLIP/fault/safety event 없음, release=true/recontact=false, fresh feedback/supervisor_stop 및 로그 보존 확인.
3. [x] 빨간 블록 단독 후보 첫 시도 3연속 통과. 실기/다중 물체 안정성을 의미하지 않으며 두 번째 실행의 개별 6 mm 초과 측정 및 첫 실행의 작은 여유도 그대로 공개한다.

세 번째 반복 `stage1-externalforces-totg001-v020-r3-20261010`에서 기존 contact geometry 선택형 로그에 SDK simulation_time/physics_step_index/physics_dt/rendering_dt를 추가한다. 설치 World API에서 속성 존재를 확인했다. bridge는 monotonic 경과시간으로 trajectory를 보간하고 World는 fixed physics dt+render update를 사용하므로 wall↔simulation 시간 진행 비율을 실제 관찰할 필요가 있다. 현재 비율은 미측정이며 시간축 문제가 미끄럼 원인이라고 단정하지 않는다. 이 변경은 기록만 추가하고 timestep/clock/제어·물리/안전 설정은 유지한다.

실제 검증: geometry 183개에서 wall184.936초/simulation128.133초, 평균 진행비율0.69285; 약1초 간격의 비율 min0.68025/median0.68642/max0.82442. physics dt1/240초/render dt1/60초 유지, step counter로 환산한 시간 오차 최대4.35e-8초. 시간축이 다름을 관측했지만 이번 작업은 통과했으므로 미끄럼 원인으로 확정하지 않고 제어 clock은 변경하지 않는다.

## 반복 14 — 두 블록 순차 작업으로 검증 범위 확대

작성일: 2026-10-10. 목적: 같은 시뮬레이터에서 앞선 배치와 복귀 이후 새 camera scene/계획/감시 수명을 거쳐 다음 블록도 완료하는지 확인한다.

1. [x] `ros2_ur_ws/scripts/pallet_task_two_blocks.json`에 red→green 순서를 작성. 좌표계 base_footprint, 단위m, red(0.24,-0.28,0.025)/green(0.28,-0.24,0.025); 기존 3블록 목적지 사용, 각 축40 mm 간격. 기존 spec validator로 검증하고 별도 unit test 추가 없이 적용한다.
2. [x] 동일 후보 2블록 실험 결과1/2, wrapper exit1/job execution_dynamics. red 첫 시도통과, XY1.817 mm/maxloaded4.258 mm, release=true/recontact=false, 정상 감독·안전 이벤트 없음. green 첫 시도code2, TILT_DETECTED 안전 measured hold/controller ABORT, maxloaded1.071 mm/SLIP 없음/이송0. 재시도 없음. 감독 summary/full log 보존.
3. [x] 다중 작업 실패로3블록 확대 보류. green 첫0.552초 샘플부터 이미90° 기울고 중심Z15 mm였다. red 작업 뒤 source가 넘어진 상태를 확인했으나 언제/어떤 링크가 접촉했는지는 아직 미측정. 이를 이송 중 새 tilt 또는 마찰 실패로 단정하지 않는다.

추가 검증: 개별 첫 물체 monitor는 두 번째 실행 전에 종료되므로 전체 작업 뒤 첫 배치 유지 여부는 그 summary만으로 입증할 수 없다. 첫 red 완료 이후 ROS pose만 구독하는 독립 관찰을 시작해 red/green 위치·방향·수신 시각을 `sequence-pose-observation.json`에 보존한다. 제어/안전 발행 없음, 480초 제한 및 수신 종료3초 후 종료; monitor를 대체하지 않는다. 초록 작업 동안 빨간 블록이 유지되는지와 마지막 두 물체 위치를 기존 배치 기준으로 확인한다. contact geometry logger는 red 전용이므로 green 접촉 기하를 측정했다고 표현하지 않는다.

독립 관찰은 green 작업 후반16.53초/각129샘플에 한정된다(전체 green 구간 아님). red XY1.817 mm/Z오차3.58e-8m 유지, 최종green 옆으로 누운Z15 mm 확인. 전체 작업후 물체 유지를 성공으로 오인하지 않는다.

## 반복 15 — 주변 블록 전도 시점과 planning scene 누락 분리

1. [x] 기존 MTC scene setup/command runner 확인. target/table/floor를 diff로 추가하며 기존 world object는 보존한다. Pallet runner는 전체 scene을 읽지만 launch에는 pick object/목표만 전달한다. 기존 scene_collision_publisher는 UR10e 별도 구성이고 현재 Pallet wrapper에 연결되지 않았다. 실제 scene 서비스로 확인한다.
2. [x] 선택형 geometry 로그의 기존 red 필드를 유지하고 `blocks`에 모든 블록 position_m/orientation_wxyz 추가(기록만 변경, Python AST/diff check 통과). 동일 red 진단은 첫 시도1/1/wrapper0, XY7.171 mm/maxloaded5.810 mm로 target 자체는 통과했지만 green/blue 전도를 재현했다. 처음 세 블록 upright, green79.099초 tilt15.87°→80.115초90°, blue83.154초30.19°→84.171초90°. 전체 장면 안전 성공은 아니다.
3. [ ] 실제 scene 누락/접촉 근거에 따라 주변 물체 scene 동기화 또는 파지 접근 경로 수정을 선택해 검증한다. 기울어진 source를 자동 정상 형상으로 가정하는 계약도 별도 점검한다.

진행 증거: 실제 `/get_planning_scene` WORLD_OBJECT_GEOMETRY 조회에 red_block/floor_z_guard/work_table/workcell_table만 존재하고 green_block/blue_block은 없다(`planning-scene-audit.json`). primitive pose는 CollisionObject pose에 상대적일 수 있으므로 이 초기 audit의 좌표를 절대 물체 위치로 해석하지 않는다. ID 누락 근거로만 사용한다. 새 전체 블록 기록 최초 샘플에서 모든 블록Z25 mm/tilt약0° 확인, 78.079초까지 green/blue tilt20° 초과 없음. 전도 원인·재현 여부는 실행 종료까지 확인한다.

실행 명령:

```bash
cd /home/minwoo/Development/LLM
ros2_ur_ws/scripts/run_palletizing_gui.sh --headless \
  --task-spec ros2_ur_ws/scripts/pallet_task_two_blocks.json \
  --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/two-block-externalforces-totg001-v020-20261010 \
  --velocity-scaling 0.2 --acceleration-scaling 0.05 \
  --gripper-close-position 0.002 --grasp-angle-delta 6.283185307179586 \
  --place-yaw 0.0 --upright-transfer --block-contact-offset-m 0.002 \
  --block-static-friction 1.0 --jaw-friction 1.0 --contact-geometry-log \
  --external-forces-every-iteration --totg-path-tolerance 0.01
```

## 반복 16 — 주변 물체 높이를 반영한 수직 여유 단일 변수 비교

측정 근거: 79.099초 red 중심Z64.674 mm, upright 바닥약39.7 mm, green은초기 top50 mm. red와green이XY상 가까워질 때 높이가 겹쳤고 green이 먼저 전도됐다. 83.154초blue도전도. 패드하단도약45~49 mm로 source block상단보다낮다. 실제접촉 pair힘은미측정이므로red몸체/특정jaw만원인으로확정하지않는다.

1. [x] 후보를 기존 선택형 `--transfer-height 0.10`으로 정함. lift/lower0.04→0.10 m 한 변수만 변경, velocity0.2/acceleration0.05/TOTG0.01/마찰/외력/force/safety 유지. 실제 경로 전체의 높이 보장은 로그로 확인한다.
2. [x] `stage1-all-block-height100-v020-20261010` 완료: red 첫 시도1/1/wrapper0, XY7.348 mm/maxloaded6.311 mm, SLIP 없음, 양 주변 블록 전도. 단, 이후 launch 누락을 발견해 **유효한100 mm 비교가 아니며 기존40 mm 실행**으로 재분류한다. 노드 적용 검증 전 명령 문자열만 믿은 기존 해석을 철회한다.
3. [ ] 결과와 별개로 주변 블록 planning scene 동기화 누락을 보완한다. 최초 및 매 작업 시작의 fresh camera 장면, unknown orientation의 보수적 충돌 형상, 기존 placed object ACM을 다음 설계 검토에 포함한다. 충돌 검사/ACM 완화나 실제 장비 실행은 하지 않는다.

후속 설계 근거: 현재 camera scene은 size/position/confidence를 검증하지만 orientation을 제공하지 않는다. SceneManager는 optional orientation을 지원하므로 임의 identity를 실제 측정 방향으로 표현하지 않는다. 후보는 unknown orientation 물체의 외접 구(반지름0.5×size 대각선 길이) 같은 보수적 형상이며 endpoint 간격/과도한 경로 제한을 검증해야 한다. `applyTouchCollisionAcm` 및 pick stage의 target 허용이 placed object에 남을 수 있으므로 주변 물체를 추가할 때 해당 물체의 이전 허용을 명시 해제하고 robot/self/target 이외의 기존 ACM을 보존해야 한다. scene 서비스 실패·오래된/불완전한 최초 장면은 모션 전 실패로 처리하며 누락을 조용히 무시하지 않는다. 아직 구현/채택한 설정은 아니다.

재현은 반복14 명령에서 task-spec을 `pallet_task_stage1_red.json`, output-dir를 `stage1-all-block-height100-v020-20261010`으로 바꾸고 `--transfer-height 0.10`을 추가한다. 과거 높은 CartesianPath 실험 실패와 다른 upright OMPL 경로 비교이며 성공을 미리 가정하지 않는다.

## 반복 17 — 높이 인수 전달 오류 수정·실제 파라미터 검증

문제→원인: runner는 lift/lower4개 launch 인수를 출력했지만 `pick_place_demo.launch.py`의 FLOAT_DEFAULTS/DeclareLaunchArgument 목록에 없어 노드에 전달되지 않았다. 기존profile40 mm가 유지되어 maxheight69 mm인 실행을100 mm 비교로 오인했다. 앞선 높은 Cartesian 실험도 적용값이 확인되지 않았다면 실제100 mm 결과로 해석할 수 없다. 원본 기록은 보존하되 이 정정을 우선한다.

1. [x] lift_min/max_distance 및 lower_min/max_distance launch 전달 추가. 미지정은 프로필 기존값 유지, pure 실제 numeric loop 회귀에서 모든프로필 default 보존 및0.10 override 전달 확인.
2. [x] `stage1-all-block-height100-fixed-v020-20261010` 실제 ROS4개값0.1 확인 및 첫 시도1/1/wrapper0. XY2.083 mm/maxloaded2.462 mm/transfer최소바닥93.840 mm, 최대높이124.681 mm. release=true/recontact=false, SLIP/fault/safety event 없음, fresh/supervisor_stop. 全geometry샘플에서 green/blue 최대tilt약0.000061°/위치변화1.5e-8m이하. 주변물체 전도 없음. 이 단일변수 비교의scene-obstacles는off.
3. [ ] 다음은 구현한 선택형 `--scene-obstacles`를 켜고 실제 ID/형상/ACM검증 및 다중작업에 적용한다.

주변물체 구현 상태: wrapper→job→runner→launch→C++ 경로 추가. 기본off/standalone launch빈목록 유지. 활성화하면 job 및 runner 모두 최초/재시도에 세블록 fresh scene 필요; camera검증된 size의 반대각선(30×30×50 mm는32.787 mm)으로 non-picked외접구 생성. 실제 orientation 측정으로 포장하지 않는다. C++shape/frame/ID/finite/radius 검증 후scene diff적용, still-attached 주변물체는 거부. 주변물체의 이전 ACM/default허용 해제, grasp의target 전체허용 뒤에도 target↔obstacle은 금지. 기존 robot self/target 등 다른 허용을 확대하지 않았다. launch helper총7개, Python관련75개 및 빌드 통과(기존 obsolete header note만). 실제 scene/ACM/물리검증은 아직 미완료다.

## 반복 18 — 주변 물체 장면·ACM 실제 적용 검증

1. [x] 실제높이100 mm 후보의 첫 물리 결과 판정. 이전40 mm 전도 재현과 구분하고 기본값은 유지.
2. [x] 같은후보에 `--scene-obstacles`만 추가한 최초 및반복2실행 판정. 실제scene ID/구반지름·좌표 확인, 두번째실행에서selected solution 6개내부장면의실효ACM금지 확인. 물리안전/배치/주변두블록상태도통과. 명시행audit의한계와실효검증은아래별도기록.
3. [ ] 통과하면 같은새후보 red단독첫시도3연속→두블록→세블록으로 재현성을 검증한다. 계획실패라면 외접구의과도한제약/장면불완전/경로거부를근거로진단하며 충돌금지나안전기준을조용히완화하지않는다.

실행은 반복17 높이수정후 명령에 `--scene-obstacles`를 추가하고 위output-dir를 사용한다. 실제 scene snapshot의 CollisionObject pose와 primitive relative pose를 함께 기록하여 초기audit의좌표해석공백을보완한다. 안전감시·code2재시도금지·force limit 유지, 실기 구동 없음.

첫 실행 물리 결과: 첫 시도1/1/wrapper0, XY2.051 mm/maxloaded2.076 mm/transfer최소바닥93.672 mm, release=true/recontact=false, SLIP/fault/safety 없음, 정상감시. 주변green/blue 최대tilt0.000061°/위치변화1.5e-8m 이하. scene에 실제non-picked구2개 및32.787 mm반지름 확인. 단, 초기/적재 service audit에서 explicit ACM행이 없는 것으로 관측되어 처음작성한 '모든명시행존재' 검증은false였다. 이원본검사결과는보존하고 전체ACM검증완료로표현하지않는다.

설치버전과같은 [MoveIt2.12.4 collision_matrix.cpp](https://github.com/moveit/moveit2/blob/2.12.4/moveit_core/collision_detection/src/collision_matrix.cpp) 확인: msg constructor는기본NEVER와동일한pair를저장하지않고 기본값·pair의실효허용을getAllowedCollision로결정한다. 따라서행부재만으로허용/금지적용실패를판정하면안된다. 또한행직렬화와실제선택경로내부장면은같다고미리단정하지않는다.

추가구현: 실제선택된MTC solution의lift/loaded-transfer/lower 시작·끝내부장면에서각obstacle의world존재와robot모든link/target에대한effective ACM을직접검사한다. NEVER 외의ALWAYS/CONDITIONAL이있거나검증장면이6개미만이면 실행전거부. scene-obstacles off일때기존동작유지. 검사기준/물리threshold완화없음. 빌드후 동일후보반복 `stage1-scene-obstacles-height100-v020-r2-20261010`로 실제audit로그와반복성검증한다.

두 번째 실행 완료: 첫 시도1/1/wrapper0, XY1.489 mm/maxloaded2.273 mm/transfer최소바닥93.773 mm. release=true/recontact=false, SLIP/fault/safety 없음, fresh/supervisor_stop, 주변green/blue 전도·위치변화 없음(최대tilt0.000061°/변화1.5e-8m 이하). 실제선택경로의lift/transfer/lower 시작·끝6개 `PALLET_OBSTACLE_SOLUTION_AUDIT effective_pairs_forbidden=true` 확인. 첫실행의엄격한명시행audit는보존하고 내부실효검증과구분한다.

실제설치MoveIt라이브러리로 C++검사helper를 검증했다: 기본pair부재금지, 명시허용거부, default허용거부/explicit금지우선, conditional허용거부, obstacledefaultNEVER우선, serialization에서all-false행생략후금지유지 6개 모두통과. CMake/colcon 검사에 C++suite 및 기존launch2개suite를등록하고 3suite(6+3+4 cases) 통과. BUILD/CMake수정은물리제어변경없음.

## 반복 19 — 새 후보 첫 시도 3연속 및 다중 작업 재진입

1. [x] scene-obstacles+실제height100 mm+velocity0.2 후보의2연속첫시도통과 및실효ACM/실패경로검증
2. [x] 같은후보세번째독립시험 `stage1-scene-obstacles-height100-v020-r3-20261010` 첫 시도1/1/wrapper0. XY2.085 mm/maxloaded2.341 mm/transfer최소바닥93.830 mm. release=true/recontact=false, SLIP/fault/safety 없음, fresh/supervisor_stop. 선택경로6개내부ACM검사 통과. 전체205geometry 샘플에서 green/blue 전도·위치변화 없음. red 방출 후55샘플 최종배치 유지.
3. [x] 3연속통과 후 red→green두블록 spec에 같은옵션으로재진입. `two-block-scene-obstacles-height100-v020-20261010` 실행 시작. 이전40 mm후보의3연속을새후보검증으로대체하지않는다.

## 반복 20 — 두 블록 연속 작업과 전체 장면 유지 검증

1. [x] `analyze_pallet_scene.py` 추가. 모든 블록의 유효한base_footprint/m pose와1 Hz geometry 기록으로 미작업물체의 초기위치/자세 유지 및 완료물체의 방출시점부터 작업종료까지 배치 유지 검사. 기존20 mm XY/5 mm Z/15° 기준 유지. logging gap 또는 종료 coverage 부족은 검증 불가로 거부(관찰품질2.5초 경계, 물리 안전 기준 아님). 실패시CLI exit1. 연속접촉 증거로 해석하지 않으며 방출시각은 공통wall-clock epoch으로 정렬.
2. [x] 두블록실행종료: wrapper1/job1/2/failure_class=vision. red첫시도성공(XY2.059 mm/maxloaded2.610 mm/transfer최소바닥93.622 mm/release확인/정상감시/SLIP·fault없음). 이후전체fresh검출에서red누락으로green실행전에중단. green모션실패나두블록성공으로표현하지않는다. 최종geometry에서red upright·green/blue 초기위치유지.
3. [x] 세블록진입보류. 원본로그보존하고카메라시야를조사했다. safety/fresh장면기준을완화하지않고새단계를아래에갱신한다.

검증: 분석기10개 단위테스트 통과(후속작업의완료블록변위, 미작업전도, stale/gap/missing pose/release/clock/unknown·duplicate ID 실패경로 포함). 실제r3 전체장면true, 과거40 mm `stage1-all-block-observation-v020-20261010`은 job_passed=true지만 scene_passed=false로 주변green/blue 전도를구분. 원본report는수정하지 않았다.

## 반복 21 — 배치 영역 카메라 시야 보완

문제→근거: 기존카메라eye(-0.55,0.65,0.85)/target(-0.32,0.18,0.025)m는source행을향한다. 두블록실행종료의실제annotated640×480영상에green/blue만표시되고red는보이지않는다. 실제calibration행렬로배치red를optical좌표로변환하면normalizedXY=(-0.225,-0.418). 시야문제후보이므로단순검출threshold/TTL/전체scene요구를완화하지않는다. shutdown후vision로그의context invalid 오류는추가관측이지만원인단정하지않는다.

1. [x] 선택형 `--camera-view workcell` 구현. 기본source기존pose보존, wrapper→launch→Isaac전달. 신규eye(-0.15,0.45,1.10)/target(-0.02,-0.09,0.025)m, focal24 mm유지. 실제USD변환으로calibration을계산하며실제CameraInfo도확인한다. 시뮬레이션카메라만변경하고검출결과를ground truth로대체하지않는다.
2. [x] `two-block-workcell-camera-height100-v020-20261010` 종료 wrapper1/job1/2/planning_or_startup. red 첫 시도 성공(XY0.580 mm/maxloaded2.401 mm/transfer최소바닥93.664 mm/release확인/정상감시/SLIP·fault없음). red 방출후83샘플배치유지, 미작업blue195샘플초기자세유지. 배치red(0.2396,-0.2809,0.025)m를포함한세블록fresh재검출과최종실제영상확인. green은3회의모션전계획실패이고물리실행하지않았다.
3. [x] 세블록진입보류, 계획충돌원인진단으로갱신. 실기/기준완화/캐시좌표복구없음. 시야수정이측정pick좌표에도영향을줄수있으므로이전3연속을새카메라결과로포장하지않는다.

검증: camera helper·launch 인수 연결 3개/Isaac CLI 5개 통과, 2개 package 빌드 통과, bash 문법/diff check 통과. 후보 방향 투영 테스트는 정적인 기하 조건만 검증하며 실제 시야/가림/검출 성공 증거가 아니다.

실제 적용 확인: `PALLET_RGBD_VIEW view=workcell` 로그와 실제calibration/CameraInfo를 해당실행폴더의 `camera-calibration.json`, `camera-info.yaml`에 보존했다. 해상도640×480/fx=fy=732.9993px, 수평/수직반시야각23.584°/18.130°. source중심 투영red(485.78,391.90)/green(492.70,342.22)/blue(499.19,295.58)px, 목표red(204.57,119.10)/green(176.36,132.53)/blue(183.46,114.89)px. 모두영상범위안이지만팔가림이나검출성공을증명하는수치는아니다. 첫red 실행전실효ACM6장면통과, 초기fresh전체장면확보후모션중. 최종배치뒤재검출은아직확인전이다.

## 반복 22 — green 최종 손 벌림 실패의 실제 충돌쌍 진단

새카메라로vision중단은해소됐으나green계획3회모두 `open hand: Goal state is in collision!` 로실패했다. place IK/lower 후보는각37/40/35개존재했으나최종open hand후단에연결되지않았다. 일부planningpath검증로그의table/J6/J8/J9·blue/jaw·selfcollision은다른후보에서발생했으므로최종손벌림실패원인과혼동하지않는다. 외접구가placed red와과도하게간섭하는지는아직추정이다.

1. [x] 로컬MTC `move_to.cpp`/`joint_interpolation.cpp` 확인: goal joint state로계획하고goalcollision이면실패trajectory 끝에goal state를보존한다. 해당실패stage의end scene을같은hand group/ACM으로재검사해충돌pair/count/depth를기록하는선택형진단callback 추가(주변물체옵션활성시만, 제어/형상/ACM변경없음).
2. [x] `two-block-open-hand-contact-audit-20261010` 종료 wrapper1/job1/2/planning_or_startup. red 첫 시도 성공(XY0.609 mm/maxloaded3.092 mm/release확인/recontact없음/SLIP·fault없음). green 3회 code1 계획 실패. 최종 open hand 실패의 동일 group/ACM 직접 검사에서 J8/red 65건, J9/red 63건만 기록됐다. 실제 물리 접촉이 아니라 모션 전 충돌 형상 간섭 증거이며 depth0 출력으로 간섭을 없는 것으로 해석하지 않는다.
3. [x] 충돌 형상/ACM을 유지하고 최종 방출 벌림만 줄이는 단일 후보를 선정했다. 목표 좌표 간격·안전/배치/fresh 기준은 유지하며 다음 반복에 기록한다.

## 반복 23 — 방출 전용 20 mm 벌림 단일 변경

근거: URDF J8/J9 범위0~33.5 mm, SRDF open33.5 mm/턱(전체67 mm). 블록 폭30 mm이고 모션 전 최종 벌림에서 주변red형상에간섭했다. 후보 한쪽20 mm(전체40 mm)는 축정렬30 mm블록 대비 명목상5 mm/턱 여유이다. 실제 방출 여유나 물리 안전을 이 계산만으로 확정하지 않고 감시 로그로 확인한다.

1. [x] 선택형 `--release-open-position` 구현: wrapper→job→runner→launch→C++ 최종open hand만 적용. 기본 -1은 기존 named open 유지. 파지 전open/pregrasp/recovery는 변경하지 않음. C++는 Pallet J8/base_footprint/finite/실제블록XY폭의반보다큰벌림/URDF범위를 검증하며 모션 전 거부한다. job/runner도 비유한·범위초과·다른프로필 거부. report planning_tuning에 실제요청값 보존. force·loaded path·형상·ACM 변경 없음.
2. [x] `two-block-release020-workcell-height100-v020-20261010` 종료 wrapper1/job1/2/planning_or_startup. 실제C++20 mm적용로그 확인, red첫시도성공(XY0.591 mm/maxloaded2.175 mm/release확인/recontact없음/정상감시/SLIP·fault없음). green3회모션전계획실패에서직접J8/red53건·J9/red50건 기록. 20 mm 후보가간섭을해소했다는주장은하지않는다.
3. [x] 세블록보류, 다음은실패자세의비실행벌림비교. 기본값채택·안전기준완화·단순물체간격확대 없음.

검증: 관련Python61개, CTest4suite(launch와ACM검사 및새release범위검사4개) 통과, build 성공(기존obsolete header note), bash문법/diff check 통과. 새방출범위 검사는 기본보존/20 mm허용/비유한·범위·무여유·넓은블록·다른프로필 거부 포함. 최종실제제어·물리검증은 진행중이다.

첫 시도 판정 보완: 전체 장면 분석기에 `first_attempt_passed`와 `regression_passed` 추가. 모든 실제 run의 유일한 attempt1/returncode0 증거가 있어야 첫 시도 통과이며 재계획 성공이나 시도 기록 누락은 회귀 통과로 세지 않는다. 장면 유지 자체의 `scene_passed`와 분리하고 CLI exit0은 두 기준을 모두 충족할 때만 반환한다. 관련 단위테스트12개 통과 및 기존실제r3의회귀통과 재확인. 제어·물리 기준 변경 없음.

## 반복 24 — 실패 자세에서 벌림 값별 충돌 비교(비실행)

1. [x] 실패 stage의goal scene/RobotState를복사해 J8/J9값16/18/20/25/33.5 mm별로 동일 hand group/ACM/주변형상에서 재검사하는 `PALLET_RELEASE_SWEEP` 진단 추가. 원래solution/scene/목표/실행명령은수정하지않는다. 충돌검사통과만으로물리방출을보장하지않는다.
2. [x] `two-block-release-sweep-audit-20261010` 종료 wrapper1/job1/2/planning_or_startup. red 첫 시도 성공(XY0.592 mm/maxloaded2.053 mm/transfer 최소 바닥93.626 mm/release 확인/recontact 없음/SLIP·fault 없음). green은 세 번의 모션 전 계획 실패였다. 실패 goal134개에서 16/18/20/25/33.5 mm 각 값 모두 collision=true. 모든 미검사 값까지 불가능하다고 확대 해석하지 않는다.
3. [x] 벌림을더줄여실행하는대신배치yaw45°후보를선정했다. unknown orientation 주변형상의축소나충돌허용완화는하지않는다.

## 반복 25 — 배치 yaw 45° 단일 변경

작성일: 2026-10-11. 비교 기준은 반복23~24의 workcell view/실제높이100 mm/velocity0.2/TOTG0.01/방출20 mm 후보이다. `place_yaw`만0→π/4 rad로 변경해 손 벌림축과이웃물체방향의관계를바꾼다. 검사한벌림값모두실패라는근거에따른다. 실제생성pose의대칭후보·IK 선택과물리동작은로그로판정하며45°가간섭을없앨것이라고미리단정하지않는다. 두대상 모두같은yaw값을요청하는job수준단일인수변경이다.

1. [x] 기존지원CLI로 `two-block-place-yaw45-release020-20261011` 실행시작. 코드/형상/ACM/좌표간격/감시기준을추가변경하지않음.
2. [x] 두블록 첫 실행 종료 wrapper0/job2/2/각1회code0, 전체scene/first_attempt/regression_passed=true. XY red0.414 mm/green0.984 mm, maxloaded2.801/2.206 mm. 두모션release확인/recontact없음/SLIP·fault없음/fresh/supervisor_stop. 각선택경로6개실효ACM검사통과. 총320geometry샘플, red방출후196개/green43개배치유지, 미작업blue320개초기자세유지. 최대gap1.0285초/종료coverage0.984초. 두블록조건1회성공이지세블록/안정화완료증거는아니다.
3. [x] 세블록40 mm인접목표형상검토를다음단계로분리했다. 원래다중작업을더넓은좌표배치로대체하지않는다.

확장 전 정적 기하 검토(진행 중 시험과 독립): `pallet_task_three_blocks.json`의 blue↔green 및blue↔red 중심 간격은40 mm이다. 현재30×30×50 mm 외접구 반지름32.787 mm이며 같은Z의 yaw0 목표박스까지 구중심 최단거리는25 mm, yaw45에서는18.787 mm라 양쪽모델이겹친다. red↔green 대각선56.569 mm는각35.355/41.569 mm로박스와구가겹치지않는다(손형상충돌은별도). 따라서대각선두블록yaw시험이통과해도인접세블록성공을주장할수없다. 현재camera는orientation을제공하지않으므로물체를임의upright/identity로가정해형상을줄이지않는다. 이계산은모델기하 근거이며 실제물체접촉측정은아니다.

## 반복 26 — 두 블록 후보 반복성·인접 배치의 측정 계약

1. [x] `two-block-place-yaw45-release020-r2-20261011` 종료 wrapper0/job2/2/각 첫 시도. 전체 장면·첫 시도·회귀 판정 모두 true. XY red1.022/green0.952 mm, maxloaded2.918/2.061 mm. release 확인/recontact 없음/SLIP·fault 없음/fresh/supervisor_stop. 전체 geometry328개, red방출후189개/green42개 유지, blue328개 초기자세 유지. 최대gap1.027초/종료coverage0.402초. 동일 후보 독립 2회 성공이며 3회 안정화 판정은 아직 아니다.
2. [x] 읽기 전용 별도 검토와 관련 소스 확인 완료. 카메라 quaternion w=1은 고정값이고 loader는 방향을 버린다. 정사각 top-face PCA는 yaw를 식별하지 못하며 경계 기반 π/2 대칭 처리가 필요하다. 방향별 관측 시각·보정 revision·품질·불확실도와 producer/store/loader/bridge 연동을 계획에 포함했다. yaw45의 투영폭42.426 mm는 40 mm 인접셀에서 실제 겹침 가능성도 있으므로 방향 측정만으로 세 블록 성공을 보장하지 않는다.
3. [ ] 측정/형상조건과인접셀의실제충돌여유를검증한뒤원래세블록목표로확장한다. 목표간격확대·unknown orientation의identity가정·ACM완화로완료기준대체하지않는다. 새로운orientation/형상후보는아직구현·채택하지않았다.

## 반복 27 — 측정 입력·평면 후보 기반 구현, 두 블록 세 번째 반복

문제 → 원인: 현재 단일 median depth/대표 pixel과 고정 quaternion으로는 물체 방향을 검증할 수 없다. top/side 분리와 경계 관측, 실제 노출 freshness가 필요하며 파일 작성 시각으로 방향 age를 대체할 수 없다.

1. [x] `/home/minwoo/Development/LLM/ur10e/vision/box_pose_samples.py` 구현. 실제 boolean segmentation mask 각 pixel의 depth를 역투영하고 검증된 optical→base_footprint 강체변환을 적용한다. m/mm 인코딩 변환, 등록·rectified·optical-Z 계약, 같은 clock의 RGB/depth exposure age와 sync, calibration revision, mask provenance를 명시적으로 요구한다. bbox fallback/미등록/오래됨/비유한 입력은 거부한다. 원래 경계 ray와 침식한 내부 평면 점을 분리하고 유효 depth 비율과 이미지 경계 절단을 기록한다. object truth/크기/테이블 높이/가정 normal을 입력하지 않는다.
2. [x] `/home/minwoo/Development/LLM/ur10e/vision/box_plane.py` 구현. 제한된 deterministic RANSAC와 SVD refit으로 측정된 top-plane 후보 normal/centroid/residual/inlier/분포를 반환한다. 모든 품질 한계는 호출자가 명시해야 하며 실기 noise 값으로 채택하지 않는다. normal의 부호만 +Z 기준으로 정리하고 방향을 +Z로 바꾸지 않는다. yaw/완전한 box pose/안전한 축소 형상은 아직 출력하지 않는다. 카메라 callback과 planner에는 연결하지 않아 진행 중 후보의 동작은 그대로이다.
3. [x] `two-block-place-yaw45-release020-r3-20261011` 종료 wrapper0/job2/2/각 첫 시도. scene/first_attempt/regression 모두 true. XY red1.143/green1.605 mm, maxloaded2.742/1.925 mm. release 확인/recontact 없음/SLIP·fault 없음/fresh/supervisor_stop. 각 경로6개 내부 실효ACM 검사 통과. geometry316개, red방출후192개/green42개 유지, blue316개 초기 자세 유지. 최대gap1.027초/종료coverage0.531초. 동일 두 블록 후보 독립3회 첫 시도 성공 기준을 충족했다. 세 블록 성공이나 기본값 채택을 뜻하지 않는다.
4. [ ] 실제 RGB/depth/CameraInfo/exposure stamp의 진단 캡처와 optical-Z/registration/외부 보정 조건을 확인한다. 관측 top/side 분리·정사각 경계 yaw fitting과 불확실도 검증을 구현하고 synthetic+실제 캡처를 검증한다. simulator pose는 평가용 비교로만 사용한다.
5. [ ] 공동 position/orientation 안정화·방향별 freshness/무효화·loader 보존·보수적 OBB bridge 및 인접셀 충돌 검증을 구현한다. 관측 실패 시 기존 외접구 유지/모션 전 거부. 이후 원래 세 블록 목표로 확장한다.

검증: `PYTHONPATH=ur10e PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q ur10e/tests/test_box_pose_samples.py ur10e/tests/test_box_plane.py ur10e/tests/test_depth_utils.py` → 52 passed. 초기 실행은 기존 테스트의 `vision` import 경로 누락으로 수집 실패했으며 프로젝트의 PYTHONPATH를 지정하여 정상 실행했다. 새 입력 검사24개와 평면 검사10개, 기존 depth18개이다. synthetic depth의 단위/비identity 변환/개별 depth 사용/시간·provenance 오류 거부/측정 tilt·outlier·지원 부족을 검증했다. `git diff --check` 통과. 실제 카메라 방향 정확도나 하드웨어 구동을 검증한 결과는 아니다.

## 반복 28 — 실제 RGB-D 캡처와 관측 품질 원인 확인

1. [x] 읽기 전용 `/home/minwoo/Development/LLM/ur10e/scripts/capture_pallet_rgbd.py` 구현 및 실제 캡처. 새 출력 폴더만 허용하고 timeout≤60초, sync≤0.1초, 실제 RGB/depth/CameraInfo와 노출 header/frame을 보존한다. calibration 파일에서 camera transform만 추출하고 object truth는 저장하지 않는다. revision은 camera transform만 hash하며 카메라 topic만 구독하고 제어 명령은 발행하지 않는다. 최신 CameraInfo 한 개만 유지한다.
2. [x] 세 번째 실행 중 `rgbd-capture-01/rgbd.npz`, `metadata.json`, 원본 RGB 인코딩 `rgb.png` 저장. 640×480/32FC1/RGB-depth-Info stamp151.791674583초/sync0/동일 optical frame/zero distortion. ROS header는 simulation time이며 Unix 시간과 혼합하여 freshness를 주장하지 않는다. 메타데이터의 등록·depth semantics는 원본 상태(unverified)를 유지한다. 실행 그래프는 RGB/depth에 동일 RenderProduct를 연결하며 설치된 `OgnROS2CameraHelper.py`의 depth→DistanceToImagePlane을 확인했다. 이는 해당 시뮬레이션 입력 계약의 근거이지 실기 registration 인증은 아니다.
3. [x] 실제 mask 점군 진단: 기존 saturation70의 red 내부250점에 Z>49 mm 점이 없어 top-plane 후보 거부. green top60점의 XY 지지 폭 약25.8×4.8 mm, 실제blue top21점 약5.7×24.8 mm라 8 mm 두 축 지지 조건에서 거부. RGB 원본에서 green/blue는 팔·그리퍼에 가려져 있었다. 파란 배경이 낮은 평면으로 검출되므로 normal/residual만으로 블록 승인 금지. 검사 숫자는 진단용이며 실행 안전 기준과 별개이다.
4. [x] 동일 캡처에서 detector의 saturation만70→50(또는35)으로 비교한 오프라인 진단. red mask346→582픽셀, 내부468점, plane inlier 비율0.42094, minor span33.753 mm, 측정 normal(0.0000875,0.0016959,0.9999986). 기존 HSV가 탈채도된 red top 일부를 제외한다는 근거다. plane centroid는 top 지지점 평균이며 box 중심/완전한 방향으로 승인하지 않는다. runtime detector/후보/충돌 형상은 변경하지 않았다.
5. [x] 측정 top-plane 좌표의 정사각 경계 후보와 모든 edge 지지·크기·경계 선분 residual 검사 구현 및 실제 캡처 진단. 아래 반복29 기록 참조. 방향·불확실도 승인이나 planner 연결은 아직 아니다.
6. [ ] 이후 팔이 가리지 않은 source/target 캡처와 독립 평가용 truth 비교, calibration/깊이·방향 불확실도 산정, 공동 안정화와 conservative collision bridge. 원래40 mm 인접 목표의 실제 상대 yaw/OBB 간섭을 확인한 뒤 세 블록 재진입한다.

검증: capture7개 + 입력24개 + 평면10개 + 기존depth18개 =59 passed, diff check 통과. camera-only revision/object truth 제외, 잘못된 timeout/sync의 ROS 초기화 전 거부, 기존 output overwrite 거부 포함. 실제 RGB-D 캡처 성공 exit0. estimator callback/실기/세 블록 완수는 아직 미검증이다.

실제 캡처 명령(진행 중 시뮬레이터가 필요하며 기존 폴더를 덮어쓰지 않음):
```bash
cd /home/minwoo/Development/LLM
python3 ur10e/scripts/capture_pallet_rgbd.py --output-dir ros2_ur_ws/regression_results/two-block-place-yaw45-release020-r3-20261011/rgbd-capture-01 --calibration /tmp/pallet_camera_calibration.json --timeout 20
PYTHONPATH=ur10e PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q ur10e/tests/test_capture_pallet_rgbd.py ur10e/tests/test_box_pose_samples.py ur10e/tests/test_box_plane.py ur10e/tests/test_depth_utils.py
```

## 반복 29 — 측정 정사각 경계 후보·비가림 초기 장면 검증

문제 → 원인 → 해결: 물체 방향이 없는 상태에서 외접구를 줄일 수 없다. `ur10e/vision/box_boundary.py`는 관측된 top 점을 실제 측정 plane basis에 투영하고 최소 면적 rectangle와 π/2 대칭 방향·top 중심·측정 크기를 계산한다. 영상의 minAreaRect 각도나 PCA yaw를 쓰지 않는다. 모든 edge의 실제 지지점 수/지지 span, 크기·평면 residual·이미지 절단을 검증하며 부분 가림을 nominal 크기로 완성하지 않는다. 아직 diagnostic candidate이며 완전한 관측성·보정 불확실도 인증은 아니다.

1. [x] 모서리 가림 회귀에서 hull 꼭짓점만 검사하면 대각선 잘림을 놓치는 결함 재현. hull 선분의 네 affine edge-distance 교점/끝점을 모두 검사해 선분 전체 최대 이탈을 계산하도록 수정. 실패는 accepted=false 진단값만 반환하며 승인된 rotation으로 노출하지 않는다. 경계22개 테스트(yaw0/45/89/90/135, tilt0/0.15, missing4edges, 크기/plane 오류, 잘린corner/잘못된gate) 통과.
2. [x] 기존 red 배치 캡처는 계속 경계 residual gate에서 거부한다. 약45.014°/29.752×29.914 mm 크기 후보가 나와도 diagnostic hull 한계1.5 mm를 넘으면 승인하지 않는다. 한계는 실행 safety 기준이나 검증된 noise bound가 아니다.
3. [x] `rgbd-source-boundary-study-20261011`은 팔 action 명령 없이 초기3블록·동일 workcell camera와 물리 조건을 60초 제한으로 실행했고 exit0 종료했다. capture-01/npz·metadata 확보. 첫 캡처는 startup calibration 파일 미생성으로 실패해 생성 확인 후 같은 live simulator로 재시도했다. 그때 deque.append가 rclpy의 signature 검사와 호환되지 않는 오류도 재현돼 inspectable Python callback으로 수정. 실제 재캡처 exit0, 단위 callback signature/최신1개 보존 회귀 추가.
4. [x] `ur10e/scripts/analyze_pallet_rgbd.py` 재현 가능한 offline 진단 추가. 시뮬레이션 RenderProduct 계약 확인을 명시적으로 요구하고 metadata frame/shape/distortion/R을 검증한다. 각 component 원인, plane/boundary 한계, capture노출/보정revision을 기록. 모든 출력 collision_model_approved=false/uncertainty=not_certified/current_motion_freshness=not_evaluated_offline 유지. runtime detector/planner/형상은 변경하지 않음.
5. [x] 초기 장면 saturation50 진단: red 경계 residual1.594 mm>1.5 mm로 거부, green/blue 후보 각각 크기29.952×29.947/29.926×29.951 mm, 경계residual1.463/1.380 mm. 후처리 독립 평가에서 capture Unix와 가장 가까운 geometry gap0.286초: green/blue XY1.016/1.009 mm, Z+0.344/+0.356 mm, yaw mod90 오차-0.066/+0.085°. 실제truth는 estimator 입력에 넣지 않았으며 단일 static 캡처 평가여서 uncertainty bound로 채택하지 않는다.
6. [ ] 다음 첫 단계: 경계 후보의 픽셀 양자화·mask morphological opening·depth/plane noise에 대한 synthetic 민감도 및 실제 반복 캡처 분산을 확인한다. 1.5 mm 한계의 임의 완화로 통과를 만들지 않고 물리 인접 clearance에 필요한 bounds를 역산한다. 방향/위치 공동 안정화·방향별 TTL/무효화 및 보수적 collision bridge, 원래3블록 후속 목표는 유지한다.

검증: 관련86개 통과(경계22+capture8+offline4+입력24+plane10+기존depth18), diff check 통과. C++/런타임 planner/실기 변경 없음. 새 단위 테스트 초기1개 실패는 실제 hull 검사 결함 근거로 사용해 원인 수정 후 재검증했다.

재현 가능한 offline 명령(출력은 진단 JSON이며 모션 승인값이 아님):
```bash
cd /home/minwoo/Development/LLM
python3 ur10e/scripts/analyze_pallet_rgbd.py ros2_ur_ws/regression_results/rgbd-source-boundary-study-20261011/capture-01 --saturation 50 --confirmed-sim-render-contract
PYTHONPATH=ur10e PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q ur10e/tests/test_analyze_pallet_rgbd.py ur10e/tests/test_box_boundary.py ur10e/tests/test_box_pose_samples.py ur10e/tests/test_box_plane.py ur10e/tests/test_capture_pallet_rgbd.py ur10e/tests/test_depth_utils.py
```

## 반복 30 — mask opening 비교·독립 반복 캡처·픽셀 오차

1. [x] HSV detector에 선택형 opening_kernel_size(1/3/5/7/9) 추가. 기존 기본3 그대로이며 offline 분석 CLI/report만 옵션 연결. 기존 runtime callback은 기본값 사용. default 동일 mask/선택형 corner 유지/잘못된kernel 회귀6개 통과.
2. [x] 기존 source 캡처의 opening1/3/5 비교에서 red 모두 경계 residual 거부(각1.657/1.594/1.594 mm), green/blue는 모두 후보 통과. 전처리만 바꾸면 red 문제가 해결된다는 가설은 지지되지 않는다.
3. [x] `rgbd-source-repeat-study-20261011` 팔 action 없는 90초 제한 독립 시뮬레이션 exit0. 서로 다른 header노출22.008/23.975/26.008/27.892/29.758초의5개640×480 캡처/sync0. 동일sat50/opening3 진단에서 red0/5, green/blue5/5. green XYZ std 약(0.066,0.039,0.009) µm, blue(0.079,0.104,0.010) µm. 정적결정적렌더의 작은반복분산이며 약1 mm systematic bias를 제거하지 못하므로 정확도 bound로 쓰지 않는다.
4. [x] `ur10e/vision/pixel_plane_uncertainty.py` 구현: 고정된 실제 plane/K/transform에서 ±pixel 오차 직사각형을 plane에 투영한 최대변위. horizon 교차/뒤쪽투영/잘못된 입력 거부. 변환·plane·depth·mask 오류를 제외한 conditional footprint이며 완전한 pose bound 아님. 실제 source 경계 반픽셀의 최대변위 red1.112/green1.171/blue1.231 mm. analytic frontal/oblique densegrid/horizon/잘못된오차6개 검증 포함 관련Python102개 통과, diff check 통과.
5. [x] 다음 단일 후보는 측정 기준 완화가 아니라 RGB-D 해상도1280×960이다. 광학pose/focal/physics/safety와 경계gate는 유지하고 실제CameraInfo로만새K를쓴다.

## 반복 31 — 선택형 고해상도 입력의 실제 비교

1. [x] `--rgbd-resolution` wrapper→launch→Isaac의640x480(default)/1280x960 선택 연결. RGB/depth가동일RenderProduct width/height 사용, 실제PALLET_RGBD_RESOLUTION 기록. helper/launch2개추가(카메라검사총5), 기본보존·잘못된wrapper값모션전거부·bash문법·diff check 확인,2package build성공. 물리 dt/rendering dt·카메라위치·focal·안전·planner 변경없음.
2. [x] `rgbd-resolution1280-study-20261011` 실제1280×960/32FC1/sync0/CameraInfo fx=fy1465.9986 확인. header30.192/47.675/49.425초의3개 캡처에서 red/green/blue 모두 같은sat50/opening3/기존경계gate 진단 후보 통과. 각경계residual 약0.765/0.719/0.749 mm. capture후 독립truth평가(Unix가까운geometry gap0.076~0.495초)에서각XY약0.497/0.510~0.512/0.516 mm, Z+0.165/+0.170/+0.176 mm, yaw mod90약+0.01054/-0.00110/-0.00138°. nominal 방향 oracle을입력하지않음. static3프레임결과이며여전히collision_model_approved=false. 90초제한실행은handle22536에서종료확인예정.
3. [ ] source/target 실제pose·부트스트랩·calibration/quantization bound와여유검토후producer/store/loader/conservative bridge 연결. 기존두블록3연속은다른해상도후보의성공으로재사용하지않는다. 원래40 mm 인접3블록목표는 유지한다.

다음 첫 단계: 실제 관측 clock/노출/calibration revision/같은위치·방향샘플을 보존하는 선택형 measurement 계약을 설계·구현한다. square π/2 대칭 통계, 기존 방향의TTL연장 금지, 관측실패의방향무효화와불확실도누락시외접구 유지/모션전거부를 검증한다. target yaw45와 다양한workspace·가림·noise 캡처 및 bounds 검증 전에는 nominal OBB로 바꾸지 않는다.

재현: 기존 source-only 90초 진단 실행 인수에 `--rgbd-resolution 1280x960`만 추가했다. `analyze_pallet_rgbd.py`의모든품질한계는 그대로이다. 선택형해상도구현의카메라검사5개/2package빌드/bash문법/diff check통과, 앞선관련Python102개통과. 실제pose stream처리latency·motion동작·고해상도새후보E2E는아직검증하지않음.

종료 확인 추가: 고해상도90초진단 handle22536 및capture handle78230 모두exit0. 실행중/종료확인예정이라는상기표현은당시상태이며현재진단은종료했다.

## 반복 32 — 위치·방향 공동 저장 계약

1. [x] `ur10e/vision/box_pose_store.py`의 StableBoxPoseStore 구현. base_footprint/m 입력을 명시적으로 요구하며 position/측정normal/rotation/yaw modπ/2를 같은 샘플로 저장한다. rotation의SO(3)/normal/yaw 일치검사, mean(sin4θ,cos4θ)의square대칭 통계, 위치std·최대yaw/normal spread를 공동판정한다. 모든 한계는 호출자가 명시해야 하며 기존position-only store나camera/planner를바꾸지않는다.
2. [x] 노출stamp/clock와monotonic수신시각을분리하고 호출자가같은sensor clock에서검증한 exposure_age_s를요구한다. TTL은수신age+기존노출age로 계산하므로 늦은노출에 새전체TTL을주지않는다. 중복노출은TTL연장안함, 역순노출은방향·위치창무효화, clock/calibration변경은전창초기화. 새방향실패에도exposurewatermark가전진해뒤늦은옛샘플재사용방지. 누락/관측실패는명시적invalidate, 카메라재시작은reset_epoch.
3. [x] diagnostic_only/collision_model_approved=false/uncertainty=not_certified 고정. 안정된position으로실패방향을채우거나신뢰할수없는identity를추가하지않는다. source/revision/exposure/receiptage/stats 보존. 아직normalized quaternion/consumer OBB승인계약은연결하지않음.
4. [x] 고해상도actual3capture의offline 공동재생에서세블록각3샘플반환. sparsecapture 간격 때문에replay용TTL30초를명시했으며 live기본/안전기준으로채택하지않았다. exposure-age0은recorded-time 재생설정이고current freshness주장아님. artifact `rgbd-resolution1280-study-20261011/joint-store-offline-replay.json`에이구분보존. 독립평가1frame 및3frame정적분산을보정/관측오차bound로표현하지않는다.
5. [ ] 다음 첫 단계: 선택형실시간producer에서header clock 현재노출age를검증할 근거/arrival freshness·부하를확인하고joint measurement를별도진단출력으로연결한다. camera pose/timing/source frame·보정revision을실제관측에서보존한다. 기본camera vision position pipeline이나방향미측정외접구fallback은유지한다. 이후target yaw/다양한위치/가림·noise/보정bounds와loader/conservative collision bridge를검증해원래3블록으로확장한다.

검증: 저장기9개 + 기존detection14개 + 앞선관련102개=125 passed, diff check통과. square90°/180°대칭, 중복TTL/지연exposure TTL, reverse/새실패·calibration/clock·explicit무효화, 불안정position/yaw·rotation불일치/잘못된unit 검사. 새실패의watermark정책추가후같은stamp를재검증하던테스트1개실패는중복의정상거부이므로새stamp로독립검증하도록수정. 실제camera callback·planner/E2E/hardware 변경은아직없음.

```bash
cd /home/minwoo/Development/LLM
PYTHONPATH=ur10e PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q ur10e/tests/test_box_pose_store.py ur10e/tests/test_detection_store.py ur10e/tests/test_pixel_plane_uncertainty.py ur10e/tests/test_hsv_block_detector.py ur10e/tests/test_analyze_pallet_rgbd.py ur10e/tests/test_box_boundary.py ur10e/tests/test_box_pose_samples.py ur10e/tests/test_box_plane.py ur10e/tests/test_capture_pallet_rgbd.py ur10e/tests/test_depth_utils.py
```

## 반복 33 — 실제 system clock·실시간 공동 pose 진단

1. [x] 설치된 CameraHelper/CameraInfoHelper useSystemTime 계약 확인 후선택형 `--rgbd-clock simulation(default)/system` wrapper→launch→Isaac 연결. RGB/depth/Info셋모두동일선택. graph clock기록·초기값보존·잘못된선택모션전거부·카메라검사총6개/2package빌드/bash문법/diffcheck통과. 물리 dt/clock이나제어stamp는변경하지않음.
2. [x] 측정배열경로 analyze_arrays를offline/live가공유하도록분리. 새 `ur10e/scripts/live_pallet_pose_diagnostic.py`는카메라만구독해별도diagnostic latest.json/events.json 출력. 실제Unix노출age≤1초·RGB/depth/Info skew≤20ms·frame·보정·후보·공동store를검증. 0.25초throttle/3개sync큐/초기후보불완전·실패시invalidate, 무관측시timer snapshot만료. 기존vision/planner/제어topic변경없으며모든출력collision_model_approved=false.
3. [x] `rgbd-system-clock-live-study-20261011` system clock/1280입력90초 제한sim exit0. live-01 20초진단exit0:75events/3skew거부/3개three-stable events. processing P50/P95/max0.250/0.359/0.382초, exposure age0.581/0.699/0.723초. 공동안정화는관측됐지만여러샘플의총노출age가1초TTL밖으로나가빈도가낮다. 이값은카메라진단통계이고물리정지시간아님.

## 반복 34 — TTL 유지·bounded RANSAC 계산량 비교

1. [x] live --ransac-trials 선택 추가, 기존default2048 보존. 96후보도plane/경계/TTL한계는같고해당trials에서유효plane을못찾으면거부. 관련Python125개재통과.
2. [x] 이전sim이90초정상종료한뒤시작된 live-trials96은무관측0events/exit1. 후보실패나성공으로세지않는다. stopped source에서latest진단objects는비어있어관측누락을성공으로포장하지않았다.
3. [ ] 동일조건독립 `rgbd-live-trials96-repeat-20261011` 90초제한sim 시작. calibration생성/실제publisher확인후96trials 20초live진단의처리·노출age·공동안정화·거부율을2048과비교. pipelineclock/기준완화없음.
4. [ ] 결과검증후사전bounds·target yaw/다양한위치·가림/noise검증과consumer계약/conservative bridge 연결. 원래40 mm 인접3블록으로확장. 초기3블록진단이실제pick-place성공은아니다.

반복34 실제 종료 추가: 독립sim handle84313 exit0 및96trials live handle76390 exit0. 75events/3skew거부/68three-stable events. 처리P50/P95/max0.0836/0.0896/0.0947초, exposure age0.1933/0.2005/0.2068초. 2048trials의3/75안정화보다개선됐으나서로다른독립sim관측이며인과를완전히통제한성능보증/물리안전증거는아님. 모든품질/TTL한계는동일하다. 96trials에서plane을못찾으면계속거부하며기본2048을바꾸지않음.

## 반복 35 — 새 입력 조건의 E2E 재진입

1. [ ] 이전두블록45°/20 mm/100 mm/scene-obstacles후보에1280×960/system clock을명시해동일두블록첫시도·SLIP/fault/release·전체장면·freshness회귀검증. 공동pose진단은아직planner형상승인에사용하지않으며기존외접구/ACM유지. 이전기본해상도3연속결과를새입력검증으로대체하지않는다.
2. [ ] 실제다중작업의source/target·yaw45·가림진단과bounds를확인한뒤consumer/OBB enclosure검증으로원래40 mm 인접3블록확장. default채택/간격확대/기준완화없음.

## 반복 36 — 모션 중 관측·보수적 형상의 수학 계약

1. [x] 별도 진단 events에 candidate/stable 관측과 revision을 기록해 source/target 측정을 추적 가능하게 했다. 첫60초 live-pose-01은209events/90CameraInfo skew거부/three-stable0/exit1. green/blue후보119/119·stable113/113, red후보0(이동/가림·plane/size거부). 별도pose진단실패이지E2E모션실패는 아니다. 원본data/판정 보존.
2. [x] 최신Info와RGB/depth를따로읽는지연을보완해3message ApproximateTimeSynchronizer(queue3/slop10ms)로변경. skew/TTL기준완화없이같은3tuple관측검사. 현재동일live E2E의 live-pose-sync3 handle44495에서60초검증중.
3. [x] `ur10e/vision/box_enclosure.py`: 호출자가확립한world XYZ위치bound·SO(3)각도bound·size bound의conditional OBB enclosure계산. corner radiusR의rotation변위2Rsin(theta/2), dimension변위||ds||/2, position interval은abs(rotation.T)로project해각axis확장. std에서bound자동생성안함, 누락/비유한/음수bound거부. planner에는아직적용하지않고bound_source보존.
4. [x] 15-axis SAT로조건부2box분리검사. touching/수치모호성은clear로취급하지않고gripper/경로안전을증명하지않는다. 원래40 mm인접의yaw0nominal은clear, 양쪽yaw45는실제box겹침, 각positionbound6 mm도clear거부. random perturbation300개×8corner포함과잘못된입력등7개tests통과.
5. [ ] E2E handle25916은red종료/green시작, 전체job판정대기. source/target·동기화3·bounds근거와consumerbridge검증후원래3block범위로확장. 기존sphere/ACM유지.

검증: 관련Python138개통과(기존125+enclosure7+live CLI/wiring6), diff check통과. 현재error bounds는수학입력계약이며카메라실제정확도의인증값으로채택하지않았다.

반복35~36 종료 추가: 두블록 `two-block-rgbd1280-system-yaw45-20261011` wrapper0/job2/2/각첫시도, 전체scene/first_attempt/regression 모두true. XY red0.535/green1.354 mm, maxloaded1.529/2.274 mm, release확인/recontact없음/SLIP·fault없음/fresh/supervisor_stop. 실제각경로6개ACM검사,geometry307개/red방출후192/green42유지/미작업blue307초기자세유지, 최대gap1.027초/coverage0.060초. 새1280/system후보의첫1회성공이며이전3회로대체안함.

sync3진단은184events/시각거부0/three-stable0/exit1. red후보181·stable146(배치된target),green9·5(이동전source),blue106·94. motion중targetgreen가림으로전부동시안정화가없음. record후독립geometry평가에서각stable pose 최대XY red0.593/green0.512/blue0.516 mm, 최대Z0.200/0.171/0.176 mm, yaw mod90각0.0062/0.0062/0.0023°, 평가epoch gap≤0.506초. 측정단계truth입력없음, noise/보정상한인증값은아님.

## 반복 37 — 새 입력 후보 반복성·전체 작업 관측

1. [ ] 동일새입력두블록r2를독립반복하고첫시도/물리감시/전체장면회귀확인. wrapper실행중파일수정금지.
2. [ ] 선택형live진단self-duration을최대600초로bounded확장(기존default20초유지)해다중작업전체source/target노출을관측한다. 툴wait는60초이하로분할하고최종파일의empty snapshot과events의실제관측을구분한다. 기록만하며motion/planner가진단을승인하지않음.
3. [ ] 현재조건부enclosure에입력할noise/보정/관측성bounds와보수적bridge검증, 원래40 mm 인접목표의실제상대yaw와최종손벌림여유검토. 두블록yaw45를인접세블록에그대로확대하지않고square동일형상의90°대칭방향을검토하되지시좌표는유지한다.

## 재현

```bash
cd /home/minwoo/Development/LLM
ros2_ur_ws/scripts/run_palletizing_gui.sh --headless \
  --task-spec ros2_ur_ws/scripts/pallet_task_two_blocks.json \
  --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/two-block-place-yaw45-release020-r2-20261011 \
  --velocity-scaling 0.2 --acceleration-scaling 0.05 \
  --gripper-close-position 0.002 --release-open-position 0.020 --grasp-angle-delta 6.283185307179586 \
  --place-yaw 0.7853981633974483 --upright-transfer --transfer-height 0.10 --scene-obstacles --camera-view workcell \
  --block-contact-offset-m 0.002 --block-static-friction 1.0 --jaw-friction 1.0 \
  --contact-geometry-log --external-forces-every-iteration --totg-path-tolerance 0.01
python3 ros2_ur_ws/scripts/analyze_pallet_scene.py ros2_ur_ws/regression_results/two-block-place-yaw45-release020-r2-20261011
python3 ros2_ur_ws/scripts/analyze_pallet_transfer.py ros2_ur_ws/regression_results/stage1-pad-geometry-20261009/job/01-red_block-motion.json ros2_ur_ws/regression_results/stage1-pad-geometry-repeat2-20261009/job/01-red_block-motion.json
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s ros2_ur_ws/src/pallet_gripper_moveit_config/test -p 'test_*.py'
```

## 반복 38 — 결과 확정·픽셀 중심 보정·물체별 실행 방향

작성일: 2026-10-11. 기준 workspace `/home/minwoo/Development/LLM`.

- 문제 → 원인: 새입력 r2 red attempt1에서 move above pick 계획 실패(code1, 약190초), attempt2 성공. job2/2 성공과 최초시도 성공은 다르다. 전체장면502샘플, 최대gap1.2596초, 완료coverage0.6114초, red 방출후192/green43/blue502샘플 통과. 최종XY red0.485/green1.354 mm, 최대적재상대변위2.011/2.057 mm, SLIP/fault/recontact 없음. regression_passed=false 유지. shutdown에서 move_group exit-11도 관측해 실행종료 정상으로 포장하지 않는다.
- 측정 문제 → 원인: 이전 source에서 반복분산은 작지만 XY약0.5 mm bias. 설치된 Isaac Camera.get_pointcloud는 pixel(u+.5,v+.5), K중심은width/2,height/2였다. 원시K는 유지하고 confirmed simulator 진단 경로에서만 offset+.5로 보정한다. generic/hardware 샘플의 기본offset0은 유지한다.
- 해결·검증: box_pose_samples/analyze_pallet_rgbd/live_pallet_pose_diagnostic에 보정과 K/D/R/해상도/알고리즘/품질정책 포함 measurement revision 연결. 기존3capture의 새 boundary-pixel-center-v2.json에서 XY6.27~12.32 µm/Z-2.28~-3.08 µm. 정적 시뮬레이션3관측이며 실제카메라 bound나 planner 승인을 의미하지 않는다. live-full-job은 이전offset0 코드1141events/603three-stable; live-pixel-center-v2-tail은 events=[]/objects={}로 새보정 live 성공 아님.
- 실행 방향: 팔 pad축 관측은 requested yaw-90° mod180. 원래blue→green→red 40 mm 셀의 후보는blue0/greenπ/2/red0. 실제30 mm 정사각45° 투영폭42.426 mm이므로 인접40 mm에 모두45°를 적용하지 않는다. 이방향은 정적pad 여유 가설이지 전체gripper/path 충돌보증 아님.
- 구현: palletizing_job.py --object-place-yaw OBJECT RAD 반복옵션·유일한job object/finite[-π,π] 검증·globalyaw보다 우선·report.planning_tuning 기록. wrapper에서 배열로 전달, --validate-only로 출력생성/ROS시작 전 검증. TaskSpec에 controller 세부값을 넣지 않고 기본fallback 유지. 실행중wrapper 수정은 피했고 관련 프로세스 종료 확인후 수정했다.
- 검증: 관련Python177 passed(픽셀중심·revision·jointstore·bounds·job/spec·wrapper 잘못된입력 포함), bash -n/git diff --check 통과. 실기·커밋·푸시 없음.

## 반복 39 — 갱신된 다음 단계

1. [x] 첫 단계: diagnostic plane inlier 거리0.5 mm(default) 대비0.1/0.05 mm를 원시capture로 비교. 더 엄격한 기준만 선택 가능하며 실패는 그대로 기록한다. plane/normal·경계후보·revision과 계산량을 비교하고 oracle 방향/위치를 estimator에 주입하지 않는다.
2. [ ] source뿐 아니라 target yaw와가림/보정·depth·mask 오류를 확인해 명시적 uncertainty bound와 loader 계약 검증. 진단output의collision_model_approved=false는 유지한다.
3. [ ] consumer/보수적형상과 전체gripper 충돌검증 후 원래3셀·물체별yaw 후보를 모션전 검사하고 E2E. 현재 외접구가 허용하지 않는 경로를ACM으로 숨기거나간격을확대하지 않는다.
4. [ ] 새입력 조건의 첫시도 연속회귀 재시작. 기존r2의재시도를통과횟수로세지 않는다. planning timeout과 shutdown crash는 별도 원인 분석 대상으로 유지한다.

반복39 결과: analyze_pallet_rgbd의 선택형 --plane-distance-limit-m은0 초과0.0005 m 이하만 허용하고 기본0.5 mm 유지. planefit과top boundary의점선택을같은거리로연결, revision에포함. 세capture×세거리별9개독립artifact를원본보존해추가했고 각거리에서실제세블록후보9/9 통과. background ROI 거부는유지했다. 최대normal기울기(default/0.1/0.05 mm) 0.00047618/0.00002775/0.00000707 rad, plane RMS최대35.860/2.721/1.228 µm, hull잔차최대0.76553/0.75942/0.75942 mm. 각분석0.220~0.236초 관측이며성능보증아님. normal은oracle수직방향으로대체하지않았다. 기존기본경로는그대로이고0.05 mm가노이즈/다른target에서유효한지아직미검증. 변경관련63 passed(신규7 포함), bash -n/diff check통과. 앞선전체177테스트도변경전통과.

## 반복 40 — 다음 계획

1. [ ] 첫 단계: stricter plane 거리옵션을 read-only live 진단에만연결하고CLI/revision/default검증. 독립적인bounded무동작sim으로실시간후보·처리량·freshness를확인한다. camera/planner/source좌표보정의실행pipeline은변경하지않는다.
2. [ ] target yaw0/90/45 및가림/rawcapture로측정계약검증. 정적source3개만으로bound를승인하지않는다.
3. [ ] 명시적오차범위/loader·보수형상·전체gripper clearance 검증 뒤 원래40 mm3셀확장. 첫시도재현성과shutdown오류는별도추적한다.

반복40 진행: read-only live 진단에 --plane-distance-limit-m 연결, 기본0.0005 m 그대로. finite/0초과/상한검사는ROS초기화/출력생성전에수행하고analyze_arrays에전달한다. 새CLI실패3개포함analyzer/live21 passed, diff check통과. 다음 첫실행은1280/system의90초 bounded 무동작sim 및0.00005 m/96trials live 관측이다. 이전v2tail 무관측을대체하는실제새관측이필요하다.

반복40 실행 결과: 첫무동작sim은 --calibration-block 누락으로geometry logging 예외가발생, actualevents0/capturetimeout. 실패로그보존. Simulator가예외를출력했는데exit0인점도기록했고성공으로세지않았다. 해당조합은이제app시작전CLI검사로exit2 거부한다. 블록생성옵션수정후 별도r2 sim handle69081 exit0, PALLET_FINAL 로그확인. stricter0.05 mm/live96trials는75events/0거부/73three-stable, 처리P50/P95/max0.07766/0.08227/0.08845초, exposure age0.18622/0.19205/0.23681초. 같은실행순차default0.5 mm도75/0/73, 처리0.08846/0.09333/0.14649초·age0.19778/0.20932/0.25694초. 각stream revision은1개로유지되고서로다르다. 정적시뮬레이션동일실행의순차비교이고causal성능보증은아니다. 원시capture1280×960/32FC1/sync0 보존.

## 반복41 — 원래 목표 위치·방향의 독립 정적 관측

1. [x] bounded simulator-only --diagnostic-block-pose OBJECT X_M Y_M YAW_RAD 추가. 알려진유일object/finite/테이블내실제회전footprint 검사, three/calibration-block/RGB-D/0초과120초이하/no fault injection 필수. 기본source배치는유지하며 estimator metadata에object truth를넘기지않는다. 진단fixture일때 /isaac_joint_commands 구독을생성하지않아모션명령과분리한다.
2. [ ] 첫실행: 원래red(.24,-.28,0),green(.28,-.24,π/2),blue(.28,-.28,0) 정적90초sim 시작(handle72086). startup/commands-disabled/settledtruth확인후rawcapture/live의실제관측과가림을확인한다. pick-place완료로세지않는다.
3. [ ] 관측오차/불확실도·보수적bridge·실제전체gripper clearance 확인후세블록E2E. 방향미측정외접구/모션전거부는유지한다.

주요파일: ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/{isaac_sim.py,workcell_scenario.py}, ur10e/scripts/{live_pallet_pose_diagnostic.py,analyze_pallet_rgbd.py}. 조합거부/좌표·yaw/기본보존/명령구독차단/reset기본pose 회귀19 passed, diff check통과.

반복41 실행수정: 첫fixture handle72086은exit0이나PALLET_BLOCK_LAYOUT_READY/실제geometry가source로되돌아갔다. set_world_pose만지정하면world.reset이초기constructor pose를복원하는원인을설치된SingleRigidPrim.set_default_state문서·실제로그로확인했다. 목표관측성공아님. fixture set_default_state와set_world_pose에동일position/scalar-first quaternion을설정하고startup settling후요청XYZ오차0.1 mm/rotation0.001 rad 이내검사를추가했다. 이검사는fixture유효성이고기존운반안전gate변경아님. 같은목표의수정r2(handle9909)를새폴더rgbd-static-original-targets-r2-20261011에서시작했다. 재실행전첫sim종료확인.

반복41 실제종료: 수정fixture sim handle9909 exit0/PALLET_FINAL, startup정착XYZ와commands-disabled 확인. stricter0.05 mm/opening3 live handle68081은74events/시각거부0/three-stable0/exit1. green의full hull residual1.641 mm>1.5 mm로후보거부이며기준은유지했다. 같은rawcapture opening1/3/5 비교에서green잔차1.067/1.641/1.641 mm로opening1만후보통과. red/blue도opening1에서통과했다. 기존source640의opening비교실패와다른해상도/target/픽셀보정조건이므로이전결과를덮어쓰지않는다.

live에선택형 --opening-kernel-size를연결하고기본3 유지, full hull/corner거부gate유지. opening1 live handle90197 exit0, 15events/0시각거부/12three-stable. publisher종료까지의노출span4.002초뿐으로20초전체관측주장은하지않는다. independent-evaluation.json에서추정완료후독립truth평가 최대XY red51.616/green67.288/blue68.187 µm, 최대yaw mod90 red0.003074/green0.001284/blue0.000637 rad, nearesttruth gap≤0.400초. 정적조건의관측값일뿐bound인증아님. rawrgb.png 실제화면도검토했다. 경계/전처리/analyzer/live54 passed, fixture/CLI19 passed, diff check통과.

## 반복42 — 갱신된 다음 계획

1. [x] 첫단계: 독립적인원래target0/90/0 bounded무동작sim에서opening1/plane0.05 mm/96trials를publisher발행확인직후20초이상관측하고서로다른노출3rawcapture 보존. startup/reset/실제target/yaw와commands-disabled, freshjointpose, 후보거부율과비용확인. source기본fixture가아닌actualtarget인지truth로그로확인한다.
2. [ ] source opening1·targetyaw45·부분가림/보정변경에대한품질거부유지검증. 기본detector/planner나전처리채택은검증전보류.
3. [ ] 명시적uncertainty bound 계약/loader·conservative bridge·전체gripper clearance검증후원래3셀E2E. 첫시도재현성과shutdown오류별도추적. 정적fixture는pick-place성공이아니다.

반복42 결과: rgbd-static-original-targets-r3-20261011 sim handle40515 exit0/PALLET_FINAL 및실제target정착/commands-disabled 확인. live handle10336 exit0, 실제노출span29.849초/113events/시각거부0/92three-stable. red/blue후보113·stable111, green후보106·stable92(7회full hull 기준초과). 관측실패는invalidate하며기준을완화하지않았다. 처리P50/P95/max0.08384/0.09039/0.14163초, exposure age0.19242/0.20622/0.25509초. capture-01~03 노출Unix1791654153.874/4154.886/4156.146, 모두1280×960/32FC1/sync0/실제3후보통과. target최대hull residual1.11037 mm. 기존source3raw도동일opening1/0.05 mm 재분석해각3후보통과, 최대0.759425 mm. source/target중일부정적조건이유효하다는증거이며새오차bound인증/모션성공으로세지않는다.

## 반복43 — 방향·관측오차 계약 확장

1. [x] 첫단계: 실제target에서green만yaw45°인분리된정적fixture(red/blue0°)로raw3capture/live30초검증. 원래0/90/0 최종배치후보를변경하지않고방향관측범위를검증한다. quarter-turn 대칭을고려해실측normal/rotation/yaw를truth와추정후별도로비교한다.
2. [ ] 공통source/target/yaw0/90/45·품질거부/가림·보정변경검증을바탕으로measurement loader·명시적uncertainty policy계약을구현한다. 반복분산을hard bound로대체하지않는다. 미승인/만료/잘못된revision이면외접구유지또는모션전거부한다.
3. [ ] 보수적bridge·전체gripper clearance와실제3셀E2E/최초시도회귀. 안전gate·목표좌표·기본전처리는유지한다.

반복43 결과: rgbd-static-target-green45-20261011 sim handle31618 exit0/PALLET_FINAL, 실제target정착/명령구독차단확인. raw3노출모두후보3/3, green measuredyaw약0.786281 rad(actualπ/4), hull최대1.08932 mm. live handle43461 exit0/113events/시각거부0/58three-stable/노출span29.87155초. red/green 후보113·stable111, blue 후보84·stable58이며29회observed square size 초과를거부했다. 처리P50/P95/max0.08391/0.14729/0.17390초, age0.19235/0.26080/0.30966초. independent-evaluation.json의추정후독립truth평가 최대XY red52.581/green38.940/blue71.036 µm, quarter-turn대칭full rotation오차0.003075/0.001798/0.000637 rad, normal오차최대2.264e-6 rad, truthgap≤0.501초. 이전원래targetr3 독립평가도추가: 최대XY red52.589/green87.577/blue74.085 µm/yaw0.003075/0.001724/0.001163 rad. 정적sample observed-error이며bounds인증아님.

독립reader계약검토결과: 기존load_pallet_scene는pose provenance를버리고SceneManager.update_object는orientation=None에옛방향을유지한다. 승인pose를이경로에그대로섞으면새위치+옛방향위험이있다. 현재CPP/launch는sphere만지원하므로Python에R을추가하는것만으로OBB가적용되지않는다. 현재진단flag(false approval/not_certified)를바꾸거나bound_source 문자열만으로인증하지않는다. 문서검토는코드변경/실행승인아님.

## 반복44 — 갱신된 다음 단계

1. [x] 첫단계: 별도diagnostic-only snapshot reader 구현. root/item diagnostic flags 정확한검사, Unix now에서root timestamp와각exposure age 재계산, future/stale/wrongclock/frame/unit/revision·알수없는object/missing3/불일치position-normal-R-yaw 거부. 같은bundle을immutable하게반환하고legacySceneManager/실행planner에는넣지않는다. 진단통계를hard bound로승인하지않는회귀추가.
2. [ ] 실제원시source/target 및가림·noise/보정조건의명시적오차bounds 정책과실행승인provenance를검증. snapshot memory갱신/실패시방향·bound·approval atomic무효화계약설계. source/target제한관측을전체workspace인증으로확대하지않는다.
3. [ ] typed sphere/box Python→launch→C++ bridge와보수적box pose/size값·실제PlanningScene 대조,전체gripper/path 충돌검증. 모든전제충족뒤원래40 mm3셀E2E/최초시도회귀. 미승인fresh중심이면기존외접구,중심만료/누락이면모션전중단.

반복44 결과: vision/box_pose_reader.py에DiagnosticPoseReader/immutable tuple 기반DiagnosticBoxPose·Snapshot 추가. expected revision/maxage(0,1] explicit필수, JSON숫자bool/string/NaN 거부. root/item false approval/uncertified flags, Unix 노출age 현재계산, 미래·만료·잘못된clock/frame/unit/source/revision·missing/unknown객체, SO(3)/normal/yaw일치, diagnostic statistics 검증. 전체bundle 검증후watermark atomic갱신, 중복·역순·write시간역행거부, 실패에cached pose 반환안함. 기존SceneManager/producer/planner 수정없음. reader/store/enclosure61 passed. 실제원래targetr3 및green45 latest를recorded timestamp재생에서각3객체수락·현재Unix조회에서는stale거부, reader-contract-replay.json보존. 재생을current freshness로주장하지않는다.

## 반복45 — 조건부 오차 범위 정책 계약

1. [x] 첫단계: diagnostic measured pose와조건부enclosure 사이에서명시적position XYZ/full SO(3)/dimension 오차범위·nominalsize·measurement revision·유효시각/관측영역·가정/근거를검증하는정책계약추가. 정책은미인증가정이며실행승인false 유지. std/spread를bounds로변환하거나40 mm를통과시키려실제bounds를임의설정하지않는다.
2. [ ] 실제camera/depth/calibration/mask오차의validity근거확보와관측실패·memory갱신시joint 무효화. 이후에만별도실행승인policy를고려한다.
3. [ ] typed sphere/box bridge·실제PlanningScene 대조·전체gripper/path 검증후원래3셀E2E/최초시도회귀.

반복45 결과: vision/box_bound_policy.py의 ConditionalBoxPolicy는 nominal size/position XYZ/full SO(3)/dimension bounds, measurement revision, 유효 Unix 시각과 위치 영역, bound_source와 assumptions를 명시적으로 요구한다. 유한 JSON 숫자·bool 거부·차원/비음수/구간·영역 검사, 사용 시각에 exposure age·policy expiry·revision·위치 오차 구간 전체의 영역 포함 여부를 재검증한다. 측정 normal/R/yaw도 사용 시 재검증한다. 결과는 ConditionalDiagnosticEnvelope로 diagnostic=true/approval=false/conditional_assumptions_not_certified 유지. 표준편차가 변해도 명시 bounds가 같으면 enclosure가 같음을 회귀 검증했고 숫자·문구만으로 실행 인증하지 않는다. reader JSON duplicate key 거부 추가, reader 출력 scalar를 immutable Python float tuple로 정규화. 연관107 passed(duplicate JSON 추가 전), 최종 reader/policy70 passed, diff check 통과. 정책 테스트 값은 synthetic test only이고 실제 camera 오차값으로 채택하지 않았다. 기존 SceneManager/모션/producer 변경 없음.

## 반복46 — 실제40 mm 셀의 조건부 여유 민감도

1. [x] 첫단계: recorded target0/90/0 측정의조건부enclosure와SAT를여러가정bounds에서비교해두상자의분리가유지되는요구조건을분석한다. 숫자는sensitivity-only이며오차인증이나채택bounds가아니다. replay time과현재freshness를구분하고모든출력승인false 유지. nominal외접구와의차이도명시한다.
2. [ ] calibration/depth/pixel/mask/관측영역의실제bound 유효성근거확보. 유효bounds를증명하지못하면외접구/모션전거부유지. reader/policy를legacymemory에섞지않는다.
3. [ ] 실행승인provenance와typed sphere/box bridge의Python→launch→CPP/PlanningScene 계약·전체gripper/path 검증후원래3셀E2E/최초시도회귀.

반복46 결과: scripts/analyze_pallet_enclosure_sensitivity.py 추가. 명시적인 nominal size/위치 XYZ 등방 오차 grid/전체 SO(3) 각도 grid/dimension 오차/수치 margin/관측영역/revision이 필수다. grid는 최대400개이며 비유한값/음수/bool/영역이탈을 거부한다. 현재 Unix 시각이 기본이며 과거 기록은 명시적인 --replay-recorded-time으로만 분석한다. enclosure_axis_gap은 최대 signed SAT 투영 간격이며 최단 Euclidean 거리나 전체 gripper 여유가 아니다. 기존 SAT 불리언도 같은 간격 기준을 공유한다.

실제targetr3의recorded latest에서position오차각축0/1/2/3/4/5/6 mm×full rotation0/.005/.01/.025/.05 rad, dimension오차0, numerical margin1 µm의35가정조합24개분리. zero 가정의최소투영gap10.0245 mm, ±4 mm/0.01 rad는1.33974 mm, ±4 mm/0.05 rad는-1.29323 mm, ±5 mm/0은-0.01611 mm로거부. nominal unknown-orientation sphere의signed gap red-green -9.079/red-blue -25.498/green-blue -25.539 mm로원래격자에서모두중첩. hypothetical값은실제bounds로채택하지않음. artifact rgbd-static-original-targets-r3-20261011/enclosure-sensitivity.json에approval=false/replay/주의사항보존. analytic40 mm/접촉/SO(3)·dimensioninflation/잘못된grid·scope 회귀포함88 passed, diff check통과. 기존모션/planner/안전기준변경없음.

## 반복47 — 독립 시뮬레이션 형상 포함 검증

1. [ ] 첫단계: 조건부측정box가독립reference box의전체convex geometry를포함하는지검사하는순수geometry guard 구현·검증. 측정center/R을truth로고치거나없는방향을채우지않는다. 기록된actualsim pose·확인된30×30×50 mm와대조하며기록시간/replay·reference freshness·물리contact margin과실행승인은구분한다.
2. [ ] 실제계획/실행에쓰기전simulation-only fresh reference 기반포함검사의타당성과contact offset/fullgripper geometry까지범위를검증한다. 이검사는제한된sim형상proof이지실제카메라오차의전역인증이아니다. live조건과검증범위가충족되지않으면미승인/외접구/중단유지.
3. [ ] 승인provenance·typed sphere/box bridge와실제PlanningScene 대조·전체gripper/path 검증후원래3셀E2E/최초시도회귀. hardware승인은포함하지않는다.

## Codex 세션 재개

```bash
cd /home/minwoo/Desktop/LLM
codex resume 019f8001-247a-73b2-b9ba-74eed73c3f71
```
