# 실물 MoveIt 위치제어: 현재 실행 기준

작성일: 2026-10-30. 이 문서는 `SERIAL_BRIDGE.md`의 과거 기록보다 현재 설정을 우선한다.
기준 workspace: `/home/minwoo/Development/LLM/ros2_ur_ws`.

## 변경과 검증 범위

| 발생한 문제 | 현재 처리 | 검증 |
| --- | --- | --- |
| 오래된 관절값으로 Execute 실패 | 완전한 새 실측만 발행하고 가장 오래된 축의 측정 시각을 stamp에 반영한다. 정상 피드백 20회 연속 확인 전 Execute를 거부한다. | 측정 시각·누락 샘플·준비 상태 시험 |
| 가상/실물 관절값 혼합 | `/joint_states` 발행자가 하나일 때만 실행한다. 실물 브리지의 `use_sim_time=true`도 거부한다. | 중복 발행자 거부 시험 |
| 간헐적 레지스터 timeout | 누락 요청만 최대 2회 읽는다. 100ms 중 18ms를 명령용으로 예약하고 읽기는 82ms 안에 완료해야 한다. 이전 값이나 0으로 대체하지 않는다. | 재읽기·deadline·고장 전파 시험 |
| 앞 축 STOP ACK 대기로 뒤 축 정지 지연 | 6축에 먼저 STOP을 보내고 공통 100ms 창에서 ACK를 수집한다. 미확인 축만 최대 3회 요청한다. 미확인이 남으면 실패다. | RX/TX 실패·누락 ACK 시험 |
| 궤적 속도·가속도를 무시한 위치 보간 | 위치만 있으면 선형, 속도까지 있으면 3차, 가속도까지 있으면 5차 보간한다. 점 사이의 범위·속도·가속도 최대값도 검사한다. | 연속 구간 극값·MoveIt TOTG 드라이런 |
| 목표 도달 후 유지 실패 | 0.2초 도달 확인 후 5초간 목표를 계속 보낸다. 유지 중 최종 오차 이탈은 `GOAL_TOLERANCE_VIOLATED`다. 유지 종료 후 STOP한다. | 유지·취소·드리프트 시험 |
| 고장 후 재실행·STOP 오류 유실 | 실행 중 고장은 latch하고 원래 오류와 STOP 실패를 함께 남긴다. reset은 STOP 확인과 연속 실측을 통과해야 하며 enable하지 않는다. | latch·reset·동시 목표 거부 시험 |
| J5 비교 도구 cleanup 실패 | 로그가 열린 상태에서 cleanup하고 원래 오류와 복원 실패를 따로 남긴다. 준비 중 임시 PP 설정을 조기 복원하지 않는다. | cleanup·프로파일 보존 시험 |

이번 환경에는 `/dev/ttyUSB0`가 없어 실물 모터를 구동하지 않았다. 읽기 전용 시험과 ROS
드라이런은 하중 상태의 떨림·정밀도·USB 지연을 검증하지 않는다. PD gain, 모터 내부 PP
속도/가속도, 전류 제한은 이번 작업에서 변경하지 않았다.

소프트웨어 검증: 관련 테스트 197개, 두 패키지 빌드, MoveIt 10점 계획·실행·취소·reset 통과.
원본 결과: `/home/minwoo/Development/LLM/ros2_ur_ws/validation/serial_reliability_20261030/result.json`.
같은 디렉터리의 `launch.log`와 `software_checks.json`에 실행 로그와 검사 범위를 남겼다.

## 현재 설정

파일: `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/config/serial_trajectory_live_tested.yaml`.

- 921600 baud, 주기 0.1초, 개별 피드백 timeout 0.015초, watchdog 0.25초.
- 정상 피드백 20회 연속 + 최근 0.2초 이내 측정 + STOP 확인 후 `READY`.
- 상위 속도 1rad/s, 가속도 1rad/s². RViz 배율을 곱한 값으로 계획한다.
- 경로 오차 0.01rad, 최종 오차 0.001rad, 수렴 대기 최대 3초, 최종 유지 5초.
- 전류 제한 기대값 J1~J6: 16 / 57 / 16 / 11 / 11 / 11 A. 불일치 시 enable하지 않는다.
- 모터 내부 PP 설정을 보존한다. 상대 이동 제한은 이전 사용자 요청대로 해제되어 있으며 URDF 관절 범위는 검사한다.
- MoveIt 실행 시간 여유는 준비·수렴·유지·정지 확인을 포함해 15초다. 실행 감시는 유지한다.

이전 실물 검증 기록은 외부 지지 상태의 작은 이동 범위다. 전체 가동범위나 하중 상태 검증을 뜻하지 않는다.

## 터미널 1: 읽기 전용 통신 확인

먼저 Motor Studio, 각도 읽기 프로그램, 다른 feedback/MoveIt launch를 종료한다.
포트는 한 프로세스가 소유한다. 이 도구는 STOP·Enable·목표각·설정 쓰기를 하지 않는다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p measurements/position
ros2 run pallet pallet_serial_health_check.py --read --samples 100 \
  --output "measurements/position/serial_health_$(date +%Y%m%d_%H%M%S).jsonl"
```

마지막 `health_summary`의 `passed: true`, `missing: 0`을 확인한다. 실패하면 `missing_sample`의
축/레지스터와 `current_limit_read`를 확인한다. 제한값을 임의로 늘려 통과시키지 않는다.
`--read`를 생략하면 포트를 열지 않는 OFFLINE 설정 확인만 수행한다.

## 터미널 1: 실물 MoveIt 실행

읽기 도구 종료 후 실행한다. 시작·종료 시 6축 STOP을 확인하며 토크가 해제될 수 있으므로
기존 검증에서 사용한 외부 지지를 유지한다.

```bash
ros2 launch pallet_physical_moveit_config serial_bridge.launch.py \
  dry_run:=false \
  config_file:=/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/config/serial_trajectory_live_tested.yaml
```

기본 launch는 DRY RUN이다. 실물 실행에는 위 두 인자가 모두 필요하다.
관찰만 하려면 같은 명령에 `observe_only:=true`를 추가한다. 관찰 모드는 쓰기를 보내지 않고 Execute를 거부한다.

## 터미널 2: 상태와 복구

```bash
source /opt/ros/jazzy/setup.bash
source /home/minwoo/Development/LLM/ros2_ur_ws/install/setup.bash
ros2 topic echo /pallet_serial_bridge/status
```

`LIVE READY`를 확인한 뒤 RViz의 `pallet_arm` 그룹에서 Start State를 current로 설정하고 새로 계획한다.
초기 확인은 속도/가속도 배율 0.05로 진행한다. 이 배율이 실물 떨림 해소를 보장하지는 않는다.
현재 실제 환경의 충돌 물체를 Planning Scene에 반영해야 한다. 여러 PC를 쓰면 ROS 시계 동기화가
별도로 필요하며 모든 실물 노드는 `use_sim_time=false`여야 한다.

대기 중 단순 누락은 `NOT_READY`로 바뀌고 연속 정상 샘플을 다시 모은다.
실행 중 통신 실패·추종 오차·모터 fault는 정지 후 `FAULT_LATCHED`가 된다.
원인 해결 후 다음 명령으로 정지와 피드백을 재확인한다.

```bash
ros2 service call /pallet_serial_bridge/reset_fault std_srvs/srv/Trigger '{}'
```

`success: true`여야 복구된 것이다. STOP 미확인이나 읽기 실패가 남아 있으면 복구하지 않는다.
복구 후에는 이전 계획 대신 현재 자세에서 다시 계획한다. 기존 RViz Stop 이벤트 경로도 유지한다.

```bash
ros2 topic pub --once /trajectory_execution_event std_msgs/msg/String "{data: stop}"
```

## 소프트웨어 검증 재현

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
pytest -q src/pallet/test src/pallet_physical_moveit_config/test
python3 validation/serial_reliability_20261030/verify_serial_dry_run.py
```

통합 도구는 별도 ROS domain 89와 `dry_run:=true`를 사용한다. MoveIt 계획·실행, 두 번째
목표 거부, 취소 후 정지, reset 서비스와 정상 종료를 확인한다. 실물 포트에는 접근하지 않는다.

## 남은 실물 검증과 그리퍼

- [x] 소프트웨어 통신 deadline·정지·상태 발행·보간·유지·복구 검증.
- [x] 실제 MoveIt 계획기를 사용한 가상 실행 연결 확인.
- [ ] 사용자 장치에서 100회 읽기 전용 통신 시험.
- [ ] 외부 지지 상태의 작은 궤적에서 실제 추종 오차·주기·5초 유지 기록.
- [ ] 같은 조건에서 떨림 확인 후 모터 PP 응답·마찰·하중 및 gain 원인 분리.
- [ ] ID7 그리퍼 실측과 동작 보정. 현재 `g_r/g_l`은 표시용 0이며 실물 제어가 아니다.

100ms마다 PP 목표를 보내는 구조에서 보간 변경만으로 기계적 떨림이 사라졌다고 판단할 수 없다.
통신 시험이 실패하면 통신부터 해결하고, 통과 후 실측 추종 기록으로 위치제어 응답을 평가한다.

## Offline arm command rate replay

`ros2 run pallet pallet_arm_rate_replay.py /path/to/bridge.log` reads the most
recent motion in a recorded `ARM_CYCLE` log. It estimates position step size
and the transmit budget for different J1–J6 rates while keeping gripper
commands and all seven axes' feedback at 10 Hz. It does not open a serial port.

The current three parameter reads per axis and 3 ms transmit spacing yield
280 outgoing frames/s at 10 Hz and 340 frames/s with only J1–J6 commands at
20 Hz. The latter exceeds the pacing allowance before reply delays and
retries. Reducing or decoupling feedback transactions is necessary before
raising the live rate. Replay estimates command spacing. Confirm actual
feedback age, retries, STOP acknowledgement, and arm vibration separately
on hardware before changing live defaults.
