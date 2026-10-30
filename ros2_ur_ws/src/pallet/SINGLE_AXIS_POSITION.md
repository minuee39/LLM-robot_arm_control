# USB 직렬 팔 단축 PP 위치제어

작성일: 2026-10-27. 기준 workspace: /home/minwoo/Development/LLM/ros2_ur_ws.
중력보상·강화학습은 사용하지 않는다. 이 단계는 MoveIt action 연결 전 단축 시험이다.

## 현재 완료 상태

- J1~J6과 실물 ID1~6, hand_guiding.yaml의 기존 영점/방향/비율 사용.
- 기본 실행은 OFFLINE. 장치 열기·STOP·ENABLE·모터 명령을 하지 않는다.
- LIVE 경로는 별도 명시, 실물 제한값, 팔 외부 지지 및 PP watchdog 검증이 필요하다.
- 실제 위치 유지·하중 추종·비상정지·낙하 방지 성능은 아직 검증하지 않았다.
- 그리퍼 ID7, 6축 동시 제어, ROS FollowJointTrajectory action은 포함하지 않는다.

## 기본 드라이런

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pallet pallet_single_axis_position.py --joint J1 --delta-rad 0.01
```

기본은 가상 현재 관절각0rad에서 J1에 +0.01rad 변화(약0.57도)를 계산한다.
J1의 direction=-1이면 가상 모터 목표는 -0.01rad이다. 실제 측정값이 아니다.
출력 mode=OFFLINE_NO_DEVICE_ACCESS, actuation=false를 확인한다.
드라이런의 simulated_limits는 계산 예제이며 실물 설정 파일에 자동 반영하지 않는다.

## 실제 통신·실행 구조

```text
실측 원시 모터각 → joint = direction * (raw-zero)/ratio
관절 상대 변화량 → 목표 joint → raw = zero + direction*joint*ratio
STOP/Reset 확인 → PP 모드와 제한 설정 → 현재각 LOC_REF 검증
→ 설정 중 이동 여부 확인 → ENABLE/Run 확인 → 작은 목표 이동
→ 위치/전류/속도 감시 → 목표 도달 및 지정 시간 유지 관측 → STOP/Reset 확인
```

- PP run_mode=1. 목표각은 원시 모터 rad이며 정규화/회전수 축약하지 않는다.
- 0x7005(mode), 0x7018(current cap), 0x7028(watchdog), 초기0x7016(loc_ref)는 독립 읽기로 검증.
- 0x7024(vel_max), 0x7025(acc_set)는 RS05 매뉴얼상 W 항목이므로 readback하지 않는다.
  로그 profile_sent_write_only는 전송 기록이며 실제 적용값 검증 성공을 뜻하지 않는다.
- 프로필은 ENABLE 전에만 설정한다. 실행 중 속도·가속도 변경 없음.
- 모터별 3ms 최소 송신 간격, 선택 ID/host 피드백·fault 확인은 기존 직렬 코드를 재사용.
- 시간 초과, 실제 속도/전류/이동거리 초과, 관절 한계, 취소/신호는 STOP 경로로 종료한다.
- STOP 확인은 오래된 수신 큐 제거 후 선택 모터 Reset/fault0 응답을 최대3회 확인한다.
- 정상 완료도 STOP하므로 토크가 해제된다. 다른 축을 제어하거나 지지하지 않는다.
- 프로그램 자체가 강제 종료되거나 통신이 끊기면 소프트웨어 STOP은 전달되지 않을 수 있다.
  PP 모드 watchdog의 실제 차단 동작과 물리 지지를 별도로 확인해야 한다.

## LIVE 전 필요한 정보

config/single_axis_position.yaml을 실측·확인 근거로 채워야 한다.
현재 필수 항목은 null, pp_watchdog_verified=false이므로 LIVE는 차단된다.

| 항목 | 의미 |
|---|---|
| current_limit_a | 해당 축 PP 전류 제한 A |
| current_trip_a | 실제 측정전류 중단 기준 A |
| velocity_rad_s | 관절 속도 상한 rad/s |
| acceleration_rad_s2 | 관절 가속도 설정 rad/s² |
| travel_rad | 시작 관절각에서 허용할 최대 이동거리 rad |
| goal_tolerance_rad | 도달/유지 판정 오차 rad |
| pp_watchdog_verified | 해당 모터/펌웨어에서 PP 통신 단절 및 실제 거동 별도 확인 |

기존 전류모드0A watchdog 실험 결과를 PP 검증으로 자동 승격하지 않는다.
다른 축/펌웨어/하중에서 얻은 제한값도 그대로 사용하지 않는다. C++/Python 코드의
commissioning ceiling은 소프트웨어 입력 상한이며 실물 안전 정격이 아니다.

필요 조건이 확인된 뒤 사용할 LIVE 문법(지금은 실행하지 않음):

```text
ros2 run pallet pallet_single_axis_position.py --joint <확인한 J1~J6> \
  --delta-rad <확인한 작은 관절 변화량> --limits <확인한 설정파일> \
  --port <실제 USB 포트> --arm-supported --live --log <새 로그 파일>
```

Motor Studio, feedback.launch.py 및 다른 USB 읽기 도구는 같은 포트를 공유하지 못한다.
종료/연결 해제로 토크가 유지될 것이라 가정하지 않는다. 팔 외부 지지 후 통신 소유권을 전환한다.
LIVE 도구 자체가 피드백을 읽으며 별도 feedback launch를 함께 실행하지 않는다.
재시도는 로그와 실제 정지 상태를 확인한 뒤 한다. 목표각으로 자동 복귀하지 않는다.

## 코드와 근거

- scripts/pallet_single_axis_position.py: CLI, OFFLINE/LIVE 구분, 설정/로그/신호 처리.
- scripts/single_axis_position_core.py: 좌표 변환, PP whitelist, bounded 실행 및 정지.
- config/single_axis_position.yaml: 실측값을 채울 템플릿.
- test/test_single_axis_position.py: 순서, 전류/속도/통신 실패, 인터럽트, STOP 실패 검증.

공식 자료:
- https://github.com/RobStride/Python_Sample/blob/main/robstride_dynamics/protocol.py
- https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/CAN2.0/RS05/RS05User%20Manual260713.pdf

RS00/RS06/RS05의 개별 펌웨어 PP 운용 차이는 해당 축 실기 전에 확인한다.
본 도구는 현재 실물 패킷/부하 시험을 완료한 드라이버로 표시하지 않는다.

## 2026-10-27 검증 결과

신규 단축 위치제어 테스트10개 포함 관련 테스트38개 통과. 문법 검사와 pallet 빌드 통과.
J1 +0.01rad OFFLINE 계산 및 미검증 LIVE 설정의 장치 접근 전 거부 확인.
Run/Reset 확인 로직과 실제 모터 PP 제어는 아직 실기 검증하지 않았다.

## 실제 설정 읽기 전용 사전 점검

```bash
ros2 run pallet pallet_position_preflight.py --motor 1 --read --port /dev/ttyUSB0
```

--read를 빼면 장치 접근 없이 읽을 항목 목록만 출력한다. 읽기 요청 타입17만 허용하며
ENABLE/STOP/영점/모드/전류/목표각 쓰기 API를 제공하지 않는다. write-only PP 프로필은 읽지 않는다.
Motor Studio 또는 feedback 노드가 포트를 점유하고 있으면 먼저 물리 지지를 확보하고 통신 소유권을 전환한다.
읽기 도구도 통신 패킷을 보내므로 watchdog 통신 침묵 시험 중에는 함께 실행하지 않는다.

2026-10-27 20:40 KST J1 실제 읽기 결과:

- run_mode=0 (PP 모드1이 아님). 운전모드 설정만으로 enable/stop 상태를 단정하지 않는다.
- mechPos=-0.7743151784rad, loc_ref=-0.7742385268rad.
- limit_cur=16A. 기존 설정 관측값이며 시험용 제한값으로 승인/복사하지 않는다.
- 측정 current=0A, mechVel=0.02859598rad/s. 단일 비동기 표본이며 정지/유지 성공 판정이 아니다.
- canTimeout=0ticks(비활성). PP watchdog 검증 여부는 여전히 false.
- 원본: measurements/position/J1_preflight_20261027.json.
- 설정 쓰기/enable/stop/이동 명령 전송 없음. LIVE 설정은 변경하지 않음.

추가 검증: 읽기 전용 allowlist, 불완전 읽기, fault 실패 처리 테스트3개 통과.
실물 실행 전 J1의 운용 전류·속도·가속도와 PP 통신 침묵 시 출력 차단을 확인해야 한다.

## 2026-10-27 운용 설정 변경

사용자 지정: 전류 제한은 모터 기존 값 유지, 관절 속도0.1rad/s, 가속도0.1rad/s².
config/single_axis_position.yaml에 current_limit_policy=preserve_existing을 반영했다.
이 모드에서는 현재 전류 제한을 읽어 로그에 남기고 0x7018 쓰기를 금지한다.
관측된16A를 신규 설정으로 쓰거나, 보정용 전류 모드 코드를 변경하지 않는다.
실제 PP 프로필 전송은 별도 LIVE 실행 시에만 한다. 현재는 하드웨어 설정 변경 없음.

전류 제한 유지와 별개로 측정전류 중단값(current_trip_a)은 아직 미지정이다.
이동범위·도달오차·PP watchdog 검증도 미확정이므로 LIVE 차단은 유지한다.
OFFLINE은 속도/가속도 선택값을 반영하되 미확정 항목은 offline_fallback_fields로 표시한다.
전류 유지 정책에서 전류 제한 쓰기가 없음을 포함한 신규 위치제어 테스트11개 통과.

## 2026-10-27 J1 PP watchdog 실물 결과

사용자가 팔 외부 지지와 다른 관절 고정을 확인한 뒤 J1만 시험했다.
새 목표 자세를 명령하지 않고 현재각을 목표로 유지했다.

- 기존 전류 제한16A 유지 (전류 제한 쓰기 없음).
- 속도0.1rad/s, 가속도0.1rad/s², watchdog5000ticks=250ms 설정.
- 150ms 유지 관측 후 송신을600ms 중단하고 수신만 관찰.
- 첫 Reset 약0.281594초, 이후 Reset/fault0 유지.
- 유지 중 측정 전류 절대최대 약0.02694A.
- 시작/종료 원시각 둘 다 -0.7817549706rad. 두 끝점 차이0rad이며 중간 이동이 없었다는 증명은 아니다.
- 종료 후 측정전류0A, 명시적 STOP/Reset 확인.
- 정상 종료. 기록: measurements/position/J1_pp_watchdog_20261027.json.

이는 지지된 J1의 Run→Reset 상태 전환 관측이며, 하중에서 실제 출력토크 차단이나
다른 축의 동작을 입증한 결과가 아니다. 시험 이후 J1은 PP 모드 설정과250ms watchdog가
남아 있고, 속도·가속도0.1로 설정됐으며 STOP 상태로 종료했다. active report는 끈 상태다.

다음 작은 변위 시험을 위한 config/single_axis_position_J1.yaml을 별도로 준비했다.
시험 guard는 current trip0.2A, travel0.02rad, goal tolerance0.001rad이며,
현재 지지 조건의 commissioning 값이다. 정격·하중 검증값으로 표시하지 않는다.
이 프로필은 J1로만 scope를 제한한다. 기존 공통 설정과 다른 축 검증 상태는 유지한다.

계산만 확인하는 명령:

```bash
ros2 run pallet pallet_single_axis_position.py --joint J1 --delta-rad 0.01 \
  --limits /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/config/single_axis_position_J1.yaml
```

위 명령은 OFFLINE이다. 새로운 목표로 실제 이동시키는 시험은 이번 watchdog 시험에서 수행하지 않았다.

## 2026-10-27 J1 +0.01rad 시험: 미완료

팔 지지·다른 축 고정 확인을 유지하고 사용자 요청으로 J1만 시험했다.
전류 제한16A는 보존하고 속도/가속도0.1, 소프트웨어 current trip0.2A를 사용했다.

1. 첫 시도는 ENABLE 전 초기LOC_REF 비교에서 중단. write=-0.7818316817,
   read=-0.7817549706rad로 약0.00007671rad 차이. STOP 확인.
2. STOP 상태 현재각 기록/읽기3회에서 약0.00003839~0.00011504rad 차이를 관측.
   원본: measurements/position/J1_loc_ref_readback_probe_20261027.json.
3. 초기LOC_REF 확인에만 goal tolerance의1/5=0.0002rad 허용범위를 적용.
   0.0003rad 불일치는 거부하는 테스트를 추가. 다른 설정 검증은 기존 정밀도 유지.
4. 두 번째 시도는 ENABLE/Run 확인 후 목표를 전송했으나 Current trip으로 종료.
   start joint0.7828671336, target0.7928671336rad.
   마지막 정상 기록은0.20994초, joint0.7834423780rad, 목표오차0.00942476rad.
   목표 도달/유지 성공이 아니며 감시 기준을 올려 자동 재시도하지 않았다.
5. 두 번째 STOP 확인 후 별도 읽기에서 current0A, 기존전류제한16A,
   run_mode1, watchdog5000ticks 확인. 이것만으로 모터 상태 비트를 직접 판정하지 않는다.

첫 두 시도 로그:
- measurements/position/J1_plus001_20261027.jsonl
- measurements/position/J1_plus001_20261027_attempt2.jsonl
- measurements/position/J1_after_plus001_attempt2_20261027.json

기존 루프가 trip 샘플 자체는 기록하지 않아 정확한 peak current는 남지 않았다.
이 문제를 수정해 앞으로 limit_event에 실제 전류·속도·위치·목표오차·사유를 기록한다.
현재13개 위치제어 테스트와 빌드 통과. 목표 도달 성공으로 기록하지 않는다.

다음 판단에는 J1의 회전이 외부 지지에 막혀 있는지 확인하고, 제한값/전류 파형을
검토해야 한다. 모터 current limit16A와 소프트웨어 측정전류 trip0.2A는 다른 설정이다.

## 2026-10-27: 8A 제한을 유지한 J1 +0.01rad 시험

사용자가 이동 경로와 외부 지지 유지 상태를 확인했다. 기존 J1 전류 제한을 16A에서
8A로 반감한 상태에서 보존했다. 추가 소프트웨어 전류 trip은 명시적으로 비활성화하고
전류를 기록하도록 선택 프로파일 single_axis_position_J1_half_limit.yaml을 추가했다.
기존 0.2A 프로파일은 보존했다. 실행 전 8A 재확인 실패 시 enable을 거부한다.

실기 결과: 시작 J1 0.79387343rad, 목표 0.80387343rad. 첫 샘플 속도
0.11150091rad/s가 감시 한계 0.11rad/s를 넘어 중단됐다. 측정 전류 -0.04338085A,
각도 0.79391181rad. STOP 확인 성공. 목표 도달/위치 유지 성공으로 판정하지 않았다.
자동 재시도나 감시 한계 상향은 하지 않았다. 설정 속도·가속도는 각각 0.1이었다.
속도 레지스터의 순간 피드백인지 실제 움직임인지 이 한 샘플만으로 확정할 수 없다.

실기 로그: measurements/position/J1_plus001_half_limit_20261027.jsonl.
종료 후 읽기: measurements/position/J1_after_half_limit_speed_trip_20261027.json.
관련 단위 테스트 26개 및 pallet 빌드 통과. 소프트웨어 검증과 실기 목표 도달은 별개다.
다음은 낮춘 속도에서 속도 피드백과 각도 변화율을 함께 기록하여 원인을 구분하는 시험이다.

## J1 재시험: 속도 중단 기준0.2rad/s（2026-10-27）

사용자 지시에 따라 명령 속도/가속도 0.1은 유지하고 속도 중단 기준만 0.2rad/s로 분리했다.
J1 +0.01rad 목표 오차0.001rad 이내 0.5초 유지 및 STOP 확인 성공.
초기 오버슈트 후 수렴했으므로 전체 구간 오차가 항상0.001rad 이내였다는 뜻은 아니다.
다른 축/다축/LIVE MoveIt 실행 검증은 포함하지 않는다.
결과: {"passed": true, "stop_confirmed": true, "peak_abs_current_a": 1.0131911039352417, "peak_abs_reported_velocity_rad_s": 0.09249846637248993, "final_goal_error_rad": 0.000277688503265372, "peak_overshoot_rad": 0.0023102068901061923, "duration_s": 2.013352981000935}
로그: measurements/position/J1_plus001_speed_trip020_20261027.jsonl
관련 테스트29개 통과, 빌드 통과.

## J1 반대 방향 시험 및 공식 오차 사양 확인

J1 -0.01rad 시험 첫 시도는 enable 전 LOC_REF 차이0.000230rad로 중단, STOP 확인.
추가 읽기 전용 20회에서 전류0A, 위치 span0.000192rad, reference span0.000230rad 확인.
초기 reference 확인만 goal_tolerance의 절반(0.0005rad)으로 변경하고 테스트29개 통과.
이는 공식 정확도 사양이 아닌 소프트웨어 판정값이다. 도달 판정0.001rad는 유지했다.
재시험에서 -0.01rad 도달·유지 및 STOP 성공. 취소 실기 시험은 아직 수행하지 않았다.

사용자가 공식 모터 허용 오차로 교체 요청하여 RobStride 공식 RS 시리즈 사양
2026.010.09 PDF와 RS00 User Manual260713을 확인했다. 해당 자료에서 출력축 위치
정확도/반복정밀도 보장값을 찾지 못했다. 매뉴얼의 encoder resolution14bit는 분해능으로,
허용 위치 오차와 동일하지 않으므로 이를 근거로 설정을 교체하지 않았다.
공식 사양 출처: https://github.com/RobStride/Product_Information
최신 사양 PDF: 灵足时代RS系列产品规格介绍(2026.010.09).pdf
{"passed": true, "stop_confirmed": true, "final_error_rad": 0.0002209043502807706, "peak_current_a": 1.1470438241958618, "peak_velocity_rad_s": 0.09264528006315231}

## 2026-10-27: 6축 동시 +0.01rad 시험

사용자가 모든 축의 이동 경로와 외부 지지를 확인하고 동시 시험을 요청했다.
pallet_multi_axis_position.py 추가: 기본 offline, LIVE는 명시 플래그/새 로그 필요.
기존 생산 브리지 commissioning gate를 변경하거나 모든 축을 검증 완료로 표시하지 않았다.
별도 제한된 commissioning 도구에서 각 축 +0.01rad를 1초 램프로 보내고,
목표 오차0.001rad 이내0.5초 유지를 확인한다. 명령 속도/가속도0.1, 속도trip0.2,
현재점 대비 이동범위0.02rad, 최대4초. 전류 제한은8/28.5/8/5.5/5.5/5.5A 재확인 후 보존.
추가 소프트웨어 전류trip은 비활성화하고 기록한다. 한 축 오류 시 전체 STOP 확인.
prepare 단계는 전 축 정지 설정과 초기 목표각 확인을 먼저 완료하고, 순차 enable 중
이미 켜진 축의 현재 위치 명령을 갱신한다. enable은 순차 CAN이므로 물리적 동시 시각은 아니다.

실기 결과: 6축 활성화 및 이동 후 약1.8초에 J2 속도 피드백0.2376468rad/s로 중단.
전체 STOP 확인. 중단 직전 모든 축 오차0.001rad 이내였지만0.5초 유지는 미완료.
J2 속도값과 위치차분의 관계는 미확정이며 속도 기준을 추가 상향하지 않았다.
관련 테스트33개 및 빌드 통과. 로그/요약은 아래와 같다.
{"passed": false, "all_stop_confirmed": true, "reason": "J2: velocity trip", "max_cycle_ms": 77.25035000112257, "last_errors_rad": {"J1": 0.0004893040657043546, "J2": 0.0009665850573219361, "J3": 0.00027767732739448353, "J4": 4.509627819061779e-05, "J5": 5.386978387832836e-05, "J6": 4.505157470703104e-05}, "last_elapsed_s": 1.7782352669983084, "log": "measurements/position/all_axes_plus001_20261027.jsonl"}

## J2 속도 피드백 교차 감시 수정: 2026-10-27

공식 RS06 CAN2.0 매뉴얼260713에서0x701B는 load speed rad/s임을 재확인했다.
단위/감속비를 임의로 바꾸지 않았다. 정지 읽기200회에서 속도는 모두0, 위치span0.000298rad,
전류0A였다. 기존 단발0.2376468rad/s의 펌웨어 내부 원인은 아직 확정하지 못했다.

VelocityMonitor 추가. 기본값은 기존 즉시 중단1샘플이다. 제한된6축 commissioning 도구만
2연속 초과 확인을 사용한다. 0.2rad/s 이상2연속 raw sample(현재 주기100ms)이면 중단,
위치차분0.2 이상 또는 raw0.4 이상은 즉시 중단한다. raw값을 버리거나 고쳐 쓰지 않고
축별 위치 읽기 monotonic timestamp, 위치차분 속도, 초과연속횟수를 모두 기록한다.
따라서 순간 속도가 항상0.2 미만임을 보장하는 방식은 아니다. 단발값 대응 정책 변경이다.
생산 LIVE 설정/commissioning 통과 표시를 임의로 해제하지 않았다.

관련 테스트34개 통과. 연속 초과/위치 기반 초과/큰raw즉시중단/기존1샘플정책 보존 검증.
실물 -0.01rad와+0.01rad 모두6축 목표오차0.001rad 이내0.5초 유지 및전체 STOP통과.
정방향에서 J2 raw -0.230842rad/s가1회 재현됐다. 같은 구간 위치차분은+0.001701rad/s,
다음 raw샘플은 정상으로 복귀했다. 교차감시는 단발 불일치를 기록하고 시험을 완료했다.
펌웨어 내부 원인 및 실제 순간 최고속도는 확정하지 않으며 후속 실행에서도 기록한다.
공식 출처: https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/CAN2.0/RS06/RS06User%20Manual260713.pdf
[{"direction": "minus", "goal_hold_passed": true, "all_stop_confirmed": true, "max_cycle_ms": 77.3526730008598, "duration_s": 2.27946035199966, "max_final_error_rad": 0.00035075306892395214, "raw_suspect_events": [], "J2_peak_raw_rad_s": 0.0, "J2_peak_position_derived_rad_s": 0.014059988490766697}, {"direction": "plus", "goal_hold_passed": true, "all_stop_confirmed": true, "max_cycle_ms": 77.72770499832404, "duration_s": 2.3800039580000885, "max_final_error_rad": 0.0003700081631541254, "raw_suspect_events": [{"joint": "J2", "position_rad": 0.012612713500857353, "current_a": -2.854123115539551, "velocity_rad_s": -0.2308422029018402, "position_sample_time_s": 8839.257235938, "position_velocity_rad_s": 0.0017010922395767586, "raw_overspeed_count": 1, "raw_speed_suspect": true, "velocity_confirmation_samples": 2}], "J2_peak_raw_rad_s": 0.2308422029018402, "J2_peak_position_derived_rad_s": 0.01070483738697468}]

## J5 고정 최종목표 vs 10Hz ramp 비교

2026-10-27 사용자 승인으로실기비교. 기존LIVE브리지PID51500을SIGINT로정상종료해
포트충돌을 방지했다.팔6축을동일초기위치로유지하고 J5만+0.01rad, ID7은제어하지않았다.
모터PP10/10과현재전류초기값복원설정을보존했다.

pallet_j5_response_compare.py 추가. 기본OFFLINE. fixed는trial내 최종목표를모터당
정확히1회전송, 이후read만으로watchdog통신을유지한다. stream은같은목표까지1초동안
100ms마다중간각도를전송한다. 두trial사이토크유지상태로원래각도복귀/0.3초확인.
각trial2.5초관찰, 최종오차0.001rad이내0.5초유지, 위치변화0.03rad이탈/주기초과시중단.
모든경로에서6축STOP확인. 기본반감전류제한프로파일로되돌리지않았다.

결과: fixed유지조건0.9741초, 최종오차0.0000944rad. stream유지조건1.6777초,
최종오차0.0000449rad, 최대동적오차0.001936rad. 둘다통과, 전체STOP확인.
fixed의step오차는첫순간목표가바뀌므로ramp의경로오차와단순동등비교하면안된다.
이시험은작은이동에서두방식이작동함을입증했으나기존더빠른이동의오차원인을확정하지않는다.
고속전범위추종이나제어게인최적화검증은아니다.

관련테스트53개/pallet빌드통과.로그: measurements/position/J5_fixed_vs_stream_20261027.jsonl,
요약:같은이름_summary.json.실행중지한MoveIt브리지는자동재시작하지않았다.
다음은같은이동량에시간조건만바꿔추종오차의속도의존성을비교하는것이다.
