# 팔 한 축 전류 펄스 시험

최신 실기 결과는 COMMISSIONING_RESULT_20261022.md에 기록했다. 기본 실행은 장치를 열지 않는 오프라인 파형 확인이며 전체 중력보상이 아니다.

```bash
python3 /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_single_axis_current_test.py --motor 6 --sign 1
```

기본 한계는 명령전류0.1A, 상승/하강0.1A/s, 2초, 이동0.035rad, 속도0.15rad/s, 측정Iq 차단0.2A다. 기본 목표주기는20ms지만 현 USB/Python 환경에서 주기 초과를 관측하여 최근 시험은 --period-s 0.05를 명시했다. 실시간 실행 보장은 없다. 소프트웨어 기본값은 장비의 검증된 안전 정격을 뜻하지 않는다.

## 실행 조건

팔 전체 외부 지지, 비시험 축 고정, 작은 움직임에도 낙하/끼임이 없는 배치와 물리 정지 수단이 필요하다. STOP은 중력 지지 토크도 없앤다. Motor Studio와 USB를 쓰는 RViz bridge를 종료한다. 사용 장치는 /dev/ttyUSB0, 921600baud RobStride AT 직렬 어댑터다.

--watchdog-tested는 검증 사실을 표현하는 옵션이며 검증을 대신하지 않는다. 현재 IDs1..7에서0A Run→Reset은 확인했지만 비영점 부하의 전류 차단 시간을 측정한 것은 아니다. 통신 단절 시 소프트웨어 STOP 전달은 보장되지 않는다.

## 명령 예시

아래 예시는 지지 조건과 해당 축의 시험 범위를 확인한 경우에 사용한다. ID4는0.1A에서 속도 한계 초과를 관측했으므로 이 예시를 그대로 적용하지 않는다.

```bash
python3 /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_single_axis_current_test.py \
  --motor 6 --sign 1 --current-a 0.1 --duration-s 2 --period-s 0.05 \
  --live --arm-supported --watchdog-tested
```

--sign은 raw 모터 Iq 부호이며 URDF 관절 방향과 구분한다. 그리퍼 ID7은 이 전류 도구에서 제외한다. 작은 움직임이나 무동작을 이유로 전류/속도 한계를 바로 높이지 않는다.

## 데이터 흐름과 실패 처리

선택 ID에 STOP → 위치 → watchdog 0x7028=5000(uint32,250ms) 쓰기/읽기 확인 → current mode3/0A 각각 쓰기/읽기 확인 → 위치 재확인 → ENABLE → 삼각 전류 펄스 순서다. 모든 팔 축에서 실제 응답한0x7028을 사용한다. 영점 설정·영구 저장·다른 축 활성화는 하지 않는다.

설정 읽기는10ms 간격으로 최대3회만 재시도한다. 실기 루프의 피드백 누락, 고장, 이동/속도/측정전류 제한 초과, 주기 초과는 중단한다. 피드백은 mechPos(0x7019), iqf(0x701A)이며 거부된 파라미터 응답은 성공으로 취급하지 않는다. 직렬 송신은 최소3ms 간격을 둔다.

정상종료·Ctrl+C·SIGTERM·예외 시0A와STOP을 독립적으로 시도한다. STOP 확인 전 오래된 수신 큐를 버리고, 해당 ID의 새8바이트 Reset/fault0 응답을 요구한다. STOP은 최대3회 한정 재시도하며 확인 실패는 오류로 남긴다. 자동 재활성화하지 않는다. watchdog을0으로 되돌리지 않는다.

JSONL 로그는 시간, 모터각, 변위, 속도, 명령Iq, 측정Iq를 기록한다. 속도 한계 발생 표본과 종료 오류도 기록한다. 명령전류 제한은 하드웨어 전류 제한과 다르다.

## 0A watchdog 시험

```bash
python3 /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_watchdog_test.py --motor 6
# 외부 지지와 실행 조건을 갖춘 경우만:
python3 /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_watchdog_test.py --motor 6 --live --arm-supported
```

이 도구만 ID7까지 허용한다. 자동 상태보고를 켜고 Run을 확인한 뒤600ms 동안 송신 없이 수신만 한다.450ms 이내Reset 진입 및 관측 종료에도Reset이어야 통과한다. 모터는 펌웨어에 따라type2 또는type24 상태보고를 보낼 수 있다. 전류는0A만 사용한다.

## 검증

관련 전류·watchdog·중력계산·보정·preview 테스트32개 통과. 실기 결과와 제한 초과는 별도 결과 문서에 보존했다. 소프트웨어 테스트 통과를 무지지 동작, 토크 보정 또는 전체 중력보상 성공으로 해석하지 않는다.
