# ID1~7 현재 위치 토크 유지

2026-10-27. 사용자 요청으로 작성. 이번 작업은 코드/단위검증만 수행했으며 실제7축 토크를
활성화하지 않았다. 기존6축 실기 검증을7축 유지 검증으로 해석하지 않는다.

`pallet_hold_all_motors.py`는 실제 시작각을 읽어 PP 위치제어 목표로 유지한다.
직접 토크값을 지정하는 전류제어나 중력보상은 아니다. ID7은 raw motor rad를 사용하며
URDF mimic finger각으로 변환하지 않는다. ID7의 기존 영점/개구폭 보정을 바꾸지 않는다.
그리퍼는 측정된 raw각 -12~12rad 범위 내에서만 이 도구를 사용할 수 있다. 이 범위는
기계적 정격이 아니라 기존 개구폭 보정의 측정 범위다.

## 실행

Motor Studio, 읽기 스크립트, feedback launch, LIVE MoveIt 브리지를 종료해 USB포트를
비운다. 이 도구와 MoveIt은 동시에 모터를 제어하지 않는다. 외부 지지를 유지한다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 기본 OFFLINE: 포트를 열지 않고 동작 설명만 출력
ros2 run pallet pallet_hold_all_motors.py
# LIVE: 현재7개 각도를 읽고 위치 유지. Ctrl+C까지 계속 유지한다.
ros2 run pallet pallet_hold_all_motors.py --live --arm-supported
# 시간 제한 시험은 --duration 5 등을 추가한다.
```

종료 시7축 STOP을 전송하고 확인한다. 토크가 해제될 수 있다. 성공 로그는
`stop_confirmed_all_7`이다. 확인 실패는 오류로 표시한다. 모터 토크를 유지하려면
프로세스를 계속 실행해야 하며, 오류/통신 끊김/watchdog로도 해제될 수 있다.
처짐/외력에 대한 위치 보정 동작은 발생할 수 있다.

## 설정과 기록

- 기존 전류 제한 보존. 팔은8/28.5/8/5.5/5.5/5.5A와 일치 확인, ID7은 기존 양수값 확인.
- PP speed0.1rad/s, acceleration0.1rad/s², watchdog250ms,100ms 갱신.
- raw속도0.2rad/s2연속/0.4즉시, 위치차분0.2즉시 중단.
- 시작점 대비0.02rad 이동 및 현재 위치 목표 오차0.01rad 이탈 시 중단.
  이 제한은 위치 유지 전용이며 사용자가 해제한 MoveIt의 상대 이동 제한과 별개다.
- 기본 로그: 현재 디렉터리 measurements/torque_hold/hold_all_날짜시간.jsonl.
- --log로 새 출력 경로 지정 가능. 기존 파일 덮어쓰기 거부.
- mode/watchdog/PP profile은 변경하지만 모터 영점/전류 제한은 변경하지 않는다.

소프트웨어 검증: 현재값을0으로 치환하지 않는 것,7축 targets 보존, drift/통신/interrupt시
STOP, STOP실패를 성공으로 표시하지 않는 것, ID7 raw좌표를 단위테스트했다.
실제7축 동시 유지와 추가 통신 부하 검증은 남아 있다.

## 이후 적용한 모터 기본값 복원

2026-10-27 사용자 후속 지시로 팔 전류 제한은16/57/16/11/11/11A로 복원했고 ID7은
관측된0.8A를 유지했다. LIVE프로파일 motor_profile_policy=preserve_existing이므로
유지 코드도 motor PP10rad/s/10rad/s²를0.1로 다시 덮어쓰지 않는다.
위의 초기 전류/PP0.1 설정 설명보다 이 후속 복원 상태가 우선한다.
속도 감시와 위치 이탈 제한은 그대로다. 토크 유지 코드 기본OFFLINE 출력에서 최신 정책과
전류 기대값을 확인할 수 있다.
