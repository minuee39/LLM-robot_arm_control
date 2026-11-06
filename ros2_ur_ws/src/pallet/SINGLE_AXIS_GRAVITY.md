# J5 직렬 단일축 부분 중력보상

현재 명령 경로는 URDF/엔코더 → J5 중력토크 → 부분 보상+속도 감쇠 → 명목 Nm/Apk 변환 → 전류 상한/변화율 제한 → ID5다. 다른 축과 그리퍼에는 구동 명령을 보내지 않는다. 다른 축의 각도는 매 주기 읽어 움직임을 감시한다. 전체 팔의 `commissioned`를 완료로 바꾸지 않는다.

## 실행

기본값은 장치 접근 없는 계산이다.

```bash
python3 /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_single_axis_gravity.py
# 실제 각도 읽기만:
python3 /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_single_axis_gravity.py --read-only
```

실기는 팔 외부 지지, 양방향 회전 여유, 다른 관절 고정 및 빈 그리퍼 조건이 필요하다. `--assume-gripper-zero`는 미측정 그리퍼를 URDF g_r=0으로 근사한다는 뜻이며 홈 보정 성공을 뜻하지 않는다. 다른 USB 클라이언트를 종료한다.

```bash
python3 /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_single_axis_gravity.py \
  --live --arm-supported --assume-gripper-zero \
  --gain 0.1 --duration-s 1 --log /tmp/j5_gravity_trial_new.jsonl
```

기본 전류 상한0.03A, 상승/하강0.06A/s, 측정전류 차단0.1A, 시작 대비 이동0.035rad, 속도0.15rad/s, 다른 축 raw 이동0.01rad, 주기50ms, 읽기대기25ms, watchdog250ms, 중력보상 ramp0.5초, 감쇠0.02Nm/(rad/s)다. 이는 제한된 연구용 초기 설정이며 정격/안전 인증값이 아니다. 중력보상 이득은 최대0.2로 제한한다. 시간 지정 모드는 최대2초이며, --continuous는 시간 종료만 해제한다. 측정전류/속도/이동/주기/고장 한계를 넘으면 중단하며 실패 원인과 표본을 기록한다. 전류 포화는 로그에 표시하며 완전 보상으로 해석하지 않는다.

모터측 watchdog 설정·0A·mode를 읽어 검증한 후 활성화한다. 종료 시0A 쓰기와 STOP 확인을 독립 시도한다. 마지막으로 iq_ref와 iqf를 읽어0A를 확인한다. 자동 원위치 복귀는 하지 않는다. 로그 경로가 존재하면 덮어쓰지 않는다. Python·USB 전송 지연 및 정지 후 운동 때문에 소프트웨어 이동 한계가 정확한 물리 정지 상한을 보장하지 않는다.

## 좌표와 환산 근거

J5는 기존 방향−1, ratio1, zero0을 사용하고 URDF 위치 제한을 검사한다. 고정된 나머지 회전축은 중력 계산에서만 2π 동등각으로 표현한다. 모터 영점, raw 엔코더 및 케이블 회전 수를 수정하지 않으며 다른 축이 움직이면 중단한다. 따라서 이 처리를 전체 팔 궤적 제어의 회전 수 보정으로 재사용하면 안 된다.

RS05 CAN2.0 User Manual260713 전기특성의0.94Nm/Arms와 정격2.4Apk/1.6Nm를 근거로, 초기 명목 환산은0.94/√2≈0.66468Nm/Apk를 사용한다. 실제 Iq와 출력축 토크의 독립 실측 보정값은 아니며 전달 효율·마찰·제품 편차가 포함되지 않는다. 링크 질량·COM·빈 그리퍼 자세 역시 URDF 가정이다.

공식 문서: https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/CAN2.0/RS05/RS05User%20Manual260713.pdf

**이전 토크 피드백 정정:** 제조사 Python_Sample은 RS05에±17Nm 테이블을 제공하지만 위 모델별 매뉴얼 type2는±5.5Nm 및u16[0,65535]를 명시한다. 현재 코드와 공통 실기 helper는 `(raw/65535*2-1)*5.5`로 수정했다. 예전 JSONL은 삭제/변조하지 않는다. 이전 보고토크와 회귀값을 실측 상수로 사용하면 안 된다. raw u16이 있으면 새 규칙으로 재계산한다.

## 2026-10-23 최초 실기 결과

현재자세 URDF J5토크 약0.013~0.015Nm에10% 중력보상+약한 감쇠를 적용해1초20표본 완료했다. 명령전류−0.002305~−0.000219A, 최대절대속도0.004973rad/s, 변위−0.03969~+0.00567도, 측정Iq 절대최대0.01560A. 전류포화0회. 종료새Reset/fault0, iq_ref=iqf=0 확인.

로그: /home/minwoo/Development/firmware_tools/id5_before_0_5_0_14/gravity_partial_20261023_01.jsonl
요약: /home/minwoo/Development/firmware_tools/id5_before_0_5_0_14/gravity_partial_20261023_01.json

이전0A1초 관측+0.235도와 비교해 이동은 작았지만 동일 시작각/마찰/외부 지지의 대조시험은 아니므로 중력보상 개선 효과를 확정하지 않는다. 현재 아래로 매달린 자세는 중력부하가 작다. 다음은 동일 시작자세의0A와 부분보상 대조실험을 설계하고, 그 후 지지된 기울어진 자세에서 확대한다. 전체6축/CTM/VSD는 아직미실험이다.

## 연속 ON 모드

`--continuous`를 지정하면 시간에 의한 종료 없이Ctrl+C 또는 보호조건 발생까지5번만 보상한다. 기본 실행은 계속오프라인이며 --continuous에는 --live가필요하다. --duration-s와동시에사용할수없다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/minwoo/Development/LLM/ros2_ur_ws/install/setup.bash

python3 /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_single_axis_gravity.py \
  --live --arm-supported --assume-gripper-zero \
  --gain 0.1 --continuous
```

Ctrl+C/SIGTERM시0A·STOP·후속읽기확인후연결을닫는다. 보호조건이나통신오류도중단하며자동재시작하지않는다. **시작점대비약2도이동제한은그대로이므로, 시간제한이없어도그범위를넘으면종료한다.** 연속모드는장시간실물시험으로검증되지않았으며이번수정중에는실제장치를활성화하지않았다. 시간초과1초를넘어동작하는모의시험, interrupt종료, 통신오류종료및CLI옵션검증을추가했다. 로그는제어주기마다파일에누적되므로무인방치용이아니다.
