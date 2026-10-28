# Pallet 실기 전환 준비

작성일: 2026-10-06. 목적: 검증된 Isaac pick/place를 실기로 옮길 때 필요한 모델 및 인터페이스 계약을 식별한다.

기준 workspace: `/home/minwoo/Development/LLM`.

## 주요 파일과 구조

- 시뮬레이터: `ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/isaac_sim.py`
- 실기 설정: `ros2_ur_ws/src/pallet/config/robstride_hardware.yaml`
- 실기 실행: `ros2_ur_ws/src/pallet/scripts/pallet_robstride_trajectory_bridge.py`
- 순수 궤적 검증: `ros2_ur_ws/src/pallet/scripts/robstride_trajectory_core.py`
- 작업 실행·저널: `ur10e/mtc_command_runner.py`, `ur10e/task_state.py`

## 문제 → 원인 → 해결 → 검증

현재 시뮬레이션은 J1~J6 arm과 선형 J8/J9 jaw를 사용한다. 실기 설정은 j1~j5 다섯 관절과 CAN ID 1~5만 정의하며 ID 6/7은 그리퍼용 예약이다. 이름 변경만으로 모델을 연결할 수 없다. 관절 수·축·영점·감속비·TCP·jaw 구동 방식의 측정 및 모델 통합이 선행돼야 한다.

실기 설정의 `commissioned: false`는 유지한다. 현재 direction=1, zero=0, ratio=1은 실측 보정 완료의 증거가 아니다. Isaac의 60 N/finger 또는 0.002 m 닫힘 목표도 실물 전류/토크로 직접 변환하지 않는다.

소프트웨어 게이트: `robstride_trajectory_core.py`는 CAN 버스 생성 전에 motor ID/host ID의 정수 범위, 방향 값, 영점의 유한성을 검사한다. 숫자형 YAML을 임의로 정수로 잘라 쓰거나 `NaN` 영점을 모터 명령에 전달하지 않는다. 모터 위치 응답과 관절↔모터 변환 결과가 유한하지 않으면 실행 전에 거부하거나 운전 중 정지 경로로 전달한다. 자동 검사는 하드웨어 설정 j1~j5의 위치 범위가 URDF와, 속도·가속도 범위가 MoveIt 설정과 일치함도 확인한다. 순수 단위 테스트 13개가 통과했다. 이는 실측 보정이나 실기 구동 검증을 대신하지 않는다.

## 실행 순서와 검증

먼저 CAN 송신을 하지 않는 순수 소프트웨어 검증을 실행한다.

```bash
cd /home/minwoo/Development/LLM
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s ros2_ur_ws/src/pallet/test -p 'test_robstride_*.py'
```

그 다음 측정된 실기 모델에서 planning-only와 dry-run을 확인한다. 실제 enable 또는 모터 이동은 이 문서의 소프트웨어 검증 범위에 포함되지 않는다.

## 전환 게이트

- [ ] 실물 arm DOF와 시뮬레이션 DOF의 대응 확정
- [ ] 각 관절 ID, 방향, 기계 영점, 감속비와 위치 한계 측정
- [ ] payload 0.5 kg, TCP, 무게중심 측정 및 모델 반영
- [ ] 그리퍼 회전/선형 변환, 개구 폭, 힘/전류 보정
- [ ] base↔table↔camera↔TCP 변환 측정
- [ ] E-stop, 통신 단절, tracking error 시 정지 및 낙하 방지 확인
- [ ] planning-only → 무부하 저속 → 100 g → 300 g → 500 g 단계별 승인
- [ ] 실기 센서로 slip, release/recontact 및 최종 배치 오차 검증

실기 전환 준비는 진행 중이다. 모델 매핑과 측정이 완료되지 않아 실기 pick/place 실행 가능 판정은 하지 않았다.
