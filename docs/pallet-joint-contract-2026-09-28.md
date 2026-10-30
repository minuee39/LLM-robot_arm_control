# Pallet 관절·실기 실행 계약 점검

> 2026-09-28 당시 회전식 그리퍼의 정적 점검 기록이다. 현재 67 mm prismatic 계약과 시뮬레이션 검증 결과는 [후속 작업 기록](pallet-simulation-handoff-2026-09-29.md)을 기준으로 한다.

## 작성일과 작업 목적

- 작성일: 2026-09-28
- 목적: Pallet 직접 좌표 Pick & Place에 사용하는 모델의 관절 계약을 고정하고, 실기 CAN 설정과의 차이를 드러낸다.

## 최종 기준 workspace

```bash
cd /home/minwoo/Development/LLM
```

- 현재 브랜치는 `pallet_robot_ver`이며, 기존 혼합 변경을 보존했다.
- 이 점검은 설정과 코드의 정적 대조다. 실제 모터, 그리퍼, Isaac 물리 동작을 실행하지 않았다.

## 현재 주요 패키지와 파일

- `ros2_ur_ws/src/pallet_gripper/urdf/pallet_gripper.urdf`: 현재 Pallet 그리퍼 모델. J1~J6이 팔, J8이 구동 그리퍼, J9는 J8의 -1배 mimic이다.
- `ros2_ur_ws/src/pallet_gripper_moveit_config/config/`: 현재 모델의 SRDF, ros2_control, MoveIt controller, 관절 한계, 초기 자세.
- `ros2_ur_ws/src/mtc_tutorial/config/robot_profiles.yaml`: `pallet_gripper` MTC 계획 계약.
- `ros2_ur_ws/src/pallet/config/robstride_hardware.yaml`: 별도 5축 `pallet` 실기 브리지 설정. `j1~j5`, CAN ID 1~5, `commissioned: false`.
- `ros2_ur_ws/scripts/audit_pallet_joint_contract.py`: 설정 간 관절 계약을 점검하는 읽기 전용 검사기.

## 문제 → 원인 → 해결 → 검증

1. **모델과 실기 관절 수·이름 차이:** 현재 Pick & Place 모델은 `J1~J6 + J8`이지만 기존 CAN 브리지는 `j1~j5`만 받는다. 서로 다른 로봇 모델/MoveIt 설정이 병존하는 것이 원인이다. 현재 모델의 관절 목록을 기준으로 명시하고 자동 검사기를 추가했다. 검사 결과 시뮬레이션 설정은 일치하며, 실기 설정 차이는 `HARDWARE GAP`으로 출력된다. 기존 5축 브리지를 6축 모델에 연결했다고 간주하지 않는다.
2. **모터 배정 미확정:** `robstride_hardware.yaml`은 CAN ID 1~5를 정의하지만 J1~J6/J8의 실측 대응표가 없다. `robstride_motors.yaml`의 물리 j1~j6 설명도 5축 설정과 다르다. 미확인 ID·방향·영점·감속비를 임의로 채우지 않고 아래 확인 항목으로 남겼다. `commissioned: false`를 유지했다.
3. **그리퍼 설명의 오래된 수치:** 현재 URDF와 MoveIt의 J8 상한은 0.103 rad이다. 과거 README에는 다른 임시 개폐 각도 설명이 남아 있다. 이 작업의 기준을 실제 실행 설정으로 정했으며, 손가락 이동량·접촉력은 실측 전 미확정이다. 검사기는 URDF와 MoveIt의 J8 한계가 일치하는지 확인한다.

## 최종 구조와 설계 결정

| 실행 대상 | 팔 관절 | 그리퍼 | 현재 상태 |
| --- | --- | --- | --- |
| Pallet 직접 좌표 MTC / Isaac | `J1`~`J6` | 명령 `J8`, 종속 `J9` | 모델·계획 설정 일치. Pick & Place 물리 성공은 아직 미검증 |
| 기존 RobStride MoveIt 브리지 | `j1`~`j5` | 없음 | 별도 5축 모델용. 실제 구동 잠김 |

실기 대응표의 빈칸은 의도적으로 미확정 상태로 둔다. 아래 관절축은 각 관절의 **URDF 로컬 좌표계** 기준이며, 모터의 실제 양의 회전 방향과 같다는 뜻이 아니다.

| 현재 모델 관절 | 역할·URDF 축 | 실기 CAN ID | 방향·영점·전달비 |
| --- | --- | --- | --- |
| `J1` | 팔, `0 0 1` | 미확인 | 미실측 |
| `J2` | 팔, `0 -1 0` | 미확인 | 미실측 |
| `J3` | 팔, `1 0 0` | 미확인 | 미실측 |
| `J4` | 팔, `0 -1 0` | 미확인 | 미실측 |
| `J5` | 팔, `1 0 0` | 미확인 | 미실측 |
| `J6` | 팔, `0 0 1` | 미확인 | 미실측 |
| `J8` | 그리퍼 구동, `0 1 0` | 미확인 | 미실측 |
| `J9` | `J8`의 -1배 mimic | 독립 구동 없음(모델 기준) | 실제 기구 연동 확인 필요 |

- 현 시뮬레이션 모델의 팔은 6축이다. [원본 CAD 표시 패키지](../ros2_ur_ws/src/pallet_gripper_description/README.md)의 과거 `g1` 미확정 설명과 현재 실행 모델을 혼동하지 않는다.
- 좌표는 `base_footprint` 계획 프레임을 쓰고, MTC 손 프레임은 `tcp`다. 관절 위치는 rad, 작업 셀 길이는 m다.
- 실기 설정의 `direction: 1`, `motor_zero_rad: 0.0`, `transmission_ratio: 1.0`은 실측 보정 완료를 의미하지 않는다. CAN ID 6·7의 용도도 장비 배선과 읽기 결과로 확인한다.
- 실제 모터 enable은 관절 대응, 기계적 한계, 지지·비상 정지 수단을 확인한 뒤 별도 단계에서 수행한다.

## 실행 순서

터미널 1에서 장비를 구동하지 않는 정적 점검을 수행한다.

```bash
cd /home/minwoo/Development/LLM
python3 ros2_ur_ws/scripts/audit_pallet_joint_contract.py
```

다음 실측 단계에서 팔을 지지하고 전원·CAN·비상 정지 상태를 확인한 뒤, 먼저 모터 위치만 읽는다. 이 명령은 실제 연결 환경에서만 실행한다.

```bash
cd /home/minwoo/Development/LLM
python3 robstride_read_angles.py --motor 1 2 3 4 5 6 7 --once
```

각 모터에 대해 물리 관절, 모델 관절, 양의 회전 방향, 0 rad 기준, 출력축/모터축 비율, 관절 상·하한을 기록한다. 그 결과로 6축 팔과 J8 그리퍼에 대한 별도의 실행 설정과 컨트롤러를 구성한다. 기존 5축 브리지 설정을 이름만 바꿔 재사용하지 않는다.

## 검증 명령과 결과

- `python3 ros2_ur_ws/scripts/audit_pallet_joint_contract.py`: 통과. URDF, SRDF, ros2_control, MoveIt controller, 초기 자세, 관절 한계, MTC 프로파일 일치. 5축 CAN 설정 차이 출력.
- `python3 -m py_compile ros2_ur_ws/scripts/audit_pallet_joint_contract.py`: 통과.
- Pallet 단위 테스트 11개, MTC 프로파일 테스트 5개: 통과.
- `ip -details link show can0`: `Device "can0" does not exist.` 따라서 이번 단계에서 모터 응답과 ID를 실측하지 못했다.
- ROS 패키지 빌드와 장비 구동은 이번 정적 점검에서 수행하지 않았다.

## 남은 주의사항

- 정적 검사 통과는 실제 기계의 축·방향·영점·전달비가 맞다는 증거가 아니다.
- 현재 시스템에서 `can0`가 보이지 않는다. 장비 연결과 CAN 인터페이스 준비 후 읽기 전용 점검을 다시 수행해야 한다.
- 현재 `pallet_gripper.ros2_control.xacro`는 `mock_components/GenericSystem`을 사용하므로 실기 하드웨어 인터페이스로 사용할 수 없다.
- Pallet 직접 좌표 MTC의 Isaac 물리 계획·실행 실패는 관절 계약 정리와 별개로 다음 단계에서 진단해야 한다.

## 다음 작업 로드맵

- [x] 현재 Pick & Place 모델의 관절·컨트롤러 계약과 기존 5축 실기 설정 차이 명시
- [x] 설정 일치 자동 검사 추가
- [ ] 실기 각 축의 모터 ID·방향·영점·비율·관절 한계 실측
- [ ] J1~J6/J8용 실기 controller 및 안전 설정 구현·검증
- [ ] Isaac 초기 관절 피드백과 모의 컨트롤러 자세 비교
- [ ] Isaac 직접 좌표 MTC 전체 계획 및 Pick & Place 물리 실행 검증
