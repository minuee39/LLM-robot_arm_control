# 6축 팔 + 1축 그리퍼 손 유도 제어

현재 전체 중력보상 `--live`는 차단되어 있다. 현 직렬 어댑터 지원, 세션별 watchdog 설정/검증, 실측 토크 보정이 미완료다. 최신 완료 범위와 남은 조건은 `COMMISSIONING_RESULT_20261022.md`를 참고한다. 아래 실기 루프 설명은 기존 SocketCAN 구현 설명이며 현재 실행 가능 상태를 뜻하지 않는다.

실물 ID/model: 1 RS00, 2 RS06, 3 RS00, 4..7 RS05. URDF는 pallet_with_gripper.urdf (J1..J6, g_r; g_l은 mimic).

제어식: tau = dU/dq - damping*dq. 위치 강성은 사용하지 않는다. q = direction*(motor_position-zero)/ratio. Iq = direction*tau/(ratio*output_nm_per_a). output_nm_per_a는 내부 감속기 이후 출력축 토크/Iq이며 외부 기구 전달비 ratio와 구분한다. RMS 전류 상수와 Iq 상수를 혼용하지 않는다.

기본 실행은 오프라인이며 CAN에 접근하지 않는다:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet
python3 scripts/pallet_hand_guiding.py
python3 scripts/pallet_hand_guiding.py --positions 0 0.2 -0.3 0 0 0 0.05
python3 -m pytest -q test/test_hand_guiding_core.py
```

출력은 URDF 가정에 따른 계산값이며 실측 검증된 토크가 아니다. gravity_root_m_s2는 URDF root인 base_footprint 기준이다. 실제 설치 방향, 링크 질량/COM, 추가 payload를 확인해야 한다.

## 실기 준비

hand_guiding.yaml의 null을 측정값으로 채운다. 영점, 방향, 외부 전달비, 전류 환산, 감쇠, 전류/토크/속도/위치 한계가 필요하다. 실제 관절 범위에 맞게 lower_rad/upper_rad를 입력한다. 다회전 그리퍼는 매 세션 원점 복귀를 수행하고 gripper_homed를 확인한다. 현재 g_r는 URDF 손가락 회전 좌표이므로 모터의 여러 바퀴를 그대로 넣지 않는다. ratio가 일정하지 않은 기구는 현재 선형 변환을 사용하지 말고 실제 기구학/Jacobian을 추가해야 한다. 물체 파지 유지 제어는 제공하지 않는다.

통신 단절/프로세스 종료 시 전류가 지속되지 않도록 실제 펌웨어의 motor-side watchdog을 구성하고 검증한 뒤 motor_watchdog_verified를 설정한다. 이 코드는 미확인 watchdog register를 임의로 쓰지 않는다. commissioned는 이 모든 확인 후에만 true로 바꾼다. 현재 --live는 명시적으로 차단한다. 다른 Motor Studio/trajectory controller는 동시에 실행하지 않는다.

실기 루프는 current mode(3), iq_ref(0x7006), mechPos(0x7019)를 사용한다. 활성화 전 mode와 zero current를 읽어 확인한다. 2초 전류 ramp, 피드백 누락, 수신 fault, 관절 범위/속도/전류/토크 초과, 주기 초과 시 모든 모터에 STOP을 시도한다. STOP은 중력 지지를 해제하므로 외부 지지가 필요하다. 통신 끊김 시 STOP 전달을 보장할 수 없어 검증된 모터 watchdog과 물리 정지 수단이 필요하다. Python/SocketCAN 50Hz는 실시간 보장이 없으며 실측 주기 검증이 필요하다.

mechPos의 전원 후 회전 수와 경계 동작은 펌웨어에서 확인해야 한다. 이 코드는 ±4π feedback을 임의로 누적하지 않는다. 위치 jump는 속도 한계로 중단한다. 재부팅 전 위치를 자동 복원하지 않는다.

## 기존 5축 코드의 범위

robstride_hardware.yaml은 6축 팔 CAN 매핑, hand_guiding.yaml은 6축+그리퍼 보정 프로파일이다. 기존 pallet_gripperless.urdf/MoveIt 시나리오는 별도의 구형 5축 시뮬레이션이다. 기존 trajectory bridge는 robstride_hardware_legacy_5axis.yaml로 명시적으로 분리했다. 구형 시뮬레이션 전체를 6축으로 마이그레이션한 것은 아니며 실물 손 유도에는 사용하지 않는다.

프로토콜 근거: https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS05/RS05User%20Manual251112.pdf

사용자 확인 방향: 사진의 1~6번 화살표는 모터 위치 감소 방향. ID 7은 모터 위치 감소 시 닫힘, 증가 시 열림. URDF 좌표와의 부호 비교 및 영점/전달비 보정은 별도이다.

## URDF 기준 보정 도구

```bash
python3 tools/calibrate_hand_guiding.py
python3 tools/calibrate_hand_guiding.py --samples measured_poses.json
# 출력 오차와 축 대응 검토 후에만 설정 반영:
python3 tools/calibrate_hand_guiding.py --samples measured_poses.json --apply
```

측정 JSON은 관절별 최소 3개의 `[URDF 관절 각도(rad), 연속 모터 각도(rad)]` 쌍이다. 실제 측정한 자세의 URDF 좌표가 필요하며 모터 각도만으로 영점을 추정하지 않는다. 예시 형식은 `{"J1": [[q0,m0],[q1,m1],[q2,m2]]}` (기호는 측정 숫자로 대체). 원점 근처, 중간, 끝 쪽의 서로 다른 자세를 사용한다. ±4π 경계의 raw 값을 그대로 섞지 않는다. 회전 방향/전달비/영점을 함께 선형 적합하며 불충분한 이동, 비유한 값, 비선형 잔차를 거부한다. 세 점의 적합은 독립적인 실물 검증을 대체하지 않는다. 적용 후 commissioned와 gripper_homed는 false로 돌아간다.

`config/hand_guiding_urdf_axes.json`은 URDF 모든 좌표가 0일 때 base_footprint 기준 관절 중심과 +회전축이다. 오른손 법칙을 사용한다. 사진 자세와 좌표계 정합이 확인되지 않아 사진 화살표를 곧바로 direction 값으로 변환하지 않았다. 그리퍼 모터 다회전과 URDF g_r 사이의 선형성이 입증되지 않으면 해당 URDF/전달 모델부터 수정해야 한다.

## 2026-10-22 사용자 기준 영점 변경

스크린샷의 자세를 유지하면서 전 관절을 0으로 변경했다. 기존 J1=0, J2=2.121, J3=-3.004, J4=-0.560, J5=-1.647, J6=3.140, g_r=0.103 rad를 적용했다. 화면 표시의 0.001 rad 정밀도를 사용했으며 실물 엔코더 영점 명령은 보내지 않았다.

q_new=q_old-offset. joint origin에 axis 기준 offset 회전을 적용하고 lower/upper에서 offset을 뺐다. mimic g_l도 함께 변환했다. 링크 로컬 질량/COM 좌표는 유지한다. pallet.urdf와 pallet_with_gripper.urdf에 동일 적용했으며 원본은 urdf/before_zero_rebase_20261022/에 보존했다. config/pallet.srdf의 home/open을 새 0 자세로 설정했다. 하드웨어 zero_rad는 재측정해야 한다.

새 g_r=0은 사진의 열린 자세이고 닫힘 쪽이 음수(-0.103까지)다. 증가 방향은 열림이다. 구형 5축 모델과 별도 MoveIt 프로파일은 이번 영점 변경 대상이 아니다.

사용자 후속 요청으로 새 영점 기준 J1..J6 위치 범위를 정확히 -π..+π로 변경했다. 이전 이동 범위를 보존한 것이 아니라 새 범위를 요청한 것이다. ID 7 모터의 -π..+π 요청도 hand_guiding.yaml에 별도 기록했으나, 전달비 미확인으로 손가락 g_r 제한값으로 환산하지 않았다. 모터 내부 한계는 변경하지 않았다.

후속 요청: J4의 axis를 0 0 1에서 0 0 -1로 반전했다. 현재 영점 자세와 ±π 제한은 유지하며, 양의 슬라이더 방향과 해당 일반화 중력 토크 부호가 반전된다. 하드웨어 direction 보정은 여전히 미확인이다.

베이스 수평 보정: base_link STL의 최하단 평행 받침면 426개 삼각형에서 면적 가중 법선을 추정했다. 약 2.589764도 기울기를 최소 회전으로 수평화하고 최저점을 z=0으로 이동했다. 전체 base_footprint_joint 고정 변환만 수정했으며 관절 로컬 영점/축/제한과 링크 물성은 유지했다. J1 수직 오차는 약 0.00045도다. 원본은 urdf/before_base_level_20261022/, 변환은 config/base_level_alignment.json에 기록했다. 기존 사진 자세의 테이블 접촉 추정은 이 전역 보정 후 재확인이 필요하다.

사진 미리보기 후속 조정: 베이스 수평 보정 후 J3 이후가 수평으로 뻗도록 physical_pose_preview.yaml의 J3/J5만 조정했다. J3→손목 및 손목→그리퍼 관절 중심 연결선의 높이 차이는 수치상 1e-8m 이하이며, 개별 짧은 구간은 CAD 원점 오프셋으로 약 0.037도 이내 편차가 있다. URDF 영점이나 모터 보정은 변경하지 않았다.

## 읽기 전용 중력 토크 진단

모터 연동 RViz가 실행 중이면 USB 포트를 다시 열지 않고 기존 /joint_states를 구독한다. 같은 ROS_DOMAIN_ID를 사용하고 joint_state_publisher_gui 등 다른 자세 발행자는 종료한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/minwoo/Development/LLM/ros2_ur_ws/install/setup.bash
python3 /home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_gravity_preview.py --from-joint-states
```

--once는 한 번만 계산한다. RViz 모터 연동을 종료했다면 --from-joint-states 없이 직접 시리얼 위치를 읽을 수 있다. --motor-positions 0 0 0 0 0 0은 장치 없는 오프라인 확인이다. 출력 gravity_joint_nm은 URDF 관절 기준 보상 토크, motor_output_nm은 확인된 방향/외부 전달비를 반영한 모터 출력축 토크다. predicted_iq_a=null이면 Iq 토크 환산 미확정이며 전류를 추측하지 않는다. 그리퍼는 g_r=0 열린 기준 자세, 추가 payload 없음으로 가정한다. 이 도구는 전류 명령을 전송하지 않는다.

2026-10-22 실제 /joint_states 구독 성공: 당시 J2 약 -10.30 N·m, J3 약 -5.59 N·m, J5 약 +1.76 N·m(URDF 관절 보상 방향). 이는 현 URDF 질량/COM 가정에 따른 계산이며 실측 토크 검증이 아니다. USB 직접 조회는 다른 프로그램 점유로 실패했으므로 기존 토픽 구독을 사용했다. 토크/전류 한계 및 Iq 상수 미확정, 구동 시험 미수행.
