# 4단계: CAD 비교와 실제 TCP 측정

작성일: 2026-10-27
기준 workspace: /home/minwoo/Development/LLM/ros2_ur_ws
사용자 확인: 팔 ID1~6 응답, 실물/RViz 축·방향·각도 일치, 약10Hz 및 OK 상태.
이 문서는 그 확인을 정밀 치수/절대 위치 정확도 검증과 구분한다.

## CAD에서 추출한 값 (실측 아님)

기준 모델: pallet/urdf/pallet_with_gripper.urdf.
원본 수치는 config/cad_joint_reference.json에 저장했다.

| 인접 관절 원점 | 원점 간 거리 mm |
|---|---:|
| J1 → J2 | 119.346 |
| J2 → J3 | 213.500 |
| J3 → J4 | 167.424 |
| J4 → J5 | 65.768 |
| J5 → J6 | 153.664 |

이는 연속된 joint origin 사이 거리이며 링크 외형 길이나 회전축 직선 간
최단거리가 아니다. CAD에서 같은 두 좌표 원점을 지정했을 때 비교하는 값이다.
측정 위치를 특정할 수 없으면 CAD 조립 모델의 축/원점 표시를 먼저 확인한다.

## 파지 TCP 정의

사용자 선택: **닫힌 상태에서 양쪽 손끝의 중심**. 기준 point-id는
closed_fingertips_center로 통일한다. 닫힘 상태에서도 간격이 있으면 실제 mm를 기록한다.
사용자 실측으로 Motor 앱 -12rad에서 0mm 닫힘을 확인했다.
이는 측정 기준이며 이 문서의 도구는 모터를 그 위치로 이동시키지 않는다.
피드백 화면의 gripper=0은 닫힘 실측을 뜻하지 않는다.

현 MoveIt tip은 gripper 링크 원점이다. 실제 접촉면 중심과 동일하다고 가정하지 않는다.
TCP는 기준 개구폭과 지점을 정하고, gripper 프레임 기준 위치(m)와 방향(rad/quaternion)을
기록해야 한다. adaptive 접촉 후 바뀐 형상에 그대로 적용할 수 있는지도 별도 확인한다.

선택된 닫힘 상태와 손끝 접촉면 위치를 모든 측정에서 동일하게 유지한다.
보이지 않는 기하학적 중심을 접촉점으로 쓸 경우 그 지점을 나타내는 고정 지그/포인터가 필요하다.
사진이나 g2의 29.921mm offset을 TCP 값으로 대입하지 않는다.

## 방법 A: CAD/실측 좌표 사용

SolidWorks에서 gripper 기준 좌표계를 표시하고 선택한 파지 중심의 x/y/z 및
축 방향을 측정한다. 실제 조립 치수와 비교한다. mm는 m로 변환해 기록한다.
회전축/좌표계가 불명확한 상태에서 자로 잰 길이를 xyz로 바로 대입하지 않는다.
이 방법의 측정값이 있으면 pivot 측정은 필수는 아니다.

## 방법 B: 고정점에 맞춘 여러 자세로 TCP 위치 계산

사용 도구: scripts/tcp_measurement.py. 모터 명령은 전송하지 않는다.
팔 자세를 만드는 동작은 이 도구의 기능이 아니며, 지지된 허용 범위에서 수행한다.
지지 상태에서 자세 변경이 어렵다면 위치제어 검증 후로 미룬다.

1. 개구폭을 하나로 유지하고, 움직이지 않는 기준점을 정한다.
2. 그리퍼의 동일한 강체 지점이 그 기준점에 닿도록 자세를 맞춘다.
3. 손가락이 접촉으로 휘거나 너트/링크가 움직이지 않아야 한다.
4. 서로 다른 축으로 방향을 바꾼 4~6개 자세에서 같은 접촉을 반복한다.
   같은 방향으로 평행 이동만 한 데이터로는 계산할 수 없다.
5. 해당 자세에서 feedback launch의 TF를 아래 명령으로 기록한다.

터미널 A (앞 단계와 동일, 가상 demo 및 다른 피드백 launch는 종료):

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pallet_physical_moveit_config feedback.launch.py \
  mode:=can transport:=serial serial_port:=/dev/ttyUSB0
```

터미널 B에서도 동일 workspace 환경을 source한 뒤:

```bash
mkdir -p /home/minwoo/Development/LLM/ros2_ur_ws/measurements/tcp
```

다음은 닫힌 상태의 실제 간격이 0mm일 때의 예제이다. 0은 모터 명령이 아니다.
닫아도 틈이 있으면 --opening-mm를 실측 간격으로 바꾼다.

```bash
ros2 run pallet_physical_moveit_config tcp_measurement.py capture \
  --opening-mm 0 --point-id closed_fingertips_center \
  --output /home/minwoo/Development/LLM/ros2_ur_ws/measurements/tcp/pose01.json
```

자세를 바꿀 때마다 pose02.json~pose06.json 등 새 파일명으로 반복한다.
출력 경로가 이미 존재하면 덮어쓰지 않는다. gripper TF가 0.5초보다 오래되었거나
5초 내 새 TF가 없으면 실패한다. 캡처 당시 팔이 정지해 있고 접촉이 유지되는지는
사용자가 확인해야 한다. 한 번에 캡처한 TF는 절대 위치 실측이 아닌 모델 예측이다.

```bash
ros2 run pallet_physical_moveit_config tcp_measurement.py solve \
  /home/minwoo/Development/LLM/ros2_ur_ws/measurements/tcp/pose*.json \
  --output /home/minwoo/Development/LLM/ros2_ur_ws/measurements/tcp_candidate.json
```

입력 파일들은 동일 모델·기준점·개구폭이어야 한다. 같은 폴더에 다른 측정 세트를
섞지 않는다. solver는 R_i * t + p_i = 고정점 관계를 최소제곱으로 푼다.

- tcp_translation_in_gripper_m: gripper 기준 TCP 위치 후보.
- residual_rms_mm/max_mm: 고정점 일관성. 절대 위치 정확도가 아니다.
- condition_number: 자세 배치에 따른 계산 민감도. 크게 나올수록 오차에 민감하다.
- TCP 방향은 이 방법으로 결정되지 않는다. 접근/닫힘 축을 별도로 정의해야 한다.
- 적합에 쓰지 않은 자세에서 기준점을 다시 맞춰 독립 검증한다.
- URDF/MoveIt 설정은 자동 변경하지 않는다.

## 현재 완료/미완료

- CAD 관절 원점 거리 추출 완료.
- TCP 캡처/위치 후보 계산 도구와 데이터 검증 구현.
- 실제 링크 치수/파지 중심/방향 측정은 미실시.
- 실측 없이 TCP를 생성하거나 MoveIt tip을 변경하지 않음.
- 실측 후 변환·기준 방향 확인, 고정 TCP 프레임 추가 및 별도 자세 검증 필요.

## 소프트웨어 검증

- 관련 pytest 6개 통과: 알려진 TCP 복원, 동일 방향 데이터 거부,
  개구폭 불일치/비정상 입력 거부, 측정파일 덮어쓰기 방지 및 기존 모델 일치.
- 가상 joint_states + robot_state_publisher로 읽기 전용 TF capture 기동 통과.
- 실물 측정값을 확보한 것으로 간주하지 않는다.

## CAD 참고 프레임으로 전환 (2026-10-27)

사용자 요청에 따라 고정점 실측은 보류했다. pose01~05는 원본 보존하고 적용하지 않았다.
최신 STL의 두 손끝 최대 +Y 끝단 중심의 평균을 사용해 CAD 참고 프레임을 추가했다.

- 이름: cad_tcp_reference_unverified
- 부모: gripper
- 위치 약 [−0.00881, 125.89582, 0.02127] mm
- 접근 후보 +Z: gripper +Y, 닫힘 축 후보 +X: move_gripper +X
- CAD 상태: g_r=g_l=0. 실제 개구폭은 확정하지 않음.
- MoveIt 활성 tip은 기존 gripper 유지. 참고 프레임은 IK 목표로 자동 선택하지 않음.

기존의 다른 gripper 패키지 TCP 94.545mm는 좌표계와 모델 상태가 달라 복사하지 않았다.
실물 motor 0rad=20mm를 CAD g_r=0으로 변환할 근거도 없다. 따라서 참고 프레임은
20mm 상태의 실측 TCP가 아니다. 실제 접촉 중심과 접근 방향 확인 후에만 최종 tip으로 정한다.

CAD 추출 과정과 mesh 해시는 config/cad_tcp_reference.json에 저장했다.
재생성 계산(출력만 하며 모터에 연결하지 않음):

```bash
ros2 run pallet_physical_moveit_config cad_tcp_reference.py
```

feedback launch를 다시 실행하면 robot_description에 참고 프레임이 포함된다.
TF 수신 상태에서 다음으로 프레임 위치를 확인할 수 있다.

```bash
ros2 run tf2_ros tf2_echo gripper cad_tcp_reference_unverified
```

TCP 실측과 별개로 팔 단축 위치제어 드라이버 개발·검증을 진행할 수 있다.
단축 시험은 관절각 기준으로 수행하며 CAD TCP를 이용한 실물 위치 접근은 아직 하지 않는다.
