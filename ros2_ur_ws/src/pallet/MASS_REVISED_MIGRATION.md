# Pallet 질량수정본 이관 기록

작성일: 2026-09-17. 목적: 새 CAD 메시/질량/관성 데이터를 ROS 2 `pallet` 기본 표시 모델로 적용하고 잘못된 조립 좌표를 복구한다.

## 기준 경로와 보존

- Workspace: `/home/minwoo/Development/LLM/ros2_ur_ws`
- 변경 패키지: `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet`
- 입력: `/home/minwoo/Downloads/그리퍼 포함 질1량수정본` (원본 변경 없음)
- 입력 전체 사본: `urdf/mass_revised_source/` (18개 파일 SHA256 검증, COLCON_IGNORE 추가).
- 새 메시: `meshes/mass_revised/` (10개 STL을 원본 그대로 복사, 변형/재메싱하지 않음).
- 기존 5축 모델: `urdf/pallet_gripperless.urdf` 보존. 기존 루트 메시/충돌 메시/CAN 설정/스크립트도 보존.
- 설치 직전 기존 패키지 전체와 기존 MoveIt xacro는 workspace의 `backups/pallet-before-mass-revised-20260917/`에 보존한다.

## 문제 → 원인 → 수정 → 검증

1. 새 메시를 기존 URDF와 혼합하면 부품과 질량 표시가 분리된다. 새 export는 메시 로컬 좌표까지 변경했지만 조인트 위치는 일관되게 갱신되지 않았다. 새 STL을 기존 `pallet_gripper_description/meshes`와 강체 정합하여 기존 조립 링크 좌표로 옮기는 변환을 복구했다. `visual`과 `collision`에 동일한 변환을 적용하며 메시 원본은 그대로 둔다.
2. 질량중심만 이전 값으로 복사하면 관성 좌표와 일치하지 않는다. 각 링크에 `c_link = R*c_export + t`, `inertial.rpy = R*R_export`를 적용했다. 관성 텐서는 같은 관성 프레임에서 원본 6개 성분을 유지한다. 이는 텐서를 링크 축으로 회전시켜 기록하는 것과 동등하며, COM 기준 텐서에 평행축 항을 추가하지 않는다.
3. base COM 원본 `(1.1834,-0.552,-0.90118)`은 원본 메시 경계 밖이다. CAD Y-up → Z-up으로 추정되는 `Rx(+pi/2)`를 질량 프레임에 먼저 적용했다. **추정 보정이지 CAD로 확인한 값이 아니다.** 변환 후 COM은 약 `(-0.0017917271,0.0392175844,0.0000222371)`이다. 메시 경계 안에 들어온다는 것만으로 물성이 맞다고 판정하지 않는다.
4. no.2/no.3 관성은 양의 정부호지만 주관성모멘트 삼각조건을 위반한다. 초과량은 각각 약 `5.866122722e-5`, `3.456911436e-5 kg*m²`이다. **숫자를 임의로 줄여 정상 처리하지 않았다.** CAD 재계산이 필요하다. `--strict-dynamics` 검증은 종료 코드 2로 실패한다.
5. 조립 좌표는 기존 assembled URDF를 사용하고, 새 메시의 로컬 변환으로 대응한다. 이전 요청대로 J6 revolute, g_r revolute 구동, g_l revolute mimic(-1), g2는 원본의 이동범위/effort가 0이므로 fixed로 정리했다. 그리퍼 표시 범위는 이전 설정인 ±0.103 rad를 유지하며 실제 기구 한계로 인증하지 않는다.
6. base_footprint에서 X축 90도 회전하여 Z-up을 만들고, 변환된 base 메시의 최저점으로 높이 `0.004224742975195261 m`를 계산했다. 최저점 z=0 수치 검사 통과. 이는 가장 낮은 점의 접촉이며 모든 발이 같은 평면임을 보장하지 않는다.
7. ROS 1 catkin 파일로 ROS 2 빌드 설정을 덮어쓰지 않았다. 기존 ament 구조를 유지하고 launch/패키지 경로 및 누락 의존성을 수정했다.

강체 정합 기록은 `config/mass_revised_audit.json`에 변환행렬, 이동량, RMS/99%/최대 잔차, 질량, COM, 고유값, 원본 해시를 포함한다. 대부분 링크는 최대 오차 약 1e-8~1e-7 m. no.3와 gripper는 메시 일부가 실제로 달라 최대 약 0.780 mm, 0.131 mm의 잔차가 남지만 99% 정점 잔차는 약 1e-8 m이다. 일부 형상 변경을 지우지 않고 새 STL을 그대로 사용한다.

## 최종 구조와 실행

`urdf/pallet.urdf`와 호환 진입점 `urdf/pallet_with_gripper.urdf`는 같은 새 모델이다. `launch/display.launch.py`가 `config/pallet.srdf`의 `pallet_arm/home`을 읽는다. `display_ros2.launch.py`와 `display_with_gripper.launch.py`도 이 런치로 연결된다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pallet pallet_moveit_config --symlink-install
source install/setup.bash
ros2 launch pallet display.launch.py
```

GUI 없는 표시 노드 검증:

```bash
ros2 launch pallet display.launch.py gui:=false rviz:=false
```

기존 `pallet_moveit_config`는 이전 `j1..j5`, `L2..L6`, `tcp`에 의존하므로 **기존 5축 모델을 계속 사용**한다. 해당 xacro의 include만 `pallet_gripperless.urdf`로 명시적으로 변경했다. 기존 그리퍼리스 Isaac 테스트 기본값도 같은 보존 파일로 바꿨다. 새 6축 모델을 예전 5축 하드웨어 설정으로 구동하지 않는다. 다른 `pallet_gripper*` 패키지는 변경하지 않았다. config/pallet.srdf는 표시 home용이며 컨트롤러를 구성하지 않는다.

## 검증

```bash
python3 src/pallet/scripts/validate_mass_revised.py
python3 -m unittest discover -s src/pallet/test -v
check_urdf src/pallet/urdf/pallet.urdf
python3 src/pallet/scripts/validate_mass_revised.py --strict-dynamics
```

사전 검증: 테스트 11개, XML/URDF 트리(11링크/10조인트), 원본/메시 해시, COM 경계, 축/리밋/mimic, 바닥 최저점, SRDF home 모두 통과. strict dynamics는 위 CAD 문제 때문에 실패하는 것이 정상이다. 이 검사는 정확한 질량분포·실기 동작을 인증하지 않는다.

## 남은 사항

- [x] 새 메시/질량 적용, 로컬 좌표 변환, 조립/바닥/그리퍼/표시 home 복구.
- [ ] base 질량 프레임을 SolidWorks에서 확인하고 추정값 교체.
- [ ] no.2/no.3의 COM 기준 관성 및 재질/질량 재계산.
- [ ] CAD의 나머지 질량 분포 정확성 확인(양의 정부호/경계 검사만으로 증명 불가).
- [ ] 새 모델 전용 MoveIt collision matrix/컨트롤러/실측 관절 한계 구성 및 실기 검증.

실기 모터/CAN 구동은 수행하지 않는다. 현재 새 모델은 시각화/기구 구조 확인용이며 정확한 동역학용으로 준비 완료된 상태가 아니다.

참고: ROS 공식 URDF inertial 프레임 설명: https://docs.ros.org/en/humble/Tutorials/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html
