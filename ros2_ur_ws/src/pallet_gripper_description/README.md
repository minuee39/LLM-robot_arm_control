# 그리퍼 포함 최종 CAD 기반 독립 description

작성일: 2026-09-16. 목적: 기존 pallet 모델과 분리하여 Desktop 최종 CAD export만으로 RViz 표시 환경을 구성한다.

## 최종 기준 workspace 및 파일

- Workspace: `/home/minwoo/Development/LLM/ros2_ur_ws`
- 원본: `/home/minwoo/Desktop/그리퍼 포함 (1).`
- 패키지: `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper_description`
- `source/`: 원본 URDF 및 CSV 바이트 그대로 보관
- `urdf/pallet_gripper.urdf`: robot 이름 및 package 메쉬 URI만 변경한 모델
- `meshes/`: 원본 STL 10개
- `launch/display.launch.py`, `config/display.rviz`: 독립 RViz 미리보기

## 문제 → 원인 → 해결 → 검증

1. 팔 회전축이 5개만 표시됨 → 원본 J1~J5만 유효 회전축이며 g1은 fixed → 원본을 보존하고 g1의 실제 6축 여부를 사용자에게 확인 요청 → 원본 URDF와 CSV 교차 확인.
2. 모델이 누워 보임 → CAD 베이스는 Y-up → 미리보기 world→base_link에 roll=π/2, z=0.004224755335599184 m 적용 → +Y→+Z 변환, 전체 10개 CAD 링크의 유한 TF 확인. 높이는 base_link.STL의 최소 Y로 계산.
3. 그리퍼 축 미정 → g2/g_r/g_l 축이 모두 0 0 0 → launch에서만 고정하고 경고 출력 → 원본과 배포 URDF의 관절 정의가 같음을 확인.
4. offscreen RViz 실행 실패 → Qt offscreen에서 GLX parentWindowHandle 생성 실패 → 실제 DISPLAY로 재검증 → RViz OpenGL 초기화 및 3개 노드의 8초간 오류 없는 기동 확인.

## 최종 구조와 설계 결정

기존 pallet, MoveIt, CAN 설정을 가져오지 않는다. 관절 이름, 원점, 축, 한계, 질량 및 관성은 CAD 원본을 유지한다. world 배치와 미정 그리퍼 고정 처리는 표시 launch에만 적용한다. 실기 제어 노드는 실행하지 않는다.

## 실행 순서

터미널 1, 최초 빌드 또는 설치 구조 변경 시:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pallet_gripper_description --symlink-install
```

터미널 1, RViz 실행:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pallet_gripper_description display.launch.py
```

Ctrl+C로 종료한다.

## 검증 명령과 결과

- 위 colcon build 성공.
- 원본 대비 XML 비교 성공: robot 이름/package URI 이외 변경 없음.
- 메쉬 참조 전부 존재하며 STL 10개 SHA256 원본 일치.
- 격리 ROS_DOMAIN_ID=87에서 `/joint_states` J1~J5 수신 및 10개 CAD 링크 TF 값의 유한성 확인.
- 실제 화면에서 위 display launch 8초 실행: robot_state_publisher, joint_state_publisher_gui, rviz2 기동, RViz OpenGL 초기화, 시작 오류 없음.
- 기동 로그: `/tmp/pallet_gripper_display_gui.log` (임시 파일).
- 화면 형상에 대한 육안 검증 및 실기 구동은 수행하지 않음.

## 남은 주의사항

- g1: no.5→gripper, fixed이나 축 0 1 0과 ±3.14 rad 한계가 남아 있다. CSV no.5 구성품에 6축 케이스와 RS05-3이 있어 손목 6축 후보이나 확인 전 변경하지 않는다.
- g2: revolute이나 축, 범위, effort, velocity 모두 0.
- g_r: prismatic이나 축, 범위, effort, velocity 모두 0.
- g_l: prismatic, zero axis, 범위 ±3.14 m 및 속도 3.14 m/s. 이 값을 실제 개폐 설정으로 사용하지 않는다.
- 그리퍼 메쉬는 여러 기계 링크가 합쳐져 있으며 리드스크류 및 링크 연동 정의가 없다.

## 다음 작업 로드맵

- [x] 최종 CAD 원본으로 독립 패키지 구성
- [x] 메쉬 경로, 빌드, 노드 및 TF 검증
- [ ] g1이 6번째 회전축인지 확인하고 관절 유형 확정
- [ ] 그리퍼 축·이동거리·연동관계 확인 후 개폐 모델 구성
- [ ] 새 모델 기준 MoveIt 및 하드웨어 매핑 검증
