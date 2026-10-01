# pallet_gripper ROS 2 이식

작성일: 2026-09-16. 목적: Desktop의 SolidWorks export를 ROS 2 Jazzy workspace로 이동하고 표시 환경을 구성한다.

## 최종 기준 workspace

`/home/minwoo/Development/LLM/ros2_ur_ws`

## 주요 파일

- 패키지: `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper`
- `urdf/pallet_gripper.urdf`: 사용자 지정 관절 유형·축 및 J1~J9 링크/관절 이름을 반영한 모델
- `source/`: 원본 URDF/CSV, ROS 1 launch/config, export 로그, 원본 manifest/CMake 보관
- `meshes/`: 원본 STL 10개
- `launch/display.launch.py`, `config/display.rviz`: ROS 2 표시 설정
- `config/joint_names.yaml`: 빈 이름을 제거한 원본 가동 관절 목록. ros2_control 설정은 아님

## 문제 → 원인 → 해결 → 검증

- ROS 2 빌드 불가 → ROS 1 catkin 및 비표준 패키지 이름 → ament_cmake, pallet_gripper 이름과 의존성으로 변경 → colcon build 성공.
- mesh 경로 미해결 → 이전 패키지 URI → package://pallet_gripper/로 수정 → visual/collision 참조 20개 존재 확인.
- RViz 설정 누락 → 원본 launch가 없는 urdf.rviz 참조 → 기존 pallet_gripper_description의 ROS 2 표시 구성을 복사하고 패키지명 수정 → 노드 3개 및 OpenGL 초기화 확인.
- g2/g_r/g_l 축 미정 → 원본 축이 0 0 0 → 표시 launch에서만 fixed 처리하고 경고 → launch 로그에서 적용 확인. 최초 이식 당시 원본과 동일함을 XML 비교로 검증. 이후 사용자 요청에 따라 아래 관절 설정으로 변경.

## 최종 구조와 설계 결정

Desktop 원본 폴더는 위 패키지 경로로 이동했다. 기존 pallet 및 pallet_gripper_description은 수정하지 않았다. 원본의 관절 원점과 질량/관성을 Z-up으로 변환하며 관절 유형·축·이름은 아래 사용자 지정 설정을 적용한다. Z-up 변환은 메시와 URDF에 이미 bake되어 있으므로 launch에서 다시 회전하지 않는다. base_footprint_joint의 z 오프셋으로 메시 최저점을 바닥에 맞춘다. 원본 ROS 1 설정은 source에 보관하고 실행용 설정과 분리한다.

## 실행 순서

터미널 1:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pallet_gripper --symlink-install
source install/setup.bash
ros2 launch pallet_gripper display.launch.py
```

Ctrl+C로 종료한다.

## 검증 명령과 결과

- `colcon build --packages-select pallet_gripper --symlink-install`: 성공.
- `check_urdf src/pallet_gripper/urdf/pallet_gripper.urdf`: 파싱 성공, base_link 루트와 10개 링크 확인.
- Python XML 비교: 이름/URI 외 원본과 동일, mesh 참조 20개 존재, launch 문법 검사 통과.
- ROS_DOMAIN_ID=89에서 display launch 10초 실행: robot_state_publisher 초기화, joint_state_publisher_gui 모델 수신, RViz OpenGL 초기화 확인.
- 종료 SIGINT 때 GUI exit code -2가 기록됨. RSP/RViz는 정상 종료. 실행 중 오류 없음.
- 로그: `/tmp/pallet_gripper_launch_check.log`.
- 화면 육안 검증 및 실제 장비 구동은 수행하지 않음.

## 남은 주의사항

J8/J9(기존 g_r/g_l)는 축이 미정이므로 미리보기에서 고정된다. 특히 J9의 원본 prismatic 범위 ±3.14 m는 실측값 확인이 필요하다. J6(기존 g1)은 revolute, 축 0 1 0, 범위 -3.14~3.14 rad이며 J7(기존 g2)은 fixed, 축 0 0 0이다. SRDF 생성 전 그리퍼 관절 축/범위/연동을 확정하거나 별도의 명시적인 고정 그리퍼 모델을 사용해야 한다. 원본 ROS 1 gazebo launch는 보관만 하며 ROS 2 실행 대상이 아니다.

## 다음 작업 로드맵

- [x] 패키지 이동 및 ROS 2 설정 이식
- [x] 빌드·URDF·mesh·표시 노드 기동 검증
- [x] g1 회전/g2 고정 및 링크·관절 이름 변경
- [ ] 손가락 J8/J9 축 및 이동 범위 확정
- [ ] 확정된 모델로 SRDF/MoveIt 설정 생성

## 사용자 지정 관절·링크 이름 (2026-09-16)

링크/슬라이더 이름 불일치 → GUI는 관절 이름을 표시 → 각 관절과 자식 링크를 같은 이름으로 통일 → URDF 파싱 및 Joint State Publisher의 J1~J6 인식 검증.

| 기존 링크 | 새 링크 | 기존 관절 | 새 관절 | 축 | 동작 |
|---|---|---|---|---|---|
| no.1~no.5 | J1~J5 | J1~J5 | J1~J5 | 기존 유지 | 회전 |
| gripper | J6 | g1 | J6 | 0 1 0 | 회전, 기존 한계 -3.14~3.14 rad 유지 |
| move_gripper | J7 | g2 | J7 | 0 0 0 | 고정 |
| gripper_r | J8 | g_r | J8 | 0 0 0 | 축 미정, 미리보기 고정 |
| gripper_l | J9 | g_l | J9 | 0 0 0 | 축 미정, 미리보기 고정 |

base_link와 launch에서 추가하는 world는 유지한다. mesh 파일명과 source 원본은 보존한다. 기존 실행을 종료하고 위 display launch를 재실행하면 반영된다. 실기 구동은 수행하지 않았다.

## 간단한 그리퍼 개폐 모델 (2026-09-16 후속 변경)

목적: 실제 나사축/연결봉 기구를 생략하고 양쪽 손가락 mesh를 평행 이동시켜 개폐를 표시한다. 이 절이 위의 J8/J9 미정·고정 설명을 대체한다.

문제 → J8/J9 축이 0이고 유효 이동 범위가 없어 슬라이더 조작 불가.
원인 → CAD export에 개폐 운동 정의가 없음.
해결 → J7 좌표계 기준 J8은 +X, J9는 -X 방향으로 이동하도록 각 관절 로컬 축을 역회전하여 지정. 두 관절은 prismatic, 범위 0~0.01 m. J9에 J8의 mimic(multiplier=1, offset=0) 적용.
검증 → check_urdf 및 설치된 Joint State Publisher로 독립 관절 J1~J6/J8, J9 종속 관계와 이동 방향 확인.

- 최종 파일: `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet_gripper/urdf/pallet_gripper.urdf`
- 기존 팔 J1~J6과 고정 J7, mesh 및 origin은 유지.
- J8=0은 CAD 초기 개방 자세, J8=0.01은 양쪽 각각 10 mm 안쪽 이동. 완전 폐쇄를 의미하지 않음.
- 10 mm는 임시 표시 범위이며 실제 하드웨어 측정값이 아님. effort/velocity는 기존 값 유지, 실기 구동 설정은 미완료.
- 연결봉까지 포함된 mesh가 통째로 이동하므로 연결 부위의 분리/겹침이 가능함. 실제 기구 및 충돌 모델의 정확성을 보장하지 않음.
- 실행: 기존 launch 종료 후 `ros2 launch pallet_gripper display.launch.py`. J8 슬라이더 하나로 양쪽 이동.
- [x] 간단한 대칭 개폐 구현
- [ ] 실측 개폐 범위 및 부품별 기구 모델 반영

## 회전식 그리퍼 개폐 모델 (2026-09-16)

목적: J8/J9 손가락을 평행 이동시키던 임시 모델을 각 손가락의 기존 CAD
origin을 회전 중심으로 사용하는 대칭 회전 모델로 교체한다. 이 절이 바로 위의
prismatic 개폐 설정을 대체한다.

문제 → prismatic 모델은 손가락과 연결봉 mesh 전체가 직선 이동하여 실제 회전식
개폐 의도와 맞지 않았다.

원인 → CAD export에 그리퍼 운동학이 없어서 표시를 위해 임시 직선축을 적용했다.

해결 → J8/J9를 revolute로 바꾸고 두 축을 `0 0 -1`로 설정했다. J8을 독립
구동 조인트로 사용하며 J9는 `multiplier=-1`로 J8을 반대 방향 추종한다. J8의
0 rad는 CAD 초기 개방 자세, 0.229 rad는 Joint State Publisher에서 확인한
최대 닫힘각이며 J9의 대응 범위는 -0.229~0 rad다. 두 관절의 effort는 10,
velocity는 1.0 rad/s로 설정했다.

검증 → `check_urdf` 파싱, mimic 관계, Joint State Publisher의 독립 슬라이더가
J8 하나인지 확인하고 RViz에서 양쪽 손가락이 안쪽으로 회전하는지 육안 확인한다.
0.229 rad는 Joint State Publisher 표시 기준이며 실제 하드웨어의 기계 한계나
충돌 없는 최대각은 실측으로 확인해야 한다.

- [x] J8/J9 revolute 및 반대 방향 mimic 구성
- [ ] RViz 개폐 방향·mesh 간섭 육안 검증
- [ ] 실측 개폐각과 속도·토크 한계 반영

## 실측 개방 폭을 반영한 직선 그리퍼 모델 (2026-09-29)

이 절이 위의 회전식 모델을 대체한다. 실측 최대 벌어짐은 두 손가락 끝 사이
`67 mm`다. J8/J9는 대칭 `prismatic` 관절이며 J9가 J8을 `multiplier=1`로
따른다. J8은 **한쪽 손가락 이동량**이므로 `0..0.0335 m`이고, 손가락 사이
간격은 `2 × J8 = 0..0.067 m`다. `J8=0`이 완전 닫힘, `J8=0.0335`가
최대 열림이다. `open` 그룹 상태와 초기 자세는 `0.0335 m`다.

기존 충돌 메시의 손가락 끝 간격을 계산해 닫힘 영점에서 두 끝이 만나도록
관절 origin을 안쪽으로 이동했다. 회전식 메시를 평행이동시키는 단순화 모델이라
연결부는 실제 기구학과 다를 수 있다. 사용자 지시에 따라 손가락끼리의
충돌은 MoveIt SRDF와 Isaac PhysX에서 모두 제외한다. 이 제외는 손가락과
블록, 손가락과 팔의 다른 충돌을 자동으로 끄지 않는다.

```bash
cd /home/minwoo/Development/LLM
python3 ros2_ur_ws/src/pallet_gripper/tools/gripper_calibration.py --object-width-mm 30
```

이 계산의 기본 손가락 끝 10 mm 영역에서 J8 `0 m`는 `0 mm`, `0.0335 m`는
`67 mm`, 30 mm 블록 목표는 `0.015 m`다. 실제 손가락 힘, 속도, 반복정밀도는
이번 개방 폭 측정만으로 확정되지 않았으므로 설정된 힘/속도는 시뮬레이션
값으로만 취급한다.

Isaac Sim에서는 CAD 충돌 메시의 연결부가 블록에 먼저 닿아 이송 중 미끄러짐이
발생했다. 따라서 시뮬레이터에 가져오는 임시 URDF의 손가락 충돌체만 평행한
손끝 패드 박스로 바꿨다. 공개 URDF 시각·충돌 메시와 MoveIt 충돌 모델은
그대로이며, 시뮬레이터 패드는 실제 접촉면 실측값이 아니다. 30 mm 블록
시험에서는 `J8=0.012 m` 닫힘 명령, 손끝 목표를 블록 중심보다 15 mm 낮춘
MTC 프로파일로 물체를 수직 상태로 옮겼다. 자세·위치 수치와 실행 절차는
`docs/pallet-simulation-handoff-2026-09-29.md`에 기록했다.

## 관성·좌표 재검증 (2026-09-18)

10개 링크의 질량중심과 관성 텐서를 보관된 SolidWorks Y-up 원본과 다시 대조했다. 현재 값은 `Rx(+pi/2)` Z-up 변환과 일치하며 모든 관성행렬이 양의 정부호이고 주관성모멘트 삼각조건을 만족한다. 따라서 관성값 자체는 변경하지 않았다.

명확한 좌표 오류였던 launch의 중복 90도 회전과 base_link 이중 부모를 제거했고, base STL 최저점에 맞춰 base_footprint_joint를 4.224755 mm 올렸다. fixed J7에 남아 있던 zero axis/limit도 제거했다. 상세 결과와 한계는 `INERTIA_AUDIT.md`, 재검증은 `tools/validate_inertials.py`를 사용한다.

## TCP 계산 및 MoveIt 적용 (2026-09-18)

J8/J9의 zero-position STL을 실제 joint origin/rpy로 J7 좌표계에 변환했다. 두 손가락 끝단은 각각 약 `(-0.02211236, 0, 0.094615155)`와 `(0.02205403, 0, 0.094474923) m`이다. CAD의 약 0.058 mm 좌우 비대칭은 설계 중심축에서 제거하고 두 끝단 z의 평균을 사용했다.

- `J7 -> tcp`: `xyz="0 0 0.094545039"`, `rpy="0 0 0"`
- TCP `+Z`: 손가락 사이에서 바깥쪽으로 향하는 접근 방향
- SRDF `pallet_arm` tip: `J7`에서 `tcp`로 변경
- end-effector parent: 실제 손가락 공통 부모인 `J7` 유지

`tools/validate_inertials.py`는 매 실행 시 STL에서 끝단을 다시 계산해 TCP가 일치하는지 확인한다. 이 위치는 손가락 최외곽 끝단의 중심이며 물체가 실제로 접촉하는 패드 중심을 실측한 결과는 아니다.
