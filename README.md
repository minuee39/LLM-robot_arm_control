# LLM Robot Arm Control

UR10e 로봇팔을 Isaac Sim, ROS 2 Humble, MoveIt 2, MoveIt Task Constructor(MTC), OpenCV/YOLO 비전 인식과 연결해 pick-and-place 작업을 수행하는 개발 저장소입니다.

현재 목표는 완성된 VLA 모델을 바로 학습하는 것이 아니라, 자연어 명령을 구조화된 JSON 명령으로 바꾸고, 비전/시뮬레이션 상태에서 물체 위치를 확인한 뒤, MoveIt 2 또는 Isaac Sim 제어기로 로봇 동작을 실행하는 중간 구조를 안정화하는 것입니다.

## 현재 구현

- 자연어 명령을 `pick_object`, `target_object`, `relation`, `offset_m` 형태의 JSON 명령으로 변환
- Isaac Sim 기반 UR10e pick-and-place 예제와 카메라/비전 파이프라인 실험
- OpenCV/YOLO 기반 색상 물체 인식 및 ROS 토픽 연동 실험
- MoveIt 2 + MTC 기반 UR10e/Robotiq 2F-140 원통 pick-and-place
- RViz Motion Planning Tasks 패널에서 task stage, solution, cost 확인
- Planning Scene에 원통 object, wall, floor guard collision object 추가
- 바닥 collision guard를 파랑 반투명 object로 표시해 로봇 링크가 바닥 영역에 닿는지 확인

## 주요 구조

```text
.
├── ur10e/
│   ├── command_parser.py          # 자연어 명령 파싱
│   ├── llm_to_json.py             # LLM 응답을 JSON 명령으로 변환
│   ├── pick_place_example.py      # Isaac Sim pick-place 진입점
│   ├── scene_config.py            # 물체 이름, 색상, 위치, relation 설정
│   ├── apps/                      # 카메라, Isaac/MoveIt bridge, target follow 실행 앱
│   ├── vision/                    # OpenCV, YOLO, depth, mock vision 모듈
│   ├── controller/                # IK, RMPflow, pick-place 제어 코드
│   └── tests/                     # 파서/비전 단위 테스트
├── ros2_ur_ws/
│   └── src/
│       ├── mtc_tutorial/          # MoveIt Task Constructor pick-and-place
│       ├── isaac_moveit_bridge/   # Isaac Sim과 MoveIt 2 연동 노드
│       ├── ur10e_robotiq_description/
│       └── ur10e_robotiq_2f140_moveit_config/
└── docs/
    └── robot-development-workflow.md
```

## 데이터 흐름

```text
사용자 자연어 명령
-> command_parser / LLM to JSON
-> command dict 검증
-> 비전 인식 또는 Isaac Sim scene state로 물체 위치 확인
-> 목표 pick/place pose 계산
-> MoveIt 2 또는 Isaac Sim controller 실행
-> UR10e + Robotiq gripper pick-and-place
-> RViz / OpenCV / 로그로 결과 확인
```

예시 명령 포맷:

```json
{
  "action": "pick_place",
  "pick_object": "blue_block",
  "target_object": "green_block",
  "relation": "near",
  "offset_m": [0.12, 0.12, 0.0],
  "confidence": 1.0
}
```

지원 relation 예시:

```text
on, left_of, right_of, front_of, behind, near
```

## ROS 2 / MoveIt 2 실행

ROS 환경을 먼저 로드합니다.

```bash
cd ~/Desktop/LLM/ros2_ur_ws
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
```

`mtc_tutorial`만 다시 빌드할 때:

```bash
colcon build --packages-select mtc_tutorial
source install/setup.bash
```

MTC pick-and-place 데모 실행:

```bash
ros2 launch mtc_tutorial mtc_demo.launch.py
```

RViz 설정을 포함한 데모 실행:

```bash
ros2 launch mtc_tutorial demo.launch.py
```

Motion Planning Tasks 패널에서 각 stage의 성공/실패 개수와 `cost`를 확인합니다. `cost`는 MTC solution ranking 값이며, 낮을수록 선호되는 해입니다. 항상 물리 단위로 해석되는 값은 아니므로, 먼저 실패 stage와 comment를 확인하는 것이 중요합니다.

## MTC Pick-And-Place 메모

핵심 설정은 [mtc_tutorial.cpp](ros2_ur_ws/src/mtc_tutorial/src/mtc_tutorial.cpp)에 있습니다.

- arm group: `ur_manipulator`
- hand group: `gripper`
- end effector: `robotiq_2f140`
- IK frame: `robotiq_grasping_frame`
- pick object: `object`
- floor collision guard: `floor_z_guard`

현재 원통을 중간에서 수평 방향으로 잡기 위해 grasp IK frame transform을 사용합니다. Planning Scene에는 원통, wall, floor guard를 함께 넣고, floor guard는 파랑 반투명으로 표시합니다.

SRDF named state는 [ur10e_robotiq_2f140.srdf](ros2_ur_ws/src/ur10e_robotiq_2f140_moveit_config/config/ur10e_robotiq_2f140.srdf)에 있습니다. revolute joint 값은 degree가 아니라 radian입니다.

RViz 초기 자세는 SRDF의 `ready`만으로 결정되지 않고 [initial_positions.yaml](ros2_ur_ws/src/ur10e_robotiq_2f140_moveit_config/config/initial_positions.yaml)도 영향을 줍니다.

## Isaac Sim / OpenCV / RViz 확인 흐름

Isaac Sim 카메라 또는 시뮬레이션 상태를 ROS 2로 내보낸 뒤, OpenCV/YOLO로 물체를 인식하고, RViz에서 planning result를 확인하는 흐름을 사용합니다.

```bash
cd ~/Desktop/LLM
python3 ur10e/apps/pick_place_with_camera.py
```

카메라 ROS publisher:

```bash
bash ur10e/scripts/run_sim_camera_publisher.sh
```

RViz에서 카메라/토픽 확인:

```bash
bash ur10e/scripts/run_sim_camera_rviz.sh
```

MoveIt bridge 관련 앱은 [isaac_moveit_bridge](ros2_ur_ws/src/isaac_moveit_bridge)와 `ur10e/apps/` 아래 실행 스크립트를 기준으로 확인합니다.

## 개발 팁

- `.cpp` 파일을 바꾸면 해당 ROS 2 패키지를 다시 `colcon build` 해야 합니다.
- `.srdf`, `.yaml` 설정 파일은 launch가 install space를 읽는지 symlink install을 쓰는지에 따라 rebuild 또는 재실행이 필요합니다.
- `ament_package` Python 오류가 나면 `PYTHONPATH`/`PYTHONHOME`을 비우고 `/opt/ros/humble/setup.bash`를 다시 source합니다.
- wall이나 floor 같은 collision object는 launch만 실행한다고 자동으로 생기지 않습니다. Planning Scene에 object를 추가하는 노드나 코드가 실제로 실행되어야 합니다.
- 원격 MoveIt dependency checkout은 별도 upstream Git 저장소입니다. 이 저장소에 포함하려면 vendoring 또는 submodule 정책을 정한 뒤 추가하는 것이 좋습니다.

## 테스트

Python 단위 테스트:

```bash
cd ~/Desktop/LLM
pytest ur10e/tests
```

ROS 2 빌드 검증:

```bash
cd ~/Desktop/LLM/ros2_ur_ws
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
colcon build --packages-select mtc_tutorial
```

## 다음 작업

- OpenCV/YOLO 인식 결과를 MoveIt target pose로 안정적으로 변환
- 카메라 좌표계 기준 left/middle/right 공간 참조 해석 추가
- `last_picked_object` 같은 작업 메모리로 "방금 옮긴 물체" 처리
- MTC 실패 stage별 원인 로깅 정리
- Isaac Sim 결과와 RViz Planning Scene 결과를 같은 좌표계 기준으로 비교
- 실제 로봇 적용 전 속도/가속도 제한, 충돌 object, 비상정지 절차 점검
