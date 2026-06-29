Pallet 로봇팔 프로젝트 개발 컨텍스트

1. 프로젝트 개요

이 프로젝트는 자연어 명령을 입력하면 로봇팔이 물체를 인식하고, 명령을 구조화한 뒤, 시뮬레이션 또는 실제 로봇에서 pick-place 동작을 수행하는 시스템이다.

최종 목표는 다음과 같다.

사용자 자연어 명령
→ LLM 기반 명령 해석
→ JSON 형태의 작업 명령 생성
→ 비전 인식 또는 시뮬레이션 상태에서 물체 위치 확인
→ 목표 좌표 계산
→ MoveIt2 또는 Isaac Sim 제어기 실행
→ 로봇팔이 물체를 집고 원하는 위치에 놓기

현재는 완전한 VLA 모델을 바로 학습하는 단계가 아니라, LLM to JSON + 기존 로봇 제어기 + 비전 인식 모듈을 조합하여 VLA 구조로 발전시키는 중간 단계이다. RT-2처럼 VLA는 이미지와 언어를 입력받고 로봇 action을 출력하는 구조이며, RT-2는 로봇 action을 텍스트 토큰처럼 표현해 closed-loop 제어에 사용한다. OpenVLA는 7B 오픈소스 VLA로, 로봇 action 예측과 fine-tuning을 지원하므로 향후 참고할 수 있다.

2. 현재 구현된 핵심 구조

현재 Isaac Sim 기반 UR10e pick-place 예제를 수정하여 자연어 명령으로 블록을 이동시키는 구조를 만들고 있다.

현재 흐름은 다음과 같다.

사용자 입력
↓
parse_user_command()
↓
LLM 또는 로컬 파서가 명령 해석
↓
command dict 생성
↓
validate_command()
↓
command_to_target_position()
↓
pick_object, target_object, relation 기반 목표 좌표 계산
↓
PickPlace task에 전달
↓
PickPlaceController 실행
↓
UR10e 로봇팔 동작
↓
동작 완료 후 다음 명령 대기

현재 사용 중인 명령 형식은 다음과 같다.

{
    "action": "pick_place",
    "pick_object": "blue_block",
    "target_object": "green_block",
    "relation": "near",
    "offset_m": [0.12, 0.12, 0.0],
    "confidence": 1.0
}

지원 relation은 다음과 같다.

on
left_of
right_of
front_of
behind
near

예시 명령:

파란 블럭을 초록 블럭 옆에 둬
빨간 블럭을 파란 블럭 위에 둬
제일 오른쪽 블럭을 가운데 블럭 옆에 둬
방금 옮긴 블럭을 빨간 블럭 왼쪽에 둬

현재 문제는 “방금 옮긴 블럭”, “제일 오른쪽 블럭”, “가운데 블럭”처럼 상황 기억과 공간 관계 추론이 필요한 명령에서 파서가 실패하거나 잘못된 물체를 선택하는 경우가 있다는 점이다.

3. 현재 주요 파일 구조

예상되는 주요 파일은 다음과 같다. 실제 repo를 확인한 뒤 이름이 다르면 맞춰서 수정해야 한다.

pick_place_example.py
- Isaac Sim 실행 진입점
- 사용자 명령 입력 루프
- PickPlace task 및 controller 실행

command_parser.py
- 자연어 명령을 command dict로 변환
- Gemini LLM 사용
- 실패 시 로컬 파서 fallback

llm_to_json.py
- LLM 호출
- JSON schema 검증
- confidence 검사
- Pydantic 모델 사용 가능

scene_config.py
- 블록 이름, 색상, 초기 위치
- relation offset 정의
- object position 관리

tasks/pick_place.py
- PickPlace task 정의
- 물체 spawn, 목표 위치 계산, task observation 생성

controllers/pick_place_controller.py
- RMPflow 또는 PickPlaceController 기반 제어

test_llm_to_json.py
test_command_parser.py
- 명령 파싱 테스트
- JSON schema 테스트
- relation 처리 테스트
4. 현재 시뮬레이션 상태

Isaac Sim에서 UR10e 로봇팔과 여러 개의 블록을 사용한다.

현재 예시 object position은 다음과 같다.

OBJECT_POSITIONS = {
    "red_block": np.array([-0.30, 0.30, 0.02575]),
    "blue_block": np.array([0.30, 0.30, 0.02575]),
    "green_block": np.array([0.0, 0.45, 0.02575]),
}

주의할 점:

Isaac Sim 카메라 기준의 “오른쪽/왼쪽/가운데”와 world coordinate 기준의 x/y 축이 다를 수 있다. 따라서 “제일 오른쪽 블럭” 같은 명령은 단순히 world x가 가장 큰 물체로 판단하면 카메라 화면과 다르게 해석될 수 있다.

해결 방향:

1. 카메라 좌표계 기준 object ordering 구현
2. world 좌표 → camera projection 변환
3. 화면상 x pixel 기준으로 left/middle/right 판단
4. fallback으로 world 좌표 기준 판단
5. LLM to JSON 구현 상태

현재 LLM은 자연어를 직접 로봇 action으로 바꾸는 것이 아니라, 중간 표현인 JSON으로 바꾼다.

이 방식의 장점:

- 로봇 제어 코드와 LLM 해석 코드를 분리할 수 있음
- LLM이 이상한 명령을 출력해도 schema로 검증 가능
- 시뮬레이션, MoveIt2, 실제 로봇 제어기에서 같은 command dict를 재사용 가능
- VLA로 가기 전 단계로 적합함

Code as Policies 논문에서는 LLM이 자연어 명령을 받아 perception API와 control API를 조합하는 policy code를 생성할 수 있다고 설명한다. 즉, 현재 프로젝트의 LLM to JSON 구조는 더 안전한 중간 단계이고, 향후에는 LLM이 JSON뿐 아니라 action sequence 또는 policy code를 생성하는 방향으로 확장할 수 있다.

현재 schema는 대략 다음과 같이 유지한다.

class PickPlaceAction(BaseModel):
    action: Literal["pick_place"]
    pick_object: str
    target_object: str | None
    relation: Literal["on", "left_of", "right_of", "front_of", "behind", "near"]
    offset_m: list[float]
    confidence: float

검증 조건:

- confidence < 0.7이면 사용자에게 재입력 요청
- pick_object가 scene에 없으면 오류
- target_object가 필요한 relation인데 없으면 오류
- relation이 허용 목록 밖이면 오류
- offset_m 크기가 너무 크면 오류
6. 현재 발생한 문제와 수정해야 할 내용
6.1 LLM API timeout / quota 문제

Gemini 호출 중 timeout 또는 429 RESOURCE_EXHAUSTED가 발생한 적이 있다.

필요한 수정:

- LLM 호출 timeout 설정
- retry 횟수 제한
- LLM 실패 시 로컬 parser fallback
- fallback 사용 시 사용자에게 로그 표시
- API key가 없어도 테스트 가능한 mock parser 추가
6.2 “방금 옮긴 블럭” 처리 문제

현재는 이전 작업 기억이 약하다.

추가해야 할 상태:

ROBOT_MEMORY = {
    "last_picked_object": None,
    "last_target_object": None,
    "last_relation": None,
    "last_command": None,
    "last_result": None,
}

명령 해석 예:

방금 옮긴 블럭을 빨간 블럭 왼쪽에 둬
→ pick_object = ROBOT_MEMORY["last_picked_object"]
→ target_object = "red_block"
→ relation = "left_of"
6.3 “제일 오른쪽”, “가운데”, “왼쪽” 처리 문제

현재는 LLM이 색상/위치를 잘못 추론할 수 있다.

해결 방향:

def resolve_spatial_reference(description, object_positions, camera=None):
    """
    description:
    - 제일 오른쪽 블럭
    - 제일 왼쪽 블럭
    - 가운데 블럭
    - 앞쪽 블럭
    - 뒤쪽 블럭

    camera가 있으면 camera image plane 기준으로 판단.
    camera가 없으면 world coordinate 기준으로 판단.
    """

우선순위:

1. 카메라 인식 결과가 있으면 image x/y 기준
2. 없으면 Isaac Sim world 좌표 기준
3. 모호하면 사용자에게 재질문
6.4 물체 위치 업데이트 문제

pick-place 완료 후 object position이 초기값 그대로 남으면 다음 명령이 잘못된다.

필요한 수정:

- 동작 완료 후 실제 object prim 위치 읽기
- OBJECT_POSITIONS 갱신
- scene_config의 정적 좌표와 runtime 좌표 분리

예상 함수:

def update_object_positions_from_sim(world, object_names):
    positions = {}
    for name in object_names:
        prim = world.scene.get_object(name)
        positions[name] = prim.get_world_pose()[0]
    return positions
6.5 로봇 속도 문제

현재 RMPflow 기반 pick-place 동작이 느리거나 떨림이 있다.

확인할 항목:

- physics dt
- controller frequency
- RMPflow gain
- damping gain
- target approach height
- end-effector offset
- waypoint 간 대기 시간
- gripper open/close 대기 시간

단, 속도를 올릴 때는 충돌과 overshoot가 증가할 수 있으므로 한 번에 큰 gain 변경을 하지 말고 단계적으로 조정해야 한다.

7. MoveIt2 전환 계획

현재 Isaac Sim RMPflow 기반 제어를 사용하고 있지만, 실제 로봇 제작 단계에서는 MoveIt2 기반 제어도 검토해야 한다.

MoveIt2에서 필요한 구성:

1. URDF 또는 Xacro 로봇 모델
2. SRDF 설정
3. joint limit 설정
4. ros2_control 설정
5. MoveIt planning group 설정
6. end-effector/gripper 설정
7. planning pipeline 설정
8. RViz MotionPlanning 확인
9. Python 또는 C++ action client 작성

MoveIt2에서 사용할 흐름:

LLM command dict
↓
object pose 확인
↓
target pose 계산
↓
MoveIt2 plan 생성
↓
trajectory 검증
↓
ros2_control로 trajectory 실행
↓
gripper close/open
↓
결과 상태 업데이트

MoveIt2용 인터페이스는 Isaac Sim 제어기와 분리해야 한다.

권장 구조:

class RobotExecutor:
    def execute_pick_place(self, command: PickPlaceCommand):
        raise NotImplementedError

class IsaacSimExecutor(RobotExecutor):
    def execute_pick_place(self, command):
        ...

class MoveIt2Executor(RobotExecutor):
    def execute_pick_place(self, command):
        ...

이렇게 하면 LLM parser와 task planner는 그대로 두고, 실행기만 Isaac Sim에서 MoveIt2로 교체할 수 있다.

8. 비전 인식 계획

현재는 object position을 코드에 직접 넣고 있지만, 실제 VLA 구조로 가려면 카메라 기반 인식이 필요하다.

비전 인식 목표:

RGB-D 카메라 입력
↓
물체 탐지
↓
색상/이름/class 추정
↓
2D bbox 추출
↓
depth 기반 3D 위치 계산
↓
camera frame → robot base frame 변환
↓
object pose 등록
↓
LLM command와 매칭

초기 단계에서는 색상 블록만 사용하므로 복잡한 모델보다 색상 기반 segmentation으로 시작해도 된다.

초기 구현:

- red_block, blue_block, green_block 색상 threshold
- contour 검출
- bbox center 계산
- depth image에서 중심 depth 추출
- camera intrinsic으로 3D 좌표 변환

이후 확장:

- YOLO / Grounding DINO / OWL-ViT 등 open-vocabulary detector
- Segment Anything 기반 mask 추출
- CLIP 또는 VLM 기반 object label 매칭
- 자연어 표현과 물체 이름 매칭

Code as Policies 방식에서도 perception API와 control API를 LLM이 조합하는 구조가 중요하다. 따라서 비전 모듈은 detect_objects(), get_object_pose(), parse_obj() 같은 API로 감싸는 것이 좋다.

권장 API:

def detect_objects(image, depth=None) -> list[DetectedObject]:
    ...

def get_object_pose(object_name: str) -> np.ndarray:
    ...

def resolve_object_name(language_ref: str, detections: list[DetectedObject]) -> str:
    ...

def project_world_to_image(world_pos, camera_params):
    ...

def deproject_pixel_to_world(pixel, depth, camera_params, tf_camera_to_base):
    ...
9. 실제 로봇팔 하드웨어 계획

실제 로봇팔은 6축 로봇팔 형태로 제작될 예정이다.

현재 고려 중인 구동계:

- RobStride QDD 00 × 3
- RobStride QDD 05 또는 유사 고토크 모터 × 3
- 총 6축
- CAN 통신 기반 제어
- USB-to-CAN 또는 CANable 계열 사용
- Jetson Orin Nano 또는 PC에서 상위 제어
- STM32 또는 ROS2 control node에서 하위 제어 가능

RobStride 00은 48V, 정격 토크 5 N·m, 피크 토크 14 N·m 사양을 갖는다. EL05/RobStride 계열 매뉴얼 기준 CAN 2.0, 1Mbps, extended frame을 사용하며, operation/current/velocity/position mode를 지원한다.

실제 로봇 전환 시 필요한 코드:

- CAN motor driver
- motor ID 설정
- zero position 설정
- joint state publisher
- ros2_control hardware interface
- MoveIt2 trajectory command 수신
- 각 joint position/velocity command 변환
- emergency stop 처리
- joint limit / torque limit / temperature monitoring

주의:

- 모터 동작 중 control mode를 바꾸면 안 됨
- zero position 설정 필요
- CAN timeout 설정 필요
- 과전압/저전압/과열/encoder 미보정 fault 처리 필요
- 실제 로봇에서는 시뮬레이션보다 속도 제한과 안전 제한을 강하게 적용
10. 앞으로 구현할 우선순위
1단계: 현재 Isaac Sim 코드 안정화
- 명령 입력 루프 안정화
- LLM timeout/retry/fallback 정리
- command schema 고정
- object position runtime update
- “방금 옮긴 블럭” memory 구현
- “왼쪽/오른쪽/가운데” spatial resolver 구현
- pytest 추가
2단계: 시뮬레이션 task 확장
- pick_place 외 place_on, place_near, stack, sort 구현
- 여러 물체 순차 이동
- “책상 정리해줘” 같은 고수준 명령을 여러 action으로 분해
- action history 저장
- 실패 시 recovery 동작 추가
3단계: 비전 인식 연결
- Isaac Sim 카메라 추가
- RGB-D 이미지 수집
- 색상 기반 블록 인식
- camera 좌표 → world 좌표 변환
- 인식된 object pose를 command_to_target_position에 연결
4단계: MoveIt2 구조 구축
- 실제 로봇 URDF 작성
- MoveIt2 setup assistant 설정
- RViz planning 확인
- Python MoveIt2 executor 작성
- IsaacSimExecutor와 동일한 command interface 사용
5단계: 실제 로봇 제어
- CAN 통신 테스트
- 모터 단독 position/velocity 제어
- 2축, 3축 순차 테스트
- 6축 joint state 확인
- MoveIt2 trajectory를 CAN command로 변환
- low speed pick-place 테스트
6단계: VLA 방향 확장
- OpenVLA 또는 유사 오픈소스 VLA 조사
- 직접 학습보다는 기존 모델 inference/fine-tuning 가능성 검토
- 현재 LLM to JSON 구조와 VLA 구조 비교
- 카메라 이미지 + 자연어 + action log 데이터셋 수집
- 나중에 imitation learning 또는 fine-tuning에 사용

OpenVLA는 PyTorch 코드베이스, HuggingFace 연동, LoRA fine-tuning, quantized inference를 지원하므로 향후 오픈소스 VLA 실험의 후보가 될 수 있다.

11. Codex가 코드를 수정할 때 지켜야 할 원칙
1. LLM parser, task planner, robot executor를 분리할 것.
2. Isaac Sim 전용 코드가 command_parser에 섞이지 않게 할 것.
3. MoveIt2 전환을 고려해 executor interface를 만들 것.
4. object position은 정적 초기값과 runtime 상태를 분리할 것.
5. 모든 command dict는 schema 검증을 거칠 것.
6. LLM 실패 시 로컬 parser나 사용자 재질문으로 안전하게 처리할 것.
7. 실제 로봇 제어 코드는 반드시 속도/가속도/토크 제한을 포함할 것.
8. 테스트 가능한 함수 단위로 쪼갤 것.
9. 좌표계 변환 함수는 독립적으로 테스트할 수 있게 만들 것.
10. 하드웨어 제어 전에는 반드시 시뮬레이션 또는 dry-run 모드를 둘 것.