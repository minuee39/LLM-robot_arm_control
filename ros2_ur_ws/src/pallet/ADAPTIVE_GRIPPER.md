# Adaptive gripper visual preview

실측 자유공간 상태를 재현하는 별도 URDF와 ROS 표시 노드이다. 실행하면 모터각 슬라이더 창이 함께 열린다. 기존
URDF, MoveIt 및 하드웨어 연결을 보존한다. 접촉 동역학 모델은 아니다.

| 모터각(rad) | 패드 측정점 사이(mm) | 너트 깊이(mm) |
|---:|---:|---:|
| -12 | 19 | 10 |
| 0 | 38 | 6 |
| 12 | 53 | 2 |

구간별 선형 보간하며 범위 밖·NaN 입력은 거부한다. 완전 닫힘 0mm 가정은
제거했다. 모터각과 URDF 링크 회전각은 다르다. 깊이는 스크류 위에서 아래로
양수, URDF g2 변위는 위쪽 양수(m)이다. 실측 이동은24rad에8mm이다.

## 구현과 가정

좌우 STL을 각각7개 연결 성분으로 나눴다. 두 helper 판은 같은 강체로
묶어 각 손가락에7개 회전관절, 너트에1개 직동관절을 둔다. 원본 CAD
회전값을 복원했다. 기존 약±0.103rad 추가 회전으로 생겼던 핀 어긋남은
새 모델에 사용하지 않는다. 실제 구멍에서 구한 고정축은 base 핀과,
helper 위쪽 구멍은 너트 구멍과 CAD 내보내기 정밀도에서 일치한다.
URDF 트리에서 끊은 폐루프 연결2개는 매 입력마다 계산하고 검사한다.

**너트 깊이와 간격 모두 실측 입력이며, 너트만으로 간격을 예측한 결과가
아니다.** 사진은 동작 예시이며 추가 각도/치수 실측으로 취급하지 않는다.
기본 동작은 안쪽 두 링크를 일직선으로 유지하고 양쪽 패드를 평행하게 유지한다.
원본 STL에서는 패드와 두 핀 사이 연결대가 한 부품이다. 두 조건을 함께
구현하기 위해 미리보기에서 이 부품의 목 부분을 안쪽 패드 핀에서 4mm 위로
분할하고, 해당 핀을 축으로 패드 보정 회전관절(wrist)을 추가했다. 절단면은
막았으며 원본 STL은 보존했다. 총 15개 구동 표시 관절을 계산한다.

**추가한 wrist는 실물에서 확인된 관절이 아닌 가상 설계이다.** 원본 일체형
부품 그대로의 동작이라고 해석하면 안 된다. 패드와 연결대는 이 핀을
기준으로 상대 회전하므로 목 부분의 접합 형상은 실제 제작 설계가 필요하다.
실물 관절 구조와 패드 각도 검증 후 실제 모델에 반영해야 한다.
기존 비교 모드는 `model.pose(angle, constraint='parallel_pads')`와
`model.pose(angle, constraint='straight_inner')`로 사용할 수 있다.
사진은 관절 움직임의 참고 자료이며 추가 실측값으로 사용하지 않는다.

간격 기준은 원본 패드 끝3mm 구간의 내측 재료점이다. 실물 측정 위치와
같은지 확인해야 한다. source CAD 전체 내측 최소 간격은 약39.24mm,
선택한 끝점 간격은43.92mm이다. CAD shaft 상단부터 너트 윗면까지
7.629066mm를 기준으로 사용한다. 사용자 측정이 **너트 윗면** 기준이라는
가정이며 실물 원점은 검증되지 않았다. CAD 자세에 실측 상태를 단정할
수 없다. geometry.json에 축, 원본SHA256, 실측 사본과 가정을 기록했다.

effort/velocity는 URDF 파서용으로 실물 제한값이 아니다. 분할 링크의
질량·관성, 충돌·접촉 시뮬레이션, 실물 commissioning을 제공하지 않는다.

## 실행

ROS Jazzy를 source한 워크스페이스에서:

```bash
colcon build --packages-select pallet pallet_physical_moveit_config
source install/setup.bash
ros2 launch pallet adaptive_gripper_preview.launch.py motor_rad:=0
ros2 topic pub --once /adaptive_gripper_preview/motor_angle std_msgs/msg/Float64 '{data: 12.0}'
```

슬라이더 범위는 -12~+12rad(0.01rad 단위)이며 너트와 양쪽 손가락을 함께
움직인다. 창에 모터각, 패드 간격, 너트 깊이가 표시된다.

출력은 /adaptive_gripper_preview/joint_states이고 TF에는 adaptive_preview/
접두어가 붙는다. 실제 모터 통신은 하지 않는다. 오프라인 확인:

```bash
ros2 run pallet adaptive_gripper_preview.py --offline --motor-rad -12
```

소스 pallet 패키지에서 원본/실측 변경 후 생성·검증:

```bash
python3 scripts/generate_adaptive_gripper.py
python3 scripts/render_adaptive_gripper.py --output /tmp/adaptive_states.png
python3 -m pytest -q test/test_adaptive_gripper.py
```

adaptive_gripper_preview.urdf는 독립 그리퍼, pallet_adaptive_preview.urdf는
전체 로봇 형태이다. 후자는 기존 g_r/g_l 인터페이스와 다르므로 기존
MoveIt 설정에 바로 넣으면 안 된다. 실측 원본은 physical 설정의
config/gripper_calibration.json이다. 변경 시 생성기를 다시 실행한다.

## 실물 모델 확정에 필요한 자료

너트 측정면과 패드 측정 위치를 맞추고, 같은3개 상태의 정확한 정면 사진
또는 패드 기울기·안쪽 팔꿈치 각도를 추가 측정한다. 이 자료로 패드 평행 가정의 실제 동작 범위와 원점을 확인한 뒤
제어 인터페이스에 통합한다. 물체 접촉 시
adaptive 동작에는 별도 스프링/접촉 검증이 필요하다.

## 재사용 URDF 출력

- `urdf/adaptive_gripper.urdf`: 그리퍼 단독, 루트 `gripper`.
- `urdf/pallet_adaptive.urdf`: 전체 로봇에 같은 그리퍼 구조를 적용.

승인된 직선 안쪽 링크·평행 패드 구성을 사용한다. RViz 전용 카메라 프레임은
제외했다. 메시 경로는 `package://pallet/` 기준이므로 pallet 패키지의 메시가
함께 필요하다. 미리보기와 같은 15개 관절 이름과 좌표계를 사용한다.
URDF 자체는 비선형 폐루프를 강제하지 않으므로 슬라이더 노드 또는
`adaptive_gripper_core.py`가 계산한 관절값을 함께 공급해야 한다.
원본 실물용 URDF를 덮어쓰거나 MoveIt 제어 설정을 자동 교체하지 않는다.
