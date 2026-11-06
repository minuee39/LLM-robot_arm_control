# move_group 종료 오류와 로컬 수정

작성일: 2026-10-27
기준 workspace: /home/minwoo/Development/LLM/ros2_ur_ws
환경: ROS 2 Jazzy, MoveIt 2.12.4.

## 문제와 근거

정상 계획·실행 후 SIGINT 종료 시 간헐적으로 exit code -11/SIGSEGV 발생.
두 위치에서 관측했다.

- MoveItCpp → TrajectoryExecutionManager → controller node → CallbackGroup 소멸.
- 컨트롤러 노드를 먼저 정리하는 시험 수정 후에는 main node → CallbackGroup 소멸.

컨트롤러 및 capability 플러그인에서 생성한 action/service 콜백의 weak_ptr
제어블록을 ROS 노드가 보유한다. 플러그인 unload 이후 이 참조를 정리하면
이미 해제된 플러그인 코드에 접근할 수 있다. 객체 수명과 코드 매핑 수명을
함께 유지해야 한다. destructor 순서만 수정한 시험안은 제거했다.

공식 소스 참조:
- https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp
- https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/move_group/src/move_group.cpp

## 최종 수정

src/plugin_code_lifetime.cpp가 아래 두 라이브러리에 RTLD_NODELETE를 적용한다.

- libmoveit_simple_controller_manager.so.2.12.4
- libmoveit_move_group_default_capabilities.so.2.12.4

이는 플러그인 코드 매핑을 프로세스 종료까지 유지한다. ROS 노드/컨트롤러 객체와
스레드는 기존 종료·소멸 절차를 따른다. dlopen 핸들은 dlclose로 짝을 맞춘다.
프로세스 실행 중 해당 플러그인 코드 매핑을 유지하는 메모리 비용이 있다.
라이브러리 로드 실패는 명시적 오류로 중단한다. 오류 로그나 비정상 종료를 숨기지 않는다.

로컬 libpallet_moveit_shutdown_fix.so를 이 패키지의 move_group 프로세스에만
LD_PRELOAD로 적용한다. demo는 move_group.launch.py의 scoped GroupAction을,
feedback.launch.py는 해당 Node의 additional_env를 사용한다.
/opt/ros 파일이나 시스템 패키지는 수정하지 않았다.

CMake와 launch 양쪽에서 MoveIt 2.12.4 버전을 확인한다. 버전이 바뀌면
그 버전에서 원래 오류가 해결됐는지 확인하고 이 가드를 제거하거나 재검증해야 한다.
이 로컬 수정은 upstream 전체 버전/모든 종류의 플러그인에 대한 보장이 아니다.

## 검증과 사용

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pallet_physical_moveit_config --symlink-install
source install/setup.bash
ros2 run pallet_physical_moveit_config verify_mock_execution.py
```

기존 demo 및 feedback launch 명령은 동일하다. 기동 로그의
[pallet_shutdown_fix] 메시지로 적용을 확인할 수 있다.

검증 도구는 /proc의 실제 move_group 라이브러리 매핑, 계획, 팔/그리퍼 실행,
컨트롤러 취소/유지, move_group 정상 종료를 확인한다.
SIGSEGV/종료 timeout/정상 종료 미확인 중 하나라도 발생하면 실패로 보고한다.
검증 도구는 launcher에 SIGINT를 한 번 전달하며 launcher가 자식 종료를 담당한다.
프로세스 그룹 전체와 launcher에 신호가 중복 전달되는 테스트 오류도 수정했다.
피드백 노드 pallet_motor_joint_state.py는 SIGINT에서 이미 종료된 컨텍스트에
shutdown을 재호출하던 RCLError를 try_shutdown으로 수정했다.
상세 결과는 출력되는 /tmp/pallet_mock_verify_*/result.json 및 launch.log에 저장된다.
실물 모터에는 연결하지 않는다.

## 최종 검증 기록

중단 전 비교 시험에서는 수정본을 적용하지 않은 feedback launch가 동일한
단일 SIGINT 조건에서 CallbackGroup 소멸 SIGSEGV를 재현했다.
컨트롤러 소멸 순서만 바꾼 초기 시험안은 main node 종료 오류가 남아 폐기했다.

재개 후 최종 수정본으로 수행한 순차 검증:

- 모의 계획·팔 실행·그리퍼 실행·취소/유지·move_group 정상 종료: 3회 연속 통과.
- 각 회차에서 실제 move_group 메모리 맵의 로컬 보정 라이브러리 로드를 확인.
- 관련 pytest 14개 통과. Python 문법 검사 및 pallet/설정 패키지 빌드 통과.
- 영구 로컬 검증 기록: workspace의 validation/shutdown_20261027/results.json 및 run_*.log.

조사 중 병렬 검증에서는 35초 내 기동 서비스가 준비되지 않는 별도 실패도 있었다.
해당 회차는 성공 횟수에서 제외했다. 이 기동 timeout의 원인은 확정하지 않았으며,
최종 순차 3회에서는 재현되지 않았다. 종료 SIGSEGV 검증과 별개로 기록한다.

이 결과는 이 패키지의 headless 모의 환경 범위이며 실물 모터 구동은 수행하지 않았다.

읽기 전용 feedback launch도 추가 검증 완료:
- 수동 모터각이 MoveIt 현재 자세에 일치.
- move_group와 pallet_motor_joint_state.py 모두 정상 종료.
- SIGSEGV 없음.
- 기록: validation/shutdown_20261027/feedback_result.json 및 feedback.log.
