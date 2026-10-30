# MoveIt 6축 USB 직렬 실행 경로

작성일: 2026-10-27. 실제 관절과 RViz 방향 일치는 사용자가 확인했다.
다음 단계로 MoveIt의 실행 목표를 USB 직렬 PP 제어에 전달하는 브리지를 추가했다.
이번 검증은 DRY RUN이며 실물 모터를 구동하지 않았다.

## 구조와 설정

`MoveIt → /pallet_arm_controller/follow_joint_trajectory → 브리지 → PP 목표 → joint_states`

- 실행 파일: `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/scripts/pallet_serial_trajectory_bridge.py`
- 통신·검증: 같은 디렉터리의 `serial_trajectory_core.py`
- 설정: `/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/config/serial_trajectory.yaml`
- 대상은 J1~J6 / ID1~6이다. 기존 hand_guiding.yaml의 방향·영점을 사용한다.
- 속도와 가속도는 각각 0.1 rad/s, 0.1 rad/s²이다. 모터 전류 제한 레지스터는 변경하지 않는다.
- USB 포트는 한 번만 열어 공유한다. 실행 중 취소를 처리하며 중복 목표는 거부한다.
- 오래된 피드백, 경로 오차, 통신 실패, 설정된 전류 초과 시 목표를 중단하고 정지를 확인한다.
- 그리퍼는 화면 표시용 영점이다. ID7 개폐 실행은 아직 연결하지 않았다.

## 실행 순서

기존 feedback/demo launch를 종료한 후 새 터미널에서 실행한다.
동시에 실행하면 joint_states와 MoveIt 노드가 중복될 수 있다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pallet_physical_moveit_config serial_bridge.launch.py
```

기본값 `dry_run:=true`에서는 USB를 열지 않는다. RViz의 pallet_arm 그룹에서
현재 자세 근처의 작은 목표를 설정하고 Plan & Execute하면 브리지의 가상 피드백이 갱신된다.
초기 가상 관절각은 모두 0이다. 기본 드라이런 설정은 serial_trajectory_dry_run.yaml이며,
URDF 범위 안에서 시작점 대비 최대 2π rad, 최대 600초 궤적을 허용한다.
실기 시험용 serial_trajectory.yaml은 0.1 rad / 30초 제한을 그대로 유지한다.
MoveIt 계획 한계도 0.1 rad/s, 0.1 rad/s²로 맞췄다. RViz scaling 0.1은 이 한계의 10%다.

LIVE는 `dry_run:=false`와 별도로 설정의 commissioned, verified_joints, 축별
current_trip_a가 필요하다. 기본 설정은 미확인 값을 임의로 채우지 않아 LIVE를 차단한다.
current_trip_a는 모터 전류 제한 변경값이 아니라 소프트웨어 측정 전류 중단 기준이다.
6축 실제 통신 주기와 watchdog 여유, 부하 상태의 추종·정지는 아직 검증하지 않았다.

## 검증 결과

- 관련 단위 테스트 23개 통과, 두 패키지 빌드 통과.
- MoveIt이 생성한 9점 궤적을 브리지로 실행하여 성공 반환 확인.
- J1 목표 0.01 rad, 최종 가상 피드백 0.0100813 rad.
- 실행 중 취소 처리와 실행 중 두 번째 목표 거부 확인.
- 이 통합 실행의 종료에서 SIGSEGV 없음.
- 결과: `/home/minwoo/Development/LLM/ros2_ur_ws/validation/serial_bridge_20261027/result.json`
- launch 로그: 같은 디렉터리의 `launch.log`.

드라이런은 ROS action/계획/피드백 연결 검증이다. 실제 USB 전송 시간과 모터 추종 성능을
입증하지 않는다. 다음 작업은 실기용 축별 중단 기준을 설정하고 이 브리지의 실제
6축 피드백 주기 및 제한된 작은 궤적을 확인하는 것이다.

2026-10-27 실행 실패 수정: 기존 메시지는 최대 30초 초과와 시간 역전을 같은 오류로 표시했다.
이제 초과 시간과 한계를 구체적으로 기록한다. 가상 실행용 범위와 실기 시험 범위를 분리했고,
60초 가상 목표 수락 및 취소, MoveIt 19점 계획·실행 성공을 별도 통합 검증했다.
관련 단위 테스트 24개 통과. 결과: validation/serial_bridge_time_fix_20261027/result.json.

## 실물 연결 읽기 전용 모드

기존 feedback/드라이런 launch와 Motor Studio를 종료한 다음 실행한다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pallet_physical_moveit_config serial_bridge.launch.py dry_run:=false observe_only:=true
```

이 모드에서는 실물 팔을 손으로 움직이면 RViz가 따라간다. Execute는 거부한다.
ID1~6의 각도·전류·속도를 순차 읽기만 한다. Enable/STOP/위치 목표/설정 쓰기가 없다.
팔의 방향 일치를 다시 검증하려는 단계가 아니라 LIVE와 같은 18회 읽기의 주기를 측정한다.
다른 터미널에서 상태를 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/minwoo/Development/LLM/ros2_ur_ws/install/setup.bash
ros2 topic echo /pallet_serial_bridge/status
```

`read_cycle_ms`는 전체 읽기 소요 시간이다. `cycle_overrun=true`이면 현재 100ms 주기를
충족하지 못하므로 실기 제어 전에 통신 방식을 조정해야 한다. false여도 위치 명령 쓰기를
추가한 LIVE 전체 주기나 부하 중 추종을 검증한 것은 아니다. 통신 오류는 FAULT_LATCHED로
표시하고 마지막 값을 새 실측처럼 재발행하지 않는다. 원인 해소 후 재시작한다.

소프트웨어 검증: 관련 테스트 28개 및 두 패키지 빌드 통과. 읽기 18회의 순서와 종료 시
추가 요청이 없는 것을 가짜 reader로 검증했다. 실제 USB 읽기와 시간 측정은 미실시다.

## 실물 전류 제한 변경: 2026-10-27

사용자가 모터에 저장된 전류 제한 자체를 기존 값의 절반으로 변경하도록 지시했다.
0x7018을 먼저 읽고 ID1~6에 각각 8, 28.5, 8, 5.5, 5.5, 5.5 A를 쓴 뒤
모든 축을 재읽어 일치 확인했다. 원래 값은 각각 16, 57, 16, 11, 11, 11 A였다.
ID7은 변경하지 않았다. 토크 활성화, STOP, 위치 명령은 보내지 않았다.
전원 재투입 후 설정 유지 여부는 미확인이다. Flash 저장 명령은 보내지 않았다.
원본·목표·검증값: measurements/position/arm_half_current_limits_20261027.json.
도구: pallet_current_limits.py. 같은 snapshot을 다시 적용해도 중복 반감하지 않는다.
이 설정 변경은 LIVE commissioning 완료를 뜻하지 않으며 기존 브리지는 현재 제한을 보존한다.

## 검증한 실물 MoveIt 실행 (2026-10-27)

별도 serial_trajectory_live_tested.yaml을 추가했다. commissioning 범위는 외부 지지 상태의
6축 +/-0.01rad와 MoveIt +/-0.005rad 궤적이다. 부하 상태나 전체 가동범위 검증을 뜻하지 않는다.
기본 launch는 계속 DRY RUN이다. LIVE 설정은6축 기존 전류 제한8/28.5/8/5.5/5.5/5.5A가
일치해야 enable한다. 추가 측정전류 trip은 명시적으로 비활성화했지만 전류를 계속 읽는다.
속도/가속도0.1, raw속도감시0.2의2연속 확인/0.4즉시중단/위치차분0.2즉시중단,
시작점 대비 최대 이동0.02rad, 도달오차0.001rad. ID7은 제어하지 않는다.
실측위치가 이동범위를 벗어나면 목표값이 범위 안에 있어도 중단한다.
MoveIt 실행 시간 여유3초는 직렬 PP 준비와 실제 수렴 시간을 위한 것이며 별도 목표/통신 제한은 유지한다.

실행 중인 Motor Studio와 feedback/드라이런 launch를 먼저 종료한다.
이 명령으로 실행하면 Plan은 계획만 수행하고 Execute는 실제6축 모터를 움직인다.
STOP과 launch 종료 시 토크가 해제될 수 있으므로 확인된 외부 지지를 유지한다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pallet_physical_moveit_config serial_bridge.launch.py dry_run:=false config_file:=/home/minwoo/Development/LLM/ros2_ur_ws/src/pallet/config/serial_trajectory_live_tested.yaml
```

RViz 그룹은 pallet_arm, 현재각 기준 작은 목표를 사용한다. URDF와 충돌 검사는 적용하지만
실물 환경 전체가 모델링됐다는 뜻은 아니다. 현재 실험 범위 밖 목표는 거부한다.

RViz Stop에 해당하는 검증된 정지 이벤트:

```bash
ros2 topic pub --once /trajectory_execution_event std_msgs/msg/String "{data: stop}"
```

검증 결과:
- MoveIt이 계획한16점 실물6축 +0.005rad 궤적: SUCCEEDED / MoveIt error1.
- RViz 방식 stop 이벤트: 실행 중단 확인. MoveIt statusABORTED/errorPREEMPTED(-7).
- 브리지 FollowJointTrajectory 직접 cancel: 요청 수락, statusCANCELED(5), STOP확인.
- 상위 ExecuteTrajectory action cancel은 이번 환경에서 실행 완료 후 처리돼 취소로 인정되지 않았다.
  이 경로는 즉시정지 수단으로 사용하지 않는다. 검증된 stop 이벤트를 사용한다.
- 두 통합 시험에서 SIGSEGV 없이 정상 종료. 관련 테스트35개와 두 패키지 빌드 통과.
- validation/serial_bridge_live_20261027/result.json: 실행 성공과 상위cancel 한계 원본 기록.
- validation/serial_bridge_stop_20261027/result.json: stop 이벤트/브리지cancel 성공 기록.

그리퍼와 실측 TCP는 여전히 미완성이므로 이번 결과는 pick and place 완료가 아니다.

## 대기 중 J6 속도 fault 수정

사용자 launch의 python3_43580_1793104779938.log에서 J6 Raw velocity trip 확인.
동일 move_group 로그에는 Execute 요청이 없어 실행 전 idle feedback 경로에서 발생한 것을 확인했다.
당시 구현은 idle sample에도 실행 속도 guard를 적용해 fault latch와 STOP을 수행했다.
해당 원시값은 기존 로그에 없어0.4 단발/0.2 연속 중 어느 조건인지 확정하지 못한다.

idle에서는 속도값과 위치차분을 기록하되 실행 속도 위반으로 latch하지 않도록 분리했다.
prepare는 초기 자세/설정 검증을 수행하며 enable 직전부터 active speed guard를 적용한다.
active에서 기존0.2 연속/0.4 즉시/위치차분0.2 즉시 기준은 유지한다. 이동 범위/비정상값/
모터 fault/통신 오류 감시는 유지한다. stop 완료 후 idle로 돌아간다.
속도 예외에 원시값, 위치차분, 기준, 연속횟수를 추가했고 피드백 fault 후 STOP확인도 로그에 남긴다.
이미 실행 중인 프로세스는 교체되지 않으므로 Ctrl+C 후 같은 LIVE 명령으로 재시작해야 한다.
관련 테스트37개 통과 및 pallet 빌드 통과. 이번 수정 후 실물 재구동은 수행하지 않았다.

## 단발 전류 읽기 timeout 처리 수정

사용자 로그: Feedback timeout:0x701a 후 STOP confirmed after feedback fault.
기존 sample은 각 파라미터 읽기15ms가 한 번 초과하면 즉시 latch했다. 이번 로그에는
대상 모터ID가 없어 특정 축이나 USB/모터 문제로 단정할 수 없다.
6축 SerialBackend의 위치/전류/속도 읽기에 timeout만1회 추가 읽기를 허용했다.
최대2회 요청, sample 전체 deadline 유지. LIVE에서는6축 목표쓰기18ms를100ms 주기에서
예약하므로 읽기 예산은82ms다. 최종 제어주기 초과 검사와250ms watchdog은 유지한다.
재읽기 성공한 새 값만 사용하며 마지막 값을 대신 넣지 않는다. 모터 fault, 비정상값,
읽기 거절은 재시도하지 않는다. 최종 실패에는 축명과 레지스터/횟수를 기록한다.
status의 feedback_retries로 마지막 완전 샘플의 재읽기 횟수를 확인할 수 있다.
관련 테스트38개와 pallet 빌드 통과. 재읽기 복구/2회 실패/전체deadline/모터fault 즉시전파
검증. 이번 수정 후 실제 USB 지연 재현은 미실시이며 실행 중 노드는 재시작해야 적용된다.

## 실기 계획 시간 초과 진단 개선

사용자 로그는30.1초 궤적 점에서30초 한계 초과를 표시했으므로 실제 전체 궤적이30.1초라는
뜻은 아니었다. 이전 display_planned_path는 volatile QoS여서 사후 확보하지 못했다.
실기 시험 프로파일 max_goal_duration_s를60초로 변경했다. max_joint_travel_rad0.02,
전류/속도/가속도/오차/통신 제한은 유지했다. 기본 미검증 프로파일30초는 보존했다.
계획의 모든 점을 기준으로 첫 점 대비 관절별 최대 이동 범위를 먼저 검증한다.
범위 초과 시 축명·실제 이동량·허용량을 표시하고, 시간 초과 시 전체 최대 시간을 표시한다.
실행 시 현재 실측점 대비 이동 범위 재검사는 계속 수행한다.
관련 테스트39개 및 pallet 빌드 통과.30.1초 작은 목표 허용,61초 거부, 큰 이동 목표 우선거부
검증. 실제 사용자 계획은 확보하지 못했고 모터 구동은 수행하지 않았다.
재시작 후 Start State current에서 작은 목표를 새로 계획한다.

## 사용자 요청: 상대 이동 제한 해제

serial_trajectory_live_tested.yaml의 max_joint_travel_rad를null로 변경했다.
계획 검증, 실행 전 현재점 대비 검사, 실행 중 실측 이동량 검사에서 시작점 대비0.02rad
제한을 해제했다. PP 원시 목표 창은 전체 URDF 관절 범위로 바인딩한다. 따라서 숨은
0.02rad 제한이 남지 않는다. 단축/별도 commissioning 시험 도구의 작은 시험범위는 보존했다.
URDF 관절범위, MoveIt 충돌검사, 시작자세 일치, 도달/추종오차, 속도/가속도/전류/통신/
watchdog/STOP 감시는 유지한다. 최대 계획시간60초도 별도 조건으로 유지한다.
실제 전체가동범위 시험은 수행하지 않았으며 이전 작은이동 실기 결과를 전체범위 검증으로
해석하지 않는다. 관련 테스트40개와 pallet 빌드 통과. 이번 변경으로 모터를 구동하지 않았다.
launch 재시작 후 이전 계획 대신현재 상태에서 새로 계획한다.

## 사용자 요청: 모터 내부 제한 초기/기본값 복원

사용자가 전류는 초기 기록값, PP속도/가속도는 제조사 기본값 복원을 선택했다.
RS00/RS05/RS06 공식 CAN2.0 UserManual260713의 PP0x7024=10rad/s,
PP0x7025=10rad/s² 기본값을 확인했다. velocity-mode acceleration0x7022의20과 구분했다.
복원 전7축STOP 확인. 팔 전류를16/57/16/11/11/11A로 복원하고 전 축 readback 일치.
ID7은 최초 전류 기록이 없고 반감 변경 대상도 아니었으므로 읽힌0.80000001A를 보존했다.
7축PP10/10은 전송 완료, write-only 모델의 적용값 readback 확인은 하지 않았다.
Enable/목표각/zero/Flash 저장 명령 없음. 전원재투입 유지 여부 미검증.
기록: measurements/position/all_motor_defaults_restored_20261027.json.
도구: pallet_restore_motor_defaults.py, 기본OFFLINE, --apply시에만STOP/정해진3레지스터 접근.

LIVE 설정의 expected_current_limit_a도 복원값과 맞췄다. motor_profile_policy를
preserve_existing으로 바꿔 다음Execute때PP10/10을0.1로 덮어쓰지 않는다.
MoveIt 계획 속도/가속도0.1 및 raw/위치차분 속도 감시는 유지한다. 이는 모터 내부 기본값과
별개의 상위 계획/감시 설정이다. 별도 이전 단축 실험 프로파일은 자동변경하지 않았다.
관련 테스트50개 및 pallet빌드 통과. 이 복원값으로 실제 궤적추종 검증은 아직 미실시.

## 사용자 요청: MoveIt 팔 속도/가속도 상한1.0

J1~J6의 joint_limits max_velocity를1rad/s, max_acceleration을1rad/s²로 변경했다.
LIVE/DRY RUN 브리지의 궤적 검증 상한도1/1로 일치시켰다. 그리퍼 MoveIt관절g_r의
0.1/0.1 제한은 유지했다. LIVE 실행 속도감시는1.2rad/s2연속, raw2.4즉시,
위치차분1.2즉시로 새 상한에 맞췄다. 위치추종오차0.01/최종오차0.001은 유지한다.
모터 PP10/10 preserve_existing 및 전류 초기값 복원은 그대로다.
RViz 배율0.1이면 유효 상한0.1/0.1이고 배율1이면1/1이다. 기본배율0.1은 유지했다.
별도7축토크유지도구는 .1/.1 및 속도감시 .2를 명시적으로 유지해 MoveIt변경이 전파되지 않게 했다.
이 속도로 실물 구동하지 않았다. 소프트웨어 검사와 두패키지 빌드만 수행했다.
