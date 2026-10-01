# Pallet MTC → Isaac 직접 pose 실행 경로

## 작성일과 작업 목적

- 작성일: 2026-09-25
- 목적: Pallet 작업대·물체 좌표를 일치시키고 Isaac, MoveGroup, Planning Scene의 순차 기동을 한 launch로 확인한다. 이 기록은 Pick & Place 성공 기록이 아니다.

## 최종 기준 workspace

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
```

- 작업 브랜치: `pallet_robot_ver`; 기존 혼합 작업 트리의 다른 변경은 보존했다.
- 이 작업에서 커밋·푸시하거나 Notion에 기록하지 않았다.

## 현재 주요 패키지와 파일

- `src/pallet_gripper_moveit_config/config/workcell.yaml`: Isaac과 MoveIt 작업대 및 물체의 공통 사양.
- `src/pallet_gripper_moveit_config/scripts/isaac_sim.py`: 작업대, 기본 컵, 선택적 블록 생성.
- `src/pallet_gripper_moveit_config/scripts/workcell_scene.py`: MoveIt 작업대 및 선택적 컵 등록.
- `src/pallet_gripper_moveit_config/launch/static_virtual_joint_tfs.launch.py`: 공통 작업대 높이로 `world → base_footprint` 고정 변환.
- `src/mtc_tutorial/launch/pallet_mtc_isaac_demo.launch.py`: Isaac 준비, MoveGroup 준비, 장면 등록 순서 제어.
- `src/mtc_tutorial/config/robot_profiles.yaml`: MTC의 Pallet 테이블·블록·pick/place 좌표.

## 작업 중 발생한 문제 → 원인 → 해결 → 검증

1. **작업대 불일치:** MTC는 `1.80 × 0.75 × 0.75 m`, Isaac/MoveIt은 `0.60 × 0.80 × 0.72 m`를 사용했다. 각 구현에 값이 중복된 것이 원인이었다. `workcell.yaml`을 Isaac/MoveIt/TF의 공통 기준으로 연결하고 MTC 프로파일과 일치시키는 테스트를 추가했다. 5개 프로파일 테스트가 통과했고, Isaac 로그에서 로봇 베이스 높이 `0.75 m`를 확인했다.
2. **MTC 대상 물체 부재:** 이전 Isaac 작업 셀에는 컵만 있어서 MTC의 `red_block`과 물리 장면이 달랐다. MTC 실행 경로에 `--calibration-block --no-cup`을 적용했다. Isaac 준비 로그와 MoveIt 작업대 등록 로그를 확인했다. 물체 파지·이동은 아직 검증되지 않았다.
3. **기동 순서 불명확:** 이전 MTC launch는 Isaac을 시작하지 않고 MoveGroup을 즉시 시작했다. Isaac 실측 joint feedback과 MoveGroup Planning Scene 준비 게이트를 거쳐 장면을 등록하도록 변경했다. Headless 실행에서 두 게이트 및 `Pallet workcell ready for direct-pose MTC execution.` 메시지를 확인했다. 준비 실패 시 launch를 종료한다.
4. **MTC 계획 미완료:** 블록만 둔 직접 pose 시도에서 grasp 후보 4개가 생성됐지만 `approach object`와 place pose IK에서 해를 얻지 못했다. OMPL 계획 요청이 5초·10초 제한에 걸렸고, 약 40초 후 MTC 프로세스가 실패 종료했다. 컵을 둔 첫 실행도 약 18초 후 계획에 실패했다. 중복 MTC 테이블을 임시로 제거한 진단 실행에서도 시간 초과가 계속되어 중단했다. 따라서 컵 또는 중복 테이블만을 원인으로 확정할 수 없다. 접근 경로·파지 자세·충돌 장면을 다음 단계에서 분리 진단해야 한다.

## 최종 구조와 설계 결정

- Isaac `world` 기준 작업대 중심 XY는 `(-0.43, 0.0) m`, 크기는 `(1.80, 0.75, 0.75) m`다. 로봇 베이스는 테이블 상면 `z=0.75 m`에 둔다.
- 블록 크기는 `(0.030, 0.030, 0.050) m`, 시작 중심 XY는 로봇 `base_link` 기준 `(-0.30, 0.30) m`다. Isaac 생성 시 낙하 안정화를 위해 테이블 위에 3 mm 여유를 둔다. MTC 목표 중심 Z는 `0.025 m`다.
- MTC 경로는 기존 컵을 생성·등록하지 않는다. 기존 컵 데모를 위한 기본 Isaac 실행에서는 컵을 유지한다.
- 준비 완료 메시지는 장면 기동 성공을 뜻하며 MTC 계획 또는 실행 성공을 뜻하지 않는다.

## 실행 순서

터미널 1에서 빌드 후 작업 셀을 시작한다. GUI가 필요하면 `headless:=false use_rviz:=true`로 바꾼다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pallet_gripper_moveit_config mtc_tutorial --symlink-install
source install/setup.bash
ros2 launch mtc_tutorial pallet_mtc_isaac_demo.launch.py headless:=true use_rviz:=false startup_timeout:=180
```

터미널 1에서 `Pallet workcell ready for direct-pose MTC execution.`을 확인한 뒤, 터미널 2에서 현재 계획 진단을 재현한다. **현재 이 명령의 Pick & Place는 계획 단계에서 실패한다.**

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py robot_profile:=pallet_gripper execution_mode:=standard
```

## 검증 명령과 결과

```bash
cd /home/minwoo/Development/LLM
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s ros2_ur_ws/src/mtc_tutorial/test -p 'test_*.py'
python3 -m py_compile ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/isaac_sim.py ros2_ur_ws/src/pallet_gripper_moveit_config/scripts/workcell_scene.py ros2_ur_ws/src/pallet_gripper_moveit_config/launch/static_virtual_joint_tfs.launch.py ros2_ur_ws/src/mtc_tutorial/launch/pallet_mtc_isaac_demo.launch.py
```

- MTC 프로파일 테스트: 5개 통과.
- Python 문법 검사, 두 패키지 `colcon build --symlink-install`, launch 인자 조회: 통과.
- Headless Isaac → 피드백 준비 → MoveGroup → 작업대 등록: 통과. `ExecuteTaskSolutionCapability` 로드 확인.
- 실제 MTC 계획·실행: 계획 실패. 블록 이동, 파지 안정성, 반복 성공률은 확인하지 못했다.
- 실제 로봇·모터 구동: 수행하지 않았다.

## 남은 주의사항

- `ros2 launch` 래퍼 종료 코드가 0이어도 내부 MTC 프로세스는 실패할 수 있다. 프로세스 종료 코드와 단계별 로그를 확인한다.
- Isaac의 블록이 물리적으로 안정화된 최종 자세와 MoveIt/MTC 충돌 물체 자세가 맞는지 별도 측정이 필요하다.
- 중단한 진단 실행에서는 ROS 프로세스 일부가 SIGTERM에 응답하지 않아 launch가 SIGKILL로 정리했다. 작업 종료 후 관련 프로세스가 남지 않은 것은 확인했다.

## 다음 작업 로드맵

- [x] 공통 작업대·물체 사양 연결
- [x] Isaac/MoveGroup/Planning Scene 순차 기동 및 실패 게이트
- [x] Pallet 직접 pose MTC 계획을 실제로 시도하고 실패 단계 기록
- [ ] 접근 경로, grasp 후보, place IK 및 충돌 물체별 실패 원인 분리
- [ ] 직접 pose Pick & Place 1회 성공과 실측 관절/물체 상태 검증
- [ ] 반복 횟수·성공률·실패 로그를 남기는 회귀 실행

## 후속 진단: 계획과 실행 분리

- 모의 Pallet 컨트롤러에서 동일 프로파일의 9개 구간 전체 계획이 17.062초 만에 생성됐다. 이는 Isaac에서 관찰한 실패가 해당 목표 자세의 절대적인 IK 불가능만으로 설명되지는 않음을 보여준다. 모의 실행은 완료 전에 중단했으므로 물체 이동 성공 근거는 아니다.
- 동일한 `workcell_scene.py --no-cup` 작업대를 모의 MoveIt에 추가한 뒤에도 36.396초 만에 계획 해가 생성됐다. 이 실행은 `max_solution_cost:=0`으로 구동을 차단했다. 따라서 외부 작업대 하나만으로 실패가 재현되지는 않았다.
- MTC 계획 실패 시 단계별 상태와 실패 설명을 출력하도록 했고, `planning_only:=true`를 추가했다. 앞으로 다음 명령은 해가 생성돼도 컨트롤러에 궤적을 보내지 않는다.
- 변경 후 `mtc_tutorial` 빌드, 프로파일 테스트 5개, launch 인자 조회가 통과했다. 모의 Pallet 장면에서 `planning_only:=true` 실행은 9개 구간 계획을 생성하고 `Planning-only mode: execution skipped`를 출력한 뒤 정상 종료했다.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py robot_profile:=pallet_gripper planning_only:=true
```

- Isaac을 다시 띄운 후 추가 진단 실행에서는 Isaac 프로세스가 높은 CPU를 사용했고 MoveGroup 준비 게이트와 `/joint_states` 단일 샘플 조회가 완료되지 않았다. 이 실행은 중단했다. 해당 현상과 앞선 MTC 계획 실패의 인과관계는 확인되지 않았다.
- 다음에는 Isaac 실측 `/joint_states`와 모의 홈 관절 상태를 비교하고, MTC 실패 설명의 단계별 충돌·IK 이유를 확보해야 한다.
