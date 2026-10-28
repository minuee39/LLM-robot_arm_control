# Pallet 통합 작업 트리 기준선과 변경 소유권

## 작성일과 작업 목적

- 작성일: 2026-09-25
- 목적: Pallet MTC → Isaac 반복 검증을 시작하기 전에 혼합 작업 트리의 출처와 기능별 소유권을 고정한다. 이 문서는 커밋 또는 시뮬레이션 성공 기록이 아니다.

## 최종 기준 workspace

```bash
cd /home/minwoo/Development/LLM
```

- 조사 당시 브랜치: `pallet_robot_ver` (`origin/pallet_robot_ver` 추적)
- 기준 HEAD: `7202d9a608d111b772cdbd060e2648dc673788ef`
- 조사 당시 변경: tracked 40개, untracked 263개. Untracked 중 `ros2_ur_ws/backups/` 177개, 그 외 86개.
- 조사 당시 tracked diff의 SHA-256 (`git diff --binary | sha256sum`): `e9e9cbbb2f2fde8062aaaaedbb463fc4e952e4c06ca6390f9b131775f565d942`. 이 값은 이 문서와 `.gitignore` 수정 **전** 상태를 식별한다. Untracked 내용은 이 해시에 포함되지 않는다.
- 원본 작업 트리는 체크아웃·스테이징·커밋 없이 보존했다. `ros2_ur_ws/backups/`는 Git에서 무시하도록 설정했지만 로컬 파일은 그대로 둔다.

## 현재 주요 패키지와 파일 및 소유 브랜치

| 소유 브랜치 | 현재 변경 범위 | 반영 전 확인 사항 |
| --- | --- | --- |
| `feature/mtc-grasp-stability` | `ros2_ur_ws/src/mtc_tutorial/`의 C++, launch, `config/robot_profiles.yaml`, 테스트; `ur10e/scripts/run_mtc_isaac_*.sh`, MoveIt/Isaac bridge 실행 스크립트; UR10e controller/OMPL 실행 설정 | Pallet·UR10e 두 프로파일과 `standard`/`blended` 경로를 각각 검증 |
| `feature/pallet-robot-description` | `ros2_ur_ws/src/pallet/`의 URDF·mesh·SRDF·표시 launch·질량 감사; `pallet_gripper_description/`, `pallet_gripper/`, `pallet_moveit_config/`, `pallet_gripper_moveit_config/`의 모델·MoveIt 설정·Isaac 작업대 | 사용 중인 canonical mesh/source만 명시적으로 선택; URDF, SRDF, Isaac/MoveIt 좌표 정합 확인 |
| `feature/robstride-can-control` | `robstride_read_angles.py`, `pallet/scripts/robstride_trajectory_core.py`, `pallet/scripts/pallet_robstride_trajectory_bridge.py`, `pallet/config/robstride_hardware.yaml`, `pallet_moveit_config/launch/robstride_demo.launch.py`, 관련 테스트와 `docs/robstride-moveit-execution.md` | `pallet/CMakeLists.txt`·`package.xml`의 RobStride 설치 hunk를 함께 확인; 하드웨어 검증과 dry-run을 구분 |
| `feature/vision-depth-stability` | `ur10e/scripts/run_yolo_camera_node.sh`, `run_sim_camera_publisher.sh`, `run_sim_camera_rviz.sh`, `run_pick_place_with_camera.sh`; `ros2_ur_ws/patches/ros2_astra_camera-jazzy.patch`와 Astra 항목의 `dependencies.repos` hunk | 카메라 전용 변경과 공통 경로·ROS distro 변경을 hunk별로 확인 |
| `feature/llm-spatial-commands` | 현재 기능 변경 없음. `ur10e/scene_manager.py`에는 설명 주석만 추가됨 | 주석의 오탈자·공백을 정리하고 필요성이 확인될 때만 포함 |
| `chore/repository-maintenance` | `.gitignore`, `README.md`의 공통 경로·distro 수정, `docs/dependency-policy.md`, `ros2_ur_ws/dependencies.repos`의 공통 dependency 정리, `ros2_ur_ws/scripts/`의 공통 빌드 절차, legacy `COLCON_IGNORE`, 이 기준선 문서 | dependency 재현성과 ROS Jazzy 빌드 확인. Astra 전용 patch와 기능별 실행 설명은 해당 기능 브랜치로 분리 |

한 파일에 기능이 섞인 경우 특히 `pallet/CMakeLists.txt`, `pallet/package.xml`, `pallet_gripper_moveit_config/launch/*`, `README.md`, `ur10e/scripts/*`를 hunk 단위로 검토한다. 현재 브랜치 이름을 소유권 판단 근거로 사용하지 않는다.

## 작업 중 발견한 문제 → 원인 → 해결 → 검증

1. **혼합 변경:** 기능 다섯 분야와 공통 의존성 변경이 한 작업 트리에 있다. 원인은 Pallet 모델, MTC, Isaac, RobStride, Jazzy 이식 작업이 동일 브랜치에서 진행된 것이다. 이 문서에 소유권을 명시하고 원본 파일을 보존했다. 검증은 `git status --short --branch`, `git diff --numstat`, 신규 파일 목록 대조로 수행했다.
2. **백업 스냅샷 노출:** `ros2_ur_ws/backups/`의 177개 파일이 untracked로 보였다. 과거 전체 소스·mesh·측정 스프레드시트의 복제본이어서 커밋 대상으로 취급할 수 없다. `.gitignore`에 해당 경로를 추가했다. 파일 삭제·이동은 하지 않았다.
3. **Diff 공백 오류:** `git diff --check`에서 `ur10e/scene_manager.py`의 41, 49, 64행에 후행 공백 3건이 나왔다. 조사 전부터 있던 주석 변경이다. 기준선 감사 범위에서 코드는 수정하지 않았으며 해당 파일을 커밋하기 전 수정이 필요하다.
4. **작업대 형상 불일치:** `mtc_tutorial/config/robot_profiles.yaml`의 Pallet 테이블은 `1.80 × 0.75 × 0.75 m`이고, `pallet_gripper_moveit_config/scripts/isaac_sim.py`와 `scripts/workcell_scene.py`의 테이블은 `0.60 × 0.80 × 0.72 m`다. MoveIt scene과 Isaac scene 내부 값은 서로 같지만 MTC 프로파일과 다르다. 직접 pose 반복 검증 전에 단일 작업대 사양으로 맞춰야 한다. 여기서는 정적 설정 비교만 했고 시뮬레이션은 실행하지 않았다.

## 최종 구조와 설계 결정

- `ros2_ur_ws/backups/`, `build/`, `install/`, `log/`, 외부 SDK·upstream checkout, 모델 weight, 카메라/측정 output은 커밋하지 않는다.
- 활성 `pallet/meshes/mass_revised/`, `pallet/meshes/with_gripper/`, `pallet_gripper_description/`은 백업과 구별한다. URDF 참조와 출처를 확인한 뒤 필요한 canonical 파일만 선별한다.
- 기능 소유 브랜치의 최신 원격 상태와 분기점을 확인하기 전에는 커밋·푸시하지 않는다. 브랜치 이동에는 원본 작업 트리를 건드리지 않는 별도 worktree를 사용한다.

## 실행 순서

터미널 1에서 기준선과 변경을 확인한다.

```bash
cd /home/minwoo/Development/LLM
git status --short --branch
git diff --numstat
git diff --check
```

터미널 2에서 시뮬레이션 연결 전에 국소 테스트를 확인한다.

```bash
cd /home/minwoo/Development/LLM
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s ros2_ur_ws/src/pallet/test -p 'test_*.py'
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s ros2_ur_ws/src/mtc_tutorial/test -p 'test_*.py'
```

## 검증 명령과 결과

- Pallet `unittest`: 11개 통과.
- MTC robot profile `unittest`: 5개 통과.
- `git diff --check`: 위 `scene_manager.py` 후행 공백 3건으로 실패.
- ROS package build, Isaac Sim GUI, 실제 모터 구동: 이번 기준선 조사에서 실행하지 않음.

## 남은 주의사항

- 위 기준 HEAD와 diff 해시는 작업 트리 내용의 완전한 백업이 아니다. 특히 untracked 파일은 Git에서 복원되지 않는다.
- Notion의 최신 통합 로드맵은 Pallet + Isaac 직접 pose Pick & Place를 Vision/LLM 연결 전에 반복 검증하도록 요구한다. 이 문서는 그 전제인 변경 분류만 완료했다.

## 다음 작업 로드맵

- [x] 혼합 변경과 신규 소스/백업 구분
- [x] 현재 HEAD, 변경 규모, 테스트 결과 기록
- [x] 백업 스냅샷을 Git 커밋 후보에서 제외
- [ ] `scene_manager.py` 공백 오류 정리 및 기능별 hunk 확정
- [ ] Pallet 테이블 치수·좌표를 MTC/MoveIt/Isaac에서 통일
- [ ] Pallet MTC → Isaac 직접 pose 실행 경로와 반복 결과 기록 구축
- [ ] 기능별 브랜치에 반영할 때 원격 최신 상태와 분기점 확인, 국소 빌드·테스트 수행
