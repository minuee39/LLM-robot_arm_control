# Persistent User Preferences

## Notion
- 사용자가 명시적으로 다시 허용하기 전까지 어떤 내용도 Notion에 작성, 추가, 수정하거나 기록하지 않는다.
- 작업 완료 보고나 변경 로그도 Notion에 남기지 않는다.
- 이 지침을 해제할 때는 사용자의 명시적인 요청이 필요하다.

## 작업 기록 형식
- 앞으로 기술 작업 기록은 Notion의 `UR10e + Robotiq 2F-140 MoveIt2 / Isaac Sim 작업 기록` 페이지 형식을 기준으로 작성한다.
- 권장 순서는 제목, 작성일과 작업 목적, 최종 기준 workspace, 현재 주요 패키지/파일, 작업 중 발생한 문제, 원인, 해결 과정, 최종 구조와 설계 결정, 실행 순서, 검증 명령과 결과, 남은 주의사항, 다음 작업 로드맵이다.
- 문제 해결 내용은 `문제 -> 원인 -> 해결 -> 검증` 흐름으로 구체적으로 적고, 관련 절대 경로, 설정값, 실행 명령, 핵심 로그를 포함한다.
- 실행 절차는 터미널별 순서와 복사 가능한 코드 블록으로 작성하고, 로드맵은 완료 여부를 확인할 수 있는 체크리스트로 작성한다.
- 사용자가 Notion 기록을 명시적으로 허용한 경우, 모든 기술 작업 기록에 `Codex 세션 재개` 항목을 포함하고 아래 명령을 복사 가능한 코드 블록으로 기록한다. 사용자가 다른 세션을 명시적으로 지정하기 전까지 이 세션 ID를 유지한다.
  ```bash
  cd /home/minwoo/Desktop/LLM
  codex resume 019f8001-247a-73b2-b9ba-74eed73c3f71
  ```
- 이미지나 영상은 실제 자료가 있을 때만 관련 설명 가까이에 배치하며, 없는 자료를 임의로 만들거나 있다고 표현하지 않는다.
- 이 형식은 대화나 로컬 문서의 작업 기록에도 적용한다. Notion 기록은 위의 Notion 금지 지침이 사용자의 명시적 허용으로 해제된 경우에만 수행한다.

## Git 브랜치 자동 라우팅
- 사용자가 `git에 올려줘`, `커밋해줘`, `push해줘`처럼 대상 브랜치를 생략해도 현재 체크아웃 브랜치에 바로 커밋하지 않는다. 먼저 변경 파일과 diff를 기능별로 분류하고 아래 소유 브랜치로 커밋·푸시한다.
- 현재 브랜치 이름보다 변경 내용의 기능적 소유권을 우선한다. 특히 `agent/*` 브랜치는 임시 통합 작업용으로 간주하며, 사용자가 그 브랜치를 명시하지 않은 한 새 커밋을 푸시하지 않는다.
- 기존 원격 feature 브랜치를 기준으로 작업하며, 커밋 전 `origin/<target>`의 최신 상태와 분기점을 확인한다. 푸시 후에는 로컬 HEAD와 `origin/<target>` SHA가 같은지 확인한다.

### 브랜치 소유 범위
- `feature/llm-spatial-commands`: 자연어·LLM 명령 해석, 공간 참조, command schema 및 parser. 대표 경로는 `ur10e/command_parser.py`, `ur10e/llm_to_json.py`, 관련 앱과 테스트다.
- `feature/mtc-grasp-stability`: MoveIt Task Constructor, grasp 검증, trajectory/action bridge, Isaac-MoveIt 실행 연동. 대표 경로는 `ros2_ur_ws/src/mtc_tutorial/`, `ros2_ur_ws/src/isaac_moveit_bridge/`, `ur10e/mtc_command_runner.py`, `ur10e/grasp_verifier.py`, 관련 실행 스크립트와 테스트다.
- `feature/pallet-robot-description`: Pallet 로봇 URDF/Xacro, mesh, joint/motor description, MoveIt config. 대표 경로는 `ros2_ur_ws/src/pallet/`, `ros2_ur_ws/src/pallet_moveit_config/`다.
- `feature/robstride-can-control`: RobStride/SocketCAN 통신, 모터 읽기·제어·진단 도구. 대표 파일은 `rs06_control.py`, `robstride_read_angles.py`와 향후 CAN hardware interface다.
- `feature/vision-depth-stability`: RGB-D 카메라, Astra, YOLO, depth 처리, detection 안정화, vision scene. 대표 경로는 `ur10e/vision/`, `ur10e/apps/*camera*`, `ur10e/scripts/astra_*`, 관련 카메라 실행 스크립트와 테스트다.
- 위 범위에 속하지 않는 저장소 전체 dependency, baseline, CI, 공통 문서, `.gitignore` 정리는 `chore/repository-maintenance`를 사용한다. 브랜치가 없으면 최신 `origin/main`에서 생성한다.

### 분류와 커밋 규칙
- 테스트는 검증 대상 구현과 같은 브랜치로 보낸다. 문서와 dependency 변경도 실제로 설명하거나 지원하는 기능 브랜치가 명확하면 그 브랜치를 따른다.
- 한 파일이 여러 분야를 포함하면 hunk 단위로 분리한다. 분리가 안전하지 않으면 공통 선행 커밋의 대상 브랜치를 판단하고, 의미가 실제로 모호할 때만 사용자에게 가장 작은 질문을 한다.
- 여러 분야의 변경이 섞인 worktree에서는 한 커밋으로 합치지 않는다. 분야별로 별도 커밋을 만들고 각각 해당 feature 브랜치에 푸시한다.
- 현재 worktree가 대상 브랜치가 아니면서 변경사항이 있으면 무리하게 checkout하지 않는다. 임시 worktree를 사용해 대상 브랜치에 해당 patch와 명시적으로 확인한 untracked 파일만 옮긴다. 원본 worktree는 각 원격 푸시가 검증될 때까지 보존한다.
- 스테이징은 확인된 경로만 `git add -- <paths>`로 수행한다. `git add .`, `git add -A`, `git add --all`은 사용하지 않는다.
- 비밀정보, 모델 weight, 데이터셋, build/install/log, 카메라 캡처와 측정 output, 외부 SDK 및 중첩 upstream checkout은 커밋하지 않는다. 필요하면 `.gitignore`와 재현 가능한 dependency manifest만 반영한다.
- 각 브랜치에서 관련 테스트·문법 검사·빌드를 실행하고, 실패한 검증과 환경 제약을 완료 보고에 명시한다.
- 사용자가 명시적으로 단일 브랜치나 단일 커밋을 요구하면 그 지시를 우선하되, 다른 분야 변경이 섞여 있으면 임의로 포함하지 않는다.
