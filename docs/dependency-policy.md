# Pallet VLA dependency policy

## 목적

이 문서는 `/home/minwoo/Development/LLM`의 프로젝트 코드와 외부 의존성을 구분하고, M0 기준선을 다른 환경에서도 재현하기 위한 관리 규칙을 정의한다.

## 소유권 경계

메인 저장소가 직접 관리하는 코드는 `ur10e/`와 다음 ROS 2 패키지다.

- `isaac_moveit_bridge`
- `mtc_tutorial`
- `ur10e_robotiq_description`
- `ur10e_robotiq_2f140_moveit_config`

외부 upstream 코드는 프로젝트 코드와 섞어 수정하지 않는다. 수정이 필요하면 별도 fork와 commit으로 보존하거나, 필요한 최소 코드만 프로젝트 전용 패키지로 이동한다.

## ROS 2 의존성

- Jazzy에서 제공되는 MoveIt, MTC, SRDF, launch 보조 패키지는 `ros2_ur_ws/scripts/install_jazzy_dependencies.sh`로 binary package를 설치한다.
- Jazzy binary가 없는 source dependency만 `ros2_ur_ws/dependencies.repos`에 원격 URL과 commit SHA를 고정한다.
- 새 workspace에서는 빈 `src`에 `vcs import`로 복원한다.
- `build/`, `install/`, `log/`는 생성물이므로 Git에 저장하지 않는다.
- `ros2_astra_camera`는 `dependencies.repos`에 고정된 upstream checkout으로 복원하고, `patches/ros2_astra_camera-jazzy.patch`를 설치 스크립트가 멱등하게 적용한다.
- `OpenNI/`와 `OrbbecSDK/`는 로컬 카메라 SDK 설치물이므로 Git에 저장하지 않는다.
- `Universal_Robots_ROS2_Description`와 `pal_robotiq_description`은 현재 로봇 description 호환성 검증을 위해 유지한다.
- Humble `moveit2_tutorials` checkout은 보존하지만 `COLCON_IGNORE`로 Jazzy build에서 제외한다.
- `ur10e_moveit2_tutorials`는 upstream 전체를 수정한 실험용 fork다. 로컬 기준 commit `38420a40faef89878d07462f283981c57cba192e`로 보존하며 canonical runtime package로 사용하지 않는다. 필요한 UR10e 전용 구현은 `mtc_tutorial`로 이동한다.

복원 명령:

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
vcs import src < dependencies.repos
./scripts/install_jazzy_dependencies.sh
./scripts/rebuild_jazzy_workspace.sh
```

## Python 환경

환경을 다음처럼 분리한다.

- `.venv`: 단위 테스트, 데이터 처리, 일반 vision 도구
- Isaac Sim Python: Isaac/Omni 전용 앱
- `remind` Conda 환경: YOLO-Seg, DINOv3, REMIND
- ROS 2 Python: `/opt/ros/jazzy`와 Jazzy에서 새로 빌드한 colcon overlay

서로 다른 환경의 `PYTHONPATH`, user-site package, OpenCV/NumPy ABI를 섞지 않는다. 현재 `.venv`의 정확한 스냅샷은 `requirements/baseline-lock.txt`에 기록한다. 이 파일은 기준선 재현용이며, 모든 환경에 그대로 설치하는 통합 requirements가 아니다.

현재 `.venv`는 `opencv-contrib-python`을 제공하지만 `ultralytics` metadata는 `opencv-python`을 요구해 `pip check` 경고가 남는다. import가 된다는 이유로 이 충돌을 정상으로 간주하지 않으며, Python packaging 단계에서 OpenCV 배포판을 하나로 통일한다.

## 모델과 데이터

- `.pt` weight, 학습 데이터셋, 실행 output은 Git에 저장하지 않는다.
- 모델의 역할, 상대 경로, SHA256은 `models/manifest.yaml`에 저장한다.
- API key, Hugging Face token, Notion token은 manifest, 문서, 로그에 기록하지 않는다.
- DINOv3와 REMIND는 모델 snapshot/repository revision/config hash를 함께 고정한다.

## 버전 변경 규칙

1. dependency 변경은 기능 변경과 별도 commit으로 만든다.
2. URL 또는 version만 바꾸지 말고 변경 이유와 검증 결과를 함께 기록한다.
3. Python 테스트와 관련 ROS package build가 통과하기 전에는 기준 tag를 이동하지 않는다.
4. dependency 실패 시 임의로 최신 버전으로 올리지 않고 기존 SHA로 복원한다.
5. 실제 로봇용 dependency 변경은 시뮬레이션과 저속 dry-run을 먼저 통과해야 한다.

## M0 기준선 완료 조건

- 기준 commit/tag로 현재 프로젝트 코드를 식별할 수 있다.
- 외부 ROS source의 URL과 SHA 또는 legacy content hash가 기록되어 있다.
- Python 환경 스냅샷과 알려진 충돌이 기록되어 있다.
- 모델 weight를 hash로 식별할 수 있다.
- 실행한 검증 명령과 실패가 baseline 문서에 기록되어 있다.
