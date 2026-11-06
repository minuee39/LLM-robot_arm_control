# Adaptive gripper serial bridge

J1..J6 + raw motor joint `gripper_motor` (ID7 RS05), one serial owner.
Arm and gripper FollowJointTrajectory actions share reservation, health gate,
STOP/fault/cancel and hold handling. Concurrent requests are rejected. A group
command holds the other group's measured start position. Live feedback publishes
6 arm + raw ID7 + 15 model-derived gripper joints. Derived fingers are not
independent contact measurements. Raw zero must match the measured calibration.

```bash
cd ~/Development/LLM/ros2_ur_ws
source install/setup.bash
ros2 launch pallet_physical_moveit_config adaptive_serial_bridge.launch.py
```

Default dry run opens no serial port. RViz Planning Group `gripper` uses
`open` (+12), `middle` (0), `closed_measured` (-12); the last is still 19mm open.
Arm group remains `pallet_arm`. ID7 profile uses 2 rad/s and 5 rad/s²;
arm caps remain 0.5 rad/s and 0.2 rad/s². Planning caps have 10% headroom.
Current limit is preserved, never overwritten by this profile.

Live requires the existing commissioned arm config plus a verified positive
`expected_current_limit_a` and `commissioned: true` in
`pallet/config/adaptive_gripper_motor.yaml`. ID7 readback has now been recorded as described below; the profile expects
0.8 A. Missing or invalid profile values fail before opening the port.
No physical actuation was performed during this integration.

The URDF includes the previously approved **virtual wrist**. URDF/MoveIt do not
encode its nonlinear closed-loop mapping. Passive joint states update from the
measured motor in the bridge; MoveIt planning does not predict future finger
collision geometry along a gripper trajectory. Therefore this is a control and
visual-feedback integration, not validated contact/grasp or swept-finger collision
planning. Hardware equivalence and collision checking require further work.

Regenerate descriptions after geometry changes:
`python3 src/pallet/scripts/generate_adaptive_moveit.py`.
The standard serial_bridge.launch.py now defaults to adaptive_gripper:=true.
Use adaptive_gripper:=false for the legacy six-axis-only model. Both routes
include the planning floor. dry_run, observe_only, config_file and use_rviz
are forwarded unchanged; the live arm config selects the shared serial port.

Dry-only end-to-end smoke test (isolated ROS domain):

```bash
ROS_DOMAIN_ID=78 ROS_LOG_DIR=/tmp/adaptive-integration-log ros2 run pallet_physical_moveit_config verify_adaptive_bridge.py
```

Starts/stops its own launch, checks arm/gripper actions and cancellation, and
submits a gripper trajectory through MoveIt's ExecuteTrajectory action.

## RViz Joints에서 그리퍼 미리보기

```bash
cd ~/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pallet_physical_moveit_config adaptive_serial_bridge.launch.py
```

**MotionPlanning → Planning Group: gripper → Joints**에서 `gripper_motor` 값을 바꾸면 목표 자세의 너트와 손가락 관절이 함께 갱신됩니다. 별도 Gripper Slider 패널은 사용하지 않습니다. RViz Joints의 회전 관절 표시는 도(degree)입니다. 실측 기준 −12, 0, +12 rad는 각각 약 −687.55°, 0°, +687.55°입니다.

`pallet_physical_moveit_config/AdaptiveMotionPlanning`은 기존 MoveIt MotionPlanning 표시를 확장하며, start/goal 상태의 raw motor 위치에서 15개 종속 관절을 계산합니다. 계산표는 기존 `AdaptiveGripper` solver에서 0.05 rad 간격으로 생성하며, 사이 구간은 보간합니다. 재생성은 `python3 src/pallet/scripts/generate_adaptive_moveit.py`로 수행합니다.

Joints 입력은 **목표 상태 미리보기**이며 모터 명령을 전송하지 않습니다. 현재 상태 모델은 그대로 피드백을 표시합니다. 그리퍼는 Planning 탭에서 **Execute**만 누르면 선택한 모터 각도로 이동합니다. Plan은 필요 없으며 이 직접 동작에는 MoveIt 바닥·장애물 충돌 검사가 적용되지 않습니다. 팔은 기존 Plan / Execute 방식을 유지합니다. 기본 실행은 dry-run입니다. 실제 하드웨어 구동은 앞서 설명한 모터 검증 설정이 필요합니다.

이 확장은 RViz 목표 상태 표시를 보정합니다. MoveIt 경로 계획 도중의 모든 중간 상태에 비선형 연동을 적용하는 것은 아니므로, 이 변경으로 손가락의 전체 이동 경로 충돌 검증이 추가되지는 않습니다. 기존 브리지의 1회 이동 제한도 유지됩니다.

검증용 `build/pallet_physical_moveit_config/adaptive_preview_smoke`는 RViz 창을 로드하고 기존 Joints 표의 값을 변경해 raw motor와 종속 관절 변화, 패드의 위치를 검사합니다. 별도의 ROS domain에서 dry-run launch와 함께 실행합니다.

### Gripper self collision check

The virtual parallel-pad split is 10 mm above the inner pin (previously 4 mm).
This removes outer-link/contact-pad interference across the measured motor range.
Only each side's yoke/wrist connection and inner-upper/yoke shared loop pin are
added as adjacent collision exceptions. Other self collisions and environment
collisions remain enabled. This is the proposed URDF geometry, not a physical CAD
modification or a hardware clearance certification.

After building, run `build/pallet_physical_moveit_config/adaptive_collision_audit`
from the workspace with its environment sourced. It checks the 481 coupled poses
at 0.05 rad spacing over [-12, 12] using MoveIt's self collision detector at the
arm default pose. It fails on any remaining self collision. This discrete sweep
does not certify continuous motion, other arm configurations, or the existing
uncoupled intermediate planning states described above.

### Stable goal preview during feedback

The gripper SRDF group includes gripper_motor and all 15 coupled passive joints.
Only gripper_motor is actuated; the passive rows are read-only in Joints. This
keeps MoveIt's updateStateExceptModified from replacing the linkage goal with
measured linkage values on each planning-scene update. Previously those updates
alternated with the preview correction, making the fingers shake. The GUI smoke
test edits the native motor row and checks 100 measured-state updates preserve
the coupled goal immediately, without waiting for a later correction frame.

### Physical hardware entry point

Source the ROS and workspace environments, then use:

```bash
ros2 launch pallet_physical_moveit_config adaptive_hardware.launch.py
```

This opens the real serial bus in observe-only mode with the existing arm
configuration, reads ID1..7, and displays measured feedback without enabling
motion. RViz uses the original MoveIt Scene Robot for measured
feedback at 25% opacity, plus an opaque query goal. Both use the
same arm + adaptive gripper model. There is no separate TF-based RobotModel
display. Start-state and planned-path robot copies are hidden.
The collision floor is dark navy (#0f1937).
While observing only, optionally turn off `Planning Request / Query Goal State`.
Stop other serial owners before starting this launch.

After confirming ID7's existing current limit and commissioning its profile:

```bash
ros2 launch pallet_physical_moveit_config adaptive_hardware.launch.py mode:=execute
```

Use `pallet_arm` or `gripper` in MoveIt. Joints edits preview the goal;
For the arm, Plan / Execute sends it to the shared serial bridge. For the
gripper, use Execute directly; Plan and Plan & Execute are disabled because
OMPL does not enforce the nonlinear linkage coupling. ID7 velocity and
acceleration caps are 2 rad/s and 5 rad/s². The launch does not change
current limits, zero calibration, or commissioning flags.

### ID7 readback, 2026-11-03

User-provided read-only preflight at 13:21:43 UTC confirmed ID7 responses:
current limit 0.8000000119 A, position -0.0010886192 rad, current 0 A,
velocity -0.0021421527 rad/s, mode 1, watchdog 0, no read errors.
The profile now expects 0.8 A and admits live bridge startup. The commissioning
flag records this limited admission, not a physical motion certification.
The bridge preserves current limit, checks it again before enable, and sets and
reads back the configured 0.25 s watchdog before enabling any motor. A zero
watchdog in this preflight is not a verified TX-silence stop test. Position zero
was not reset. Physical trajectory execution is still unverified.

## Direct raw-angle command (without OMPL)

With the bridge launch already running, use
`ros2 run pallet_physical_moveit_config gripper_direct.py 0.2`.
The argument is an absolute ID7 raw motor angle in radians, not finger angle or
opening width. The tool reads fresh feedback and submits the start and final
angles with a bounded time budget to the existing gripper action. It preserves bridge
fault, range, profile, reservation, cancellation and hold checks. It does not
perform MoveIt collision planning. RViz Plan and Plan & Execute remain available
for the arm and are disabled for the gripper. Execute directly controls the
selected gripper motor angle with the existing bridge checks.
Moves needing more than 60 seconds are rejected; use intermediate targets.

For a gripper-only RViz/CLI goal, the bridge immediately commands the final
angle and refreshes that same angle every cycle for the watchdog. It sends no
intermediate positions. The drive generates motion using its 2 rad/s speed
and 5 rad/s² acceleration limits. The validated cubic duration supplies a
conservative arrival time budget (minimum 1 second), plus the configured
settling allowance; arrival may complete earlier.
The moving gripper is checked against final position tolerance and a bounded
arrival deadline, rather than an interpolated path. Arm path-error monitoring,
gripper speed monitoring at 2.2 rad/s, cancellation, measured joint bounds,
fault handling, final-position hold and the preserved 0.8 A current limit
remain active. Endpoint execution has software validation only; physical
arrival and drive ramp behavior remain unverified.
