# Gripperless Pallet Isaac Sim physics test

This experiment validates the five-axis Pallet arm (`j1` through `j5`) without
a gripper. It imports the source URDF directly into Isaac Sim, enables gravity,
uses the URDF mass/inertia, and clamps each position drive to its URDF rated
effort limit in force mode.

Run from the package source tree:

```bash
cd /home/minwoo/Desktop/LLM/ros2_ur_ws/src/pallet
./scripts/run_pallet_isaac_gripperless_physics_test.sh
```

When running from a Git worktree, use that worktree's package path instead. The
default headless run writes:

```text
/tmp/pallet_gripperless_physics_report.json
```

For a visible Isaac Sim window:

```bash
./scripts/run_pallet_isaac_gripperless_physics_test.sh --no-headless
```

To check only the five-joint/gripperless model and experiment configuration
without requiring a GPU:

```bash
./scripts/run_pallet_isaac_gripperless_physics_test.sh --validate-only
```

To tune poses, gains, tolerances, or phase duration, edit
`config/isaac_gripperless_physics_test.json`. A successful process exits with
status `0`; a physics constraint, settling, model, or import failure exits with
status `1`. The report records final joint error plus applied and measured
effort for every phase.

## MoveIt2 integration

Build `pallet` and `pallet_moveit_config` in the ROS workspace-specific build
directories. This avoids reusing a stale CMake cache from the repository root:

```bash
cd /home/minwoo/Desktop/LLM
source /opt/ros/humble/setup.bash
colcon --log-base ros2_ur_ws/log build \
  --base-paths ros2_ur_ws/src/pallet ros2_ur_ws/src/pallet_moveit_config \
  --packages-select pallet pallet_moveit_config \
  --build-base ros2_ur_ws/build \
  --install-base ros2_ur_ws/install \
  --symlink-install
source ros2_ur_ws/install/setup.bash
```

After sourcing `ros2_ur_ws/install/setup.bash` in each shell, use three
terminals:

```bash
# Terminal 1: torque-limited Isaac articulation
ros2 run pallet run_pallet_isaac_moveit_bridge.sh --no-headless
```

```bash
# Terminal 2: feedback-aware FollowJointTrajectory action
ros2 run pallet pallet_follow_joint_trajectory_bridge.py
```

```bash
# Terminal 3: MoveIt2
ros2 launch pallet_moveit_config move_group.launch.py
```

For a directly visible Isaac Sim + RViz session, use two terminals instead:

```bash
# Terminal 1: visible Isaac Sim, holding the same zero-radian home pose as RViz
ros2 run pallet run_pallet_isaac_moveit_bridge.sh --no-headless --rviz-home
```

```bash
# Terminal 2: robot_state_publisher + action bridge + MoveIt2 + RViz
ros2 launch pallet_moveit_config isaac_demo.launch.py
```

`--rviz-home` disables gravity only on the imported Pallet articulation. This
keeps its `j1` through `j5` zero-radian pose aligned with RViz, whose display
does not simulate gravity. Omit the option when validating rated-torque gravity
holding; the standalone gripperless physics test always keeps gravity enabled.

If Isaac reports an undefined `spdlog`/`fmt` symbol while importing `rclpy`,
do not append its bundled ROS libraries after `/opt/ros/humble`. The
`run_pallet_isaac_moveit_bridge.sh` wrapper deliberately prepends
`isaacsim.ros2.bridge/humble/lib` so the bundled `rclpy`, `rcl`, `spdlog`, and
`fmt` ABI versions stay together. The bridge can be checked without opening a
window:

```bash
ros2 run pallet run_pallet_isaac_moveit_bridge.sh \
  --headless --smoke-test-seconds 2 \
  --output /tmp/pallet_isaac_moveit_bridge_smoke.json
```

A successful check prints `rclpy loaded` and a report with `"passed": true`.

The action bridge accepts exactly `j1` through `j5`. It returns success only
after Isaac feedback settles within the configured goal tolerance. Torque
saturation or insufficient gravity holding therefore appears as a trajectory
goal tolerance failure instead of a false successful execution.

### Current rated-torque result

The host-GPU test passes with the checked-in continuous rated-torque limits.
Isaac fixed-joint merging is required: without it, the importer assigns a 1 kg
fallback mass to each massless coordinate-only link (`base_footprint`, `tool0`,
and `tcp`), creating a phantom payload and false torque saturation. The test
uses smooth, acceleration-bounded commands and collision-safe poses. The
MoveIt Isaac launch also installs a floor collision object matching the Isaac
ground plane so planning rejects below-floor trajectories before execution.
