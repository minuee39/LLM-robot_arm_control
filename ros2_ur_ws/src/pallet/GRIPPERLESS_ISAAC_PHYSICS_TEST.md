# Gripperless Pallet Isaac Sim physics test

This experiment validates the five-axis Pallet arm (`j2` through `j6`) without
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

Build and source `pallet` and `pallet_moveit_config`, then use three terminals:

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
# Terminal 1: visible Isaac Sim
ros2 run pallet run_pallet_isaac_moveit_bridge.sh --no-headless
```

```bash
# Terminal 2: robot_state_publisher + action bridge + MoveIt2 + RViz
ros2 launch pallet_moveit_config isaac_demo.launch.py
```

The action bridge accepts exactly `j2` through `j6`. It returns success only
after Isaac feedback settles within the configured goal tolerance. Torque
saturation or insufficient gravity holding therefore appears as a trajectory
goal tolerance failure instead of a false successful execution.

### Current rated-torque result

The host-GPU test with the checked-in rated torque limits currently fails the
physics acceptance criteria. `move_pose_a` misses `j3` by about `0.789 rad`,
and `move_pose_b` misses `j6` by about `0.761 rad`. The action bridge correctly
returns `GOAL_TOLERANCE_VIOLATED` for these cases. Do not increase the action
tolerance to hide this result; verify link inertia/frames and the motor-side to
joint-side torque conversion before changing the rated effort limits.
