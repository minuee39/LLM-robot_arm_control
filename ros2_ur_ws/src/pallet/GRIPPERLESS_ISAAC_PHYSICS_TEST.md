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

This test intentionally does not claim MoveIt2 integration. It isolates the
physics/controller layer first. A MoveIt2 controller must later send the same
five joint names and be tested against this force-limited articulation; a
six-joint or gripper-enabled planning model is incompatible with this test.
