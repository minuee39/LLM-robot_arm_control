# Pallet runtime safety stop and fail-closed recovery — 2026-10-01

## Scope

This chapter connects the existing `/pallet/safety_events` telemetry to the
Isaac controller bridge. It remains opt-in and was tested only in Isaac Sim;
no physical robot or CAN motor was operated.

## Runtime contract

- `safety_cancel_enabled` defaults to `false`.
- Accepted event codes are `SLIP_DETECTED`, `BLOCK_DROP_DETECTED`,
  `TILT_DETECTED`, and `TELEMETRY_STALE`.
- Events must contain a finite `published_at_unix` no more than two seconds
  old. Malformed, unsupported, stale, or future events are ignored.
- The first valid event latches the bridge. The bridge immediately commands
  the measured J1–J6 and J8/J9 positions, aborts the active controller goal,
  and rejects new goals until an explicit reset.
- A reset requires fresh Isaac feedback and idle arm/gripper controllers.
- Recovery is conditional: hold, plan a bounded +Z retreat, open, detach, and
  return home. If planning is rejected, no recovery trajectory is executed and
  the runner publishes a fresh event to re-latch measured hold.

## Validation results

### Injected safety event

- Scenario: 500 g `mass_500_release_stable`, event injected during transfer.
- Event to bridge hold latency: about 5.2 ms in the measured run.
- Active arm goal result: `PATH_TOLERANCE_VIOLATED: Safety stop latched`.
- Normal task completion message: absent, as required.
- The 40 mm recovery path was rejected at planning time with zero execution.
- The fallback event re-latched measured hold; outcome:
  `refused_relatched`.
- Report: `/tmp/pallet-safety-failclosed-20261001/report.json`.

The planning refusal is a safe result, not a completed recovery motion. It
shows that the system does not open the gripper or force a Cartesian move when
the interrupted state cannot produce a collision-valid retreat.

### Normal 500 g regression with cancellation enabled

- Result: 3/3 successful pick-place cycles.
- Safety events: 0/3.
- Post-release bilateral re-contact: 0/3.
- Final XY error: mean 1.921 mm, maximum 2.174 mm.
- Transfer bottom clearance: mean 27.682 mm, minimum 27.633 mm.
- Loaded jaw asymmetry: mean 1.647 mm, maximum 1.773 mm.
- Report: `/tmp/pallet-safety-normal-repeat3-20261001/report.json`.

## Commands

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Normal 500 g repeatability with runtime cancellation armed:

```bash
python3 scripts/run_pallet_scenario_matrix.py \
  --scenario-file scripts/pallet_scenarios_500g.json \
  --scenario mass_500_release_stable --repeat 3 \
  --safety-cancel-enabled \
  --startup-timeout 240 --task-timeout 420 --monitor-seconds 420 \
  --settle-seconds 3 --min-clearance 0.020 \
  --slip-threshold 0.005 --slip-confirm-samples 3 \
  --output-dir /tmp/pallet-safety-normal-repeat3
```

Injected transfer safety stop and conditional recovery:

```bash
python3 scripts/run_pallet_scenario_matrix.py \
  --scenario-file scripts/pallet_scenarios_500g.json \
  --scenario mass_500_release_stable --repeat 1 \
  --safety-cancel-enabled --expect-safety-stop \
  --inject-safety-event-after-contact 10.0 \
  --recovery-timeout 240 --startup-timeout 240 \
  --task-timeout 420 --monitor-seconds 420 \
  --settle-seconds 3 --min-clearance 0.020 \
  --slip-threshold 0.005 --slip-confirm-samples 3 \
  --output-dir /tmp/pallet-safety-failclosed
```

The injection option is test-only and is disabled by default. Every output
directory must be new because the runner intentionally refuses to overwrite a
previous experiment.

## Next chapter

1. Add independent block-drop, tilt, and stale-telemetry detectors using
   debounced thresholds and the same event contract.
2. Record the exact collision pair or joint-bound reason when a recovery plan
   is refused.
3. Define multiple bounded recovery candidates and choose only a collision-
   valid path; keep measured hold if all candidates fail.
4. Validate the safety state machine against real gripper current/force data
   only after simulation fault coverage is complete.
