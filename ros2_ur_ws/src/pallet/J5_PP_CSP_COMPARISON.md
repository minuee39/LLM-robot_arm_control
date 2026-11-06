# J5 PP/CSP comparison

This is a separate bench experiment; the MoveIt bridge remains PP.

The two trials send identical indexed 10 Hz joint references: J5 moves from the
captured baseline to +0.1 rad in 2 s, returns in 2 s, and holds for 2 s.
Other arm axes hold captured positions in PP; ID7 is not controlled. All axes
must initially report PP mode. J5 mode is written and verified before enabling.
All six axes stop between trials and on exit; external support must remain in
place across torque release. J5 returns to PP only after STOP is confirmed.

PP profile velocity/acceleration registers are preserved, not read back or
claimed identical to CSP parameters. CSP has a separate limit_spd register
0x7017, set and verified at 0.1 rad/s. This volatile CSP speed setting remains
after the test; no Flash save is requested. Current limits are preserved and
checked against the existing live configuration. Maximum requested reference
speed is approximately 0.07854 rad/s. This is a small-motion screening test,
not proof of full-speed six-axis MoveIt tracking or a firmware upgrade.

Abort guards: original feedback/fault/current-limit validation, 0.2 rad/s speed
monitor, 0.12 rad measured travel, 0.005 rad other-axis drift, 0.01 rad J5
reference error, cycle overruns, and baseline return/hold timeout. Initial and
between-trial baseline mismatch above 0.001 rad aborts. No stale feedback reuse.

Stop Motor Studio, MoveIt serial bridge, independent feedback and hold programs
before the live command; they cannot share the serial device.

```bash
cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Offline: does not open the device or move motors.
ros2 run pallet pallet_j5_pp_csp_compare.py
# Live: enables six arm motors, moves J5, then releases all six.
mkdir -p measurements/position
ros2 run pallet pallet_j5_pp_csp_compare.py --live --arm-supported \
  --log "measurements/position/J5_pp_csp_$(date +%Y%m%d_%H%M%S).jsonl"
```

Each mode produces `trial_sample` records including reference tick, send/sample
times, all joint feedback, and `trial_result` peak/RMS errors. Error is measured
against the last sent reference at the subsequent sequential feedback sample;
this is not a synchronized zero-latency measurement. Compare both timing and
errors. Keep 10 Hz unchanged for this first comparison; a higher-rate transport
requires separate implementation and validation.

Protocol source: manufacturer Python sample `examples/position_control.py`,
`robstride_dynamics/protocol.py`, and RS05 manual (private protocol mode 1/5).
https://github.com/RobStride/Python_Sample/blob/main/examples/position_control.py
https://files.seeedstudio.com/products/RobStride/Product%20Literature/RS05/RS05User%20Manual251112.pdf

Software validation: 133 pallet tests passed, including identical reference
sequences, PP default rejection of CSP, bounded CSP speed, failure STOP, and
restoration ordering. Physical results must be recorded separately.

## J5 PP profile comparison (2026-10-30)

`pallet_j5_response_compare.py --visual-20deg --j5-pp-slow` keeps PP mode,
changes J5 only to joint-space PP 0.1rad/s and 0.1rad/s² while all arm axes
are stopped, then uses the existing +20deg/10s cosine reference and 5s hold.
ID7 is not controlled. No current limits, PD gains, MoveIt settings, watchdog,
tracking tolerance or sampling period are changed. No collision planning occurs.
After confirmed six-axis STOP, J5 raw PP 10/10 is transmitted based on the
recorded baseline all_motor_defaults_restored_20261027.json, not a fresh snapshot.
These fields are write-only in the existing hardware workflow: neither the trial
values nor restored values have readback verification. No Flash save is sent.
Partial setting failures also request restoration; an unconfirmed STOP prevents
restoration writes and reports an error. Loss of communication can prevent restore.
Stop competing serial programs, support the arm, and clear the +20deg motion path
before live execution. Offline is the default; live requires --live --arm-supported
and a new --log path. Physical trial remains to be performed by the operator.
