"""Bounded J5 gravity controller; motor current in peak amperes, torque in Nm."""
from dataclasses import dataclass
import math

# RS05 CAN2.0 User Manual260713: 0.94Nm/Arms, 2.4Apk at 1.6Nm.
# Nominal conversion only. Not an independent output torque calibration.
RS05_NOMINAL_NM_PER_APK = .94 / math.sqrt(2.)


def rs05_reported_torque(raw):
    if type(raw) is not int or not 0 <= raw <= 65535:
        raise ValueError('Expected unsigned 16-bit torque')
    return (raw / 65535. * 2. - 1.) * 5.5


@dataclass(frozen=True)
class GravityLimits:
    current_a: float = .03
    measured_current_a: float = .1
    slew_a_s: float = .06
    travel_rad: float = .035
    speed_rad_s: float = .15
    period_s: float = .05
    timeout_s: float = .025
    duration_s: float = 1.
    gain: float = .1
    ramp_s: float = .5
    damping_nm_s_rad: float = .02
    upstream_motion_rad: float = .01

    def validate(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('Finite positive limits required')
        if self.current_a > .03 or self.measured_current_a > .1 or self.slew_a_s > .06:
            raise ValueError('Exceeds initial J5 commissioning current envelope')
        if self.travel_rad > .035 or self.speed_rad_s > .15 or self.duration_s > 2 or self.gain > .2:
            raise ValueError('Exceeds initial J5 motion/time/gain envelope')
        if not self.timeout_s < self.period_s <= .1:
            raise ValueError('Require timeout < period <= 0.1s')


def output(gravity_nm, joint_velocity, direction, ratio, elapsed, previous_a, dt, limits):
    limits.validate()
    if not all(math.isfinite(v) for v in (gravity_nm,joint_velocity,ratio,elapsed,previous_a,dt)):
        raise ValueError('Nonfinite control input')
    if direction not in (-1,1) or ratio <= 0 or not 0 < dt <= 2*limits.period_s or elapsed < 0:
        raise ValueError('Invalid direction/ratio/timing')
    tau = limits.gain * min(1., elapsed/limits.ramp_s) * gravity_nm - limits.damping_nm_s_rad*joint_velocity
    requested = direction*tau/(ratio*RS05_NOMINAL_NM_PER_APK)
    bounded = max(-limits.current_a,min(limits.current_a,requested))
    step = limits.slew_a_s*min(dt,limits.period_s)
    current = max(previous_a-step,min(previous_a+step,bounded))
    current = max(-limits.current_a,min(limits.current_a,current))
    return {'command_iq_a':current,'requested_iq_a':requested,'current_clipped':abs(requested)>limits.current_a,
            'requested_joint_nm':tau,'nominal_command_joint_nm':direction*ratio*RS05_NOMINAL_NM_PER_APK*current}
