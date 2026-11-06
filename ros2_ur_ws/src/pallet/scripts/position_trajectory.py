"""Time-preserving position interpolation for MoveIt JointTrajectory points.

Position only: linear; position/velocity: cubic; position/velocity/acceleration:
quintic. Check continuous segment extrema before permitting motor commands.
"""
import bisect
import math
import numpy as np
from numpy.polynomial import polynomial as poly


def extrema_values(coefficients,derivative=0):
    c=poly.polyder(coefficients,derivative)
    roots=poly.polyroots(poly.polyder(c))
    times=[0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<1]
    return [float(poly.polyval(t,c)) for t in times]


class PositionTrajectory:
    def __init__(self,trajectory,joints,config):
        self.names=[j.name for j in joints]
        order=[list(trajectory.joint_names).index(n) for n in self.names]
        points=trajectory.points
        self.times=[p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in points]
        self.positions=[[p.positions[i] for i in order] for p in points]
        self.segments=[]
        for left,right,p0,p1,t0,t1 in zip(points,points[1:],self.positions,self.positions[1:],self.times,self.times[1:]):
            dt=t1-t0;segment=[]
            if bool(left.velocities)!=bool(right.velocities) or bool(left.accelerations)!=bool(right.accelerations):
                raise ValueError('Inconsistent trajectory derivative fields')
            if (left.accelerations or right.accelerations) and not (left.velocities and right.velocities):
                raise ValueError('Acceleration interpolation requires velocity fields')
            for axis,(joint,a,b) in enumerate(zip(joints,p0,p1)):
                i=order[axis]
                if left.velocities and right.velocities:
                    v0=left.velocities[i]*dt;v1=right.velocities[i]*dt
                    if left.accelerations and right.accelerations:
                        c0=a;c1=v0;c2=left.accelerations[i]*dt*dt/2
                        dp=b-c0-c1-c2;dv=v1-c1-2*c2;da=right.accelerations[i]*dt*dt-2*c2
                        c=[c0,c1,c2,10*dp-4*dv+da/2,-15*dp+7*dv-da,6*dp-3*dv+da/2]
                    else:c=[a,v0,3*(b-a)-2*v0-v1,-2*(b-a)+v0+v1]
                else:c=[a,b-a]
                if not all(math.isfinite(v) for v in c):raise ValueError('Nonfinite interpolation coefficients')
                if any(v<joint.lower-1e-9 or v>joint.upper+1e-9 for v in extrema_values(c)):
                    raise ValueError(f'{joint.name}: interpolated position outside URDF bounds')
                for derivative,cap,label in [(1,config['velocity_rad_s'],'velocity'),(2,config['acceleration_rad_s2'],'acceleration')]:
                    cap=config.get('axis_limits',{}).get(joint.name,{}).get('velocity_rad_s' if derivative==1 else 'acceleration_rad_s2',cap)
                    peak=max(abs(v)/dt**derivative for v in extrema_values(c,derivative))
                    if peak>cap*1.001:raise ValueError(f'{joint.name}: interpolated {label} {peak:.6f} exceeds {cap}')
                travel=config['max_joint_travel_rad']
                if travel is not None and any(abs(v-self.positions[0][axis])>travel+1e-9 for v in extrema_values(c)):
                    raise ValueError(f'{joint.name}: interpolated path exceeds travel envelope')
                segment.append(c)
            self.segments.append(segment)
        # MoveIt TOTG may end at zero velocity with nonzero acceleration.
        # Preserve that acceleration up to the last knot; hold its position
        # afterwards, as a position controller does at trajectory completion.
        if any(abs(v)>1e-8 for v in points[-1].velocities):
            raise ValueError('Final velocities must be zero for position hold')

    def at(self,elapsed):
        if elapsed<=self.times[0]:return list(self.positions[0])
        if elapsed>=self.times[-1]:return list(self.positions[-1])
        index=bisect.bisect_right(self.times,elapsed)-1
        u=(elapsed-self.times[index])/(self.times[index+1]-self.times[index])
        return [float(poly.polyval(u,c)) for c in self.segments[index]]


def build_checked_trajectory(trajectory, joints, config):
    """Picklable worker entry: all trajectory checks, no hardware access."""
    from serial_trajectory_core import validate_goal
    names, points = validate_goal(trajectory, config, joints)
    return names, points, PositionTrajectory(trajectory, joints, config)
