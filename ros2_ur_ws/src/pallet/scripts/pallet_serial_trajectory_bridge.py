#!/usr/bin/env python3
"""Latest six-axis FollowJointTrajectory bridge. Default DRY RUN, no device access."""
from pathlib import Path
import copy
import math
import json
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory as FJT
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from position_trajectory import PositionTrajectory, build_checked_trajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from serial_trajectory_core import NAMES,load_config,validate_goal,DryBackend,SerialBackend,ObserveBackend,tracking_violation,FeedbackUnavailable,HealthGate


class GoalToleranceError(RuntimeError):pass


class Bridge(Node):
    def __init__(self):
        super().__init__('pallet_serial_trajectory_bridge')
        root=Path(get_package_share_directory('pallet'))
        self.declare_parameter('dry_run',True)
        self.declare_parameter('observe_only',False)
        self.declare_parameter('config_file',str(root/'config/serial_trajectory.yaml'))
        self.declare_parameter('adaptive_gripper',False)
        self.adaptive=self.get_parameter('adaptive_gripper').value
        self.names=list(NAMES)
        self.mapping=None
        self.dry=self.get_parameter('dry_run').value
        self.observe=self.get_parameter('observe_only').value
        if self.observe and self.dry:raise ValueError('observe_only requires dry_run:=false')
        if not self.dry and self.get_parameter('use_sim_time').value:
            raise ValueError('Physical robot feedback requires use_sim_time:=false')
        self.config,self.joints=load_config(self.get_parameter('config_file').value,root,live=not self.dry and not self.observe)
        if self.adaptive:
            from adaptive_motor_mapping import AdaptiveMotorMapping,load_motor_profile
            from single_axis_position_core import Joint
            profile=load_motor_profile(root/'config/adaptive_gripper_motor.yaml')
            if not self.dry and not self.observe:
                expected=profile.get('expected_current_limit_a')
                if profile.get('commissioned') is not True or type(expected) not in (int,float) or not math.isfinite(expected) or expected<=0:
                    raise ValueError('ID7 LIVE requires commissioned profile and verified expected_current_limit_a; no serial device opened')
            self.names.append('gripper_motor')
            self.joints.append(Joint('gripper_motor',7,1,0.,1.,-12.,12.))
            self.config.setdefault('axis_limits',{})['gripper_motor']=profile
            self.config['current_trip_a']['gripper_motor']=None
            self.config.setdefault('expected_current_limit_a',{})['gripper_motor']=profile['expected_current_limit_a']
            self.mapping=AdaptiveMotorMapping(root/'config/adaptive_gripper/geometry.json')
        self.validation_pool=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'))
        self.io=threading.Lock();self.reservation=threading.Lock();self.closing=threading.Event()
        self.backend=DryBackend() if self.dry else (ObserveBackend(self.config,self.joints) if self.observe else SerialBackend(self.config,self.joints))
        if self.dry and self.adaptive:self.backend.positions.append(0.)
        self.cached=None
        self.hold_target=None
        self.hold_caps=None
        self.health=HealthGate(1 if self.dry else self.config['readiness_samples'],min(self.config['watchdog_s'],2*self.config['period_s']))
        if not self.dry and not self.observe:
            self.health.stop_confirmed=False
            try:self.backend.stop();self.health.stop_confirmed=True
            except Exception as error:self.health.latch(str(error),False)
        self.publisher=self.create_publisher(JointState,'/joint_states',10)
        self.status=self.create_publisher(String,'/pallet_serial_bridge/status',10)
        self.server=ActionServer(self,FJT,'/pallet_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute,goal_callback=self.goal,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        self.gripper_server=None
        if self.adaptive:
            self.gripper_server=ActionServer(self,FJT,'/pallet_gripper_controller/follow_joint_trajectory',execute_callback=self.execute,goal_callback=self.goal,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        self.reset_service=self.create_service(Trigger,'/pallet_serial_bridge/reset_fault',self.reset_fault,callback_group=ReentrantCallbackGroup())
        self.stop_service=self.create_service(Trigger,'/pallet_serial_bridge/stop_hold',self.reset_fault,callback_group=ReentrantCallbackGroup())
        self.timer=self.create_timer(self.config['period_s'],self.publish_state)
        self.get_logger().warn('DRY RUN: no serial device access' if self.dry else ('OBSERVE ONLY: parameter reads; Execute rejected; no motor writes' if self.observe else 'LIVE SERIAL: exclusive owner; physical commissioning required'))

    def goal(self,request):
        if self.observe:
            self.get_logger().error('OBSERVE ONLY: execution disabled; no motor commands sent')
            return GoalResponse.REJECT
        if len(self.get_publishers_info_by_topic('/joint_states'))!=1:
            self.get_logger().error('Execution requires this bridge to be the only /joint_states publisher; stop other robot state/feedback launches')
            return GoalResponse.REJECT
        try:
            Bridge.checked_trajectory(self,request.trajectory)
            for tolerance in list(request.path_tolerance)+list(request.goal_tolerance):
                if tolerance.name not in self.names or not all(math.isfinite(v) for v in [tolerance.position,tolerance.velocity,tolerance.acceleration]):raise ValueError('Invalid tolerance')
                if tolerance.velocity or tolerance.acceleration:raise ValueError('Only position tolerances supported')
            if request.goal_time_tolerance.sec<0 or not 0<=request.goal_time_tolerance.nanosec<1000000000:raise ValueError('Invalid goal time tolerance')
        except ValueError as error:
            self.get_logger().error(str(error));return GoalResponse.REJECT
        if self.closing.is_set() or not (self.health.ready(time.monotonic()) or (getattr(self,'hold_target',None) is not None and self.health.fault is None and self.health.good>=self.health.required and time.monotonic()-self.health.last_sample<=self.health.max_age)):
            self.get_logger().error(f'Execution unavailable: fault={self.health.fault}; healthy_samples={self.health.good}/{self.health.required}; wait for READY or use reset_fault after resolving the fault')
            return GoalResponse.REJECT
        if not self.reservation.acquire(blocking=False):return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def sample(self):
        values=self.backend.sample();self.cached=list(values)
        self.health.sample_ok(self.backend.last_sample)
        # Publish immediately from the measured sample, including during action
        # execution. Never retimestamp a cached angle as newly measured.
        age=max(0.,time.monotonic()-self.backend.last_sample)
        msg=JointState();msg.header.stamp=(self.get_clock().now()-Duration(seconds=age)).to_msg()
        if self.mapping:
            state=self.mapping.measured_state(self.names[:6],list(values)[:6],values[6])
            msg.name=list(state);msg.position=list(state.values())
        else:
            msg.name=self.names+['g_r','g_l'];msg.position=list(values)+[0.,0.]
        self.publisher.publish(msg)
        return values

    def publish_status(self):
        mode='DRY_RUN' if self.dry else ('OBSERVE_ONLY' if self.observe else 'LIVE')
        state=('FAULT_LATCHED: '+self.health.fault if self.health.fault else
               ('EXECUTING_OR_RESETTING' if self.reservation.locked() else ('HOLDING' if getattr(self,'hold_target',None) is not None else ('READY' if self.health.ready(time.monotonic()) else 'NOT_READY'))))
        self.status.publish(String(data=f'{mode} {state}; healthy_samples={self.health.good}/{self.health.required}; '
            f'stop_confirmed={self.health.stop_confirmed}; read_cycle_ms={getattr(self.backend,"cycle_s",0.)*1000:.1f}; '
            f'feedback_retries={getattr(self.backend,"last_feedback_retries",0)}; gripper state: {'ID7 measured/model-derived' if self.adaptive else 'legacy placeholder'}'))

    def publish_state(self):
        if self.closing.is_set():return
        try:
            with self.io:
                # Goal reservation includes validation/dispatch time. Keep the
                # active hold alive until execute transfers ownership under io.
                if not self.reservation.locked() or getattr(self,'hold_target',None) is not None:
                    if getattr(self,'hold_target',None) is not None:
                        tick=time.monotonic()
                        try:
                            if tick-self.health.last_sample>self.health.max_age:
                                raise RuntimeError('Final hold feedback stale')
                            self.backend.command(self.hold_target)
                            actual=self.sample()
                            violation=tracking_violation(actual,self.hold_target,self.hold_caps)
                            if violation:raise RuntimeError('Final hold: '+violation)
                            if time.monotonic()-tick>self.config['period_s']:
                                raise RuntimeError('Final hold control cycle overrun')
                        except Exception as error:
                            # Active hold must stop on even transient missing feedback.
                            raise RuntimeError(str(error)) from error
                    elif self.health.stop_confirmed:self.sample()
            self.publish_status()
        except FeedbackUnavailable as error:
            # At idle all axes are stopped. Recover telemetry automatically,
            # but require a full run of healthy samples before allowing motion.
            self.cached=None;self.health.sample_failed()
            self.status.publish(String(data='NOT_READY: '+str(error)))
            self.get_logger().warn(str(error))
        except Exception as error:
            self.cached=None
            with self.io:
                self.hold_target=None
                stopped=False
                try:self.backend.stop();stopped=True
                except Exception as stop_error:self.get_logger().error(str(stop_error))
                self.health.latch(str(error),stopped)
            self.get_logger().error(str(error));self.publish_status()

    def reset_fault(self,request,response):
        if not self.reservation.acquire(blocking=False):
            response.success=False;response.message='Active goal or reset in progress';return response
        try:
            if self.observe:raise RuntimeError('Observe-only mode has no actuation/reset writes')
            with self.io:
                self.hold_target=None
                self.health.stop_confirmed=False
                self.backend.stop();self.health.stop_confirmed=True
                self.health.sample_failed()
                for _ in range(self.health.required):
                    if self.closing.is_set():raise RuntimeError('Bridge closing')
                    tick=time.monotonic();self.sample()
                    time.sleep(max(0.,self.config['period_s']-(time.monotonic()-tick)))
                self.health.reset(time.monotonic())
            response.success=True;response.message='STOP confirmed; fresh feedback verified; READY (no motor enabled)'
        except Exception as error:
            self.health.latch(str(error),self.health.stop_confirmed)
            response.success=False;response.message=str(error)
        finally:self.reservation.release()
        self.publish_status();return response

    @staticmethod
    def result(code,message):
        result=FJT.Result();result.error_code=code;result.error_string=message;return result

    def checked_trajectory(self,trajectory):
        if getattr(self,'adaptive',False):
            requested=list(trajectory.joint_names)
            if requested not in [self.names[:6],['gripper_motor'],self.names]:
                if len(requested)!=6 or set(requested)!=set(self.names[:6]):
                    raise ValueError('Expected arm J1..J6 or gripper_motor group')
            if requested!=self.names:
                if self.cached is None:raise ValueError('Complete feedback required before group command')
                trajectory=copy.deepcopy(trajectory)
                snapshot=dict(zip(self.names,self.cached))
                for point in trajectory.points:
                    if len(point.positions)!=len(requested):raise ValueError('Invalid group position count')
                    values=dict(zip(requested,point.positions))
                    point.positions=[values.get(n,snapshot[n]) for n in self.names]
                    for field in ['velocities','accelerations']:
                        data=list(getattr(point,field))
                        if data:
                            if len(data)!=len(requested):raise ValueError('Invalid group derivative count')
                            lookup=dict(zip(requested,data))
                            setattr(point,field,[lookup.get(n,0.) for n in self.names])
                trajectory.joint_names=list(self.names)
        pool=getattr(self,'validation_pool',None)
        if pool is None:  # Offline unit-test nodes have no worker lifecycle.
            names,points=validate_goal(trajectory,self.config,self.joints)
            return names,points,PositionTrajectory(trajectory,self.joints,self.config)
        future=None
        try:
            future=pool.submit(build_checked_trajectory,trajectory,self.joints,self.config)
            return future.result(timeout=10.)
        except Exception as error:
            if future is not None:future.cancel()
            raise ValueError(f'Trajectory validation failed: {error}') from error

    def start_snapshot(self):
        # Called under io. Holding already polls every period; avoid an extra
        # 18-register cycle between the hold tick and the first execution tick.
        if getattr(self,'hold_target',None) is not None:
            if self.health.fault:raise RuntimeError(self.health.fault)
            if self.cached is None or time.monotonic()-self.health.last_sample>self.health.max_age:
                raise RuntimeError('Cannot continue from stale hold feedback')
            return list(self.cached)
        return self.sample()

    def execute(self,handle):
        code=FJT.Result.SUCCESSFUL;message='';canceled=False;execution_fault=False;keep_holding=False
        try:
            direct_gripper=(getattr(self,'adaptive',False) and
                            list(handle.request.trajectory.joint_names)==['gripper_motor'])
            names,points,trajectory=Bridge.checked_trajectory(self,handle.request.trajectory)
            order=[names.index(n) for n in self.names]
            points=[(t,[q[i] for i in order]) for t,q in points]
            with self.io:start=Bridge.start_snapshot(self)
            start_error=max(abs(a-b) for a,b in zip(start,points[0][1]))
            if start_error>self.config['max_start_error_rad']:raise ValueError('First point too far from current state')
            if points[0][0]==0 and start_error>self.config['goal_tolerance_rad']:raise ValueError('Nonzero jump at time zero')
            if points[0][0]>0 and start_error/points[0][0]>self.config['velocity_rad_s']:raise ValueError('Initial segment too fast')
            if self.config['max_joint_travel_rad'] is not None and any(abs(q-s)>self.config['max_joint_travel_rad'] for _,qs in points for q,s in zip(qs,start)):
                raise ValueError('Trajectory exceeds commissioning travel envelope')
            should_cancel=lambda:handle.is_cancel_requested or self.closing.is_set()
            if should_cancel():raise InterruptedError('Canceled before preparation')
            with self.io:
                if self.health.fault:raise RuntimeError(self.health.fault)
                continuing=getattr(self,'hold_target',None) is not None
                self.hold_target=None
                self.health.stop_confirmed=False
                if not continuing:self.backend.prepare(start,should_cancel)
            began=time.monotonic()
            final_t,final=points[-1];settle=self.config['settle_timeout_s']
            requested=handle.request.goal_time_tolerance.sec+handle.request.goal_time_tolerance.nanosec*1e-9
            if requested>0:settle=min(settle,requested)
            path_cap={n:self.config['max_path_error_rad'] for n in self.names}
            goal_cap={n:self.config['goal_tolerance_rad'] for n in self.names}
            for items,caps in [(handle.request.path_tolerance,path_cap),(handle.request.goal_tolerance,goal_cap)]:
                for item in items:
                    if item.position>0:caps[item.name]=min(caps[item.name],item.position)
            reached_since=None;holding_since=None;hold_s=self.config['final_hold_s'];last_trace=-1.
            trace_axes = {name: i for i, name in enumerate(self.names)
                          if name in ('J2', 'J3') and name in handle.request.trajectory.joint_names}
            while time.monotonic()-began<=final_t+settle+hold_s:
                tick=time.monotonic();elapsed=tick-began
                if should_cancel():raise InterruptedError('Trajectory canceled')
                # A gripper-only goal is an endpoint move. The drive generates
                # its own ramp using the configured speed/acceleration limits.
                # Refresh the same endpoint for the watchdog; do not stream a
                # second interpolated ramp on top of the drive's ramp.
                desired=list(final) if direct_gripper else trajectory.at(elapsed)
                with self.io:
                    command_started = time.monotonic()
                    self.backend.command(desired)
                    actual = self.sample()
                if time.monotonic()-tick>self.config['period_s']:raise RuntimeError('Serial control cycle overrun')
                feedback=FJT.Feedback();feedback.header.stamp=self.get_clock().now().to_msg();feedback.joint_names=names
                desired_by=dict(zip(self.names,desired));actual_by=dict(zip(self.names,actual))
                feedback.desired=JointTrajectoryPoint(positions=[desired_by[n] for n in names])
                feedback.actual=JointTrajectoryPoint(positions=[actual_by[n] for n in names])
                feedback.error=JointTrajectoryPoint(positions=[desired_by[n]-actual_by[n] for n in names])
                handle.publish_feedback(feedback)
                monitored=[i for i,n in enumerate(self.names) if not (direct_gripper and n=='gripper_motor')]
                violation=tracking_violation([actual[i] for i in monitored],[desired[i] for i in monitored],
                    {self.names[i]:path_cap[self.names[i]] for i in monitored})
                if violation or elapsed-last_trace>=.5:
                    self.get_logger().info('TRACK '+json.dumps({'elapsed_s':elapsed,'desired_rad':desired,'actual_rad':actual,
                        'feedback':getattr(self.backend,'last_values',[])},allow_nan=False))
                    last_trace=elapsed
                if trace_axes:
                    samples = {
                        sample['joint']: sample
                        for sample in getattr(self.backend, 'last_values', [])
                        if sample['joint'] in trace_axes
                    }
                    self.get_logger().info('ARM_CYCLE '+json.dumps({
                        'command_start_s': command_started,
                        'elapsed_s': elapsed,
                        'phase': 'motion' if elapsed < final_t else 'endpoint',
                        'axes': [
                            {'joint': name, 'desired_rad': desired[index],
                             'actual_rad': actual[index], 'feedback': samples.get(name)}
                            for name, index in trace_axes.items()
                        ],
                    }, allow_nan=False))
                if violation:raise RuntimeError(violation+f'; elapsed={elapsed:.3f}s')
                if (direct_gripper or elapsed>=final_t) and all(abs(a-d)<=goal_cap[n] for n,a,d in zip(self.names,actual,final)):
                    reached_since=reached_since if reached_since is not None else time.monotonic()
                    if holding_since is None and time.monotonic()-reached_since>=.2:
                        holding_since=time.monotonic()
                        self.get_logger().info(f'Goal reached; holding final position for {hold_s:.1f}s')
                    if holding_since is not None and time.monotonic()-holding_since>=hold_s:
                        keep_holding=self.config.get('hold_after_success',False)
                        if keep_holding:
                            with self.io:
                                self.hold_target=list(final)
                                self.hold_caps={n:self.config['hold_tolerance_rad'] for n in self.names}
                            message='Goal reached; continuous position hold active'
                        else:message=f'Goal reached; held final position for {hold_s:.1f}s; stop confirmed'
                        break
                else:
                    reached_since=None
                    if holding_since is not None:raise GoalToleranceError('Goal tolerance violated during final position hold: '+str(tracking_violation(actual,final,goal_cap)))
                if holding_since is None and time.monotonic()-began>final_t+settle:
                    raise GoalToleranceError('Goal tolerance/settling timeout')
                time.sleep(max(0,self.config['period_s']-(time.monotonic()-tick)))
            else:raise GoalToleranceError('Goal tolerance/settling timeout')
        except InterruptedError as error:canceled=handle.is_cancel_requested;code=FJT.Result.PATH_TOLERANCE_VIOLATED;message=str(error)
        except ValueError as error:
            code=FJT.Result.INVALID_GOAL;message=str(error);execution_fault=not self.health.stop_confirmed
        except GoalToleranceError as error:
            code=FJT.Result.GOAL_TOLERANCE_VIOLATED;message=str(error);execution_fault=True
            self.get_logger().error(message)
        except Exception as error:
            code=FJT.Result.PATH_TOLERANCE_VIOLATED;message=str(error);execution_fault=True
            self.get_logger().error(message)
        finally:
            try:
                with self.io:
                    if not keep_holding:
                        self.hold_target=None
                        self.backend.stop();self.health.stop_confirmed=True
            except Exception as error:
                code=FJT.Result.PATH_TOLERANCE_VIOLATED;message+='; STOP NOT CONFIRMED: '+str(error);canceled=False;execution_fault=True
                self.health.stop_confirmed=False
            if execution_fault:self.health.latch(message,self.health.stop_confirmed)
            elif not keep_holding:self.health.sample_failed()
            self.reservation.release()
        if canceled:handle.canceled()
        elif code==FJT.Result.SUCCESSFUL:handle.succeed()
        else:handle.abort()
        return self.result(code,message)

    def destroy_node(self):
        self.closing.set();self.timer.cancel()
        acquired=self.reservation.acquire(timeout=3)
        if not acquired:raise RuntimeError('Active goal failed to stop; backend not closed')
        try:
            with self.io:
                try:self.backend.stop()
                finally:self.backend.close()
            self.validation_pool.shutdown(wait=False,cancel_futures=True)
            if self.gripper_server:self.gripper_server.destroy()
            self.server.destroy();return super().destroy_node()
        finally:self.reservation.release()


def main():
    rclpy.init();node=None;executor=MultiThreadedExecutor(num_threads=4)
    try:
        node=Bridge();executor.add_node(node);executor.spin()
    except KeyboardInterrupt:pass
    finally:
        if node:node.closing.set()
        executor.shutdown(timeout_sec=4)
        if node:node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
