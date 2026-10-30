#!/usr/bin/env python3
"""Latest six-axis FollowJointTrajectory bridge. Default DRY RUN, no device access."""
from pathlib import Path
import math
import json
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
from trajectory_msgs.msg import JointTrajectoryPoint
from serial_trajectory_core import NAMES,load_config,validate_goal,DryBackend,SerialBackend,ObserveBackend,tracking_violation


class Bridge(Node):
    def __init__(self):
        super().__init__('pallet_serial_trajectory_bridge')
        root=Path(get_package_share_directory('pallet'))
        self.declare_parameter('dry_run',True)
        self.declare_parameter('observe_only',False)
        self.declare_parameter('config_file',str(root/'config/serial_trajectory.yaml'))
        self.dry=self.get_parameter('dry_run').value
        self.observe=self.get_parameter('observe_only').value
        if self.observe and self.dry:raise ValueError('observe_only requires dry_run:=false')
        self.config,self.joints=load_config(self.get_parameter('config_file').value,root,live=not self.dry and not self.observe)
        self.io=threading.Lock();self.reservation=threading.Lock();self.closing=threading.Event()
        self.backend=DryBackend() if self.dry else (ObserveBackend(self.config,self.joints) if self.observe else SerialBackend(self.config,self.joints))
        self.cached=None;self.fault=None
        self.publisher=self.create_publisher(JointState,'/joint_states',10)
        self.status=self.create_publisher(String,'/pallet_serial_bridge/status',10)
        self.server=ActionServer(self,FJT,'/pallet_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute,goal_callback=self.goal,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        self.timer=self.create_timer(self.config['period_s'],self.publish_state)
        self.get_logger().warn('DRY RUN: no serial device access' if self.dry else ('OBSERVE ONLY: parameter reads; Execute rejected; no motor writes' if self.observe else 'LIVE SERIAL: exclusive owner; physical commissioning required'))

    def goal(self,request):
        if self.observe:
            self.get_logger().error('OBSERVE ONLY: execution disabled; no motor commands sent')
            return GoalResponse.REJECT
        try:
            validate_goal(request.trajectory,self.config,self.joints)
            for tolerance in list(request.path_tolerance)+list(request.goal_tolerance):
                if tolerance.name not in NAMES or not all(math.isfinite(v) for v in [tolerance.position,tolerance.velocity,tolerance.acceleration]):raise ValueError('Invalid tolerance')
                if tolerance.velocity or tolerance.acceleration:raise ValueError('Only position tolerances supported')
            if request.goal_time_tolerance.sec<0 or not 0<=request.goal_time_tolerance.nanosec<1000000000:raise ValueError('Invalid goal time tolerance')
        except ValueError as error:
            self.get_logger().error(str(error));return GoalResponse.REJECT
        if self.closing.is_set() or self.fault or not self.reservation.acquire(blocking=False):return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def sample(self):
        values=self.backend.sample();self.cached=list(values);return values

    def publish_state(self):
        if self.closing.is_set():return
        if self.fault:
            self.status.publish(String(data='FAULT_LATCHED: '+self.fault));return
        try:
            with self.io:
                if not self.reservation.locked():self.sample()
                if self.cached is None or time.monotonic()-self.backend.last_sample>self.config['watchdog_s']:
                    self.status.publish(String(data='STALE: no complete fresh joint sample'));return
                q=list(self.cached);age=time.monotonic()-self.backend.last_sample
            msg=JointState();msg.header.stamp=(self.get_clock().now()-Duration(seconds=age)).to_msg()
            msg.name=NAMES+['g_r','g_l'];msg.position=q+[0.,0.]
            self.publisher.publish(msg)
            mode='DRY_RUN' if self.dry else ('OBSERVE_ONLY' if self.observe else 'LIVE')
            timing=(f'; read_cycle_ms={self.backend.cycle_s*1000:.1f}; period_ms={self.config["period_s"]*1000:.1f}; cycle_overrun={self.backend.cycle_s>self.config["period_s"]}' if self.observe else '')
            if not self.dry and not self.observe:timing+=f'; feedback_retries={getattr(self.backend,"last_feedback_retries",0)}'
            self.status.publish(String(data=mode+' OK; gripper display placeholder, not measured'+timing))
        except Exception as error:
            self.cached=None;self.fault=str(error)
            self.status.publish(String(data='FEEDBACK ERROR: '+str(error)))
            self.get_logger().error(str(error))
            with self.io:
                try:
                    self.backend.stop()
                    self.get_logger().warn('STOP confirmed after feedback fault')
                except Exception as stop_error:self.get_logger().error(str(stop_error))

    @staticmethod
    def result(code,message):
        result=FJT.Result();result.error_code=code;result.error_string=message;return result

    def execute(self,handle):
        code=FJT.Result.SUCCESSFUL;message='';canceled=False
        try:
            names,points=validate_goal(handle.request.trajectory,self.config,self.joints)
            order=[names.index(n) for n in NAMES]
            points=[(t,[q[i] for i in order]) for t,q in points]
            with self.io:start=self.sample()
            start_error=max(abs(a-b) for a,b in zip(start,points[0][1]))
            if start_error>self.config['max_start_error_rad']:raise ValueError('First point too far from current state')
            if points[0][0]==0 and start_error>self.config['goal_tolerance_rad']:raise ValueError('Nonzero jump at time zero')
            if points[0][0]>0 and start_error/points[0][0]>self.config['velocity_rad_s']:raise ValueError('Initial segment too fast')
            if self.config['max_joint_travel_rad'] is not None and any(abs(q-s)>self.config['max_joint_travel_rad'] for _,qs in points for q,s in zip(qs,start)):
                raise ValueError('Trajectory exceeds commissioning travel envelope')
            should_cancel=lambda:handle.is_cancel_requested or self.closing.is_set()
            if should_cancel():raise InterruptedError('Canceled before preparation')
            with self.io:self.backend.prepare(start,should_cancel)
            began=time.monotonic();before=list(start);before_t=0.;last_q=start
            final_t,final=points[-1];settle=self.config['settle_timeout_s']
            requested=handle.request.goal_time_tolerance.sec+handle.request.goal_time_tolerance.nanosec*1e-9
            if requested>0:settle=min(settle,requested)
            path_cap={n:self.config['max_path_error_rad'] for n in NAMES}
            goal_cap={n:self.config['goal_tolerance_rad'] for n in NAMES}
            for items,caps in [(handle.request.path_tolerance,path_cap),(handle.request.goal_tolerance,goal_cap)]:
                for item in items:
                    if item.position>0:caps[item.name]=min(caps[item.name],item.position)
            reached_since=None;index=0;last_trace=-1.
            while time.monotonic()-began<=final_t+settle:
                tick=time.monotonic();elapsed=tick-began
                if should_cancel():raise InterruptedError('Trajectory canceled')
                while index<len(points) and elapsed>=points[index][0]:
                    before_t,before=points[index];index+=1
                if index<len(points):
                    t,target=points[index];ratio=max(0.,min(1.,(elapsed-before_t)/max(t-before_t,1e-9)))
                    desired=[a+ratio*(b-a) for a,b in zip(before,target)]
                else:desired=final
                with self.io:
                    self.backend.command(desired);actual=self.sample()
                if time.monotonic()-tick>self.config['period_s']:raise RuntimeError('Serial control cycle overrun')
                feedback=FJT.Feedback();feedback.header.stamp=self.get_clock().now().to_msg();feedback.joint_names=names
                desired_by=dict(zip(NAMES,desired));actual_by=dict(zip(NAMES,actual))
                feedback.desired=JointTrajectoryPoint(positions=[desired_by[n] for n in names])
                feedback.actual=JointTrajectoryPoint(positions=[actual_by[n] for n in names])
                feedback.error=JointTrajectoryPoint(positions=[desired_by[n]-actual_by[n] for n in names])
                handle.publish_feedback(feedback)
                violation=tracking_violation(actual,desired,path_cap)
                if violation or elapsed-last_trace>=.5:
                    self.get_logger().info('TRACK '+json.dumps({'elapsed_s':elapsed,'desired_rad':desired,'actual_rad':actual,
                        'feedback':getattr(self.backend,'last_values',[])},allow_nan=False))
                    last_trace=elapsed
                if violation:raise RuntimeError(violation+f'; elapsed={elapsed:.3f}s')
                if elapsed>=final_t and all(abs(a-d)<=goal_cap[n] for n,a,d in zip(NAMES,actual,final)):
                    reached_since=reached_since if reached_since is not None else time.monotonic()
                    if time.monotonic()-reached_since>=.2:message='Goal reached; stop confirmed';break
                else:reached_since=None
                time.sleep(max(0,self.config['period_s']-(time.monotonic()-tick)))
            else:raise RuntimeError('Goal tolerance/settling timeout')
        except InterruptedError as error:canceled=handle.is_cancel_requested;code=FJT.Result.PATH_TOLERANCE_VIOLATED;message=str(error)
        except ValueError as error:code=FJT.Result.INVALID_GOAL;message=str(error)
        except Exception as error:
            code=FJT.Result.PATH_TOLERANCE_VIOLATED;message=str(error)
            self.get_logger().error(message)
        finally:
            try:
                with self.io:self.backend.stop()
            except Exception as error:
                code=FJT.Result.PATH_TOLERANCE_VIOLATED;message+='; STOP NOT CONFIRMED: '+str(error);canceled=False
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
