#!/usr/bin/env python3
"""Read-only TCP measurement support. Never sends joint or motor commands."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np


def rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-4:
        raise ValueError('Expected a unit quaternion [x,y,z,w]')
    x,y,z,w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def solve(samples):
    if len(samples) < 4:
        raise ValueError('At least four fixed-point contact poses with different orientations required')
    keys = ('base_frame','tool_frame','opening_mm','point_id','model_sha256')
    for sample in samples:
        if any(sample.get(k) is None or sample[k] != samples[0].get(k) for k in keys):
            raise ValueError('All captures must use the same frames, opening, contact point and model')
    blocks, rhs = [], []
    for s in samples:
        p = np.asarray(s['translation_m'], dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError('Expected a finite translation in meters')
        blocks.append(np.hstack([rotation(s['quaternion_xyzw']), -np.eye(3)]))
        rhs.append(-p)
    a,b = np.vstack(blocks), np.concatenate(rhs)
    solution,_,rank,sv = np.linalg.lstsq(a,b,rcond=None)
    if rank != 6:
        raise ValueError('Insufficient orientation diversity: TCP is not identifiable')
    residual = (a@solution-b).reshape(-1,3)
    errors = np.linalg.norm(residual,axis=1)*1000
    return {**{k:samples[0][k] for k in keys},
            'status':'candidate_requires_independent_physical_validation',
            'tcp_translation_in_gripper_m':solution[:3].tolist(),
            'fixed_point_in_base_m':solution[3:].tolist(),
            'residual_rms_mm':float(np.sqrt(np.mean(errors**2))),
            'residual_max_mm':float(errors.max()),
            'condition_number':float(sv[0]/sv[-1]),
            'sample_count':len(samples),
            'tcp_orientation':'not_observed_by_pivot_calibration',
            'note':'Residual is fit consistency, not absolute positioning accuracy. No URDF is modified.'}


def write(path, data):
    # A measurement must not silently replace an existing record.
    with Path(path).open('x') as stream:
        json.dump(data,stream,indent=2,allow_nan=False)
        stream.write('\n')


def missing_tf_message(last_error, age):
    if age is None:
        detail = f'TF unavailable: {last_error}'
    elif age < 0:
        detail = f'TF timestamp is {-age:.3f}s in the future; check ROS clock/use_sim_time'
    else:
        detail = f'Latest TF is {age:.3f}s old (maximum 0.5s)'
    return (f'No fresh base_footprint -> gripper TF within 5 seconds. {detail}. '
            'Run feedback.launch.py and check /joint_states and /pallet_motor_state/status. '
            'Motor Studio alone does not publish ROS TF; manual mode needs new measured '
            'motor-angle input on /pallet_motor_positions. Do not share its serial port with Motor Studio.')


def capture(args):
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from ament_index_python.packages import get_package_share_directory
    if not np.isfinite(args.opening_mm) or args.opening_mm < 0:
        raise ValueError('opening-mm must be a finite nonnegative measured value')
    rclpy.init()
    node = rclpy.create_node('capture_tcp_reference')
    buffer = Buffer(); listener = TransformListener(buffer,node)
    try:
        deadline = time.monotonic()+5
        last_error = "No transform received"
        age = None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            try:
                tf = buffer.lookup_transform('base_footprint','gripper',Time())
            except TransformException as error:
                last_error = str(error)
                continue
            stamp = Time.from_msg(tf.header.stamp)
            age = (node.get_clock().now()-stamp).nanoseconds/1e9
            if not 0 <= age <= .5:
                continue
            t,q = tf.transform.translation, tf.transform.rotation
            model = Path(get_package_share_directory('pallet'))/'urdf/pallet_with_gripper.urdf'
            write(args.output,{'base_frame':'base_footprint','tool_frame':'gripper',
                'opening_mm':args.opening_mm,'point_id':args.point_id,
                'model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),
                'stamp_sec':tf.header.stamp.sec,'stamp_nanosec':tf.header.stamp.nanosec,
                'translation_m':[t.x,t.y,t.z], 'quaternion_xyzw':[q.x,q.y,q.z,q.w]})
            print(args.output);return
        raise TimeoutError(missing_tf_message(last_error, age))
    finally:
        node.destroy_node();rclpy.try_shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    c=commands.add_parser('capture',help='Record TF while a chosen rigid tool point touches a fixed reference')
    c.add_argument('--opening-mm',type=float,required=True)
    c.add_argument('--point-id',required=True)
    c.add_argument('--output',required=True)
    s=commands.add_parser('solve',help='Fit TCP translation from fixed-point captures; orientation needs separate measurement')
    s.add_argument('samples',nargs='+');s.add_argument('--output',required=True)
    args=parser.parse_args()
    try:
        if args.command=='capture':capture(args)
        else:
            result=solve([json.loads(Path(p).read_text()) for p in args.samples])
            write(args.output,result);print(json.dumps(result,indent=2))
    except (TimeoutError, OSError, ValueError) as error:
        parser.exit(1, f'Error: {error}\n')


if __name__=='__main__':
    main()
