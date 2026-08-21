#!/usr/bin/env python3
"""Run the gripperless, torque-limited Pallet articulation as a ROS 2 endpoint."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from pallet_isaac_gripperless_physics_test import (
    DEFAULT_CONFIG,
    DEFAULT_URDF,
    EXPECTED_JOINTS,
    load_json,
    make_importable_urdf,
    read_urdf_limits,
    validate_config,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--smoke-test-seconds", type=float, default=0.0)
    parser.add_argument("--output", type=Path, default=Path("/tmp/pallet_isaac_moveit_report.json"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_json(args.config.resolve())
    limits = read_urdf_limits(args.urdf.resolve())
    validate_config(config, limits)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    patched_urdf = None
    report = {
        "test": "pallet_isaac_moveit_bridge",
        "passed": False,
        "joints": list(EXPECTED_JOINTS),
        "received_commands": 0,
    }
    try:
        import numpy as np
        import omni.kit.commands
        import omni.usd
        import rclpy
        from pxr import UsdPhysics
        from sensor_msgs.msg import JointState

        from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.types import ArticulationAction

        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        patched_urdf = make_importable_urdf(args.urdf.resolve())
        physics_dt = float(config["physics_dt"])
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=max(physics_dt, 1.0 / 60.0),
            backend="numpy",
        )
        world.scene.add_default_ground_plane()

        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        if not status:
            raise RuntimeError("URDFCreateImportConfig failed")
        import_config.merge_fixed_joints = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = True
        import_config.distance_scale = 1.0
        import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
        status, prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(patched_urdf),
            import_config=import_config,
            get_articulation_root=True,
        )
        if not status:
            raise RuntimeError("URDFParseAndImportFile failed")

        stage = omni.usd.get_context().get_stage()
        configured_drives = set()
        for prim in stage.Traverse():
            name = prim.GetName()
            if name not in EXPECTED_JOINTS or not prim.HasAPI(UsdPhysics.DriveAPI):
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            drive.GetTypeAttr().Set("force")
            drive.GetMaxForceAttr().Set(limits[name]["effort"])
            drive.GetStiffnessAttr().Set(float(config["joints"][name]["stiffness"]))
            drive.GetDampingAttr().Set(float(config["joints"][name]["damping"]))
            configured_drives.add(name)
        if configured_drives != set(EXPECTED_JOINTS):
            raise RuntimeError(f"Configured drives do not match the five-axis model: {configured_drives}")

        robot = world.scene.add(SingleArticulation(prim_path=prim_path, name="pallet_moveit"))
        world.reset()
        dof_names = tuple(robot.dof_names)
        if set(dof_names) != set(EXPECTED_JOINTS) or len(dof_names) != len(EXPECTED_JOINTS):
            raise RuntimeError(f"Unexpected Isaac joints: {dof_names}")
        kps = np.array([config["joints"][name]["stiffness"] for name in dof_names])
        kds = np.array([config["joints"][name]["damping"] for name in dof_names])
        rated_efforts = np.array([limits[name]["effort"] for name in dof_names])
        controller = robot.get_articulation_controller()
        controller.set_gains(kps=kps, kds=kds, save_to_usd=False)
        target = np.zeros(len(dof_names), dtype=float)
        robot.set_joint_positions(target)
        robot.set_joint_velocities(np.zeros(len(dof_names), dtype=float))

        rclpy.init(args=None)
        node = rclpy.create_node("pallet_isaac_moveit_endpoint")
        state_pub = node.create_publisher(JointState, "/isaac_joint_states", 10)

        def on_command(msg: JointState) -> None:
            nonlocal target
            if len(msg.name) != len(msg.position):
                node.get_logger().error("Rejected joint command with mismatched names/positions")
                return
            updated = target.copy()
            for name, position in zip(msg.name, msg.position):
                if name not in dof_names:
                    node.get_logger().error(f"Rejected unknown joint: {name}")
                    return
                updated[dof_names.index(name)] = float(position)
            target = updated
            report["received_commands"] += 1

        node.create_subscription(JointState, "/isaac_joint_commands", on_command, 10)
        started = time.monotonic()
        max_effort_ratio = np.zeros(len(dof_names), dtype=float)
        max_measured_effort = np.zeros(len(dof_names), dtype=float)
        while simulation_app.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            controller.apply_action(ArticulationAction(joint_positions=target))
            world.step(render=not args.headless)
            applied = np.abs(np.asarray(robot.get_applied_joint_efforts(), dtype=float))
            measured = np.abs(np.asarray(robot.get_measured_joint_efforts(), dtype=float))
            max_effort_ratio = np.maximum(max_effort_ratio, applied / rated_efforts)
            max_measured_effort = np.maximum(max_measured_effort, measured)
            state = JointState()
            state.header.stamp = node.get_clock().now().to_msg()
            state.name = list(dof_names)
            state.position = [float(value) for value in robot.get_joint_positions()]
            state.velocity = [float(value) for value in robot.get_joint_velocities()]
            state.effort = [float(value) for value in measured]
            state_pub.publish(state)
            if args.smoke_test_seconds > 0 and time.monotonic() - started >= args.smoke_test_seconds:
                break

        report.update(
            {
                "passed": True,
                "drive_mode": "force",
                "rated_effort_nm": dict(zip(dof_names, rated_efforts.tolist())),
                "max_applied_effort_ratio": dict(zip(dof_names, max_effort_ratio.tolist())),
                "max_measured_effort_nm": dict(zip(dof_names, max_measured_effort.tolist())),
                "final_position_rad": dict(zip(dof_names, robot.get_joint_positions().tolist())),
                "final_target_rad": dict(zip(dof_names, target.tolist())),
            }
        )
        node.destroy_node()
        rclpy.shutdown()
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
        if patched_urdf is not None:
            patched_urdf.unlink(missing_ok=True)
        simulation_app.close()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
