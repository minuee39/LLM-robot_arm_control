#!/usr/bin/env python3
"""Validate the gripperless Pallet arm under gravity and rated joint torque limits.

Run this file with Isaac Sim's python.sh, not the system Python.  It imports the
five-axis Pallet URDF at runtime, forces every revolute drive into force mode,
and clamps it to the URDF effort limit.  The process exits with status 0 only
when all configured poses settle within the position/velocity tolerances and
no commanded effort exceeds its rated limit.
"""

from __future__ import annotations

import argparse
import json
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


SCRIPT_PATH = Path(__file__).resolve()


def resolve_package_root() -> Path:
    source_root = SCRIPT_PATH.parents[1]
    if (source_root / "urdf" / "pallet.urdf").is_file():
        return source_root
    installed_root = SCRIPT_PATH.parents[2] / "share" / "pallet"
    if (installed_root / "urdf" / "pallet.urdf").is_file():
        return installed_root
    raise FileNotFoundError("Could not locate the Pallet package urdf directory")


PACKAGE_ROOT = resolve_package_root()
DEFAULT_URDF = PACKAGE_ROOT / "urdf" / "pallet.urdf"
DEFAULT_CONFIG = PACKAGE_ROOT / "config" / "isaac_gripperless_physics_test.json"
EXPECTED_JOINTS = ("j1", "j2", "j3", "j4", "j5")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/pallet_gripperless_physics_report.json"),
    )
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Validate the gripperless URDF and test config without starting Isaac Sim.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def read_urdf_limits(path: Path) -> dict[str, dict[str, float]]:
    root = ET.parse(path).getroot()
    named_elements = [
        element.attrib.get("name", "").lower()
        for element in root.findall("link") + root.findall("joint")
    ]
    gripper_elements = [name for name in named_elements if "gripper" in name or "finger" in name]
    if gripper_elements:
        raise ValueError(f"This experiment requires a gripperless URDF; found {gripper_elements}")
    limits: dict[str, dict[str, float]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        if name not in EXPECTED_JOINTS:
            continue
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"URDF joint {name} has no <limit> element")
        limits[name] = {
            "effort": float(limit.attrib["effort"]),
            "velocity": float(limit.attrib["velocity"]),
            "lower": float(limit.attrib["lower"]),
            "upper": float(limit.attrib["upper"]),
        }
    if set(limits) != set(EXPECTED_JOINTS):
        raise ValueError(
            f"Expected exactly the gripperless joints {EXPECTED_JOINTS}; found {tuple(limits)}"
        )
    return limits


def validate_config(config: dict[str, Any], limits: dict[str, dict[str, float]]) -> None:
    if set(config["joints"]) != set(EXPECTED_JOINTS):
        raise ValueError("Config must contain exactly j1, j2, j3, j4 and j5")
    for phase in config["phases"]:
        targets = phase["target_rad"]
        if set(targets) != set(EXPECTED_JOINTS):
            raise ValueError(f"Phase {phase['name']} does not target all five joints")
        for name, target in targets.items():
            if not limits[name]["lower"] <= float(target) <= limits[name]["upper"]:
                raise ValueError(f"Phase {phase['name']}: {name} target {target} is outside URDF limits")


def make_importable_urdf(source: Path) -> Path:
    """Replace package://pallet paths so Isaac can import an unbuilt source tree."""
    package_uri = PACKAGE_ROOT.as_uri().rstrip("/") + "/"
    text = source.read_text(encoding="utf-8").replace("package://pallet/", package_uri)
    temp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", prefix="pallet_gripperless_", delete=False, encoding="utf-8"
    )
    with temp:
        temp.write(text)
    return Path(temp.name)


def as_floats(values: Any) -> list[float]:
    return [float(value) for value in values]


def main() -> int:
    args = parse_args()
    urdf_path = args.urdf.resolve()
    config_path = args.config.resolve()
    config = load_json(config_path)
    limits = read_urdf_limits(urdf_path)
    validate_config(config, limits)

    if args.validate_only:
        print(
            json.dumps(
                {
                    "passed": True,
                    "mode": "validate_only",
                    "urdf": str(urdf_path),
                    "config": str(config_path),
                    "gripper_present": False,
                    "joints": limits,
                    "phases": [phase["name"] for phase in config["phases"]],
                },
                indent=2,
                ensure_ascii=False,
            )
        )
        return 0

    # Isaac modules are intentionally imported only after SimulationApp starts.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    patched_urdf: Path | None = None
    report: dict[str, Any] = {
        "test": "pallet_gripperless_physics",
        "urdf": str(urdf_path),
        "config": str(config_path),
        "expected_joints": list(EXPECTED_JOINTS),
        "gripper_present": False,
        "passed": False,
        "phases": [],
    }

    try:
        import numpy as np
        import omni.kit.commands
        import omni.usd
        from pxr import UsdPhysics

        from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction

        patched_urdf = make_importable_urdf(urdf_path)
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
        drive_paths: dict[str, str] = {}
        for prim in stage.Traverse():
            name = prim.GetName()
            if name not in EXPECTED_JOINTS or not prim.HasAPI(UsdPhysics.DriveAPI):
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                continue
            drive.GetTypeAttr().Set("force")
            drive.GetMaxForceAttr().Set(limits[name]["effort"])
            drive.GetStiffnessAttr().Set(float(config["joints"][name]["stiffness"]))
            drive.GetDampingAttr().Set(float(config["joints"][name]["damping"]))
            drive_paths[name] = str(prim.GetPath())
        if set(drive_paths) != set(EXPECTED_JOINTS):
            raise RuntimeError(f"Could not configure every joint drive; found {drive_paths}")

        robot = world.scene.add(SingleArticulation(prim_path=prim_path, name="pallet_gripperless"))
        world.reset()
        dof_names = tuple(robot.dof_names)
        if set(dof_names) != set(EXPECTED_JOINTS) or len(dof_names) != len(EXPECTED_JOINTS):
            raise RuntimeError(f"Imported articulation is not the expected five-axis model: {dof_names}")

        dof_index = {name: dof_names.index(name) for name in EXPECTED_JOINTS}
        efforts = np.array([limits[name]["effort"] for name in dof_names], dtype=float)
        kps = np.array([config["joints"][name]["stiffness"] for name in dof_names], dtype=float)
        kds = np.array([config["joints"][name]["damping"] for name in dof_names], dtype=float)
        controller = robot.get_articulation_controller()
        controller.set_gains(kps=kps, kds=kds, save_to_usd=False)

        zero = np.zeros(len(dof_names), dtype=float)
        robot.set_joint_positions(zero)
        robot.set_joint_velocities(zero)
        world.step(render=not args.headless)

        settle_window = max(1, math.ceil(float(config["settle_window_seconds"]) / physics_dt))
        position_tol = float(config["position_tolerance_rad"])
        velocity_tol = float(config["velocity_tolerance_rad_s"])
        effort_margin = float(config["effort_limit_margin"])

        for phase in config["phases"]:
            target = np.array([phase["target_rad"][name] for name in dof_names], dtype=float)
            sample_count = max(settle_window, math.ceil(float(phase["duration_seconds"]) / physics_dt))
            position_samples: list[Any] = []
            velocity_samples: list[Any] = []
            applied_samples: list[Any] = []
            measured_samples: list[Any] = []
            action = ArticulationAction(joint_positions=target)

            for _ in range(sample_count):
                controller.apply_action(action)
                world.step(render=not args.headless)
                position_samples.append(np.asarray(robot.get_joint_positions(), dtype=float))
                velocity_samples.append(np.asarray(robot.get_joint_velocities(), dtype=float))
                applied_samples.append(np.asarray(robot.get_applied_joint_efforts(), dtype=float))
                measured_samples.append(np.asarray(robot.get_measured_joint_efforts(), dtype=float))

            positions = np.stack(position_samples[-settle_window:])
            velocities = np.stack(velocity_samples[-settle_window:])
            applied = np.stack(applied_samples)
            measured = np.stack(measured_samples)
            final_error = np.abs(target - positions[-1])
            settle_max_velocity = np.max(np.abs(velocities), axis=0)
            max_applied = np.max(np.abs(applied), axis=0)
            max_measured = np.max(np.abs(measured), axis=0)
            applied_ratio = max_applied / efforts
            joint_metrics = {
                name: {
                    "target_rad": float(target[index]),
                    "final_position_rad": float(positions[-1, index]),
                    "final_error_rad": float(final_error[index]),
                    "settle_max_velocity_rad_s": float(settle_max_velocity[index]),
                    "rated_effort_nm": float(efforts[index]),
                    "max_applied_effort_nm": float(max_applied[index]),
                    "max_measured_effort_nm": float(max_measured[index]),
                    "max_applied_effort_ratio": float(applied_ratio[index]),
                }
                for name, index in dof_index.items()
            }
            phase_passed = bool(
                np.all(final_error <= position_tol)
                and np.all(settle_max_velocity <= velocity_tol)
                and np.all(applied_ratio <= effort_margin)
            )
            report["phases"].append(
                {"name": phase["name"], "passed": phase_passed, "joints": joint_metrics}
            )

        report.update(
            {
                "passed": all(phase["passed"] for phase in report["phases"]),
                "physics_dt": physics_dt,
                "drive_mode": "force",
                "drive_paths": drive_paths,
                "dof_order": list(dof_names),
                "limits_source": "URDF joint limit effort",
                "criteria": {
                    "position_tolerance_rad": position_tol,
                    "velocity_tolerance_rad_s": velocity_tol,
                    "effort_limit_margin": effort_margin,
                },
            }
        )
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2, ensure_ascii=False))
        if patched_urdf is not None:
            patched_urdf.unlink(missing_ok=True)
        simulation_app.close()

    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
