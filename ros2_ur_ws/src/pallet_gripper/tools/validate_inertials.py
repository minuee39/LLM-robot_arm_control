#!/usr/bin/env python3
"""Validate pallet_gripper inertial frames against the archived Y-up export."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / "urdf/pallet_gripper.urdf"
SOURCE = PACKAGE / "source/urdf/어셈블리1.SLDASM.urdf"
LINK_NAMES = {
    "base_link": "base_link",
    "no.1": "J1",
    "no.2": "J2",
    "no.3": "J3",
    "no.4": "J4",
    "no.5": "J5",
    "gripper": "J6",
    "move_gripper": "J7",
    "gripper_r": "J8",
    "gripper_l": "J9",
}
Q = np.array(((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)))


def vector(text):
    return np.fromstring(text, sep=" ")


def tensor(element):
    a = element.attrib
    return np.array((
        (float(a["ixx"]), float(a["ixy"]), float(a["ixz"])),
        (float(a["ixy"]), float(a["iyy"]), float(a["iyz"])),
        (float(a["ixz"]), float(a["iyz"]), float(a["izz"])),
    ))


def rpy_matrix(text):
    roll, pitch, yaw = vector(text)
    cr, cp, cy = np.cos((roll, pitch, yaw))
    sr, sp, sy = np.sin((roll, pitch, yaw))
    return np.array((
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    ))


def mesh_bounds(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"invalid STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"invalid binary STL length: {path}")
    dtype = np.dtype([
        ("normal", "<f4", 3),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ])
    vertices = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].reshape(-1, 3)
    if not np.isfinite(vertices).all():
        raise ValueError(f"non-finite STL vertex: {path}")
    return vertices.min(axis=0), vertices.max(axis=0)


def mesh_vertices(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"invalid binary STL length: {path}")
    dtype = np.dtype([
        ("normal", "<f4", 3),
        ("vertices", "<f4", (3, 3)),
        ("attribute", "<u2"),
    ])
    return np.frombuffer(data, dtype=dtype, offset=84)["vertices"].reshape(-1, 3)


def main():
    robot = ET.parse(URDF).getroot()
    source = ET.parse(SOURCE).getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    source_links = {link.get("name"): link for link in source.findall("link")}

    for source_name, name in LINK_NAMES.items():
        link = links[name]
        original = source_links[source_name]
        inertial = link.find("inertial")
        original_inertial = original.find("inertial")
        mass = float(inertial.find("mass").get("value"))
        if mass <= 0 or not np.isfinite(mass):
            raise ValueError(f"{name}: invalid mass")
        if mass != float(original_inertial.find("mass").get("value")):
            raise ValueError(f"{name}: mass differs from archived export")

        com = vector(inertial.find("origin").get("xyz"))
        expected_com = Q @ vector(original_inertial.find("origin").get("xyz"))
        if not np.allclose(com, expected_com, atol=5e-12, rtol=0):
            raise ValueError(f"{name}: incorrect Y-up to Z-up COM transform")

        inertia = tensor(inertial.find("inertia"))
        expected_inertia = Q @ tensor(original_inertial.find("inertia")) @ Q.T
        if not np.allclose(inertia, expected_inertia, atol=3e-12, rtol=0):
            raise ValueError(f"{name}: incorrect Y-up to Z-up inertia transform")
        principal = np.linalg.eigvalsh(inertia)
        if principal[0] <= 0 or principal[2] > principal[0] + principal[1] + 1e-12:
            raise ValueError(f"{name}: physically invalid principal inertia moments")

        mesh_uri = link.find("visual/geometry/mesh").get("filename")
        mesh = PACKAGE / mesh_uri.removeprefix("package://pallet_gripper/")
        low, high = mesh_bounds(mesh)
        if np.any(com < low - 1e-9) or np.any(com > high + 1e-9):
            raise ValueError(f"{name}: COM is outside the mesh AABB")

    for joint in robot.findall("joint"):
        axis = joint.find("axis")
        if joint.get("type") == "fixed":
            if axis is not None or joint.find("limit") is not None:
                raise ValueError(f"{joint.get('name')}: fixed joint has axis/limit")
            continue
        values = vector(axis.get("xyz"))
        if not np.isclose(np.linalg.norm(values), 1.0, atol=1e-9):
            raise ValueError(f"{joint.get('name')}: non-unit joint axis")

    base_low, _ = mesh_bounds(PACKAGE / "meshes/base_link.STL")
    mount_z = float(robot.find("joint[@name='base_footprint_joint']/origin").get("xyz").split()[2])
    if not np.isclose(base_low[2] + mount_z, 0.0, atol=1e-9):
        raise ValueError("base mesh is not grounded at base_footprint z=0")

    # Recalculate the TCP from the maximum J7-z fingertip endpoint of each jaw.
    tip_z = []
    for name in ("J8", "J9"):
        joint = robot.find(f"joint[@name='{name}']")
        origin = joint.find("origin")
        rotation = rpy_matrix(origin.get("rpy"))
        translation = vector(origin.get("xyz"))
        mesh_uri = robot.find(f"link[@name='{name}']/visual/geometry/mesh").get("filename")
        vertices = mesh_vertices(PACKAGE / mesh_uri.removeprefix("package://pallet_gripper/"))
        in_j7 = vertices @ rotation.T + translation
        tip_z.append(float(in_j7[:, 2].max()))
    expected_tcp = np.array((0.0, 0.0, sum(tip_z) / 2.0))
    tcp_joint = robot.find("joint[@name='tcp_joint']")
    if tcp_joint is None or tcp_joint.get("type") != "fixed":
        raise ValueError("tcp_joint must be fixed")
    if tcp_joint.find("parent").get("link") != "J7":
        raise ValueError("tcp_joint parent must be J7")
    tcp = vector(tcp_joint.find("origin").get("xyz"))
    if not np.allclose(tcp, expected_tcp, atol=5e-10, rtol=0):
        raise ValueError(f"tcp {tcp} does not match fingertip midpoint {expected_tcp}")
    if robot.find("link[@name='tcp']") is None:
        raise ValueError("tcp link is missing")

    print("PASS: inertials, STL bounds, joint axes, base ground offset, and fingertip TCP")


if __name__ == "__main__":
    main()
