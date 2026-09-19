#!/usr/bin/env python3
"""Bake the SolidWorks Y-up frame into Pallet meshes and URDF coordinates."""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np
from lxml import etree
from scipy.spatial.transform import Rotation


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE.parent
URDF = PACKAGE / "urdf" / "pallet_gripper.urdf"
MESHES = PACKAGE / "meshes"
SOURCE_MESHES = WORKSPACE_SRC / "pallet_gripper_description" / "meshes"
ROTATION = Rotation.from_euler("x", np.pi / 2).as_matrix()
ROTATION_H = np.eye(4)
ROTATION_H[:3, :3] = ROTATION


def numbers(text: str) -> np.ndarray:
    return np.array([float(value) for value in text.split()], dtype=float)


def clean(value: float) -> float:
    return 0.0 if abs(value) < 5e-11 else float(value)


def vector_text(values: np.ndarray) -> str:
    return " ".join(f"{clean(value):.12g}" for value in values)


def origin_matrix(element: etree._Element) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, 3] = numbers(element.get("xyz", "0 0 0"))
    matrix[:3, :3] = Rotation.from_euler(
        "xyz", numbers(element.get("rpy", "0 0 0"))
    ).as_matrix()
    return matrix


def transform_origin(element: etree._Element) -> None:
    transformed = ROTATION_H @ origin_matrix(element) @ ROTATION_H.T
    element.set("xyz", vector_text(transformed[:3, 3]))
    element.set(
        "rpy",
        vector_text(Rotation.from_matrix(transformed[:3, :3]).as_euler("xyz")),
    )


def transform_inertia(inertia: etree._Element) -> None:
    tensor = np.array(
        [
            [float(inertia.get("ixx")), float(inertia.get("ixy")), float(inertia.get("ixz"))],
            [float(inertia.get("ixy")), float(inertia.get("iyy")), float(inertia.get("iyz"))],
            [float(inertia.get("ixz")), float(inertia.get("iyz")), float(inertia.get("izz"))],
        ]
    )
    tensor = ROTATION @ tensor @ ROTATION.T
    for key, row, column in (
        ("ixx", 0, 0), ("ixy", 0, 1), ("ixz", 0, 2),
        ("iyy", 1, 1), ("iyz", 1, 2), ("izz", 2, 2),
    ):
        inertia.set(key, f"{clean(tensor[row, column]):.12g}")


def transform_stl(source: Path, destination: Path) -> None:
    data = bytearray(source.read_bytes())
    if len(data) < 84:
        raise ValueError(f"Invalid binary STL: {source}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + triangle_count * 50:
        raise ValueError(f"Expected binary STL records in {source}")
    for triangle in range(triangle_count):
        offset = 84 + triangle * 50
        normal = np.array(struct.unpack_from("<3f", data, offset), dtype=float)
        vertices = np.array(struct.unpack_from("<9f", data, offset + 12), dtype=float).reshape(3, 3)
        struct.pack_into("<3f", data, offset, *(ROTATION @ normal))
        struct.pack_into("<9f", data, offset + 12, *((vertices @ ROTATION.T).reshape(-1)))
    destination.write_bytes(data)


def main() -> None:
    parser = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(str(URDF), parser)
    robot = tree.getroot()
    base_joint = robot.xpath("./joint[@name='base_footprint_joint']")[0]
    base_origin = base_joint.find("origin")
    expected = np.array([np.pi / 2, 0.0, 0.0])
    if not np.allclose(numbers(base_origin.get("rpy")), expected, atol=1e-8):
        raise RuntimeError("URDF is not in the expected pre-normalized Y-up state")

    for source_mesh in sorted(SOURCE_MESHES.glob("*.STL")):
        destination = MESHES / source_mesh.name
        if not destination.exists():
            raise FileNotFoundError(destination)
        transform_stl(source_mesh, destination)

    for link in robot.findall("link"):
        if link.get("name") == "base_footprint":
            continue
        for origin in link.xpath("./inertial/origin | ./visual/origin | ./collision/origin"):
            transform_origin(origin)
        inertia = link.find("inertial/inertia")
        if inertia is not None:
            transform_inertia(inertia)

    for joint in robot.findall("joint"):
        if joint.get("name") == "base_footprint_joint":
            base_origin.set("xyz", "0 0 0")
            base_origin.set("rpy", "0 0 0")
            continue
        origin = joint.find("origin")
        if origin is not None:
            transform_origin(origin)
        axis = joint.find("axis")
        if axis is not None:
            axis.set("xyz", vector_text(ROTATION @ numbers(axis.get("xyz"))))

    marker = etree.Comment(
        " Native ROS Z-up coordinates; converted from the SolidWorks Y-up export. "
    )
    marker.tail = "\n  "
    robot.insert(0, marker)
    previous = robot.getprevious()
    if previous is not None:
        previous.tail = "\n"
    tree.write(str(URDF), encoding="utf-8", xml_declaration=True, pretty_print=True)
    print(f"Normalized {URDF} and {len(list(MESHES.glob('*.STL')))} meshes to Z-up")


if __name__ == "__main__":
    main()
