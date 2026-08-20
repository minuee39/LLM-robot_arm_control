#!/usr/bin/env python3
"""Generate lightweight convex-decomposed collision OBJ files from binary STL meshes."""

from pathlib import Path
import argparse
import struct
import tempfile

import pybullet


def stl_to_obj(stl_path: Path, obj_path: Path) -> None:
    data = stl_path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(data) != expected_size:
        raise ValueError(f"Only binary STL is supported: {stl_path}")

    vertices: dict[tuple[float, float, float], int] = {}
    faces: list[tuple[int, int, int]] = []

    for triangle_index in range(triangle_count):
        offset = 84 + triangle_index * 50 + 12
        face = []
        for vertex_index in range(3):
            vertex = struct.unpack_from("<3f", data, offset + vertex_index * 12)
            if vertex not in vertices:
                vertices[vertex] = len(vertices) + 1
            face.append(vertices[vertex])
        if len(set(face)) == 3:
            faces.append(tuple(face))

    with obj_path.open("w", encoding="ascii") as obj_file:
        for vertex in vertices:
            obj_file.write(f"v {vertex[0]:.9g} {vertex[1]:.9g} {vertex[2]:.9g}\n")
        for face in faces:
            obj_file.write(f"f {face[0]} {face[1]} {face[2]}\n")


def generate_collision_mesh(stl_path: Path, output_path: Path) -> None:
    with tempfile.TemporaryDirectory(prefix="pallet_vhacd_") as temporary_directory:
        input_obj = Path(temporary_directory) / f"{stl_path.stem}.obj"
        log_path = Path(temporary_directory) / f"{stl_path.stem}_vhacd.log"
        stl_to_obj(stl_path, input_obj)
        pybullet.vhacd(
            str(input_obj),
            str(output_path),
            str(log_path),
            resolution=100000,
            maxNumVerticesPerCH=64,
            minVolumePerCH=0.0001,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_directory", type=Path)
    parser.add_argument("output_directory", type=Path)
    args = parser.parse_args()

    args.output_directory.mkdir(parents=True, exist_ok=True)

    for stl_path in sorted(args.mesh_directory.glob("*.STL")):
        output_path = args.output_directory / f"{stl_path.stem}_collision.obj"
        print(f"Generating {output_path.name} from {stl_path.name}", flush=True)
        generate_collision_mesh(stl_path, output_path)


if __name__ == "__main__":
    main()
