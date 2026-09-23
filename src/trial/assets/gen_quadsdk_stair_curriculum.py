"""Generate an up-and-down Quad-SDK stair curriculum world and terrain PLY."""
from __future__ import annotations

import argparse
import struct
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", required=True)
    parser.add_argument("--riser", type=float, required=True)
    parser.add_argument("--tread", type=float, required=True)
    parser.add_argument("--steps", type=int, required=True)
    parser.add_argument("--approach", type=float, default=3.0)
    parser.add_argument("--top-run", type=float, default=1.0)
    parser.add_argument("--exit-run", type=float, default=3.0)
    parser.add_argument("--y-half", type=float, default=1.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.riser <= 0 or args.tread <= 0 or args.steps < 1:
        raise SystemExit("riser and tread must be positive, and steps must be >= 1")

    repo = Path(__file__).resolve().parents[3]
    qsdk = (
        repo
        / "external"
        / "quad-sdk"
        / "quad_simulator"
        / "quad_sim_scripts"
    )
    worlds = qsdk / "worlds"
    mesh_dir = qsdk / "models" / args.name / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    x_back = -1.0
    x0 = args.approach
    x_top_start = x0 + (args.steps - 1) * args.tread
    x_landing_end = x_top_start + args.top_run
    x_exit0 = x_landing_end + (args.steps - 1) * args.tread
    x_end = x_exit0 + args.exit_run
    top = args.steps * args.riser

    def box(name: str, start: float, end: float, height: float, rgba: str) -> str:
        half_x = (end - start) / 2.0
        center_x = (start + end) / 2.0
        half_z = (height + 0.10) / 2.0
        center_z = height - half_z
        return (
            f'    <geom name="{name}" type="box" '
            f'size="{half_x:.4f} {args.y_half:.4f} {half_z:.4f}" '
            f'pos="{center_x:.4f} 0 {center_z:.4f}" rgba="{rgba}" '
            'friction="1.5 0.1 0.05" solref="0.007 1" '
            'solimp="0.95 0.99 0.002"/>'
        )

    geoms = [box("approach", x_back, x0, 0.0, "0.75 0.82 0.75 1")]
    surfaces: list[tuple[float, float, float]] = [(x_back, x0 + 0.01, 0.0)]
    for step in range(1, args.steps):
        start = x0 + (step - 1) * args.tread
        end = start + args.tread
        height = step * args.riser
        color = "0.72 0.62 0.48 1" if step % 2 else "0.60 0.50 0.38 1"
        geoms.append(box(f"up_{step:02d}", start, end, height, color))
        surfaces.append((start, end + 0.01, height))
    geoms.append(
        box("top_run", x_top_start, x_landing_end, top, "0.86 0.84 0.70 1")
    )
    surfaces.append((x_top_start, x_landing_end + 0.01, top))
    for step in range(1, args.steps):
        start = x_landing_end + (step - 1) * args.tread
        end = start + args.tread
        height = (args.steps - step) * args.riser
        color = "0.62 0.58 0.70 1" if step % 2 else "0.50 0.46 0.60 1"
        geoms.append(box(f"down_{step:02d}", start, end, height, color))
        surfaces.append((start, end + 0.01, height))
    geoms.append(box("exit_flat", x_exit0, x_end, 0.0, "0.75 0.82 0.75 1"))
    surfaces.append((x_exit0, x_end, 0.0))

    xacro = f"""<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{args.name}" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Up-and-down stair curriculum: {args.steps} step(s), riser {args.riser:.2f} m,
       tread {args.tread:.2f} m, first riser x={x0:.2f}, top z={top:.2f}
       Terrain MAP is models/{args.name}/meshes/{args.name}.ply -->
  <xacro:arg name="meshdir" default=""/>
  <xacro:arg name="mjcf_path" default=""/>
  <compiler angle="radian" meshdir="$(arg meshdir)" texturedir="$(arg meshdir)" autolimits="true"/>
  <include file="$(arg mjcf_path)"/>
  <worldbody>
    <light directional="true" diffuse=".8 .8 .8" specular=".2 .2 .2" pos="0 0 5" dir="0 0 -1"/>
{chr(10).join(geoms)}
  </worldbody>
</mujoco>
"""
    world_path = worlds / f"{args.name}.xml.xacro"
    world_path.write_text(xacro)

    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    for start, end, height in surfaces:
        base = len(vertices)
        vertices += [
            (start, -args.y_half, height),
            (end, -args.y_half, height),
            (end, args.y_half, height),
            (start, args.y_half, height),
        ]
        faces += [(base, base + 1, base + 2), (base, base + 2, base + 3)]

    header = (
        "ply\r\n"
        "format binary_little_endian 1.0\r\n"
        f"comment {args.name} up-and-down curriculum\r\n"
        f"element vertex {len(vertices)}\r\n"
        "property float x\r\nproperty float y\r\nproperty float z\r\n"
        f"element face {len(faces)}\r\n"
        "property uchar red\r\nproperty uchar green\r\nproperty uchar blue\r\n"
        "property uchar alpha\r\n"
        "property list uchar int vertex_indices\r\n"
        "end_header\r\n"
    ).encode("ascii")
    body = bytearray()
    for x, y, z in vertices:
        body += struct.pack("<fff", x, y, z)
    for a, b, c in faces:
        body += struct.pack("<BBBBB", 202, 209, 238, 0, 3)
        body += struct.pack("<iii", a, b, c)
    mesh_path = mesh_dir / f"{args.name}.ply"
    mesh_path.write_bytes(header + bytes(body))

    print(f"world={world_path}")
    print(f"mesh={mesh_path}")
    print(
        f"first_riser_x={x0:.2f} top_start_x={x_top_start:.2f} "
        f"down_start_x={x_landing_end:.2f} exit_x={x_exit0:.2f} top_z={top:.2f}"
    )


if __name__ == "__main__":
    main()
