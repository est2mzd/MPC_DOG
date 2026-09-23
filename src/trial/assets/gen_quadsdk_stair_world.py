"""Japanese residential stair world for Quad-SDK (physics XML + terrain PLY).

Course, walking +x. The body starts at x = 1, which is 2 m before
the first riser, and the command ends 2 m after the exit floor.
The mesh keeps 2 m of floor behind the spawn and 1 m past the stop.

  3 m flat, first riser at x = 3
  -> 10 risers up
  -> 2 m landing
  -> 9 treads down, then the exit floor at z = 0
  -> 2 m flat
  -> stop

Stair size is a common house stair, inside the residential limit
(建築基準法施行令第23条: riser <= 23 cm, tread >= 15 cm):

  riser  = 0.18 m   (蹴上)
  tread  = 0.24 m   (踏面)
  2 * riser + tread = 0.60 m

The planner map is the PLY, not the XML. Each walking surface is a
horizontal quad. The upper surface overlaps the lower one by 1 cm at
the nosing so the height map keeps the higher z on that cell.
"""
from __future__ import annotations

import struct
from pathlib import Path

RISER = 0.18
TREAD = 0.24
N_STEPS = 10
APPROACH = 3.0
LANDING = 2.0
EXIT = 2.0
MARGIN = 1.0
# Half-width 1.5 m matches the floor that walked straight.
# foothold_search_radius is 0.7 m, so 0.8 m puts the search disk off the map edge.
Y_HALF = 1.5
X_BACK = -1.0
NAME = "jp_stair_10"

X0 = APPROACH  # first riser, x = 3
X_TOP = X0 + N_STEPS * TREAD  # 5.4, start of the 2 m landing
X_LAND_END = X_TOP + LANDING  # 7.4, first down riser
X_EXIT0 = X_LAND_END + (N_STEPS - 1) * TREAD  # 9.56, z returns to 0
X_STOP = X_EXIT0 + EXIT  # 12.56
X_END = X_STOP + MARGIN  # 13.56

QSDK = (
    Path(__file__).resolve().parents[3]
    / "external"
    / "quad-sdk"
    / "quad_simulator"
    / "quad_sim_scripts"
)
WORLDS = QSDK / "worlds"
MESH = QSDK / "models" / NAME / "meshes"
MESH.mkdir(parents=True, exist_ok=True)


def box(name: str, x0: float, x1: float, top: float, rgba: str) -> str:
    half_x = (x1 - x0) / 2.0
    cx = (x0 + x1) / 2.0
    half_z = (top + 0.10) / 2.0
    cz = top - half_z
    return (
        f'    <geom name="{name}" type="box" '
        f'size="{half_x:.4f} {Y_HALF:.4f} {half_z:.4f}" '
        f'pos="{cx:.4f} 0 {cz:.4f}" rgba="{rgba}" '
        f'friction="1.5 0.1 0.05" solref="0.007 1" solimp="0.95 0.99 0.002"/>'
    )


def grid(name: str, x: float, top: float) -> str:
    return (
        f'    <geom name="{name}" type="box" size="0.015 {Y_HALF:.4f} 0.001" '
        f'pos="{x:.4f} 0 {top + 0.003:.4f}" rgba="0.25 0.25 0.25 1" '
        f'contype="0" conaffinity="0"/>'
    )


geoms: list[str] = []
geoms.append(box("approach", X_BACK, X0, 0.0, "0.75 0.82 0.75 1"))
for k in range(1, N_STEPS + 1):
    x0 = X0 + (k - 1) * TREAD
    rgba = "0.72 0.62 0.48 1" if k % 2 else "0.60 0.50 0.38 1"
    geoms.append(box(f"up_{k:02d}", x0, x0 + TREAD, k * RISER, rgba))
geoms.append(box("landing", X_TOP, X_LAND_END, N_STEPS * RISER, "0.86 0.84 0.70 1"))
for j in range(1, N_STEPS):
    x0 = X_LAND_END + (j - 1) * TREAD
    top = (N_STEPS - j) * RISER
    rgba = "0.62 0.58 0.70 1" if j % 2 else "0.50 0.46 0.60 1"
    geoms.append(box(f"down_{j:02d}", x0, x0 + TREAD, top, rgba))
geoms.append(box("exit_flat", X_EXIT0, X_END, 0.0, "0.75 0.82 0.75 1"))
# 1 m ticks on the 3 m approach, plus the stop line 3 m after the stairs.
for gx in (0.0, 1.0, 2.0, 3.0, X_STOP):
    geoms.append(grid(f"grid_{gx:.0f}", gx, 0.0 if gx <= X0 or gx >= X_EXIT0 else N_STEPS * RISER))

xacro = f"""<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{NAME}" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Japanese house stair.
       riser {RISER:.2f} m, tread {TREAD:.2f} m, 2R+T = {2*RISER+TREAD:.2f} m.
       first riser x={X0:.2f} (spawn x=0 is 3 m before it), landing x=[{X_TOP:.2f}, {X_LAND_END:.2f}]
       z={N_STEPS*RISER:.2f}, exit floor from x={X_EXIT0:.2f}, stop x={X_STOP:.2f}.
       Terrain MAP is models/{NAME}/meshes/{NAME}.ply. -->
  <xacro:arg name="meshdir" default=""/>
  <xacro:arg name="mjcf_path" default=""/>
  <compiler angle="radian" meshdir="$(arg meshdir)" texturedir="$(arg meshdir)" autolimits="true"/>
  <include file="$(arg mjcf_path)"/>
  <worldbody>
    <light directional="true" diffuse=".8 .8 .8" specular=".2 .2 .2" pos="0 0 8" dir="0 0 -1"/>
{chr(10).join(geoms)}
  </worldbody>
</mujoco>
"""
(WORLDS / f"{NAME}.xml.xacro").write_text(xacro)

# Horizontal quads. Upper surface overlaps the lower by 1 cm at each nosing.
surfaces: list[tuple[float, float, float]] = []
surfaces.append((X_BACK, X0 + 0.01, 0.0))
for k in range(1, N_STEPS + 1):
    x0 = X0 + (k - 1) * TREAD
    x1 = x0 + TREAD + (0.01 if k < N_STEPS else 0.0)
    surfaces.append((x0, x1, k * RISER))
surfaces.append((X_TOP, X_LAND_END + 0.01, N_STEPS * RISER))
for j in range(1, N_STEPS):
    x0 = X_LAND_END + (j - 1) * TREAD
    x1 = x0 + TREAD + 0.01
    surfaces.append((x0, x1, (N_STEPS - j) * RISER))
surfaces.append((X_EXIT0, X_END, 0.0))

RGBA = (202, 209, 238, 0)
pv: list[tuple[float, float, float]] = []
pf: list[tuple[int, int, int]] = []
for x0, x1, z in surfaces:
    b = len(pv)
    pv += [
        (x0, -Y_HALF, z),
        (x1, -Y_HALF, z),
        (x1, Y_HALF, z),
        (x0, Y_HALF, z),
    ]
    pf += [(b, b + 1, b + 2), (b, b + 2, b + 3)]

hdr = (
    "ply\r\n"
    "format binary_little_endian 1.0\r\n"
    f"comment {NAME} stair treads riser={RISER} tread={TREAD}\r\n"
    f"element vertex {len(pv)}\r\n"
    "property float x\r\nproperty float y\r\nproperty float z\r\n"
    f"element face {len(pf)}\r\n"
    "property uchar red\r\nproperty uchar green\r\nproperty uchar blue\r\n"
    "property uchar alpha\r\n"
    "property list uchar int vertex_indices\r\n"
    "end_header\r\n"
).encode("ascii")
body = bytearray()
for x, y, z in pv:
    body += struct.pack("<fff", x, y, z)
for a, b_, c in pf:
    body += struct.pack("<BBBBB", *RGBA, 3) + struct.pack("<iii", a, b_, c)
(MESH / f"{NAME}.ply").write_bytes(hdr + bytes(body))

print(f"world : {WORLDS / (NAME + '.xml.xacro')}")
print(f"mesh  : {MESH / (NAME + '.ply')}  ({len(pv)} verts, {len(pf)} tris)")
print(f"stop_x: {X_STOP:.2f}")
print(f"x_exit0: {X_EXIT0:.2f}  x_top: {X_TOP:.2f}  x_land_end: {X_LAND_END:.2f}  height: {N_STEPS * RISER:.2f}")
