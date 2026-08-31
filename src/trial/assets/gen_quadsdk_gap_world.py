"""Generate a Quad-SDK gap world (physics XML + terrain-map PLY).

    python gen_quadsdk_gap_world.py <spacing_m> [depth_m] [tag]

Quad-SDK splits the terrain in two:
  - the MuJoCo world XML (`worlds/<name>.xml.xacro`) is what the robot
    physically walks on and can fall into;
  - the terrain MAP the planner sees is built by `mjcf_to_grid_map_converter`
    from `models/<name>/meshes/<name>.ply` (NOT the XML).

So a gap world needs BOTH:
  - box floor strips in the XML with `depth`-deep gaps every `spacing` m,
  - a PLY whose top surface has matching MISSING rectangles at the gaps, so
    the grid_map `traversability_hole_mask` filter flags them and
    `LocalFootstepPlanner::getNearestValidFoothold` steers footholds onto
    solid ground.

Walkway: y in [-2.5, 2.5] (5 m). Strips 0.30 m shorter than `spacing`
(the 0.30 m difference is the hole). Strip tops flush at z = 0. Robot
spawns near x = 0 on a strip and walks +x.

Writes into external/quad-sdk (new files, same pattern as flat_wide).
"""
from __future__ import annotations
import sys
from pathlib import Path

SPACING = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
DEPTH = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
TAG = sys.argv[3] if len(sys.argv) > 3 else (
    f"{SPACING:g}".replace(".", "p") + "m"
)
NAME = f"flat_gaps_{TAG}"

HOLE_LEN = 0.30
STRIP_LEN = SPACING - HOLE_LEN
HALF = STRIP_LEN / 2.0
Y_HALF = 2.5
X_MIN, X_MAX = -3.0, 25.0

QSDK = Path(__file__).resolve().parents[3] / "external" / "quad-sdk" / "quad_simulator" / "quad_sim_scripts"
worlds_dir = QSDK / "worlds"
mesh_dir = QSDK / "models" / NAME / "meshes"
mesh_dir.mkdir(parents=True, exist_ok=True)

# strip centres at k * SPACING; hole centres at k*SPACING + SPACING/2
k_lo = int(X_MIN // SPACING) - 1
k_hi = int(X_MAX // SPACING) + 2
strip_centres = [round(k * SPACING, 4) for k in range(k_lo, k_hi)]
hole_centres = [round(k * SPACING + SPACING / 2.0, 4) for k in range(k_lo, k_hi)]

# ---------------------------------------------------------------- world XML
geoms = [
    f'    <geom name="trench_floor" type="box" size="18 {Y_HALF} 0.05" '
    f'pos="11 0 {-DEPTH-0.05:.4f}" rgba="0.55 0.45 0.4 1"/>'
]
for i, xc in enumerate(strip_centres):
    nm = "floor" if abs(xc) < 1e-9 else f"strip_{i}"
    geoms.append(
        f'    <geom name="{nm}" type="box" size="{HALF:.4f} {Y_HALF} {DEPTH/2:.4f}" '
        f'pos="{xc} 0 {-DEPTH/2:.4f}" rgba="0.8 0.9 0.8 1"/>'
    )
# 5 m grid lines (visual only, no collision) like flat_wide
for gx in (0, 5, 10, 15, 20):
    geoms.append(
        f'    <geom name="grid_x{gx}" type="box" size="0.02 {Y_HALF} 0.001" '
        f'pos="{gx} 0 0.002" rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"/>'
    )
geom_xml = "\n".join(geoms)

xacro = f"""<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{NAME}" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Gap world: {STRIP_LEN:.2f} m box strips (top z=0) with {HOLE_LEN:.2f} m,
       {DEPTH:.2f} m-deep, full-width (5 m) trenches every {SPACING:.2f} m in x.
       Hole centres x = {hole_centres[:8]} ...
       Simple box primitives only (no detail mesh) to avoid the big_flat.xml
       instability. Terrain MAP comes from models/{NAME}/meshes/{NAME}.ply. -->
  <xacro:arg name="meshdir" default=""/>
  <xacro:arg name="mjcf_path" default=""/>
  <compiler angle="radian" meshdir="$(arg meshdir)" texturedir="$(arg meshdir)" autolimits="true"/>
  <include file="$(arg mjcf_path)"/>
  <worldbody>
    <light directional="true" diffuse=".8 .8 .8" specular=".2 .2 .2" pos="0 0 5" dir="0 0 -1"/>
{geom_xml}
  </worldbody>
</mujoco>
"""
(worlds_dir / f"{NAME}.xml.xacro").write_text(xacro)

# ---------------------------------------------------------------- terrain PLY
# ONE CONTINUOUS single-valued surface. Strips are flat at z = 0. Each hole
# x-band is a fine ZIGZAG of amplitude MAP_DIP (NOT a net dip): the local
# tilt/roughness of the zigzag makes the grid_map `slope`/`roughness`
# (small-radius) spike there, so `traversability` tanks and
# getNearestValidFoothold steers footholds off the hole.
#   Crucially the *mean* height and the *smoothed* normal over the hole band
#   stay ~0, so local_planner's `getTerrainHeight` (z_smooth -> ref body
#   height) and `getTerrainSlope` (smooth normals -> ref body pitch/roll) do
#   NOT get a fake step-down / fake pitch command over the hole -- that fake
#   pitch reference was what nose-dived the robot at the hole edge.
# Written in flat_wide.ply's exact binary format.
import struct

MAP_DIP = float(sys.argv[4]) if len(sys.argv) > 4 else 0.03  # zigzag amplitude [m]
ZIG_W = 0.037  # zigzag half-pitch [m] (a few grid cells; res is 0.05)
RGBA = (202, 209, 238, 0)

sx_min = strip_centres[0] - HALF
sx_max = strip_centres[-1] + HALF
pts: list[tuple[float, float]] = [(sx_min, 0.0)]
for xc in strip_centres:
    xa, xb = xc - HALF, xc + HALF
    pts.append((xa, 0.0))
    pts.append((xb, 0.0))
    if xb < sx_max - 1e-6:
        # zigzag across [xb, xb+HOLE_LEN], starting and ending at z=0
        x = xb + ZIG_W
        sign = -1.0
        while x < xb + HOLE_LEN - 1e-6:
            pts.append((x, sign * MAP_DIP))
            sign = -sign
            x += ZIG_W
        pts.append((xb + HOLE_LEN, 0.0))

surf = [pts[0]]
for p in pts[1:]:
    if abs(p[0] - surf[-1][0]) > 1e-6 or abs(p[1] - surf[-1][1]) > 1e-9:
        surf.append(p)

# Extrude across y in [-Y_HALF, Y_HALF]: each consecutive (x,z) pair -> a quad
# -> 2 triangles. Surface only (open, single-valued) -- simplest thing the
# grid_map ray-cast-down converter can rasterise.
pv, pf = [], []
for (x0, z0), (x1, z1) in zip(surf[:-1], surf[1:]):
    b = len(pv)
    pv += [(x0, -Y_HALF, z0), (x1, -Y_HALF, z1), (x1, Y_HALF, z1), (x0, Y_HALF, z0)]
    pf += [(b, b + 1, b + 2), (b, b + 2, b + 3)]

hdr = (
    "ply\r\n"
    "format binary_little_endian 1.0\r\n"
    f"comment {NAME} terrain: continuous surface, {MAP_DIP:g} m dips at hole bands\r\n"
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
(mesh_dir / f"{NAME}.ply").write_bytes(hdr + bytes(body))

print(f"world  : {worlds_dir / (NAME + '.xml.xacro')}")
print(f"mesh   : {mesh_dir / (NAME + '.ply')}  ({len(pv)} verts, {len(pf)} tris, binary)")
print(f"spacing {SPACING} m  depth {DEPTH} m  strip {STRIP_LEN:.2f} m  hole {HOLE_LEN} m")
