"""Generate a Quad-SDK *single wide trench* world (physics XML + terrain PLY).

    python gen_quadsdk_wide_trench_world.py [width_m] [x0_m] [depth_m] [tag] [mesh_margin_m]

背景:
  Phase 2A(無効足場を NMPC へ渡さない安全停止)を検証するための地形。
  ユーザー指定シナリオ = 「進行方向に幅 10 m の穴を用意し、穴の手前で
  3 秒止まれたら OK」。穴が 10 m もあれば `foothold_search_radius`(0.7 m)
  では絶対に向こう岸へスナップできず、前方 touchdown はすべて
  `NO_TRAVERSABLE_CANDIDATE` になる。Phase 2A が効いていれば
  `computeLocalPlan()` が `false` を返し、local plan が古くなって
  robot_driver が起立姿勢へ PD ホールドする(= 穴の手前で停止)。

目的:
  `flat_trench_<tag>.xml.xacro`(物理ワールド)と
  `models/flat_trench_<tag>/meshes/flat_trench_<tag>.ply`(地形マップ用メッシュ)
  を external/quad-sdk 側へ書き出す。step03/04 の gen_quadsdk_gap_world.py と
  同じ二枚構成・同じ PLY バイナリ形式。既存ファイルは上書きしない
  (tag が違えば別名になる)。

地形:
  - 通路 y ∈ [-2.5, 2.5](5 m 幅)。step03/04 と同じ。
  - 助走: x ∈ [X_MIN, x0] は上面 z=0 の solid。
  - 穴:   x ∈ [x0, x0 + width]、深さ depth、全幅。ここにメッシュ面を置かない
          → 生 z = NaN → traversability_hole_mask 発火 → traversability = NaN。
  - 着地: x ∈ [x0 + width, X_MAX] も solid(ロボットは届かない想定)。
  - mesh_margin: メッシュの solid 端を物理縁より mesh_margin だけ手前で切る
          (穴側へ広げる)。既定 0.05 m(ユーザー確定値)。スナップ先が
          1 m 落下の物理縁から mesh_margin だけ内側に載る。
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

WIDTH = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0   # trench length in x [m]
X0 = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0       # trench near edge x [m]
DEPTH = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0    # trench depth [m]
TAG = sys.argv[4] if len(sys.argv) > 4 else f"{WIDTH:g}m".replace(".", "p")
MESH_MARGIN = float(sys.argv[5]) if len(sys.argv) > 5 else 0.05  # per side [m]
# Optional: trim the APPROACH (near) side of the terrain mesh back by this much
# instead of MESH_MARGIN. Used by the Phase 4 IK-reach demo: pull the last
# valid map cells so far back that the forward foothold has to snap to the far
# strip, > 0.4 m past the leg's reach -> IK_UNREACHABLE. Physical ground is
# unchanged (the robot still stands there); only the map keep-out widens.
APPROACH_MARGIN = float(sys.argv[6]) if len(sys.argv) > 6 else MESH_MARGIN

NAME = f"flat_trench_{TAG}"
Y_HALF = 2.5
X_MIN, X_MAX = -3.0, X0 + WIDTH + 6.0
X_GAP_A, X_GAP_B = X0, X0 + WIDTH

QSDK = (
    Path(__file__).resolve().parents[3]
    / "external"
    / "quad-sdk"
    / "quad_simulator"
    / "quad_sim_scripts"
)
worlds_dir = QSDK / "worlds"
mesh_dir = QSDK / "models" / NAME / "meshes"
mesh_dir.mkdir(parents=True, exist_ok=True)

# ---------------------------------------------------------------- world XML
approach_half = (X_GAP_A - X_MIN) / 2.0
approach_ctr = (X_MIN + X_GAP_A) / 2.0
landing_half = (X_MAX - X_GAP_B) / 2.0
landing_ctr = (X_GAP_B + X_MAX) / 2.0

geoms = [
    f'    <geom name="trench_floor" type="box" size="{(X_MAX - X_MIN) / 2 + 2:.4f} '
    f'{Y_HALF} 0.05" pos="{(X_MIN + X_MAX) / 2:.4f} 0 {-DEPTH - 0.05:.4f}" '
    f'rgba="0.55 0.45 0.4 1"/>',
    f'    <geom name="approach" type="box" size="{approach_half:.4f} {Y_HALF} '
    f'{DEPTH / 2:.4f}" pos="{approach_ctr:.4f} 0 {-DEPTH / 2:.4f}" '
    f'rgba="0.8 0.9 0.8 1"/>',
    f'    <geom name="landing" type="box" size="{landing_half:.4f} {Y_HALF} '
    f'{DEPTH / 2:.4f}" pos="{landing_ctr:.4f} 0 {-DEPTH / 2:.4f}" '
    f'rgba="0.8 0.9 0.8 1"/>',
]
for gx in range(0, int(X_MAX) + 1, 5):
    geoms.append(
        f'    <geom name="grid_x{gx}" type="box" size="0.02 {Y_HALF} 0.001" '
        f'pos="{gx} 0 0.002" rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"/>'
    )
geom_xml = "\n".join(geoms)

xacro = f"""<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{NAME}" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Single wide trench: solid top z=0 for x in [{X_MIN:.1f}, {X_GAP_A:.2f}] and
       [{X_GAP_B:.2f}, {X_MAX:.1f}]; a {WIDTH:.2f} m long, {DEPTH:.2f} m deep,
       full-width ({2 * Y_HALF:.0f} m) trench in between. Robot spawns near x = 0
       and walks +x; the trench is far too wide to cross, so this world tests
       whether Phase 2A halts the robot at the near lip.
       Terrain MAP comes from models/{NAME}/meshes/{NAME}.ply. -->
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
# Two disjoint flat quads at z = 0 (approach + landing). Nothing over the trench
# x-band -> the ray-cast converter leaves z = NaN there -> traversability = NaN.
# Same binary format as flat_wide.ply / gen_quadsdk_gap_world.py.
RGBA = (202, 209, 238, 0)
quads_x = [
    (X_MIN, X_GAP_A - APPROACH_MARGIN),
    (X_GAP_B + MESH_MARGIN, X_MAX),
]
pv, pf = [], []
for xa, xb in quads_x:
    b = len(pv)
    pv += [
        (xa, -Y_HALF, 0.0),
        (xb, -Y_HALF, 0.0),
        (xb, Y_HALF, 0.0),
        (xa, Y_HALF, 0.0),
    ]
    pf += [(b, b + 1, b + 2), (b, b + 2, b + 3)]

hdr = (
    "ply\r\n"
    "format binary_little_endian 1.0\r\n"
    f"comment {NAME} terrain: approach + landing quads (z=0), real {WIDTH:g} m gap\r\n"
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

print(f"world : {worlds_dir / (NAME + '.xml.xacro')}")
print(f"mesh  : {mesh_dir / (NAME + '.ply')}  ({len(pv)} verts, {len(pf)} tris)")
print(
    f"trench x in [{X_GAP_A:.2f}, {X_GAP_B:.2f}]  width {WIDTH} m  depth {DEPTH} m  "
    f"mesh_margin {MESH_MARGIN} m/side  approach_margin {APPROACH_MARGIN} m  "
    f"map keep-out x in [{X_GAP_A - APPROACH_MARGIN:.2f}, {X_GAP_B + MESH_MARGIN:.2f}]"
)
