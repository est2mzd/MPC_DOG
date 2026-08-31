"""Generate a Quad-SDK *repeated narrow gap* world (physics XML + terrain PLY).

    python gen_quadsdk_repeated_gap_world.py [strip_m] [gap_m] [n] [x0_m] [depth_m] [tag] [mesh_margin_m]

背景:
  Step 05:進行方向に「平地 strip_m → 穴 gap_m」を N 回くり返す地形で、
  Terrain Map / Foot Placement / NMPC が連続した狭い支持面をどこまで扱えるか、
  扱えないとき Phase 2A/3 で安全に止まれるかを見る。既定は strip=gap=0.15 m。
  ユーザー方針:N=2 で破綻したら strip/gap サイズを緩めて同種シナリオを反復し、
  「いろいろなパターンの成功/安全停止/失敗」の分布を表にする。

地形(gen_quadsdk_wide_trench_world.py と同じ二枚構成・同じ PLY バイナリ形式):
  - 通路 y ∈ [-2.5, 2.5](5 m 幅)。
  - 助走: x ∈ [X_MIN, x0] 上面 z=0 solid。
  - くり返し区間: pitch P = strip + gap。i=0..N-1 について
      支持 [x0 + P*i,        x0 + P*i + strip]
      穴   [x0 + P*i + strip, x0 + P*i + P    ](深さ depth、面を置かない)
  - 着地: x ∈ [x0 + P*N, X_MAX] 上面 z=0 solid。
  - mesh_margin: メッシュの solid 端を穴側へこのぶん引っ込める(片側)。既定 0.05。
    → プランナが見る立入禁止帯 = gap + 2*mesh_margin、実際に落ちる穴 = gap。
  - 助走/着地の物理床は連続 box、くり返し区間は strip ごとに box。穴底 box は
    z = -depth - 0.05 に 1 枚(落下時に落ち切らないように)。
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

STRIP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
GAP = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
N = int(sys.argv[3]) if len(sys.argv) > 3 else 2
X0 = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
DEPTH = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
TAG = sys.argv[6] if len(sys.argv) > 6 else (
    f"s{STRIP*100:g}g{GAP*100:g}n{N}".replace(".", "p")
)
MESH_MARGIN = float(sys.argv[7]) if len(sys.argv) > 7 else 0.05

NAME = f"flat_repgap_{TAG}"
Y_HALF = 2.5
PITCH = STRIP + GAP
TEST_LEN = PITCH * N
X_MIN = -3.0
X_END = X0 + TEST_LEN          # first x of the landing plane
X_MAX = X_END + 6.0

QSDK = (
    Path(__file__).resolve().parents[3]
    / "external" / "quad-sdk" / "quad_simulator" / "quad_sim_scripts"
)
worlds_dir = QSDK / "worlds"
mesh_dir = QSDK / "models" / NAME / "meshes"
mesh_dir.mkdir(parents=True, exist_ok=True)

# solid top-surface x-spans: approach, each inter-gap strip, landing
solid_spans = [(X_MIN, X0)]
for i in range(N):
    a = X0 + PITCH * i
    solid_spans.append((a, a + STRIP))
solid_spans.append((X_END, X_MAX))

# ---------------------------------------------------------------- world XML
def box(name, xa, xb, ztop, zbot):
    cx = (xa + xb) / 2.0
    hx = (xb - xa) / 2.0
    cz = (ztop + zbot) / 2.0
    hz = (ztop - zbot) / 2.0
    return (f'    <geom name="{name}" type="box" size="{hx:.4f} {Y_HALF} {hz:.4f}" '
            f'pos="{cx:.4f} 0 {cz:.4f}" rgba="0.8 0.9 0.8 1"/>')

geoms = [
    f'    <geom name="trench_floor" type="box" size="{(X_MAX - X_MIN) / 2 + 2:.4f} '
    f'{Y_HALF} 0.05" pos="{(X_MIN + X_MAX) / 2:.4f} 0 {-DEPTH - 0.05:.4f}" '
    f'rgba="0.55 0.45 0.4 1"/>',
]
for k, (xa, xb) in enumerate(solid_spans):
    nm = "approach" if k == 0 else ("landing" if k == len(solid_spans) - 1
                                    else f"strip_{k - 1}")
    geoms.append(box(nm, xa, xb, 0.0, -DEPTH))
for gx in range(0, int(X_MAX) + 1, 1):
    geoms.append(
        f'    <geom name="grid_x{gx}" type="box" size="0.005 {Y_HALF} 0.001" '
        f'pos="{gx} 0 0.002" rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"/>'
    )
geom_xml = "\n".join(geoms)

xacro = f"""<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{NAME}" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Repeated gap world: {STRIP:.2f} m strip / {GAP:.2f} m gap, x{N},
       depth {DEPTH:.2f} m, full-width ({2 * Y_HALF:.0f} m). Test region
       x in [{X0:.2f}, {X_END:.2f}] (len {TEST_LEN:.2f} m). Robot spawns near
       x = 0 on the approach plane and walks +x.
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
# One flat z=0 quad per solid span, trimmed by MESH_MARGIN on each gap-facing
# edge (approach: right edge only; landing: left edge only; strips: both).
RGBA = (202, 209, 238, 0)
pv, pf = [], []
for k, (xa, xb) in enumerate(solid_spans):
    la = xa + (MESH_MARGIN if k != 0 else 0.0)
    lb = xb - (MESH_MARGIN if k != len(solid_spans) - 1 else 0.0)
    if lb <= la:  # strip fully eaten by the margins -> nothing to place
        continue
    b = len(pv)
    pv += [(la, -Y_HALF, 0.0), (lb, -Y_HALF, 0.0),
           (lb, Y_HALF, 0.0), (la, Y_HALF, 0.0)]
    pf += [(b, b + 1, b + 2), (b, b + 2, b + 3)]

hdr = (
    "ply\r\n"
    "format binary_little_endian 1.0\r\n"
    f"comment {NAME}: {STRIP:g}/{GAP:g} strip/gap x{N}, real gaps in the mesh\r\n"
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
print(f"strip {STRIP} gap {GAP} N {N} pitch {PITCH:g}  test x in "
      f"[{X0:.2f}, {X_END:.2f}] (len {TEST_LEN:.2f})  mesh_margin {MESH_MARGIN}")
if len(pv) < 4 * (N + 1):
    print("WARNING: some strips vanished from the mesh "
          "(strip - 2*mesh_margin <= 0). The map will show a wider void.")
