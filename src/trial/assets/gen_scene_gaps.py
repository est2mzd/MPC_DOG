"""Generate a gap-walkway MuJoCo scene for Step 03 / 04.

    python gen_scene_gaps.py <depth_m> [spacing_m] [out_name]

Walkway y in [-2.5, 2.5] (5 m wide). Along +x: solid raised strips whose top
surface is flush at z = 0, separated by 0.30 m full-width trenches ("holes")
spaced `spacing_m` apart in x. Strip length = spacing_m - 0.30 m.

The trench floor is a continuous slab at z = -depth: the "holes" are
depth-deep, 0.30 m-long, 5 m-wide ruts you can put a foot into, not a
bottomless void. `depth` is the knob tuned per step.

  Step 03: spacing 2.0 m -> scene_gaps.xml
  Step 04: spacing 1.5 m -> scene_gaps_1p5.xml
"""
from __future__ import annotations
import sys
from pathlib import Path

DEPTH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05      # trench depth [m]
SPACING = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0     # hole centre spacing [m]
OUT_NAME = sys.argv[3] if len(sys.argv) > 3 else "scene_gaps.xml"

HOLE_LEN = 0.30
STRIP_LEN = SPACING - HOLE_LEN
HALF = STRIP_LEN / 2.0

# Strip centres at x = k * SPACING. Hole centres at x = k * SPACING + SPACING/2.
# Start one strip before the origin so the robot spawns on solid ground.
strip_centres = [round(k * SPACING, 4) for k in range(-1, int(30 / SPACING) + 2)]
strips = []
for i, xc in enumerate(strip_centres):
    name = "floor" if abs(xc) < 1e-9 else f"gap_strip_{i}"
    strips.append(
        f'        <geom name="{name}" type="box" size="{HALF:.4f} 2.5 {DEPTH/2:.4f}" '
        f'pos="{xc} 0 {-DEPTH/2:.4f}" material="groundplane" friction="1.0 0.005 0.0"/>'
    )
strip_xml = "\n".join(strips)
hole_centres = [round(k * SPACING + SPACING / 2.0, 4) for k in range(-1, len(strip_centres))]

xml = f"""<mujoco model="scene">
    <statistic center="0 0 0.1" extent="0.8"/>
    <visual>
        <headlight diffuse="0.4 0.4 0.4" ambient="0.25 0.25 0.25" specular="0.25 0.25 0.25"/>
        <rgba haze="0.99 0.99 0.99 1"/>
        <global azimuth="-130" elevation="-20"/>
    </visual>
    <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.99 0.99 0.99" rgb2="0.99 0.99 0.99" width="512" height="3072"/>
        <texture type="2d" name="groundplane" builtin="checker" mark="edge"
                 rgb1="0.93 0.93 0.93" rgb2="0.82 0.86 0.93" markrgb="0 0 0" width="250" height="250"/>
        <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="4 4" reflectance="0.05"/>
    </asset>
    <worldbody>
        <light pos="0 0 5.0" dir="0 0 -1" directional="true" castshadow="true"/>
        <!-- Continuous trench floor at z = -{DEPTH} (the "hole" bottom). -->
        <geom name="trench_floor" type="box" size="24 2.5 0.05" pos="16 0 {-DEPTH-0.05:.4f}"
              material="groundplane" friction="1.0 0.005 0.0"/>
        <!-- Walkway y in [-2.5, 2.5] (5 m). {STRIP_LEN:.2f} m raised strips (top z = 0) +
             {HOLE_LEN:.2f} m full-width holes every {SPACING:.2f} m in x.
             Hole centres x = {hole_centres[:8]}...  Hole depth {DEPTH} m. -->
{strip_xml}
    </worldbody>
</mujoco>
"""

out = Path(__file__).with_name(OUT_NAME)
out.write_text(xml)
print(f"wrote {out}  |  depth {DEPTH} m  spacing {SPACING} m  strip {STRIP_LEN:.2f} m")
