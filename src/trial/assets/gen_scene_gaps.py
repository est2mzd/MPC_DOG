"""Generate src/trial/assets/scene_gaps.xml.

Walkway y in [-2.5, 2.5] (5 m wide). Along +x: 1.7 m solid strips whose top
surface is flush at z = 0, separated by 0.30 m full-width trenches ("holes")
every 2.0 m. Strip centers x = ..., -2, 0, 2, 4, ...; trench centers the odd
x = ..., -1, 1, 3, ....

The trench floor is a continuous slab at z = -DEPTH: the "holes" are
DEPTH-deep, 0.30 m-long, 5 m-wide ruts you can put a foot into, not a
bottomless void. DEPTH is the knob tuned for Step 03.
"""
from __future__ import annotations
import sys
from pathlib import Path

DEPTH = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10  # trench depth [m]

xs = [2 * k for k in range(-1, 16)]  # raised-strip centers, x = -2 .. 30
strips = []
for i, xc in enumerate(xs):
    name = "floor" if xc == 0 else f"gap_strip_{i}"
    # Raised strip: DEPTH tall, sitting on the base slab, top flush at z = 0.
    strips.append(
        f'        <geom name="{name}" type="box" size="0.85 2.5 {DEPTH/2:.4f}" '
        f'pos="{xc} 0 {-DEPTH/2:.4f}" material="groundplane" friction="1.0 0.005 0.0"/>'
    )
strip_xml = "\n".join(strips)
gap_centers = [2 * k + 1 for k in range(-1, 15)]

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
        <geom name="trench_floor" type="box" size="20 2.5 0.05" pos="14 0 {-DEPTH-0.05:.4f}"
              material="groundplane" friction="1.0 0.005 0.0"/>
        <!-- Walkway y in [-2.5, 2.5] (5 m). 1.7 m raised strips (top z = 0) + 0.30 m
             full-width holes every 2.0 m in x. Hole centers x = {gap_centers}.
             Hole depth {DEPTH} m. -->
{strip_xml}
    </worldbody>
</mujoco>
"""

out = Path(__file__).with_name("scene_gaps.xml")
out.write_text(xml)
print(f"wrote {out} with hole depth {DEPTH} m")
