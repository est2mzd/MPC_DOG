"""Add 10s/10m criteria notes to 07-14. Do not delete old cells."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_markdown_cell

NOTE = r"""

**判定の更新（05 以降）:** 連続 hold は **10.0 s 以上**。歩く段はさらに水平 **10.0 m 以上**。ゲイトは PyMPC trot \(1.35\,\mathrm{Hz}/0.74\)、遊脚高さ \(5.6\)–\(8\,\mathrm{cm}\)（LC `swingHeight`）から大きく外さない。下の旧表（5.0 s、高い duty、数 mm リフト）は合格に使わない。旧セルと GIF は消さない。
"""

TAIL = r"""## 続き — 新判定では未成功

旧セルの hold 5 s と直立ループは、10 s / 10 m / 上流ゲイトの表では不合格である。05 が標準足踏みで 10 s 持つまで、この番号は進めない。失敗ログは残す。
"""

ROOT = Path(__file__).resolve().parent
NAMES = [
    "07_slow_vx.ipynb",
    "08_planner_signature.ipynb",
    "09_cmd_split.ipynb",
    "10_linear_kf.ipynb",
    "11_two_rate.ipynb",
    "12_lc_grf.ipynb",
    "13_wbc_torque.ipynb",
    "14_hybrid_joint.ipynb",
]


def main() -> None:
    for name in NAMES:
        path = ROOT / name
        nb = nbformat.read(path, as_version=4)
        if any("判定の更新（05 以降）" in "".join(c.source) for c in nb.cells):
            print("skip", name)
            continue
        src = "".join(nb.cells[0].source)
        key = "### 成功条件"
        i = src.find(key)
        if i < 0:
            raise SystemExit(f"no success header in {name}")
        # insert note after the first heading line
        nl = src.find("\n", i)
        src = src[: nl + 1] + NOTE + src[nl + 1 :]
        # bump 5.0 s hold mentions in the first cell to point at the update
        nb.cells[0].source = src
        if not any("続き — 新判定では未成功" in "".join(c.source) for c in nb.cells):
            nb.cells.append(new_markdown_cell(TAIL))
        nbformat.write(nb, path)
        print("patched", name, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
