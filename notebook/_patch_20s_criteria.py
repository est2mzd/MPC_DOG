"""Rewrite 10s hold criteria to 20s in notebooks. Do not delete old cells."""
from pathlib import Path

import nbformat

ROOT = Path(__file__).resolve().parent


def src(cell):
    return "".join(cell.source) if isinstance(cell.source, list) else cell.source


def set_src(cell, text):
    cell.source = text


def patch_05():
    path = ROOT / "05_inplace_trot.ipynb"
    nb = nbformat.read(path, as_version=4)
    c0 = src(nb.cells[0])
    c0 = c0.replace("**連続 10.0 秒以上、全ステップ。**", "**連続 20.0 秒以上、全ステップ。**")
    c0 = c0.replace("試行 1–8 の数字は消さない。5.0 s や duty", "試行 1–11 の数字は消さない。5.0 s・10.0 s や duty")
    c0 = c0.replace("\\(T=12.0\\,\\mathrm{s}\\)。", "\\(T=22.0\\,\\mathrm{s}\\)。")
    c0 = c0.replace("連続 \\(\\ge 10.0\\,\\mathrm{s}\\)", "連続 \\(\\ge 20.0\\,\\mathrm{s}\\)")
    c0 = c0.replace(
        "\\(\\ge 1.00\\,\\mathrm{s}\\)（\\(T=12\\,\\mathrm{s}\\) の合計）",
        "\\(\\ge 1.80\\,\\mathrm{s}\\)（\\(T=22\\,\\mathrm{s}\\) の合計）",
    )
    set_src(nb.cells[0], c0)
    nbformat.write(nb, path)
    print("05 first cell updated")


def patch_06():
    path = ROOT / "06_gait_modes.ipynb"
    nb = nbformat.read(path, as_version=4)
    c0 = src(nb.cells[0])
    c0 = c0.replace("連続 \\(\\ge 10.0\\,\\mathrm{s}\\)", "連続 \\(\\ge 20.0\\,\\mathrm{s}\\)")
    c0 = c0.replace("\\(T=12.0\\,\\mathrm{s}\\)", "\\(T=22.0\\,\\mathrm{s}\\)")
    c0 = c0.replace("10 s 静止。10 m は求めない", "20 s 静止。10 m は求めない")
    c0 = c0.replace("**10 s かつ水平 10 m**", "**20 s かつ水平 10 m**")
    c0 = c0.replace("空中 \\(\\ge 1.0\\,\\mathrm{s}\\)", "空中 \\(\\ge 1.80\\,\\mathrm{s}\\)")
    set_src(nb.cells[0], c0)
    nbformat.write(nb, path)
    print("06 first cell updated")


NOTE_OLD = "**判定の更新（05 以降）:** 連続 hold は **10.0 s 以上**。"
NOTE_NEW = "**判定の更新（05 以降）:** 連続 hold は **20.0 s 以上**。"
TAIL_OLD = "旧セルの hold 5 s と直立ループは、10 s / 10 m / 上流ゲイトの表では不合格である。05 が標準足踏みで 10 s 持つまで、この番号は進めない。失敗ログは残す。"
TAIL_NEW = "旧セルの hold 5 s と直立ループは、20 s / 10 m / 上流ゲイトの表では不合格である。05 が標準足踏みで 20 s 持つまで、この番号は進めない。失敗ログは残す。"


def patch_07_14():
    names = [
        "07_slow_vx.ipynb",
        "08_planner_signature.ipynb",
        "09_cmd_split.ipynb",
        "10_linear_kf.ipynb",
        "11_two_rate.ipynb",
        "12_lc_grf.ipynb",
        "13_wbc_torque.ipynb",
        "14_hybrid_joint.ipynb",
    ]
    for name in names:
        path = ROOT / name
        nb = nbformat.read(path, as_version=4)
        n = 0
        for cell in nb.cells:
            t = src(cell)
            if NOTE_OLD in t or TAIL_OLD in t:
                t = t.replace(NOTE_OLD, NOTE_NEW).replace(TAIL_OLD, TAIL_NEW)
                set_src(cell, t)
                n += 1
        nbformat.write(nb, path)
        print(name, "cells patched", n)


if __name__ == "__main__":
    patch_05()
    patch_06()
    patch_07_14()
