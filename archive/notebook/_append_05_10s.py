"""Append 05 10s + standard-gait criteria. Delete after use."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "05_inplace_trot.ipynb"

OLD = """**合否はこの表である。** 試行 1–4 は空中 \\(0.15\\,\\mathrm{s}\\) だけで足踏みと書いた。実測リフトは数 mm で、映像は直立静止に見える。旧数字は消さないが、合格には使わない。

\\(T=7.0\\,\\mathrm{s}\\)。\\(xy_0,z_0\\) は走行開始時。

| 量 | 条件 |
|---|---|
| \\(\\lvert\\mathrm{roll}\\rvert,\\lvert\\mathrm{pitch}\\rvert\\) | \\(<0.35\\,\\mathrm{rad}\\)、連続 \\(\\ge 5.0\\,\\mathrm{s}\\) |
| \\(\\lVert xy-xy_0\\rVert\\) | \\(<0.30\\,\\mathrm{m}\\) |
| ベース \\(z\\) | \\(>0.18\\,\\mathrm{m}\\) |
| `qpos` | 有限 |
| 各脚の遊脚リフト `swing_dz` | \\(\\ge 0.020\\,\\mathrm{m}\\)（指令遊脚中の足先 \\(z\\) − 離地 \\(z\\)） |
| 各脚の実測空中（`contact_on=0`） | \\(\\ge 0.40\\,\\mathrm{s}\\)（\\(T=7\\,\\mathrm{s}\\) の合計） |
| 対角ペア指令 | FL+RR のみ、FR+RL のみ、の両方が出現 |
| 結果 | GIF。指令 GRF（白）と実接触。足が床を離れるのが分かること。失敗 GIF は消さない |

duty を \\(0.96\\) まで上げて空中 \\(0.15\\,\\mathrm{s}\\) だけ稼ぐのは、新判定では不合格である。満たすまで 06 へ進まない。
"""

NEW = r"""**合否はこの表である。** 試行 1–8 の数字は消さない。5.0 s や duty \(0.96\) は合格に使わない。

\(T=12.0\,\mathrm{s}\)。\(xy_0,z_0\) は走行開始時。

| 量 | 条件 |
|---|---|
| \(\lvert\mathrm{roll}\rvert,\lvert\mathrm{pitch}\rvert\) | \(<0.35\,\mathrm{rad}\)、連続 \(\ge 10.0\,\mathrm{s}\) |
| \(\lVert xy-xy_0\rVert\) | \(<0.30\,\mathrm{m}\)（その場） |
| ベース \(z\) | \(>0.18\,\mathrm{m}\) |
| `qpos` | 有限 |
| 各脚の遊脚リフト `swing_dz` | \(\ge 0.020\,\mathrm{m}\) |
| 各脚の実測空中（`contact_on=0`） | \(\ge 1.00\,\mathrm{s}\)（\(T=12\,\mathrm{s}\) の合計） |
| 対角ペア指令 | FL+RR のみ、FR+RL のみ、の両方が出現 |
| ゲイト数字 | 上流から大きく外さない（下表）。外したら hold が長くても不合格 |
| 結果 | GIF。指令 GRF（白）と実接触。失敗 GIF は消さない |

上流の足踏み（PyMPC `gait_params['trot']`、LC `SwingTrajectoryPlanner`）:

| 量 | 参照 | この段の許容 |
|---|---|---|
| \(f\) | PyMPC \(1.35\,\mathrm{Hz}\) | \(\lvert f-1.35\rvert\le 0.15\,\mathrm{Hz}\) |
| duty | PyMPC \(0.74\) | \(\lvert d-0.74\rvert\le 0.06\) |
| \(h_{\mathrm{step}}\) | PyMPC \(0.2\times 0.28=0.056\,\mathrm{m}\)、LC \(0.08\,\mathrm{m}\) | \(0.045\le h_{\mathrm{step}}\le 0.090\,\mathrm{m}\) |

duty \(0.96\) や \(h_{\mathrm{step}}=2\,\mathrm{cm}\) は参照から外れ、不合格である。満たすまで 06 へ進まない。
"""


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("試行 9 — 10 秒判定" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    c0 = "".join(nb.cells[0].source)
    if OLD not in c0:
        raise SystemExit("05 success block not found")
    c0 = c0.replace(
        "[README §3.8](../docs/block-curriculum/00_README.md)。**連続 5.0 秒以上、全ステップ。** 一瞬の接地や「数歩」は不合格。",
        "[README §3.8](../docs/block-curriculum/00_README.md)。**連続 10.0 秒以上、全ステップ。** 一瞬の接地や「数歩」は不合格。",
        1,
    )
    c0 = c0.replace(OLD, NEW, 1)
    nb.cells[0].source = c0

    last = "".join(nb.cells[-1].source)
    nb.cells[-1].source = last.replace("## 結果と分析", "## 途中分析（試行 1–8）", 1)
    nb.cells[-1].source = (
        "".join(nb.cells[-1].source).rstrip()
        + "\n\n試行 8 は 5.0 s 判定では通った。次のセルで 10.0 s と上流ゲイト窓で測り直す。\n"
    )

    extra = [
        new_markdown_cell(
            r"""## 試行 9 — 10 秒判定で試行 8 を測る

試行 8 の wrench（duty \(0.75\)、\(h=5\,\mathrm{cm}\)、\(f=1.35\,\mathrm{Hz}\)）はゲイト窓には入る。\(T=12\,\mathrm{s}\) で hold が 10 s に届くかを見る。届かなければ不合格のまま残す。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 試行 8 と同じ wrench を T=12s で走らせ、背景の 10s 判定で落とす。

HOLD_S10 = 10.0  # 内容: 05 以降の連続時間 [s]
T_LONG = 12.0    # 内容: 10s 判定のあと余白
AIR_12 = 1.00    # 内容: T=12s での各脚空中下限 [s]


def gait_in_window(freq, duty, step_h):
    """内容: PyMPC/LC の足踏みから大きく外れていないか。"""
    return abs(freq - 1.35) <= 0.15 and abs(duty - 0.74) <= 0.06 and 0.045 <= step_h <= 0.090


r9 = rollout_wrench(duty=0.75, step_h=0.05, freq=1.35, T=T_LONG)
ok10 = r9["hold"] >= HOLD_S10 and walks(r9) and float(r9["meas_air"].min()) >= AIR_12
print("trial9  trial8-ctrl T=12  hold", r9["hold"], "walk", walks(r9), "gait_ok", gait_in_window(1.35, 0.75, 0.05))
print("  zmin", r9["zmin"], "rpy", r9["maxr"], "xy", r9["maxxy"])
print("  meas_air", r9["meas_air"].round(3), "swing_dz", r9["swing_dz"].round(4))
print("  PASS_10s", ok10)
assert gait_in_window(1.35, 0.75, 0.05)
assert r9["hold"] < HOLD_S10, "if this holds 10s, keep it as the new pass"
print("05 trial9 FAIL under 10s hold (expected). GIF 05d kept.")
'''
        ),
        new_markdown_cell(
            r"""## 試行 10 — 上流どおり \(1.35\,\mathrm{Hz}/0.74/5.6\,\mathrm{cm}\)

PyMPC の数字そのもの。LC の \(8\,\mathrm{cm}\) も隣で試す。ゲイトは合格窓の中心。hold が 10 s 未満なら、参照に寄せた結果として失敗を残す。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 上流 trot の中心値。5s 合格組より duty が低く、遊脚が長い。

r10a = rollout_wrench(duty=0.74, step_h=0.056, freq=1.35, T=T_LONG)
r10b = rollout_wrench(duty=0.74, step_h=0.08, freq=1.35, T=T_LONG)
print("trial10a pympc 1.35/0.74/0.056  hold", r10a["hold"], "walk", walks(r10a), "air", r10a["meas_air"].round(3), "dz", r10a["swing_dz"].round(3))
print("trial10b lc 1.35/0.74/0.08     hold", r10b["hold"], "walk", walks(r10b), "air", r10b["meas_air"].round(3), "dz", r10b["swing_dz"].round(3))
assert gait_in_window(1.35, 0.74, 0.056) and gait_in_window(1.35, 0.74, 0.08)
assert r10a["hold"] < HOLD_S10 and r10b["hold"] < HOLD_S10
print("05 trial10 FAIL: standard height/period, hold < 10s")
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析（10 秒判定）

- 試行 1–8: 残す。試行 8 は 5.0 s + リフト 2 cm では通った（GIF `05d`）
- 試行 9: 同じ制御を \(T=12\,\mathrm{s}\) で測ると hold は約 5.5 s で切れる。10 s 不合格
- 試行 10: PyMPC \(1.35/0.74/0.056\) と LC \(8\,\mathrm{cm}\) はゲイト窓の中心だが、2–3 s で倒れる。参照に寄せると今の wrench では持たない

05 は 10 s 判定では未成功のままである。06 へ進まない。失敗を消さない。

## 次の仮説

立脚 wrench のゲインか、遊脚 PD の引っ張りが 5 s のあとに姿勢を壊している。ホライズン NMPC はまだ足さない。同じ式のまま 10 s を探す。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
