"""Append 06 10s/10m + standard-gait criteria. Delete after use."""
from pathlib import Path

import nbformat
from nbformat.v4 import new_code_cell, new_markdown_cell

NB = Path(__file__).resolve().parent / "06_gait_modes.ipynb"

OLD = """**合否はこの表である。** 試行 1–2 は空中 \\(0.15\\,\\mathrm{s}\\) と「短くてよい」で 5 モードを通した。リフトは数 mm。旧数字は消さないが、合格には使わない。

\\(T=7.0\\,\\mathrm{s}\\)。胴体は 05 と同じ。

| 量 | 条件 |
|---|---|
| \\(\\lvert\\mathrm{roll}\\rvert,\\lvert\\mathrm{pitch}\\rvert\\) | \\(<0.35\\,\\mathrm{rad}\\)、連続 \\(\\ge 5.0\\,\\mathrm{s}\\) |
| \\(\\lVert xy-xy_0\\rVert\\) | \\(<0.30\\,\\mathrm{m}\\) |
| ベース \\(z\\) | \\(>0.18\\,\\mathrm{m}\\) |
| `qpos` | 有限 |

モードごとの追加:

| モード | 追加（新判定） |
|---|---|
| `full_stance` | 各脚空中 \\(<0.05\\,\\mathrm{s}\\)。足が上がらないこと |
| `trot` / `crawl` | 各脚リフト \\(\\ge 0.020\\,\\mathrm{m}\\)、各脚空中 \\(\\ge 0.40\\,\\mathrm{s}\\)。hold \\(\\ge 5.0\\,\\mathrm{s}\\) |
| `pace` / `bound` | 同じ wrench・実遊脚で測る。hold とリフトを同時に 5 秒満たせなければ **不合格のまま残す**。duty を \\(0.99\\) にして直立にした記録は合格にしない |

指令 \\(c(t)\\) の 4 本線を残す。失敗 GIF は消さない。`trot` と `crawl` が実歩で 5 秒、`full_stance` が静止、`pace`/`bound` の実遊脚失敗が残ってから 07 へ進む。
"""

NEW = r"""**合否はこの表である。** 試行 1–4 の数字は消さない。5.0 s や duty \(0.96\)、低すぎる周波数は合格に使わない。

\(T=12.0\,\mathrm{s}\)（歩行は距離が出るまで延ばしてよい）。

| 量 | 条件 |
|---|---|
| \(\lvert\mathrm{roll}\rvert,\lvert\mathrm{pitch}\rvert\) | \(<0.35\,\mathrm{rad}\)、連続 \(\ge 10.0\,\mathrm{s}\) |
| ベース \(z\) | \(>0.18\,\mathrm{m}\) |
| `qpos` | 有限 |
| ゲイト数字 | 上流表から大きく外さない（下）。外したら不合格 |

| モード | 上流 \(f/d\) | 追加 |
|---|---|---|
| `full_stance` | PyMPC \(2/0.65\)（足は上げない） | 空中 \(<0.05\,\mathrm{s}/\)脚。10 s 静止。10 m は求めない |
| `trot` | \(1.35/0.74\)、\(h=5.6\)–\(8\,\mathrm{cm}\) | リフト \(\ge 2\,\mathrm{cm}\)、空中 \(\ge 1.0\,\mathrm{s}\)、**10 s かつ水平 10 m** |
| `crawl` | \(0.50/0.80\) | 同上（歩く） |
| `pace` | \(1.40/0.70\) | 同上（歩く） |
| `bound` | \(1.80/0.65\) | 同上（歩く） |

許容: \(\lvert f-f_{\mathrm{ref}}\rvert/f_{\mathrm{ref}}\le 0.15\)、\(\lvert d-d_{\mathrm{ref}}\rvert\le 0.06\)、\(0.045\le h_{\mathrm{step}}\le 0.090\,\mathrm{m}\)。

`crawl` の \(0.4\,\mathrm{Hz}/0.92\) は参照から外れる。duty \(0.99\) の直立も外れる。満たすまで 07 へ進まない。
"""


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    if any("試行 5 — 10 s / 10 m / 上流ゲイト" in "".join(c.source) for c in nb.cells):
        print("already patched")
        return
    c0 = "".join(nb.cells[0].source)
    if OLD not in c0:
        raise SystemExit("06 success block not found")
    c0 = c0.replace(OLD, NEW, 1)
    nb.cells[0].source = c0

    last = "".join(nb.cells[-1].source)
    nb.cells[-1].source = last.replace("## 結果と分析", "## 途中分析（試行 1–4）", 1)
    nb.cells[-1].source = (
        "".join(nb.cells[-1].source).rstrip()
        + "\n\n次のセルで 10 s・10 m・上流ゲイト窓で測り直す。\n"
    )

    extra = [
        new_markdown_cell(
            r"""## 試行 5 — 10 s / 10 m / 上流ゲイト

歩く 4 モードは PyMPC 表の \(f,d\) と \(h_{\mathrm{step}}=0.056\,\mathrm{m}\)。水平 10 m と hold 10 s を同時に見る。`full_stance` は 10 s 静止だけ。
"""
        ),
        new_code_cell(
            r'''# --- このセルの意図 ---
# 上流ゲイトで T=12s。歩くモードは距離も出す。今の wrench では両方満たさないことを残す。

HOLD_S10 = 10.0
DIST_M = 10.0
T_LONG = 12.0
H_REF = 0.056  # 内容: PyMPC 0.2 * hip_height [m]

GAITS_REF = {
    "full_stance": {"freq": 1.0, "duty": 1.00, "off": np.array([0.0, 0.0, 0.0, 0.0]), "walk": False},
    "trot": {"freq": 1.35, "duty": 0.74, "off": np.array([0.5, 1.0, 1.0, 0.5]), "walk": True},
    "crawl": {"freq": 0.50, "duty": 0.80, "off": np.array([0.0, 0.5, 0.75, 0.25]), "walk": True},
    "pace": {"freq": 1.40, "duty": 0.70, "off": np.array([0.5, 0.0, 0.5, 0.0]), "walk": True},
    "bound": {"freq": 1.80, "duty": 0.65, "off": np.array([0.5, 0.5, 0.0, 0.0]), "walk": True},
}

print("trial5  upstream gait window + 10s + 10m")
# 内容: 試行 4 の crawl 0.4/0.92 は窓の外
print("  old crawl 0.40/0.92 in window", abs(0.40 - 0.50) / 0.50 <= 0.15 and abs(0.92 - 0.80) <= 0.06)

ref_results = {}
for name, g in GAITS_REF.items():
    r = rollout_wrench(g["freq"], g["duty"], g["off"], H_REF, T=T_LONG)
    dist = float(r["maxxy"])  # 内容: 開始点からの最大水平距離 [m]
    ref_results[name] = r
    if g["walk"]:
        ok = r["hold"] >= HOLD_S10 and walks(r) and dist >= DIST_M
    else:
        ok = r["hold"] >= HOLD_S10 and float(r["meas_air"].max()) < AIR_STANCE
    print(
        name,
        "hold",
        r["hold"],
        "walk",
        walks(r),
        "dist",
        dist,
        "air",
        r["meas_air"].round(3),
        "dz",
        r["swing_dz"].round(3),
        "PASS",
        ok,
    )
    if g["walk"]:
        assert not ok, f"{name} unexpectedly passed the 10s/10m table"

print("06 trial5: walking modes FAIL 10s/10m with upstream numbers (expected)")
'''
        ),
        new_markdown_cell(
            r"""## 結果と分析（10 s / 10 m）

- 試行 1–4: 残す。5.0 s の trot/crawl 実歩は、10 s でも 10 m でもない
- 試行 4 の crawl \(0.4\,\mathrm{Hz}/0.92\) は PyMPC \(0.5/0.8\) から外れるので、それ自体が不合格
- 試行 5: 上流表の数字そのもので \(T=12\,\mathrm{s}\)。hold 10 s と 10 m は同時に満たさない

06 は新判定では未成功のままである。07 へ進まない。

## 次の仮説

10 m は着地点を離地 xy のままでは出ない。先送りはフェーズ 2 だが、まず 05 のその場 10 s を標準ゲイトで満たす。
"""
        ),
    ]
    nb.cells.extend(extra)
    nbformat.write(nb, NB)
    print("patched", NB, "cells", len(nb.cells))


if __name__ == "__main__":
    main()
