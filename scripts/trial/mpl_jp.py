"""Shared matplotlib setup for the Step charts: use a Japanese-capable font
(Noto Serif CJK JP) so labels don't render as tofu. Import for side effects:

    import scripts.trial.mpl_jp   # or: exec(open('scripts/trial/mpl_jp.py').read())
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt  # noqa: F401  (re-exported for callers)

_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc",
]
for _p in _CANDIDATES:
    try:
        fm.fontManager.addfont(_p)
    except Exception:
        pass

for _name in ("Noto Sans CJK JP", "Noto Serif CJK JP"):
    if any(f.name == _name for f in fm.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _name
        break

matplotlib.rcParams["axes.unicode_minus"] = False
