"""Execute 05 trial 13 and embed GIF 05g. Reuses trial 12 helpers."""
from __future__ import annotations

import base64
import io
import os
import sys
import traceback
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import nbformat
from nbformat.v4 import new_output

os.environ.setdefault("MUJOCO_GL", "egl")
NB = Path(__file__).resolve().parent / "05_inplace_trot.ipynb"
GIF = NB.parent / "assets/05g_inplace_trot_20s_f122.gif"


class Image:
    def __init__(self, filename=None, **_k):
        self.filename = filename


def display(obj):
    print("[display]", getattr(obj, "filename", obj))


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    ns = {"__name__": "__main__", "Image": Image, "display": display}
    idxs = [1]
    for i, c in enumerate(nb.cells):
        src = "".join(c.source)
        if "trial12  trial11-ctrl" in src or "trial13  f1.22" in src:
            idxs.append(i)
    if len(idxs) != 3:
        raise SystemExit(f"expected setup+t12+t13, got {idxs}")
    os.chdir(NB.parent)
    for idx in idxs:
        src = "".join(nb.cells[idx].source)
        # 試行 12 の GIF は作り直さない。関数定義と数値だけ使う。
        if "trial12  trial11-ctrl" in src:
            src = src.split("st12 = {")[0]
        buf_out, buf_err = io.StringIO(), io.StringIO()
        print(f"=== exec cell {idx} ===", flush=True)
        try:
            with redirect_stdout(buf_out), redirect_stderr(buf_err):
                exec(compile(src, f"05c{idx}", "exec"), ns, ns)
            ok, err = True, ""
        except Exception:
            ok, err = False, traceback.format_exc()
            print(err, file=sys.stderr)
        text, errc = buf_out.getvalue(), buf_err.getvalue()
        print(text, end="")
        if "trial13  f1.22" not in "".join(nb.cells[idx].source):
            if not ok:
                raise SystemExit(f"cell {idx} failed")
            continue
        outputs = []
        if text:
            outputs.append(new_output("stream", name="stdout", text=text))
        if errc:
            outputs.append(new_output("stream", name="stderr", text=errc))
        if not ok:
            outputs.append(
                new_output(
                    "error",
                    ename="ExecutionError",
                    evalue=err.splitlines()[-1],
                    traceback=err.splitlines(),
                )
            )
        if GIF.is_file():
            outputs.append(
                new_output(
                    "display_data",
                    data={
                        "image/gif": base64.b64encode(GIF.read_bytes()).decode("ascii"),
                        "text/plain": ["<IPython.core.display.Image object>"],
                    },
                    metadata={},
                )
            )
        nb.cells[idx].outputs = outputs
        nb.cells[idx].execution_count = 13
        if not ok:
            nbformat.write(nb, NB)
            raise SystemExit(f"cell {idx} failed")
    nbformat.write(nb, NB)
    print("wrote", NB)


if __name__ == "__main__":
    main()
