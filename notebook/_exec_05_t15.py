"""Execute 05 trial 15 and embed GIF 05i."""
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
GIF = NB.parent / "assets/05i_inplace_trot_20s_kdatt.gif"
MARKER = "trial15  t13+kdatt6"


class Image:
    def __init__(self, filename=None, **_k):
        self.filename = filename


def display(obj):
    print("[display]", getattr(obj, "filename", obj))


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    ns = {"__name__": "__main__", "Image": Image, "display": display}

    def gait_in_window(freq, duty, step_h):
        return (
            abs(freq - 1.35) <= 0.15 + 1e-9
            and abs(duty - 0.74) <= 0.06 + 1e-9
            and 0.045 <= step_h <= 0.090
        )

    ns["gait_in_window"] = gait_in_window
    idxs = [1]
    tnew = None
    for i, c in enumerate(nb.cells):
        src = "".join(c.source)
        if "trial12  trial11-ctrl" in src:
            idxs.append(i)
        if MARKER in src:
            tnew = i
    if tnew is None:
        raise SystemExit("trial 15 not found")
    idxs.append(tnew)
    os.chdir(NB.parent)
    for idx in idxs:
        src = "".join(nb.cells[idx].source)
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
        if idx != tnew:
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
        nb.cells[idx].execution_count = 15
        if not ok:
            nbformat.write(nb, NB)
            raise SystemExit(f"cell {idx} failed")
    nbformat.write(nb, NB)
    print("wrote", NB)


if __name__ == "__main__":
    main()
