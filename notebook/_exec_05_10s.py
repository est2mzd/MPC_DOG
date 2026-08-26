"""Execute new 05 10s cells."""
from __future__ import annotations

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
CODE_IDX = [1, 12, 16, 21, 23]


class Image:
    def __init__(self, filename=None, **_k):
        self.filename = filename


def display(obj):
    print("[display]", getattr(obj, "filename", obj))


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    ns = {"__name__": "__main__", "Image": Image, "display": display}
    for idx in CODE_IDX:
        cell = nb.cells[idx]
        src = "".join(cell.source)
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
        outputs = []
        if text:
            outputs.append(new_output("stream", name="stdout", text=text))
        if errc:
            outputs.append(new_output("stream", name="stderr", text=errc))
        if not ok:
            outputs.append(
                new_output("error", ename="ExecutionError", evalue=err.splitlines()[-1], traceback=err.splitlines())
            )
        cell.outputs = outputs
        cell.execution_count = CODE_IDX.index(idx) + 1
        if not ok:
            nbformat.write(nb, NB)
            raise SystemExit(f"cell {idx} failed")
    nbformat.write(nb, NB)
    print("wrote", NB)


if __name__ == "__main__":
    main()
