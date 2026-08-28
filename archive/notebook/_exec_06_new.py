"""Execute new 06 cells and inject outputs."""
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

NB = Path(__file__).resolve().parent / "06_gait_modes.ipynb"
CODE_IDX = [1, 10, 12, 14]
GIFS = (
    "assets/06c_crawl_lift.gif",
    "assets/06d_pace_real_swing_flip.gif",
)


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
                exec(compile(src, f"06_cell_{idx}", "exec"), ns, ns)
            ok, err_text = True, ""
        except Exception:
            ok, err_text = False, traceback.format_exc()
            print(err_text, file=sys.stderr)
        out_text, err_cap = buf_out.getvalue(), buf_err.getvalue()
        print(out_text, end="")
        outputs = []
        if out_text:
            outputs.append(new_output("stream", name="stdout", text=out_text))
        if err_cap:
            outputs.append(new_output("stream", name="stderr", text=err_cap))
        if not ok:
            outputs.append(
                new_output(
                    "error",
                    ename="ExecutionError",
                    evalue=err_text.splitlines()[-1] if err_text else "",
                    traceback=err_text.splitlines(),
                )
            )
        for name in GIFS:
            if name in src:
                p = NB.parent / name
                if p.is_file():
                    outputs.append(
                        new_output(
                            "display_data",
                            data={
                                "image/gif": base64.b64encode(p.read_bytes()).decode("ascii"),
                                "text/plain": ["<IPython.core.display.Image object>"],
                            },
                            metadata={},
                        )
                    )
        png = NB.parent / "assets/06_gait_contact_walk.png"
        if "06_gait_contact_walk.png" in src and png.is_file():
            outputs.append(
                new_output(
                    "display_data",
                    data={
                        "image/png": base64.b64encode(png.read_bytes()).decode("ascii"),
                        "text/plain": ["<IPython.core.display.Image object>"],
                    },
                    metadata={},
                )
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
