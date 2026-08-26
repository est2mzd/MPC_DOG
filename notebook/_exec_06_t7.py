"""Execute 06 trial 7 and embed GIF 06f."""
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
GIF = NB.parent / "assets" / os.environ.get("TRIAL_GIF", "06f_trot_vx015.gif")
MARKER = os.environ.get("TRIAL_MARKER", "trial7  05-11 + vx0.15")
EXECUTION_COUNT = int(os.environ.get("TRIAL_EXECUTION_COUNT", "7"))
EMBED_GIF = os.environ.get("TRIAL_EMBED_GIF", "1") == "1"


class Image:
    def __init__(self, filename=None, **_k):
        self.filename = filename


def display(obj):
    print("[display]", getattr(obj, "filename", obj))


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    ns = {"__name__": "__main__", "Image": Image, "display": display}
    idxs = [1]
    tnew = None
    for i, c in enumerate(nb.cells):
        if MARKER in "".join(c.source):
            tnew = i
    if tnew is None:
        raise SystemExit(f"{MARKER} not found")
    idxs.append(tnew)
    os.chdir(NB.parent)
    for idx in idxs:
        src = "".join(nb.cells[idx].source)
        buf_out, buf_err = io.StringIO(), io.StringIO()
        print(f"=== exec cell {idx} ===", flush=True)
        try:
            with redirect_stdout(buf_out), redirect_stderr(buf_err):
                exec(compile(src, f"06c{idx}", "exec"), ns, ns)
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
        if EMBED_GIF and GIF.is_file():
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
        nb.cells[idx].execution_count = EXECUTION_COUNT
        if not ok:
            nbformat.write(nb, NB)
            raise SystemExit(f"cell {idx} failed")
    nbformat.write(nb, NB)
    print("wrote", NB)


if __name__ == "__main__":
    main()
