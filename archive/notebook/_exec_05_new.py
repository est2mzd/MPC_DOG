"""Execute new 05 cells, inject outputs. Do not import from notebooks."""
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
# setup + new trial code cells (skip old GIF trials 1-4)
CODE_IDX = [1, 12, 14, 16, 18]


class Image:
    def __init__(self, filename=None, **_k):
        self.filename = filename


def display(obj):
    print("[display]", getattr(obj, "filename", obj))


def stream_out(text: str, name: str = "stdout"):
    if not text:
        return None
    return new_output("stream", name=name, text=text)


def gif_out(path: Path):
    raw = path.read_bytes()
    return new_output(
        "display_data",
        data={
            "image/gif": base64.b64encode(raw).decode("ascii"),
            "text/plain": ["<IPython.core.display.Image object>"],
        },
        metadata={},
    )


def main() -> None:
    nb = nbformat.read(NB, as_version=4)
    ns = {
        "__name__": "__main__",
        "Image": Image,
        "display": display,
    }
    for idx in CODE_IDX:
        cell = nb.cells[idx]
        src = "".join(cell.source)
        buf_out, buf_err = io.StringIO(), io.StringIO()
        print(f"=== exec cell {idx} ===", flush=True)
        try:
            with redirect_stdout(buf_out), redirect_stderr(buf_err):
                exec(compile(src, f"05_cell_{idx}", "exec"), ns, ns)
            ok = True
            err_text = ""
        except Exception:
            ok = False
            err_text = traceback.format_exc()
            print(err_text, file=sys.stderr)
        out_text = buf_out.getvalue()
        err_cap = buf_err.getvalue()
        print(out_text, end="")
        if err_cap:
            print(err_cap, file=sys.stderr)
        outputs = []
        so = stream_out(out_text, "stdout")
        if so is not None:
            outputs.append(so)
        se = stream_out(err_cap, "stderr")
        if se is not None:
            outputs.append(se)
        if not ok:
            outputs.append(
                new_output(
                    "error",
                    ename="ExecutionError",
                    evalue=err_text.splitlines()[-1] if err_text else "",
                    traceback=err_text.splitlines(),
                )
            )
        # embed GIFs mentioned in this cell
        for name in (
            "assets/05c_equalshare_real_swing_flip.gif",
            "assets/05d_inplace_trot_lift.gif",
        ):
            if name in src:
                p = NB.parent / name
                if p.is_file():
                    outputs.append(gif_out(p))
        cell.outputs = outputs
        cell.execution_count = CODE_IDX.index(idx) + 1
        if not ok:
            nbformat.write(nb, NB)
            raise SystemExit(f"cell {idx} failed")
    nbformat.write(nb, NB)
    print("wrote", NB)


if __name__ == "__main__":
    main()
