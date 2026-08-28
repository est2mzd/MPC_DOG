"""Validate generated notebook syntax, nbformat, and pedagogical annotations."""

from __future__ import annotations

import io
import json
import tokenize
from pathlib import Path

import nbformat


HERE = Path(__file__).resolve().parent
FORMULA_TOKENS = ("=", "+", "-", "*", "/", "@", "np.", "solve", "clip", "sum(", "mean(", "return ")


def executable_line_numbers(source: str) -> set[int]:
    """Return code-bearing physical lines; blank/comments/multiline-string bodies are exempt."""
    ignored = {
        tokenize.ENCODING,
        tokenize.ENDMARKER,
        tokenize.INDENT,
        tokenize.DEDENT,
        tokenize.NEWLINE,
        tokenize.NL,
        tokenize.COMMENT,
    }
    return {
        token.start[0]
        for token in tokenize.generate_tokens(io.StringIO(source).readline)
        if token.type not in ignored
    }


def validate_source(source: str, label: str) -> int:
    """Require an immediately preceding 背景/目的 comment for every executable line."""
    if isinstance(source, list):
        source = "".join(source)
    compile(source, label, "exec")
    lines = source.splitlines()
    executable = executable_line_numbers(source)
    failures: list[str] = []
    for line_number in sorted(executable):
        code_line = lines[line_number - 1].strip()
        previous = lines[line_number - 2].strip() if line_number > 1 else ""
        if not previous.startswith("#") or "背景:" not in previous or "目的:" not in previous:
            failures.append(f"{label}:{line_number}: missing immediate 背景/目的 annotation")
        if any(token in code_line for token in FORMULA_TOKENS) and "数式:" not in previous:
            failures.append(f"{label}:{line_number}: formula/transform line missing 数式 annotation")
    if failures:
        raise AssertionError("\n".join(failures))
    return len(executable)


def main() -> None:
    notebooks = sorted(HERE.glob("[0-1][0-9]_*.ipynb"))
    if len(notebooks) != 16:
        raise AssertionError(f"expected 16 generated notebooks, found {len(notebooks)}")
    total = 0
    per_notebook: dict[str, int] = {}
    for path in notebooks:
        payload = json.loads(path.read_text(encoding="utf-8"))
        nbformat.validate(nbformat.from_dict(payload))
        covered = 0
        for cell_index, cell in enumerate(payload["cells"]):
            if cell["cell_type"] == "code":
                covered += validate_source(cell["source"], f"{path.name}:cell-{cell_index}")
        per_notebook[path.name] = covered
        total += covered
    print(json.dumps({"notebooks": per_notebook, "covered_executable_lines": total}, indent=2))
    print("Exemptions: blank/comment-only lines and text inside multiline string literals.")


if __name__ == "__main__":
    main()
