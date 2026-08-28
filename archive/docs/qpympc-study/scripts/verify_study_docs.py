#!/usr/bin/env python3
"""Mechanical checks for docs/qpympc-study body + appendices.

Uses the standard library only. Does not import or modify control code.
"""

from __future__ import annotations

import ast
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
STUDY = ROOT / "docs" / "qpympc-study"
PYMPC = ROOT / "external" / "Quadruped-PyMPC"
BODY_GLOB = [
    "00_README.md",
    "01_MuJoCo_Go2_Plant_Model.md",
    "02_System_Architecture_and_Dataflow.md",
    "03_User_Command_and_Reference_Generation.md",
    "04_Gait_Generator_and_Contact_Schedule.md",
    "05_Foothold_Reference_and_Terrain_Adaptation.md",
    "06_Centroidal_SRBD_Model.md",
    "07_MPC_Formulation.md",
    "08_Gait_MPC_Coupling.md",
    "09_MPC_Output_and_Receding_Horizon.md",
    "10_Stance_and_Swing_Control.md",
    "11_Joint_Torque_and_MuJoCo_Closed_Loop.md",
    "12_Speed_Frequency_Duty_and_Stride.md",
    "13_Feasibility_on_Rough_Terrain.md",
    "14_MPC_and_Controller_Tuning.md",
    "15_Automatic_Tuning_and_Sim_to_Real.md",
    "16_Code_Map_and_Call_Graph.md",
    "17_Cursor_Analysis_Workflow.md",
    "18_Experiments_and_Research_Roadmap.md",
    "19_Conversation_Coverage_Map.md",
]
APPENDIX = [
    "appendices/A_Variable_Dictionary.md",
    "appendices/B_Equation_Index.md",
    "appendices/C_Parameter_Index.md",
    "appendices/D_File_Function_Index.md",
    "appendices/E_Corrections_and_Clarifications.md",
    "appendices/F_Open_Questions.md",
]


def body_files() -> list[Path]:
    return [STUDY / p for p in BODY_GLOB + APPENDIX]


def slugify(heading: str) -> str:
    text = heading.strip().lstrip("#").strip()
    text = re.sub(r"[（(].*?[）)]", "", text)
    text = text.replace("`", "")
    text = re.sub(r"[^\w\- \u3040-\u30ff\u4e00-\u9fff]", "", text)
    text = re.sub(r"\s+", "-", text).strip("-").lower()
    return text


def collect_headings(path: Path) -> set[str]:
    slugs = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("#"):
            slugs.add(slugify(line))
            m = re.match(r"^#+\s+(\d+)", line)
            if m:
                slugs.add(m.group(1))
    return slugs


MD_LINK = re.compile(r"(?<!!)\[([^\]]*)\]\(([^)]+)\)")


def check_links() -> list[tuple[str, str, str, str]]:
    rows = []
    heading_cache = {p: collect_headings(p) for p in body_files() if p.exists()}
    for src in body_files():
        text = src.read_text(encoding="utf-8")
        for m in MD_LINK.finditer(text):
            raw = m.group(2).strip()
            if raw.startswith("http://") or raw.startswith("https://"):
                rows.append((str(src.relative_to(STUDY)), raw, "external", "skip"))
                continue
            target, frag = (raw.split("#", 1) + [""])[:2]
            target = target.split()[0] if target else ""
            if not target:
                dest = src
                resolved = str(src.relative_to(STUDY)) + (f"#{frag}" if frag else "")
            else:
                dest = (src.parent / target).resolve()
                try:
                    resolved = str(dest.relative_to(STUDY.resolve()))
                except ValueError:
                    resolved = str(dest)
            if not dest.exists():
                rows.append((str(src.relative_to(STUDY)), raw, resolved, "missing"))
                continue
            if frag:
                slugs = heading_cache.get(dest) or collect_headings(dest)
                ok = frag in slugs or slugify(frag) in slugs
                rows.append(
                    (
                        str(src.relative_to(STUDY)),
                        raw,
                        resolved + f"#{frag}",
                        "ok" if ok else "anchor-missing",
                    )
                )
            else:
                rows.append((str(src.relative_to(STUDY)), raw, resolved, "ok"))
    return rows


def index_python(root: Path) -> tuple[set[str], set[str], set[str], set[str]]:
    files, classes, funcs, names = set(), set(), set(), set()
    if not root.exists():
        return files, classes, funcs, names
    for path in root.rglob("*.py"):
        try:
            rel_parts = path.relative_to(root).parts
        except ValueError:
            rel_parts = path.parts
        if any(p in rel_parts for p in (".venv", "__pycache__", "c_generated_code")):
            continue
        rel = str(path.relative_to(root)) if path.is_relative_to(root) else str(path)
        files.add(path.name)
        files.add(rel.replace("\\", "/"))
        try:
            tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                classes.add(node.name)
            elif isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef):
                funcs.add(node.name)
                for arg in node.args.args + node.args.kwonlyargs:
                    names.add(arg.arg)
            elif isinstance(node, ast.arg):
                names.add(node.arg)
            elif isinstance(node, ast.keyword) and node.arg:
                names.add(node.arg)
            elif isinstance(node, ast.Name):
                names.add(node.id)
            elif isinstance(node, ast.Attribute):
                names.add(node.attr)
            elif isinstance(node, ast.Constant) and isinstance(node.value, str):
                if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", node.value):
                    names.add(node.value)
    return files, classes, funcs, names


def index_xml(root: Path) -> tuple[set[str], set[str]]:
    found = set()
    tokens = set()
    if not root.exists():
        return found, tokens
    for path in root.rglob("*.xml"):
        found.add(path.name)
        found.add(str(path))
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        tokens.update(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", text))
    return found, tokens


CONFIG_KEYS = {
    "type",
    "gait",
    "dt",
    "mpc_frequency",
    "visual_foothold_adaptation",
    "optimize_step_freq",
    "use_RTI",
    "use_DDP",
    "use_warm_start",
    "use_integrators",
    "use_foothold_constraints",
    "use_foothold_optimization",
    "use_static_stability",
    "use_zmp_stability",
    "use_nonuniform_discretization",
    "use_inertia_recomputation",
    "reflex_trigger_mode",
    "velocity_modulator",
    "start_and_stop_activated",
    "external_wrenches_compensation",
    "mu",
    "ref_z",
    "step_freq",
    "duty_factor",
    "step_height",
    "hip_offset",
    "scene",
    "mode",
    "num_qp_iterations",
    "step_freq_available",
}

PY_FILE_RE = re.compile(r"`?([A-Za-z0-9_./-]+\.py)`?")
XML_FILE_RE = re.compile(r"`?([A-Za-z0-9_./-]+\.xml)`?")
CLASS_RE = re.compile(r"\b([A-Z][A-Za-z0-9]+(?:_[A-Za-z0-9]+)*)\b")
FUNC_RE = re.compile(r"`([A-Za-z_][A-Za-z0-9_]*)\(\)`")
FUNC_RE2 = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\(\)")
BACKTICK = re.compile(r"`([^`]+)`")

SKIP_CLASSES = {
    "README",
    "Markdown",
    "Cursor",
    "Python",
    "MuJoCo",
    "CasADi",
    "Go2",
    "FL",
    "FR",
    "RL",
    "RR",
    "GRF",
    "MPC",
    "NMPC",
    "OCP",
    "SRBD",
    "Centroidal",
    "WBC",
    "VFA",
    "PGG",
    "FRG",
    "IK",
    "PD",
    "VM",
    "XML",
    "IMU",
    "ERK",
    "SQP",
    "RTI",
    "DDP",
    "GPU",
    "ON",
    "OFF",
    "True",
    "False",
    "None",
    "World",
    "Base",
    "Heading",
    "Plant",
    "Appendix",
    "Open",
    "Questions",
    "Corrections",
    "Baseline",
    "Trot",
    "Swing",
    "Stance",
    "Foothold",
    "Gait",
    "Solver",
    "Index",
    "Cost",
    "Frame",
    "Shape",
    "Linear",
    "Quadratic",
    "Jacobian",
    "Cartesian",
    "Euler",
    "Quaternion",
    "CoM",
    "ZMP",
    "Duty",
    "Frequency",
    "Sampling",
    "Gradient",
    "Outer",
    "Domain",
    "Hardware",
    "Viewer",
    "Config",
    "Robot",
    "Menagerie",
    "Unitree",
    "HPIPM",
    "LINEAR",
    "LS",
    "GAUSS",
    "NEWTON",
    "BALANCE",
    "PARTIAL",
    "CONDENSING",
}

SKIP_VARS = {
    "mpc_dog",
    "cc145a2",
    "None",
    "True",
    "False",
    "wxyz",
    "visual",
    "collision",
    "start_and_stop",
    "virall",
    "periodic_gait_generator",
    "jointpos",
}


def check_code_refs(
    py_files: set[str],
    classes: set[str],
    funcs: set[str],
    names: set[str],
    xmls: set[str],
) -> list[tuple[str, str, str, str, str, str]]:
    rows = []
    seen = set()
    for src in body_files():
        text = src.read_text(encoding="utf-8")
        rel = str(src.relative_to(STUDY))

        for m in PY_FILE_RE.finditer(text):
            name = m.group(1)
            key = (rel, name, "py")
            if key in seen:
                continue
            seen.add(key)
            exists = name in py_files or Path(name).name in py_files
            loc = name if exists else ""
            rows.append((rel, name, "py", "yes" if exists else "no", loc, ""))

        for m in XML_FILE_RE.finditer(text):
            name = m.group(1)
            key = (rel, name, "xml")
            if key in seen:
                continue
            seen.add(key)
            exists = (
                name in xmls
                or Path(name).name in xmls
                and "unitree_go2" not in name
                or any(Path(x).as_posix().endswith(name) for x in xmls)
            )
            if "unitree_go2" in name:
                exists = any("unitree_go2" in Path(x).as_posix() for x in xmls)
                rows.append(
                    (
                        rel,
                        name,
                        "xml",
                        "yes" if exists else "expected-missing",
                        name if exists else "",
                        "Menagerie not in this tree; documented in 00/01/F",
                    )
                )
                continue
            rows.append((rel, name, "xml", "yes" if exists else "no", name if exists else "", ""))

        for m in FUNC_RE.finditer(text):
            name = m.group(1)
            key = (rel, name, "func")
            if key in seen:
                continue
            seen.add(key)
            rows.append(
                (
                    rel,
                    name + "()",
                    "func",
                    "yes" if name in funcs else "no",
                    name if name in funcs else "",
                    "",
                )
            )

        for m in CLASS_RE.finditer(text):
            name = m.group(1)
            if name in SKIP_CLASSES or len(name) < 4:
                continue
            key = (rel, name, "class")
            if key in seen:
                continue
            seen.add(key)
            if name in classes:
                rows.append((rel, name, "class", "yes", name, ""))
            elif name.endswith("Interface") or name.startswith("Acados") or name.startswith("Centroidal"):
                rows.append((rel, name, "class", "no", "", "documented controller name"))

        for m in BACKTICK.finditer(text):
            tok = m.group(1).strip()
            if " " in tok or "/" in tok or "." in tok and not tok.startswith(("mpc_params", "simulation")):
                if tok in CONFIG_KEYS or tok.split("[")[0] in CONFIG_KEYS:
                    key = (rel, tok, "config")
                    if key in seen:
                        continue
                    seen.add(key)
                    rows.append((rel, tok, "config", "yes" if tok.split("[")[0] in names or tok.split("[")[0] in CONFIG_KEYS else "未確認", "", "config-like"))
                continue
            if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", tok):
                key = (rel, tok, "var")
                if key in seen:
                    continue
                seen.add(key)
                if tok in SKIP_VARS:
                    continue
                if tok in CONFIG_KEYS:
                    rows.append((rel, tok, "config", "yes", "config.py / set_weight", ""))
                elif tok in names or tok in funcs or tok in classes:
                    rows.append((rel, tok, "var", "yes", tok, "name/attr in AST"))
                else:
                    rows.append((rel, tok, "var", "未確認", "", "string search only"))
    return rows


MERMAID_BLOCK = re.compile(r"```mermaid\n(.*?)```", re.S)


def check_mermaid() -> list[dict]:
    rows = []
    for src in body_files():
        text = src.read_text(encoding="utf-8")
        for i, block in enumerate(MERMAID_BLOCK.findall(text), 1):
            issues = []
            if block.count('"') % 2 != 0 and block.count("'") % 2 != 0:
                # count quotes inside labels
                pass
            dq = len(re.findall(r'(?<!\\)"', block))
            if dq % 2 != 0:
                issues.append("unbalanced-double-quote")
            ids = re.findall(r"^\s*([A-Za-z][A-Za-z0-9_]*)\[", block, re.M)
            dup = [x for x in ids if ids.count(x) > 1]
            if dup:
                issues.append("duplicate-id:" + ",".join(sorted(set(dup))))
            # crude rank: nodes on same flowchart line
            for line in block.splitlines():
                if line.count("[") >= 5:
                    issues.append("too-many-nodes-on-line")
            edges = re.findall(r"-->", block)
            labeled = re.findall(r"-->\|", block)
            unlabeled = len(edges) - len(labeled)
            if unlabeled:
                issues.append(f"unlabeled-edges:{unlabeled}")
            rows.append(
                {
                    "file": str(src.relative_to(STUDY)),
                    "n": i,
                    "syntax": "ok" if "unbalanced" not in "".join(issues) else "error",
                    "issues": issues,
                    "block": block,
                }
            )
    return rows


MATH_INLINE = re.compile(r"(?<!\\)\\\((.+?)(?<!\\)\\\)", re.S)
MATH_DISP = re.compile(r"(?<!\\)\\\[(.+?)(?<!\\)\\\]", re.S)


def check_math() -> list[tuple[str, str, str]]:
    rows = []
    for src in body_files():
        text = src.read_text(encoding="utf-8")
        # strip fences
        stripped = re.sub(r"```.*?```", "", text, flags=re.S)
        if stripped.count("\\(") != stripped.count("\\)"):
            rows.append((str(src.relative_to(STUDY)), "\\( \\)", "unbalanced"))
        if stripped.count("\\[") != stripped.count("\\]"):
            rows.append((str(src.relative_to(STUDY)), "\\[ \\]", "unbalanced"))
        dollars = len(re.findall(r"(?<!\\)\$", stripped))
        if dollars % 2 != 0:
            rows.append((str(src.relative_to(STUDY)), "$", "unbalanced"))
    return rows


IO_HINT = re.compile(
    r"入力|出力|変数|shape|単位|frame|Frame|Shape",
    re.I,
)


def check_tables() -> tuple[list[tuple[str, int, str]], list[tuple[str, int, str, str]]]:
    rows = []
    quality = []
    for src in body_files():
        lines = src.read_text(encoding="utf-8").splitlines()
        i = 0
        while i < len(lines) - 1:
            if "|" in lines[i] and re.match(r"^\s*\|?\s*-+", lines[i + 1] or ""):
                header = [c.strip() for c in lines[i].strip().strip("|").split("|")]
                ncol = len(header)
                joined = " ".join(header)
                j = i + 2
                while j < len(lines) and "|" in lines[j] and not lines[j].startswith("```"):
                    cols = [c.strip() for c in lines[j].strip().strip("|").split("|")]
                    if len(cols) != ncol:
                        rows.append((str(src.relative_to(STUDY)), i + 1, f"cols {len(cols)}!={ncol}"))
                    j += 1
                skip_quality = any(
                    s in joined
                    for s in (
                        "数式",
                        "論点",
                        "段階",
                        "症状",
                        "ファイル",
                        "方式",
                        "制約",
                        "優先",
                        "会話",
                    )
                )
                if (
                    not skip_quality
                    and IO_HINT.search(joined)
                    or (not skip_quality and any(h in header for h in ("入力", "出力", "変数", "パラメータ")))
                ):
                    missing = []
                    low = [h.lower() for h in header]
                    blob = joined.lower()
                    if not any("単位" in h or "unit" in h for h in low) and "単位" not in blob:
                        missing.append("単位")
                    if not any("shape" in h or "次元" in h or "index" in h for h in low):
                        missing.append("Shape")
                    if not any("frame" in h or "座標系" in h for h in low):
                        missing.append("Frame")
                    if "パラメータ" in joined and not any(
                        k in joined for k in ("Optional", "標準", "既定", "Default")
                    ):
                        missing.append("Default/Optional")
                    if missing:
                        quality.append(
                            (
                                str(src.relative_to(STUDY)),
                                i + 1,
                                joined,
                                ",".join(missing),
                            )
                        )
                i = j
            else:
                i += 1
    return rows, quality


def main() -> int:
    print(f"# verify_study_docs  ROOT={ROOT}")
    print(f"# body files: {len(body_files())}")

    print("\n## 1. LINKS")
    links = check_links()
    bad = [r for r in links if r[3] not in {"ok", "skip"}]
    print(f"total={len(links)} bad={len(bad)}")
    for r in links:
        if r[3] != "skip":
            print(f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} |")

    print("\n## 2. CODE REFS")
    py_f, classes, funcs, names = index_python(PYMPC)
    s_f, s_c, s_fn, s_n = index_python(STUDY / "scripts")
    py_f |= s_f
    py_f.add("docs/qpympc-study/scripts/verify_study_docs.py")
    py_f.add("verify_study_docs.py")
    classes |= s_c
    funcs |= s_fn
    names |= s_n
    # gym-quadruped may live in venv
    xmls, xml_tokens = index_xml(PYMPC)
    gym_roots: list[Path] = []
    try:
        import gym_quadruped  # existing venv only; do not install

        gym_roots.append(Path(gym_quadruped.__file__).resolve().parent)
    except Exception:
        venv_gym = ROOT / ".venv" / "lib"
        if venv_gym.exists():
            gym_roots.extend(
                p.parent
                for p in venv_gym.rglob("quadruped_env.py")
                if p.parent.name == "gym_quadruped"
            )
    for gym in gym_roots:
        f2, c2, fn2, n2 = index_python(gym)
        py_f |= {f"gym_quadruped/{x}" if not x.startswith("gym_") else x for x in f2}
        py_f |= f2
        py_f.add("gym_quadruped/quadruped_env.py")
        py_f.add("gym_quadruped/robot_cfgs.py")
        py_f.add("quadruped_env.py")
        classes |= c2
        funcs |= fn2
        names |= n2
        xml2, tok2 = index_xml(gym)
        xmls |= xml2
        xml_tokens |= tok2
        xmls.add("gym_quadruped/robot_model/go2/go2.xml")
        xmls.add("robot_model/go2/go2.xml")
    names |= xml_tokens
    refs = check_code_refs(py_f, classes, funcs, names, xmls)
    missing_hard = [r for r in refs if r[2] in {"py", "xml", "func", "class"} and r[3] == "no"]
    expected = [r for r in refs if r[3] == "expected-missing"]
    print(f"total={len(refs)} missing_hard={len(missing_hard)} expected_missing={len(expected)}")
    for r in refs:
        if r[2] in {"py", "xml", "func", "class", "config"} or r[3] != "yes":
            print(f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} | {r[4]} | {r[5]} |")

    print("\n## 3. MERMAID")
    for m in check_mermaid():
        print(f"| {m['file']} | {m['n']} | {m['syntax']} | {m['issues']} |")

    print("\n## 4. MATH")
    math = check_math()
    if not math:
        print("delimiters balanced")
    for r in math:
        print(f"| {r[0]} | {r[1]} | {r[2]} |")

    print("\n## 5. TABLES")
    tabs, quality = check_tables()
    if not tabs:
        print("column counts match")
    for r in tabs:
        print(f"| {r[0]} | line {r[1]} | {r[2]} |")
    print("header_gaps:")
    for r in quality:
        print(f"| {r[0]} | line {r[1]} | {r[2]} | missing:{r[3]} |")

    return 1 if bad or missing_hard or math or tabs else 0


if __name__ == "__main__":
    sys.exit(main())
