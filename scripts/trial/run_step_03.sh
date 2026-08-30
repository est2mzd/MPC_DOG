#!/usr/bin/env bash
# 背景: Step 03 では前進方向に穴(トレンチ)を並べた平面マップで、
#       穴に落ちずに前進できるかを記録する。external/ は変更せず、
#       マップ(src/trial/assets/scene_gaps.xml)を gym_quadruped 側へ
#       実行時コピーして読み込ませる。
# 目的: src/trial/step_03_gap_crossing.py を実行し、ログとGIFを生成する。
# 前提: acados ビルド済み、Quadruped-PyMPC インストール済み(Step 01/02 と同じ)。
# 上書き: 環境変数 STEP03_VEL / STEP03_FREQ / STEP03_SECONDS

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYMPC_DIR="${REPO_ROOT}/external/Quadruped-PyMPC"
ACADOS_DIR="${PYMPC_DIR}/quadruped_pympc/acados"

export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${ACADOS_DIR}}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ACADOS_SOURCE_DIR}/lib"

cd "${REPO_ROOT}"
uv run python "./src/trial/step_03_gap_crossing.py"
