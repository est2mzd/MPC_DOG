#!/usr/bin/env bash
# 背景: Step 02ではexternal/を変更せず公式実装の動作を記録する必要があり、
#       記録用ハーネス(step_02_frequency.py)をどこからでも同じ手順で
#       実行できるようにするため作成した。
# 目的: src/trial/step_02_frequency.py を実行し、基準ログとGIFを生成する。
# 前提: acadosビルド済み、`pip install -e .`(Quadruped-PyMPC)完了済み。

set -euo pipefail

# 環境変数の設定
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)" # リポジトリルートの絶対パス
PYMPC_DIR="${REPO_ROOT}/external/Quadruped-PyMPC" # Quadruped-PyMPCの絶対パス
ACADOS_DIR="${PYMPC_DIR}/quadruped_pympc/acados"  # acadosの絶対パス

# acadosの共有ライブラリのパスを設定
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${ACADOS_DIR}}" # acadosのソースディレクトリ
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ACADOS_SOURCE_DIR}/lib" # acadosの共有ライブラリのパス

cd "${REPO_ROOT}" # 実行場所はプロジェクトルート(external/には入らない)
uv run python "./src/trial/step_02_frequency.py" # pythonコード を実行
