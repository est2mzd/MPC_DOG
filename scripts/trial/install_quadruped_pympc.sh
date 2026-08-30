#!/usr/bin/env bash
# 背景: acadosのビルドとQuadruped-PyMPC本体のインストールは別工程であり、
#       数式(モデル)変更時に毎回acadosを再ビルドしないよう分離している。
# 目的: Quadruped-PyMPC本体をeditableインストールする(README_install.md手順7)。
# 前提: scripts/trial/build_acados.sh 済みであること。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)" # リポジトリルートの絶対パス
PYMPC_DIR="${REPO_ROOT}/external/Quadruped-PyMPC" # Quadruped-PyMPCの絶対パス

# -e: editableインストール(ソースを直接参照、コピーしない)
# --project: どのuvプロジェクト(=.venv)にインストールするかをREPO_ROOTで明示
uv pip install --project "${REPO_ROOT}" -e "${PYMPC_DIR}"
