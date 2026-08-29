#!/usr/bin/env bash
# ============================================================================
# Step 01: Quadruped-PyMPC 公式サンプルの起動スクリプト
#
# 目的:
#   external/Quadruped-PyMPC を一切変更せず、README_install.md が示す公式の
#   最小実行方法(`python3 simulation/simulation.py`)をそのまま呼び出す。
#   MPC_DOG独自のロジックはここには一切含まない(参照実装をそのまま動かす
#   だけの薄いラッパー)。
#
# 前提(README_install.md より):
#   1. Pixi または Conda 環境が有効化されていること
#   2. `git submodule update --init --recursive` 済みであること
#   3. quadruped_pympc/acados が cmake + make でビルド済みであること
#   4. ACADOS_SOURCE_DIR / LD_LIBRARY_PATH が設定されていること
#   5. `pip install -e .`(Quadruped-PyMPC自身)が完了していること
#
# 2026-08-29時点の状態(docs/steps/step_01_reference_baseline.md 参照):
#   当初このホストには cmake / make / gcc / g++ が無く実行不可だったが、
#   ユーザーがツールチェインを導入後、acadosのビルド・公式サンプルの実行に
#   成功した。以下のpreflightチェックはツールチェイン欠如等の環境不備を
#   早期に検知して原因を明示するために残してある(場当たり的に実行を
#   続けて不可解なエラーを出さないため)。
#
# 既定では、公式 simulation.py をそのまま実行するのではなく、
# src/trial/record_step01_baseline.py (MPC_DOG側の記録ハーネス。制御ロジックは
# 一切変更せず呼び出すだけ、詳細は同ファイル冒頭のdocstring参照) を実行し、
# ログ(artifacts/logs/step_01/)とGIF(artifacts/gifs/)を生成する。
# 公式スクリプトそのものを対話的に動かしたいだけの場合は
# `RUN_OFFICIAL_ONLY=1 bash scripts/trial/run_reference_baseline.sh` を使うこと。
# ============================================================================

set -euo pipefail

# このスクリプトは scripts/trial/ 配下(リポジトリ直下から2階層下)にあるため
# 2階層上をリポジトリルートとする。
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYMPC_DIR="${REPO_ROOT}/external/Quadruped-PyMPC"
ACADOS_DIR="${PYMPC_DIR}/quadruped_pympc/acados"
VENV_PYTHON="${REPO_ROOT}/.venv/bin/python"

# README_install.mdの手順6(.bashrcへの追記)の代わりに、このスクリプトの
# プロセス内だけで環境変数を設定する(ユーザー環境やexternal/を変更しない)。
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${ACADOS_DIR}}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ACADOS_SOURCE_DIR}/lib"

echo "=== Step 01 reference baseline: preflight checks ==="

# --- 1. Quadruped-PyMPC submodule が存在するか -----------------------------
if [ ! -f "${PYMPC_DIR}/simulation/simulation.py" ]; then
  echo "[NG] ${PYMPC_DIR}/simulation/simulation.py が見つかりません。" >&2
  echo "     external/Quadruped-PyMPC submodule が初期化されているか確認してください。" >&2
  exit 1
fi
echo "[OK] simulation/simulation.py を発見: ${PYMPC_DIR}/simulation/simulation.py"

# --- 2. C/C++ ビルドツールチェイン(acadosのビルドに必須) -------------------
missing_tools=()
for tool in cmake make gcc g++; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    missing_tools+=("${tool}")
  fi
done
if [ "${#missing_tools[@]}" -gt 0 ]; then
  echo "[NG] 以下のビルドツールが見つかりません: ${missing_tools[*]}" >&2
  echo "     acados のビルド(README_install.md 手順5)にはこれらが必須です。" >&2
  echo "     詳細: docs/steps/step_01_reference_baseline.md の「未解決事項」参照。" >&2
  exit 1
fi
echo "[OK] cmake / make / gcc / g++ を検出"

# --- 3. acados が既にビルド済みか(quadruped_pympc/acados/lib の有無で判定) -
if [ ! -d "${ACADOS_DIR}/lib" ]; then
  echo "[NG] ${ACADOS_DIR}/lib が見つかりません。acados が未ビルドです。" >&2
  echo "     README_install.md 手順5(cmake && make install)を先に実行してください。" >&2
  exit 1
fi
echo "[OK] acados のビルド成果物(lib/)を検出"

# --- 4. ACADOS_SOURCE_DIR が設定されているか --------------------------------
if [ -z "${ACADOS_SOURCE_DIR:-}" ]; then
  echo "[NG] 環境変数 ACADOS_SOURCE_DIR が未設定です。" >&2
  echo "     export ACADOS_SOURCE_DIR=\"${ACADOS_DIR}\" を実行してください。" >&2
  exit 1
fi
echo "[OK] ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"

echo "=== preflight OK ==="

if [ "${RUN_OFFICIAL_ONLY:-0}" = "1" ]; then
  echo "=== RUN_OFFICIAL_ONLY=1: 公式 simulation.py を対話的にそのまま起動します ==="
  # 公式手順そのまま: external/Quadruped-PyMPC ディレクトリで simulation.py を実行する。
  # ロボット種別・MPCタイプ・歩容等は external/Quadruped-PyMPC/quadruped_pympc/config.py
  # 側の設定に従う(README_install.md 91行目の記載通り、このスクリプトからは変更しない)。
  cd "${PYMPC_DIR}"
  "${VENV_PYTHON}" simulation/simulation.py
else
  echo "=== 記録ハーネス(src/trial/record_step01_baseline.py)を実行します ==="
  cd "${PYMPC_DIR}"
  "${VENV_PYTHON}" "${REPO_ROOT}/src/trial/record_step01_baseline.py"
fi
