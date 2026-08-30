#!/usr/bin/env bash
# 背景: acadosはCライブラリで、Quadruped-PyMPC実行前にcmake/makeで手動ビルド
#       する必要があった。その手順を再現可能にするため作成した。
# 目的: acadosのC実装(blasfeo/hpipm/acados本体)をビルドし、acados_templateを入れる。
# 前提: cmake/make/gcc/g++ が導入済みであること。1回実行すれば以後は不要。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)" # リポジトリルートの絶対パス
ACADOS_DIR="${REPO_ROOT}/external/Quadruped-PyMPC/quadruped_pympc/acados" # acadosの絶対パス

mkdir -p "${ACADOS_DIR}/build" # ビルドディレクトリを作成
cd "${ACADOS_DIR}/build"

# -DACADOS_WITH_SYSTEM_BLASFEO: ON/OFF。READMEはONだがシステムにblasfeoが無くcmakeが失敗する。
#   OFF=acados同梱のソースからビルドする(こちらを使う)。
# -DCMAKE_POLICY_VERSION_MINIMUM=3.5: blasfeo/hpipm側の古いcmake_minimum_requiredとの
#   互換性を確保するために必要な最小ポリシーバージョン。
cmake -DACADOS_WITH_SYSTEM_BLASFEO:BOOL=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
make install -j4 # -j4: 並列ビルドのジョブ数(コア数に応じて増減可)

# -e: editableインストール(ソースを直接参照、コピーしない)
# --project: どのuvプロジェクト(=.venv)にインストールするかをREPO_ROOTで明示
#   (このコマンドの直前でacadosビルドディレクトリへcd済みのため必要)
uv pip install --project "${REPO_ROOT}" -e "${ACADOS_DIR}/interfaces/acados_template"
