#!/usr/bin/env bash
# 階段の入口。初期位置と時間の計算は run_quadsdk_jp_stair.sh にある。
# run_quadsdk_gap_1m.sh は溝と平地のまま残す。
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "${REPO_ROOT}/scripts/trial/run_quadsdk_jp_stair.sh"
