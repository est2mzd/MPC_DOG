#!/usr/bin/env bash
# 先端 × 実装安定 OSS のみ clone
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXT="$ROOT/external"
mkdir -p "$EXT"

clone_if_missing() {
  local url="$1"
  local dir="$2"
  local branch="${3:-}"
  if [[ -d "$EXT/$dir/.git" ]]; then
    echo "[skip] $dir"
    return
  fi
  echo "[clone] $url -> external/$dir ${branch:+-b $branch}"
  if [[ -n "$branch" ]]; then
    git clone --depth 1 -b "$branch" "$url" "$EXT/$dir"
  else
    git clone --depth 1 "$url" "$EXT/$dir"
  fi
}

# ★1 メイン: IIT DLS Lab — 実機 Unitree, acados + JAX sampling
clone_if_missing "https://github.com/iit-DLSLab/Quadruped-PyMPC.git" "Quadruped-PyMPC"

# 実機デプロイ周辺（Quadruped-PyMPC README 参照）
clone_if_missing "https://github.com/iit-DLSLab/muse.git" "muse" "unitree_sdk"
clone_if_missing "https://github.com/iit-DLSLab/unitree-ros2-dls.git" "unitree-ros2-dls"

# ★2 Whole-body MPC — DeepMind MuJoCo MPC + CMU deploy (Go1 branch)
clone_if_missing "https://github.com/johnzhang3/mujoco_mpc.git" "mujoco_mpc_go1" "go1"
clone_if_missing "https://github.com/johnzhang3/mujoco_mpc_deploy.git" "mujoco_mpc_deploy"

# ★3 ETH系 — OCS2 ROS2 + quadruped controller (知覚NMPC path)
clone_if_missing "https://github.com/legubiao/ocs2_ros2.git" "ocs2_ros2" "ros2"
clone_if_missing "https://github.com/legubiao/quadruped_ros2_control.git" "quadruped_ros2_control"

# Menagerie（MuJoCo 公式モデル）
clone_if_missing "https://github.com/google-deepmind/mujoco_menagerie.git" "mujoco_menagerie"

echo ""
echo "=== Next: Quadruped-PyMPC (recommended start) ==="
echo "  cd external/Quadruped-PyMPC"
echo "  # follow README_install.md (acados build required)"
echo ""
echo "=== Alt: MuJoCo whole-body iLQR ==="
echo "  cd external/mujoco_mpc_go1 && follow google-deepmind/mujoco_mpc build"
echo ""
echo "=== Alt: OCS2 perceptive (ROS2) ==="
echo "  see external/ocs2_ros2 README section 4.1 Perceptive Locomotion"
