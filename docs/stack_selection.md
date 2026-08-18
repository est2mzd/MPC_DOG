# スタック選定 — 先端技術 × 実装安定性

前回の go2-convex-mpc 起点は **不適切** でした。  
評価軸を明示し、コンサル/開発の **出発点** を再定義します。

---

## 評価軸（5項目）

| 軸 | 意味 | 重み |
|----|------|------|
| **論文鮮度** | 2023–2026 の査読付き成果と一致 | 高 |
| **実機実績** | Unitree 等で動いた公開証拠 | 高 |
| **メンテ** | 2025–2026 に commit がある | 中 |
| **スタック成熟度** | acados / DeepMind / ETH OCS2 等の実績基盤 | 高 |
| **不整地への伸び** | 足場opt / 知覚 / whole-body | 高 |

---

## 結論：出発点はこの3段

```
Phase 1 (必須)  Quadruped-PyMPC     … SRB MPC + 足場opt + 実機パイプライン
Phase 2 (並行)  MuJoCo MPC iLQR    … 全身MPC, Sim=Real モデル
Phase 3 (拡張)  OCS2 Perceptive   … Grandia系 NMPC + elevation map
```

**Gym / RL** は MPC 土台の **比較実験** 用。出発点にしない。

---

## ★1 Quadruped-PyMPC（メイン）

- **Repo:** https://github.com/iit-DLSLab/Quadruped-PyMPC (~490★, 2026-05更新)
- **論文:**
  - IROS 2024 — GPU Sample-Based Stochastic MPC
  - RAL 2025 — Adaptive Non-Linear Centroidal MPC + stability guarantees
- **技術:**
  - 勾配MPC: **acados** (<5ms on i7), RTI / Advanced-step RTI
  - サンプリングMPC: **JAX** MPPI/CEM (10k rollouts <2ms on laptop GPU)
  - 足場最適化, ZMP/CoM constraints, Lyapunov criteria
- **実機:** muse (state est) + unitree-ros2-dls
- **Sim:** MuJoCo
- **なぜ先端か:** 2018 Convex MPC の再実装ではなく、**2024–2025 の centroidal / sampling MPC 研究の参照実装**
- **なぜ安定か:** IIT DLS Lab、実機デプロイ手順が README に一体

**触る設定:** `quadruped_pympc/config.py`（robot, mpc_type, gait, foothold opt 等）

---

## ★2 MuJoCo MPC + iLQR（全身・Sim/Real同一）

- **Paper:** Zhang et al. 2025 — Whole-Body MPC with MuJoCo (ICRA accepted)
- **Repo:**
  - Solver: https://github.com/google-deepmind/mujoco_mpc (DeepMind メンテ)
  - Deploy: https://github.com/johnzhang3/mujoco_mpc_deploy
  - Go1: `git clone -b go1 https://github.com/johnzhang3/mujoco_mpc`
- **実機:** Go1, Go2, H1 — iLQR ~50Hz, TV-LQR ~300Hz
- **なぜ先端か:** SRB 近似なしの **whole-body**, MuJoCo の動力学をそのまま MPC に使用
- **なぜ安定か:** DeepMind 公式 MJPC + CMU 実機論文 + Menagerie モデル
- **注意:** 状態推定は WIP（OptiTrack 等）。Phase 2 向き

---

## ★3 OCS2 ROS2 + quadruped_ros2_control（知覚NMPC）

- **Repo:**
  - https://github.com/leggedrobotics/ocs2 (ETH 本家)
  - https://github.com/legubiao/ocs2_ros2 (ROS2, 2026-04 release v1.2.0)
  - https://github.com/legubiao/quadruped_ros2_control (526★, Go2 等)
- **論文 lineage:** Grandia 2023 Perceptive NMPC, OCS2 legged robot examples
- **機能:** NMPC + WBC, **Perceptive Locomotion** サンプル (elevation map)
- **なぜ先端か:** 不整地で足場+全身を NMPC — コンサルで「ETH系」と言える
- **なぜ安定か:** 10年+ OCS2 エコシステム、ROS2 移植が進行中
- **注意:** C++ ビルド重い。legged_control は **非推奨**（作者が新框架へ移行中）→ **ocs2_ros2 を使う**

---

## 参考（レイヤー2 — 出発点ではない）

| スタック | 位置づけ |
|----------|----------|
| RL-augmented MPC (DRCL-USC) | ICRA'24, A1 3m/s — **MPC上にRLプラグイン**。Phase1後 |
| legged_gym / Isaac | RL — MPC比較・ハイブリッド用 |
| go2-convex-mpc | 2018論文の学生再実装 — **学習用のみ** |

---

## AIコーディングでの試行錯誤（Quadruped-PyMPC）

```
1. config.py だけ変更（mpc_type: gradient | sampling）
2. 1機能フラグだけ（foothold_optimization, rti, …）
3. MuJoCo sim でログ → 転倒なら acados solver status を AI に渡す
4. git で1変更1commit
```

**禁止:** ゼロから QP 書く / go2-convex-mpc に戻る

---

## 不整地・犬速度への道筋（【推測】）

| 段階 | スタック | 到達イメージ |
|------|----------|--------------|
| 1 | PyMPC + gradient + foothold opt | 段差・斜面 sim |
| 2 | PyMPC + sampling (MPPI) | モデル誤差に強い走行 |
| 3 | OCS2 perceptive または MuJoCo iLQR + terrain | 知覚/全身で 1.5–2 m/s 級 |
| 4 | + RL-augmented 層 | 盲階段・荷重変動 |

---

## リンク

- Quadruped-PyMPC install: https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/README_install.md
- MuJoCo iLQR: https://johnzhang3.github.io/mujoco_ilqr/
- OCS2 perceptive: https://github.com/legubiao/ocs2_ros2 (advance examples)
- 自社サーベイ: `docs/quadruped_mpc_rl_survey.md`
