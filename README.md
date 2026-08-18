# mpc_dog — 四足MPC（先端 × 実装安定）

**出発点:** 論文デモ再現ではなく、**実機実績 + 活発メンテ + 2024–2026 技術** の OSS。

## 2日ワークショップ（GRF · MPC · WBC）

👉 **[docs/pympc_2day/WORKSHOP.md](docs/pympc_2day/WORKSHOP.md)** — 統合教材（計算結果・GIF 付き）

```bash
./scripts/setup_references.sh
./scripts/setup_uv_workshop.sh
source .venv/bin/activate && . .env.workshop
python scripts/run_workshop_pipeline.py   # param study + GIF + 実行済み notebooks
jupyter lab docs/pympc_2day/notebooks/
```

## 採用スタック

| 優先 | スタック | 用途 |
|------|----------|------|
| **★1** | [Quadruped-PyMPC](https://github.com/iit-DLSLab/Quadruped-PyMPC) | **ワークショップ・コンサル土台** |
| **★2** | [mujoco_mpc](https://github.com/google-deepmind/mujoco_mpc) | 全身 iLQR |
| **★3** | [ocs2_ros2](https://github.com/legubiao/ocs2_ros2) | 知覚統合 NMPC |

## ディレクトリ

| パス | 内容 |
|------|------|
| `docs/pympc_2day/` | WORKSHOP.md + 実行済み notebooks + assets |
| `scripts/` | `setup_uv_workshop.sh`, `run_workshop_pipeline.py` 等 |
| `configs/pympc_presets/` | セッション別プリセット |
| `external/` | Quadruped-PyMPC clone 先 |

## 参考

- [docs/stack_selection.md](docs/stack_selection.md)
- [docs/top2_stack_comparison.md](docs/top2_stack_comparison.md)
- [docs/quadruped_mpc_rl_survey.md](docs/quadruped_mpc_rl_survey.md)
