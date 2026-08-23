# Log 01: Baseline

対応プロンプト: コード/Markdownを変更せず現在のBaselineを記録する。
記録日: 2026-08-23。学習資料本文は未修正。

| 項目 | 確認結果 | 根拠ファイル・コマンド | 確定/未確定 |
|---|---|---|---|
| Git remote | `origin` = `https://github.com/est2mzd/MPC_DOG.git` | `git remote -v`（`mpc_dog`） | 確定 |
| 現在のbranch | `main` | `git rev-parse --abbrev-ref HEAD` | 確定 |
| wrapper commit | `3adfad9f814c499fb996cf046c8fb4ac3a574e55` | `git rev-parse HEAD` | 確定 |
| 未Commit差分 | `docs/pympc_2day/notebooks/05_*.ipynb`, `06_*.ipynb` 変更。`docs/qpympc-study/` 未追跡 | `git status` | 確定 |
| Python | 3.11.16 | `sys.version` | 確定 |
| MuJoCo | 3.11.0 | `importlib.metadata.version('mujoco')` | 確定 |
| CasADi | 3.7.2 | `importlib.metadata.version('casadi')` | 確定 |
| acados_template | 0.5.1 | `importlib.metadata.version('acados_template')` | 確定 |
| acados Cライブラリ本体commit | 未確認 | バイナリ経路のみ | 未確定 |
| gym-quadruped | 1.1.5（`.venv` の `site-packages`） | `importlib.metadata.version('gym_quadruped')` | 確定 |
| gym-quadruped git commit | pip wheel。git commitなし | `site-packages` | 未確定（版番号のみ確定） |
| Quadruped-PyMPC取得 | `external/Quadruped-PyMPC` は `.git` なしの展開ディレクトリ | `ls external/Quadruped-PyMPC/.git` | 確定 |
| Quadruped-PyMPC commit | この作業領域ではgit hashを取れない。学習資料初版記録は zip comment `cc145a2` | `.git` 不在 | 未確定（現行treeはgit管理外） |
| Menagerie Go2 | 実行時未ロード。Menagerie checkoutなし | `00_README` / `QuadrupedEnv` 読込 | 確定（未使用） |
| Entrypoint | `external/Quadruped-PyMPC/simulation/simulation.py` の `run_simulation()`。`if __name__` から起動 | 同ファイル | 確定 |
| 標準Config | ディスク上 `quadruped_pympc/config.py`。`robot='go2'` | `config.py` | 確定 |
| 標準Gait | `simulation_params['gait']='trot'`。`step_freq=1.35`, `duty_factor=0.74`, `type=GaitType.TROT.value` | `config.py` | 確定 |
| 標準MPC | `mpc_params['type']='nominal'` | `config.py` | 確定 |
| Simulation timestep | `simulation_params['dt']=0.002` s | `config.py` | 確定 |
| MPC更新周期 | `mpc_frequency=100` Hz。`step_num % 5 == 0` | `config.py`, `quadruped_pympc_wrapper.py` | 確定 |
| MPC horizon / dt | 12段、0.02 s、予測長 0.24 s | `config.py` | 確定 |

実行に必要なコマンド（インストールなし）:

```text
python3 external/Quadruped-PyMPC/simulation/simulation.py
```

またはワークショップ用 headless スクリプト（本ログでは未再実行）。
