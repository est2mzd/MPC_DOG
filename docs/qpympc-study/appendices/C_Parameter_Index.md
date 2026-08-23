# User-Tuned Parameter Index

仕様から決まる質量、慣性、関節定格は除外する。数値の正本は[07](../07_MPC_Formulation.md) §4。調整の物語は[14](../14_MPC_and_Controller_Tuning.md)。

Shapeは重みベクトル以外ほぼ scalar / flag。Frameは指令・状態ではなく設定値なので「なし」。

| パラメータ | 場所 | 既定例 | 単位 | Shape | Frame | 調整段階 | 標準で効くか（Optional区分） |
|---|---|---|---|---|---|---|---|
| Gait type | `config.py` | `trot` | なし | str | なし | A | はい |
| Step frequency | `gait_params['trot']` | 1.35 | Hz | scalar | なし | A | はい |
| Duty factor | `gait_params['trot']` | 0.74 | 無次元 | scalar | なし | A | はい |
| Step height | `simulation_params` | `0.2*hip_height` | m | scalar | なし | A | はい |
| `ref_z` | `simulation_params` | `0.28*1.08` | m | scalar | なし | A | はい |
| `hip_offset` | FRG | 0.1 | m | scalar | なし | B | はい |
| `mu` | `mpc_params` | 0.42 | なし | scalar | なし | A | はい（OCP。Plant摩擦とは別） |
| `mpc_frequency` | `simulation_params` | 100 | Hz | scalar | なし | B | はい |
| MPC horizon | `mpc_params` | 12 | 段 | scalar | なし | B | はい |
| MPC dt | `mpc_params` | 0.02 | s | scalar | なし | B | はい |
| Velocity weight | `set_weight()` | `[200,200,200]` | コスト対角 | `(3,)` | なし | A | はい |
| Height weight | `set_weight()` | z=1500 | コスト対角 | scalar（xのz） | なし | A | はい |
| Base angle weight | `set_weight()` | `[500,500,0]` | コスト対角 | `(3,)` | なし | A | はい |
| Angular rate weight | `set_weight()` | `[20,20,50]` | コスト対角 | `(3,)` | なし | A | はい |
| Foot position weight | `set_weight()` | `[300,300,300]` | コスト対角 | `(3,)`×4 | なし | B | はい |
| Foot velocity weight | `set_weight()` | `[1e-4,1e-4,1e-5]` | コスト対角 | `(3,)`×4 | なし | B | はい |
| GRF weight | `set_weight()` | 0.001 | コスト対角 | scalar×12 | なし | A/B | はい |
| Swing position gain | `simulation_params` | 500 | コード上無次元ゲイン | scalar | なし | A | はい |
| Swing velocity gain | `simulation_params` | 10 | コード上無次元ゲイン | scalar | なし | A | はい |
| Foothold optimization | `mpc_params` | True | なし | bool | なし | B | はい |
| Foothold constraints | `mpc_params` | False | なし | bool | なし | B/C | **OFF** |
| Stability | `mpc_params` | False | なし | bool | なし | B/C | **OFF** |
| Frequency candidates | `step_freq_available` | `[1.4,2.0,2.4]` | Hz | `(3,)` | なし | C | **OFF**（`optimize_step_freq=False`）。1.35は候補に無い |
| Integral enable | `mpc_params` | False | なし | bool | なし | C | **OFF** |
| Terrain adaptation | `simulation_params` | `blind` | なし | str | なし | C | HeightMapなし |
| Reflex | `simulation_params` | `False` | なし | bool/str | なし | C | **OFF**。有効時モード名が `tracking` / `geom_contact` |
| Joint impedance | `simulation_params` | 10/2 | 混在 | scalar×2 | なし | C | **実装あり・標準無効** |
| GRF rate weight | `input_rates` 専用 | — | コスト対角 | — | なし | D | **nominal未実装** |
