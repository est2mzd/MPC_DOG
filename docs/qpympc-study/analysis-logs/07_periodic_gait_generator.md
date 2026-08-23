# Log 07: PeriodicGaitGenerator

対応プロンプト: Trot位相の決定箇所、MPCが位相を変えられるか、現在/将来接触列。本文未修正。

標準: `gait='trot'`, `step_freq=1.35`, `duty_factor=0.74`, `horizon=12`, `mpc_dt=0.02`, `simulation_dt=0.002`。

## 確認項目

| # | 項目 | 確認結果 | 根拠 |
|---|---|---|---|
| 1 | `gait_type` | `WBInterface.__init__` が `gait_params['trot']['type']` = `GaitType.TROT.value` = `0` を渡す | `config.py`, `quadruped_utils.py`, `wb_interface.py` |
| 2 | `step_freq` | 初期 1.35 Hz。`optimize_swing==1` のときだけ `best_sample_freq` で上書き。標準 `optimize_step_freq=False` なので固定 | `config.py`, `wb_interface.py` 357–361 |
| 3 | `duty_factor` | 0.74。周波数更新時も duty は変えない | `gait_params['trot']` |
| 4 | `phase_offset` | Trot: `[0.5, 1.0, 1.0, 0.5]`。`reset()` が gait_type で選択 | `periodic_gait_generator.py` 24–25 |
| 5 | `phase_signal` | `_phase_signal`。reset時は offset そのもの。`run()` が進める | 同 43, 48–56 |
| 6 | 位相更新式 | \(\phi_i \leftarrow (\phi_i + \Delta t \cdot f)\bmod 1\) | `run()` |
| 7 | 接地判定 | `_init[i]=False` のとき \(c_i=1\) iff \(\phi_i < d\)。reset後 `_init` は全False | `run()` 68–74 |
| 8 | 脚順 | 配列順 FL, FR, RL, RR。`legs_order=('FL','FR','RL','RR')` | `wb_interface.py` 31 |
| 9 | `contact_sequence` shape | 標準 `(4, 12)`。`FULL_STANCE` だけ `(4, 2*horizon)` を返して即reset | `compute_contact_sequence` |
| 10 | Horizon方向 | 列0が現在、列が増えるほど未来 | 同 107–116 |
| 11 | MPC timestep | 標準 `contact_sequence_dts=[0.02]`, `lenghts=[12]`。lookaheadは `run(0.02, step_freq)` | `wb_interface.py` 62–63 |
| 12 | `current_contact` | `contact_sequence[:,0]` を4要素に分解 | `wb_interface.py` 208–210 |
| 13 | `start_and_stop_activated` | `__init__` で False。標準simは触らない。ROS2 `console.py` だけ True/False 切替 | `periodic_gait_generator.py` 16, `ros2/console.py` |
| 14 | Full stance切替 | `set_full_stance()` が `gait_type=FULL_STANCE` にして `reset()`。`update_start_and_stop` からのみ。標準未到達 | 120–126, 181–196 |
| 15 | 周波数更新 | `compute_stance_and_swing_torque` 冒頭。`optimize_swing==1` のとき `pgg.step_freq` と `frg.stance_time` と swing_period を更新 | `wb_interface.py` 357–361 |
| 16 | Batched frequency | `SRBDBatchedControllerInterface.optimize_gait` が候補 `[1.4,2.0,2.4]` ごとに仮PGGを作り接触列をバッチ。`optimize_step_freq=False` なら未生成 | `srbd_batched_controller_interface.py` |

2回の `run`:

1. `pgg.run(simulation_dt, step_freq)` … 位相だけ進める。戻り接触は捨てる。500 Hz。
2. `compute_contact_sequence` … 位相を保存し、`run(0.0)` で列0、以降 `run(dt)` で未来列、最後に位相復元。

よってlookaheadはシミュレータ位相を永続的に進めない。

`offset=1.0` は最初の `% 1.0` で `0.0` になる。実効初期位相は `[0.5, 0.0, 0.0, 0.5]`。

## Trot 最初の2周期

\(T=1/1.35\approx0.7407\) s。`duty=0.74`。実効初期 \(\phi=[0.5,0,0,0.5]\)。接触は \(\phi<0.74\)。

| 時刻 [s] | 位相 FL/FR/RL/RR | FL | FR | RL | RR | 支持脚組 |
|---|---|---|---|---|---|---|
| 0.000 | 0.50 / 0.00 / 0.00 / 0.50 | 1 | 1 | 1 | 1 | 4脚（overlap） |
| 0.178 | 0.74 / 0.24 / 0.24 / 0.74 | 0 | 1 | 1 | 0 | FR+RL |
| 0.370 | 0.00 / 0.50 / 0.50 / 0.00 | 1 | 1 | 1 | 1 | 4脚 |
| 0.548 | 0.24 / 0.74 / 0.74 / 0.24 | 1 | 0 | 0 | 1 | FL+RR |
| 0.741 | 0.50 / 0.00 / 0.00 / 0.50 | 1 | 1 | 1 | 1 | 4脚（1周期） |
| 0.919 | 0.74 / 0.24 / 0.24 / 0.74 | 0 | 1 | 1 | 0 | FR+RL |
| 1.111 | 0.00 / 0.50 / 0.50 / 0.00 | 1 | 1 | 1 | 1 | 4脚 |
| 1.289 | 0.24 / 0.74 / 0.74 / 0.24 | 1 | 0 | 0 | 1 | FL+RR |
| 1.481 | 0.50 / 0.00 / 0.00 / 0.50 | 1 | 1 | 1 | 1 | 4脚（2周期） |

`duty=0.74>0.5` のため対角切替の間に4脚overlapがある。`04` の概念行列（overlapなしの2段ずつ）は現行数値列ではない。

## 数式とコード

### 対角脚が同相になる根拠

Trot offset は FL=RR=0.5、FR=RL=1.0。同じ脚は同じ初期位相を持ち、同じ `dt*f` で進む。よって常に \(\phi_{FL}=\phi_{RR}\)、\(\phi_{FR}=\phi_{RL}\)。

### 別対角組が逆相になる根拠

\(1.0\equiv0\pmod{1}\)。差は \(0.5\)。一方が \(\phi\) のとき他方は \(\phi+0.5\bmod 1\)。

### MPCがTrotの逆相を回答できない理由

接触 \(c_{i,k}\) は決定変数ではない。`compute_contact_sequence` の出力が各段 `p[0:4]` に入り、力学は \(c_i F_i\) でGateする。OCPは \(F\) と足速度を動かすが `stanceFL` 等は動かさない。

### `contact_sequence` → acados parameter

```text
PGG.compute_contact_sequence
  -> WBInterface.update_state_and_reference の戻り
  -> Wrapper.compute_actions
  -> SRBDControllerInterface.compute_control(..., contact_sequence)
  -> Acados_NMPC_Nominal.compute_control
       FL_contact_sequence = contact_sequence[0] 等
       param = [FL[j], FR[j], RL[j], RR[j], mu, stance_prox..., ...]
       solver.set(j, "p", param)
```

### 同じ `current_contact` → 低レベル

```text
current_contact = contact_sequence[:,0]
  -> FRG lift/touch エッジ
  -> TerrainEstimator（高さ平均には現在未使用）
  -> SRBD mask: nmpc_GRFs.leg *= current_contact[i]
  -> compute_stance_and_swing_torque:
       全脚でまず -J.T @ GRF（遊脚はmask後0）
       current_contact[i]==0 なら swing Cartesian
```

## 資料照合

| 資料 | 記載 | 判定 | 必要な修正 |
|---|---|---|---|
| `04` 位相式・判定式 | コードと一致 | 正しい | なし |
| `04` 2回runと復元 | 一致 | 正しい | なし |
| `04` Trot行列 | 概念例と明記。duty=0.74のoverlapなし | 不完全 | 上表を正本にするか「概念」を残す |
| `04` start/stop未到達 | ROS2のみTrue | 正しい | なし |
| `08` 位相は決定変数でない | 一致 | 正しい | なし |
| `08` 遊脚GRFのOCP内ゼロ | 未再検証と書いてFへ | 正しい（本ログ10/11で再検証） | 本文は未変更 |
| E §2 「MPCが逆相を回答」訂正 | 固定schedule | 正しい | なし |

未確認: `_init=True` 経路はreset後使われない。`set_phase_signal` はbatched仮PGGが呼ぶ。
