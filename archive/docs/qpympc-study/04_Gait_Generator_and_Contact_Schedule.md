# Gait Generator and Contact Schedule

## 1. 結論

Gait Generatorは、MPCより先に「各脚が将来いつ立脚・遊脚になるか」を決める。Trot位相はMPCの回答ではなく、MPCが変更できない既知の接触モードである。

本章がGaitと`contact_sequence`の正本である。MPCへの入れ方は[08](08_Gait_MPC_Coupling.md)。到達不能な`start_and_stop`は[16](16_Code_Map_and_Call_Graph.md)。

## 2. Gaitパラメータ

既定は`simulation_params['gait']='trot'`である。

| コードキー | 意味 | trot既定 |
|---|---|---|
| `gait_params[gait]['type']` | `GaitType`値 | `GaitType.TROT.value` |
| `step_freq` | 周期周波数 | 1.35 Hz |
| `duty_factor` | 1周期中の接地割合 | 0.74 |
| `phase_offset` | 脚間位相差。FL,FR,RL,RR | `[0.5, 1.0, 1.0, 0.5]` |
| `_phase_signal` | 現在位相 | reset時はoffsetそのもの |

位相更新は、

\[
\phi_i\leftarrow(\phi_i+\Delta t\, f)\bmod 1
\]

接地判定は、

\[
c_i=
\begin{cases}
1 & \phi_i<d\\
0 & \phi_i\ge d
\end{cases}
\]

| 数式 | コード変数 |
|---|---|
| \(\phi_i\) | `_phase_signal[i]` |
| \(\Delta t\) | `run()`の`dt` |
| \(f\) | `step_freq` / `new_step_freq` |
| \(d\) | `duty_factor` |
| \(c_i\) | `contact[i]` |

対応コード: `quadruped_pympc/helpers/periodic_gait_generator.py` の `PeriodicGaitGenerator.run()`。

## 3. 2回の`run`と位相復元

`update_state_and_reference()`は次の順である。

1. `pgg.run(simulation_dt, step_freq)`。`simulation_dt=0.002`。戻り`contact`は捨て、生きている位相だけ進める。周期500 Hz。
2. `pgg.compute_contact_sequence(dts=[mpc_dt], lengths=[horizon])`。既定`mpc_dt=0.02`、`horizon=12`。

`compute_contact_sequence`は位相を保存し、`run(0.0)`で先頭列を評価し、lookahead列は`run(0.02)`で進め、最後に位相を復元する。したがってlookaheadはシミュレータ位相を永続的に進めない。

`use_nonuniform_discretization=False`のとき`contact_sequence.shape==(4, 12)`。行順はFL, FR, RL, RR。

| 入力 | shape | 単位 | frame | 出力 | shape | 単位 | frame |
|---|---|---|---|---|---|---|---|
| `_phase_signal` | `(4,)` | 0–1 | なし | `contact_sequence` | `(4,12)` | 0/1 | なし |
| `step_freq` | scalar | Hz | なし | `current_contact`（呼出側） | `(4,)` | 0/1 | なし |
| `contact_sequence_dts` | `[0.02]` | s | なし | | | | |
| `contact_sequence_lenghts` | `[12]` | 段数 | なし | | | | |

対応コード: `WBInterface.update_state_and_reference()` と `PeriodicGaitGenerator.compute_contact_sequence()`。

## 4. Trot

対角脚が同じ組になる。

- FLとRR（offset 0.5）
- FRとRL（offset 1.0）

次の行列は**概念例**であり、現行`phase_offset`と`duty_factor=0.74`から数値生成した列ではない。実列は`compute_contact_sequence`の出力を使う。

標準 \(d=0.74>0.5\) のため、対角切替の前後に**4脚接地のoverlap**がある。概念行列は overlap を描いていない。

\[
C^{\mathrm{Trot,\,concept}}
=
\begin{bmatrix}
1&1&0&0&1&1\\
0&0&1&1&0&0\\
0&0&1&1&0&0\\
1&1&0&0&1&1
\end{bmatrix}
\]

行はFL、FR、RL、RR、列は未来の予測段である。

## 5. 出力

```python
contact_sequence.shape == (4, horizon)  # 既定 (4, 12)
current_contact = contact_sequence[:, 0]
```

\[
c_i^{current}=C_{i,0}
\]

| 数式 | コード変数 |
|---|---|
| \(C\) | `contact_sequence` |
| \(c_i^{current}\) | `current_contact[i]` |

将来列はMPC予測、先頭列は現在の立脚・遊脚制御切替とGRF Maskに使う。

対応コード: `wb_interface.py` の先頭列抽出、`srbd_controller_interface.py` のMask。

## 6. 速度指令との関係

デフォルトでは速度指令はGait種類を変更しない。`start_and_stop_activated`も通常Falseであり、目標速度ゼロでも設定された位相生成が継続し得る。`update_start_and_stop()`は標準経路では到達しない。TrueにするのはROS2 consoleである。詳細は[16](16_Code_Map_and_Call_Graph.md)。

したがって、次を区別する。

- 速度目標：上位指令。正本[03](03_User_Command_and_Reference_Generation.md)
- Gait type/frequency/duty：接触Timing設定
- Foothold：足位置。正本[05](05_Foothold_Reference_and_Terrain_Adaptation.md)
- MPC：指定されたTiming内でGRFと足運びを最適化

## 7. 対応コード

- `helpers/periodic_gait_generator.py`
  - `reset()`
  - `run()`
  - `compute_contact_sequence()`
  - `update_start_and_stop()`（標準では未到達）
- `config.py`: `simulation_params["gait"]`, `gait_params`
- `interfaces/wb_interface.py`: 接触列生成・先頭列抽出

## 8. Cursor確認課題

各Gaitの`phase_offset`から最初の2周期の接地列を生成し、脚順とDocstringが一致するかテストする。
