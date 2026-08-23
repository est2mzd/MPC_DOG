# User Command and Reference Generation

## 1. 結論

標準MuJoCo実行では、ユーザーは目的地ではなく胴体目標並進速度とYaw速度を指令する。その値をWorld frameへ変換し、安全補正・地形補正を行って`ref_state`へ格納する。

指令生成と`ref_state`組み立ては本章を正本とする。閉ループ全体の境界表は[02](02_System_Architecture_and_Dataflow.md)。地形角の**推定**は[05](05_Foothold_Reference_and_Terrain_Adaptation.md)、変数一覧は[A](appendices/A_Variable_Dictionary.md)である。

`config.simulation_params['mode']`は定義されるが、`run_simulation()`は読まない。実指令種別は引数`base_vel_command_type`である。

## 2. 指令の2経路

### 2.1 初期化（全モード）

`QuadrupedEnv.reset()`は必ず`_sample_ref_vel()`を呼ぶ。

| 入力 | shape | 単位 | 出力 | shape | 単位 | frame |
|---|---|---|---|---|---|---|
| `base_vel_command_type` | str | なし | `_ref_base_lin_vel_H` | `(3,)` | m/s | H |
| `base_lin_vel_range` | `(2,)` | m/s | `_ref_base_ang_yaw_dot` | scalar | rad/s | z |
| `base_ang_vel_range` | `(2,)` | rad/s | | | | |

| `base_vel_command_type` | `_ref_base_lin_vel_H` | `_ref_base_ang_yaw_dot` |
|---|---|---|
| `'human'` | `0 * [1,0,0]` | `0` |
| `'forward'` | `U(range) * [1,0,0]` | `0`。`'rotate'`を含むときだけ`U(ang_range)` |
| `'random'` | `U(range) * [cos θ, sin θ, 0]` | `'rotate'`を含むときだけ非ゼロ |
| 文字列に`'reset'` | 上記に加え、1000–3000 stepごとに再サンプル | |

Go2の`run_simulation`既定では`ref_base_lin_vel`引数に`hip_height`を掛ける。`hip_height=0.28` m。ワークショップheadlessは`base_vel_command_type='forward'`と`(0.4, 0.6)*hip_height`を渡す。

対応コード: `gym_quadruped/quadruped_env.py` の `QuadrupedEnv.reset()` と `QuadrupedEnv._sample_ref_vel()`。呼び出しは`simulation/simulation.py` の `run_simulation()`。

### 2.2 キー入力（viewer時のみ）

`QuadrupedEnv.render()`がviewerを開くときだけ`key_callback=lambda x: self._key_callback(x)`を登録する。`render=False`（headless）ではこの経路は存在しない。

| キー | 変更 | Go2数値（`hip_height=0.28` m） |
|---|---|---|
| Up / Down | `_ref_base_lin_vel_H[0] ± 0.25 * hip_height` | ±0.07 m/s |
| Left / Right | `_ref_base_ang_yaw_dot ± π/6` | ±0.524 rad/s |
| Ctrl | 並進とYawをゼロ | |
| Space | 一時停止トグル | 指令は変えない |

クリップは並進`±6 * hip_height`（Go2で±1.68 m/s）、Yaw`±2π` rad/s。

対応コード: `gym_quadruped/quadruped_env.py` の `QuadrupedEnv.render()` と `QuadrupedEnv._key_callback()`。

## 3. 座標変換

ユーザーの「前進」はロボットHeading基準である。`target_base_vel(frame='world')`はyawだけを残した`heading_orientation_SO3`でWorldへ変換する。

\[
v_W^{ref}=R_H^W v_H^{ref}
\]

\[
\omega^{ref}=[0,0,\dot\psi^{ref}]^\mathsf T
\]

| 数式 | コード変数 |
|---|---|
| \(v_H^{ref}\) | `_ref_base_lin_vel_H` |
| \(R_H^W\) | `heading_orientation_SO3` |
| \(\dot\psi^{ref}\) | `_ref_base_ang_yaw_dot` |
| \(v_W^{ref}\) | `ref_base_lin_vel` |
| \(\omega^{ref}\) | `ref_base_ang_vel` |

角速度3成分をheading回転する処理はない。World zとheading zは同一である。

| 入力 | shape | 単位 | frame | 出力 | shape | 単位 | frame |
|---|---|---|---|---|---|---|---|
| `_ref_base_lin_vel_H` | `(3,)` | m/s | H | `ref_base_lin_vel` | `(3,)` | m/s | W |
| `_ref_base_ang_yaw_dot` | scalar | rad/s | z | `ref_base_ang_vel` | `(3,)` | rad/s | `[0,0,ψ̇]` |

更新周期: 毎シミュレーションstepで読む（500 Hz）。指令値自体の更新はresetまたはキーイベント。

対応コード: `gym_quadruped/quadruped_env.py` の `QuadrupedEnv.target_base_vel()` と `heading_orientation_SO3`。呼び出しは`simulation/simulation.py`。

## 4. Velocity Modulator

`simulation_params['velocity_modulator']=True`のとき、`VelocityModulator.modulate_velocities()`が指令を補正する。Go2の`max_distance`は0.2 m。

\[
d_i=\|p_{\mathrm{foot},i,xy}-p_{\mathrm{hip},i,xy}\|
\]

| 数式 | コード変数 |
|---|---|
| \(d_i\) | `distance_*_to_hip_xy` |
| 閾値 | `self.max_distance` |

実装事実:

1. `ref_base_lin_vel[0] < 0.01` かつ `[1] < 0.01` なら距離を見ずに入力を返す。
2. それ以外でいずれかの\(d_i>0.2\)なら並進・角速度をゼロにする。

| 入力 | shape | 単位 | frame | 出力 |
|---|---|---|---|---|
| `ref_base_lin_vel` | `(3,)` | m/s | W（この段では未回転） | 同型。値のみ補正 |
| `ref_base_ang_vel` | `(3,)` | rad/s | `[0,0,ψ̇]` | 同型 |
| `feet_pos`, `hip_pos` | 各脚`(3,)` | m | W | 距離計算のみ |

更新周期: 500 Hz。

対応コード: `quadruped_pympc/helpers/velocity_modulator.py` の `VelocityModulator.modulate_velocities()`。呼び出しは`WBInterface.update_state_and_reference()`。

## 5. 地形回転と参照状態

`TerrainEstimator`の出力の使い方だけをここに書く。推定式は[05](05_Foothold_Reference_and_Terrain_Adaptation.md)。

Foothold生成のあと、並進指令だけを地形roll/pitchで回転する。

```text
ref_base_lin_vel ← R_xyz(terrain_roll, terrain_pitch, 0) @ ref_base_lin_vel
if terrain_pitch > 0:  ref_base_lin_vel[2] ← -ref_base_lin_vel[2]
if |terrain_pitch| > 0.2:  x ← x/2,  z ← 2z
```

`ref_base_ang_vel`は回転しない。

`ref_pos`の高さは`ref_z + terrain_height`から、baseとCoMの高さ差を引く。

| 入力 | shape | 単位 | 出力キー | shape | 単位 | frame |
|---|---|---|---|---|---|---|
| 補正後`ref_base_lin_vel`（回転後） | `(3,)` | m/s | `ref_linear_velocity` | `(3,)` | m/s | 地形付き |
| `ref_base_ang_vel` | `(3,)` | rad/s | `ref_angular_velocity` | `(3,)` | rad/s | `[0,0,ψ̇]` |
| `terrain_roll`, `terrain_pitch` | scalar | rad | `ref_orientation` | `(3,)` | rad | `[roll,pitch,0]` |
| `ref_z`, `terrain_height`, `base_pos`, `com_pos` | scalar / `(3,)` | m | `ref_position` | `(3,)` | m | xyは`[0,0,*]`、zは補正高さ |
| `ref_feet_pos.*` | 各`(3,)` | m W | `ref_foot_FL` 等 | 各`(1,3)` | m | W |
| `ref_feet_constraints.*`（blindでは`None`） | — | — | `ref_foot_constraints_FL` 等 | — | — | W |

現行の`ref_state`キーは次である。旧名`ref_foot_FL_constraints`は誤りであり、理由は[E](appendices/E_Corrections_and_Clarifications.md) §12。

```python
ref_state = {
    "ref_foot_FL": ref_feet_pos.FL.reshape((1, 3)),
    "ref_foot_FR": ...,
    "ref_foot_RL": ...,
    "ref_foot_RR": ...,
    "ref_foot_constraints_FL": ref_feet_constraints.FL,
    "ref_foot_constraints_FR": ...,
    "ref_foot_constraints_RL": ...,
    "ref_foot_constraints_RR": ...,
    "ref_linear_velocity": ref_base_lin_vel,  # 地形回転後
    "ref_angular_velocity": ref_base_ang_vel,
    "ref_orientation": np.array([terrain_roll, terrain_pitch, 0.0]),
    "ref_position": ref_pos,
}
```

`mpc_params['type']=='kinodynamic'`のときはこの辞書を組まない。標準`nominal`では組む。

更新周期: 500 Hz。

対応コード: `quadruped_pympc/interfaces/wb_interface.py` の `WBInterface.update_state_and_reference()`。

## 6. 速度指令の分岐

同じ関数内でも、FootholdとMPCは**同じ時刻の同じ配列を共有しない**。

| 使用先 | 使う速度 | 目的 |
|---|---|---|
| `FootholdReferenceGenerator.compute_footholds_reference` | 地形回転**前**の`ref_base_lin_vel[0:2]` | 着地点 |
| `ref_state['ref_linear_velocity']` | 地形回転**後** | MPC胴体速度追従 |
| `pgg.update_start_and_stop` | 回転前。ただし`start_and_stop_activated=False`なら未到達 | Full stance切替 |

速度指令からTrot等のGait種類を自動選択する標準ロジックはない。

## 7. 上位Planner追加時

次は **推奨改善** であり、現行標準経路には無い。

Plannerが出すべき最低限の境界は次である。

```text
Planner output: desired linear/angular body velocity
Controller input: ref_base_lin_vel / ref_base_ang_vel
```

不整地でTimingまで扱う場合は、速度だけでなくFoothold、Touchdown時刻、接触列を整合させる必要がある。

## 8. 対応コード

- `gym_quadruped/quadruped_env.py`: `_sample_ref_vel()`, `_key_callback()`, `target_base_vel()`, `heading_orientation_SO3`
- `simulation/simulation.py`: `env.target_base_vel()`, `compute_actions()`
- `quadruped_pympc/interfaces/wb_interface.py`: `update_state_and_reference()`
- `quadruped_pympc/helpers/velocity_modulator.py`: `modulate_velocities()`

## 9. Cursor確認課題

Heading、Base、World frameが混在する全変数を抽出し、回転行列の向きと関数Docstringが一致するか確認する。
