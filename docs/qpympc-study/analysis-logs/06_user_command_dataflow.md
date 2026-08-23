# Log 06: ユーザー指令 → MPC参照

対応プロンプト: キーボードまたは標準指令源から `ref_state` / Foothold速度 / start-stop まで追跡する。本文未修正。

## データフロー表

| 順序 | 入力変数 | 処理 | 出力変数 | shape | 単位 | 変換前 | 変換後 | 次の使用先 |
|---|---|---|---|---|---|---|---|---|
| 1 | `base_vel_command_type` | `_sample_ref_vel`（reset時） | `_ref_base_lin_vel_H` | (3,) | m/s | — | H | `target_base_vel` |
| 2 | 同上 | 同上 | `_ref_base_ang_yaw_dot` | scalar | rad/s | — | z | `target_base_vel` |
| 3 | キー（viewer時） | `_key_callback` | 上記2変数を加減 | 同上 | 同上 | H | H | 同上 |
| 4 | `_ref_base_lin_vel_H` | `target_base_vel(frame='world')` | `ref_base_lin_vel` | (3,) | m/s | H | W | wrapper |
| 5 | `_ref_base_ang_yaw_dot` | 同上（z成分のみ） | `ref_base_ang_vel` | (3,) | rad/s | z | `[0,0,ψ̇]` | wrapper |
| 6 | `ref_base_lin/ang_vel` | `VelocityModulator.modulate_velocities`（`activated=True`） | 同名（スケール後） | 同上 | 同上 | W | W | PGG判定、Foothold、`ref_state` |
| 7 | 地形roll/pitch | `R.from_euler(...) @ ref_base_lin_vel` | `ref_base_lin_vel` | (3,) | m/s | W | 地形傾き回転後W | `ref_state['ref_linear_velocity']` |
| 8 | `ref_z`, terrain_height, CoM-Base差 | `ref_pos` 組立 | `ref_state['ref_position']` | (3,) | m | — | W高さ | yref[0:3] |
| 9 | `terrain_roll/pitch` | 代入 | `ref_state['ref_orientation']` | (3,) | rad | — | `[roll,pitch,0]` | yref[6:9] |
| 10 | `ref_feet_pos` | reshape | `ref_state['ref_foot_*']` | (1,3) | m | W | W | yref足位置 |

`human` 初期値: 並進ゼロ、Yawゼロ。キー: 前後 `±0.25*hip_height`（Go2で±0.07 m/s）、左右 Yaw `±π/6`。クリップ並進 `±6*hip_height`、Yaw `±2π`。

`state_current` は `WBInterface.update_state_and_reference` で `com_pos + com_pos_offset_w`, `base_lin_vel`, Euler, `base_ang_vel`, 現在足位置、関節角から作る。指令ではない。

## 明確化

1. ユーザーが与えるのは目的地ではなく速度（並進Heading + Yaw rate）。
2. Heading→World: `heading_orientation_SO3` で `v_W = R_H^W v_H`。角速度3成分は回転しない。
3. 目標位置: `ref_position` の x,y は0（scaling前も絶対位置追従ではない）。zだけ `ref_z + terrain_height` をBase/CoM差で補正。
4. Velocity Modulator: 脚が伸び切る等の危険姿勢で速度指令を縮小。種類は変えない。
5. 速度指令はGait種類を自動変更しない。
6. 速度ゼロでも `start_and_stop_activated=False` なら位相は止まらない。

## `03` との不一致

| 項目 | 判定 |
|---|---|
| 指令は速度 | 正しい |
| `_sample_ref_vel` が全モード初期化 | 正しい |
| キーはrender時のみ | 正しい |
| `simulation_params['mode']` 未使用 | 正しい |
| 地形回転はFootholdより後 | 正しい（Footholdは回転前速度） |
| 目標xy位置を作る、と読める箇所 | 本文は `ref_position` x,y=0 と書いているので正しい |

残る注意: wrapper観測分岐の `ref_foot_FL_constraints` は実キーと不一致（`03`/Eは訂正済み）。
