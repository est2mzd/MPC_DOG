# Log 08: Foothold Reference Generator

対応プロンプト: Base/Hip/速度/周波数から `ref_feet_pos` / `ref_state['ref_foot_*']` / VFA / MPC foothold まで。本文未修正。

標準: `visual_foothold_adaptation='blind'`, `hip_offset=0.1`, `hip_height=0.28`, `ref_z=0.28*1.08=0.3024`, `stance_time=(1/1.35)*0.74≈0.548 s`。

## 確認項目

| # | 項目 | 結果 |
|---|---|---|
| 1 | World/Heading | `R_W2H` はyawのみの2×2。`v_H=R_W2H v_W`。足基準も同じ。最後に `R_W2H.T` でWorldへ戻す |
| 2 | Hip基準 | `ref_feet.leg[0:2] = R_W2H @ (hips.leg[0:2]-base[0:2])` |
| 3 | 目標速度先送り | `delta_ref_H = (stance_time/2) * ref_base_lin_vel_H`。現在速度平均はコメントアウト |
| 4 | 速度誤差補正 | `sqrt(com_height_nominal/g) * (base_vel_mvg - ref_base_lin_vel_H)`。移動平均長20 |
| 5 | CoM高さ | `com_height_nominal=simulation_params['ref_z']`。誤差項にだけ使う。z着地はlift-off z |
| 6 | clip | 先送り `±hip_height*1.5`。誤差 `±0.05` m |
| 7 | 脚別offset | `hip_offset=0.1`。FL/RL の H-y に `+`、FR/RR に `-` |
| 8 | `stance_time` 生成 | `__init__`: `(1/step_freq)*duty_factor` |
| 9 | 周波数変更時 | `optimize_swing==1` のときだけ再計算。標準無効 |
| 10 | Terrain estimator | 先に走るが、Nominal xyには未使用。zはlift-off。地形角は速度回転（Footholdより後） |
| 11 | blind/height/vfa | 標準 `blind`。HeightMapもVFAも作らない |
| 12 | VFA入力 | 非blindかつapex: `ref_feet_pos`, hip, heightmaps, vel, Euler, angvel |
| 13 | VFA出力 | `get_footholds_adapted`: 未初期化なら参照そのまま。`height`はz置換。`vfa`は外部`virall`（未インストールならprint） |
| 14 | Foothold constraint | 標準 `use_foothold_constraints=False`。blindでは `ref_feet_constraints=None` |
| 15 | IK可到達保証 | なし。FRGは幾何heuristic。IKはトルク側で目標関節を出すだけ。差は±3 rad clip |
| 16 | 残りSwing時間 | FRGは見ない。STCが `swing_period=(1-d)/f` で軌道を作るだけ |

## 主要式 ↔ コード

| 数式項 | コード変数 | 生成箇所 | Frame | 単位 |
|---|---|---|---|---|
| \(R_W^H\) | `R_W2H` (2×2) | `compute_footholds_reference` | W→H | — |
| \(v_W\) | `base_xy_lin_vel` | `base_lin_vel[0:2]`（地形回転前） | W | m/s |
| \(v_W^{ref}\) | `ref_base_xy_lin_vel` | 同上、地形回転前 | W | m/s |
| \(v_H\) | `base_lin_vel_H` | `R_W2H @ v_W` | H | m/s |
| \(v_H^{ref}\) | `ref_base_lin_vel_H` | `R_W2H @ v_W^{ref}` | H | m/s |
| \(\bar v_H\) | `base_vel_mvg` | deque平均 | H | m/s |
| \(T_{st}\) | `self.stance_time` | `(1/f)*d` | — | s |
| \(\Delta p_{ref}\) | `vel_offset` | `(T_st/2) v_H^{ref}` + z=0 | H | m |
| \(\Delta p_{err}\) | `error_compensation` | \(\sqrt{h/g}(\bar v-v^{ref})\) | H | m |
| \(h\) | `com_height_nominal` | `ref_z` | — | m |
| \(g\) | `cfg.gravity_constant` | 9.81 | — | m/s² |
| \(p_{hip}^H\) | `ref_feet.leg[0:2]` 初期 | hip−base をHへ | H | m |
| \(p_{off,y}\) | `hip_offset` | ±0.1 | H | m |
| \(p_{td}^W\) | 戻り `ref_feet` | `R_W2H.T` + base | W | m |
| \(p_{td,z}\) | `lift_off_positions[leg][2]` | 前回離地z | W | m |
| `com_pos_offset_w` | `R_B2W @ com_pos_offset_b` | 既定0 | W | m |

## 5種の足位置の区別

| 種類 | 変数 | 生成 | 標準での扱い |
|---|---|---|---|
| Nominal foothold | `ref_feet_pos` | FRG幾何 | 毎500 Hz。これが `ref_state['ref_foot_*']` |
| Terrain-adapted | VFA出力 | 非blindのみ | 標準はNominalのまま |
| MPC decisionの足 | `states[12:24]` と `u[0:12]` | OCP。`use_foothold_optimization=True` なら遊脚が動く | 出力は次TD抽出 |
| Swingへ渡すTD | `nmpc_footholds` | 立脚=現在足、遊脚=次接触切替の予測x、または参照 | STC `touch_down=` |
| 実際の接地位置 | `feet_pos` / `touch_down_positions` | MuJoCo接触とエッジ記録 | Plant。指令ではない |

`last_reference_footholds` はVFA前のコピー。標準ではVFAが無いので `ref_feet` と同じ。

## `05` / `13` 照合

| 資料 | 記載 | 判定 | 修正 |
|---|---|---|---|
| `05` Terrainが先頭、入力はlift-off | 一致 | 正しい | なし |
| `05` 先送り・誤差・clip | 一致 | 正しい | なし |
| `05` Footholdは地形回転前速度 | 一致 | 正しい | なし |
| `05` z=lift-off z | 一致 | 正しい | なし |
| `05` blind既定 | 一致 | 正しい | なし |
| `13` 3集合 | 理論。FRGは \(\mathcal S\cap\mathcal R_{kin}\cap\mathcal R_{time}\) を実装しない | 正しい（理論章） | 「未実装」を本文に明示するとよい |
| `13` 対応順序 | 推奨改善 | 正しい | 現行コードに自動再計画はない |

IK可到達と残りSwing時間は現行FRGに無い。これが `13` の主張と実装の差である。
