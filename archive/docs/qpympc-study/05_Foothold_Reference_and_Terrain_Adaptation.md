# Foothold Reference and Terrain Adaptation

## 1. 結論

Gaitは足を上げる時刻を決めるが、着地点を決めない。Foothold Reference Generatorが速度とHip位置からNominal着地点を作る。標準`blind`では地形適応は走らない。MPCは足位置Costで参照へ寄せるが、\(\mathcal S_{terrain}\) を保証しない。3集合の理論は[13](13_Feasibility_on_Rough_Terrain.md)。

本章が`TerrainEstimator`、lift-off/touch-down、Nominal foothold、VFAの正本である。地形角を使った**速度回転と`ref_state`**は[03](03_User_Command_and_Reference_Generation.md)。可到達集合の議論は[13](13_Feasibility_on_Rough_Terrain.md)。

## 2. TerrainEstimator

`update_state_and_reference()`の先頭で呼ばれる。入力の足は現在の`feet_pos`ではなく`frg.lift_off_positions`である。

| 入力 | shape | 単位 | frame |
|---|---|---|---|
| `base_position` | `(3,)` | m | W |
| `yaw` | scalar | rad | `base_ori_euler_xyz[2]` |
| `feet_pos`（実引数はlift-off） | 各脚`(3,)` | m | W |
| `current_contact` | `(4,)` | 0/1 | なし。高さ平均には現在未使用 |

| 出力 | shape | 単位 | frame | 備考 |
|---|---|---|---|---|
| `terrain_roll` | scalar | rad | H差分から算出。標準0 | `roll_activated=False`のため常に0 |
| `terrain_pitch` | scalar | rad | H差分から算出 | 0.99/0.01フィルタ |
| `terrain_height` | scalar | m | W z | 4足z平均の0.2/0.8フィルタ |
| `robot_height` | scalar | m | — | 同様。`base_pos[2]`代入はコメントアウト |

接触重み付き高さはコメントアウトされている。

対応コード: `quadruped_pympc/helpers/terrain_estimator.py` の `TerrainEstimator.compute_terrain_estimation()`。呼び出しは`WBInterface.update_state_and_reference()`。

## 3. Lift-off / Touch-down更新

接触の立脚→遊脚エッジでlift-offを、遊脚→立脚エッジでtouch-downを、現在の`feet_pos`で記録する。遊脚継続中はheading frameに保持した位置をWorldへ戻す。

| 入力 | shape | 単位 | frame | 出力 | shape | 単位 | frame | 周期 |
|---|---|---|---|---|---|---|---|---|
| `previous_contact`, `current_contact` | `(4,)` | 0/1 | なし | `lift_off_positions`, `touch_down_positions` | 各脚`(3,)` | m | W | 500 Hz |
| `feet_pos`, `base_pos`, Euler | 各`(3,)` | m / rad | W / SciPy xyz | | | | | |

対応コード: `FootholdReferenceGenerator.update_lift_off_positions()` と `update_touch_down_positions()`。

## 4. Nominal Footholdの入力

`compute_footholds_reference`が受け取る目標速度は、地形回転**前**の`ref_base_lin_vel[0:2]`である。回転後速度を使わない。

| 入力 | shape | 単位 | frame |
|---|---|---|---|
| `base_position` | `(3,)` | m | W |
| `base_ori_euler_xyz` | `(3,)` | rad | SciPy xyz |
| `base_xy_lin_vel` | `(2,)` | m/s | W |
| `ref_base_xy_lin_vel` | `(2,)` | m/s | W、地形回転前 |
| `hips_position` | 各脚`(3,)` | m | W |
| `com_height_nominal` | scalar | m | `simulation_params['ref_z']` |
| `stance_time` | scalar | s | `(1/step_freq)*duty_factor` |

## 5. Nominal Foothold

World速度をHeading frameへ変換する。

\[
v_H=R_W^H v_W,\qquad v_H^{ref}=R_W^H v_W^{ref}
\]

目標速度による先送りは、

\[
\Delta p_{ref}=\frac{T_{stance}}{2}v_H^{ref}
\]

速度誤差補償は、

\[
\Delta p_{err}=\sqrt{\frac{h}{g}}(\bar v_H-v_H^{ref})
\]

で、現行コードは補正量をクリップする（`±hip_height*1.5`と誤差`±0.05` m）。

各脚の基準位置は概念的に、

\[
p_{foot,i}^{ref,H}
=
p_{hip,i}^H+\Delta p_{ref}+\Delta p_{err}+p_{offset,i}
\]

であり、最後にWorld frameへ戻す。zは`lift_off_positions[leg][2]`をコピーする。`hip_offset=0.1` mを左右に足す。

| 数式 | コード変数 |
|---|---|
| \(R_W^H\) | `R_W2H`（2×2） |
| \(v_H^{ref}\) | `ref_base_lin_vel_H` |
| \(\bar v_H\) | `base_vel_mvg`（長さ20の移動平均） |
| \(T_{stance}\) | `self.stance_time` |
| \(\Delta p_{ref}\) | `vel_offset` |
| \(\Delta p_{err}\) | `error_compensation` |
| \(p_{offset,i}\) | `hip_offset` の左右符号 |
| \(h\) | `com_height_nominal` |
| \(g\) | `cfg.gravity_constant` |

| 出力 | shape | 単位 | frame |
|---|---|---|---|
| `ref_feet_pos.*` | 各`(3,)` | m | W。zはlift-off z |

これが`ref_state["ref_foot_*"]`へ入り、MPCの足位置参照になる。reshapeは`(1,3)`。[03](03_User_Command_and_Reference_Generation.md)。

対応コード: `quadruped_pympc/helpers/foothold_reference_generator.py` の `FootholdReferenceGenerator.compute_footholds_reference()`。

更新周期: 500 Hz。`optimize_step_freq=True`かつ`optimize_swing==1`のときだけ`stance_time`を再計算する。標準では無効。

## 6. ReferenceとOptimizationの違い

- Generator：安価な幾何Heuristicで基準位置を作る。
- VFA：地形情報で基準位置・許容領域を修正する。標準`blind`では実行されない。
- MPC：足位置Cost（既定 `[300,300,300]`）で参照へ寄せ、遊脚足速度を動かす。`use_foothold_optimization=True`（標準ON）。地形安全集合の保証ではない。箱制約は `use_foothold_constraints=False`。

## 7. 地形適応

`visual_foothold_adaptation`は`blind`、`height`、`vfa`を選べる。デフォルト`blind`には、カメラ・LiDARによる穴認識や経路計画はない。HeightMapも作られない。

非`blind`のときだけ、apex付近でheightmap更新と`VisualFootholdAdaptation.compute_adaptation()`が走る。標準経路の到達不能表は[16](16_Code_Map_and_Call_Graph.md)。

地形適応後の足位置は、次を同時に満たす必要がある。これは理論上の必要集合であり、標準VFAだけでは保証しない。

\[
p_{td}\in
\mathcal S_{terrain}
\cap
\mathcal R_{kinematic}
\cap
\mathcal R_{timing}
\]

詳細は[不整地実現可能性](13_Feasibility_on_Rough_Terrain.md)を参照する。

## 8. 対応コード

- `helpers/foothold_reference_generator.py`: `update_lift_off_positions`, `update_touch_down_positions`, `compute_footholds_reference`
- `helpers/terrain_estimator.py`: `compute_terrain_estimation`
- `helpers/visual_foothold_adaptation.py`: 非blind時
- `interfaces/wb_interface.py`: 呼出順

## 9. Cursor確認課題

1. `stance_time`がGait frequency変更時にどこで更新されるか追跡する。
2. VFA出力の座標系とMPC foothold constraintの座標系を照合する。
3. Foothold clipが脚の厳密なIK可到達域を保証するか確認する。
