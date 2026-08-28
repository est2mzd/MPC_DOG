# Log 09: Centroidal / SRBD（nominal）

対応プロンプト: `centroidal_model_nominal.py` の状態・入力・parameter・運動方程式。本文未修正。

`nx=30`（胴体12+足12+積分6）、`nu=24`、`np=29`。
`use_integrators=False` でも積分状態はベクトルに在る。初期は0。

記号定義に `omega_*_integral` があるが `self.states` には入っていない。

## 状態

| Index | コード変数 | shape | 単位 | Frame | 微分方程式 | 初期値生成元 | 参照値生成元 |
|---|---|---|---|---|---|---|---|
| 0:3 | `com_position_*` | 3 | m | W（`perform_scaling`後は原点相対） | \(\dot p=v\) | `state['position']`（CoM+offset、原点化） | `ref_position`。x,y重み0 |
| 3:6 | `com_velocity_*` | 3 | m/s | W | \(\dot v=(1/m)(\sum c_i F_i+F_{ext})+g\) | `state['linear_velocity']` | `ref_linear_velocity` |
| 6:9 | `roll,pitch,yaw` | 3 | rad | SciPy xyz | \(\dot\Theta=E(r,p)^{-1}\omega\) | `state['orientation']` | `ref_orientation`=`[terrain_roll,pitch,0]`。yaw重み0 |
| 9:12 | `omega_*` | 3 | rad/s | Base（`b_R_w`でWorldトルクをBodyへ） | 下の回転式 | `state['angular_velocity']` | `ref_angular_velocity` |
| 12:15 | `foot_position_fl` | 3 | m | W（原点相対） | \(\dot p_{FL}=(1-c)(1-s)v_{FL}\) | 現在足。遊脚なら参照へteleport | `ref_foot_FL` |
| 15:18 | `foot_position_fr` | 3 | m | W | 同上 | 同上 | `ref_foot_FR` |
| 18:21 | `foot_position_rl` | 3 | m | W | 同上 | 同上 | `ref_foot_RL` |
| 21:24 | `foot_position_rr` | 3 | m | W | 同上 | 同上 | `ref_foot_RR` |
| 24 | `com_position_z_integral` | 1 | m·（離散積算） | — | 実装は `integral[0]+=z` を \(\dot x\) に返す | `integral_errors[0]` | yrefは0 |
| 25:28 | vel積分 x,y,z | 3 | (m/s)· | — | 各速度を加算 | `integral_errors[1:4]` | 0 |
| 28:30 | roll/pitch積分 | 2 | rad· | — | 角度加算 | `integral_errors[4:6]` | 0 |

重力: `g=[0,0,-9.81]`。`config.gravity_constant`。

積分の連続時間解釈: コードは `integral_states = states[24:]; integral_states[i]+=states[...]` を `xdot` として返す。ERKがこれを積分するので、実効は \(\dot e_z=z\) 等。`use_integrators=False` でもダイナミクスは動くが、初期0・参照0・コストは小さい。

## 入力

| Index | コード変数 | 意味 | 単位 | Frame | Gate | Cost weight | 制約 |
|---|---|---|---|---|---|---|---|
| 0:3 | `foot_velocity_fl` | 足先速度 | m/s | W | `(1-c)(1-s)`。`use_foothold_optimization=False` なら入力自体0 | `[1e-4,1e-4,1e-5]` | 入力boundなし |
| 3:6 | `foot_velocity_fr` | 同上 | m/s | W | 同上 | 同上 | なし |
| 6:9 | `foot_velocity_rl` | 同上 | m/s | W | 同上 | 同上 | なし |
| 9:12 | `foot_velocity_rr` | 同上 | m/s | W | 同上 | 同上 | なし |
| 12:15 | `foot_force_fl` | GRF | N | W | 並進/回転に `* stanceFL` | `[0.001]*3` | 摩擦錐20式（全脚常時） |
| 15:18 | `foot_force_fr` | GRF | N | W | `* stanceFR` | 同上 | 同上 |
| 18:21 | `foot_force_rl` | GRF | N | W | `* stanceRL` | 同上 | 同上 |
| 21:24 | `foot_force_rr` | GRF | N | W | `* stanceRR` | 同上 | 同上 |

標準 `use_foothold_optimization=True` なので遊脚足速度は生きる。`stance_proximity` はコード上 `1*0` で常に0。

## Parameter（各段 `p`）

| Index | 意味 | shape | 生成元 | 最適化変数か |
|---|---|---|---|---|
| 0:4 | `stanceFL/FR/RL/RR` | 4 | `contact_sequence[:,j]` | 固定 |
| 4 | `mu_friction` | 1 | `mpc_params['mu']=0.42` | 固定 |
| 5:9 | `stance_proximity_*` | 4 | 計算するが `*0` → 常に0 | 固定 |
| 9:12 | `base_position` | 3 | scaling後の `state['position']`（0,0,0） | 固定 |
| 12 | `base_yaw` | 1 | `state['orientation'][2]` | 固定 |
| 13:16 | 外力 | 3 | wrapperは渡さない → 0 | 固定 |
| 16:19 | 外モーメント | 3 | 0 | 固定 |
| 19:28 | `inertia` 9要素 | 9 | `config.inertia` または再計算CCRBI | 固定 |
| 28 | `mass` | 1 | `config.mass=15.019` | 固定 |

`external_wrenches_compensation=True` でも wrapper は引数を渡さない。既定 `zeros(6,)`。

## 運動方程式 ↔ コード

| # | 対象 | コード | 数式 |
|---|---|---|---|
| 1 | CoM位置 | `linear_com_vel = states[3:6]` | \(\dot p=v\) |
| 2 | CoM速度 | `temp=Σ F_i @ stance_i + F_ext`; `acc=(1/mass)@temp+g` | \(\dot v=(1/m)(\sum c_i F_i+F_{ext})+g\) |
| 3 | 姿勢 | `euler_rates = inv(conj_euler_rates) @ w` | \(\dot\Theta=E^{-1}\omega\) |
| 4 | 角速度 | `b_R_w=Rx@Ry@Rz`; `ang_acc=inv(I)@(b_R_w@temp2 - skew(w)@I@w)` | \(I\dot\omega=R(\sum c_i(p_i-p)\times F_i+\tau_{ext})-\omega\times I\omega\) |
| 5 | 足位置 | `lin_foot = v_foot @ (1-c) @ (1-s)` | \(\dot p_i=(1-c_i)(1-s_i)v_i\) |
| 6 | 積分 | `integral_states[k]+=対応状態` | \(\dot e = \mathrm{state}\) |
| 7 | 外力 | `param[13:16]` | \(F_{ext}\) |
| 8 | 外モーメント | `param[16:19]` を `temp2` に加算 | \(\tau_{ext}\) |
| 9 | 接触Gate | `F @ stance`, `v @ (1-stance)` | \(c_i\) |
| 10 | Stance proximity | `param[5:9]`。標準0 | \(s_i=0\) |

## 出力表（指定形式）

| 状態Index | コード変数 | 数式 | 単位 | Frame | 参照値生成元 |
|---|---|---|---|---|---|
| 0:3 | `com_position` | \(p\) | m | W* | `ref_position` |
| 3:6 | `com_velocity` | \(v\) | m/s | W | `ref_linear_velocity` |
| 6:9 | `roll,pitch,yaw` | \(\Theta\) | rad | Euler | `ref_orientation` |
| 9:12 | `omega` | \(\omega\) | rad/s | B | `ref_angular_velocity` |
| 12:24 | `foot_position_*` | \(p_i\) | m | W* | `ref_foot_*` |
| 24:30 | integrals | \(e\) | 混在 | — | 0 |

\*OCP内部は現在CoMを原点にした相対。

| 入力Index | コード変数 | 物理的意味 | 単位 | Frame | 制約 |
|---|---|---|---|---|---|
| 0:12 | `foot_velocity_*` | 遊脚足速度 | m/s | W | なし（Gateのみ） |
| 12:24 | `foot_force_*` | GRF | N | W | 線形摩擦錐+`Fz∈[0, mg]` 全脚常時 |

## MuJoCoにあってSRBDに無いもの

- 12関節角・速度・加速度
- リンク個別質量（脚の慣性変化）
- 関節トルクと可動域
- 接触コンプライアンス、滑り、複数接触点
- Swing脚の慣性反作用（SRBDは足を質点なしの作用点とする）
- モータ・センサ遅延
- `condim=6` のねじり摩擦

## `06` 照合

| 記載 | 判定 | 差分 |
|---|---|---|
| 基本24 + 積分6 = 30 | 正しい | Index表が本文に無い |
| 入力24 = 足速度12 + GRF12 | 正しい | 同上 |
| pは固定parameter | 正しい | index未記載 |
| 並進Gate | 正しい | なし |
| 回転に `b_R_w` とEuler map | 不完全 | 本文は簡略式のみ |
| \(\dot p_i=(1-c)(1-s)v\) | 正しい | `s≡0` と `use_foothold_optimization` を本文に無い |
| 省略リスト | 正しい | なし |
| omega積分記号 | 誤りになりうる | 定義だけありstatesに未接続 |
