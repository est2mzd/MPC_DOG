# 04 — 観測値から `state_current`・`ref_state` まで

日付: 2026-08-25

対象: `external/Quadruped-PyMPC`

関連:

- [01_execution_order_trace_v2.md](01_execution_order_trace_v2.md) — B0、B1-1
- [02_gait_and_contact_sequence_v2.md](02_gait_and_contact_sequence_v2.md)
- [03_foothold_reference_generation_v2.md](03_foothold_reference_generation_v2.md)

対象ファイル:

- `simulation/simulation.py`
- `quadruped_pympc/quadruped_pympc_wrapper.py`
- `quadruped_pympc/interfaces/wb_interface.py`
- `quadruped_pympc/helpers/terrain_estimator.py`
- `quadruped_pympc/helpers/velocity_modulator.py`
- `quadruped_pympc/config.py`

スコープ外:

- VFA内部
- NMPCの運動方程式・評価関数・制約
- GRF・関節トルク計算

---

## 0. この章の結論

`WBInterface.update_state_and_reference()`は、現在の観測値から次の3つを作る。

| 出力 | 内容 | NMPCでの役割 |
|---|---|---|
| `state_current` | 現在のCoM・ベース・足先などの状態 | 初期状態 |
| `ref_state` | 目標速度、姿勢、高さ、着地点 | 各ステージの参照 |
| `contact_sequence` | 現在および未来の接地予定 | ステージごとの接地条件 |

処理順は次のとおりである。

```text
MuJoCoから観測取得
→ 地形推定
→ state_current作成
→ 速度指令の安全停止判定
→ 歩容位相・contact_sequence更新
→ 離地・着地位置更新
→ Raibert着地点参照作成
→ 任意でVFA
→ 目標速度を地形傾斜に合わせて変更
→ 目標高さ・姿勢を作成
→ ref_state作成
→ NMPCへ渡す
```

### 最初に押さえるべき事実

1. `state_current["position"]`はCoM位置だが、`linear_velocity`はベース速度である。
2. `linear_velocity`はworld frame、`angular_velocity`はbase frameである。
3. `state_current["joint_*"]`には、実際の関節角ではなく`qvel`のインデックス配列が渡されている。
4. `TerrainEstimator`へ渡す足先位置は実測`feet_pos`ではなく、`FootholdReferenceGenerator.lift_off_positions`である。
5. `current_contact`は`TerrainEstimator`の引数に存在するが、実行されるコードでは使用されない。
6. `terrain_roll`は設定により常に0、`terrain_pitch`だけが参照へ反映される。
7. Raibert着地点計算には、VelocityModulator後・地形回転前の目標速度が使われる。
8. 地形回転後の目標速度は、`ref_state["ref_linear_velocity"]`に格納される。

---

## 1. MuJoCoから取得する観測値

`simulation/simulation.py`の制御ループでは、毎ステップ次の値を取得する。

| 変数 | 取得コード | shape | 座標系 | 単位 |
|---|---|---:|---|---|
| `feet_pos` | `env.feet_pos(frame="world")` | 各脚`(3,)` | world | m |
| `hip_pos` | `env.hip_positions(frame="world")` | 各脚`(3,)` | world | m |
| `base_lin_vel` | `env.base_lin_vel(frame="world")` | `(3,)` | world | m/s |
| `base_ang_vel` | `env.base_ang_vel(frame="base")` | `(3,)` | base | rad/s |
| `base_ori_euler_xyz` | `env.base_ori_euler_xyz` | `(3,)` | world基準 | rad |
| `base_pos` | `env.base_pos` | `(3,)` | world | m |
| `com_pos` | `env.com` | `(3,)` | world | m |
| `ref_base_lin_vel` | `env.target_base_vel()` | `(3,)` | world | m/s |
| `ref_base_ang_vel` | `env.target_base_vel()` | 外部実装依存 | 外部実装依存 | rad/s |

これらは`QuadrupedPyMPC_Wrapper.compute_actions()`を経由し、同名の引数として`WBInterface.update_state_and_reference()`へ渡される。

### 1.1 `joints_pos`の実体

`simulation.py`では次の代入が行われる。

```python
legs_qvel_idx = env.legs_qvel_idx
legs_qpos_idx = env.legs_qpos_idx

joints_pos = LegsAttr(
    FL=legs_qvel_idx.FL,
    FR=legs_qvel_idx.FR,
    RL=legs_qvel_idx.RL,
    RR=legs_qvel_idx.RR,
)
```

#### コードから確認できた事実

- 変数名は`joints_pos`だが、値は`qvel`配列内のインデックスである。
- 実際の関節角`qpos[legs_qpos_idx.*]`は代入されていない。
- この値はそのまま`state_current["joint_FL/FR/RL/RR"]`へ入る。
- nominalコントローラは、これらの`joint_*`キーを参照しない。
- 調査した範囲では、`joint_*`を読むのはkinodynamicコントローラである。

これがkinodynamic構成で意図された処理か不具合かは、本章では判断しない。

---

## 2. 実際の処理順序

`update_state_and_reference()`内の重要な順序は次のとおりである。

| 順 | 処理 | 使用する主な値 | 更新される値 |
|---:|---|---|---|
| 1 | 地形推定 | `base_pos`、yaw、前ステップまでの`lift_off_positions` | `terrain_roll/pitch/height` |
| 2 | 現在状態作成 | `com_pos`、ベース状態、実測足先位置 | `state_current` |
| 3 | 速度安全判定 | 目標速度、足先・hip位置 | `ref_base_lin_vel/ang_vel` |
| 4 | 歩容更新 | 位相、時間 | `contact_sequence` |
| 5 | 離地・着地位置更新 | 新しい接地状態、実測足先位置 | `lift_off_positions`など |
| 6 | Raibert着地点計算 | 安全判定後・地形回転前の目標速度 | `ref_feet_pos` |
| 7 | 任意のVFA | 着地点参照・高さマップ | 補正後着地点・制約 |
| 8 | 地形に応じた目標速度変更 | `terrain_roll/pitch` | `ref_base_lin_vel` |
| 9 | 目標高さ・姿勢作成 | `terrain_height/pitch`、base・CoM高さ | `ref_pos`など |
| 10 | 参照辞書作成 | 上記の参照値 | `ref_state` |

### 実行順序から分かる注意点

#### 地形推定に使う離地位置

地形推定は、今ステップの`update_lift_off_positions()`より先に実行される。

したがって、地形推定が読む`self.frg.lift_off_positions`は、今ステップの接地遷移を反映する前の値である。

#### `com_pos_offset_w`のタイミング

`state_current`は`compute_footholds_reference()`より前に作られる。一方、`com_pos_offset_w`は`compute_footholds_reference()`内で更新される。

したがって`state_current["position"]`へ加算される`com_pos_offset_w`は、その呼び出しより前に保持していた値である。

ただし、デフォルトでは元になる`com_pos_offset_b`がゼロなので、通常は数値差が生じない。

---

## 3. `state_current`の構築

```python
state_current = dict(
    position=com_pos + self.frg.com_pos_offset_w,
    linear_velocity=base_lin_vel,
    orientation=base_ori_euler_xyz,
    angular_velocity=base_ang_vel,
    foot_FL=feet_pos.FL,
    foot_FR=feet_pos.FR,
    foot_RL=feet_pos.RL,
    foot_RR=feet_pos.RR,
    joint_FL=joints_pos.FL,
    joint_FR=joints_pos.FR,
    joint_RL=joints_pos.RL,
    joint_RR=joints_pos.RR,
)
```

| キー | 実際に入る値 | shape | 座標系 | 単位 |
|---|---|---:|---|---|
| `position` | `com_pos + com_pos_offset_w` | `(3,)` | world | m |
| `linear_velocity` | `base_lin_vel` | `(3,)` | world | m/s |
| `orientation` | `base_ori_euler_xyz` | `(3,)` | world基準 | rad |
| `angular_velocity` | `base_ang_vel` | `(3,)` | base | rad/s |
| `foot_FL/FR/RL/RR` | 実測`feet_pos.*` | 各脚`(3,)` | world | m |
| `joint_FL/FR/RL/RR` | `legs_qvel_idx.*` | 各脚`(3,)` | — | インデックス |

### 3.1 CoMとbaseの混在

#### 事実

- 位置は`env.com`由来のCoM位置である。
- 並進速度は`env.base_lin_vel(frame="world")`由来のベース速度である。
- 姿勢と角速度もベースの状態である。

#### 未確認

CoM速度とベース速度を同一視できるという仮定を置いているか、その物理的根拠はコードに書かれていない。

### 3.2 座標系の混在

#### 事実

- `linear_velocity`はworld frame。
- `angular_velocity`はbase frame。
- `foot_*`はworld frame。

したがって、`state_current`全体が1つの座標系で統一されているわけではない。各キーは、後段モデルが要求する座標系に合わせて個別に格納されている。

---

## 4. TerrainEstimator

呼び出し:

```python
terrain_roll, terrain_pitch, terrain_height, robot_height = (
    self.terrain_computation.compute_terrain_estimation(
        base_position=base_pos,
        yaw=base_ori_euler_xyz[2],
        feet_pos=self.frg.lift_off_positions,
        current_contact=self.current_contact,
    )
)
```

### 4.1 入力

| 引数 | 実際の値 | 注意点 |
|---|---|---|
| `base_position` | `base_pos` | world frame |
| `yaw` | `base_ori_euler_xyz[2]` | 現在のベースyaw |
| `feet_pos` | `self.frg.lift_off_positions` | 実測`feet_pos`ではない |
| `current_contact` | 前回までの`self.current_contact` | 実行コードでは未使用 |

### 4.2 roll・pitchの計算

足先位置をyawでhorizontal frameへ回し、左右・前後の足先高さ差からrollとpitchの候補値を計算する。

```text
FL−FR、RL−RR → roll候補
FL−RL、FR−RR → pitch候補
```

符号は、それぞれの足先高さ差から後段で調整される。

### 4.3 有効化フラグとローパスフィルタ

```python
self.roll_activated = False
self.pitch_activated = True
```

実行される更新式は次のとおりである。

$$
terrain\_pitch_k
=
0.99\,terrain\_pitch_{k-1}
+0.01\,pitch_k
$$

#### コードから確認できた事実

- `roll_activated=False`なので、`terrain_roll`は毎回0へ設定される。
- rollの候補値は計算されるが、出力には反映されない。
- `terrain_pitch`だけが係数0.99/0.01で平滑化される。
- 調査した範囲では、これらのフラグを後から変更する処理は確認できなかった。

### 4.4 `current_contact`が使われない

接地脚だけで高さを平均する処理は、三重引用符で囲まれており実行されない。

実際に実行される式は、

$$
z_{mean}
=
\frac{
z_{FL}+z_{FR}+z_{RL}+z_{RR}
}{4}
$$

$$
terrain\_height_k
=
0.2\,terrain\_height_{k-1}
+0.8\,z_{mean}
$$

である。

#### 事実

- 4脚すべてを、接地状態に関係なく平均する。
- `current_contact`は関数引数にあるが、実行されるコードでは使用しない。
- 平均するZ値は`lift_off_positions`由来である。

### 4.5 `robot_height`

$$
h_{robot,temp}
=
\frac{1}{4}
\sum_i
(p_{base,z}-p_{foot,i,z})
$$

$$
robot\_height_k
=
0.2\,robot\_height_{k-1}
+0.8\,h_{robot,temp}
$$

`robot_height`は関数から返されるが、`update_state_and_reference()`内で後続処理には使用されない。ベース・CoM高さへ代入するコードはコメントアウトされている。

---

## 5. VelocityModulator

`VelocityModulator`は、足先がhipから離れすぎたとき、目標速度をゼロにする。

### 5.1 有効化

```python
self.activated = cfg.simulation_params["velocity_modulator"]
```

デフォルト設定では有効である。

`max_distance`はrobot別の分岐を持つが、すべての分岐で0.2 mに設定される。

### 5.2 距離

各脚について、world frameのXY平面で次を計算する。

$$
d_i
=
\sqrt{
(p_{foot,i,x}-p_{hip,i,x})^2
+(p_{foot,i,y}-p_{hip,i,y})^2
}
$$

### 5.3 早期return

```python
if ref_base_lin_vel[0] < 0.01 and ref_base_lin_vel[1] < 0.01:
    return ref_base_lin_vel, ref_base_ang_vel
```

#### 事実

- 絶対値を取っていない。
- XとYの両方が0.01未満なら、距離による停止判定を実行しない。
- 大きな負の速度も「0.01未満」という条件を満たす。

したがって、後退方向の指令では停止判定を迂回する場合がある。

### 5.4 閾値超過時

いずれか1脚で`d_i > 0.2 m`なら、

```python
ref_base_lin_vel = ref_base_lin_vel * 0.0
ref_base_ang_vel = ref_base_ang_vel * 0.0
```

を実行する。

段階的に速度を調整する処理ではなく、条件成立時に並進・角速度指令をすべてゼロへ変更する。

---

## 6. 着地点参照と目標速度の処理順

VelocityModulatorの後、Raibert着地点参照を計算する。

```python
ref_feet_pos = self.frg.compute_footholds_reference(
    base_xy_lin_vel=base_lin_vel[0:2],
    ref_base_xy_lin_vel=ref_base_lin_vel[0:2],
    ...
)
```

その後で、目標速度を地形傾斜に合わせて変更する。

```python
ref_base_lin_vel = (
    R.from_euler(
        "xyz",
        [terrain_roll, terrain_pitch, 0],
    ).as_matrix()
    @ ref_base_lin_vel
)
```

### コードから確認できる使い分け

| 使用先 | 渡される目標速度 |
|---|---|
| Raibert着地点参照 | VelocityModulator後、地形回転前 |
| `ref_state["ref_linear_velocity"]` | VelocityModulator後、地形回転・追加補正後 |

したがって、同じ制御サイクルでも、着地点参照とNMPCの速度参照は異なる処理段階の`ref_base_lin_vel`を使用する。

---

## 7. 地形推定値による参照変更

### 7.1 目標並進速度

```python
ref_base_lin_vel = (
    R.from_euler(
        "xyz",
        [terrain_roll, terrain_pitch, 0],
    ).as_matrix()
    @ ref_base_lin_vel
)

if terrain_pitch > 0.0:
    ref_base_lin_vel[2] = -ref_base_lin_vel[2]

if np.abs(terrain_pitch) > 0.2:
    ref_base_lin_vel[0] /= 2.0
    ref_base_lin_vel[2] *= 2.0
```

#### 事実

- `terrain_roll=0`なので、デフォルトではpitchだけが回転へ影響する。
- pitchが正の場合、回転後のZ速度の符号を反転する。
- `abs(terrain_pitch) > 0.2 rad`の場合、X速度を半分、Z速度を2倍にする。
- 角速度参照`ref_base_ang_vel`には、この地形補正を適用しない。

#### 解釈

急な傾斜でX方向速度を下げる処理は、安全側へ速度を抑えるヒューリスティックと解釈できる。ただし、設計意図を説明するコメントはない。

### 7.2 目標姿勢

```python
ref_orientation = np.array([
    terrain_roll,
    terrain_pitch,
    0.0,
])
```

デフォルトでは、

```text
目標roll  = 0
目標pitch = terrain_pitch
目標yaw   = 0
```

となる。

### 7.3 目標位置

```python
ref_pos = np.array([0, 0, cfg.hip_height])
ref_pos[2] = cfg.simulation_params["ref_z"] + terrain_height
ref_pos[2] -= (
    base_pos[2]
    - (com_pos[2] + self.frg.com_pos_offset_w[2])
)
```

XY成分は0である。Z成分は次式になる。

$$
p_{ref,z}
=
ref_z
+terrain\_height
-p_{base,z}
+p_{com,z}
+p_{comoffset,z}
$$

#### コードから確認できた事実

- `terrain_height`が目標高さへ直接加算される。
- base高さとCoM高さの差を補正する。
- この時点の`com_pos_offset_w`は、同じサイクル内のRaibert着地点計算で更新された後の値である。

#### コメントから読み取れる設計意図

コードコメントでは、MPCはCoM位置を追従する一方、通常指定したいのはベース高さなので、baseとCoMの高さ差を目標CoM高さへ反映すると説明されている。

---

## 8. `ref_state`の構築

nominal構成では次の辞書を作る。

| キー | 実際に入る値 | shape | 座標系 | 単位 |
|---|---|---:|---|---|
| `ref_foot_FL/FR/RL/RR` | `ref_feet_pos.*.reshape((1,3))` | `(1,3)` | world | m |
| `ref_foot_constraints_*` | VFA由来。blind時は`None` | — | — | — |
| `ref_linear_velocity` | 地形補正後の`ref_base_lin_vel` | `(3,)` | コード上の回転後 | m/s |
| `ref_angular_velocity` | `ref_base_ang_vel` | 外部実装依存 | 外部実装依存 | rad/s |
| `ref_orientation` | `[terrain_roll, terrain_pitch, 0]` | `(3,)` | world基準 | rad |
| `ref_position` | `[0,0,p_ref_z]` | `(3,)` | world | m |

### blind構成

```python
ref_feet_constraints = LegsAttr(
    FL=None,
    FR=None,
    RL=None,
    RR=None,
)
```

したがって、デフォルトのblind構成では各脚の着地点制約は`None`である。

`mpc_params['type'] == 'kinodynamic'`の場合、この`ref_state`構築ブロックは実行されない。本章ではnominal構成だけを対象とする。

---

## 9. NMPCへ渡されるまで

```text
WBInterface.update_state_and_reference()
→ state_current
→ ref_state
→ contact_sequence
→ QuadrupedPyMPC_Wrapper.compute_actions()
→ mpc_frequencyの条件判定
→ SRBDControllerInterface.compute_control()
→ Acados_NMPC_Nominal.compute_control()
```

`update_state_and_reference()`は毎シミュレーションステップ呼ばれるが、NMPCへの受け渡しとOCP求解は`mpc_frequency`で間引かれる。

NMPC内部では、`state_current`が初期状態制約、`ref_state`が各ステージの参照、`contact_sequence`が接地条件の生成に使われる。詳細な定式化は次章で扱う。

---

## 10. 同じサイクル内の値の対応

| 値 | どの時点の値か |
|---|---|
| 地形推定に使う`lift_off_positions` | 今サイクルの離地位置更新前 |
| `state_current["position"]`の`com_pos_offset_w` | Raibert計算による今サイクルの更新前 |
| Raibertに使う目標速度 | VelocityModulator後、地形回転前 |
| `ref_state["ref_linear_velocity"]` | VelocityModulator後、地形回転・追加補正後 |
| `ref_position`の`com_pos_offset_w` | Raibert計算による今サイクルの更新後 |
| `contact_sequence` | 今サイクルで位相更新後 |

デフォルトでは`com_pos_offset_b=0`なので、`com_pos_offset_w`の更新前後による数値差は通常生じない。

---

## 11. 事実・解釈・未確認事項

### コードから確認できた事実

- `state_current`はCoM位置、ベース速度・姿勢、実測足先位置を混在させて構成される。
- `state_current["joint_*"]`には関節角ではなくインデックス配列が入る。
- TerrainEstimatorは実測足先位置ではなく`lift_off_positions`を使用する。
- TerrainEstimatorは接地状態を使用せず、4脚すべての高さを平均する。
- `terrain_roll`は無効、`terrain_pitch`は有効である。
- VelocityModulatorは閾値超過時に速度指令をゼロへする。
- 負の速度指令ではVelocityModulatorの早期return条件が成立する場合がある。
- Raibert着地点とNMPC速度参照は、異なる処理段階の目標速度を使用する。

### コードコメントから読み取れる設計意図

- baseとCoMの高さ差を`ref_position.z`へ反映し、CoM位置制御を介してベース高さを目標へ合わせる。

### 理論上の解釈

- pitchに合わせて目標速度・姿勢を変える処理は、斜面追従を目的とするヒューリスティックと解釈できる。

### 未確認事項

- `joints_pos`へインデックスを渡す処理がkinodynamic構成で不具合になるか
- CoM速度の代わりにベース速度を使う設計根拠
- `com_pos_offset_b`を非ゼロへ変更する経路
- `robot_height`の他モジュールでの利用
- 外部`gym_quadruped`における`ref_base_ang_vel`の正確なshape・座標系
- kinodynamic構成での`ref_state`生成経路
