# 04 — 観測・速度指令・地形推定・着地点参照から state_current / ref_state まで

日付: 2026-08-25
対象: `external/Quadruped-PyMPC`
関連: [01_execution_order_trace_v2.md](01_execution_order_trace_v2.md) の B0・B1-1節、
[03_foothold_reference_generation_v2.md](03_foothold_reference_generation_v2.md)（`ref_feet_pos`の生成）

対象ファイル:
- `simulation/simulation.py`
- `quadruped_pympc/quadruped_pympc_wrapper.py`
- `quadruped_pympc/interfaces/wb_interface.py`
- `quadruped_pympc/helpers/terrain_estimator.py`
- `quadruped_pympc/helpers/velocity_modulator.py`
- `quadruped_pympc/config.py`

スコープ外: VFA内部（`VisualFootholdAdaptation`）、NMPCの運動方程式・評価関数・制約
（`centroidal_model_nominal.py`/`centroidal_nmpc_nominal.py`の定式化そのもの）、トルク計算。

本ファイルの記述は次の3種類に分けて明記する。

- **事実**: 読んだコードにそのまま書かれている内容
- **解釈**: 事実から導かれる、コード上は明示されていない理論的な意味づけ（推測を含む場合は明記）
- **不明**: コードだけでは確認できない事項

方針: コメントと実装が矛盾する場合は、実行されるコード（コメントアウトされていない部分）を優先する。
本ファイルには、その具体例が複数含まれる（2.1節、5節、7節）。

---

## 0. 結論（データフロー）

```text
env（gym_quadruped, 外部）
  │
  ├─ feet_pos, hip_pos, base_lin_vel, base_ang_vel, base_ori_euler_xyz, base_pos, com_pos
  ├─ ref_base_lin_vel, ref_base_ang_vel  (env.target_base_vel())
  └─ qpos, qvel, legs_qvel_idx, legs_qpos_idx
        │ (simulation.py, 毎ステップ取得)
        ▼
QuadrupedPyMPC_Wrapper.compute_actions(引数として上記一式を受け取る)
        │
        ▼
WBInterface.update_state_and_reference(...)
  ├─ TerrainEstimator.compute_terrain_estimation(...)      → terrain_roll(常に0), terrain_pitch, terrain_height, robot_height
  ├─ state_current = dict(position=..., linear_velocity=..., ...)   ← ここで一度確定
  ├─ VelocityModulator.modulate_velocities(...)（activated時）      → ref_base_lin_vel, ref_base_ang_vel を上書きしうる
  ├─ PeriodicGaitGenerator.run/compute_contact_sequence            → contact_sequence（02参照）
  ├─ FootholdReferenceGenerator.update_*_positions/compute_footholds_reference → ref_feet_pos（03参照）
  ├─ terrain_roll/pitchでref_base_lin_velを回転・補正
  └─ ref_state = dict(ref_foot_*, ref_linear_velocity, ref_angular_velocity, ref_orientation, ref_position)
        │
        ▼
return state_current, ref_state, contact_sequence, step_height, optimize_swing
        │
        ▼ (quadruped_pympc_wrapper.py, mpc_frequencyでゲート)
SRBDControllerInterface.compute_control(state_current, ref_state, contact_sequence, inertia, ...)
        │
        ▼ (NMPC内部。本ファイルのスコープ外)
Acados_NMPC_Nominal.compute_control(...)
```

**事実（先出しの重要な注意）**: `state_current`の`position`はCoM位置、`linear_velocity`はベース
（CoM ではない）速度であり、`angular_velocity`はbase**frame**、`linear_velocity`はworld**frame**
という、キーごとに異なる基準点・座標系が混在した辞書である（詳しくは5節・7節）。

---

## 1. `update_state_and_reference()` へ渡される全入力の生成元

`wb_interface.py::update_state_and_reference()`（L108–123）の引数を、呼び出し元まで遡る。

| 引数 | 直接の呼び出し元（`quadruped_pympc_wrapper.py::compute_actions`の同名引数） | さらに遡った生成元（`simulation.py`） |
|---|---|---|
| `com_pos` | `com_pos` | L180: `copy.deepcopy(env.com)` |
| `base_pos` | `base_pos` | L179: `copy.deepcopy(env.base_pos)` |
| `base_lin_vel` | `base_lin_vel` | L176: `env.base_lin_vel(frame="world")` |
| `base_ori_euler_xyz` | `base_ori_euler_xyz` | L178: `env.base_ori_euler_xyz` |
| `base_ang_vel` | `base_ang_vel` | L177: `env.base_ang_vel(frame="base")` |
| `feet_pos` | `feet_pos` | L173: `env.feet_pos(frame="world")` |
| `hip_pos` | `hip_pos` | L175: `env.hip_positions(frame="world")` |
| `joints_pos` | `joints_pos` | L196: `LegsAttr(FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR, RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR)` |
| `heightmaps` | `heightmaps` | L102–117: `HeightMap(...)`のLegsAttr、または`None`（VFA無効時） |
| `legs_order` | `legs_order` | L93: `["FL", "FR", "RL", "RR"]`（固定リテラル） |
| `simulation_dt` | `simulation_dt` | L219: 引数、元は`qpympc_cfg.simulation_params["dt"]`（L49） |
| `ref_base_lin_vel` / `ref_base_ang_vel` | 同名 | L183: `env.target_base_vel()` |
| `mujoco_contact` | 同名 | L235: `env.mjData.contact` |

**事実（重要な発見: `joints_pos`）**: `simulation.py` L196は次の通りである。

```python
legs_qvel_idx = env.legs_qvel_idx  # leg_name: [idx1, idx2, idx3] ...
legs_qpos_idx = env.legs_qpos_idx  # leg_name: [idx1, idx2, idx3] ...
joints_pos = LegsAttr(FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR, RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR)
```

変数名`joints_pos`（「関節角度」を意味する名前）に反して、実際に代入されているのは
**`legs_qvel_idx`（qvel配列内のインデックス番号の配列、例えば`[7,8,9]`）**であり、
実際の関節角度（`qpos[legs_qpos_idx.FL]`のような値）ではない。この`joints_pos`は
`compute_actions`・`update_state_and_reference`を経て、そのまま
`state_current["joint_FL"]`等（5節）に格納される。

**事実（この値の消費先）**: リポジトリ全体を検索した結果、`state["joint_FL"]`等を
実際に読んでいるのは`controllers/gradient/kinodynamic/kinodynamic_nmpc.py`
（L1408–1411）のみであり、本シリーズが対象とする`nominal`型のNMPC
（`centroidal_nmpc_nominal.py`）は`state["joint_FL"]`等を一切参照しない
（＝5節の状態辞書のうち`joint_FL/FR/RL/RR`キーは、既定の`nominal`経路では
NMPCに実質的な影響を与えない）。

**不明**: `kinodynamic`型を実際に使用した場合、インデックス配列がそのまま関節角度として
渡ることが意図的（例えば別の場所で先に上書きされる、等）なのか、単純な不具合なのかは、
`kinodynamic_nmpc.py`の該当箇所（NMPC内部、本シリーズのスコープ外）を読まないと判断できない。

---

## 2. `TerrainEstimator` の入力・出力と計算内容

`terrain_estimator.py::compute_terrain_estimation()` L14–113。

**入力**（シグネチャ、L14–16）: `base_position (3,) world`, `yaw (float, rad)`,
`feet_pos (dict、FL/FR/RL/RR各(3,) world)`, `current_contact ((4,), {0,1})`。

呼び出し元（`wb_interface.py` L153–158）:
```python
terrain_roll, terrain_pitch, terrain_height, robot_height = self.terrain_computation.compute_terrain_estimation(
    base_position=base_pos,
    yaw=base_ori_euler_xyz[2],
    feet_pos=self.frg.lift_off_positions,   # 実測feet_posではなく、FootholdReferenceGeneratorが保持する値（03参照）
    current_contact=self.current_contact,
)
```

**事実（重要）**: `feet_pos`引数には実測`feet_pos`（`env.feet_pos()`）ではなく、
`self.frg.lift_off_positions`（`03_foothold_reference_generation_v2.md`で解説した、
離地時点の相対位置をベース追従させた推定値）が渡される。実測の足先位置とは異なりうる。

**出力**: `(terrain_roll, terrain_pitch, terrain_height, robot_height)`、すべてスカラー、
`self`に保持され毎回更新される（ローパスフィルタ、下記）。

### 2.1 事実と実装の食い違い（`current_contact`引数）

L89–98は次のようになっている（インデントに注目）:

```python
"""number_foot_in_contact = current_contact[0] + \
                         current_contact[1] + \
                         current_contact[2] + \
                         current_contact[3]
if (number_foot_in_contact != 0):
    z_foot_mean_temp = (z_foot_FL * current_contact[0] + \
                        z_foot_FR * current_contact[1] + \
                        z_foot_RL * current_contact[2] + \
                        z_foot_RR * current_contact[3]) / number_foot_in_contact
    self.terrain_height = self.terrain_height * 0.6 + z_foot_mean_temp * 0.4"""

z_foot_mean_temp = (z_foot_FL + z_foot_FR + z_foot_RL + z_foot_RR) / 4
self.terrain_height = self.terrain_height * 0.2 + (z_foot_mean_temp) * 0.8
```

**事実**: 前半のブロックは`"""..."""`で囲まれた**文字列リテラル**であり、実行されない
（Pythonの構文上、副作用のない式文になる）。実際に実行されるのは最後の2行であり、
`current_contact`による重み付け（接地中の脚だけを平均する）は**行われていない**。
4脚すべてのZ座標の単純平均が、接地状態に関係なく毎回使われる。したがって
`current_contact`という引数は、関数シグネチャには存在するが、実行されるコードパスでは
**一度も使用されない**（デッドパラメータ）。

### 2.2 roll/pitchの計算とアクティベーションフラグ

```python
R_W2H = np.array([[cos(yaw), sin(yaw), 0], [-sin(yaw), cos(yaw), 0], [0, 0, 1]])
front_difference = R_W2H @ (feet_pos["FL"] - base_position) - R_W2H @ (feet_pos["FR"] - base_position)
back_difference  = R_W2H @ (feet_pos["RL"] - base_position) - R_W2H @ (feet_pos["RR"] - base_position)
left_difference  = R_W2H @ (feet_pos["FL"] - base_position) - R_W2H @ (feet_pos["RL"] - base_position)
right_difference = R_W2H @ (feet_pos["FR"] - base_position) - R_W2H @ (feet_pos["RR"] - base_position)

pitch = (atan(|left_difference.z| / |left_difference.x + 0.001|) + atan(|right_difference.z| / |right_difference.x + 0.001|)) * 0.5
roll  = (atan(|front_difference.z| / |front_difference.y + 0.001|) + atan(|back_difference.z| / |back_difference.y + 0.001|)) * 0.5
# 符号調整（L68-71）
```

**事実（ローパスフィルタと`roll_activated`）**:
```python
self.roll_activated = False   # __init__, L11
self.pitch_activated = True   # __init__, L12
...
if self.roll_activated:
    self.terrain_roll = self.terrain_roll * 0.99 + roll * 0.01
else:
    self.terrain_roll = 0.0
if self.pitch_activated:
    self.terrain_pitch = self.terrain_pitch * 0.99 + pitch * 0.01
else:
    self.terrain_pitch = 0.0
```

リポジトリ全体を検索したが、`roll_activated`・`pitch_activated`を`__init__`以外で
変更する箇所（config経由も含め）は見つからなかった。したがって
**`terrain_roll`は常に`0.0`に固定され、実際に計算された`roll`ローカル変数は捨てられる**。
`terrain_pitch`のみが実際に反映され、時定数約100サンプルのローパスフィルタ
（`x = 0.99x + 0.01 new`）で更新される。

### 2.3 `terrain_height` / `robot_height`

```python
z_foot_mean_temp = (z_foot_FL + z_foot_FR + z_foot_RL + z_foot_RR) / 4
self.terrain_height = self.terrain_height * 0.2 + z_foot_mean_temp * 0.8

feet_to_base_mean = mean(base_position.z - feet_pos[leg].z for leg in 4脚)
self.robot_height = self.robot_height * 0.2 + feet_to_base_mean * 0.8
```

**事実**: `terrain_height`は4脚の（`lift_off_positions`由来の）Z座標の平均、`robot_height`は
ベースから各脚までの高さ差の平均。どちらも時定数の短い（0.2/0.8）ローパスフィルタ。

**事実**: `robot_height`は`update_state_and_reference()`の戻り値としては使われず
（L159–160はコメントアウト: `#base_pos[2] = robot_height`）、本ファイルで読んだ範囲では
以降どこにも使われていない（**不明**: 他モジュールでの利用有無は本ファイルの対象外調査）。

---

## 3. terrain roll・pitch・heightが目標速度・姿勢・高さへ与える変更

`wb_interface.py` L262–275。

### 3.1 目標速度（`ref_base_lin_vel`）への影響

```python
ref_base_lin_vel = R.from_euler("xyz", [terrain_roll, terrain_pitch, 0]).as_matrix() @ ref_base_lin_vel
if terrain_pitch > 0.0:
    ref_base_lin_vel[2] = -ref_base_lin_vel[2]
if abs(terrain_pitch) > 0.2:
    ref_base_lin_vel[0] = ref_base_lin_vel[0] / 2.0
    ref_base_lin_vel[2] = ref_base_lin_vel[2] * 2.0
```

**事実**: `terrain_roll`は2.2節より常に`0`なので、実行上この回転は
「pitchのみによるオイラー回転」に等しい（roll成分は回転に寄与しない）。
`terrain_pitch`が正のとき、回転後のZ成分の符号を反転させる。
`|terrain_pitch| > 0.2 rad`（約11.5°）のとき、水平方向(x)の目標速度を半分に、
垂直方向(z)の目標速度を2倍にする。

**解釈（推測）**: これは「急な坂では前進速度を落とし、上下方向の速度指令を強調する」という
ヒューリスティックな安全策と考えられるが、コード中にその意図を説明するコメントはなく、
**推測**である。

### 3.2 目標姿勢（`ref_orientation`）への影響

```python
ref_orientation=np.array([terrain_roll, terrain_pitch, 0.0])
```

**事実**: `ref_state['ref_orientation']`の roll 成分・pitch 成分は、それぞれ
`terrain_roll`・`terrain_pitch`がそのまま代入される（yaw成分は常に`0.0`）。
`terrain_roll`が常に`0`である以上、**`ref_orientation`のroll成分も常に`0`**になる
（2.2節の帰結）。

### 3.3 目標高さ（`ref_position[2]`）への影響

```python
ref_pos = np.array([0, 0, cfg.hip_height])
ref_pos[2] = cfg.simulation_params["ref_z"] + terrain_height
ref_pos[2] -= base_pos[2] - (com_pos[2] + self.frg.com_pos_offset_w[2])
```

**事実**: `ref_position`のZ成分は「`ref_z`（`config.py`の固定基準値）+ `terrain_height`
（2.3節）」からスタートし、さらにbase高さとCoM高さのズレ分だけ補正される（7節）。
`terrain_height`が高いほど（＝足元が高いほど）、目標のCoM高さもそのまま高くシフトする。

---

## 4. `VelocityModulator` が有効な場合の速度補正条件

`velocity_modulator.py`全文（46行）。

**事実（有効化条件）**: `self.activated = cfg.simulation_params['velocity_modulator']`
（`config.py`では既定`True`）。`wb_interface.py` L180: `if self.vm.activated:` のときのみ
`modulate_velocities(...)`が呼ばれる。

**事実（`max_distance`）**:
```python
if cfg.robot == "aliengo": self.max_distance = 0.2
elif cfg.robot == "go1" or cfg.robot == "go2": self.max_distance = 0.2
else: self.max_distance = 0.2
```
すべての分岐で同じ値`0.2`（m）が設定されており、`robot`の値によらず結果は変わらない。

**事実（早期リターン条件、符号に注意）**:
```python
if(ref_base_lin_vel[0] < 0.01 and ref_base_lin_vel[1] < 0.01):
    return ref_base_lin_vel, ref_base_ang_vel  # 変更なしでそのまま返す
```
`np.abs(...)`は使われていない。したがって、目標速度のx・y成分が両方とも`0.01`未満
（**ゼロに近い正の小さい値だけでなく、大きな負の値でも真になる**）の場合、
以降の距離チェック（下記）は一切実行されずそのまま返される。すなわち、
たとえば`ref_base_lin_vel = [-5.0, 0.0]`（大きく後退する指令）は「動いていない」扱いになり、
距離超過による安全停止ロジックを迂回する。

**事実（距離チェックによるゼロ化）**:
```python
distance_leg_to_hip_xy = sqrt((feet_pos[leg].x - hip_pos[leg].x)^2 + (feet_pos[leg].y - hip_pos[leg].y)^2)  # world frame、脚ごと
if いずれかの脚で distance > self.max_distance (=0.2m):
    ref_base_lin_vel = ref_base_lin_vel * 0.0
    ref_base_ang_vel = ref_base_ang_vel * 0.0
return ref_base_lin_vel, ref_base_ang_vel
```
**事実**: 補正は「段階的な減速（modulate）」ではなく、**全か無かの0クリップ**である
（関数名`modulate_velocities`が示唆する「調整」ではなく、閾値超過時の緊急停止に近い）。
距離はworld frameの生座標差で計算されており、回転（yaw）による座標変換は行われていない
（ノルムなので回転不変、結果に影響はない）。

---

## 5. `state_current` の全キーと値の生成元

`wb_interface.py` L163–177:

```python
state_current = dict(
    position=com_pos + self.frg.com_pos_offset_w,
    linear_velocity=base_lin_vel,
    orientation=base_ori_euler_xyz,
    angular_velocity=base_ang_vel,
    foot_FL=feet_pos.FL, foot_FR=feet_pos.FR, foot_RL=feet_pos.RL, foot_RR=feet_pos.RR,
    joint_FL=joints_pos.FL, joint_FR=joints_pos.FR, joint_RL=joints_pos.RL, joint_RR=joints_pos.RR,
)
```

| キー | 値の生成元 | 座標系 | 備考 |
|---|---|---|---|
| `position` | `com_pos`（`env.com`）+ `self.frg.com_pos_offset_w`（既定`[0,0,0]`、3節・03参照） | world | **CoM位置**（base位置ではない） |
| `linear_velocity` | `base_lin_vel`（`env.base_lin_vel(frame="world")`） | world | **base速度**（CoM速度ではない）。`position`とはCoM/baseで基準点が異なる（7節） |
| `orientation` | `base_ori_euler_xyz`（`env.base_ori_euler_xyz`） | world基準roll,pitch,yaw | |
| `angular_velocity` | `base_ang_vel`（`env.base_ang_vel(frame="base")`） | **base frame** | `linear_velocity`はworld frameなので、`state_current`内で座標系が混在している |
| `foot_FL/FR/RL/RR` | `feet_pos.*`（`env.feet_pos(frame="world")`） | world | 実測足先位置 |
| `joint_FL/FR/RL/RR` | `joints_pos.*` = `legs_qvel_idx.*`（1節） | — | **実際は関節角度ではなくqvelインデックス配列**。`nominal`型では未使用（1節） |

**事実（`position`の`com_pos_offset_w`について）**: `self.frg.com_pos_offset_w`は
`foothold_reference_generator.py`の`compute_footholds_reference()`内（4.5節、03参照）で
`R_B2W @ self.com_pos_offset_b`として更新されるが、`com_pos_offset_b`はコード上
`np.zeros((3,))`で初期化されたまま、本ファイルで読んだ範囲では変更箇所が見つからない
（＝既定では`position = com_pos`と実質的に同じ）。

---

## 6. `ref_state` の全キーと値の生成元

`wb_interface.py` L278–294（`mpc_params['type'] != 'kinodynamic'`のとき）:

```python
ref_state = {}
ref_state |= dict(
    ref_foot_FL=ref_feet_pos.FL.reshape((1, 3)),
    ref_foot_FR=ref_feet_pos.FR.reshape((1, 3)),
    ref_foot_RL=ref_feet_pos.RL.reshape((1, 3)),
    ref_foot_RR=ref_feet_pos.RR.reshape((1, 3)),
    ref_foot_constraints_FL=ref_feet_constraints.FL,
    ref_foot_constraints_FR=ref_feet_constraints.FR,
    ref_foot_constraints_RL=ref_feet_constraints.RL,
    ref_foot_constraints_RR=ref_feet_constraints.RR,
    ref_linear_velocity=ref_base_lin_vel,
    ref_angular_velocity=ref_base_ang_vel,
    ref_orientation=np.array([terrain_roll, terrain_pitch, 0.0]),
    ref_position=ref_pos,
)
```

| キー | 値の生成元 | 座標系 | 備考 |
|---|---|---|---|
| `ref_foot_FL/FR/RL/RR` | `ref_feet_pos.*`（`FootholdReferenceGenerator.compute_footholds_reference()`、03参照）を`(1,3)`にreshape | world | VFA有効時はさらに`vfa.get_footholds_adapted()`で上書きされうる（スコープ外） |
| `ref_foot_constraints_FL/FR/RL/RR` | `ref_feet_constraints.*` | — | **既定（VFA無効=`'blind'`）では`LegsAttr(FL=None, FR=None, RL=None, RR=None)`（L256）。つまり値は`None`** |
| `ref_linear_velocity` | 3.1節で terrain_roll/pitch により回転・補正済みの`ref_base_lin_vel` | world（terrain補正込み） | `VelocityModulator`（4節）で先にゼロ化されている場合もある |
| `ref_angular_velocity` | `ref_base_ang_vel`（terrain補正は適用されない。3節参照） | `env.target_base_vel()`が返す座標系のまま | `VelocityModulator`でゼロ化されうる点は`ref_linear_velocity`と同じ |
| `ref_orientation` | `[terrain_roll(常に0), terrain_pitch, 0.0]` | — | 3.2節 |
| `ref_position` | `[0,0,ref_z+terrain_height]`から7節の補正を引いたもの | world（CoM高さ相当、7節） | 3.3節 |

**事実**: `mpc_params['type'] == 'kinodynamic'`の場合、この`ref_state`構築ブロック自体が
`if`文でスキップされる（L278の条件）。`kinodynamic`型でどのような`ref_state`が
組み立てられるかは、本ファイルで読んだ`wb_interface.py`の範囲には存在しない
（**不明**：別ファイルで構築されている可能性があるが未調査、スコープ外）。

---

## 7. CoM位置・base位置・base速度がコード内でどう使い分けられているか

**事実（使われ方の一覧）**:

| 変数 | 使われ方 |
|---|---|
| `com_pos`（`env.com`） | `state_current["position"]`の元（CoM位置がNMPCの状態「position」になる） |
| `base_pos`（`env.base_pos`） | `TerrainEstimator`・`FootholdReferenceGenerator`（03）・`ref_position`補正（下記）で使用。`state_current`のキーには直接入らない |
| `base_lin_vel`（`env.base_lin_vel(frame="world")`） | `state_current["linear_velocity"]`の元（base速度がNMPCの状態「linear_velocity」になる） |

**事実（CoM高さとbase高さの整合、L273–275のコメントそのまま）**:
```python
# Since the MPC close in CoM position, but usually we have desired height for the base,
# we modify the reference to bring the base at the desired height and not the CoM
ref_pos[2] -= base_pos[2] - (com_pos[2] + self.frg.com_pos_offset_w[2])
```

**解釈**: このコメントより、設計意図は次の通りと読み取れる。NMPCの状態量`position`は
CoM位置で閉じている（5節）が、ユーザーが実際に指定したいのは「baseの高さ」である。
両者のズレ（`base_pos.z - com_pos.z`、`com_pos_offset_w`込み）をあらかじめ
`ref_position.z`から差し引いておくことで、NMPCが`ref_position`（CoM基準）を追従した結果として
base高さが目標値に一致するようにしている。

**解釈（`position`はCoM、`linear_velocity`はbase、という非対称性について）**: コード上、
`state_current["position"]`はCoM位置、`state_current["linear_velocity"]`はCoM速度ではなく
base速度である。両者を混在させることが物理的に厳密かどうか（CoM速度とbase速度を
同一視できる前提を置いているのかどうか）は、コード中に明示的な説明がなく、
**推測の域を出ない**。少なくとも位置についてはCoM/baseのズレを7節の補正式で
明示的に扱っているのに対し、速度については同様の補正コードが見当たらない。

**不明**: CoM速度とbase速度の差が無視できるという仮定（あるいはその根拠）が
設計者にあったかどうかは、コードだけでは確認できない。

---

## 8. `state_current`、`ref_state`、`contact_sequence` がNMPCへ渡されるまで

1. `wb_interface.py` L305: `return state_current, ref_state, contact_sequence, self.step_height, optimize_swing`
2. `quadruped_pympc_wrapper.py` L114–131: `compute_actions()`が上記をタプルで受け取る。
3. `quadruped_pympc_wrapper.py` L134: `if step_num % round(1/(mpc_frequency*simulation_dt)) == 0:`
   の条件が真の場合のみ、次に進む（`mpc_frequency`による間引き。詳細は`02`参照）。
4. `quadruped_pympc_wrapper.py` L143–151:
   ```python
   self.srbd_controller_interface.compute_control(
       state_current, ref_state, contact_sequence, inertia,
       self.wb_interface.pgg.phase_signal, self.wb_interface.pgg.step_freq, optimize_swing,
   )
   ```
5. `srbd_controller_interface.py::SRBDControllerInterface.compute_control()` L210:
   `nominal`型では`self.controller.compute_control(state_current, ref_state, contact_sequence, inertia=inertia, ...)`
   としてそのまま`Acados_NMPC_Nominal`へ委譲する（`01`のB1-2節参照）。
6. これ以降、`state_current`・`ref_state`の各キーがacadosの初期状態制約（`lbx`/`ubx`）や
   各ステージの参照（`yref`）に変換される（`01`のB1-2節②-b～②-dで既述、NMPC内部の定式化
   そのものは本ファイル・`01`ともにスコープ外）。

---

## 9. 主要変数まとめ（shape・座標系・単位・生成元・次の使用先）

| 変数 | shape | 座標系 | 単位 | 生成元 | 次の使用先 |
|---|---|---|---|---|---|
| `com_pos` | `(3,)` | world | m | `env.com`（外部） | `state_current["position"]` |
| `base_pos` | `(3,)` | world | m | `env.base_pos`（外部） | `TerrainEstimator`, `FootholdReferenceGenerator`, `ref_position`補正 |
| `base_lin_vel` | `(3,)` | world | m/s | `env.base_lin_vel(frame="world")`（外部） | `state_current["linear_velocity"]` |
| `base_ang_vel` | `(3,)` | **base** | rad/s | `env.base_ang_vel(frame="base")`（外部） | `state_current["angular_velocity"]` |
| `base_ori_euler_xyz` | `(3,)` | world基準roll,pitch,yaw | rad | `env.base_ori_euler_xyz`（外部） | `state_current["orientation"]`, `TerrainEstimator`の`yaw`引数 |
| `feet_pos` | 脚ごと`(3,)` | world | m | `env.feet_pos(frame="world")`（外部） | `state_current["foot_*"]`, `FootholdReferenceGenerator` |
| `joints_pos` | 脚ごと`(3,)`（実体はインデックス配列） | — | — | `env.legs_qvel_idx`（誤って"joint position"として渡される、1節） | `state_current["joint_*"]`（`nominal`では未使用） |
| `ref_base_lin_vel`（入力時） | `(3,)` | world | m/s | `env.target_base_vel()`（外部） | `VelocityModulator`, terrain補正を経て`ref_state["ref_linear_velocity"]` |
| `ref_base_ang_vel` | 形状不明（`(3,)`または`(1,)`相当、コードからは角速度スカラー/ベクトルの区別を確認できず） | — | rad/s | `env.target_base_vel()`（外部） | `VelocityModulator`を経て`ref_state["ref_angular_velocity"]`（terrain補正なし） |
| `terrain_roll` | スカラー | — | rad | `TerrainEstimator`（常に`0.0`、2.2節） | `ref_base_lin_vel`回転、`ref_orientation[0]` |
| `terrain_pitch` | スカラー | — | rad | `TerrainEstimator`（ローパスフィルタ済み） | `ref_base_lin_vel`回転・補正、`ref_orientation[1]` |
| `terrain_height` | スカラー | — | m | `TerrainEstimator` | `ref_position[2]` |
| `state_current["position"]` | `(3,)` | world | m | `com_pos + com_pos_offset_w` | NMPC初期状態（スコープ外） |
| `state_current["linear_velocity"]` | `(3,)` | world | m/s | `base_lin_vel` | NMPC初期状態（スコープ外） |
| `ref_state["ref_position"]` | `(3,)` | world | m | 3.3節の式 | NMPCの`yref`（スコープ外） |
| `ref_state["ref_linear_velocity"]` | `(3,)` | world（terrain回転込み） | m/s | 3.1節の式 | NMPCの`yref`（スコープ外） |
| `ref_state["ref_orientation"]` | `(3,)` | — | rad | `[terrain_roll, terrain_pitch, 0.0]` | NMPCの`yref`（スコープ外） |
| `ref_state["ref_foot_FL/FR/RL/RR"]` | `(1,3)` | world | m | `ref_feet_pos.*`（03参照） | NMPCの`yref`（スコープ外） |
| `contact_sequence` | `(4, horizon)` | — | `{0,1}` | `PeriodicGaitGenerator`（02参照） | NMPCのステージパラメータ（スコープ外） |

---

## 10. 未確認事項（まとめ）

- `state_current["joint_FL"]`等に誤ってインデックス配列が渡っていることが、
  `kinodynamic`型で実害を生むかどうか（NMPC内部未読、スコープ外）。
- `com_pos_offset_b`が非ゼロに設定される経路（`03`でも同様の指摘、見つかっていない）。
- `TerrainEstimator.robot_height`の戻り値が、本ファイルで読んだ範囲以外で使われているか。
- CoM速度とbase速度を同一視している設計上の前提（7節）。
- `ref_base_ang_vel`の正確なshape・意味（`env.target_base_vel()`の戻り値の詳細は
  `gym_quadruped`外部パッケージ側であり、本ファイルの対象ファイルには含まれない）。
- `kinodynamic`型における`ref_state`の構築箇所（`wb_interface.py`の対象範囲外の可能性）。
