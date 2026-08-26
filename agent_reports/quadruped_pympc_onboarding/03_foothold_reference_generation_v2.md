# 03 — 接地状態から着地点参照（Raibert則）まで

日付: 2026-08-25

対象: `external/Quadruped-PyMPC`

関連:

- [01_execution_order_trace_v2.md](01_execution_order_trace_v2.md) B1-1節の順番7〜9
- [02_gait_and_contact_sequence_v2.md](02_gait_and_contact_sequence_v2.md) — `previous_contact`と`current_contact`の生成

対象ファイル:

- `quadruped_pympc/interfaces/wb_interface.py`
- `quadruped_pympc/helpers/foothold_reference_generator.py`
- `quadruped_pympc/config.py`

スコープ外:

- `TerrainEstimator`
- `VisualFootholdAdaptation`
- NMPC内部の定式化
- 遊脚軌道生成
- GRF・関節トルク計算

---

## 0. この章の結論

この処理には、次の2つの流れがある。

```text
接地状態の変化
→ 離地位置・着地位置の記録

ベース・hip位置・現在速度・目標速度
→ Raibert則による着地点参照の計算
→ ref_state
→ NMPC
```

両者は完全な直列処理ではない。

### コードから確認できた事実

- `lift_off_positions`は、Raibert着地点参照のZ座標に使用される。
- Raibert着地点参照のXY座標は、hip位置、ベース位置・yaw、目標速度、現在速度の移動平均、`stance_time`から計算される。
- `touch_down_positions`は更新されるが、調査した現行コードでは実行中の制御処理から読み出されていない。
- 目標yaw rateは`compute_footholds_reference()`へ渡されず、yaw rate補正は未実装である。
- Raibertで求める`ref_feet_pos`はNMPCへの参照値であり、NMPCが返す`nmpc_footholds`とは別の変数である。

### 重要な注意

`lift_off_positions`という名前から「離地した瞬間のworld座標を遊脚中ずっと固定する」と考えると、実装と一致しない。遊脚中は、離地時に保存したhorizontal frame上の相対位置を、現在のベース位置・yawでworld frameへ戻して毎ステップ更新する。

---

## 1. 接地状態からイベントを判定する

`WBInterface.update_state_and_reference()`では、次の順番で接地状態を更新する。

```python
self.previous_contact = copy.deepcopy(self.current_contact)
self.current_contact = np.array([
    contact_sequence[0][0],
    contact_sequence[1][0],
    contact_sequence[2][0],
    contact_sequence[3][0],
])
```

したがって、

- `previous_contact[i]`: 1シミュレーションステップ前の脚 `i` の状態
- `current_contact[i]`: 現在の脚 `i` の状態
- 1: 接地
- 0: 遊脚

である。

| 前回 | 現在 | 状態遷移 | 更新される値 |
|---:|---:|---|---|
| 1 | 0 | 離地（stance → swing） | `lift_off_positions`を実測足先位置から記録 |
| 0 | 0 | 遊脚継続 | 保存したhorizontal frame位置から`lift_off_positions`を再計算 |
| 0 | 1 | 着地（swing → stance） | `touch_down_positions`を実測足先位置から記録 |
| 1 | 1 | 接地継続 | 保存したhorizontal frame位置から`touch_down_positions`を再計算 |

`gait_type == FULL_STANCE`の場合は遷移を判定せず、両方の位置を毎回現在の実測足先位置で上書きする。

---

## 2. 離地位置の更新

対象関数:

```text
FootholdReferenceGenerator.update_lift_off_positions()
```

### 2.1 離地した瞬間

条件:

```python
previous_contact[leg_id] == 1 and current_contact[leg_id] == 0
```

実行内容:

```python
self.lift_off_positions[leg_name] = feet_pos[leg_name]
self.lift_off_positions_h[leg_name] = (
    R_W2H @ (self.lift_off_positions[leg_name] - base_position)
)
```

#### 事実

- `feet_pos[leg_name]`は実測足先位置で、world frameの3次元位置である。
- `lift_off_positions`には、その実測値を保存する。
- `lift_off_positions_h`には、ベース位置を原点としたhorizontal frame上の相対位置を保存する。

式で書くと、

$$
p_{LO,i}^{H}
=
R_{W\rightarrow H}(\psi)
\left(p_{foot,i}^{W}-p_{base}^{W}\right)
$$

である。

### 2.2 遊脚中

条件:

```python
previous_contact[leg_id] == 0 and current_contact[leg_id] == 0
```

実行内容:

```python
self.lift_off_positions[leg_name] = (
    R_W2H.T @ self.lift_off_positions_h[leg_name]
    + base_position
)
```

式で書くと、

$$
p_{LO,i}^{W}(k)
=
R_{W\rightarrow H}(\psi_k)^T p_{LO,i}^{H}
+p_{base}^{W}(k)
$$

である。

#### 事実

- 遊脚中は、実測足先位置を再度保存していない。
- 離地時に保存した`lift_off_positions_h`を保持する。
- 現在のベース位置とyawを使い、world frameの`lift_off_positions`を毎ステップ再計算する。
- 3×3の`R_W2H`はyawのみを含み、Z軸は回転させない。

#### 解釈

`lift_off_positions`は「world frame上で固定された過去の接地点」ではなく、「離地時のベース相対位置を現在のベース姿勢へ戻した値」と理解する方が実装に近い。

この設計を採用した理由は、コード内に明記されていない。

---

## 3. 着地位置の更新

対象関数:

```text
FootholdReferenceGenerator.update_touch_down_positions()
```

処理は離地位置と同じ構造である。

### 着地した瞬間

```text
previous_contact = 0
current_contact  = 1
```

現在の実測足先位置を`touch_down_positions`へ保存し、ベース相対のhorizontal frame位置を`touch_down_positions_h`へ保存する。

### 接地継続中

```text
previous_contact = 1
current_contact  = 1
```

`touch_down_positions_h`を現在のベース位置・yawでworld frameへ戻し、`touch_down_positions`を更新する。

### 現行コードでの利用状況

#### コードから確認できた事実

- `touch_down_positions`と`touch_down_positions_h`は、このクラス内で更新される。
- `WBInterface.compute_stance_and_swing_torque()`には、`touch_down_positions`を使う行があるがコメントアウトされている。
- `EarlyStanceDetector.update_detection()`の`touch_down`引数には、`touch_down_positions`ではなく`nmpc_footholds`が渡される。
- 調査した範囲では、現在の実行経路から`touch_down_positions`を読み出す有効なコードは確認できなかった。

したがって現行経路では、`touch_down_positions`は更新されるが、その後の制御計算では使用されていない。

---

## 4. Raibert着地点参照の入力

対象関数:

```python
compute_footholds_reference(
    base_position,
    base_ori_euler_xyz,
    base_xy_lin_vel,
    ref_base_xy_lin_vel,
    hips_position,
    com_height_nominal,
)
```

| 引数 | shape | 座標系 | 単位 | 実際の用途 |
|---|---:|---|---|---|
| `base_position` | `(3,)` | world | m | hip位置をベース相対へ変換する原点 |
| `base_ori_euler_xyz` | `(3,)` | world基準 | rad | yawをhorizontal frame変換に使用 |
| `base_xy_lin_vel` | `(2,)` | world | m/s | 移動平均後、速度誤差項へ使用 |
| `ref_base_xy_lin_vel` | `(2,)` | world | m/s | 目標速度項と速度誤差項へ使用 |
| `hips_position` | 各脚`(3,)` | world | m | 各脚のXY基準位置 |
| `com_height_nominal` | scalar | — | m | 速度誤差項のゲインに使用 |

`base_xy_lin_vel`と`ref_base_xy_lin_vel`は、関数内のassertでshape `(2,)`が要求される。

呼び出し側では、

```python
base_position=base_pos
base_xy_lin_vel=base_lin_vel[0:2]
ref_base_xy_lin_vel=ref_base_lin_vel[0:2]
com_height_nominal=cfg.simulation_params["ref_z"]
```

が渡される。

#### 事実

`com_height_nominal`へ渡されるのは現在の実測高さではなく、設定値`simulation_params['ref_z']`である。

---

## 5. Raibert着地点参照の計算

まず記号を定義する。

| 記号 | コード | 意味 |
|---|---|---|
| $R$ | `R_W2H` | worldからhorizontal frameへのyaw回転 |
| $v^W$ | `base_xy_lin_vel` | 現在のベースXY速度 |
| $v_{ref}^W$ | `ref_base_xy_lin_vel` | 目標ベースXY速度 |
| $\bar v^H$ | `base_vel_mvg` | horizontal frameでの現在速度の移動平均 |
| $T_s$ | `stance_time` | 立脚時間 |
| $h_{ref}$ | `com_height_nominal` | 設定された基準高さ |
| $o_i$ | `hip_offset`による脚別オフセット | 脚をhip直下から左右へずらす量 |

### 5.1 worldからhorizontal frameへ変換

$$
R=
\begin{bmatrix}
\cos\psi & \sin\psi\\
-\sin\psi & \cos\psi
\end{bmatrix}
$$

$$
v^H=Rv^W
$$

$$
v_{ref}^H=Rv_{ref}^W
$$

`psi`は現在のベースyaw角である。rollとpitchは、このXY速度変換には使用しない。

### 5.2 現在速度の移動平均

```python
self.base_vel_hist.append(base_lin_vel_H)
base_vel_mvg = np.mean(list(self.base_vel_hist), axis=0)
```

`base_vel_hist`は最大20サンプルのdequeである。

#### 事実

- 起動直後は、存在するサンプルだけで平均する。
- 20サンプル蓄積後は、直近20サンプルの移動平均になる。

### 5.3 目標速度による着地点オフセット

$$
\Delta_{ref}^{H}
=
\operatorname{clip}
\left(
\frac{T_s}{2}v_{ref}^{H},
-1.5h_{hip},
+1.5h_{hip}
\right)
$$

コード:

```python
delta_ref_H = (self.stance_time / 2.0) * ref_base_lin_vel_H
delta_ref_H = np.clip(
    delta_ref_H,
    -self.hip_height * 1.5,
    self.hip_height * 1.5,
)
```

目標速度が大きいほど、参照着地点をその速度方向へ遠く配置する。ただしhorizontal frameの各軸で`1.5 * hip_height`に制限される。

### 5.4 現在速度と目標速度の差による補正

$$
e_v^{H}
=
\operatorname{clip}
\left(
\sqrt{\frac{h_{ref}}{g}}
(\bar v^{H}-v_{ref}^{H}),
-0.05,
+0.05
\right)
$$

コード:

```python
error_compensation = (
    np.sqrt(com_height_nominal / self.gravity_constant)
    * (base_vel_mvg - ref_base_lin_vel_H)
)
```

#### コードから確認できた事実

- 差分の順番は「現在速度の移動平均 − 目標速度」である。
- 補正量はhorizontal frameの各軸で±0.05 mに制限される。
- 現在速度が目標速度より正方向に大きい場合、その軸の補正値は正になる。

#### 解釈上の注意

コードだけから直接言えるのは「速度誤差と同じ符号方向へ着地点参照をずらす」ことまでである。

これによって実際に加速・減速のどちらへ作用するかは、接地後のGRF、NMPCの解、ロボットの状態にも依存するため、この関数だけを根拠に断定しない。

### 5.5 hip位置と左右オフセット

各脚のhorizontal frame基準位置は、

$$
p_{hiprel,i}^{H}
=
R(p_{hip,i}^{W,xy}-p_{base}^{W,xy})
$$

である。

その後、Y方向へ脚別オフセットを加える。

$$
o_i=
\begin{cases}
(0,+0.1)^T & i\in\{FL,RL\}\\
(0,-0.1)^T & i\in\{FR,RR\}
\end{cases}
$$

`hip_offset = 0.1`はクラス内にハードコードされている。

### 5.6 XY参照を合成してworld frameへ戻す

horizontal frame上のXY参照は、

$$
p_{ref,i}^{H}
=
p_{hiprel,i}^{H}
+o_i
+\Delta_{ref}^{H}
+e_v^{H}
$$

である。

world frameへ戻すと、

$$
p_{ref,i}^{W,xy}
=
R^T p_{ref,i}^{H}
+p_{base}^{W,xy}
+p_{comoffset}^{W,xy}
$$

となる。

`p_comoffset`は、

```python
self.com_pos_offset_w = R_B2W @ self.com_pos_offset_b
```

でbody frameからworld frameへ変換される。

#### 事実

- `R_B2W`はroll、pitch、yawをすべて使用する。
- `com_pos_offset_b`はコンストラクタでゼロ初期化される。
- 調査した範囲では、これを非ゼロに設定する経路は確認できなかった。

### 5.7 Z座標

各脚のZ参照は、次の代入だけで決まる。

```python
ref_feet[leg_id][2] = self.lift_off_positions[leg_id][2]
```

すなわち、

$$
p_{ref,i}^{W,z}=p_{LO,i}^{W,z}
$$

である。

#### 事実

- Raibertの速度項はXYにだけ使用される。
- Z座標には`lift_off_positions`のZ成分をそのまま使用する。
- `touch_down_positions`は、この着地点参照計算では使用しない。

---

## 6. 計算式全体

XY参照を1本にまとめると、

$$
\boxed{
p_{ref,i}^{W,xy}
=
R^T
\left[
R(p_{hip,i}^{W,xy}-p_{base}^{W,xy})
+o_i
+\Delta_{ref}^{H}
+e_v^{H}
\right]
+p_{base}^{W,xy}
+p_{comoffset}^{W,xy}
}
$$

$$
\boxed{
p_{ref,i}^{W,z}=p_{LO,i}^{W,z}
}
$$

ただし、

$$
\Delta_{ref}^{H}
=
\operatorname{clip}
\left(
\frac{T_s}{2}Rv_{ref}^{W},
\pm1.5h_{hip}
\right)
$$

$$
e_v^{H}
=
\operatorname{clip}
\left(
\sqrt{\frac{h_{ref}}{g}}
(\bar v^{H}-Rv_{ref}^{W}),
\pm0.05
\right)
$$

である。

### この式から確実に言えること

- hip位置が脚ごとの基準点になる。
- 目標速度は`stance_time / 2`を掛けて着地点へ反映される。
- 現在速度は、最大20サンプルの移動平均を通して反映される。
- 計算はyawで向きをそろえたhorizontal frame上で行う。
- yaw rateは式に入っていない。
- ZはRaibert計算ではなく離地位置から与えられる。

---

## 7. yaw角とyaw rateの違い

| 入力 | 実装状況 | 使用場所 |
|---|---|---|
| 現在のyaw角 | 使用される | `R_W2H`を作り、XY位置・速度をhorizontal frameへ変換 |
| 目標yaw rate | 使用されない | 関数の引数に存在しない |

`compute_footholds_reference()`のTODOコメントには、目標yaw rateによる補正を追加すべきと書かれている。

したがって、yaw rate補正は将来案としてコメントに存在するが、現行コードには実装されていない。

---

## 8. `ref_state`からNMPCまで

計算された`ref_feet_pos`は、各脚`(3,)`のworld座標である。

`WBInterface.update_state_and_reference()`で次のように変換される。

```python
ref_foot_FL = ref_feet_pos.FL.reshape((1, 3))
ref_foot_FR = ref_feet_pos.FR.reshape((1, 3))
ref_foot_RL = ref_feet_pos.RL.reshape((1, 3))
ref_foot_RR = ref_feet_pos.RR.reshape((1, 3))
```

データフローは次のとおりである。

```text
FootholdReferenceGenerator.compute_footholds_reference()
→ ref_feet_pos
→ ref_state["ref_foot_FL/FR/RL/RR"]
→ QuadrupedPyMPC_Wrapper.compute_actions()
→ SRBDControllerInterface.compute_control()
→ Acados_NMPC_Nominal.compute_control()
→ acadosのyref
```

VFAが有効な場合は、`ref_state`へ入る前に`ref_feet_pos`が補正される可能性がある。本章では扱わない。

---

## 9. Raibert参照とNMPC出力の違い

| 項目 | `ref_feet_pos` | `nmpc_footholds` |
|---|---|---|
| 生成元 | `compute_footholds_reference()` | `Acados_NMPC_Nominal.compute_control()` |
| 役割 | OCPの着地点参照 | OCP求解後に取り出す着地点 |
| nominal OCP内 | `yref`の一部 | 足先位置状態の求解結果 |
| 更新タイミング | 毎シミュレーションステップ生成 | `mpc_frequency`で間引かれたOCP求解時 |

`use_foothold_optimization=True`の場合、Raibert着地点はNMPCへ参照として渡され、NMPCが着地点状態を最適化する。

`use_foothold_optimization=False`の場合について、`config.py`には「参照として与えた着地点だけを使う」というコメントがある。

ただし、本章ではその分岐をNMPC内部まで追跡していないため、`nmpc_footholds`と`ref_feet_pos`が数値的に完全一致するかは未確認である。

---

## 10. 主要変数

| 変数 | shape | 座標系 | 単位 | 用途 |
|---|---:|---|---|---|
| `previous_contact` | `(4,)` | — | 0/1 | 1ステップ前の接地状態 |
| `current_contact` | `(4,)` | — | 0/1 | 現在の接地状態 |
| `feet_pos` | 各脚`(3,)` | world | m | 実測足先位置 |
| `lift_off_positions` | 各脚`(3,)` | world | m | 離地位置。参照Zなどに使用 |
| `lift_off_positions_h` | 各脚`(3,)` | horizontal | m | 離地時のベース相対位置 |
| `touch_down_positions` | 各脚`(3,)` | world | m | 着地位置。現行経路での消費箇所は未確認 |
| `base_position` | `(3,)` | world | m | 座標変換の原点 |
| `base_xy_lin_vel` | `(2,)` | world | m/s | 現在速度 |
| `ref_base_xy_lin_vel` | `(2,)` | world | m/s | 目標速度 |
| `hips_position` | 各脚`(3,)` | world | m | 着地点の基準 |
| `stance_time` | scalar | — | s | 目標速度項の係数 |
| `hip_offset` | scalar | horizontal Y | m | 左右方向の脚別オフセット |
| `ref_feet_pos` | 各脚`(3,)` | world | m | Raibert着地点参照 |
| `ref_state['ref_foot_*']` | `(1,3)` | world | m | NMPCへ渡す参照 |

---

## 11. 事実・解釈・未確認事項

### コードから確認できた事実

- 離地・着地イベントは`previous_contact`と`current_contact`の組み合わせで判定する。
- Raibert参照のXYはhip位置と速度補正から計算する。
- Raibert参照のZは`lift_off_positions.z`から設定する。
- yaw角は座標変換に使うが、yaw rate補正は未実装である。
- 速度誤差項の符号は`base_vel_mvg - ref_base_lin_vel_H`である。
- `touch_down_positions`を現在の制御処理から読み出す有効な箇所は、調査範囲では確認できない。

### 理論上の解釈

- `stance_time / 2`の目標速度項は、立脚期間中の移動を見越して着地点をずらすRaibert型の項と解釈できる。
- `sqrt(h/g)`は倒立振子の時間スケールに対応する係数と解釈できる。

これらはコードの計算式に対する理論的な対応づけであり、コード内で設計意図として説明されているわけではない。

### 未確認事項

- `lift_off_positions`を遊脚中にベース追従させる設計理由
- `com_pos_offset_b`を非ゼロへ変更する経路
- `use_foothold_optimization=False`時のNMPC内部の正確な処理
- VFA有効時の着地点参照の変更
- yaw rate補正を追加する場合の具体的な設計
