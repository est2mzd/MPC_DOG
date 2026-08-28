# 05 — nominal OCPの状態・入力・パラメータと運動方程式

日付: 2026-08-25
対象: `external/Quadruped-PyMPC`（`mpc_params['type'] = 'nominal'`）
関連: [01_execution_order_trace_v2.md](01_execution_order_trace_v2.md)（B1-2節、OCP求解の全体呼び出し順）、
[04_state_and_reference_assembly_v2.md](04_state_and_reference_assembly_v2.md)（`state_current`/`ref_state`の生成、本ファイルへの入力に相当）

対象ファイル:
- `quadruped_pympc/controllers/gradient/nominal/centroidal_model_nominal.py`
- `quadruped_pympc/controllers/gradient/nominal/centroidal_nmpc_nominal.py`
- `quadruped_pympc/config.py`

スコープ外: 評価関数（`Q`/`R`重み）、摩擦錐制約、着地点制約、安定性制約、SQP/RTIの詳細、
`solve()`以降の解の取り出し処理（それらの一部は`01`のB1-2節で既述）。

本ファイルの記述は次の4種類に分けて明記する。

- **事実**: 読んだコードにそのまま書かれている内容
- **解釈**: 事実から導かれる、コード上は明示されていない理論的な意味づけ（推測を含む場合は明記）
- **コードとdocstring・コメントの不一致**: 実装と文書化の間に見つかった食い違い
- **未確認事項**: コードだけでは確認できない事項

方針: 状態・入力・パラメータの次元は`forward_dynamics()`のdocstringを信用せず、
`cs.vertcat()`の実引数列とインデックスアクセス（`states[a:b]`, `param[i]`）を数えることで決定した。

---

## 0. 結論（先出し）

**事実**: 状態 `x` は30次元、入力 `u` は24次元、パラメータ `p` は29次元である。
`forward_dynamics()`のdocstringはこれら3つすべてについて誤った次元（すべて「29」または「4」）を
記載しており、実装の実次元とは一致しない（4節・9節で詳述）。

```text
x (30)  = [CoM位置(3), CoM速度(3), roll,pitch,yaw(3), 角速度ω(3), 4脚足先位置(12), 積分状態(6)]
u (24)  = [4脚足先速度(12), 4脚GRF(12)]
p (29)  = [接地フラグ(4), 摩擦係数(1), stance_proximity(4), base位置(3), base yaw(1),
           外力・外モーメント(6), 慣性テンソル(9), 質量(1)]
```

---

## 1. OCP、NLP、acadosの関係

**事実**: `centroidal_model_nominal.py`はCasADiのシンボリック変数（`cs.SX`）で状態・入力・
パラメータ・運動方程式を定義し、`export_robot_model()`で`acados_template.AcadosModel`
（`f_impl_expr`, `f_expl_expr`, `x`, `xdot`, `u`, `p`）を構築する（8節で詳述）。

`centroidal_nmpc_nominal.py::Acados_NMPC_Nominal.__init__()`（L49–63）はこの`AcadosModel`を受け取り、
`create_ocp_solver_description()`（L78–274）でコスト・制約・ソルバーオプションを追加した
`acados_template.AcadosOcp`（=OCP、Optimal Control Problemの定式化そのもの）を組み立て、
最後に`AcadosOcpSolver(self.ocp, ...)`でC言語コードを生成・コンパイルした
NLP（Nonlinear Program、離散化されたOCPをSQPやRTIで解く数値最適化問題）ソルバーを得る。

**解釈**: 3つの用語の関係は次のように整理できる。「OCP」は連続時間の最適制御問題の定式化
（状態方程式・コスト・制約）、「NLP」はOCPをホライズン方向に離散化して得られる有限次元の
非線形計画問題、「acados」はそのNLPを実際に高速に解くC言語コード生成・実行フレームワークである。
本ファイルが対象とする2ファイルは、それぞれ「OCPの定式化（モデル）」と
「OCP→acados OCP→NLPソルバーの組み立てと実行」を担当する。

---

## 2. 状態 (x) の全要素・index・次元

`centroidal_model_nominal.py::__init__()` L54–77、`self.states = cs.vertcat(...)`の引数を
出現順に数える。

| index | 変数（コード変数名） | 次元 | 座標系/単位（4節・7節で詳述） |
|---|---|---|---|
| 0 | `com_position_x` | 1 | world, m |
| 1 | `com_position_y` | 1 | world, m |
| 2 | `com_position_z` | 1 | world, m |
| 3 | `com_velocity_x` | 1 | world, m/s |
| 4 | `com_velocity_y` | 1 | world, m/s |
| 5 | `com_velocity_z` | 1 | world, m/s |
| 6 | `roll` | 1 | rad |
| 7 | `pitch` | 1 | rad |
| 8 | `yaw` | 1 | rad |
| 9 | `omega_x` | 1 | rad/s（7節で座標系を議論） |
| 10 | `omega_y` | 1 | 同上 |
| 11 | `omega_z` | 1 | 同上 |
| 12–14 | `foot_position_fl` | 3 | world, m |
| 15–17 | `foot_position_fr` | 3 | world, m |
| 18–20 | `foot_position_rl` | 3 | world, m |
| 21–23 | `foot_position_rr` | 3 | world, m |
| 24 | `com_position_z_integral` | 1 | m·s（6節で式を議論） |
| 25 | `com_velocity_x_integral` | 1 | (m/s)·s |
| 26 | `com_velocity_y_integral` | 1 | (m/s)·s |
| 27 | `com_velocity_z_integral` | 1 | (m/s)·s |
| 28 | `roll_integral` | 1 | rad·s |
| 29 | `pitch_integral` | 1 | rad·s |

**合計: 30次元**（index 0–29）。

**コードとdocstringの不一致**: `forward_dynamics()`のdocstring（L161）は
`states: A numpy array of shape (29,)`と記載しているが、実際は30である。

**コードから確認できた事実（未使用のシンボル）**: `__init__()` L50–52で
`omega_x_integral`, `omega_y_integral`, `omega_z_integral`という3つの`cs.SX.sym`が
定義されているが、L54–77の`self.states`の`vertcat`引数リストには含まれていない。
すなわちこれら3つの角速度積分シンボルは生成されるが、状態ベクトルには
組み込まれず、以降どこにも使われない（デッドシンボル）。

**解釈**: 実際に積分される量は「CoM高さ・CoM速度3成分・roll・pitch」の6個のみであり、
「角速度そのもの（omega_x/y/z）」の積分は行われていない。これは`config.py`の
`integrator_cap`が`[0.5, 0.2, 0.2, 0.0, 0.0, 1.0]`（6要素）であることや、
`centroidal_nmpc_nominal.py`の`self.integral_errors = np.zeros((6,))`（L44）とも
数として整合する。

`self.states_dim`（`centroidal_nmpc_nominal.py` L52: `acados_model.x.size()[0]`）は
`AcadosModel.x`の実サイズから動的に取得されるため、この値自体は正しく`30`になる
（ハードコードされた誤った定数ではない）。

---

## 3. 入力 (u) の全要素・index・次元

`centroidal_model_nominal.py::__init__()` L96–115、`self.inputs = cs.vertcat(...)`。

| index | 変数 | 次元 | 座標系/単位 |
|---|---|---|---|
| 0–2 | `foot_velocity_fl` | 3 | world, m/s |
| 3–5 | `foot_velocity_fr` | 3 | world, m/s |
| 6–8 | `foot_velocity_rl` | 3 | world, m/s |
| 9–11 | `foot_velocity_rr` | 3 | world, m/s |
| 12–14 | `foot_force_fl` | 3 | world, N（6節で座標系を議論） |
| 15–17 | `foot_force_fr` | 3 | world, N |
| 18–20 | `foot_force_rl` | 3 | world, N |
| 21–23 | `foot_force_rr` | 3 | world, N |

**合計: 24次元**（index 0–23）。`self.inputs_dim`（`centroidal_nmpc_nominal.py` L53）も
`acados_model.u.size()[0]`から動的に取得され、実際に`24`になる。

**コードとdocstringの不一致**: `forward_dynamics()`のdocstring（L162）は
`inputs: A numpy array of shape (29,)`としているが、実際は24である
（`states`の29という誤記述をそのまま流用したような値になっている）。

---

## 4. パラメータ (p) の全要素・index・次元

`centroidal_model_nominal.py::__init__()` L141–150（および`export_robot_model()` L317–326で同一の
`vertcat`が再構築される）:

```python
param = cs.vertcat(
    self.stance_param,      # stanceFL, stanceFR, stanceRL, stanceRR
    self.mu_friction,
    self.stance_proximity,  # 4要素
    self.base_position,     # 3要素
    self.base_yaw,
    self.external_wrench,   # 6要素
    self.inertia,           # 9要素
    self.mass,
)
```

| index | 変数 | 次元 | 単位/意味 |
|---|---|---|---|
| 0 | `stanceFL` | 1 | `{0,1}` 接地フラグ |
| 1 | `stanceFR` | 1 | 同上 |
| 2 | `stanceRL` | 1 | 同上 |
| 3 | `stanceRR` | 1 | 同上 |
| 4 | `mu_friction` | 1 | 摩擦係数（`forward_dynamics()`内では**未読**、5節参照） |
| 5 | `stance_proximity_FL` | 1 | `{0,1}`相当（6節） |
| 6 | `stance_proximity_FR` | 1 | 同上 |
| 7 | `stance_proximity_RL` | 1 | 同上 |
| 8 | `stance_proximity_RR` | 1 | 同上 |
| 9–11 | `base_position` | 3 | world, m（`forward_dynamics()`内では**未読**） |
| 12 | `base_yaw` | 1 | rad（`forward_dynamics()`内では**未読**） |
| 13–15 | `external_wrench`（線形成分） | 3 | N（5節） |
| 16–18 | `external_wrench`（角成分） | 3 | N·m（5節） |
| 19–27 | `inertia` | 9 | kg·m²（3×3にreshape、7節） |
| 28 | `mass` | 1 | kg |

**合計: 29次元**（index 0–28）。

**コードとdocstringの不一致**: `forward_dynamics()`のdocstring（L163）は
`param: A numpy array of shape (4,)`としているが、実際は29である
（おそらく`stance_param`単体の次元「4」を誤って全パラメータの次元として
記載している）。

**コードから確認できた事実（`forward_dynamics()`内で未使用のパラメータ）**:
`mu_friction`（index 4）、`base_position`（9–11）、`base_yaw`（12）は
パラメータベクトルの一部として定義・登録されているが、`forward_dynamics()`の本体
（L169–199のローカル変数抽出）では**一切読み出されていない**
（`stanceFL`からindex 0–3、次に飛んで`stance_proximity`がindex 5–8から読まれ、
index 4の`mu_friction`は読み飛ばされている）。

**解釈**: これらは運動方程式そのものには使われず、摩擦錐制約・着地点制約・安定性制約
（`create_friction_cone_constraints()`, `create_foothold_constraints()`,
`create_stability_constraints()`）側で使われるパラメータであると考えられる
（いずれも本ファイルのスコープ外）。

---

## 5. `forward_dynamics()` の運動方程式（コード順・要求された順序で記述）

`centroidal_model_nominal.py::forward_dynamics()` L154–308。返り値`vertcat`の並びが
`states_dot`の並びと対応する。

### 5.1 CoM位置（の微分 = CoM並進速度、そのまま）

```python
linear_com_vel = states[3:6]   # = 現在の状態のCoM速度をそのまま返す
```

$$
\dot p_{com} = v_{com}
$$

コード変数: `states[3:6]`（=`com_velocity_x/y/z`）。座標系: world。単位: m/s。
これは「CoM位置の時間微分がCoM速度である」という定義そのものであり、力学的な計算はない。

### 5.2 CoM並進速度（の微分 = CoM並進加速度）

```python
temp = foot_force_fl @ stanceFL + foot_force_fr @ stanceFR + foot_force_rl @ stanceRL + foot_force_rr @ stanceRR
temp += external_wrench_linear
gravity = np.array([0, 0, -self.gravity_constant])
linear_com_acc = (1 / mass) @ temp + gravity
```

$$
\dot v_{com} = \frac{1}{m}\Big(\sum_{i\in\{FL,FR,RL,RR\}} c_i F_i + F_{ext}\Big) + g,\qquad g=(0,0,-g_0)^T
$$

コード変数: `foot_force_*`＝入力`u`のGRF部分（world, N）、`stanceFL..RR`＝パラメータ`p[0:4]`
（接地フラグ）、`external_wrench_linear`＝`param[13:16]`（world, N）、`mass`＝`param[28]`（kg）、
`self.gravity_constant`＝`config.gravity_constant`（`9.81`）。

**事実**: GRFは接地フラグ`stance_i`（0または1）と単純に乗算されるだけであり、
遊脚中（`stance_i=0`）の脚の力はこの式の中で自動的に0倍される。入力`u`自体が
0に固定される制約が別途あるかどうかは、本ファイルのスコープ外（制約定式化）である。

### 5.3 Euler角（の微分 = Euler角速度）

```python
w = states[9:12]        # omega_x, omega_y, omega_z
conj_euler_rates = eye(3)
conj_euler_rates[1,1] = cos(roll)
conj_euler_rates[2,2] = cos(pitch)*cos(roll)
conj_euler_rates[2,1] = -sin(roll)
conj_euler_rates[0,2] = -sin(pitch)
conj_euler_rates[1,2] = cos(pitch)*sin(roll)
euler_rates_base = inv(conj_euler_rates) @ w
```

行列を明示すると：

$$
E(\phi,\theta)=\begin{bmatrix}1&0&-\sin\theta\\0&\cos\phi&\cos\theta\sin\phi\\0&-\sin\phi&\cos\theta\cos\phi\end{bmatrix},
\qquad
\begin{bmatrix}\dot\phi\\\dot\theta\\\dot\psi\end{bmatrix}=E(\phi,\theta)^{-1}\,\omega
$$

（$\phi$=roll, $\theta$=pitch, $\psi$=yaw、`states[6:9]`）

コード変数: `w`＝`states[9:12]`（角速度状態、7節で座標系を議論）。

**解釈**: 変数名`conj_euler_rates`（"conjugate Euler rates"）は、剛体力学・レッグドロボットの
centroidal MPC文献（例: Di Carlo et al. 2018, MIT Cheetah 3）で使われる、角速度とEuler角速度を
結びつける行列と同種の構成に見える。ただし本ファイルでは、この行列が文献の定義と
完全に一致するかどうかまでは式変形を追って検証していない（**解釈にとどまる**）。

### 5.4 角速度（の微分 = 角加速度）

```python
Rx = [[1,0,0],[0,cos(roll),sin(roll)],[0,-sin(roll),cos(roll)]]
Ry = [[cos(pitch),0,-sin(pitch)],[0,1,0],[sin(pitch),0,cos(pitch)]]
Rz = [[cos(yaw),sin(yaw),0],[-sin(yaw),cos(yaw),0],[0,0,1]]
b_R_w = Rx @ Ry @ Rz          # コードコメント: "Z Y X rotations!"

temp2 = skew(p_fl - p_com)@F_fl@stanceFL + ... (4脚分の和)
temp2 = temp2 + external_wrench_angular
angular_acc_base = inv(inertia) @ (b_R_w @ temp2 - skew(w) @ inertia @ w)
```

$$
\dot\omega = I^{-1}\Big({}^{b}R_{w}\Big(\sum_i c_i\,(p_i-p_{com})\times F_i + M_{ext}\Big) - \omega\times I\omega\Big)
$$

コード変数: `p_fl..p_rr`＝状態`states[12:24]`（4脚足先位置、world, m）、`p_com`＝`states[0:3]`、
`F_fl..F_rr`＝入力のGRF部分（world, N）、`external_wrench_angular`＝`param[16:19]`（N·m）、
`inertia`＝`param[19:28]`を3×3にreshapeしたもの（kg·m²）、`w`＝`states[9:12]`。

**コードから確認できた事実（回転行列の構成とコメントの対応が未検証）**: `Rx`, `Ry`, `Rz`は
標準的な軸周り回転行列を成分ごとに手書きしたものだが、`Rx`の`[1,2]=sin(roll)`,
`[2,1]=-sin(roll)`という符号配置は、教科書的な能動回転行列 $R_x(\theta)$（$[1,2]=-\sin\theta$）
とは符号が反転している。`b_R_w = Rx @ Ry @ Rz`という合成順序に対し、直上のコメントは
`"Z Y X rotations!"`と書かれているが、この合成順序・各行列の符号定義がコメント通りの
「Z→Y→X」回転列として矛盾なく成立しているかどうかは、本ファイルでは数式的に
完全には検証できていない（**未確認事項**）。

**事実（足先位置とCoM位置の関係、モーメント計算）**: モーメント項`temp2`の各脚の寄与
`skew(p_i - p_com) @ F_i @ stance_i`は、足先とCoMの位置差（world frame）と
GRF（world frame、想定）の外積であり、`stance_i`により遊脚中の脚は寄与しない
（5.2節と同様の接地フラグによるゲーティング）。

**7節で詳述する不一致**: `inertia`パラメータの由来（`simulation.py`側で「world frame」と
コメントされている）と、この式が慣性テンソルを扱っている文脈（`skew(w)@inertia@w`という
オイラーの剛体回転方程式の形、通常はbody frameで使う）が整合するかどうかを7節で扱う。

### 5.5 4脚の足先位置（の微分 = 足先速度）

```python
if not config.mpc_params["use_foothold_optimization"]:
    foot_velocity_fl = foot_velocity_fl @ 0.0
    foot_velocity_fr = foot_velocity_fr @ 0.0
    foot_velocity_rl = foot_velocity_rl @ 0.0
    foot_velocity_rr = foot_velocity_rr @ 0.0

linear_foot_vel_FL = foot_velocity_fl @ (1 - stanceFL) @ (1 - stance_proximity_FL)
linear_foot_vel_FR = foot_velocity_fr @ (1 - stanceFR) @ (1 - stance_proximity_FR)
linear_foot_vel_RL = foot_velocity_rl @ (1 - stanceRL) @ (1 - stance_proximity_RL)
linear_foot_vel_RR = foot_velocity_rr @ (1 - stanceRR) @ (1 - stance_proximity_RR)
```

$$
\dot p_i = u_{v,i}\cdot(1-c_i)\cdot(1-\text{prox}_i)\qquad(\text{ただし}\ \texttt{use\_foothold\_optimization=False}\ \text{時は}\ u_{v,i}\equiv0)
$$

**事実（3条件の組み合わせ）**:
- 立脚中（$c_i=1$）: 足先位置状態の微分は常に0（足は動かない）。
- 遊脚中かつstance_proximityでない（$c_i=0,\ \text{prox}_i=0$）: 微分＝入力の足先速度そのもの。
- 遊脚中だが着地間近（$c_i=0,\ \text{prox}_i=1$）: 微分は0（足先位置状態はそこで凍結）。
- `config.mpc_params['use_foothold_optimization'] = False`のとき、上記条件によらず
  入力`foot_velocity_*`自体がまず0に上書きされるため、**足先位置状態の微分は常に0**になる。

**解釈（`03`の未確認事項の解決）**: `03_foothold_reference_generation_v2.md`で
「`use_foothold_optimization=False`のとき`nmpc_footholds`が`ref_feet_pos`と一致するか不明」
としていたが、本ファイルの調査により機構が判明した。`use_foothold_optimization=False`の場合、
足先位置状態はホライズン全体で微分が常に0となり、初期状態制約で与えられた値
（`04`で述べた通り、遊脚中の脚については`reference["ref_foot_*"]`＝Raibert参照で
上書きされる、`01`のB1-2節参照）がホライズン全体で不変のまま保持される。したがって
この設定では、着地点はRaibert参照と（数値誤差を除き）一致すると考えられる。

### 5.6 積分状態（の微分）

```python
integral_states = states[24:]       # states[24:30] の6要素（積分状態そのものの現在値）
integral_states[0] += states[2]      # + com_position_z
integral_states[1] += states[3]      # + com_velocity_x
integral_states[2] += states[4]      # + com_velocity_y
integral_states[3] += states[5]      # + com_velocity_z
integral_states[4] += roll           # states[6]
integral_states[5] += pitch          # states[7]
return cs.vertcat(..., integral_states)
```

**事実（返り値の意味を式で確認）**: この関数が`states_dot`の積分部分として返す値は、
「積分状態の**現在値**（`states[24:30]`）+ 対応する追跡量の現在値」である。すなわち

$$
\dot s_{z} = s_{z} + p_{com,z},\quad
\dot s_{v_x} = s_{v_x} + v_{com,x},\quad \dots,\quad
\dot s_{roll} = s_{roll} + \phi,\quad
\dot s_{pitch} = s_{pitch} + \theta
$$

**解釈**: これは一般的な「積分器」の定義（誤差 $e$ に対し $\dot s = e$、すなわち
$s(t)=\int_0^t e\,d\tau$）とは異なる形である。標準的な積分器であれば
返り値は`states[2]`, `states[3]`, ...（追跡量そのもの）だけのはずだが、コードは
それに現在の積分状態自身を加算した値を返している。これがacados内部で
連続時間ODE（`f_impl_expr = states_dot - f_expl`）としてそのまま離散化・積分される場合、
数式上は $\dot s = s + e$ という素直な線形常微分方程式になり、$e$一定の下での解は
$s(t) = (s_0+e)e^{t}-e$ という指数的な振る舞いになる（**解釈、コード上で積分方法や
離散化がどう吸収するかまでは未検証**）。これは変数名（「積分」）から素朴に期待される
「時間に比例して増える量」とは異なる可能性がある。

**未確認事項**: この定式化が意図的（例えば安定化や特定の離散化スキームを前提とした
設計）か、単純な実装上の誤りかは、コードのコメントからは判断できない。
`use_integrators`が既定`False`（`config.py`の`mpc_params`）であるため、この項が
実際にコストに使われるかどうか自体も、評価関数側（スコープ外）を見ないと確定できない。

---

## 6. 接地フラグ (`stanceFL..RR`) がGRFと足先速度へ与える影響（まとめ）

| 状態 | `stance_i` | `stance_proximity_i` | GRFの寄与（5.2/5.4節） | 足先速度状態の微分（5.5節） |
|---|---|---|---|---|
| 立脚 | 1 | — | $F_i$がそのまま並進加速度・モーメントに寄与 | 常に0（足先固定） |
| 遊脚（proximityでない） | 0 | 0 | $F_i$は0倍され力学に寄与しない | 入力の足先速度がそのまま反映 |
| 遊脚（着地間近） | 0 | 1 | 同上（0倍） | 0（足先位置を凍結） |

**事実**: `stance_i`は`{0,1}`のパラメータ`p`であり、CasADiのシンボリック式の中では
連続値（0や1をとる実数）として乗算に使われる。ソルバーが「0または1」以外の値を
このパラメータに設定した場合の振る舞いは、パラメータの値自体が呼び出し側
（`centroidal_nmpc_nominal.py::compute_control()`、`01`参照）で`contact_sequence`の
`{0,1}`値からそのまま設定されるため、通常は起こらない。

---

## 7. world / body frameの使い分け

**事実（座標系が明示されている変数）**:
- 状態の`foot_position_*`・`com_position_*`は、5.2・5.4節の式（`foot_position - com_position`の
  差やGRFとの外積）から、互いに同じフレーム（world frame、`04`で確認した実測値の由来と整合）
  で扱われていると読み取れる。
- `b_R_w`（5.4節）は、`temp2`（world frameのモーメント）を`inertia`・`skew(w)@inertia@w`と
  同じフレームに揃えるための回転行列として使われており、変数名（`b`=body, `w`=world）から
  「world→body」変換であることが読み取れる。

**コードとコメントの不一致（`inertia`のフレーム）**: `simulation/simulation.py`
（`04`で既読）には次のコメントがある。

```python
inertia = env.get_base_inertia().flatten()  # Reflected inertia of base at qpos, in world frame
```

すなわち、既定設定（`use_inertia_recomputation=True`）では、`forward_dynamics()`に
渡される`inertia`パラメータは**world frameの慣性行列**であるとコメントされている。
一方`forward_dynamics()`内の使い方（`angular_acc_base = inv(inertia) @ (b_R_w @ temp2 - skew(w) @ inertia @ w)`）は、
`w`（角速度、`04`より`base_ang_vel(frame="base")`＝body frame由来と確認済み）と
`b_R_w @ temp2`（world→bodyへ回転させたモーメント、すなわちbody frame）を`inertia`と
直接組み合わせる、剛体力学における標準的な「body frameのオイラー方程式」の形
（$I\dot\omega + \omega\times I\omega = M_{body}$）をしている。

**解釈**: 「`inertia`はworld frame」というコメントと、「この式はbody frameの慣性行列を
前提としている」という式の構造は、額面通りには整合しない。ただし、
`get_base_inertia()`が実際にどう計算されているか（`gym_quadruped`外部パッケージの内部実装）は
本ファイルの対象外であり、コメントの「world frame」という記述が不正確である可能性、
あるいは式側の解釈が誤っている可能性のどちらもありうる。**これは推測であり、
どちらが正しいかはコードだけでは断定できない**。

**未確認事項**: `env.get_base_inertia()`の実装（`gym_quadruped`パッケージ内、対象ファイル外）。

---

## 8. `export_robot_model()` から acados model へ登録されるまで

`centroidal_model_nominal.py::export_robot_model()` L310–339:

```python
self.param = cs.vertcat(...)                      # 4節のparamを再構築
f_expl = self.forward_dynamics(self.states, self.inputs, self.param)
f_impl = self.states_dot - f_expl

acados_model = AcadosModel()
acados_model.f_impl_expr = f_impl
acados_model.f_expl_expr = f_expl
acados_model.x = self.states
acados_model.xdot = self.states_dot
acados_model.u = self.inputs
acados_model.p = self.param
acados_model.name = "centroidal_model"
return acados_model
```

**事実**: ここで`f_expl`（明示的なODE右辺、5節の式そのもの）と
`f_impl = states_dot - f_expl`（陰的形式、acadosの`IRK`系積分器や一部の定式化で使われる形）の
両方が登録される。実際に使われる積分方式は`create_ocp_solver_description()`の
`ocp.solver_options.integrator_type = "ERK"`（明示的Runge-Kutta）であり、これは`f_expl_expr`を
使う設定である（`f_impl_expr`は定義はされるが、`ERK`選択時に実際に使われるかどうかまでは
本ファイルでは検証していない、**未確認事項**）。

呼び出し元 `centroidal_nmpc_nominal.py::Acados_NMPC_Nominal.__init__()` L49–63:

```python
self.centroidal_model = Centroidal_Model_Nominal()
acados_model = self.centroidal_model.export_robot_model()
self.states_dim = acados_model.x.size()[0]     # 実測30
self.inputs_dim = acados_model.u.size()[0]     # 実測24

self.ocp = self.create_ocp_solver_description(acados_model)
code_export_dir = pathlib.Path(__file__).parent / "c_generated_code"
self.ocp.code_export_directory = str(code_export_dir)
self.acados_ocp_solver = AcadosOcpSolver(self.ocp, json_file=... + "centroidal_nmpc" + ".json")
```

`create_ocp_solver_description()` L78–274（該当箇所のみ抜粋、コスト・制約の中身は
スコープ外）:

```python
ocp = AcadosOcp()
ocp.model = acados_model          # L81: ここでAcadosModelがOCPに登録される
ocp.dims.N = self.horizon         # L87
...（コスト・制約：スコープ外）...
ocp.solver_options.integrator_type = "ERK"   # L205
ocp.solver_options.tf = self.T_horizon       # L259
return ocp
```

**事実**: `AcadosOcpSolver(self.ocp, ...)`の呼び出しが、acadosによるC言語コード生成・
コンパイル・ソルバーのロードを行う箇所である（生成物は`c_generated_code/`に出力される。
`AGENTS.md`にも既述）。

---

## 9. horizon、MPCの`dt`、予測時間の関係

`centroidal_nmpc_nominal.py::__init__()` L23–25:

```python
self.horizon = config.mpc_params['horizon']   # 既定12
self.dt = config.mpc_params['dt']             # 既定0.02 s
self.T_horizon = self.horizon * self.dt        # 既定 12*0.02 = 0.24 s
```

`create_ocp_solver_description()` L259: `ocp.solver_options.tf = self.T_horizon`。
すなわち**acadosのOCP全体の予測時間`tf`は、`horizon`と`dt`の単純な積として常に設定される**。

**事実（`use_nonuniform_discretization`有効時の追加処理、L261–272）**:

```python
if config.mpc_params['use_nonuniform_discretization']:
    time_steps_fine_grained = np.tile(config.mpc_params['dt_fine_grained'], config.mpc_params['horizon_fine_grained'])
    time_steps = np.concatenate((time_steps_fine_grained, np.tile(self.dt, self.horizon - config.mpc_params['horizon_fine_grained'])))
    shooting_nodes = np.zeros((self.horizon + 1,))
    for i in range(len(time_steps)):
        shooting_nodes[i + 1] = shooting_nodes[i] + time_steps[i]
    ocp.solver_options.shooting_nodes = shooting_nodes
```

**事実（数値的な食い違い）**: 既定値（`horizon=12`, `dt=0.02`, `horizon_fine_grained=2`,
`dt_fine_grained=0.01`）で計算すると、

$$
\text{shooting\_nodesの終端時刻} = 2\times0.01 + 10\times0.02 = 0.02+0.20 = 0.22\ \text{s}
$$

一方、`ocp.solver_options.tf`は`use_nonuniform_discretization`の値に関係なく常に
`self.T_horizon = horizon \times dt = 12\times0.02=0.24`秒に設定される（L259はこの`if`ブロックより
前で無条件に実行される）。したがって**`use_nonuniform_discretization=True`のとき、
`tf`（0.24秒）と`shooting_nodes`が実際に表す終端時刻（0.22秒）は一致しない**。

**解釈**: `use_nonuniform_discretization`は既定`False`であるため、この不一致は既定設定では
顕在化しない（潜在的な食い違いである）。この不一致がacados側でどう扱われるか
（`shooting_nodes`が`tf`を上書きするのか、あるいは矛盾としてエラーになるのか、
無視されるのか）は、acados本体の実装（対象ファイル外）を確認しないと分からない。
**未確認事項**である。

**事実（`optimal_next_state`とstateのスライス）**: `__init__()` L41で
`self.optimal_next_state = np.zeros((24,))`と、状態次元30ではなく24で初期化されている。
これは意図的な整合であることが後段（L1647）で確認できる:
```python
optimal_next_state = self.acados_ocp_solver.get(optimal_next_state_index, "x")[0:24]
```
状態30次元のうち、先頭24次元（CoM位置3+CoM速度3+姿勢3+角速度3+4脚足先位置12）だけを
取り出しており、末尾6つの積分状態（index 24–29）は除外される。したがって
`np.zeros((24,))`という初期化は、状態次元の誤解ではなく、この後段のスライスと
一致させた意図的な設計である。

---

## 10. 主要まとめ表

| 変数 | 次元 | 座標系/単位 | 生成元（ファイル） | 用途 |
|---|---|---|---|---|
| `x`（状態） | 30 | 混在（本文参照） | `centroidal_model_nominal.py::__init__` L54–77 | acados OCPの状態ベクトル |
| `u`（入力） | 24 | world, m/s・N | 同上 L96–115 | acados OCPの入力ベクトル |
| `p`（パラメータ） | 29 | 混在 | 同上 L141–150 | ステージごとに`compute_control()`から`set(j,"p",...)`で設定（`01`参照） |
| `T_horizon` | スカラー | s | `centroidal_nmpc_nominal.py` L25 | `ocp.solver_options.tf`（9節） |
| `horizon` | スカラー | ステップ数 | `config.mpc_params['horizon']`（既定12） | 状態・入力の予測ステップ数、`ocp.dims.N` |
| `dt` | スカラー | s | `config.mpc_params['dt']`（既定0.02） | `T_horizon`の計算に使用 |

---

## 11. 未確認事項（まとめ）

- 5.3節：`conj_euler_rates`が、参照した文献の定義と厳密に一致するかどうかの式的な検証。
- 5.4節：`b_R_w = Rx @ Ry @ Rz`という合成順序・各行列の符号が、コメント
  `"Z Y X rotations!"`と数学的に整合するかどうか。
- 5.6節：積分状態の`states_dot = states + tracked_value`という定式化が、acadosの
  ERK積分でどのように扱われ、実際の`use_integrators=True`時の挙動として
  意図通りかどうか。
- 7節：`inertia`パラメータが実際にworld frameかbody frameか（`simulation.py`のコメントと
  `forward_dynamics()`内の使われ方の食い違い）。`env.get_base_inertia()`の内部実装
  （`gym_quadruped`、対象ファイル外）。
- 8節：`f_impl_expr`が`integrator_type="ERK"`選択時に実際に使われるかどうか。
- 9節：`tf`と`shooting_nodes`の終端時刻不一致（`use_nonuniform_discretization=True`時）を
  acadosがどう扱うか。
- 4節：`mu_friction`, `base_position`, `base_yaw`が実際に使われる制約定式化の詳細
  （本ファイルではスコープ外としているため未調査）。
