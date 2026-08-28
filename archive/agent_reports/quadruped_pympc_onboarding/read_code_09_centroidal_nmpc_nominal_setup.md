# OCPの定義 controllers/gradient/nominal/centroidal_nmpc_nominal.py(構築部分)逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.srbd_controller_interface.compute_control(...)  (read_code_07)
          → self.controller = Acados_NMPC_Nominal()   ← 本ファイル(__init__、プロセス起動時に1回)
              → self.centroidal_model = Centroidal_Model_Nominal()      (read_code_08)
              → self.ocp = self.create_ocp_solver_description(...)      ← 本ファイル
              → self.acados_ocp_solver = AcadosOcpSolver(self.ocp, ...) (acados本体、未読)
```

read_code_08と同じく、この章で扱う関数群(`__init__`・`create_ocp_solver_description`・
制約生成関数・`set_weight`・`reset`)は**プロセス起動時に1回だけ**実行される。毎制御周期
実行されるのは、この章では扱わない`set_stage_constraint`と`compute_control`(次章以降)。

## このファイルの役割(全体の中での位置づけ、本章が扱う範囲)

`Acados_NMPC_Nominal`が担当するのは、read_code_08の力学モデルの上に「**何を最小化し、
何を制約するか**」というOCPの定式化を載せ、acadosソルバーを実際に生成することです。
本章(read_code_09)では、そのうち**OCPの構造を定義する部分**(コスト関数の形・制約の
種類・ソルバーオプション)だけを扱います。実行時に毎ステップ値を詰め替える部分
(`set_stage_constraint`, `compute_control`)は次章以降です。

対象は `external/Quadruped-PyMPC/quadruped_pympc/controllers/gradient/nominal/centroidal_nmpc_nominal.py`
のうち1〜576行付近(`__init__`〜`set_weight`〜`reset`)です(ファイル全体は1705行)。

---

## 1〜21行:import とクラスの位置づけ

```python
from acados_template import AcadosOcp, AcadosOcpSolver
ACADOS_INFTY = 1000
import casadi as cs
import numpy as np
import scipy.linalg
import quadruped_pympc.config as config
from .centroidal_model_nominal import Centroidal_Model_Nominal

class Acados_NMPC_Nominal:
```

- `ACADOS_INFTY = 1000`：acadosで「実質的な無限大」を表すのに使う定数(無次元)。値は`1000`
- `AcadosOcp`/`AcadosOcpSolver`：acados公式ライブラリのクラス。OCPの定義(`AcadosOcp`)とソルバー本体(`AcadosOcpSolver`)

---

## 22〜77行:`__init__`

この関数の役割:設定を読み込み、力学モデルを生成し、OCPを定義し、acadosソルバーを実際に構築する。

### 23〜44行:設定フラグの読み込みと内部変数の初期化

```python
self.horizon = config.mpc_params['horizon']
self.dt = config.mpc_params['dt']
self.T_horizon = self.horizon * self.dt
self.use_RTI = config.mpc_params["use_RTI"]
self.use_integrators = config.mpc_params["use_integrators"]
self.use_warm_start = config.mpc_params["use_warm_start"]
self.use_foothold_constraints = config.mpc_params["use_foothold_constraints"]
self.use_static_stability = config.mpc_params["use_static_stability"]
self.use_zmp_stability = config.mpc_params["use_zmp_stability"]
self.use_stability_constraints = self.use_static_stability or self.use_zmp_stability
self.use_DDP = config.mpc_params["use_DDP"]
self.verbose = config.mpc_params["verbose"]

self.previous_status = -1
self.previous_contact_sequence = np.zeros((4, self.horizon))
self.optimal_next_state = np.zeros((24,))
self.previous_optimal_GRF = np.zeros((12,))
self.integral_errors = np.zeros((6,))
self.initial_base_position = np.array([0, 0, 0])
```

- `self.horizon`：予測ホライズンのステップ数(無次元)。既定`12`
- `self.dt`：OCPの内部離散化の時間刻み(秒)。既定`0.02`
- `self.T_horizon`：予測ホライズンの総時間(秒)。`horizon × dt`。既定`12×0.02=0.24`秒
- `self.use_RTI`：Real-Time Iterationを使うかのフラグ(`bool`)。既定`False`
- `self.use_integrators`：積分項を使うかのフラグ(`bool`)。既定`False`
- `self.use_warm_start`：ウォームスタートを使うかのフラグ(`bool`)。既定`False`
- `self.use_foothold_constraints`：着地点のbox制約を使うかのフラグ(`bool`)。既定`False`
- `self.use_static_stability`/`self.use_zmp_stability`：安定性制約の種類を選ぶフラグ(`bool`)。どちらも既定`False`
- `self.use_stability_constraints`：上記2つの論理和。既定`False`
- `self.use_DDP`：DDP(微分動的計画法)を使うかのフラグ(`bool`)。既定`False`
- `self.verbose`：詳細ログ出力のフラグ(`bool`)。既定`False`
- `self.previous_status`：前回のソルバー終了ステータス(`int`)。初期値`-1`
- `self.previous_contact_sequence`：前回の接地スケジュール(4×horizon、無次元)。初期値は全`0`
- `self.optimal_next_state`：MPCが予測した次の状態(24要素)。初期値は全`0`
- `self.previous_optimal_GRF`：前回の最適GRF(12要素、N)。初期値は全`0`
- `self.integral_errors`：積分誤差(6要素)。初期値は全`0`
- `self.initial_base_position`：胴体位置を原点に揃えるための基準点(m、3要素)。値は`[0,0,0]`固定

**気になる点**：`self.optimal_next_state`が24要素で初期化されている。read_code_08で確認した状態ベクトルは30次元(積分項6を含む)なので、この24は「積分項を除いた状態」(重心位置3+重心速度3+姿勢角3+角速度3+4脚位置12=24)に対応すると考えられる(**設計上の解釈**、この変数がどこで読み書きされるかは未読のため未確認)。

### 49〜63行:モデルの生成とOCP・ソルバーの構築

```python
self.centroidal_model = Centroidal_Model_Nominal()
acados_model = self.centroidal_model.export_robot_model()
self.states_dim = acados_model.x.size()[0]
self.inputs_dim = acados_model.u.size()[0]

self.ocp = self.create_ocp_solver_description(acados_model)

code_export_dir = pathlib.Path(__file__).parent / "c_generated_code"
self.ocp.code_export_directory = str(code_export_dir)

self.acados_ocp_solver = AcadosOcpSolver(
    self.ocp, json_file=self.ocp.code_export_directory + "/centroidal_nmpc" + ".json"
)
```

- `self.centroidal_model`：read_code_08の`Centroidal_Model_Nominal`インスタンス
- `self.states_dim`：状態の次元数(無次元)。read_code_08の実測通り`30`
- `self.inputs_dim`：入力の次元数(無次元)。read_code_08の実測通り`24`
- `self.ocp`：これから読む`create_ocp_solver_description`が組み立てる`AcadosOcp`オブジェクト
- `self.acados_ocp_solver`：**ここでacadosの実際のソルバーが生成される**。`build=False`・`generate=False`が指定されていないため、既定では毎回コード生成とコンパイルが行われる(後述の`reset`との違い)

### 65〜74行:ソルバーの初期化

```python
for stage in range(self.horizon + 1):
    self.acados_ocp_solver.set(stage, "x", np.zeros((self.states_dim,)))
for stage in range(self.horizon):
    self.acados_ocp_solver.set(stage, "u", np.zeros((self.inputs_dim,)))

if self.use_RTI:
    self.acados_ocp_solver.options_set("rti_phase", 1)
    status = self.acados_ocp_solver.solve()
```

- ホライズン全ステージ(`horizon+1`個、境界含む)の状態を全`0`(30次元)、`horizon`個の入力を全`0`(24次元)で初期化する
- `use_RTI`が既定`False`のため、RTIの準備フェーズ(`rti_phase=1`での初回`solve()`)は**既定では実行されない**

---

## 78〜274行:`create_ocp_solver_description`

この関数の役割:コスト関数・制約・ソルバーオプションを設定した`AcadosOcp`オブジェクトを組み立てて返す。

### 78〜110行:コスト関数の形式

```python
ocp = AcadosOcp()
ocp.model = acados_model
nx = self.states_dim
nu = self.inputs_dim
ocp.dims.N = self.horizon

Q_mat, R_mat = self.set_weight(nx, nu)
ocp.cost.cost_type = "LINEAR_LS"
ocp.cost.cost_type_e = "LINEAR_LS"

ocp.cost.W_e = Q_mat
ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

ocp.cost.Vx = np.zeros((ny, nx))
ocp.cost.Vx[:nx, :nx] = np.eye(nx)
Vu = np.zeros((ny, nu))
Vu[nx : nx + nu, 0:nu] = np.eye(nu)
ocp.cost.Vu = Vu
ocp.cost.Vx_e = np.eye(nx)

ocp.cost.yref = np.zeros((ny,))
ocp.cost.yref_e = np.zeros((ny_e,))
```

- `nx`(無次元)：状態次元。`30`
- `nu`(無次元)：入力次元。`24`
- `ocp.dims.N`：ホライズンのステージ数。`12`
- `Q_mat`/`R_mat`：状態・入力のコスト重み行列(下記`set_weight`で計算、それぞれ30×30・24×24の対角行列)
- `ocp.cost.cost_type`：コスト形式。`"LINEAR_LS"`(線形最小二乗)。DDP使用時のみ後で`"NONLINEAR_LS"`に上書きされる(206〜215行、既定では通らない)
- `ocp.cost.W_e`：終端コストの重み行列。`Q_mat`をそのまま使う(終端専用の別重みは無い)
- `ocp.cost.W`：中間ステージのコスト重み。`Q_mat`と`R_mat`をブロック対角に結合(54×54)
- `ocp.cost.Vx`/`ocp.cost.Vu`：状態・入力を「コストが見る量」(`y = Vx@x + Vu@u`)へ写す選択行列。ここでは単位行列を使い、状態・入力をそのままコストの対象にしている
- `ocp.cost.yref`/`ocp.cost.yref_e`：参照値。ここでは全`0`で初期化されるだけで、実際の目標値は毎ステップ`set_stage_constraint`(未読、次章)が上書きする

### 112〜145行:制約の追加(摩擦錐は常時、着地点・安定性は条件付き)

```python
expr_h_friction, self.constr_uh_friction, self.constr_lh_friction = self.create_friction_cone_constraints()
ocp.model.con_h_expr = expr_h_friction
ocp.constraints.uh = self.constr_uh_friction
ocp.constraints.lh = self.constr_lh_friction
ocp.model.con_h_expr_0 = expr_h_friction
ocp.constraints.uh_0 = self.constr_uh_friction
ocp.constraints.lh_0 = self.constr_lh_friction
nsh = expr_h_friction.shape[0]
nsh_state_constraint_start = copy.copy(nsh)

if self.use_foothold_constraints:
    expr_h_foot, self.constr_uh_foot, self.constr_lh_foot = self.create_foothold_constraints()
    ocp.model.con_h_expr = cs.vertcat(ocp.model.con_h_expr, expr_h_foot)
    ocp.constraints.uh = np.concatenate((ocp.constraints.uh, self.constr_uh_foot))
    ocp.constraints.lh = np.concatenate((ocp.constraints.lh, self.constr_lh_foot))
    nsh += expr_h_foot.shape[0]

if self.use_stability_constraints:
    self.nsh_stability_start = copy.copy(nsh)
    expr_h_support_polygon, self.constr_uh_support_polygon, self.constr_lh_support_polygon = (
        self.create_stability_constraints()
    )
    ocp.model.con_h_expr = cs.vertcat(ocp.model.con_h_expr, expr_h_support_polygon)
    ocp.constraints.uh = np.concatenate((ocp.constraints.uh, self.constr_uh_support_polygon))
    ocp.constraints.lh = np.concatenate((ocp.constraints.lh, self.constr_lh_support_polygon))
    nsh += expr_h_support_polygon.shape[0]
    self.nsh_stability_end = copy.copy(nsh)

nsh_state_constraint_end = copy.copy(nsh)
```

- 摩擦錐制約(下記`create_friction_cone_constraints`、20本)は`if`なしで**常に**`ocp.model.con_h_expr`へ設定される
- `self.use_foothold_constraints`(既定`False`)がTrueのときだけ、着地点のbox制約(8本、下記`create_foothold_constraints`)が`vertcat`で追記される。**既定では実行されない**
- `self.use_stability_constraints`(既定`False`)がTrueのときだけ、支持多角形の安定性制約(6本、下記`create_stability_constraints`)が追記される。**既定では実行されない**
- `nsh`：現在の制約本数の累計(無次元)。既定設定では摩擦錐の20本のまま増えない

### 147〜172行:スラック変数の設定(既定では未実行)

```python
num_state_cstr = nsh_state_constraint_end - nsh_state_constraint_start
if num_state_cstr > 0:
    ocp.constraints.lsh = np.zeros(num_state_cstr)
    ocp.constraints.ush = np.zeros(num_state_cstr)
    ocp.constraints.idxsh = np.array(range(nsh_state_constraint_start, nsh_state_constraint_end))
    ns = num_state_cstr
    ocp.cost.zl = 1000 * np.ones((ns,))
    ocp.cost.Zl = 1 * np.ones((ns,))
    ocp.cost.zu = 1000 * np.ones((ns,))
    ocp.cost.Zu = 1 * np.ones((ns,))

list_upper_bound = []
list_lower_bound = []
for j in range(self.horizon):
    list_upper_bound.append(np.zeros((nsh,)))
    list_lower_bound.append(np.zeros((nsh,)))
self.upper_bound = np.array(list_upper_bound, dtype=object)
self.lower_bound = np.array(list_lower_bound, dtype=object)
```

- `num_state_cstr`：着地点・安定性制約で追加された本数。既定設定ではどちらもFalseのため`nsh_state_constraint_start == nsh_state_constraint_end`となり、**`num_state_cstr=0`、この`if`ブロック全体が既定では実行されない**
- `ocp.cost.zl`/`Zl`/`zu`/`Zu`：ソフト制約(スラック変数)のペナルティ係数。既定では設定されないままになる
- `self.upper_bound`/`self.lower_bound`：ホライズン分の制約上下限を保存する入れ物(`horizon`個、各`nsh`要素のゼロ配列)。摩擦錐だけなら`nsh=20`

### 174〜199行:初期状態と初期パラメータ

```python
X0 = np.zeros(shape=(nx,))
ocp.constraints.x0 = X0

init_contact_status = np.array([1.0, 1.0, 1.0, 1.0])
init_mu = np.array([0.5])
init_stance_proximity = np.array([0, 0, 0, 0])
init_base_position = np.array([0, 0, 0])
init_base_yaw = np.array([0])
init_external_wrench = np.array([0, 0, 0, 0, 0, 0])
init_inertia = config.inertia.reshape((9,))
init_mass = np.array([config.mass])

ocp.parameter_values = np.concatenate(
    (init_contact_status, init_mu, init_stance_proximity, init_base_position,
     init_base_yaw, init_external_wrench, init_inertia, init_mass)
)
```

- `X0`：初期状態制約(m等、30要素)。全`0`で初期化(実際の初期状態は毎ステップ上書きされる)
- `init_contact_status`(無次元)：初期の接地フラグ。`[1,1,1,1]`(全脚接地から開始)
- `init_mu`(無次元)：初期摩擦係数。`0.5`
- `init_stance_proximity`(無次元)：`[0,0,0,0]`
- `init_base_position`(m)：`[0,0,0]`
- `init_base_yaw`(rad)：`[0]`
- `init_external_wrench`：`[0,0,0,0,0,0]`
- `init_inertia`(kg·m²)：`config.py`の固定慣性値(Go2なら`[[0.158,...],...]`の9要素平坦化)。これは`use_inertia_recomputation=True`のときは後で毎ステップ上書きされる初期値に過ぎない
- `init_mass`(kg)：`config.py`の質量。Go2なら`15.019`
- これらはすべて**OCP構築時の1回限りの初期値**で、実際の制御では毎ステップ`set_stage_constraint`(次章)が新しい値で上書きする

### 201〜256行:ソルバーオプション

```python
ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
ocp.solver_options.integrator_type = "ERK"

if self.use_DDP:
    ...(既定では通らない)
elif self.use_RTI:
    ...(既定では通らない)
else:
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = config.mpc_params["num_qp_iterations"]

if config.mpc_params['solver_mode'] == "balance":
    ocp.solver_options.hpipm_mode = "BALANCE"
elif ...

ocp.solver_options.levenberg_marquardt = 1e-3
ocp.solver_options.tf = self.T_horizon
```

- `qp_solver`：`"PARTIAL_CONDENSING_HPIPM"`固定
- `hessian_approx`：`"GAUSS_NEWTON"`固定(厳密なヘッセ行列ではなく、1階微分の外積で近似する手法)
- `integrator_type`：`"ERK"`(陽的ルンゲクッタ)固定
- `use_DDP`(既定`False`)・`use_RTI`(既定`False`)がどちらも`False`なので、**既定では`else`節が使われ、`nlp_solver_type="SQP"`**
- `nlp_solver_max_iter`：`config.py`の`num_qp_iterations`。既定`1`。**つまり既定設定では、1制御周期あたりSQPの反復を1回しか行わない**。ホライズンを跨いだ収束は、次の周期の(ウォームスタートに近い)再求解に委ねる設計と考えられる(**設計上の解釈**)
- `solver_mode`：既定`'balance'`のため`hpipm_mode="BALANCE"`
- `levenberg_marquardt`：正則化係数。`1e-3`固定
- `ocp.solver_options.tf`：予測ホライズンの総時間(秒)。`T_horizon`(既定`0.24`)

### 258〜272行:非一様離散化(既定では未実行)

```python
if config.mpc_params['use_nonuniform_discretization']:
    ...
```

- `use_nonuniform_discretization`は既定`False`のため、このブロックは実行されず、`ocp.solver_options.shooting_nodes`は明示的に設定されない(acados側の既定である等間隔`dt`刻みのまま)

---

## 277〜381行:`create_stability_constraints`(既定では未使用)

この関数の役割:ZMP(またはCoM)が支持多角形の内側にあるという条件を、6本の線形不等式として作る。

- 4脚の位置を胴体中心のhorizontal frameへ変換したあと、`use_static_stability`(既定`False`)なら原点(0,0)、そうでなければZMP(x,y)を計算する
- ZMPの計算式：`zmp = base_w[0:2] - linear_com_acc[0:2] * (robotHeight / g)`。倒立振子近似に基づく、重心の水平加速度から支持点を逆算する古典的な式
- 4脚を頂点とする四角形の各辺(FL-FR, FR-RR, RR-RL, RL-FL)について「ZMPがその辺の内側にあるか」を表す線形不等式を6本作る(対角のFL-RR、FR-RLも含む)
- この制約自体は`use_stability_constraints`(`use_static_stability or use_zmp_stability`、既定どちらも`False`)がTrueのときしか`create_ocp_solver_description`から呼ばれないため、**既定では一度も呼ばれない**

---

## 384〜427行:`create_foothold_constraints`(既定では未使用)

この関数の役割:4脚の着地点がworld座標系のどこにあるかを、そのままbox制約として表現する。

- docstringのshape記載(`(8,1)`)は実際の`Jbu`(4脚×3成分=12要素)と食い違っている
- 各脚の位置をhorizontal frameのx,yへ変換し、zはそのまま状態の値を使う
- 上下限(`ubu`/`lbu`)は`±ACADOS_INFTY`のまま返される。つまりこの関数自体は「制約の枠(どの量を制約するか)」を定義するだけで、実際の上下限値(着地可能な範囲)はここでは設定されておらず、別の場所(未読)で上書きされる設計と考えられる
- `use_foothold_constraints`が既定`False`のため、**既定では一度も呼ばれない**

---

## 430〜499行:`create_friction_cone_constraints`(常時使用)

この関数の役割:各脚の地面反力が摩擦円錐の中に収まるという条件を、20本の線形不等式として作る。

- `f_max`(N)：`config.py`の`grf_max = mass * gravity_constant`。Go2(`mass=15.019`, `gravity_constant=9.81`)なら約`147.3`N
- `f_min`(N)：`config.py`の`grf_min`。既定`0`
- Focchi論文(コメントに明記)の線形近似による摩擦円錐。1脚あたり5本(左右2方向のピラミッド近似4本+垂直力の上下限1本)×4脚=20本
- この関数は`if`なしで`create_ocp_solver_description`から常に呼ばれる。既定設定で唯一有効な制約はこれだけ

---

## 501〜551行:`set_weight`

この関数の役割:コスト関数の重み行列(状態用`Q_mat`、入力用`R_mat`)を数値で組み立てて返す。

```python
Q_position = np.array([0, 0, 1500])       # x, y, z
Q_velocity = np.array([200, 200, 200])    # x_vel, y_vel, z_vel
Q_base_angle = np.array([500, 500, 0])    # roll, pitch, yaw
Q_base_angle_rates = np.array([20, 20, 50])  # roll_rate, pitch_rate, yaw_rate
Q_foot_pos = np.array([300, 300, 300])    # 1脚あたり、4脚分繰り返す
Q_com_position_z_integral = np.array([50])
Q_com_velocity_x_integral = np.array([10])
Q_com_velocity_y_integral = np.array([10])
Q_com_velocity_z_integral = np.array([10])
Q_roll_integral_integral = np.array([10])
Q_pitch_integral_integral = np.array([10])

R_foot_vel = np.array([0.0001, 0.0001, 0.00001])
R_foot_force = np.array([0.001, 0.001, 0.001])  # hyqrealのみ [0.00001]×3
```

- `Q_position`：重心x,y,zの追従コスト重み(無次元)。**x,yは`0`(気にしない)、zだけ`1500`**
- `Q_velocity`：重心速度x,y,zの追従コスト重み。`[200,200,200]`(等方)
- `Q_base_angle`：roll,pitch,yawの追従コスト重み。**roll,pitchは`500`、yawは`0`(気にしない)**
- `Q_base_angle_rates`：角速度の追従コスト重み。`[20,20,50]`
- `Q_foot_pos`：各脚の目標着地点からのずれのコスト重み。4脚とも`[300,300,300]`
- `Q_*_integral`：各積分項のコスト重み。`50`(重心z)または`10`(その他)
- `R_foot_vel`：脚速度のコスト重み(小さいほど自由に動ける)。`[0.0001, 0.0001, 0.00001]`
- `R_foot_force`：脚力のコスト重み。Go2を含む大半のロボットは`[0.001, 0.001, 0.001]`、`hyqreal`だけ`[0.00001]×3`(条件式の判定は`config.robot=="hyqreal"`だが、実在するロボット名リストには`'hyqreal1'`/`'hyqreal2'`しかなく`'hyqreal'`という名前は無いため、**この`if`が真になることは無い**可能性が高い、**未確認**)
- `Q_mat`/`R_mat`：上記をこの順で対角に並べた30×30・24×24の対角行列

---

## 553〜561行:`reset`

この関数の役割:既存の生成済みコードを再利用したまま、acadosソルバーだけを作り直す。

```python
def reset(self):
    self.acados_ocp_solver.reset()
    self.acados_ocp_solver = AcadosOcpSolver(
        self.ocp,
        json_file=self.ocp.code_export_directory + "/centroidal_nmpc" + ".json",
        build=False,
        generate=False,
    )
```

- `__init__`内の`AcadosOcpSolver(...)`(61〜63行)と違い、ここでは`build=False`・`generate=False`が明示的に指定されている
- つまり`__init__`時は(デフォルト`build=True, generate=True`のため)コード生成とコンパイルが走るが、`reset()`はそれを行わず、**既にコンパイル済みの共有ライブラリをそのまま再利用する**。エピソード切り替え時に毎回コンパイルし直すと極端に遅くなるため、この違いは意図的な設計と考えられる

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `create_foothold_constraints`のdocstringが`shape (8,1)`と書いているが、実際は12要素(4脚×3)
  2. `R_foot_force`の`hyqreal`分岐の条件`config.robot == "hyqreal"`が、実在するロボット名(`hyqreal1`/`hyqreal2`)と一致しない可能性が高く、死んだ分岐になっているおそれがある
- 既定設定(`use_foothold_constraints=False`, `use_static_stability=False`, `use_zmp_stability=False`, `use_DDP=False`, `use_RTI=False`, `use_nonuniform_discretization=False`)では:
  - 有効な制約は摩擦錐(20本)のみ
  - スラック変数(ソフト制約)の設定ブロックは実行されない
  - ソルバーは`SQP`、1周期あたり最大反復回数`1`回
- 次は、この定義済みOCPへ毎ステップ実際の状態・目標値・パラメータを詰め込む`set_stage_constraint`を読みます。かなり大きい関数(486行)のため、独立した章として扱います。
