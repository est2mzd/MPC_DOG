# SRBD力学モデル controllers/gradient/nominal/centroidal_model_nominal.py 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.srbd_controller_interface.compute_control(...)  (read_code_07)
          → self.controller = Acados_NMPC_Nominal()  (centroidal_nmpc_nominal.py、未解説)
              → self.centroidal_model = Centroidal_Model_Nominal()   ← 本ファイル
              → acados_model = self.centroidal_model.export_robot_model()  ← 本ファイル
```

**このファイルだけ、呼ばれるタイミングが他と違う**。read_code_02〜07までのファイルは
`simulation.py`のループが1回まわるたびに(または5ステップに1回)毎回呼ばれるものだった。
このファイルは、`Acados_NMPC_Nominal`が**最初に1回だけ生成されるとき**(プロセス起動時)に
シンボリックな数式を組み立てるだけで、以後の制御ループでは呼び出されない。ここで作られる
CasADiのシンボリック式(`self.states`, `self.inputs`, `forward_dynamics`の返り値)が、
acadosによってC言語のコードへ変換・コンパイルされ、それ以降はそのコンパイル済みコードが
使われる。

## このクラスの役割(全体の中での位置づけ)

`Centroidal_Model_Nominal`が担当するのは、「**MPCが状態・入力・パラメータをどう定義し、
それらの間にどんな運動方程式が成り立つか**」というOCPの土台(力学モデル)を定義すること
です。

- 状態:MPCが予測する対象(重心位置・速度、姿勢、角速度、4脚の位置、積分項)
- 入力:MPCが決める対象(4脚の速度、4脚の地面反力)
- パラメータ:MPCの外から与えられ、最適化されない値(接地フラグ、摩擦係数、慣性等)
- このクラス自体はコスト関数や制約を持たない。それらは次に読む`centroidal_nmpc_nominal.py`
  (`Acados_NMPC_Nominal`)の役割

対象は `external/Quadruped-PyMPC/quadruped_pympc/controllers/gradient/nominal/centroidal_model_nominal.py`
(340行)です。

---

## 1〜16行:import とクラスの位置づけ

```python
import os
import casadi as cs
import numpy as np
from acados_template import AcadosModel
import quadruped_pympc.config as config

class Centroidal_Model_Nominal:
    def __init__(self) -> None:
```

- `casadi`(`cs`という別名)：数式を記号のまま扱うライブラリ。ここで作る状態・入力・運動方程式は、数値ではなく「まだ数値の入っていない記号」として定義される
- `AcadosModel`：acados側が要求する、モデル定義をまとめる入れ物クラス。`export_robot_model`(340行目)が最終的にこれを返す

---

## 17〜152行:`__init__`

この関数の役割:状態・入力・パラメータをCasADiのシンボルとして定義し、運動方程式(`forward_dynamics`)を1回だけ評価してデバッグ用関数を作る。

### 23〜52行:状態シンボルの定義

```python
com_position_x = cs.SX.sym("com_position_x")
com_position_y = cs.SX.sym("com_position_y")
com_position_z = cs.SX.sym("com_position_z")

com_velocity_x = cs.SX.sym("com_velocity_x")
com_velocity_y = cs.SX.sym("com_velocity_y")
com_velocity_z = cs.SX.sym("com_velocity_z")

roll = cs.SX.sym("roll", 1, 1)
pitch = cs.SX.sym("pitch", 1, 1)
yaw = cs.SX.sym("yaw", 1, 1)
omega_x = cs.SX.sym("omega_x", 1, 1)
omega_y = cs.SX.sym("omega_y", 1, 1)
omega_z = cs.SX.sym("omega_z", 1, 1)

foot_position_fl = cs.SX.sym("foot_position_fl", 3, 1)
foot_position_fr = cs.SX.sym("foot_position_fr", 3, 1)
foot_position_rl = cs.SX.sym("foot_position_rl", 3, 1)
foot_position_rr = cs.SX.sym("foot_position_rr", 3, 1)

com_position_z_integral = cs.SX.sym("com_position_z_integral")
com_velocity_x_integral = cs.SX.sym("com_velocity_x_integral")
com_velocity_y_integral = cs.SX.sym("com_velocity_y_integral")
com_velocity_z_integral = cs.SX.sym("com_velocity_z_integral")
roll_integral = cs.SX.sym("roll_integral")
pitch_integral = cs.SX.sym("pitch_integral")
omega_x_integral = cs.SX.sym("omega_x_integral")
omega_y_integral = cs.SX.sym("omega_y_integral")
omega_z_integral = cs.SX.sym("omega_z_integral")
```

- `com_position_x/y/z`(m)：重心位置の3成分
- `com_velocity_x/y/z`(m/s)：重心速度の3成分
- `roll`/`pitch`/`yaw`(rad)：胴体の姿勢角
- `omega_x/y/z`(rad/s)：胴体の角速度
- `foot_position_fl`〜`rr`(m、3要素×4)：4脚の足位置
- `com_position_z_integral`(m·s)：重心高さの時間積分
- `com_velocity_x/y/z_integral`(m)：重心速度の時間積分
- `roll_integral`/`pitch_integral`(rad·s)：roll/pitch角の時間積分
- `omega_x/y/z_integral`(rad)：角速度の時間積分として定義されているシンボル

**実装上の問題点**：`omega_x_integral`/`omega_y_integral`/`omega_z_integral`(50〜52行)の
3つは、この直後の`self.states`(54〜77行)の`vertcat`に**含まれていない**。つまりシンボルとして
定義されるだけで、状態ベクトルには一切組み込まれない「死んだシンボル」になっている。

### 54〜77行:状態ベクトルの組み立て

```python
self.states = cs.vertcat(
    com_position_x, com_position_y, com_position_z,
    com_velocity_x, com_velocity_y, com_velocity_z,
    roll, pitch, yaw,
    omega_x, omega_y, omega_z,
    foot_position_fl, foot_position_fr, foot_position_rl, foot_position_rr,
    com_position_z_integral, com_velocity_x_integral, com_velocity_y_integral, com_velocity_z_integral,
    roll_integral, pitch_integral,
)
```

- `self.states`：状態ベクトル。実際に`vertcat`の引数を数えると、重心位置(3)+重心速度(3)+姿勢角(3)+角速度(3)+4脚の足位置(12)+積分項6個(1+3+1+1)=**30次元**

### 79〜93行:状態の時間微分の定義

```python
self.states_dot = cs.vertcat(
    cs.SX.sym("linear_com_vel", 3, 1),
    cs.SX.sym("linear_com_acc", 3, 1),
    cs.SX.sym("euler_rates_base", 3, 1),
    cs.SX.sym("angular_acc_base", 3, 1),
    cs.SX.sym("linear_vel_foot_FL", 3, 1),
    cs.SX.sym("linear_vel_foot_FR", 3, 1),
    cs.SX.sym("linear_vel_foot_RL", 3, 1),
    cs.SX.sym("linear_vel_foot_RR", 3, 1),
    cs.SX.sym("linear_com_vel_z_integral", 1, 1),
    cs.SX.sym("linear_com_acc_integral", 3, 1),
    cs.SX.sym("euler_rates_roll_integral", 1, 1),
    cs.SX.sym("euler_rates_pitch_integral", 1, 1),
)
```

- `self.states_dot`：`self.states`の各成分に対応する「変化率」の名前だけを定義した、30次元のプレースホルダ(3+3+3+3+12+1+3+1+1=30)。実際の値は`forward_dynamics`が計算する

### 95〜115行:入力シンボルの定義と組み立て

```python
foot_velocity_fl = cs.SX.sym("foot_velocity_fl", 3, 1)
foot_velocity_fr = cs.SX.sym("foot_velocity_fr", 3, 1)
foot_velocity_rl = cs.SX.sym("foot_velocity_rl", 3, 1)
foot_velocity_rr = cs.SX.sym("foot_velocity_rr", 3, 1)

foot_force_fl = cs.SX.sym("foot_force_fl", 3, 1)
foot_force_fr = cs.SX.sym("foot_force_fr", 3, 1)
foot_force_rl = cs.SX.sym("foot_force_rl", 3, 1)
foot_force_rr = cs.SX.sym("foot_force_rr", 3, 1)

self.inputs = cs.vertcat(
    foot_velocity_fl, foot_velocity_fr, foot_velocity_rl, foot_velocity_rr,
    foot_force_fl, foot_force_fr, foot_force_rl, foot_force_rr,
)
```

- `foot_velocity_fl`〜`rr`(m/s、3要素×4)：4脚の足速度
- `foot_force_fl`〜`rr`(N、3要素×4)：4脚の地面反力
- `self.inputs`：入力ベクトル。脚速度(12)+脚力(12)=**24次元**。MPCが最終的に決めたいのはこのうちの力の部分

### 117〜119行:デバッグ用の結合ベクトル

```python
self.y_ref = cs.vertcat(self.states, self.inputs)
```

- `self.y_ref`：状態と入力を1本に連結したもの(30+24=54次元)。コメント通り、`compute_control`関数内で「`y_ref`のどこに何が入っているか」を確認するデバッグ目的の変数

### 121〜138行:パラメータシンボルの定義

```python
self.stanceFL = cs.SX.sym("stanceFL", 1, 1)
self.stanceFR = cs.SX.sym("stanceFR", 1, 1)
self.stanceRL = cs.SX.sym("stanceRL", 1, 1)
self.stanceRR = cs.SX.sym("stanceRR", 1, 1)
self.stance_param = cs.vertcat(self.stanceFL, self.stanceFR, self.stanceRL, self.stanceRR)

self.mu_friction = cs.SX.sym("mu_friction", 1, 1)
self.stance_proximity = cs.SX.sym("stanceProximity", 4, 1)
self.base_position = cs.SX.sym("base_position", 3, 1)
self.base_yaw = cs.SX.sym("base_yaw", 1, 1)

self.external_wrench = cs.SX.sym("external_wrench", 6, 1)

self.inertia = cs.SX.sym("inertia", 9, 1)
self.mass = cs.SX.sym("mass", 1, 1)

self.gravity_constant = config.gravity_constant
```

- `self.stanceFL`〜`RR`(無次元、0か1)：各脚が接地しているかのフラグ。read_code_02の`contact_sequence`に由来
- `self.stance_param`：上記4つをまとめたベクトル(4要素)
- `self.mu_friction`(無次元)：摩擦係数
- `self.stance_proximity`(無次元、4要素)：各脚が接地に近いかのフラグ
- `self.base_position`(m、3要素)：胴体位置
- `self.base_yaw`(rad)：胴体yaw角
- `self.external_wrench`(力3+モーメント3、6要素)：外力補償用の外力・外モーメント
- `self.inertia`(kg·m²、9要素=3×3行列を平坦化)：慣性テンソル
- `self.mass`(kg)：ロボット質量
- `self.gravity_constant`：`config.py`と共有。既定`9.81`(m/s²)

### 140〜152行:パラメータベクトルの組み立てとデバッグ関数

```python
param = cs.vertcat(
    self.stance_param, self.mu_friction, self.stance_proximity,
    self.base_position, self.base_yaw, self.external_wrench, self.inertia, self.mass,
)
fd = self.forward_dynamics(self.states, self.inputs, param)
self.fun_forward_dynamics = cs.Function("fun_forward_dynamics", [self.states, self.inputs, param], [fd])
```

- `param`：パラメータベクトル。stance(4)+摩擦係数(1)+stance_proximity(4)+胴体位置(3)+yaw(1)+外力(6)+慣性(9)+質量(1)=**29次元**
- `self.fun_forward_dynamics`：`forward_dynamics`をCasADiの呼び出し可能関数としてラップしたもの。コメント「Not so useful」の通り、実際のOCP求解には使われず、デバッグ・検証用と考えられる

---

## 154〜308行:`forward_dynamics`

この関数の役割:今の状態・入力・パラメータから、状態の時間微分(=運動方程式の右辺)をシンボリックに計算して返す。

### 154〜167行:シグネチャとdocstring

```python
def forward_dynamics(self, states: np.ndarray, inputs: np.ndarray, param: np.ndarray) -> cs.SX:
    """
    ...
    Args:
        states: A numpy array of shape (29,) representing the current state of the robot.
        inputs: A numpy array of shape (29,) representing the inputs to the robot.
        param: A numpy array of shape (4,) representing the parameters (contact status) of the robot.

    Returns:
        A CasADi SX object of shape (29,) representing the predicted state of the robot.
    """
```

**実装上の問題点(docstringの誤りが4箇所)**：上のブロックで数えた実際の次元と比較すると、docstringはすべて誤っている。

| docstring上の記載 | 実際の次元 |
|---|---|
| `states`: shape (29,) | **30** |
| `inputs`: shape (29,) | **24** |
| `param`: shape (4,) | **29** |
| 戻り値: shape (29,) | **30**(`states_dot`と同じ) |

### 169〜199行:引数の切り出し

```python
foot_velocity_fl = inputs[0:3]
...
foot_force_fl = inputs[12:15]
...
com_position = states[0:3]
foot_position_fl = states[12:15]
...
stanceFL = param[0]
...
stance_proximity_FL = param[5]
...
external_wrench_linear = param[13:16]
external_wrench_angular = param[16:19]
inertia = param[19:28]
inertia = inertia.reshape((3, 3))
mass = param[28]
```

- `inputs`/`states`/`param`の各要素を、インデックスのスライスで個別の変数へ切り出す。この対応関係は、`__init__`で`vertcat`した順番と一致していなければならない(コード上は一致している)
- `inertia = inertia.reshape((3, 3))`：平坦化されていた9要素を3×3行列に戻す

### 201〜211行:並進運動方程式

```python
linear_com_vel = states[3:6]

temp = foot_force_fl @ stanceFL
temp += foot_force_fr @ stanceFR
temp += foot_force_rl @ stanceRL
temp += foot_force_rr @ stanceRR
temp += external_wrench_linear
gravity = np.array([0, 0, -self.gravity_constant])
linear_com_acc = (1 / mass) @ temp + gravity
```

- `linear_com_vel`(m/s)：states_dotの1番目の成分。今の重心速度をそのまま「重心位置の変化率」として使う(位置の微分=速度、という自明な関係)
- `linear_com_acc`(m/s²)：**重心の加速度 = (各脚の力×接地フラグの合計 + 外力) ÷ 質量 + 重力**。接地していない脚(`stance_i=0`)は自動的に効かなくなる

### 213〜232行:姿勢角の変化率(euler_rates_base)

```python
w = states[9:12]
roll = states[6]
pitch = states[7]
yaw = states[8]

conj_euler_rates = cs.SX.eye(3)
conj_euler_rates[1, 1] = cs.cos(roll)
conj_euler_rates[2, 2] = cs.cos(pitch) * cs.cos(roll)
conj_euler_rates[2, 1] = -cs.sin(roll)
conj_euler_rates[0, 2] = -cs.sin(pitch)
conj_euler_rates[1, 2] = cs.cos(pitch) * cs.sin(roll)

temp2 = cs.skew(foot_position_fl - com_position) @ foot_force_fl @ stanceFL
temp2 += cs.skew(foot_position_fr - com_position) @ foot_force_fr @ stanceFR
temp2 += cs.skew(foot_position_rl - com_position) @ foot_force_rl @ stanceRL
temp2 += cs.skew(foot_position_rr - com_position) @ foot_force_rr @ stanceRR

euler_rates_base = cs.inv(conj_euler_rates) @ w
```

- `w`(rad/s)：角速度(states[9:12])を取り出しただけ
- `conj_euler_rates`：角速度(`w`)からオイラー角の変化率へ変換するための行列。roll・pitchに応じて中身が変わる
- `temp2`：各脚の力が重心まわりに作るモーメントの合計(`skew(足位置-重心位置)`は外積を行列積で表す反対称行列)
- `euler_rates_base`(rad/s)：states_dotの3番目の成分。`conj_euler_rates`の逆行列を角速度に掛けて、オイラー角の変化率を得る

### 234〜272行:角加速度(angular_acc_base、回転の運動方程式)

```python
Rx = cs.SX.eye(3)
...(roll回りの回転行列)
Ry = cs.SX.eye(3)
...(pitch回りの回転行列)
Rz = cs.SX.eye(3)
...(yaw回りの回転行列)

b_R_w = Rx @ Ry @ Rz

temp2 = temp2 + external_wrench_angular
angular_acc_base = cs.inv(inertia) @ (b_R_w @ temp2 - cs.skew(w) @ inertia @ w)
```

- `Rx`/`Ry`/`Rz`：roll・pitch・yawそれぞれの単軸回転行列。コメント「Z Y X rotations!」の通りの順で組み合わされる
- `b_R_w`：world座標系からbody座標系への回転行列(`Rx@Ry@Rz`)
- `angular_acc_base`(rad/s²)：states_dotの4番目の成分。剛体の回転運動の基本法則(オイラーの運動方程式)そのもの:「角加速度 = 慣性の逆行列 × (world座標系のモーメントをbody座標系に変換したもの − 角速度に起因する慣性項)」
- 直後(274〜275行)に2行のコメントアウトされた別バージョンの式が残っている(過去の実装の試行錯誤の跡)

### 277〜286行:足位置の変化率(遊脚時のみ動く)

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

- `config.py`の`mpc_params['use_foothold_optimization']`は既定`True`。つまり`not True`で条件は`False`になり、**既定ではこの`if`ブロック(足速度を強制的にゼロにする処理)は実行されない**
- `linear_foot_vel_FL`(m/s)等：states_dotの5〜8番目の成分。「入力として与えられた足速度 × (1-接地フラグ) × (1-接地近接フラグ)」。接地中(`stance_i=1`)または接地間近(`stance_proximity_i=1`)なら、この積は0になり、足位置は動かないものとして扱われる。遊脚中はMPCが決めた`foot_velocity`がそのまま反映される

### 288〜295行:積分項

```python
integral_states = states[24:]
integral_states[0] += states[2]
integral_states[1] += states[3]
integral_states[2] += states[4]
integral_states[3] += states[5]
integral_states[4] += roll
integral_states[5] += pitch
```

- `integral_states`：状態ベクトルの24番目以降(積分項の6成分)を取り出し、対応する量(重心z位置、重心速度xyz、roll、pitch)を1ステップ分だけ足し込む。オイラー陽解法的な積分の近似で、PID制御のI項に相当する定常偏差の抑制に使われると考えられる

### 297〜308行:戻り値の組み立て

```python
return cs.vertcat(
    linear_com_vel, linear_com_acc, euler_rates_base, angular_acc_base,
    linear_foot_vel_FL, linear_foot_vel_FR, linear_foot_vel_RL, linear_foot_vel_RR,
    integral_states,
)
```

- ここまで計算した8つの成分(3+3+3+3+12)と積分項(6)を`self.states_dot`と同じ順番で連結し、30次元のベクトルとして返す

---

## 310〜339行:`export_robot_model`

この関数の役割:力学モデルをacadosが要求する`AcadosModel`形式にまとめて返す。

```python
def export_robot_model(self) -> AcadosModel:
    self.param = cs.vertcat(
        self.stance_param, self.mu_friction, self.stance_proximity,
        self.base_position, self.base_yaw, self.external_wrench, self.inertia, self.mass,
    )
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

- `self.param`：`__init__`内の`param`(ローカル変数)と同じ内容を、今度は`self.`付きでもう一度組み立て直している(29次元)
- `f_expl`：`forward_dynamics`を呼んで得た「明示的な」状態微分の式(states_dot = f_expl)
- `f_impl`：`states_dot - f_expl`という「陰的な」形の式。acadosの積分器はこの陰的形式を要求する
- `acados_model`：`x`(状態)・`xdot`(状態微分)・`u`(入力)・`p`(パラメータ)・力学式(`f_impl_expr`/`f_expl_expr`)・モデル名(`"centroidal_model"`)をまとめて、次に読む`centroidal_nmpc_nominal.py`側の`AcadosOcp`へ渡す

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `omega_x_integral`/`omega_y_integral`/`omega_z_integral`が定義されるが状態ベクトルに組み込まれない「死んだシンボル」
  2. `forward_dynamics`のdocstringが、states・inputs・param・戻り値の4箇所すべてで実際の次元と食い違っている(30→29、24→29、29→4、30→29)
  3. `angular_acc_base`の計算に、コメントアウトされた過去バージョンの式が2行残っている
- 確認できた事実:状態30次元(重心位置3+重心速度3+姿勢角3+角速度3+4脚位置12+積分項6)、入力24次元(脚速度12+脚力12)、パラメータ29次元(接地4+摩擦1+接地近接4+胴体位置3+yaw1+外力6+慣性9+質量1)
- `use_foothold_optimization`(既定`True`)により、足速度を強制ゼロにする分岐(277〜282行)は既定では通らない
- 次は、このモデルの上にコスト関数・制約・acadosソルバー設定を載せる`centroidal_nmpc_nominal.py`(`Acados_NMPC_Nominal`)を読みます。
