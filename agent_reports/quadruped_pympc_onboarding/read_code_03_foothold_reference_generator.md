# 上流の計画② helpers/foothold_reference_generator.py 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

このファイルは`simulation.py`から直接は呼ばれていませんが、次の連鎖でサブのサブとして呼ばれています。

```text
simulation.py (run_simulationのループ、read_code_01で解説)
  → quadrupedpympc_wrapper.compute_actions(...) (NNで解説)
      → self.wb_interface.update_state_and_reference(...) (wb_interface.py、未解説)
          → self.frg.update_lift_off_positions(...)      ← 本ファイル
          → self.frg.update_touch_down_positions(...)     ← 本ファイル
          → self.frg.compute_footholds_reference(...)     ← 本ファイル(このファイルの中心処理)
```

`self.frg`は`WBInterface.__init__`の中で`FootholdReferenceGenerator(...)`として生成されるインスタンスです(NNで見た`quadruped_pympc_wrapper.py`37行目の`WBInterface(...)`生成の中でさらに作られます)。呼び出し頻度は、NNで確認した「MPCの間引き」とは無関係に、**`simulation.py`の内側ループが1回まわるたびに毎回**呼ばれます(既定500Hz相当)。歩容(read_code_02で読んだ`PeriodicGaitGenerator`)は「いつ接地するか」を決め、このファイルは「どこに接地するか」を決めます。

## このクラスの役割(全体の中での位置づけ)

`FootholdReferenceGenerator`が担当するのは、制御パイプライン全体のうち「**各脚をどこに着地させるか**」という着地点の決定です。[read_code_02](read_code_02_periodic_gait_generator.md)の`PeriodicGaitGenerator`が決めた「いつ」(接地スケジュール)を前提として、今度は「どこ」(x,y,z座標)を、Raibertヒューリスティックという古典的な幾何公式で計算します。

- 入力：現在の胴体速度・目標速度・股関節位置・接地状態の遷移(前ステップとの比較による離地/着地の検出)
- 出力：`ref_feet_pos`(4脚のworld座標系での目標着地点)
- この出力は、後段のMPC(`centroidal_nmpc_nominal.py`、まだ未解説)のコスト関数の**参照値**として使われます(着地点そのものを厳密な制約にはせず、「近づけたい目標」として扱う設計です)
- 「いつ接地するか」はこのクラスの責務ではなく、前段の`PeriodicGaitGenerator`が決めたスケジュール(`current_contact`/`previous_contact`)をそのまま受け取って使います

対象は `external/Quadruped-PyMPC/quadruped_pympc/helpers/foothold_reference_generator.py`(229行)です。

---

## 1〜13行：importとクラスの位置づけ

```python
import collections
import copy

import mujoco
import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr
from scipy.spatial.transform import Rotation

from quadruped_pympc.helpers.quadruped_utils import GaitType
from quadruped_pympc import config as cfg

# Class for the generation of the reference footholds
# TODO: @Giulio Should we convert this to a single function instead of a class? Stance time, can be passed as argument
class FootholdReferenceGenerator:
```

- `collections`：`deque`(両端キュー)を使った移動平均のために使用
- `mujoco`：202行目以降の`__main__`ブロック(このファイルを直接実行したときのテストコード)でのみ使用
- クラス直前のコメントに開発者自身のTODOが残っている
  - 「クラスではなく単一の関数にすべきでは?」という設計への迷いがそのままコードに残っている

---

## 15〜51行：コンストラクタ

この関数の役割:着地点計算に必要な状態(離地位置・着地位置・速度履歴等)を初期化する。

```python
def __init__(
    self, stance_time: float, lift_off_positions: LegsAttr, vel_moving_average_length=20, hip_height: float = None
) -> None:
```

- `stance_time`：接地期の時間(秒)。デフォルト値なし(必須引数)
- `lift_off_positions`：離地位置の初期値(m、world座標系、`LegsAttr`)。デフォルト値なし(必須引数)
- `vel_moving_average_length`：速度移動平均のウィンドウ幅(件数)。デフォルト`20`
- `hip_height`：股関節の高さ(m)。デフォルト`None`(実際の呼び出しでは`config.py`の値が渡される)

```python
self.base_vel_hist = collections.deque(maxlen=vel_moving_average_length)
self.stance_time = stance_time
self.hip_height = hip_height
self.lift_off_positions = copy.deepcopy(lift_off_positions)
self.touch_down_positions = copy.deepcopy(lift_off_positions)
```

- `self.base_vel_hist`：直近の胴体速度の履歴(m/s、最大20件、デフォルト値なし・空で開始)。
- `self.stance_time`：1回の接地期間の長さ(秒)。
- `self.hip_height`：股関節の高さ(m)。
- `self.lift_off_positions`：各脚が最後に地面を離れた瞬間の位置(m、world座標系)。
- `self.touch_down_positions`：各脚が最後に地面へ着いた瞬間の位置(m、world座標系)。

```python
self.com_pos_offset_b = np.zeros((3,))
self.com_pos_offset_w = np.zeros((3,))
```

- `self.com_pos_offset_b`：重心位置への手動オフセット(m、base座標系)。初期値は`[0,0,0]`
- `self.com_pos_offset_w`：同じオフセットのworld座標系版(m)。初期値は`[0,0,0]`
- コメント「The offset of the COM position wrt the estimated one. HACK compensation」。推定された重心位置に対する手動オフセットであり、コメント自身が「HACK(その場しのぎの対処)」と認めている

```python
"""R_W2H = np.array([np.cos(yaw), np.sin(yaw),
                  -np.sin(yaw), np.cos(yaw)])
R_W2H = R_W2H.reshape((2, 2))
self.lift_off_positions_h =  R_W2H @ (self.lift_off_positions - base_position[0:2])"""
self.lift_off_positions_h = copy.deepcopy(lift_off_positions)  # TODO wrong
self.touch_down_positions_h = copy.deepcopy(lift_off_positions)  # TODO wrong
```

**実装上の問題点(開発者自身が明記)**:

- 三重引用符でコメントアウトされているのは、「本来こうあるべき」というコード(world座標系からhorizontal frameへ回転変換してから保存する)
- しかし実際に実行される行は、その変換をせず**単純にコピーしているだけ**
- しかも`# TODO wrong`というコメントが両方の行に明記されている。つまりこれは私の推測ではなく、**開発者自身が「これは間違っている」と認めている未修正の箇所**
- `lift_off_positions_h`/`touch_down_positions_h`という「horizontal frame版」の変数が、実際にはworld frameの値のままになっている可能性が高い

```python
self.hip_offset = 0.1

self.last_reference_footholds = LegsAttr(
    FL=np.array([0, 0, 0]), FR=np.array([0, 0, 0]), RL=np.array([0, 0, 0]), RR=np.array([0, 0, 0])
)

self.gravity_constant = cfg.gravity_constant
```

- `self.hip_offset`：股関節位置から左右にどれだけずらすかのハードコード値(m)。値は`0.1`固定。この直後のTODOコメント(124〜125行、後述)で「ロボット設定のプロパティにすべき」と書かれている
- `self.last_reference_footholds`：直近の参照着地点を保持するプレースホルダ(m、`LegsAttr`)。初期値は各脚`[0,0,0]`
- `self.gravity_constant`：重力加速度(m/s²)。デフォルト値はないが`config.py`の`gravity_constant`が渡される(既定`9.81`)

---

## 53〜85行：`compute_footholds_reference`の入口

この関数の役割:Raibertヒューリスティックで、4脚の目標着地点(world座標系)を計算して返す。

```python
def compute_footholds_reference(
    self,
    base_position: np.ndarray,
    base_ori_euler_xyz: np.ndarray,
    base_xy_lin_vel: np.ndarray,
    ref_base_xy_lin_vel: np.ndarray,
    hips_position: LegsAttr,
    com_height_nominal: np.float32,
) -> LegsAttr:
```

- 引数の意味(すべてデフォルト値はなく、呼び出し元が毎回渡す):
  - `base_position`：胴体(重心)の現在位置(m、3要素、world座標系)
  - `base_ori_euler_xyz`：胴体の現在の姿勢(rad、roll, pitch, yaw)
  - `base_xy_lin_vel`：胴体の現在の水平速度(m/s、x, yの2要素)
  - `ref_base_xy_lin_vel`：目標の水平速度(m/s、x, yの2要素)
  - `hips_position`：4本の股関節位置(m、world座標系)
  - `com_height_nominal`：目標とする重心の高さ(m)。呼び出し元では`config.py`の`ref_z`(既定`hip_height`と同じ、Go2なら`0.28`)が渡される
- docstringのTODOに開発者自身の課題意識が書かれている:
  - 「地形の傾斜推定を使うべき」
  - 「目標のyaw角速度の誤差補正も、並進速度の補正と同様に行うべき」
  - 「脚ごとの計算をベクトル化すべき」
- 3つとも未実装のまま残っている

```python
assert base_xy_lin_vel.shape == (2,) and ref_base_xy_lin_vel.shape == (2,), (
    f"Expected shape (2,):=[x_dot, y_dot], got {base_xy_lin_vel.shape} and {ref_base_xy_lin_vel.shape}."
)
```

- 引数のshapeを実行時に検証する`assert`文。このファイルの中では珍しく、明示的な形状チェックが入っている

```python
yaw = base_ori_euler_xyz[2]
R_W2H = np.array([np.cos(yaw), np.sin(yaw), -np.sin(yaw), np.cos(yaw)])
R_W2H = R_W2H.reshape((2, 2))
```

- world座標系からhorizontal frame(yaw回転のみを取り除いた、胴体中心の水平座標系)への回転行列
- read_code_02で読んだ`periodic_gait_generator.py`の`update_start_and_stop`と全く同じパターンの回転行列がここでも使われている(コード上は別々に書かれており、共通化されていない)

---

## 87〜105行：速度補正項(移動平均と目標速度)

```python
base_lin_vel_H = R_W2H @ base_xy_lin_vel
ref_base_lin_vel_H = R_W2H @ ref_base_xy_lin_vel

self.base_vel_hist.append(base_lin_vel_H)
base_vel_mvg = np.mean(list(self.base_vel_hist), axis=0)
# delta_ref_H = (self.stance_time / 2.) * base_vel_mvg

delta_ref_H = (self.stance_time / 2.0) * ref_base_lin_vel_H
delta_ref_H = np.clip(delta_ref_H, -self.hip_height * 1.5, self.hip_height * 1.5)
vel_offset = np.concatenate((delta_ref_H, np.zeros(1)))
```

- 実速度・目標速度をどちらもhorizontal frameへ変換する
- 実速度の方は`base_vel_hist`へ追加して移動平均`base_vel_mvg`を取る(直近0.04秒相当の平均)
- `delta_ref_H = (stance_time/2) * 目標速度`という、Raibertヒューリスティックの基本形
  - 直感的な意味：「次の接地期間の半分の間、目標速度で進むとしたら、着地点は今の位置からどれだけ前にあるべきか」という単純な予測
  - コメントアウトされた行(100行目)を見ると、以前は実速度の移動平均でこの補正を計算していたが、今は目標速度ベースに変更されている(過去の実装の名残がコメントとして残っている)
- `np.clip(..., -hip_height*1.5, hip_height*1.5)`：補正量が大きくなりすぎないよう、股関節高さの1.5倍を上限としてクリップする
- `vel_offset = np.concatenate((delta_ref_H, np.zeros(1)))`：`delta_ref_H`はx,yの2要素しかないので、Z成分として`0`を1個追加し、3要素のベクトルにしている(このあと`ref_feet`という3次元の座標へそのまま加算するため)

```python
error_compensation = np.sqrt(com_height_nominal / self.gravity_constant) * (base_vel_mvg - ref_base_lin_vel_H)
error_compensation = np.where(error_compensation > 0.05, 0.05, error_compensation)
error_compensation = np.where(error_compensation < -0.05, -0.05, error_compensation)
error_compensation = np.concatenate((error_compensation, np.zeros(1)))
```

- $\sqrt{h/g}$という係数は、倒立振子の固有周期に由来する古典的なRaibert則の形
- 括弧の中が`base_vel_mvg - ref_base_lin_vel_H`、つまり「実測平均 − 目標」という順序になっている
  - 教科書的なRaibert則の説明では「目標 − 実測」という順で書かれることが多く、この実装は符号が逆になっている
  - 符号がどちらであるべきかは着地点の動く向きの解釈次第であり、一概に「誤り」とは言えないが、他の資料と比較する際は符号の取り違えに注意が必要
- `np.where(...)`で上下`±0.05m`にクリップ
- `error_compensation = np.concatenate((error_compensation, np.zeros(1)))`：こちらも同様に、x,yの2要素にZ成分として`0`を追加して3要素にしている

---

## 113〜132行：股関節基準の着地点計算

```python
ref_feet = LegsAttr(*[np.zeros(3) for _ in range(4)])

ref_feet.FL[0:2] = R_W2H @ (hips_position.FL[0:2] - base_position[0:2])
ref_feet.FR[0:2] = R_W2H @ (hips_position.FR[0:2] - base_position[0:2])
ref_feet.RL[0:2] = R_W2H @ (hips_position.RL[0:2] - base_position[0:2])
ref_feet.RR[0:2] = R_W2H @ (hips_position.RR[0:2] - base_position[0:2])
```

- 4本の股関節位置を、胴体中心のhorizontal frameへ変換する。これが着地点計算の出発点になる

```python
ref_feet.FL[1] += self.hip_offset
ref_feet.FR[1] -= self.hip_offset
ref_feet.RL[1] += self.hip_offset
ref_feet.RR[1] -= self.hip_offset
```

- 左脚(FL, RL)はY方向に`+hip_offset`、右脚(FR, RR)は`-hip_offset`
- コメント(121〜125行)で「Y軸方向のオフセットは左右の広さ、X軸方向のオフセットは前後の開き/交差に影響する。本来ハードコードすべきではなく、ロボット設定のプロパティにすべき」と、44行目の`hip_offset=0.1`と同じ問題意識が繰り返し書かれている

```python
ref_feet += vel_offset + error_compensation
```

- ここまでで計算した「速度に応じた前後方向の補正(`vel_offset`)」と「速度誤差の補正(`error_compensation`)」を、4本まとめて一度に加算する
- `LegsAttr`が`+`演算子をサポートしている(4脚まとめてブロードキャストされる)ことがここから読み取れる

---

## 134〜151行：world座標系への変換と最終的なZ座標の決定

```python
ref_feet.FL[0:2] = R_W2H.T @ ref_feet.FL[:2] + base_position[0:2]
ref_feet.FR[0:2] = R_W2H.T @ ref_feet.FR[:2] + base_position[0:2]
ref_feet.RL[0:2] = R_W2H.T @ ref_feet.RL[:2] + base_position[0:2]
ref_feet.RR[0:2] = R_W2H.T @ ref_feet.RR[:2] + base_position[0:2]
```

- horizontal frameで計算した結果を、回転行列の転置(`R_W2H.T`、逆回転に相当)でworld座標系へ戻す

```python
R_B2W = Rotation.from_euler("xyz", base_ori_euler_xyz).as_matrix()
self.com_pos_offset_w = R_B2W @ self.com_pos_offset_b
ref_feet.FL[0:2] += self.com_pos_offset_w[0:2]
...
```

- 31〜33行目で見た「HACK」の重心オフセットを、ここで実際に着地点へ加算している
- `self.com_pos_offset_b`はデフォルトでは全ゼロ(コンストラクタで初期化されたまま変更されていなければ)なので、この補正自体は既定では効果を持たない可能性が高い(この値がどこか他の場所で更新されるかどうかは本ファイルの範囲では未確認)

```python
for leg_id in ['FL', 'FR', 'RL', 'RR']:
    ref_feet[leg_id][2] = self.lift_off_positions[leg_id][2]
```

- X,Y座標はここまでの計算結果を使うが、**Z座標(高さ)だけは`self.lift_off_positions`(離地位置)のZ成分でそのまま上書きする**
- ここが重要な点：コンストラクタで用意されていた`self.touch_down_positions`(着地位置の記録)は、**この関数の中では一度も参照されていない**
  - 着地点の高さは「最後に離陸したときの高さ」を仮定しており、地形の起伏(次に着地する場所が今と違う高さかもしれない)は考慮されていない
  - 直後のコメント(148〜149行)にも「地形推定を考慮して回転させるべき、あるいは外部センサによる高さ調整をすべき」と、これも未実装のまま残っていることが明記されている

```python
self.last_reference_footholds = copy.deepcopy(ref_feet)
return ref_feet
```

- 計算結果を`last_reference_footholds`(可視化・記録用)にも保存してから返す

---

## 159〜178行：`update_lift_off_positions`(離地位置の追跡)

## simulation.py との結びつき

呼び出し連鎖は冒頭と同じで、`wb_interface.py::update_state_and_reference`の中から、`compute_footholds_reference`より**先に**呼ばれます。

この関数の役割:接地→遊脚に切り替わった脚の離地位置を記録し、遊脚中はworld座標を更新し続ける。

```python
def update_lift_off_positions(
    self, previous_contact, current_contact, feet_pos, legs_order, gait_type, base_position, base_ori_euler_xyz
):
    yaw = base_ori_euler_xyz[2]
    R_W2H = np.array([np.cos(yaw), np.sin(yaw), 0, -np.sin(yaw), np.cos(yaw), 0, 0, 0, 1])
    R_W2H = R_W2H.reshape((3, 3))
```

- 引数の意味(すべてデフォルト値はなく、呼び出し元が毎回渡す):
  - `previous_contact`/`current_contact`：4脚分の0/1配列(無次元)。1つ前の周期と今の周期の接地状態(read_code_02の`PeriodicGaitGenerator`が計算した`contact_sequence`の列0に由来)
  - `feet_pos`：4脚の現在の実際の足位置(m、world座標系。離地位置ではない)
  - `legs_order`：脚名の並び順のタプル(文字列)
  - `gait_type`：現在の歩容タイプ(`GaitType`列挙型。`FULL_STANCE`かどうかの判定に使う)
  - `base_position`：胴体の現在位置(m、world座標系)
  - `base_ori_euler_xyz`：胴体の現在の姿勢(rad)
- ここでの回転行列は`3×3`(Z成分も含む)。53〜85行の`compute_footholds_reference`内の回転行列は`2×2`だったので、同じ「horizontal frameへの変換」という目的でも、関数によって次元の異なる回転行列が個別に定義されている(共通化されていない)

```python
for leg_id, leg_name in enumerate(legs_order):
    if gait_type == GaitType.FULL_STANCE.value:
        self.lift_off_positions[leg_name] = feet_pos[leg_name]
        continue

    if previous_contact[leg_id] == 1 and current_contact[leg_id] == 0:
        self.lift_off_positions[leg_name] = feet_pos[leg_name]
        self.lift_off_positions_h[leg_name] = R_W2H @ (self.lift_off_positions[leg_name] - base_position)

    elif previous_contact[leg_id] == 0 and current_contact[leg_id] == 0:
        self.lift_off_positions[leg_name] = R_W2H.T @ self.lift_off_positions_h[leg_name] + base_position
```

- `FULL_STANCE`(全脚接地)のときは、単純に今の足位置をそのまま離地位置として記録する
- 通常時、脚ごとに3つの場合分け:
  - **接地→遊脚の瞬間**(`1→0`)：その瞬間の足位置を離地位置として記録し、同時にhorizontal frame版(`_h`サフィックス)も計算して保存する
  - **遊脚が継続中**(`0→0`)：離地位置そのものは変えず、保存しておいたhorizontal frame版を**現在の胴体位置・向き**で再投影して、world座標系の値を更新する
  - 接地が継続中(`1→1`)：どちらの分岐にも当てはまらないため、**何もしない**(このブロックには存在しない)

- 遊脚が継続中の再投影処理には注意が要る:
  - horizontal frame版の値は、離地した瞬間の`base_position`を基準に計算されている
  - それを毎ステップ、**そのときどきの最新の`base_position`**で world 座標へ戻している
  - つまり地面についていない間、記録された離地位置は、胴体の動きに合わせてworld座標上で位置がわずかに動き続ける可能性がある
  - これは物理的には奇妙に見えるが、状態推定(ベース位置の推定値)が毎ステップ更新されることを踏まえると、「離地位置を、最新の状態推定と整合する形で保ち続けるための補正」という設計だと解釈すれば筋が通る(**設計上の解釈**、確証はない)

---

## 180〜199行：`update_touch_down_positions`(着地位置の追跡、ただし未使用)

この関数の役割:遊脚→接地に切り替わった脚の着地位置を記録する(`update_lift_off_positions`と対称)。

```python
def update_touch_down_positions(
    self, previous_contact, current_contact, feet_pos, legs_order, gait_type, base_position, base_ori_euler_xyz
):
    ...
    for leg_id, leg_name in enumerate(legs_order):
        if gait_type == GaitType.FULL_STANCE.value:
            self.touch_down_positions[leg_name] = feet_pos[leg_name]
            continue

        if previous_contact[leg_id] == 0 and current_contact[leg_id] == 1:
            self.touch_down_positions[leg_name] = feet_pos[leg_name]
            self.touch_down_positions_h[leg_name] = R_W2H @ (self.touch_down_positions[leg_name] - base_position)

        elif previous_contact[leg_id] == 1 and current_contact[leg_id] == 1:
            self.touch_down_positions[leg_name] = R_W2H.T @ self.touch_down_positions_h[leg_name] + base_position
```

- `update_lift_off_positions`と完全に対称な構造:「遊脚→接地の瞬間」(`0→1`)で記録し、「接地が継続中」(`1→1`)で再投影する
- **実装上の問題点**:この関数は`wb_interface.py`から呼ばれてはいるものの(冒頭の呼び出し連鎖参照)、その計算結果である`self.touch_down_positions`は、`compute_footholds_reference`(このクラスのメイン処理)の中では一度も読まれていません(151行目で使われるのは`self.lift_off_positions`だけ)
  - つまりこの関数は「毎ステップ計算だけはされるが、結果はこのクラス内では使われない」状態になっている

---

## 202〜229行：`__main__`ブロック(呼ばれない、保留)

```python
if __name__ == "__main__":
    m = mujoco.MjModel.from_xml_path("./../simulation/unitree_go1/scene.xml")
    ...
    foothold_generator = FootholdReferenceGenerator(stance_time)
    footholds_reference = foothold_generator.compute_footholds_reference(
        linear_com_velocity[0:2], desired_linear_com_velocity[0:2], hip_pos, com_height
    )
```

- このファイルを直接実行したときだけ動く、単体テスト的なコード
- `simulation.py`からの呼び出し連鎖には一切含まれない(`import`もされない)ため、重要度は低いと判断し、詳細な解説は保留とします
- 参考までに一点だけ：ここでの`FootholdReferenceGenerator(stance_time)`という呼び出しは、現在のコンストラクタのシグネチャ(`stance_time, lift_off_positions, ...`)と引数の数が合っておらず、**このテストコード自体は現在のクラス定義のままでは動かない**古い呼び出し方になっています。ただし呼ばれない経路なので実害はありません。

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `lift_off_positions_h`/`touch_down_positions_h`の初期化に`# TODO wrong`という開発者自身のコメントが残っている(未修正のバグの可能性)
  2. `touch_down_positions`は計算され続けるが、`compute_footholds_reference`の中では一度も使われない
  3. `hip_offset=0.1`がハードコードされており、ロボットごとの設定にすべきというTODOが残ったまま
  4. 着地点のZ座標(高さ)は地形を考慮せず、常に「最後に離陸した高さ」がそのまま使われる
  5. `__main__`ブロックのテストコードが、現在のコンストラクタのシグネチャと合っておらず、実行できない状態で放置されている
- 呼び出し連鎖の中での位置づけ:歩容(read_code_02、いつ接地するか)の次に、着地点(どこに接地するか)を決めるのがこのファイル。次はいよいよ、これらの計画を実際の状態と組み合わせて`state_current`/`ref_state`を組み立てる`wb_interface.py`の`update_state_and_reference`(および、まだ読んでいないもう1つのサブ、`terrain_estimator.py`)に進みます。
