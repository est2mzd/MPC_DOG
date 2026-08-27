# 単体プロセス版② quadruped_pympc/quadruped_pympc_wrapper.py 逐次解説

対象は `external/Quadruped-PyMPC/quadruped_pympc/quadruped_pympc_wrapper.py`(259行)です。

このファイルを選んだ理由は「上流からの流れ」です。[read_code_01](read_code_01_simulation_py.md)の207〜236行で、`simulation.py`は`quadrupedpympc_wrapper.compute_actions(...)`を呼んでいました。実行順序として、その次に読むべきコードはこのファイルになります。ランダムに別の章(MPC内部やROS2版)へ飛ぶのではなく、実際の呼び出しの流れをそのまま辿ります。

---

## 1〜9行：import と既定の観測名

```python
import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as cfg
from quadruped_pympc.interfaces.srbd_batched_controller_interface import SRBDBatchedControllerInterface
from quadruped_pympc.interfaces.srbd_controller_interface import SRBDControllerInterface
from quadruped_pympc.interfaces.wb_interface import WBInterface
```

- `SRBDControllerInterface`：MPC(OCP)の呼び出し窓口。read_code_01で読んだ`simulation.py`は一度も直接importしていなかったクラス
- `SRBDBatchedControllerInterface`：歩容周波数の最適化(`optimize_step_freq`)専用の別インターフェース。既定ではほぼ使われない(後述)
- `WBInterface`：歩容生成・着地点生成・状態集約・トルク変換をまとめたクラス
- つまりこのファイルは、これら3つのクラスをすべて生成し、束ねているだけの「薄いラッパー」

```python
_DEFAULT_OBS = ("ref_base_height", "ref_base_angles", "nmpc_GRFs", "nmpc_footholds", "swing_time")
```

- `compute_actions`が記録・可視化用に保存する観測名の既定値
- read_code_01で見た`simulation.py`側は独自に8項目のタプルを作って渡していたため、この既定値は既定実行では**使われない**(呼び出し側が明示的に上書きしている)

---

## 12〜28行：クラス定義とコンストラクタのシグネチャ

```python
class QuadrupedPyMPC_Wrapper:
    """A simple class wrapper of all the mpc submodules (swing, contact generator, mpc itself)."""

    def __init__(
        self,
        initial_feet_pos: LegsAttr,
        legs_order: tuple[str, str, str, str] = ('FL', 'FR', 'RL', 'RR'),
        feet_geom_id: LegsAttr = None,
        quadrupedpympc_observables_names: tuple[str, ...] = _DEFAULT_OBS,
    ):
```

- クラスのdocstring自体が「MPCのサブモジュール(スイング、接触生成器、MPC本体)をまとめた単純なラッパー」と説明している
- `initial_feet_pos: LegsAttr`という型注釈がついているが、read_code_01で確認した通り`simulation.py`側は実際には`env.feet_pos`という**関数**を渡している
  - 型注釈と実際に渡されるものが一致していない箇所
- `feet_geom_id: LegsAttr = None`：read_code_01の119〜139行の節で見た`env._feet_geom_id`(内部用属性)がここに渡ってくる引数

---

## 30〜37行：`__init__`本体、制御ロジック3クラスの生成

```python
self.mpc_frequency = cfg.simulation_params["mpc_frequency"]

self.srbd_controller_interface = SRBDControllerInterface()

if cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['optimize_step_freq']:
    self.srbd_batched_controller_interface = SRBDBatchedControllerInterface()

self.wb_interface = WBInterface(initial_feet_pos=initial_feet_pos(frame='world'), legs_order=legs_order, feet_geom_id=feet_geom_id)
```

- `self.mpc_frequency = cfg.simulation_params["mpc_frequency"]`
  - MPCの計算周波数(既定100Hz)をここで取得
  - read_code_01の44〜49行の節で「500Hzのうち5ステップに1回だけMPCを解く」と予告していた仕組みの元になる値
- `self.srbd_controller_interface = SRBDControllerInterface()`
  - MPC(OCP)を呼び出す本体。引数なしで生成される(内部で`config.py`から`mpc_params['type']`を読んで、`nominal`/`sampling`等のどの実装を使うか自分で決める)
- `if ... optimize_step_freq: self.srbd_batched_controller_interface = ...`
  - `optimize_step_freq`が有効なときだけ、歩行周波数を最適化する別インターフェースを追加で生成する
  - 既定では`optimize_step_freq=False`と考えられるため、この行自体が実行されない可能性が高い(`config.py`の実値は本パスでは未確認)
- `self.wb_interface = WBInterface(initial_feet_pos=initial_feet_pos(frame='world'), ...)`
  - ここで前節の疑問が解消する：`initial_feet_pos`という引数(実体は関数)を`(frame='world')`付きで**呼び出して**いる
  - つまりコンストラクタは「関数を受け取って、自分で呼び出して値に変換する」という設計を前提にしている

---

## 39〜48行：出力用変数の初期化

```python
self.nmpc_GRFs = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_footholds = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_joints_pos = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_joints_vel = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_joints_acc = LegsAttr(FL=np.zeros(3), FR=np.zeros(3), RL=np.zeros(3), RR=np.zeros(3))
self.nmpc_predicted_state = np.zeros(12)
self.best_sample_freq = self.wb_interface.pgg.step_freq
```

- MPCが計算する値(GRF・着地点・関節位置/速度/加速度・予測状態・最適歩行周波数)を、すべてゼロで初期化して`self`に保持しておく
- これらは**毎周期MPCが更新するとは限らない**値である点が重要
  - 後述の134行目で、MPCは`simulation_dt`ごとではなく間引かれた頻度でしか解かれない
  - 解かれない周期では、この`self.nmpc_GRFs`等の**古い値がそのまま使い回される**
  - つまりこのクラスが「前回の解を次のトルク計算まで保持しておく」というキャッシュの役割も兼ねている
- `self.nmpc_predicted_state = np.zeros(12)`
  - 12次元。MPCが内部で扱うOCPの状態は実際には30次元(重心位置3+重心速度3+姿勢角3+角速度3+4脚の足位置12+積分項6)だが、この12はその一部(重心位置3+重心速度3+姿勢角3+角速度3)だけを保持する設計だと考えられる

```python
self.quadrupedpympc_observables_names = quadrupedpympc_observables_names
self.quadrupedpympc_observables = {}
```

- 記録・可視化用の観測名リストと、実際の値を入れる空辞書を用意する
- この辞書は`get_obs()`(247〜253行)経由で外部(`simulation.py`)から読まれる

---

## 50〜79行：`compute_actions`のシグネチャとdocstring

```python
def compute_actions(
    self,
    com_pos: np.ndarray,
    base_pos: np.ndarray,
    base_lin_vel: np.ndarray,
    base_ori_euler_xyz: np.ndarray,
    base_ang_vel: np.ndarray,
    feet_pos: LegsAttr,
    hip_pos: LegsAttr,
    joints_pos: LegsAttr,
    heightmaps,
    legs_order: tuple[str, str, str, str],
    simulation_dt: float,
    ref_base_lin_vel: np.ndarray,
    ref_base_ang_vel: np.ndarray,
    step_num: int,
    qpos: np.ndarray,
    qvel: np.ndarray,
    feet_jac: LegsAttr,
    feet_jac_dot: LegsAttr,
    feet_vel: LegsAttr,
    legs_qfrc_passive: LegsAttr,
    legs_qfrc_bias: LegsAttr,
    legs_mass_matrix: LegsAttr,
    legs_qpos_idx: LegsAttr,
    legs_qvel_idx: LegsAttr,
    tau: LegsAttr,
    inertia: np.ndarray,
    mujoco_contact: np.ndarray,
) -> LegsAttr:
```

- 引数の並び順はread_code_01で確認した`simulation.py`側の呼び出し(207〜236行)と1対1で対応している
- docstring(83〜107行)を実際に読むと、いくつか不完全な記載がある:
  - `com_pos (np.ndarray): center of mass position in` — 文が途中で切れている(「world frameで」等が続くはずが未記載)
  - `heightmaps (_type_): TODO` — 型も説明も未記入のプレースホルダのまま
  - `feet_vel`、`legs_qpos_idx`、`mujoco_contact`：docstringの引数リストに**記載自体がない**(実引数には存在するのに)
- コード自体は動作するが、docstringが実装に追いついていない箇所が複数ある

---

## 113〜131行：状態と参照値の更新(`WBInterface`の呼び出し)

```python
state_current, ref_state, contact_sequence, step_height, optimize_swing = (
    self.wb_interface.update_state_and_reference(
        com_pos, base_pos, base_lin_vel, base_ori_euler_xyz, base_ang_vel,
        feet_pos, hip_pos, joints_pos, heightmaps, legs_order, simulation_dt,
        ref_base_lin_vel, ref_base_ang_vel, mujoco_contact,
    )
)
```

- ここで初めて`WBInterface`(歩容・着地点・状態集約を担当するクラス)が呼ばれる
- 受け取った5つの戻り値:
  - `state_current`：現在の状態を表す辞書(重心位置・速度・姿勢・角速度・4脚の足位置など)
  - `ref_state`：目標(参照)状態の辞書(目標速度・目標姿勢・目標着地点など)
  - `contact_sequence`：これから先のホライズン分の接地スケジュール(0/1の配列)
  - `step_height`：スイング(遊脚)の目標ステップ高さ
  - `optimize_swing`：歩行周波数最適化のタイミングを知らせるフラグ
- この関数の内部(歩容の位相更新、着地点のRaibert則計算、地形推定など)は本ファイルの範囲では展開されない。次に読むべきファイルは`interfaces/wb_interface.py`になる

---

## 133〜156行：OCPを解く(間引き処理とRTI)

```python
if step_num % round(1 / (self.mpc_frequency * simulation_dt)) == 0:
```

- ここがread_code_01で予告していた「500Hzのうち5ステップに1回だけMPCを解く」という間引き処理の実体
- 数値で確認すると、既定値`mpc_frequency=100`、`simulation_dt=0.002`のとき

$$
\frac{1}{100 \times 0.002} = \frac{1}{0.2} = 5
$$

- つまり`step_num`が5の倍数のときだけ、この`if`ブロックの中身(MPCを解く処理)が実行される
- 5で割り切れない残り4ステップでは、このブロックはスキップされ、39〜44行で初期化した`self.nmpc_GRFs`等の**前回の値がそのまま使われ続ける**

```python
(
    self.nmpc_GRFs,
    self.nmpc_footholds,
    self.nmpc_joints_pos,
    self.nmpc_joints_vel,
    self.nmpc_joints_acc,
    self.best_sample_freq,
    self.nmpc_predicted_state,
) = self.srbd_controller_interface.compute_control(
    state_current, ref_state, contact_sequence, inertia,
    self.wb_interface.pgg.phase_signal, self.wb_interface.pgg.step_freq,
    optimize_swing,
)
```

- MPCを解いた結果(GRF・着地点・関節の予測値・予測状態)を、すべて`self.`のインスタンス変数へ上書きする
- この関数の内部(acadosのOCPを実際に解く処理)は本ファイルの範囲では展開されない。次に読むべきファイルは`interfaces/srbd_controller_interface.py`になる

```python
if cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['use_RTI']:
    self.srbd_controller_interface.compute_RTI()
```

- 勾配ベース(acados)のMPCで、かつ`use_RTI`(Real-Time Iteration)が有効なときだけ、追加でもう1回`compute_RTI()`を呼ぶ
- コメントによれば、これは「次回の状態→制御の遅延を最小化するため、計算後にMPCを事前線形化しておく」という処理
- OCPを解く処理そのものの中身と同様、次に読むべきファイルの範囲になる

---

## 158〜169行：歩行周波数の最適化(既定では動かない経路)

```python
if cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['optimize_step_freq']:
    self.best_sample_freq = self.srbd_batched_controller_interface.optimize_gait(
        state_current, ref_state, inertia,
        self.wb_interface.pgg.phase_signal, self.wb_interface.pgg.step_freq,
        self.wb_interface.pgg.duty_factor, self.wb_interface.pgg.gait_type,
        optimize_swing,
    )
```

- `optimize_step_freq`が有効なときだけ、複数の候補歩行周波数を並列に評価するバッチ処理(`SRBDBatchedControllerInterface`)を呼ぶ
- この条件は34行目の`self.srbd_batched_controller_interface`の生成条件と同じ
  - もし`config.py`の値が実行中に変わらない前提が崩れると(通常はあり得ないが)、インスタンスが存在しないのにこの行が実行されてエラーになる、という依存関係になっている
- 既定では`optimize_step_freq=False`と考えられるため、このブロック自体が実行されない可能性が高い(config.py側の実値は未確認)

---

## 171〜195行：立脚・遊脚トルクの計算

```python
tau, des_joints_pos, des_joints_vel = self.wb_interface.compute_stance_and_swing_torque(
    simulation_dt, qpos, qvel, feet_jac, feet_jac_dot, feet_pos, feet_vel,
    legs_qfrc_passive, legs_qfrc_bias, legs_mass_matrix,
    self.nmpc_GRFs, self.nmpc_footholds,
    legs_qpos_idx, legs_qvel_idx, tau, optimize_swing, self.best_sample_freq,
    self.nmpc_joints_pos, self.nmpc_joints_vel, self.nmpc_joints_acc,
    self.nmpc_predicted_state, mujoco_contact,
)
```

- ここで初めて、力(`self.nmpc_GRFs`)が実際の関節トルクへ変換される
- 引数`tau`(この関数の引数として渡されてきた、`simulation.py`側で全ゼロ初期化されたもの)は、**そのまま`compute_stance_and_swing_torque`の中へ渡され、内部でその配列自体に値が書き込まれてから返ってくる**
  - つまり戻り値の`tau`を新しく作っているのではなく、渡した入れ物を使い回して書き込んでいる可能性が高い(`interfaces/wb_interface.py`側の実装を見るとこの推測が裏付けられる：`tau.FL = -np.matmul(...)`のように直接代入している)
  - この時点で左辺の`tau`(ローカル変数)は、関数引数として受け取った`tau`を**上書き**している。関数の外(`simulation.py`)からは同じ名前の変数がそのまま更新されて返ってくるように見える
- この関数の内部(立脚の`τ=-JᵀF`、遊脚のスイング軌道PD制御)も、本ファイルの範囲では展開されない。次に読むべきファイルは引き続き`interfaces/wb_interface.py`

---

## 197〜203行：コメントアウトされた関節PD制御

```python
kp_joint_motor = cfg.simulation_params['impedence_joint_position_gain']
kd_joint_motor = cfg.simulation_params['impedence_joint_velocity_gain']
# for leg in legs_order:
#    tau[leg] += kp_joint_motor * (des_joints_pos[leg] - qpos[legs_qpos_idx[leg]]) + \
#                kd_joint_motor * (des_joints_vel[leg] - qvel[legs_qvel_idx[leg]])
```

- コメント：「これらの値は通常、低レベルのモーターコントローラへ渡される。ここでシミュレートを試みることもできる」
- `kp_joint_motor`/`kd_joint_motor`は`config.py`から取得されるが、それを使うはずの`for`ループは丸ごとコメントアウトされている
- つまりこの2つの変数は**代入されるだけで、実際には一切使われていない**
  - read_code_01で見た`joints_pos`(実はインデックス配列)や、未使用の`state_obs_history`と同じ系統の、「宣言されているが実行経路に組み込まれていない」コードの一例
- 設計として読めるのは、「実機の低レベルコントローラが本来やる関節インピーダンス制御を、シミュレーション側でも試せるようにするためのフック」だが、現状は無効化されている

---

## 205〜243行：観測値の辞書組み立て

```python
self.quadrupedpympc_observables = {}
for obs_name in self.quadrupedpympc_observables_names:
    if obs_name == 'ref_base_height':
        data = {'ref_base_height': ref_state['ref_position'][2]}
    elif obs_name == 'ref_base_angles':
        data = {'ref_base_angles': ref_state['ref_orientation']}
    elif obs_name == 'ref_feet_pos':
        ...
    elif obs_name == 'nmpc_GRFs':
        data = {'nmpc_GRFs': self.nmpc_GRFs}
    elif obs_name == 'nmpc_footholds':
        data = {'nmpc_footholds': self.nmpc_footholds}
    elif obs_name == 'swing_time':
        data = {'swing_time': self.wb_interface.stc.swing_time}
    elif obs_name == 'phase_signal':
        data = {'phase_signal': self.wb_interface.pgg._phase_signal}
    elif obs_name == 'lift_off_positions':
        data = {'lift_off_positions': self.wb_interface.frg.lift_off_positions}
    else:
        data = {}
        raise ValueError(f"Unknown observable name: {obs_name}")
    self.quadrupedpympc_observables.update(data)
```

- read_code_01の119〜139行で見た`quadrupedpympc_observables_names`のタプルを1つずつループし、名前に応じた値を辞書へ詰めていく、というif/elifの羅列
- `else`節で未知の名前なら例外を投げる、というバリデーションになっている

`phase_signal`の行で気になる実装がある:

```python
data = {'phase_signal': self.wb_interface.pgg._phase_signal}
```

- 先頭に`_`が付いた**内部用属性**を直接読んでいる
- `interfaces/periodic_gait_generator.py`を見ると、これに対応する公開プロパティ`phase_signal`が別に用意されており、その中身は`return np.array(self._phase_signal)`、つまり**コピーを返す**実装になっている
- ここではその公開プロパティを使わず、`_phase_signal`(コピーではない、生の内部配列そのもの)を辞書に入れている
- この配列は毎周期`pgg.run()`の中で`self._phase_signal[leg] += dt * new_step_freq`という形でその場で書き換えられる(新しい配列を作り直すのではなく、既存配列の中身を更新する)
- 結果として、`get_obs()`経由でこの辞書を受け取った側が値を保持し続けると、**知らないうちに中身が後から変わってしまう**可能性がある(コピーではなく同じ配列への参照を持ってしまっているため)
- これは実装上の問題点の候補で、ログや可視化(read_code_01の描画ブロックのような使い方)に限れば大きな実害は出にくいが、値をそのまま保存して後で比較するような使い方をすると、意図しない値になりうる

```python
return tau
```

- このメソッドが最終的に返すのは`tau`のみ。`des_joints_pos`/`des_joints_vel`(171行目で受け取っていた)はこの後どこにも使われず、破棄される
- read_code_01で見た通り、これがそのまま`simulation.py`側の`tau`になり、トルククリップ(238〜240行)へ渡される

---

## 247〜253行：`get_obs`

```python
def get_obs(self) -> dict:
    """Get some user-defined observables from withing the control loop.

    Returns:
        Dict: dictionary of observables
    """
    return self.quadrupedpympc_observables
```

- `self.quadrupedpympc_observables`(直前のブロックで組み立てた辞書)をそのまま返すだけ
- read_code_01の253行目`ctrl_state = quadrupedpympc_wrapper.get_obs()`がこれを呼んでいた

---

## 255〜259行：`reset`

```python
def reset(self, initial_feet_pos: LegsAttr):
    """Reset the controller."""
    self.wb_interface.reset(initial_feet_pos)
    self.srbd_controller_interface.controller.reset()
```

- `WBInterface`側のリセット(歩容位相・着地点履歴など)と、MPCソルバー側のリセット(`acados_ocp_solver.reset()`)を両方呼ぶ
- read_code_01の318〜327行で見た、エピソード終了時の`quadrupedpympc_wrapper.reset(initial_feet_pos=env.feet_pos(frame="world"))`がこれを呼んでいた
- ここでも`initial_feet_pos`は「呼び出し結果の値」として渡されている(`__init__`のときのような「関数そのもの」ではない)点に注意。同じ名前の引数でも、呼び出し元によって「関数」なのか「値」なのかが変わる、という一貫性のない使われ方になっている

---

## この章のまとめ:次に読むべきファイル

このファイル自体は薄いラッパーで、実質的な計算はすべて次の2つのファイルに委譲されていました。

- `interfaces/wb_interface.py`(`update_state_and_reference`と`compute_stance_and_swing_torque`の中身)
- `interfaces/srbd_controller_interface.py`(`compute_control`と`compute_RTI`の中身、MPC本体の呼び出し窓口)

呼び出された順番でいえば、次は`update_state_and_reference`が最初に呼ばれているので、上流からの流れをそのまま辿るなら`interfaces/wb_interface.py`が次に読むべきファイルになります。
