# 状態集約 interfaces/wb_interface.py(update_state_and_referenceのみ)逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.wb_interface.update_state_and_reference(...)   ← 本ファイル
          → self.terrain_computation.compute_terrain_estimation(...)  (read_code_04で解説済み)
          → self.vm.modulate_velocities(...)                          (read_code_05で解説済み)
          → self.pgg.run(...), self.pgg.compute_contact_sequence(...) (read_code_02で解説済み)
          → self.frg.update_lift_off_positions(...) など            (read_code_03で解説済み)
```

`self.wb_interface`は`quadrupedpympc_wrapper.py`の`QuadrupedPyMPC_Wrapper.__init__`の中で`WBInterface(...)`として生成されます。呼び出し頻度は`simulation.py`の内側ループが1回まわるたびに毎回(既定500Hz相当)、MPCの間引き(5ステップに1回)とは無関係です。

## このファイル・このメソッドの役割(全体の中での位置づけ)

`WBInterface`は歩容・着地点・地形推定・速度変調・スイング軌道・逆運動学・早期接地検知という、複数のコンポーネントを束ねるクラスです。このクラスには大きく2つの仕事があります。

1. `update_state_and_reference`：ここまで読んできた上流の計画(read_code_02〜05)を全部集めて、MPCが直接読み込める形(`state_current`/`ref_state`/`contact_sequence`という3点セット)に**組み立てる**仕事
2. `compute_stance_and_swing_torque`：MPCが計算したGRFを実際の関節トルクへ**変換する**仕事(WBC)

この2つは制御パイプラインの中で全く違う位置にあります(1つ目はMPCより前、2つ目はMPCより後)。本ファイルでは**1つ目だけ**を扱います。2つ目は、MPC本体(`srbd_controller_interface.py`)を読み終えたあとに、別の章として読みます。

対象は `external/Quadruped-PyMPC/quadruped_pympc/interfaces/wb_interface.py`のうち、クラス定義・コンストラクタ・`update_state_and_reference`メソッド(1〜305行)です。

---

## 1〜21行：import

```python
import copy
import time

import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr
from scipy.spatial.transform import Rotation as R

from quadruped_pympc import config as cfg
from quadruped_pympc.helpers.foothold_reference_generator import FootholdReferenceGenerator
from quadruped_pympc.helpers.inverse_kinematics.inverse_kinematics_numeric_mujoco import InverseKinematicsNumeric
from quadruped_pympc.helpers.periodic_gait_generator import PeriodicGaitGenerator
from quadruped_pympc.helpers.swing_trajectory_controller import SwingTrajectoryController
from quadruped_pympc.helpers.terrain_estimator import TerrainEstimator
from quadruped_pympc.helpers.velocity_modulator import VelocityModulator
from quadruped_pympc.helpers.early_stance_detector import EarlyStanceDetector

if cfg.simulation_params['visual_foothold_adaptation'] != 'blind':
    from quadruped_pympc.helpers.visual_foothold_adaptation import VisualFootholdAdaptation
```

- `PeriodicGaitGenerator`・`FootholdReferenceGenerator`・`TerrainEstimator`・`VelocityModulator`：本章で扱う4つのコンポーネント、それぞれread_code_02〜05で読んだクラス
- `SwingTrajectoryController`・`InverseKinematicsNumeric`・`EarlyStanceDetector`：`compute_stance_and_swing_torque`(WBC側)でのみ使われるクラス。本章では生成される様子だけ確認し、中身は別の章に回す
- `VisualFootholdAdaptation`は、`visual_foothold_adaptation`が`"blind"`でないときだけimportされる。既定は`"blind"`なので、この行自体が実行されないのが標準的な状態

---

## 22〜42行：クラス定義とコンストラクタの前半

```python
class WBInterface:
    """
    WBInterface is responsible for interfacing with the whole body controller of a quadruped robot.
    It initializes the necessary components for motion planning and control, including gait generation,
    swing trajectory control, and terrain estimation.
    """

    def __init__(self,
                 initial_feet_pos: LegsAttr,
                 legs_order: tuple[str, str, str, str] = ('FL', 'FR', 'RL', 'RR'),
                 feet_geom_id : LegsAttr = None):
```

- クラスdocstringが「歩容生成・スイング軌道制御・地形推定を含む、モーションプランニングと制御に必要なコンポーネントを初期化する」役割だと説明している。実際には着地点生成・速度変調・逆運動学・早期接地検知も含まれており、docstringの説明は網羅的ではない
- 引数の意味:
  - `initial_feet_pos`：初期の足位置(m、`LegsAttr`)。デフォルト値はなく必須引数
  - `legs_order`：脚名の並び順のタプル。デフォルト`('FL', 'FR', 'RL', 'RR')`
  - `feet_geom_id`：MuJoCo上の足geomのID(`LegsAttr`)。デフォルト`None`

```python
        mpc_dt = cfg.mpc_params['dt']
        horizon = cfg.mpc_params['horizon']
        self.legs_order = legs_order

        gait_name = cfg.simulation_params['gait']
        gait_params = cfg.simulation_params['gait_params'][gait_name]
        gait_type, duty_factor, step_frequency = (
            gait_params['type'],
            gait_params['duty_factor'],
            gait_params['step_freq'],
        )
        self.pgg = PeriodicGaitGenerator(
            duty_factor=duty_factor, step_freq=step_frequency, gait_type=gait_type, horizon=horizon
        )
```

- `mpc_dt`：OCPの内部離散化の時間刻み(秒)。`config.py`の`mpc_params['dt']`は既定`0.02`
- `horizon`：OCPの予測ホライズンのステップ数(無次元)。`config.py`の`mpc_params['horizon']`は既定`12`(=0.24秒先まで予測)
- `config.py`の`gait_params`辞書から、選択された歩容名(`gait_name`)に対応する`type`・`duty_factor`・`step_freq`を取り出し、`PeriodicGaitGenerator`(read_code_02)を生成する
- ここでようやく、read_code_02で見た`duty_factor`(既定`0.65`)・`step_freq`(既定`1.4`)が、実際に`config.py`のどこから来るかが分かる

```python
        if cfg.mpc_params['use_nonuniform_discretization']:
            self.contact_sequence_dts = [cfg.mpc_params['dt_fine_grained'], mpc_dt]
            self.contact_sequence_lenghts = [cfg.mpc_params['horizon_fine_grained'], horizon]
        else:
            self.contact_sequence_dts = [mpc_dt]
            self.contact_sequence_lenghts = [horizon]
```

- 非一様な時間刻み(ホライズンの前半だけ細かく刻む、等)を使うかどうかで、`contact_sequence_dts`/`contact_sequence_lenghts`(read_code_02の`compute_contact_sequence`が使っていた引数)を組み立てる

---

## 65〜106行：コンストラクタの後半(残りのコンポーネント生成)

```python
        stance_time = (1 / self.pgg.step_freq) * self.pgg.duty_factor
        self.frg = FootholdReferenceGenerator(
            stance_time=stance_time, hip_height=cfg.hip_height, lift_off_positions=initial_feet_pos
        )
```

- `stance_time = (1/step_freq) * duty_factor`：1周期の時間(`1/step_freq`)に接地割合(`duty_factor`)を掛けて、接地期の実時間(秒)を計算する
- 数値例：既定値`step_freq=1.4`、`duty_factor=0.65`なら、$stance\_time = (1/1.4) \times 0.65 \approx 0.464$秒
- この`stance_time`が、read_code_03で読んだ`FootholdReferenceGenerator.compute_footholds_reference`の`delta_ref_H = (stance_time/2) * ref_vel`という式の`stance_time`の出どころ

```python
        self.step_height = cfg.simulation_params['step_height']
        swing_period = (1 - self.pgg.duty_factor) * (1 / self.pgg.step_freq)
        ...
        self.stc = SwingTrajectoryController(...)
        self.last_des_foot_pos = LegsAttr(*[np.zeros((3,)) for _ in range(4)])

        self.terrain_computation = TerrainEstimator()
        self.ik = InverseKinematicsNumeric()

        if cfg.simulation_params['visual_foothold_adaptation'] != 'blind':
            self.vfa = VisualFootholdAdaptation(...)

        self.vm = VelocityModulator()
        self.esd = EarlyStanceDetector(feet_geom_id)

        self.current_contact = np.array([1, 1, 1, 1])
        self.previous_contact = np.array([1, 1, 1, 1])
```

- `self.step_height`：スイング軌道の目標高さ(m)。`config.py`の`simulation_params['step_height']`は`0.2 * hip_height`で計算され、Go2(`hip_height=0.28`)なら`0.056`m
- `swing_period = (1-duty_factor) * (1/step_freq)`：`stance_time`の逆(遊脚に使う時間、秒)。数値例：$(1-0.65)\times(1/1.4)=0.25$秒
- `self.current_contact`/`self.previous_contact`：4脚分の0/1配列(無次元)。初期値はどちらも`[1,1,1,1]`(4本とも接地)
- `self.stc`(`SwingTrajectoryController`)・`self.ik`(`InverseKinematicsNumeric`)・`self.esd`(`EarlyStanceDetector`)は、ここで生成だけ確認し、中身はWBCの章で読みます
- `self.terrain_computation = TerrainEstimator()`(read_code_04)、`self.vm = VelocityModulator()`(read_code_05)がここで生成される
- `current_contact`/`previous_contact`をどちらも「4本とも接地」で初期化する

---

## 108〜150行：`update_state_and_reference`の入口

```python
    def update_state_and_reference(
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
        mujoco_contact: np.ndarray = None,
    ) -> [dict, dict, list, LegsAttr, list, list, float, bool]:
```

- 引数の意味(すべてデフォルト値はなく、呼び出し元が毎回渡す。`mujoco_contact`のみデフォルト`None`):
  - `com_pos`：重心位置(m、world座標系)
  - `base_pos`：胴体位置(m、world座標系)
  - `base_lin_vel`：胴体並進速度(m/s、world座標系)
  - `base_ori_euler_xyz`：胴体姿勢(rad)
  - `base_ang_vel`：胴体角速度(rad/s、base座標系)
  - `feet_pos`：4脚の位置(m、world座標系)
  - `hip_pos`：4脚の股関節位置(m、world座標系)
  - `joints_pos`：関節の値(実体は`legs_qvel_idx`、インデックス配列。read_code_01参照)
  - `heightmaps`：4脚分のHeightMapオブジェクト。既定(`"blind"`)では`None`
  - `legs_order`：脚名の並び順のタプル
  - `simulation_dt`：シミュレーションの時間刻み(秒)。既定`0.002`
  - `ref_base_lin_vel`：目標並進速度(m/s)
  - `ref_base_ang_vel`：目標角速度(rad/s)
  - `mujoco_contact`：MuJoCoの接触情報。デフォルト`None`
- 引数の並びは、呼び出し元(`quadrupedpympc_wrapper.py`の`compute_actions`)からそのまま渡されてくる観測一式と一致する
- 戻り値の型注釈`[dict, dict, list, LegsAttr, list, list, float, bool]`は要素が8個だが、実際の`return`文(305行目)は5個の値しか返していない。型注釈と実装が一致していない箇所

---

## 152〜160行：地形推定の呼び出しと、無効化されたコード

```python
        terrain_roll, terrain_pitch, terrain_height, robot_height = self.terrain_computation.compute_terrain_estimation(
            base_position=base_pos,
            yaw=base_ori_euler_xyz[2],
            feet_pos=self.frg.lift_off_positions,
            current_contact=self.current_contact,
        )
        #base_pos[2] = robot_height
        #com_pos[2] = robot_height #TODO, this is an error
```

- read_code_04で解説した`TerrainEstimator`をここで呼んでいる。`feet_pos`引数には実際の足位置ではなく`self.frg.lift_off_positions`(離地位置)を渡している点は、read_code_04で述べた通り
- **注目すべき箇所**：直後の2行がコメントアウトされている。もし有効だったら、`base_pos[2]`と`com_pos[2]`(胴体・重心のZ座標)を、地形推定から得た`robot_height`で強制的に上書きする、という処理になっていた
- 2行目には開発者自身のコメントで`#TODO, this is an error`(これはバグだ)と明記されている
- つまりこれは私の推測ではなく、**開発者自身が「誤りだ」と認めて無効化したコード**。今は無効なので実害はないが、コードの履歴として「一度は重心の高さを地形推定値で上書きしようとして、誤りだと気づいて取り消した」ことが分かる

---

## 163〜177行：`state_current`辞書の組み立て

```python
        state_current = dict(
            position=com_pos + self.frg.com_pos_offset_w,  # manual com offset
            # position=base_pos,
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

- `position`(m、world座標系)は`com_pos`(重心位置)に、read_code_03で見た`com_pos_offset_w`(HACK補正)を加えたもの。コメントアウトされた`# position=base_pos`が、以前は胴体位置を使っていた可能性を示している
- `linear_velocity`(m/s、world座標系)には胴体速度(`base_lin_vel`)が入り、`position`は重心基準なのに`linear_velocity`は胴体基準という非対称性がある(重心速度は使われていない)
- `orientation`(rad)：胴体の姿勢(roll, pitch, yaw)をそのまま格納する。加工は一切ない
- `angular_velocity`(rad/s、base座標系)：胴体の角速度をそのまま格納する。こちらも加工はない
- `foot_FL`〜`foot_RR`(m、world座標系)：4本の足の現在位置をそのまま格納する。read_code_04で見た地形推定が使う「離地位置」とは別物で、こちらは今この瞬間の実際の位置
- `joint_FL`等には`joints_pos.FL`が入るが、`simulation.py`側の観測取得(`read_code_01`)で確認した通り、`joints_pos`の実体は関節角度の値ではなく`legs_qvel_idx`(インデックス配列)である。つまりこの`state_current['joint_FL']`は、見た目は「関節角度の状態」に見えるが、中身はインデックスの配列になっている

---

## 179〜183行：速度変調の適用

```python
        if self.vm.activated:
            ref_base_lin_vel, ref_base_ang_vel = self.vm.modulate_velocities(
                ref_base_lin_vel, ref_base_ang_vel, feet_pos, hip_pos
            )
```

- `config.py`の既定`velocity_modulator=True`によりこの分岐は毎回実行される
- read_code_05で見た`modulate_velocities`がここで呼ばれ、脚が伸びきりそうなら目標速度を強制的にゼロへ書き換える
- この時点の`ref_base_lin_vel`は、まだ地形の傾きによる回転補正(262行目以降)を受ける前の、素の目標速度

---

## 185〜210行：歩容の更新

```python
        if self.pgg.start_and_stop_activated:
            self.pgg.update_start_and_stop(...)

        self.pgg.run(simulation_dt, self.pgg.step_freq)
        contact_sequence = self.pgg.compute_contact_sequence(
            contact_sequence_dts=self.contact_sequence_dts, contact_sequence_lenghts=self.contact_sequence_lenghts
        )

        self.previous_contact = copy.deepcopy(self.current_contact)
        self.current_contact = np.array(
            [contact_sequence[0][0], contact_sequence[1][0], contact_sequence[2][0], contact_sequence[3][0]]
        )
```

- `start_and_stop_activated`は既定`False`(read_code_02で確認済み)なので、この`if`ブロックは通常実行されない
- `self.pgg.run(...)`と`self.pgg.compute_contact_sequence(...)`はread_code_02で読んだメソッドそのもの。ここでようやく、それらが実際にどこから・どういう順番で呼ばれているかが分かる
- `previous_contact`を更新してから`current_contact`を`contact_sequence`の1列目(今この瞬間の接地状態)で更新する、という順序になっている点が重要。次のブロック(着地点更新)がこの2つの新旧比較を使う

---

## 212〜238行：着地点の更新

```python
        self.frg.update_lift_off_positions(
            self.previous_contact, self.current_contact, feet_pos, legs_order,
            self.pgg.gait_type, base_pos, base_ori_euler_xyz,
        )
        self.frg.update_touch_down_positions(
            self.previous_contact, self.current_contact, feet_pos, legs_order,
            self.pgg.gait_type, base_pos, base_ori_euler_xyz,
        )
        ref_feet_pos = self.frg.compute_footholds_reference(
            base_position=base_pos, base_ori_euler_xyz=base_ori_euler_xyz,
            base_xy_lin_vel=base_lin_vel[0:2], ref_base_xy_lin_vel=ref_base_lin_vel[0:2],
            hips_position=hip_pos, com_height_nominal=cfg.simulation_params['ref_z'],
        )
```

- read_code_03で読んだ3つのメソッドが、この順番(離地更新→着地更新→着地点計算)で呼ばれる
- `ref_base_xy_lin_vel=ref_base_lin_vel[0:2]`には、直前の速度変調(179〜183行)を通過した後の値が渡される。つまり脚が伸びきっていて速度がゼロに書き換えられていれば、着地点計算もその「ゼロ速度」を前提に行われる

---

## 240〜256行：地形を考慮した着地点調整(既定では未実行)

```python
        if cfg.simulation_params['visual_foothold_adaptation'] != 'blind':
            ...
            ref_feet_pos, ref_feet_constraints = self.vfa.get_footholds_adapted(ref_feet_pos)
        else:
            ref_feet_constraints = LegsAttr(FL=None, FR=None, RL=None, RR=None)
```

- 既定(`"blind"`)ではこのブロックはすべて`else`側に入り、`ref_feet_constraints`は4脚とも`None`になる
- `visual_foothold_adaptation`(HeightMapを使った着地点補正)自体は`simulation.py`の該当節(read_code_01)で見た通り既定OFFの機能なので、この章では深掘りせず保留とします

---

## 260〜275行：地形に応じた目標速度・目標位置の補正

```python
        ref_base_lin_vel = R.from_euler("xyz", [terrain_roll, terrain_pitch, 0]).as_matrix() @ ref_base_lin_vel
        if(terrain_pitch > 0.0):
            ref_base_lin_vel[2] = -ref_base_lin_vel[2]
        if(np.abs(terrain_pitch) > 0.2):
            ref_base_lin_vel[0] = ref_base_lin_vel[0]/2.
            ref_base_lin_vel[2] = ref_base_lin_vel[2]*2
```

- 目標速度を、地形の傾き(roll, pitch)で作った回転行列で回転させる
- read_code_04の結論により`terrain_roll`は常に`0.0`なので、この回転は実質**pitchの分だけ**しか効かない
- `terrain_pitch > 0.0`なら、速度のZ成分の符号を反転させる。さらに`|terrain_pitch| > 0.2`(ラジアン、約11.5度)という急な傾きのときは、前後速度を半分に、上下速度を2倍にする
- この`0.2`という閾値や、半分・2倍という係数がなぜこの値なのかを説明するコメントはコード中になく、根拠は本ファイルの範囲では分からない(未確認)

```python
        ref_pos = np.array([0, 0, cfg.hip_height])
        ref_pos[2] = cfg.simulation_params["ref_z"] + terrain_height
        # Since the MPC close in CoM position, but usually we have desired height for the base,
        # we modify the reference to bring the base at the desired height and not the CoM
        ref_pos[2] -= base_pos[2] - (com_pos[2] + self.frg.com_pos_offset_w[2])
```

- `ref_pos`：目標位置(m、world座標系)。x,yは`0`で仮初期化、Zだけ直後に上書きされる
- 目標のZ位置(m)を、設定値`ref_z`(既定`hip_height`と同じ、Go2なら`0.28`)と推定地形高さ`terrain_height`の和として計算する
- 続くコメントが明快にこの補正の意図を説明している：「MPCは重心(CoM)位置で閉ループするが、通常欲しいのは胴体(base)の目標高さである。そこでCoMではなく胴体を目標高さに持っていくよう、参照値を修正する」
- 具体的には、「胴体位置と重心位置の差」(`base_pos[2] - (com_pos[2] + offset)`)を目標値から引くことで、間接的に胴体の高さを狙う、という補正になっている

---

## 278〜303行：`ref_state`辞書の組み立てと、型が原因で起こりうる例外

```python
        if cfg.mpc_params['type'] != 'kinodynamic':
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

- `ref_foot_FL`〜`ref_foot_RR`(m、world座標系)：read_code_03で読んだ`compute_footholds_reference`の戻り値`ref_feet_pos`を、そのまま`(1,3)`のshapeに整形して格納する
- `ref_foot_constraints_FL`〜`ref_foot_constraints_RR`：240〜256行のVFAブロックが作った`ref_feet_constraints`をそのまま格納する。既定(`"blind"`)ではVFAが動かないため4脚とも`None`が入る
- `ref_linear_velocity`(m/s)：262〜268行で地形の傾きに応じて回転・補正した後の目標並進速度
- `ref_angular_velocity`(rad/s)：目標角速度。並進速度と違い、こちらは地形による回転補正を一切受けず、179〜183行の速度変調(`VelocityModulator`)を通過した後の値がそのまま入る
- `ref_orientation`(rad)：地形roll・pitchと固定の`0.0`(yaw)を並べたベクトル
- `ref_position`(m)：270〜275行で計算した、重心基準に変換済みの目標高さを含む位置

**実装上の問題点(重大な疑い)**：

- `ref_state`という変数は、この`if cfg.mpc_params['type'] != 'kinodynamic':`ブロックの**内側でしか定義されていない**
- このメソッドの最後(305行目)は`return state_current, ref_state, contact_sequence, self.step_height, optimize_swing`で、`ref_state`を無条件に参照している
- つまり、もし`mpc_params['type']`が`'kinodynamic'`に設定されていた場合、`ref_state`はどこにも代入されないまま`return`文で参照されることになり、**`UnboundLocalError`(ローカル変数が定義される前に参照された、という実行時エラー)で例外が発生する**はずである
- 標準設定(`'nominal'`)ではこの分岐に入らないため問題は起きないが、`kinodynamic`タイプを試そうとした場合、この関数自体がここで落ちる可能性が高い(コードを読んで導いた指摘であり、実際に`kinodynamic`を指定して実行し確認したわけではない)

- `ref_orientation=np.array([terrain_roll, terrain_pitch, 0.0])`：read_code_04の結論(`terrain_roll`は常に0)から、**MPCの目標roll角は常にちょうど0**になる。地形が実際に左右へ傾いていても、目標姿勢としてその傾きに合わせて胴体を傾けることは(標準設定では)ない、ということがここで具体的に確認できる

```python
        if cfg.mpc_params['optimize_step_freq']:
            optimize_swing = self.stc.check_touch_down_condition(...)
        else:
            optimize_swing = 0

        return state_current, ref_state, contact_sequence, self.step_height, optimize_swing
```

- `optimize_swing`(`int`、0または1)：`config.py`の`mpc_params['optimize_step_freq']`は既定`False`なので、既定では常に`0`
- 最後に5つの値をタプルで返す。この戻り値をそのまま受け取っていたのが、`quadruped_pympc_wrapper.py`の`compute_actions`(`NN_quadruped_pympc_wrapper_walkthrough.md`で読んだ113〜131行)だった

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `com_pos[2] = robot_height`という行に開発者自身が`#TODO, this is an error`と書いて無効化している
  2. `ref_state`が`mpc_params['type'] == 'kinodynamic'`のとき定義されないまま`return`され、`UnboundLocalError`になる可能性が高い
  3. 戻り値の型注釈(8要素)と実際の`return`文(5要素)が一致していない
  4. 地形傾斜に応じた速度補正(`|pitch|>0.2`で半分・2倍)の根拠がコード中に説明されていない
- read_code_02(歩容)・read_code_03(着地点)・read_code_04(地形推定)・read_code_05(速度変調)の4つの出力が、すべてこの1つのメソッドの中で`state_current`/`ref_state`/`contact_sequence`という3点セットへ集約されることが確認できた
- 次はいよいよMPC本体です。`interfaces/srbd_controller_interface.py`(この3点セットを受け取ってOCPを解く窓口)に進みます。
