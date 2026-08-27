# MPCの窓口 interfaces/srbd_controller_interface.py 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.srbd_controller_interface.compute_control(...)   ← 本ファイル
      → self.srbd_controller_interface.compute_RTI()           ← 本ファイル(use_RTI=Trueのときのみ)
```

`self.srbd_controller_interface`は`QuadrupedPyMPC_Wrapper.__init__`の中で`SRBDControllerInterface()`として生成されます。`compute_control`が呼ばれるのは`quadrupedpympc_wrapper.py`の`step_num % round(1/(mpc_frequency*simulation_dt)) == 0`の条件内だけ(既定では5シミュレーションステップに1回)で、`update_state_and_reference`(read_code_06、毎ステップ)より低い頻度です。

## このファイルの役割(全体の中での位置づけ)

`SRBDControllerInterface`が担当するのは、「**`mpc_params['type']`の値に応じて、どのMPC実装(acadosの具体的なOCPソルバークラス)を使うかを選び、そのソルバーを呼び出す**」という窓口の役割です。

- 入力：read_code_06で組み立てた`state_current`/`ref_state`/`contact_sequence`
- 出力：GRF・着地点・(kinodynamicのみ)関節位置速度加速度・予測状態
- OCPの数式(コスト・制約・力学モデル)そのものはこのファイルには無い。実際の数式は`self.controller`(`Acados_NMPC_Nominal`等)の中にあり、本ファイルは分岐と後処理だけを行う
- 出力のGRFは、このファイルの中でさらに接地マスクを掛けられてから返される(下記)

対象は `external/Quadruped-PyMPC/quadruped_pympc/interfaces/srbd_controller_interface.py`(246行)です。

---

## 1〜8行：import とクラスの位置づけ

```python
import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr

from quadruped_pympc import config as cfg


class SRBDControllerInterface:
    """This is an interface for a controller that uses the SRBD method to optimize the gait"""
```

- クラスdocstringは「SRBD法を使ってgaitを最適化するコントローラのインターフェース」と説明しているが、実際に最適化するのはgait(歩容のタイミング)ではなくGRF(地面反力)である。docstringが実態と少しずれている

---

## 10〜19行：コンストラクタ前半(設定の読み込み)

```python
    def __init__(self):
        self.type = cfg.mpc_params['type']
        self.mpc_dt = cfg.mpc_params['dt']
        self.horizon = cfg.mpc_params['horizon']
        self.optimize_step_freq = cfg.mpc_params['optimize_step_freq']
        self.step_freq_available = cfg.mpc_params['step_freq_available']

        self.previous_contact_mpc = np.array([1, 1, 1, 1])
```

- `self.type`：使用するMPC実装の種類(文字列)。デフォルト値はないが`config.py`の`mpc_params['type']`は既定`'nominal'`
- `self.mpc_dt`：OCPの内部離散化の時間刻み(秒)。既定`0.02`(read_code_06で見た値と同じ`config.py`のキー)
- `self.horizon`：OCPの予測ホライズンのステップ数(無次元)。既定`12`
- `self.optimize_step_freq`：歩行周波数を最適化するかのフラグ(`bool`)。既定`False`
- `self.step_freq_available`：歩行周波数の候補リスト(Hz)。既定`[1.4, 2.0, 2.4]`。`optimize_step_freq=True`のときのみ使われる(既定では未使用)
- `self.previous_contact_mpc`：4脚分の0/1配列(無次元)。初期値`[1,1,1,1]`。117行目以降で確認する通り`self.type=='sampling'`のときだけ使われ、既定の`'nominal'`では一度も読まれない

---

## 21〜83行：MPC実装の選択(dispatch)

```python
        if self.type == "nominal":
            from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal import Acados_NMPC_Nominal
            self.controller = Acados_NMPC_Nominal()
            if self.optimize_step_freq:
                from quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_gait_adaptive import (
                    Acados_NMPC_GaitAdaptive,
                )
                self.batched_controller = Acados_NMPC_GaitAdaptive()

        elif self.type == 'input_rates':
            ...
        elif cfg.mpc_params['type'] == 'lyapunov':
            ...
        elif cfg.mpc_params['type'] == 'kinodynamic':
            ...
        elif self.type == "sampling":
            ...
```

- `self.type`の値によって、5種類のうちどのMPC実装クラスを`self.controller`として生成するかを選ぶ。既定`'nominal'`では`Acados_NMPC_Nominal()`が選ばれる(まだ未解説、次に読む候補)
- **実装上の細かい不統一**：`if`は`self.type ==`で判定しているが、`lyapunov`と`kinodynamic`の分岐だけ`cfg.mpc_params['type'] ==`で直接判定している。`self.type`は13行目で`cfg.mpc_params['type']`から代入されたばかりの同じ値なので動作上の違いは無いが、同じ関数内で2通りの書き方が混在している
- `optimize_step_freq`が`True`のときだけ、追加で`self.batched_controller`(`Acados_NMPC_GaitAdaptive`)を生成する。既定`False`なので、`self.batched_controller`という属性自体が**既定では存在しない**
- `'sampling'`の分岐(77〜83行)はJAXベースの実装(`Sampling_MPC`)を選ぶ。既定`'nominal'`ではこの分岐には入らないため、本章では深掘りせず、選ばれるクラスの存在確認のみに留める

---

## 85〜111行：`compute_control`のシグネチャ

```python
    def compute_control(
        self,
        state_current: dict,
        ref_state: dict,
        contact_sequence: np.ndarray,
        inertia: np.ndarray,
        pgg_phase_signal: np.ndarray,
        pgg_step_freq: float,
        optimize_swing: int,
        external_wrenches: np.ndarray = np.zeros((6,)),
    ) -> [LegsAttr, LegsAttr, LegsAttr, LegsAttr, LegsAttr, float]:
```

- 引数の意味:
  - `state_current`：read_code_06で組み立てた現在状態の辞書。デフォルト値はなく必須引数
  - `ref_state`：read_code_06で組み立てた目標状態の辞書。デフォルト値はなく必須引数
  - `contact_sequence`：4脚×ホライズンの接地スケジュール(無次元、0/1)。デフォルト値はなく必須引数
  - `inertia`：慣性(9要素)。デフォルト値はなく必須引数
  - `pgg_phase_signal`：4脚分の歩容位相(無次元、0〜1)。デフォルト値はなく必須引数
  - `pgg_step_freq`：現在の歩行周波数(Hz)。デフォルト値はなく必須引数
  - `optimize_swing`：歩行周波数最適化のタイミングフラグ(`int`、0か1)。デフォルト値はなく必須引数
  - `external_wrenches`：外力補償用の外力・外トルク(6要素、力3+モーメント3)。デフォルト`np.zeros((6,))`

**実装上の問題点**：`external_wrenches`のデフォルト値が`np.zeros((6,))`という**可変オブジェクト(numpy配列)のリテラル**になっている。Pythonの一般的な注意点として、可変オブジェクトを関数のデフォルト引数にすると、複数回の呼び出しで**同じオブジェクトが使い回される**(呼び出しのたびに新しく作られない)。本ファイルの中ではこの引数を書き換える処理は無いため今のところ実害は無いが、将来どこかでこの引数がin-placeで変更されるコードが追加されると、意図しない副作用を生む典型的な落とし穴になる

---

## 113〜116行：現在の接地状態の取り出し

```python
        current_contact = np.array(
            [contact_sequence[0][0], contact_sequence[1][0], contact_sequence[2][0], contact_sequence[3][0]]
        )
```

- `current_contact`：4脚分の0/1配列(無次元)。`contact_sequence`の1列目(今この瞬間の接地状態)を取り出したもの
- read_code_06の`WBInterface.current_contact`と同じ考え方だが、こちらは`SRBDControllerInterface`が独自に持つ**別のローカル変数**であり、`self`には保存されない(このメソッド内だけで使われて捨てられる)

---

## 117〜181行:サンプリングMPC経路(既定では通らない、概要のみ)

```python
        if self.type == 'sampling':
            state_current_jax, reference_state_jax = self.controller.prepare_state_and_reference(...)
            self.previous_contact_mpc = current_contact
            for iter_sampling in range(self.controller.num_sampling_iterations):
                ...
                self.controller.jitted_compute_control(...)
            nmpc_footholds = LegsAttr(FL=ref_state["ref_foot_FL"][0], ...)
            nmpc_GRFs = np.array(nmpc_GRFs)
            nmpc_joints_pos = None
            nmpc_joints_vel = None
            nmpc_joints_acc = None
```

- 既定`self.type='nominal'`ではこの`if`ブロックには入らない
- 概要だけ確認すると、サンプリング方式では`nmpc_footholds`がMPCの最適化結果ではなく`ref_state`(read_code_06で計算した参照着地点)をそのまま使っている点が、勾配ベース方式(次節)と異なる
- JAX側の内部実装(`jitted_compute_control`等)は未読で、本章では深掘りしない

---

## 182〜222行:勾配ベースMPC経路(既定`'nominal'`が通る場所)

```python
        else:
            if self.type == 'kinodynamic':
                (
                    nmpc_GRFs, nmpc_footholds, nmpc_joints_pos, nmpc_joints_vel,
                    nmpc_joints_acc, nmpc_predicted_state, status,
                ) = self.controller.compute_control(
                    state_current, ref_state, contact_sequence, inertia=inertia, external_wrenches=external_wrenches
                )
                nmpc_joints_pos = LegsAttr(FL=nmpc_joints_pos[0:3], FR=nmpc_joints_pos[3:6], RL=nmpc_joints_pos[6:9], RR=nmpc_joints_pos[9:12])
                ...
            else:
                nmpc_GRFs, nmpc_footholds, nmpc_predicted_state, _ = self.controller.compute_control(
                    state_current, ref_state, contact_sequence, inertia=inertia, external_wrenches=external_wrenches
                )
                nmpc_joints_pos = None
                nmpc_joints_vel = None
                nmpc_joints_acc = None

            nmpc_footholds = LegsAttr(FL=nmpc_footholds[0], FR=nmpc_footholds[1], RL=nmpc_footholds[2], RR=nmpc_footholds[3])
            best_sample_freq = pgg_step_freq
```

- 既定`'nominal'`は`kinodynamic`でもないため、**最後の`else`(209〜216行)を通る**
- `self.controller.compute_control(...)`が呼ばれ、4つの値(`nmpc_GRFs`, `nmpc_footholds`, `nmpc_predicted_state`, 使わない4番目の値)を受け取る。この`self.controller`は`Acados_NMPC_Nominal`のインスタンスであり、その`compute_control`メソッドの中身(コスト・制約・acadosの求解)は本ファイルには無い。**次に読むべきファイルはここ**
- 4番目の戻り値は`_`という捨て変数で受けており、名前が付いていない(**未確認**：何が返っているかは`Acados_NMPC_Nominal.compute_control`を読まないと分からない)
- `nmpc_joints_pos`/`nmpc_joints_vel`/`nmpc_joints_acc`は`kinodynamic`のときだけ実際の値が入り、それ以外(既定含む)では`None`になる
- `nmpc_footholds`：受け取った配列(4要素、インデックス0〜3)を、脚名でアクセスできる`LegsAttr`へ変換する
- `best_sample_freq`：勾配ベース方式では周波数最適化を行わないため、単純に入力の`pgg_step_freq`をそのまま返す(既定では歩行周波数`1.4`Hzがそのまま素通りする)

---

## 224〜230行:GRFの接地マスク

```python
        # TODO: Indexing should not be hardcoded. Env should provide indexing of leg actuator dimensions.
        nmpc_GRFs = LegsAttr(
            FL=nmpc_GRFs[0:3] * current_contact[0],
            FR=nmpc_GRFs[3:6] * current_contact[1],
            RL=nmpc_GRFs[6:9] * current_contact[2],
            RR=nmpc_GRFs[9:12] * current_contact[3],
        )
```

- OCPが返す12要素のGRF配列(4脚×3成分、N)を、脚ごとに切り出して`current_contact`(0か1)を掛ける
- 遊脚中(`current_contact[i]=0`)の脚は、OCPがどんな値を計算していても、ここで強制的にゼロへ落とされる
- 開発者自身のTODOコメントの通り、インデックス(`0:3`, `3:6`等)がハードコードされており、脚の並び順が変わると壊れる作りになっている

---

## 232〜240行:戻り値

```python
        return (
            nmpc_GRFs, nmpc_footholds, nmpc_joints_pos, nmpc_joints_vel,
            nmpc_joints_acc, best_sample_freq, nmpc_predicted_state,
        )
```

- 7個の値を返す。read_code_06で見た`quadruped_pympc_wrapper.py`の`compute_actions`(133〜151行)が、これをそのまま`self.nmpc_GRFs`等へ受け取っていた

---

## 242〜245行:`compute_RTI`

```python
    def compute_RTI(self):
        self.controller.acados_ocp_solver.options_set("rti_phase", 1)
        self.controller.acados_ocp_solver.solve()
```

- `self.controller.acados_ocp_solver`(acadosのソルバーオブジェクト、`Acados_NMPC_Nominal`内部で保持)に対し、RTI(Real-Time Iteration)の準備フェーズを直接呼ぶ
- `quadrupedpympc_wrapper.py`側の呼び出し条件は`cfg.mpc_params['type'] != 'sampling' and cfg.mpc_params['use_RTI']`。`config.py`の`mpc_params['use_RTI']`は既定`False`のため、**このメソッド自体が既定では一度も呼ばれない**

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. クラスdocstringが「gaitを最適化する」と書いているが、実際に最適化するのはGRF
  2. 分岐条件の書き方が`self.type ==`と`cfg.mpc_params['type'] ==`で混在している(動作は同じ)
  3. `external_wrenches`のデフォルト値が可変オブジェクト(`np.zeros((6,))`)になっている、Pythonの典型的な落とし穴
  4. GRFの接地マスクのインデックスがハードコードされている(開発者自身のTODOで既知)
- 既定設定(`type='nominal'`, `optimize_step_freq=False`, `use_RTI=False`)では、このファイルの大半の分岐(`sampling`, `kinodynamic`, `input_rates`, `lyapunov`, バッチ最適化, `compute_RTI`)は通らず、実質「`Acados_NMPC_Nominal.compute_control`を呼んで、結果にGRFマスクを掛けて返す」という薄い窓口になっている
- 次に読むべきファイルは`self.controller.compute_control`の実体、`controllers/gradient/nominal/centroidal_nmpc_nominal.py`(OCPのコスト・制約・acados呼び出し)です。その前提として、OCPが扱う状態・入力・パラメータの次元を定義する`controllers/gradient/nominal/centroidal_model_nominal.py`を先に読む方が理解しやすいため、次はそちらから読みます。
