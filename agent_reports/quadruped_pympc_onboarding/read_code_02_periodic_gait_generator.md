# 上流の計画① helpers/periodic_gait_generator.py 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

このファイルは`simulation.py`から直接は呼ばれていませんが、次の連鎖でサブのサブとして呼ばれています。

```text
simulation.py (run_simulationのループ、read_code_01で解説)
  → quadrupedpympc_wrapper.compute_actions(...) (NN_quadruped_pympc_wrapper_walkthrough.md、保留)
      → self.wb_interface.update_state_and_reference(...) (wb_interface.py、未解説)
          → self.pgg.run(...)                    ← 本ファイル、位相を1ステップ進める
          → self.pgg.compute_contact_sequence(...) ← 本ファイル、未来の接地スケジュールを先読み
```

`self.pgg`は`WBInterface.__init__`の中で`PeriodicGaitGenerator(...)`として生成されるインスタンスです。呼び出し頻度は、MPCの間引き(`NN_quadruped_pympc_wrapper_walkthrough.md`で確認した`step_num % round(...)`)とは無関係に、`simulation.py`の内側ループが1回まわるたびに毎回呼ばれます(既定500Hz相当)。

## このクラスの役割(全体の中での位置づけ)

`PeriodicGaitGenerator`が担当するのは、制御パイプライン全体のうち「**いつ、どの脚を接地させるか**」という歩容スケジュールの決定だけです。

- 入力：歩容の種類(トロット等)・歩行周波数・duty factor
- 出力：`contact_sequence`(4脚×ホライズンの0/1配列)
- この出力は、後段のMPC(`centroidal_model_nominal.py`、まだ未解説)の運動方程式に**決定変数ではなくパラメータ**として入る(接地している脚だけに地面反力が乗り、遊脚には乗らない、という形で力学式に反映される)
- 「どこに接地するか」(着地点の座標)はこのクラスの責務ではありません。それは次に読む[read_code_03](read_code_03_foothold_reference_generator.md)(`FootholdReferenceGenerator`)の役割です

言い換えると、このクラスは「歩容の時計」であり、位置や力の計算は一切行いません。

パイプラインの正しい順序は「上流で計画を立てる(歩容→着地点)→センシング済みの状態と合わせる→下流のMPC→WBC」であり、MPCが最初ではありません。`WBInterface.update_state_and_reference`はこの歩容計画クラス(`PeriodicGaitGenerator`)を内部で呼んでいるだけで、計画そのものの中身はまだ読んでいませんでした。MPC(`srbd_controller_interface.py`)やWBC(`wb_interface.py`のトルク計算部分)より先に、この「計画」の実体を読みます。

対象は `external/Quadruped-PyMPC/quadruped_pympc/helpers/periodic_gait_generator.py`(197行)です。

---

## 1〜6行：import

```python
import copy
import numpy as np
from gym_quadruped.utils.quadruped_utils import LegsAttr
from quadruped_pympc.helpers.quadruped_utils import GaitType
```

- `GaitType`：歩容の種類(トロット、ペース等)を表す列挙型。22行目以降の`reset()`で使われる
- `LegsAttr`：read_code_01・NNで既出の、4脚の値を持つ入れ物クラス

---

## 8〜20行：コンストラクタ

この関数の役割:歩容パラメータを受け取り、4脚の位相を初期状態にリセットする。

```python
class PeriodicGaitGenerator:
    def __init__(self, duty_factor, step_freq, gait_type: GaitType, horizon):
        self.duty_factor = duty_factor
        self.step_freq = step_freq
        self.horizon = horizon
        self.gait_type = gait_type
        self.previous_gait_type = copy.deepcopy(gait_type)

        self.start_and_stop_activated = False

        self._phase_signal, self._init = None, None
        self.reset()
```

- `duty_factor`：1周期のうち接地に使う割合(無次元、0〜1)。デフォルト値はないが、`config.py`のトロット設定では`0.65`が渡される
- `step_freq`：歩行周波数(Hz)。デフォルト値はないが、`config.py`のトロット設定では`1.4`が渡される
- `gait_type`：歩容の種類(`GaitType`列挙型)。デフォルト値はないが、`config.py`の既定`gait='trot'`では`GaitType.TROT.value`が渡される
- `horizon`：MPCの予測ホライズンのステップ数(無次元)。デフォルト値はないが、`config.py`の`mpc_params['horizon']`は既定`12`
- `previous_gait_type`：現在の歩容を退避しておく変数(`GaitType`)。120〜126行の「フルスタンスへ切り替えて、また元に戻す」処理のために使う
- `self.start_and_stop_activated = False`
  - 既定でFalse。NNで読んだ`wb_interface.py`側(`if self.pgg.start_and_stop_activated:`)もこのフラグを見てから128行目の`update_start_and_stop`を呼んでいたので、既定設定ではこの機能全体が動かない
- コンストラクタの最後で自分自身の`self.reset()`を呼んでいる

---

## 22〜46行：`reset()`(歩容ごとの位相オフセット表)

この関数の役割:歩容タイプごとの位相オフセット表を選び、位相・接地状態を初期化する。

```python
def reset(self):
    if self.gait_type == GaitType.TROT.value:
        self.phase_offset = [0.5, 1.0, 1.0, 0.5]
    elif self.gait_type == GaitType.PACE.value:
        self.phase_offset = [0.8, 0.3, 0.8, 0.3]
    elif self.gait_type == GaitType.BOUNDING.value:
        self.phase_offset = [0.5, 0.5, 0.0, 0.0]
    elif self.gait_type == GaitType.CIRCULARCRAWL.value:
        self.phase_offset = [0.0, 0.25, 0.75, 0.5]
    elif self.gait_type == GaitType.BFDIAGONALCRAWL.value:
        self.phase_offset = [0.0, 0.25, 0.5, 0.75]
    elif self.gait_type == GaitType.BACKDIAGONALCRAWL.value:
        self.phase_offset = [0.0, 0.5, 0.75, 0.25]
    elif self.gait_type == GaitType.FRONTDIAGONALCRAWL.value:
        self.phase_offset = [0.5, 1.0, 0.75, 1.25]
    else:
        self.phase_offset = [0.0, 0.5, 0.5, 0.0]
```

- 各歩容タイプに対して、4脚(`legs_order=[FL,FR,RL,RR]`の順)の位相オフセットを固定のリストとして定義している
- トロット(既定)：`[0.5, 1.0, 1.0, 0.5]`
  - FLとRRが同位相(0.5と0.5)、FRとRLが同位相(1.0と1.0、実質0.0と同じ、後述のmod演算で扱いが決まる)
  - つまり対角の2本ずつが同時に接地・浮遊する組み合わせ
- 気になる点：`FRONTDIAGONALCRAWL`だけ`1.25`という**1.0を超える値**が入っている
  - 他の歩容の値はすべて`[0.0, 1.0)`の範囲に収まっている
  - この値が48行目の`run()`でどう扱われるかは、次のブロックで具体的に追う

```python
    self._phase_signal = np.asarray(self.phase_offset)
    self._init = [False] * len(self.phase_offset)
    self.n_contact = len(self.phase_offset)
    self.time_before_switch_freq = 0
```

- `self._phase_signal`：4脚それぞれの現在の位相(無次元、0〜1)。初期値は`phase_offset`(トロットなら`[0.5, 1.0, 1.0, 0.5]`)
- `self._init`：4脚分の`bool`のリスト。初期値は全脚`False`。この後の`run()`で「立ち上がり時の特別扱い」に使われるフラグ
- `self.n_contact`：`len(self.phase_offset)`、つまり単純に**脚の本数**。初期値は`4`(トロットなら`phase_offset`が4要素のため)。「今接地している脚の数」という意味ではない、やや紛らわしい名前
- `self.time_before_switch_freq`：初期値`0`(単位不明)。本ファイルの中では以降どこにも参照されていない(未確認：他ファイルで使われている可能性はあるが、本パスでは確認していない)

---

## 48〜76行：`run(dt, new_step_freq)`(位相を1ステップ進める)

この関数の役割:各脚の位相を時間刻み分だけ進め、その瞬間の接地状態(0/1)を返す。

```python
def run(self, dt, new_step_freq):
    contact = np.zeros(self.n_contact)
    for leg in range(self.n_contact):
        self._phase_signal[leg] += dt * new_step_freq
        self._phase_signal[leg] = self._phase_signal[leg] % 1.0
```

- `dt`：時間刻み(秒)。デフォルト値はなく呼び出し元が渡す。`update_state_and_reference`からの通常呼び出しでは`simulation_dt`(既定`0.002`秒)が渡される
- `new_step_freq`：歩行周波数(Hz)。デフォルト値はなく呼び出し元が渡す。通常呼び出しでは`self.pgg.step_freq`(トロット既定`1.4`)がそのまま渡される
- 各脚の位相を`位相 += dt × 周波数`で進め、`% 1.0`で0〜1の範囲に折り返す
- 数値例：`step_freq=1.4`、`dt=0.002`なら、1ステップで位相は$0.002\times1.4=0.0028$だけ進む
- ここで**`FRONTDIAGONALCRAWL`の`1.25`という初期値**がどう扱われるかが分かる
  - 最初の`run()`呼び出しで`1.25 + dt×freq`を計算し、`% 1.0`するので、**初回の呼び出しの時点で`1.25`は`0.25`前後に折り返される**
  - つまり`_phase_signal`自体は常に`[0.0, 1.0)`に収まるように強制される

```python
        if self._init[leg]:
            if self._phase_signal[leg] <= self.phase_offset[leg]:
                contact[leg] = 1
            else:
                self._init[leg] = False
                contact[leg] = 1
                self._phase_signal[leg] = 0
        else:
            if self._phase_signal[leg] < self.duty_factor:
                contact[leg] = 1
            else:
                contact[leg] = 0
    return contact
```

- `_init[leg]`が`True`の間(立ち上がり時)は、「位相がまだ`phase_offset[leg]`を超えていなければ接地のまま」という特別な待機ロジックが働く
- `_init[leg]`が`False`になった後の通常時は、「位相が`duty_factor`未満なら接地、以上なら遊脚」という単純な判定になる(既定`duty_factor=0.65`なら、1周期の65%が接地期)

**実装上の問題点(強い疑い)**:ここで`FRONTDIAGONALCRAWL`の`phase_offset=1.25`という値が効いてくる。

- `_phase_signal[leg]`は上で見た通り常に`[0.0, 1.0)`の範囲
- `_init[leg]`が`True`の間の判定は`_phase_signal[leg] <= phase_offset[leg]`、すなわち`<= 1.25`
- `_phase_signal[leg]`が`1.25`を超えることは構造上あり得ない(`% 1.0`されるため最大でも1.0未満)ので、この条件は**常に真**になる
- つまり`_init[leg]`を`False`に切り替える`else`分岐(65行目)へ**一度も到達できない**
- 結果として、この歩容タイプの該当脚(`phase_offset`が4番目=RRに対応)は、`_init`が永遠に`True`のままになり、`contact[leg]=1`(常に接地扱い)が固定され続ける可能性が高い
- これは実際に動かして確認したわけではなく、コードを読んで論理的に導いた推測だが、根拠は明確(他の歩容の`phase_offset`はすべて1.0未満に収まっており、この歩容だけ設計上の想定を外れた値になっている可能性が高い)

---

## 78〜87行：`set_phase_signal`

この関数の役割:位相と立ち上がりフラグを外部から直接上書きするセッター。

```python
def set_phase_signal(self, phase_signal: np.ndarray, init: np.ndarray | None = None):
    assert len(phase_signal) == len(self._phase_signal)
    self._phase_signal = phase_signal
    if init is not None:
        assert len(init) == len(self._init)
        self._init = init
    else:
        self._init = [False for _ in range(len(self._phase_signal))]
```

- `_phase_signal`(と任意で`_init`)を外部から直接上書きするためのセッター
- `compute_contact_sequence`(93行目以降)が、位相を一時的に進めてからこの関数で元の位相に戻す、という使い方をする(次のブロックで確認する)

---

## 89〜91行：`phase_signal`プロパティ

この関数の役割:内部の位相配列を、外部から安全に読めるようコピーして返す。

```python
@property
def phase_signal(self):
    return np.array(self._phase_signal)
```

- `np.array(...)`で**コピー**を作って返す、公開用のプロパティ
- NNで見た`quadruped_pympc_wrapper.py`235行目は、この公開プロパティを使わず`pgg._phase_signal`(コピーではない生の内部配列)を直接読んでいた
- ここで初めて「公開プロパティ側はちゃんとコピーを返す設計になっている」ことが確認できる。つまりNNで指摘した問題は、こちらの`_phase_signal`側の設計が悪いのではなく、**呼び出し側(quadruped_pympc_wrapper.py)がこの安全なプロパティを使わずに内部変数へ直接アクセスしている**ことが原因だと分かる

---

## 93〜118行：`compute_contact_sequence`(未来の接地スケジュールを作る)

この関数の役割:位相を副作用なく先読みし、ホライズン分の接地スケジュールを作って返す。

```python
def compute_contact_sequence(self, contact_sequence_dts, contact_sequence_lenghts):
    if self.gait_type == GaitType.FULL_STANCE.value:
        contact_sequence = np.ones((4, self.horizon * 2))
        self.reset()
        return contact_sequence
```

- `contact_sequence_dts`：時間刻みのリスト(秒)。デフォルト値はなく呼び出し元が渡す。既定設定(`use_nonuniform_discretization=False`)では`[0.02]`(要素1つ、`mpc_params['dt']`の値)
- `contact_sequence_lenghts`：各時間刻みを何ステップ使うかのリスト(無次元)。デフォルト値はなく呼び出し元が渡す。既定設定では`[12]`(要素1つ、`mpc_params['horizon']`の値)
- 歩容が`FULL_STANCE`(全脚接地、止まっているとき用)なら、全部`1`の配列を返して終わり
- **実装上の問題点**:このときのshapeは`(4, self.horizon * 2)`、つまり列数が`horizon`の**2倍**になっている
  - 通常経路(下記)が返すshapeは`(n_contact, horizon)`で、列数は`horizon`そのもの
  - この2つの経路で戻り値のshapeが違うのは、呼び出し側が両方のケースを同じように扱おうとした場合に不整合の原因になりうる。ここは本当に意図した仕様なのか、それとも単純な誤記(`*2`が不要)なのか、本ファイルの範囲だけでは判断できない

```python
    else:
        t_init = np.array(self._phase_signal)
        init_init = np.array(self._init)

        contact_sequence = np.zeros((self.n_contact, self.horizon))
        contact_sequence[:, 0] = self.run(0.0, self.step_freq)

        j = 0
        for i in range(1, self.horizon):
            if i >= contact_sequence_lenghts[j]:
                j += 1
            dt = contact_sequence_dts[j]
            contact_sequence[:, i] = self.run(dt, self.step_freq)
        self.set_phase_signal(t_init, init_init)
        return contact_sequence
```

- 通常の歩容の場合の処理:
  - まず現在の位相(`t_init`)と`_init`フラグ(`init_init`)を退避する
  - `contact_sequence`という`(4, horizon)`の配列を用意する
  - 1列目(`contact_sequence[:, 0]`)は、`dt=0.0`で`run()`を呼んだ結果、つまり**現在の接地状態そのもの**
  - 2列目以降は、`run()`を`horizon-1`回連続で呼びながら、その都度`self._phase_signal`(内部状態)を実際に**書き換えながら**先読みしていく
  - 先読みが終わったら`set_phase_signal(t_init, init_init)`で元の位相へ**巻き戻す**
- この「実際に内部状態を進めてから、最後に元へ戻す」という設計により、副作用なく未来の接地スケジュールを覗き見ることができる
- `contact_sequence_dts`/`contact_sequence_lenghts`は非一様な時間刻みに対応するための仕組み。`j`というインデックスで「今何番目の時間刻み帯にいるか」を管理し、`contact_sequence_lenghts[j]`ステップに達したら次の時間刻み`contact_sequence_dts[j+1]`へ切り替える
- コメント「TODO: This function can be vectorized and computed with numpy vectorized operations」がそのまま残っており、`for`ループでの逐次計算が最適化されていないことを実装者自身が認識している

---

## 120〜126行：フルスタンスへの切り替えと復帰

```python
def set_full_stance(self):
    self.gait_type = GaitType.FULL_STANCE.value
    self.reset()

def restore_previous_gait(self):
    self.gait_type = copy.deepcopy(self.previous_gait_type)
    self.reset()
```

- `set_full_stance()`：歩容を`FULL_STANCE`に切り替えて`reset()`する。これで93行目の`compute_contact_sequence`が全脚接地の経路に入る
- `restore_previous_gait()`：14行目で退避しておいた`previous_gait_type`へ戻す
- どちらも呼び出し元は次のブロックの`update_start_and_stop`

---

## 128〜197行：`update_start_and_stop`(エネルギー節約のための自動停止、既定では未使用)

この関数の役割:止まる条件を満たしたら全脚接地へ切り替え、動き出したら歩容を復元する。

```python
def update_start_and_stop(
    self, feet_pos, hip_pos, hip_offset, base_pos, base_ori_euler_xyz,
    base_lin_vel, base_ang_vel, ref_base_lin_vel, ref_base_ang_vel, current_contact,
):
    yaw = base_ori_euler_xyz[2]
    R_W2H = np.array([np.cos(yaw), np.sin(yaw), -np.sin(yaw), np.cos(yaw)])
    R_W2H = R_W2H.reshape((2, 2))
```

- ワールド座標系からhorizontal frame(ロボットのyaw方向だけ回転させた座標系)への回転行列を作る
- この回転行列は、着地点計算を担当する`foothold_reference_generator.py`([read_code_03](read_code_03_foothold_reference_generator.md))の中で使われている変換と同じパターン(world座標系から、yaw角だけを取り除いた胴体中心の水平座標系への変換)

```python
    feet_pos_h = LegsAttr(*[np.zeros(3) for _ in range(4)])
    feet_pos_h.FL[:2] = R_W2H @ (feet_pos.FL[:2] - base_pos[0:2])
    ...
    feet_pos_h.FL[1] -= hip_offset
    feet_pos_h.FR[1] += hip_offset
    ...
```

- 各足の位置を、胴体を原点とした水平座標系へ変換し、さらに`hip_offset`(左右方向のオフセット、股関節の位置に合わせるための補正)を加える

```python
    feet_to_hip_distance_h_FL = np.sqrt(
        np.square(feet_pos_h.FL[0] - hip_pos_h.FL[0]) + np.square(feet_pos_h.FL[1] - hip_pos_h.FL[1])
    )
    ...
    feet_to_hip_distance_h = np.mean([...4脚分...])
```

- 4脚それぞれについて「足先が股関節の直下からどれだけ離れているか(水平距離)」を計算し、その平均を取る
- ロボットが完全に立ち止まって足を胴体の真下に揃えているかどうかを判定するための指標

```python
    if (
        np.linalg.norm(ref_base_lin_vel) == 0.0
        and np.linalg.norm(ref_base_ang_vel) == 0.0
        and np.linalg.norm(base_lin_vel) < 0.1
        and np.linalg.norm(base_ang_vel) < 0.1
        and np.abs(base_ori_euler_xyz[0]) < 0.05
        and np.abs(base_ori_euler_xyz[1]) < 0.05
        and np.sum(current_contact) == 4
        and feet_to_hip_distance_h_FL < 0.06
        and feet_to_hip_distance_h_FR < 0.06
        and feet_to_hip_distance_h_RL < 0.06
        and feet_to_hip_distance_h_RR < 0.06
    ):
        self.set_full_stance()
    elif self.gait_type == GaitType.FULL_STANCE.value:
        self.restore_previous_gait()
```

- 「止まる」ための安全条件をすべて`and`で並べている:
  - 目標速度(並進・角速度)がゼロ
  - 実際の速度もほぼゼロ(並進0.1 m/s未満、角速度0.1 rad/s未満)
  - roll/pitchの傾きがほぼゼロ(0.05 rad未満、約2.9度)
  - 4本すべてが接地中
  - 4本すべての足が股関節の真下6cm以内に収まっている
- すべて満たせば`set_full_stance()`でトロット等の周期的な足踏みをやめて全脚接地に切り替える(エネルギー消費を抑えるため)
- そうでなく、かつ現在フルスタンス中なら`restore_previous_gait()`で元の歩容に戻す(動き出す合図があれば復帰する、という設計)
- **ただしこの関数自体、8〜20行目で見た`start_and_stop_activated=False`という既定値により、NNで確認した`wb_interface.py`側のガード(`if self.pgg.start_and_stop_activated:`)を通らないと呼ばれない**。既定設定ではこの節の処理全体が実行されない

---

## この章のまとめ

- 歩容計画の中身は、位相を進める(`run`)・その位相からどの脚が接地しているかを判定する・その判定を`horizon`分先読みする(`compute_contact_sequence`)、という比較的シンプルなロジックだった
- 見つかった実装上の問題点:
  1. `FRONTDIAGONALCRAWL`の`phase_offset=1.25`が、`_init`フラグを永遠に`True`のまま固定してしまう可能性がある(論理的な推測、実行しての確認はしていない)
  2. `FULL_STANCE`のときだけ`compute_contact_sequence`の戻り値のshapeが`horizon*2`列になり、通常経路の`horizon`列と食い違う
  3. `time_before_switch_freq`が初期化されるだけで本ファイル内では未使用
- `start_and_stop_activated`が既定`False`のため、`update_start_and_stop`(自動停止機能)は既定では動かない
- 次に読むべきは、同じく「上流の計画」に属するもう1つのファイル、着地点計画(`helpers/foothold_reference_generator.py`)です。歩容(いつ接地するか)の次は、着地点(どこに接地するか)という順序になります。
