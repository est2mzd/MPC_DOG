# 上流の計画③ helpers/terrain_estimator.py 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.wb_interface.update_state_and_reference(...)
          → self.terrain_computation.compute_terrain_estimation(...)   ← 本ファイル
```

`self.terrain_computation`は`WBInterface.__init__`の中で`TerrainEstimator()`として生成されるインスタンスです。呼び出し頻度は、歩容(`PeriodicGaitGenerator`)・着地点(`FootholdReferenceGenerator`)と同様、`simulation.py`の内側ループが1回まわるたびに毎回呼ばれます(既定500Hz相当)。

`update_state_and_reference`の実際のコードを読むと、この地形推定の呼び出しは、歩容更新や着地点計算より**先**(関数の一番最初)に実行されています。これまで歩容(`PeriodicGaitGenerator`)→着地点(`FootholdReferenceGenerator`)の順で読んできましたが、実行順としては地形推定が最初です。

## このクラスの役割(全体の中での位置づけ)

`TerrainEstimator`が担当するのは、「**地面がどれだけ傾いているか、ロボットは今どれくらいの高さにいるか**」を、4本の足の位置だけから推定することです。

- 入力：胴体の位置・yaw角、4本の足の位置(実際には後述の通り「離地位置」)、接地状態
- 出力：地形のroll(左右の傾き)・pitch(前後の傾き)・地形の高さ・ロボットの高さの4つの推定値
- 外部センサ(カメラ・IMU等)は一切使わず、**足位置の幾何学的な差分だけ**から傾きを推定する、という設計
- この出力は、後段(`WBInterface.update_state_and_reference`の続き、まだ未解説)で、目標姿勢(`ref_orientation`)や目標速度の回転補正に使われます

対象は `external/Quadruped-PyMPC/quadruped_pympc/helpers/terrain_estimator.py`(114行)です。

---

## 1〜12行：import とコンストラクタ

この関数の役割:roll/pitch/地形高さ/ロボット高さの推定値を0で初期化する。

```python
import numpy as np

class TerrainEstimator:
    def __init__(self) -> None:
        self.terrain_roll = 0
        self.terrain_pitch = 0
        self.terrain_height = 0
        self.robot_height = 0

        self.roll_activated = False
        self.pitch_activated = True
```

- 依存は`numpy`だけで、外部センサ関連のimportは一切ない
- `self.terrain_roll`：地形のroll推定値(rad)。初期値`0`
- `self.terrain_pitch`：地形のpitch推定値(rad)。初期値`0`
- `self.terrain_height`：地形の高さ推定値(m)。初期値`0`
- `self.robot_height`：ロボットの高さ推定値(m)。初期値`0`
- `self.roll_activated`：roll推定を使うかのフラグ(`bool`)。値は`False`固定
- `self.pitch_activated`：pitch推定を使うかのフラグ(`bool`)。値は`True`固定
- **重要な非対称性**：`roll_activated = False`だが`pitch_activated = True`
  - 左右方向の傾き(roll)の推定は既定で**無効化**されている
  - 前後方向の傾き(pitch)の推定だけが既定で有効
  - この非対称の理由は本ファイルの中には書かれていない(未確認)。トロット等の前進歩行では、進行方向の傾き(登り坂・下り坂)の方が重要度が高く、左右の傾きは相対的に軽視されている、という設計判断があった可能性がある(**設計上の解釈**)

---

## 14〜37行：`compute_terrain_estimation`の入口

この関数の役割:4脚の位置の差分から、地形のroll/pitch/高さとロボットの高さを推定する。

```python
def compute_terrain_estimation(
    self, base_position: np.ndarray, yaw: float, feet_pos: dict, current_contact: np.ndarray
) -> [float, float]:
```

- 引数の意味(すべてデフォルト値はなく、呼び出し元が毎回渡す):
  - `base_position`：胴体の現在位置(m、world座標系)
  - `yaw`：胴体の現在のyaw角(rad)
  - `feet_pos`：4脚の位置(m、`dict`。実体は下記の通り離地位置)
  - `current_contact`：4脚分の0/1配列(無次元、現在の接地状態)
- docstringには「roll, pitch」の2つしか書かれていないが、実際の戻り値(113行目)は4つ(roll, pitch, terrain_height, robot_height)。docstringが実装に追いついていない箇所
- `feet_pos`という引数名だが、呼び出し元(`update_state_and_reference`)では`self.frg.lift_off_positions`(前章`read_code_03`で見た、離地位置の記録)が渡されている
  - つまりここで使われる足位置は、**今この瞬間の実際の足位置ではなく、各脚が最後に地面から離れた瞬間の位置**
  - 遊脚中の足は今まさに宙にあるが、ここでの計算にはその足の「過去の離地位置」が使われる、という点に注意が必要

```python
R_W2H = np.array([[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

seg0 = feet_pos["FL"]
seg3 = feet_pos["FR"]
seg6 = feet_pos["RL"]
seg9 = feet_pos["RR"]
```

- world座標系からhorizontal frameへの回転行列。`read_code_02`・`read_code_03`で見たのと同じパターンの行列がここでも個別に定義されている(3ファイルとも共通化されていない)
- `seg0`/`seg3`/`seg6`/`seg9`という変数名は、4脚それぞれ3要素(x,y,z)のベクトルを、もっと大きな配列の一部として扱っていた過去の実装の名残のように見える名前(**設計上の解釈**、確証はない)

---

## 48〜71行：roll/pitchの推定

```python
front_difference = R_W2H @ (seg0 - base_position) - R_W2H @ (seg3 - base_position)
back_difference = R_W2H @ (seg6 - base_position) - R_W2H @ (seg9 - base_position)
left_difference = R_W2H @ (seg0 - base_position) - R_W2H @ (seg6 - base_position)
right_difference = R_W2H @ (seg3 - base_position) - R_W2H @ (seg9 - base_position)
```

- `front_difference`(m)：FL(左前)とFR(右前)の差 → 前側の左右の高さ差
- `back_difference`(m)：RL(左後)とRR(右後)の差 → 後側の左右の高さ差
- `left_difference`(m)：FL(左前)とRL(左後)の差 → 左側の前後の高さ差
- `right_difference`(m)：FR(右前)とRR(右後)の差 → 右側の前後の高さ差
- コメント「TODO: Feet position in base frame?」：開発者自身が、この計算が本当にbase frameで行われるべきか(あるいは既にそうなっているか)に確信を持てていないことが読み取れる

```python
pitch = (
    np.arctan(np.abs(left_difference[2]) / np.abs(left_difference[0] + 0.001))
    + np.arctan(np.abs(right_difference[2]) / np.abs(right_difference[0] + 0.001))
) * 0.5

roll = (
    np.arctan(np.abs(front_difference[2]) / np.abs(front_difference[1] + 0.001))
    + np.arctan(np.abs(back_difference[2]) / np.abs(back_difference[1] + 0.001))
) * 0.5
```

- `pitch`(rad、前後の傾き)：左側の前後差・右側の前後差それぞれについて「高さ差÷前後方向の距離」の逆正接(=傾斜角)を計算し、平均する
- `roll`(rad、左右の傾き)：前側の左右差・後側の左右差それぞれについて同様に計算し、平均する
- 分母に`+0.001`を足しているのは、前後(または左右)の距離がゼロに近いときのゼロ除算を避けるための安全策
- `np.abs(...)`で絶対値を取っているため、この時点では**傾きの符号(登り/下りのどちら向きか)がまだ決まっていない**

```python
if (front_difference[2] * 0.5 + back_difference[2] * 0.5) < 0:
    roll = -roll
if (left_difference[2] * 0.5 + right_difference[2] * 0.5) > 0:
    pitch = -pitch
```

- コメント「TODO: Adjusting what and for what?」：この符号調整の意図そのものが、開発者自身にも明確に文章化されていない
- `front_difference[2]`と`back_difference[2]`の平均(左右の高さ差の平均)が負なら、rollの符号を反転させる。おそらく「右が高いか左が高いか」の向きを、傾いている実際の方向に合わせるための補正と考えられるが、確証は持てない(**未確認**)

```python
if self.roll_activated:
    self.terrain_roll = self.terrain_roll * 0.99 + roll * 0.01
else:
    self.terrain_roll = 0.0

if self.pitch_activated:
    self.terrain_pitch = self.terrain_pitch * 0.99 + pitch * 0.01
else:
    self.terrain_pitch = 0.0
```

- 指数移動平均(EMA)で滑らかに更新する：新しい推定値の影響は1%だけ、99%は過去の値を維持する
- 数値例：仮に地面の傾きが急に5度から0度へ変化しても、この更新式では1ステップあたり0.05度分しか反映されない。500Hzの制御周期なら、63%まで追従するのに約$-\ln(0.37)/0.01\approx99$ステップ、つまり約0.2秒かかる計算になる(等比的な収束のため厳密な整定時間ではないが、目安として)
- **実装上の重要な事実**：`roll_activated=False`(既定)なので、直前のブロックで計算した`roll`という値は、**ここで単純に捨てられ、常に`self.terrain_roll = 0.0`になる**。48〜65行のroll計算そのものは無駄ではないが、その結果は最終的に一切使われない、という状態になっている

---

## 84〜113行：地形の高さとロボットの高さ

```python
z_foot_FL = feet_pos["FL"][2]
z_foot_FR = feet_pos["FR"][2]
z_foot_RL = feet_pos["RL"][2]
z_foot_RR = feet_pos["RR"][2]
```

- 4本の足(こちらも実際には離地位置)のZ座標を取り出す

```python
"""number_foot_in_contact = current_contact[0] + ...
if (number_foot_in_contact != 0):
    z_foot_mean_temp = (z_foot_FL * current_contact[0] + ...) / number_foot_in_contact
    self.terrain_height = self.terrain_height * 0.6 + z_foot_mean_temp * 0.4"""

z_foot_mean_temp = (z_foot_FL + z_foot_FR + z_foot_RL + z_foot_RR) / 4
#self.terrain_height = self.terrain_height * 0.2 + (base_position[2] - z_foot_mean_temp) * 0.8
self.terrain_height = self.terrain_height * 0.2 + (z_foot_mean_temp) * 0.8
```

**実装上の問題点(顕著な箇所)**：

- 三重引用符でコメントアウトされている旧コードは、「今実際に接地している脚だけ」を使って平均を取る、という設計だった(`current_contact`で重み付け)
- 実際に動いている行(100行目)は、**接地・遊脚を問わず4本すべての平均**を単純に取っている。引数`current_contact`は関数に渡されてはいるが、この計算では一切使われていない
- さらに、もう1行コメントアウトされている(101行目)。そちらは`base_position[2] - z_foot_mean_temp`(胴体から足までの相対的な高さ)を使う設計だったが、実際に動いている行(102行目)は`z_foot_mean_temp`そのもの、つまり**胴体位置を考慮しない、足のZ座標の絶対値の平均**をそのまま`terrain_height`としている
- まとめると、現在の`terrain_height`は「4本の脚の離地位置のZ座標(接地・遊脚を区別しない)の指数移動平均」であり、コメントに残る2つの旧設計(接地脚だけの平均、胴体との相対高さ)のどちらとも異なる、より単純化された値になっている

```python
feet_to_base_FL = base_position[2] - feet_pos["FL"][2]
feet_to_base_FR = base_position[2] - feet_pos["FR"][2]
feet_to_base_RL = base_position[2] - feet_pos["RL"][2]
feet_to_base_RR = base_position[2] - feet_pos["RR"][2]
feet_to_base_mean = (feet_to_base_FL + feet_to_base_FR + feet_to_base_RL + feet_to_base_RR) / 4
self.robot_height = self.robot_height * 0.2 + (feet_to_base_mean) * 0.8
```

- `robot_height`は「胴体位置 − 各脚の離地位置Z」の平均。こちらは`terrain_height`とは違い、胴体を基準にした相対的な高さになっている
- 同じ0.2/0.8の重みでEMA更新される

```python
return self.terrain_roll, self.terrain_pitch, self.terrain_height, self.robot_height
```

- 4つの推定値をタプルで返す

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `roll_activated=False`により、計算された`roll`の値が最終的に常に捨てられ`0.0`になる(無駄な計算)
  2. `terrain_height`の計算は、接地脚のみを使う設計・胴体との相対高さを使う設計の両方がコメントアウトされ、現在は「4本すべての離地位置Zの単純平均」というより簡略化された式になっている
  3. `current_contact`引数は関数に渡されるが、実際の計算(80行目以降)では使われていない
  4. docstringの戻り値の説明(roll, pitchの2つ)が、実際の戻り値(4つ)と食い違っている
  5. 符号調整ロジック(68〜71行)の意図が、開発者自身のTODOコメント通り、コード上明確に説明されていない
- 入力に使われる`feet_pos`は、実際の現在の足位置ではなく`FootholdReferenceGenerator`が記録する離地位置である点も、地形推定の精度を考える上で覚えておく価値がある
- 次は同じく`update_state_and_reference`の中から呼ばれる、もう1つの小さなヘルパー`helpers/velocity_modulator.py`を読みます。
