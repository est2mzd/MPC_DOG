# 毎ステップの制約設定 centroidal_nmpc_nominal.py::set_stage_constraint 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.srbd_controller_interface.compute_control(...)  (read_code_07)
          → self.controller.compute_control(...)  (compute_control本体、未読、次章)
              → self.set_stage_constraint(...)   ← 本ファイル、ホライズンの各ステージに対して呼ばれる
```

read_code_08・read_code_09と違い、この関数は**プロセス起動時の1回だけ**ではなく、
`compute_control`から**MPCが解かれるたびに**(既定では5シミュレーションステップに1回)
呼ばれる。read_code_02(歩容)・read_code_03(着地点)・read_code_06(状態集約)と同じ
「毎回呼ばれる」グループに属する。

## この関数の役割(全体の中での位置づけ)

`set_stage_constraint`が担当するのは、read_code_09で「枠だけ」定義した制約(摩擦錐・
着地点box・安定性)に対して、**今の脚位置・目標着地点・接地スケジュールに基づいた
具体的な上下限の数値**を計算し、acadosソルバーへ`constraints_set`で書き込むことです。

- 入力:今の足位置、目標着地点、接地スケジュール、回転行列、外部から与えられた制約(VFA由来、既定では`None`)
- 出力:なし(acadosソルバー内部の状態を書き換える副作用のみ)
- read_code_09で確認した通り、既定設定(`use_foothold_constraints=False`, `use_stability_constraints=False`)ではacadosのOCPに実際に効くのは摩擦錐制約だけである。この関数はそれでも着地点・安定性の制約値を計算するが、後述の通り**その大部分は既定では最終的に使われない**

対象は`external/Quadruped-PyMPC/quadruped_pympc/controllers/gradient/nominal/centroidal_nmpc_nominal.py`
の562〜1047行(486行)です。

---

## 562〜578行:シグネチャとdocstring

```python
def set_stage_constraint(self, constraint, state, reference, contact_sequence, h_R_w, stance_proximity):
    """
    Set the stage constraint for the centroidal NMPC problem. We only consider the stance constraint, and the swing
    constraint up to 2 maximum references.
    ...
    """
```

- `constraint`：外部(VFA/視覚)から与えられた着地点制約。デフォルト値はなく必須引数だが、既定設定(`visual_foothold_adaptation='blind'`)では呼び出し元から`None`が渡されると考えられる(**未確認**、呼び出し元の`compute_control`は未読)
- `state`：read_code_06で組み立てた現在状態の辞書
- `reference`：read_code_06で組み立てた目標状態の辞書
- `contact_sequence`：4脚×ホライズンの接地スケジュール(無次元、0/1)
- `h_R_w`：horizontal frame↔world frameの回転行列(2×2に整形される)
- `stance_proximity`：docstringは`float`と書いているが、実際にはこの引数はこの関数の本体で一度も使われていない(**実装上の問題点**、引数として受け取るだけで未使用)
- docstring通り、この関数が扱うのは「stance中の脚の制約」と「最大2つまでの遊脚の着地予定(touchdown)の制約」の2種類

---

## 579〜653行:接地中の脚のbox制約(4脚分、同一パターンの繰り返し)

```python
FL_contact_sequence = contact_sequence[0]
...
FL_actual_foot = state["foot_FL"]
...
FL_reference_foot = reference["ref_foot_FL"]
...
base = state["position"]
h_R_w = h_R_w.reshape((2, 2))

# FL Stance Constraint
stance_up_constraint_FL = np.array([FL_actual_foot[0], FL_actual_foot[1], FL_actual_foot[2] + 0.002])
stance_up_constraint_FL[0:2] = h_R_w @ (stance_up_constraint_FL[0:2] - base[0:2])
stance_up_constraint_FL[0:2] = stance_up_constraint_FL[0:2] + 0.1
stance_up_constraint_FL[2] = stance_up_constraint_FL[2] + 0.01

stance_low_constraint_FL = np.array([FL_actual_foot[0], FL_actual_foot[1], FL_actual_foot[2] - 0.002])
stance_low_constraint_FL[0:2] = h_R_w @ (stance_low_constraint_FL[0:2] - base[0:2])
stance_low_constraint_FL[0:2] = stance_low_constraint_FL[0:2] - 0.1
stance_low_constraint_FL[2] = stance_low_constraint_FL[2] - 0.01
```

- FL(左前)の例のみ示す。FR・RL・RRについても**全く同じ計算パターンが個別に書かれている**(共通化されていない、4倍の重複コード)
- `stance_up_constraint_FL`/`stance_low_constraint_FL`(m)：接地中の脚について、「今の実際の足位置」を中心に、水平方向`±0.1`m・垂直方向`±0.01`m(それぞれさらに`±0.002`mの余白付き)だけ広げたbox。コメント「制約を無効化できないので、実際の位置を単純に広げたboxを使う」の通り、接地中の脚は「ほぼ動かない」という緩い制約になる

---

## 655〜747行:次の着地予定地点の制約(VFA由来 or 通常の参照値)

```python
if constraint is not None:
    # From the VFA
    first_up_constraint_FL = np.array([constraint[0][0], constraint[1][0], constraint[2][0] + 0.002])
    ...
    first_up_constraint_FL[0:2] = h_R_w @ (first_up_constraint_FL[0:2] - base[0:2])
    first_up_constraint_FL = first_up_constraint_FL + 0.005
else:
    # Constrain taken from the nominal foothold (NO VISION)
    first_up_constraint_FL = np.array(
        [FL_reference_foot[0][0], FL_reference_foot[0][1], FL_reference_foot[0][2] + 0.002]
    )
    first_up_constraint_FL[0:2] = h_R_w @ (first_up_constraint_FL[0:2] - base[0:2]) + 0.15
```

- `constraint is not None`(VFA有効時、既定では通らないと考えられる)：外部から与えられた制約点を中心に`±0.005`mの狭いboxを作る
- `constraint is None`(既定)：read_code_03で計算した`ref_foot_FL`(参照着地点)を中心に`±0.15`mの、VFA版より10倍以上広いboxを作る
- 「VFAがあれば狭い範囲、無ければ広い範囲」という設計は、視覚情報が無い場合は着地点の自由度を大きく残す、という意図と読める(**設計上の解釈**)

---

## 749〜821行:制約の積み上げと、2つ目の着地予定地点(該当する場合のみ)

```python
up_constraint_FL = np.vstack((stance_up_constraint_FL, first_up_constraint_FL))
low_constraint_FL = np.vstack((stance_low_constraint_FL, first_low_constraint_FL))
...

if FL_reference_foot.shape[0] == 2:
    second_up_constraint_FL = np.array(
        [FL_reference_foot[1][0], FL_reference_foot[1][1], FL_reference_foot[1][2] + 0.002]
    )
    second_up_constraint_FL[0:2] = h_R_w @ (second_up_constraint_FL[0:2] - base[0:2]) + 0.15
    ...
    up_constraint_FL = np.vstack((up_constraint_FL, second_up_constraint_FL))
    low_constraint_FL = np.vstack((low_constraint_FL, second_low_constraint_FL))
```

- 「stance中のbox」と「1つ目の着地予定box」を縦に積んで`up_constraint_FL`(最大2行×3列)にする
- ホライズンの中でその脚が**2回目の着地**をする予定があれば(`FL_reference_foot.shape[0]==2`)、同じパターンでもう1段追加する。コメント「2つより多い参照着地点は想定していない」の通り、3回目以降には対応していない
- この4脚×3パターン(stance/1回目/2回目)の重複コードが、ここまでで合計12ブロック(FL/FR/RL/RR × それぞれ最大3種)書かれている

---

## 823〜861行:ホライズンループの準備と、着地点制約の適用可否

```python
ub_friction = self.constr_uh_friction
lb_friction = self.constr_lh_friction

idx_constraint = np.array([0, 0, 0, 0])
if FL_contact_sequence[0] == 0:
    idx_constraint[0] = 1
...

for j in range(0, self.horizon):
    ub_foot_FL = up_constraint_FL[idx_constraint[0]]
    lb_foot_FL = low_constraint_FL[idx_constraint[0]]
    ...
    ub_foot = copy.deepcopy(np.concatenate((ub_foot_FL, ub_foot_FR, ub_foot_RL, ub_foot_RR)))
    lb_foot = copy.deepcopy(np.concatenate((lb_foot_FL, lb_foot_FR, lb_foot_RL, lb_foot_RR)))
    if self.use_foothold_constraints:
        ub_total = np.concatenate((ub_friction, ub_foot))
        lb_total = np.concatenate((lb_friction, lb_foot))
    else:
        ub_total = ub_friction
        lb_total = lb_friction
```

- `idx_constraint`：4脚それぞれ、今どのbox(stance用=0番目、または既に遊脚中なら1番目)を使うかを示すインデックス。今すでに遊脚中の脚は`1`から始める
- ホライズンの各ステージ`j`について、`idx_constraint`が指す段のbox(`ub_foot_FL`等)を取り出し、4脚分連結して`ub_foot`/`lb_foot`(12要素)を作る

**実装上の重大な問題点**：ここまで(579〜855行、約280行)かけて`ub_foot`/`lb_foot`を計算しているにもかかわらず、`self.use_foothold_constraints`(read_code_09で確認した既定`False`)が偽なら、**`ub_total`/`lb_total`には摩擦錐の制約(`ub_friction`/`lb_friction`)だけが入り、`ub_foot`/`lb_foot`は一切使われない**。つまり既定設定では、この関数の大部分の計算(接地中box・着地予定box・2回目着地box)が**毎回計算されるが、結果は最終的に捨てられている**。read_code_03の`touch_down_positions`(計算されるが未使用)と同種の問題だが、こちらは計算量・行数ともにはるかに大きい

---

## 863〜1010行:安定性制約(既定では未実行)とホライズン末尾の分岐(到達不能)

```python
if self.use_stability_constraints:
    ...
    if (FL_contact_sequence[j]==1 and FR_contact_sequence[j]==1 and RL_contact_sequence[j]==1 and RR_contact_sequence[j]==1):
        # FULL STANCE TODO
        ...(全方向±ACADOS_INFTYで無制約)
    elif np.array_equal(FL_contact_sequence, RR_contact_sequence) and np.array_equal(FR_contact_sequence, RL_contact_sequence):
        # TROT
        stability_margin = config.mpc_params['trot_stability_margin']
        ...
    elif np.array_equal(FL_contact_sequence, RL_contact_sequence) and np.array_equal(FR_contact_sequence, RR_contact_sequence):
        # PACE
        stability_margin = config.mpc_params['pace_stability_margin']
        ...
    else:
        # CRAWL BACKDIAGONALCRAWL ONLY
        stability_margin = config.mpc_params["crawl_stability_margin"]
        ...
    ub_total = np.concatenate((ub_total, ub_support))
    lb_total = np.concatenate((lb_total, lb_support))

if j == self.horizon:
    ...
```

- `self.use_stability_constraints`(既定`False`)がTrueのときだけ実行される、支持多角形に関する追加の6本の不等式(read_code_09の`create_stability_constraints`に対応する数値版)
- どの対角ペアに制約をかけるかを、接地パターン(4脚同時=FULL STANCE、対角ペアが同期=TROT、左右ペアが同期=PACE、それ以外=CRAWL)で場合分けする
- `stability_margin`(m)：`config.py`の値。トロット`0.04`、ペース`0.1`、クロール`0.04`(コメントでは「一般に0.02が良い値」とも書かれている)
- **実装上の問題点(到達不能コード)**：`for j in range(0, self.horizon):`というループの範囲は`0`から`self.horizon - 1`までであり、`j`が`self.horizon`と等しくなることは構造上あり得ない。したがって997行目の`if j == self.horizon:`という条件分岐(998〜1010行、「ホライズン末尾では摩擦錐が無いので着地点・安定性制約だけ使う」という処理)は**一度も実行されない、コード上存在するが到達不能なブロック**

---

## 1012〜1041行:acadosへの書き込みと、インデックスの更新

```python
if j == 0:
    self.acados_ocp_solver.constraints_set(j, "uh", ub_friction)
    self.acados_ocp_solver.constraints_set(j, "lh", lb_friction)
if j > 0:
    self.acados_ocp_solver.constraints_set(j, "uh", ub_total)
    self.acados_ocp_solver.constraints_set(j, "lh", lb_total)

self.upper_bound[j] = ub_total.tolist()
self.lower_bound[j] = lb_total.tolist()

if j >= 1:
    if FL_contact_sequence[j] == 0 and FL_contact_sequence[j - 1] == 1:
        if idx_constraint[0] < up_constraint_FL.shape[0] - 1:
            idx_constraint[0] += 1
    ...
```

- `j == 0`(先頭ステージ)は`ub_total`ではなく`ub_friction`だけを設定している。既定設定では`ub_total`も結局`ub_friction`と同じ値のはずなので実質差はないが、着地点・安定性制約が有効なときは先頭ステージだけ意図的に摩擦錐のみにする、という特別扱いになる
- `self.acados_ocp_solver.constraints_set(j, "uh"/"lh", ...)`：acadosソルバーへ、そのステージの制約上下限を直接書き込むAPI呼び出し。ここが「毎ステップ具体的な数値をソルバーに反映する」実処理そのもの
- `self.upper_bound`/`self.lower_bound`(read_code_09で用意した記録用配列)へも同じ値を保存する(可視化・ログ用と考えられる)
- ループの最後で、「1つ前のステージでは接地していたが、このステージで遊脚になった」という遷移を検出すると、その脚の`idx_constraint`を1つ進める(次の着地予定boxへ切り替える)。ただし`up_constraint_FL.shape[0] - 1`を超えないようにキャップされている(用意した段数を超えて参照しない安全策)

---

## 579行・1042〜1046行:関数全体を覆う`try`/`except`(既定では無音で失敗する)

```python
try:
    ...(579〜1041行の全処理)
except:
    if self.verbose:
        print("###WARNING: error in setting the constraints")

return
```

**実装上の重大な問題点**：関数本体のほぼ全て(579〜1041行、463行)が1つの`try`ブロックに
入っており、例外の種類を指定しない**bare except**(`except:`)で受けている。

- `self.verbose`は`config.py`で既定`False`(read_code_09で確認済み)
- つまり、この関数の中でどんな例外(型の不一致、インデックス範囲外、ゼロ除算等)が発生しても、**既定設定では何のログも出さずに黙って`return`する**
- 結果として、その周期は制約が正しく更新されないままacadosソルバーが前回の値で解かれることになるが、呼び出し元(`compute_control`)からはエラーが発生したことが一切分からない
- デバッグ時にこの関数の中身を変更して動作がおかしくなった場合、例外メッセージが出ないため原因の特定が難しくなる、という実務上のリスクがある

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `stance_proximity`引数が受け取られるだけで関数内で一度も使われない
  2. 接地中box・着地予定box(1回目・2回目)の計算(約280行、4脚×3パターンの重複コード)は、`use_foothold_constraints`が既定`False`のため**計算されるが最終的に捨てられる**
  3. `if j == self.horizon:`という分岐が、ループの範囲(`0`〜`horizon-1`)の外側を指しているため**到達不能**
  4. 関数全体を覆う`except:`(bare except)が、`verbose=False`の既定設定では例外を完全に無音化する
- 既定設定でこの関数が実質行っていることは:「4脚の摩擦錐制約の上下限を、ステージ0とそれ以外で同じ値を使って`constraints_set`する」という、コードの分量に対してかなり単純な処理に絞られる
- 次は、この制約設定を含めて実際にOCPを毎周期解く`compute_control`(centroidal_nmpc_nominal.py、約567行)を読みます。
