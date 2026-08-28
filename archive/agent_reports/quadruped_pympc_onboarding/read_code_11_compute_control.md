# OCPを解く centroidal_nmpc_nominal.py::compute_control 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.srbd_controller_interface.compute_control(...)  (read_code_07)
          → self.controller.compute_control(...)   ← 本ファイル、MPCを解く本体
              → self.perform_scaling(...)    ← 本ファイル
              → self.set_warm_start(...)     ← 本ファイル(use_warm_start=Trueのときのみ、既定では未実行)
              → self.set_stage_constraint(...) (read_code_10、条件付き、下記参照)
              → self.acados_ocp_solver.solve()  (acados本体、未読)
```

read_code_09・read_code_10と同じく、この章の関数は**MPCが解かれるたびに**(既定では
5シミュレーションステップに1回)呼ばれる。この関数(`compute_control`)自体が、
`SRBDControllerInterface.compute_control`(read_code_07)から直接呼ばれる、勾配ベース
MPCの中心処理である。

## この章が扱う3つの関数の役割

- `set_warm_start`:前回の解を初期推定値として使い、ソルバーの収束を助ける(既定では未使用)
- `perform_scaling`:胴体位置を原点に揃えて、数値的に解きやすくする
- `compute_control`:参照値・パラメータ・制約をacadosへ詰め込み、ソルバーを解いて結果を取り出す。**この章の中心**

対象は`centroidal_nmpc_nominal.py`の1048〜1705行(658行)です。

---

## 1048〜1113行:`set_warm_start`(既定では呼ばれない)

この関数の役割:各ホライズン段の状態の一部(4脚の目標着地点に対応する成分)を、前回のacados内部状態の代わりに参照値で置き換えて、ウォームスタートの精度を上げる。

```python
def set_warm_start(self, state_acados, reference, FL_contact_sequence, FR_contact_sequence, RL_contact_sequence, RR_contact_sequence):
    idx_ref_foot_to_assign = np.array([0, 0, 0, 0])
    for j in range(self.horizon):
        warm_start = copy.deepcopy(self.acados_ocp_solver.get(j, "x"))
        ...(接地→遊脚の遷移でidx_ref_foot_to_assignを進める)
        warm_start[8] = state_acados[8]
        if idx_ref_foot_to_assign[0] == 0:
            warm_start[12:15] = state_acados[12:15].reshape((3,))
        else:
            warm_start[12:15] = reference["ref_foot_FL"][0]
        ...
        self.acados_ocp_solver.set(j, "x", warm_start)
```

- `state_acados`(m等、30要素)：現在の状態をacados形式に並べたベクトル。デフォルト値はなく必須引数
- `reference`：目標状態の辞書。デフォルト値はなく必須引数
- `FL/FR/RL/RR_contact_sequence`：各脚の接地スケジュール(無次元)。デフォルト値はなく必須引数
- `warm_start[8]`：yaw角の成分だけ、常に現在の実測値で上書きする
- 脚の位置成分は、まだ接地中(`idx==0`)なら現在の実測値、遊脚に入っていれば目標着地点で上書きする
- **read_code_09で確認した通り`self.use_warm_start`は既定`False`。この関数は`compute_control`内で`if self.use_warm_start:`の中でしか呼ばれないため、既定設定では一度も実行されない**

---

## 1116〜1135行:`perform_scaling`

この関数の役割:胴体位置を原点(0,0,0)に平行移動し、状態・参照値・着地点をすべてその原点基準に変換する。

```python
def perform_scaling(self, state, reference, constraint=None):
    self.initial_base_position = copy.deepcopy(state["position"])
    reference = copy.deepcopy(reference)
    state = copy.deepcopy(state)

    reference["ref_position"] = reference["ref_position"] - state["position"]
    reference["ref_foot_FL"] = reference["ref_foot_FL"] - state["position"]
    ...
    state["foot_FL"] = state["foot_FL"] - state["position"]
    ...
    state["position"] = np.array([0, 0, 0])

    return state, reference, constraint
```

- `state`：現在状態の辞書。デフォルト値はなく必須引数
- `reference`：目標状態の辞書。デフォルト値はなく必須引数
- `constraint`：外部制約(VFA由来)。デフォルト`None`
- `self.initial_base_position`(m)：スケーリング前の胴体位置を退避しておく(read_code_09の`__init__`で`[0,0,0]`初期化されていた変数)。`compute_control`の最後(1693〜1698行)で、この値を使って結果を元の座標系へ戻す(decentering)
- コメント「TODO: Docstringが必要。辞書が特定のキーを持つ前提はあまりPythonicではない」と、開発者自身がこの設計への疑問を残している
- 効果:重心位置を常に原点付近に保つことで、OCPが扱う数値の桁を絶対座標(数百m単位になりうる)ではなく相対座標(数m単位)に抑え、数値的な精度・収束性を改善すると考えられる(**設計上の解釈**)

---

## 1138〜1162行:`compute_control`の入口

この関数の役割:参照値・パラメータ・制約をacadosソルバーへ設定し、`solve()`を呼び、結果(GRF・着地点・次状態)を取り出して返す。

```python
def compute_control(
    self, state, reference, contact_sequence, constraint=None,
    external_wrenches=np.zeros((6,)),
    inertia=config.inertia.reshape((9,)),
    mass=config.mass,
):
```

- `state`/`reference`/`contact_sequence`：デフォルト値はなく必須引数
- `constraint`：デフォルト`None`
- `external_wrenches`(力3+モーメント3、6要素)：デフォルト`np.zeros((6,))`
- `inertia`(kg·m²、9要素)：デフォルト`config.inertia.reshape((9,))`
- `mass`(kg)：デフォルト`config.mass`(Go2なら`15.019`)

**実装上の問題点(read_code_07と同種)**：`external_wrenches`・`inertia`の2つのデフォルト値が、`np.zeros((6,))`・`config.inertia.reshape((9,))`という**式の評価結果**になっている。Pythonの関数デフォルト引数は関数定義時(モジュール読み込み時)に**1度だけ**評価されるため、これらの配列は複数回の呼び出しで**同じオブジェクトが使い回される**。read_code_07で指摘した`SRBDControllerInterface.compute_control`の`external_wrenches`と同じ落とし穴が、この関数にも(`inertia`も含めて)存在する

```python
FL_contact_sequence = contact_sequence[0]
...
FL_previous_contact_sequence = self.previous_contact_sequence[0]
...
state, reference, constraint = self.perform_scaling(state, reference, constraint)
```

- `self.previous_contact_sequence`(read_code_09で`__init__`初期化)から4脚分取り出すが、**この後この4変数(`FL_previous_contact_sequence`等)は関数内で一度も使われていない**(**実装上の問題点**、代入されるだけの未使用変数)

---

## 1163〜1235行:参照値(`yref`)をホライズン全段に詰める

```python
idx_ref_foot_to_assign = np.array([0, 0, 0, 0])
for j in range(self.horizon):
    yref = np.zeros(shape=(self.states_dim + self.inputs_dim,))
    yref[0:3] = reference["ref_position"]
    yref[3:6] = reference["ref_linear_velocity"]
    yref[6:9] = reference["ref_orientation"]
    yref[9:12] = reference["ref_angular_velocity"]
    yref[12:15] = reference["ref_foot_FL"][idx_ref_foot_to_assign[0]]
    ...
    # (接地→遊脚の遷移でidx_ref_foot_to_assignを進める、read_code_10のidx_constraintと同種のロジック)

    number_of_legs_in_stance = np.array([FL_contact_sequence[j], ...]).sum()
    if number_of_legs_in_stance == 0:
        reference_force_stance_legs = 0
    else:
        reference_force_stance_legs = (mass * self.centroidal_model.gravity_constant) / number_of_legs_in_stance
    reference_force_fl_z = reference_force_stance_legs * FL_contact_sequence[j]
    ...
    yref[44] = reference_force_fl_z
    yref[47] = reference_force_fr_z
    yref[50] = reference_force_rl_z
    yref[53] = reference_force_rr_z

    self.acados_ocp_solver.set(j, "yref", yref)
```

- `yref`(54要素=状態30+入力24)：read_code_09の`ocp.cost.Vx`/`Vu`が「状態と入力をそのままコストの対象にする」設定だったことに対応する、実際の目標値ベクトル
- 位置・速度・姿勢・角速度・4脚の目標着地点を、read_code_08で確認した状態の並び順通りに詰める
- **GRFの目標値(z成分のみ)を計算している点が重要**：`reference_force_stance_legs = 質量×重力 ÷ 接地脚数`、つまり「ロボットの重さを、今接地している脚の数で均等に割った分の力」を各脚の目標鉛直GRFとする。これは物理的に理にかなった目標値で、**この計算は実際に`yref`(44,47,50,53番目=入力側のGRF z成分)へ反映され、使われている**(後述の1666〜1684行にある「似ているが結果的に捨てられる」同種の計算とは違う)
- 入力側のx,y成分の目標値(GRF・脚速度)は明示的に設定されないため`0`のまま(コスト関数の重みは既にread_code_09で確認済み)

```python
yref_N = np.zeros(shape=(self.states_dim,))
yref_N[0:3] = reference["ref_position"]
...
self.acados_ocp_solver.set(self.horizon, "yref", yref_N)
```

- `yref_N`(30要素)：ホライズン最終段(`N`番目)の終端コスト用の参照値。入力が無いため状態分(30)だけ

---

## 1237〜1281行:`stance_proximity`の計算(実質すべて0になる)

```python
stance_proximity_FL = np.zeros((self.horizon,))
...
for j in range(self.horizon):
    if FL_contact_sequence[j] == 0:
        if (j + 1) < self.horizon:
            if FL_contact_sequence[j + 1] == 1:
                stance_proximity_FL[j] = 1 * 0
        if (j + 2) < self.horizon:
            if FL_contact_sequence[j + 1] == 0 and FL_contact_sequence[j + 2] == 1:
                stance_proximity_FL[j] = 1 * 0
    ...(FR/RL/RRも同様)
```

**実装上の重大な問題点**：コメントには「接地の直前は着地点最適化を無効化する(実際の足はそんなに速く動けないため)」という明確な意図が書かれている。しかし条件が成立したときに実際に代入される値は`1 * 0`、すなわち**常に`0`**である。`stance_proximity_FL`は最初から`np.zeros`で初期化されており、この`if`ブロック全体が「特定の条件でだけ`0`を代入する」という、**結果的に何もしていない処理**になっている。おそらく元は`1`(有効化)を代入する設計だったのが、デバッグや無効化のために`1 * 0`へ書き換えられ、そのまま残っていると考えられる(**設計上の解釈**)。この`stance_proximity`は、read_code_08の`forward_dynamics`で`(1-stance_proximity_i)`として脚速度に掛かる係数だったが、常に`0`である以上、**この係数は常に`1`になり、実質的に無効化されている**

---

## 1283〜1330行:パラメータ(`p`)をホライズン全段に詰める

```python
for j in range(self.horizon):
    if (config.mpc_params['external_wrenches_compensation']
        and config.mpc_params['external_wrenches_compensation_num_step']
        and j < config.mpc_params['external_wrenches_compensation_num_step']):
        external_wrenches_estimated_param = copy.deepcopy(external_wrenches)
        external_wrenches_estimated_param = external_wrenches_estimated_param.reshape((6,))
    else:
        external_wrenches_estimated_param = np.zeros((6,))

    param = np.array([
        FL_contact_sequence[j], FR_contact_sequence[j], RL_contact_sequence[j], RR_contact_sequence[j],
        mu,
        stance_proximity_FL[j], stance_proximity_FR[j], stance_proximity_RL[j], stance_proximity_RR[j],
        state["position"][0], state["position"][1], state["position"][2],
        state["orientation"][2],
        external_wrenches_estimated_param[0], ..., external_wrenches_estimated_param[5],
        inertia[0], ..., inertia[8],
        mass,
    ])
    self.acados_ocp_solver.set(j, "p", copy.deepcopy(param))
```

- `mu`(無次元)：`config.py`の`mpc_params['mu']`。既定`0.5`
- `config.mpc_params['external_wrenches_compensation']`は既定`True`、`external_wrenches_compensation_num_step`は既定`15`。つまり既定では、ホライズンの最初の15ステージ(ホライズン自体は12ステージなので実質**全段**)で`external_wrenches`パラメータを反映する
- ただし、この`compute_control`を呼ぶ側(`SRBDControllerInterface.compute_control`、read_code_07)の呼び出しでは`external_wrenches`引数が渡されておらず、上で確認したデフォルト値`np.zeros((6,))`がそのまま使われる。**つまり「外力補償の仕組み」自体は既定で有効だが、補償すべき実際の外力の推定値がどこからも供給されないため、常に0を補償している(何もしていないのと同じ)**、という状態になっている(**設計上の解釈**、外力推定器が別途どこかにあるかは未確認)
- `param`(29要素)：read_code_08で確認したパラメータの並び順(接地4+摩擦1+接地近接4+胴体位置3+yaw1+外力6+慣性9+質量1)にそのまま対応する

---

## 1332〜1346行:接地していない脚の初期位置を参照着地点へ「テレポート」

```python
if FL_contact_sequence[0] == 0:
    state["foot_FL"] = reference["ref_foot_FL"][0]
...
```

- コメント：「ロボットの着地点を前回の最適着地点へテレポートさせる。そうしないと、着地の瞬間に一切考慮されない着地点を最適化してしまう。高さは常にVFAから来る」
- 今遊脚中(`contact_sequence[0]==0`)の脚については、初期状態として渡す足位置を**実測値ではなく参照着地点**に置き換える。この後の初期状態制約(1419〜1420行)に反映される

---

## 1348〜1403行:積分誤差の更新(既定では未実行)

- `self.use_integrators`(read_code_09で既定`False`)がTrueのときだけ実行される
- 実行される場合、高さ・速度・roll・pitchの誤差を`alpha_integrator`(既定`0.1`)倍して積算し、`integrator_cap`(既定`[0.5, 0.2, 0.2, 0.0, 0.0, 1.0]`)で上下限にクリップする
- **既定では実行されないため、`self.integral_errors`は`__init__`で初期化された全`0`のまま変化しない**。read_code_08で確認した状態の積分項(6要素)は、常に`0`のまま推移することになる

---

## 1405〜1439行:初期状態制約とウォームスタート・制約設定の呼び出し

```python
state_acados = np.concatenate((
    state["position"], state["linear_velocity"], state["orientation"], state["angular_velocity"],
    state["foot_FL"], state["foot_FR"], state["foot_RL"], state["foot_RR"],
    self.integral_errors,
)).reshape((self.states_dim, 1))
self.acados_ocp_solver.set(0, "lbx", state_acados)
self.acados_ocp_solver.set(0, "ubx", state_acados)

if self.use_warm_start:
    self.set_warm_start(...)

h_R_w = np.array([np.cos(yaw), np.sin(yaw), -np.sin(yaw), np.cos(yaw)])
if self.use_foothold_constraints or self.use_stability_constraints:
    stance_proximity = np.vstack((stance_proximity_FL, stance_proximity_FR, stance_proximity_RL, stance_proximity_RR))
    self.set_stage_constraint(constraint, state, reference, contact_sequence, h_R_w, stance_proximity)
```

- `state_acados`(30要素)：現在状態を、read_code_08の状態順で1本のベクトルに詰めたもの
- `lbx`/`ubx`を両方同じ値に設定する、というのがacadosでの「初期状態を固定する」標準的なやり方(上下限を同じにして等式制約にする)
- `self.use_warm_start`は既定`False`のため`set_warm_start`は呼ばれない

**極めて重要な発見**:`self.set_stage_constraint`(read_code_10で読んだ486行の関数)は、**`self.use_foothold_constraints or self.use_stability_constraints`が真のときしか呼ばれない**。read_code_09で確認した通り、この2つのフラグは**どちらも既定`False`**。

- つまり**既定設定では、read_code_10で読んだ`set_stage_constraint`関数は、ホライズンのどのステージに対しても一度も呼ばれない**。read_code_10では「関数の中身の一部(着地点box計算)が計算されても使われない」という問題を指摘したが、実際には既定設定では**関数自体が丸ごと呼ばれていない**、というさらに上位の事実がここで判明した。read_code_10の内容(関数の中身の解説)自体は、`use_foothold_constraints`または`use_stability_constraints`を有効にしたときの動作として引き続き有効だが、「既定のトロット歩行でこの関数が実行されているか」という問いへの答えは「**呼ばれてすらいない**」が正しい

---

## 1441〜1456行:ソルバーを解く

```python
if self.use_RTI:
    self.acados_ocp_solver.options_set("rti_phase", 2)
    status = self.acados_ocp_solver.solve()
else:
    status = self.acados_ocp_solver.solve()

control = self.acados_ocp_solver.get(0, "u")
optimal_GRF = control[12:]
```

- `self.use_RTI`は既定`False`のため`else`側、つまり通常の`solve()`が呼ばれる
- `status`(`int`)：ソルバーの終了コード
- `control`(24要素)：先頭ステージ(`stage=0`)の最適入力。read_code_08の入力順(脚速度12+脚力12)に従い、`control[12:]`が**GRF(12要素)**

---

## 1460〜1638行:次の着地点の抽出(4脚分の繰り返し、FLのみ例示)

```python
if FL_contact_sequence[0] == 1:
    optimal_foothold[0] = state["foot_FL"]
    optimal_footholds_assigned[0] = True
...

for j in range(1, self.horizon):
    if FL_contact_sequence[j] != FL_contact_sequence[j - 1] and not optimal_footholds_assigned[0]:
        optimal_foothold[0] = self.acados_ocp_solver.get(j, "x")[12:15]
        optimal_footholds_assigned[0] = True
        # 参照(またはVFA制約)の範囲でクリップ
        ...
        optimal_foothold[0][0:2] = np.clip(optimal_foothold[0][0:2], first_low_constraint_FL[0:2], first_up_constraint_FL[0:2])
        ...
```

- 今接地中(`contact_sequence[0]==1`)の脚は、着地点として単純に「今の実測位置」を使う(最適化しない、動いていないので当然)
- 遊脚中の脚は、ホライズンの中で最初に接地状態が変化するステージを探し、その時点のOCP予測状態(`x[12:15]`等、脚位置の成分)を次の着地点として採用する
- その値を、参照着地点(または`constraint`があればVFA由来の制約)を中心とした`±0.15`m(またはVFA時`±0.005`程度)の範囲で`np.clip`する。read_code_10で見た制約値の計算と同じ数値(`0.15`/`0.002`)がここでも独立に再計算されている(共通化されていない)
- 4脚とも全く同じパターンが個別に書かれている(4倍の重複コード、read_code_10と同種の構造)

```python
if optimal_footholds_assigned[0] == False:
    optimal_foothold[0] = reference["ref_foot_FL"][0]
...
```

- ホライズン内で一度も接地状態が変化しなかった脚(ずっと遊脚のまま)は、参照着地点をそのまま使う

---

## 1640〜1650行:次状態の抽出

```python
if config.mpc_params['dt'] <= 0.02 or (config.mpc_params['use_nonuniform_discretization'] and config.mpc_params['dt_fine_grained'] <= 0.02):
    optimal_next_state_index = 2
else:
    optimal_next_state_index = 1
optimal_next_state = self.acados_ocp_solver.get(optimal_next_state_index, "x")[0:24]
```

- `config.mpc_params['dt']`は既定`0.02`、条件`<= 0.02`を満たすため`optimal_next_state_index = 2`
- つまり既定設定では、**ホライズンの先頭(stage 0)ではなく2つ先(stage 2)の予測状態**を「次状態」として採用する。`dt=0.02`秒刻みで2ステージ先は`0.04`秒後に相当し、この関数が呼ばれる周期(MPCは5シミュレーションステップ=`0.01`秒どと、read_code_09/NN参照)より粗いホライズン刻みとの整合を取るための調整と考えられる(**設計上の解釈**)
- `[0:24]`：30次元の状態のうち積分項(24〜29)を除いた24要素だけを取り出す。`__init__`で見た`self.optimal_next_state = np.zeros((24,))`の次元と一致する

---

## 1652〜1690行:ソルバー失敗時のフォールバック(計算されるが捨てられる値がある)

```python
if status == 1 or status == 4:
    if FL_contact_sequence[0] == 0:
        optimal_foothold[0] = reference["ref_foot_FL"][0]
    ...
    number_of_legs_in_stance = np.array([...]).sum()
    reference_force_stance_legs = (mass * self.centroidal_model.gravity_constant) / number_of_legs_in_stance
    reference_force_fl_z = reference_force_stance_legs * FL_contact_sequence[0]
    ...
    optimal_GRF = np.zeros((12,))
    optimal_GRF[2] = reference_force_fl_z
    optimal_GRF[5] = reference_force_fr_z
    optimal_GRF[8] = reference_force_rl_z
    optimal_GRF[11] = reference_force_rr_z

    optimal_GRF = self.previous_optimal_GRF
    self.reset()

self.previous_optimal_GRF = optimal_GRF
self.previous_status = status
self.previous_contact_sequence = contact_sequence
```

**実装上の重大な問題点**：ソルバーのステータスが`1`または`4`(失敗系)のとき、コードは「体重を接地脚数で割った、鉛直方向だけのGRF」を丁寧に計算して`optimal_GRF`へ代入している(`optimal_GRF[2]=...`, `[5]=...`, `[8]=...`, `[11]=...`)。ところが、その**直後の行(`optimal_GRF = self.previous_optimal_GRF`)でこの計算結果を完全に上書きしてしまう**。つまり実際にソルバー失敗時に使われるのは「体重按分のGRF」ではなく「**前回成功した周期のGRF**」であり、直前の十数行の計算はすべて無駄になる。read_code_10の`ub_foot`/`lb_foot`と同じ「計算されるが使われない」パターンが、ここでも見つかる

- `self.reset()`(read_code_09で確認済み)：失敗時に呼ばれ、`build=False, generate=False`でacadosソルバーを作り直す(コード生成・コンパイルはせず、既存の共有ライブラリを再利用する軽い再構築)

---

## 1692〜1705行:座標系を元に戻して返す

```python
optimal_foothold[0] = optimal_foothold[0] + self.initial_base_position
...
optimal_next_state[0:3] = optimal_next_state[0:3] + self.initial_base_position
optimal_next_state[12:15] = optimal_foothold[0]
...
return optimal_GRF, optimal_foothold, optimal_next_state, status
```

- `perform_scaling`で原点へ移動していた分を、`self.initial_base_position`を足し戻すことで元のworld座標系へ戻す(decentering)
- `optimal_next_state`の脚位置成分(12〜24)を、計算し直した`optimal_foothold`で上書きする
- 4つの値(GRF・着地点・次状態・ステータス)を返す。read_code_07の`SRBDControllerInterface.compute_control`が、この戻り値のうち先頭3つを受け取り、4つ目(`status`)は`_`で捨てていた

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `set_warm_start`・`set_stage_constraint`の呼び出し条件フラグ(`use_warm_start`, `use_foothold_constraints`, `use_stability_constraints`)がすべて既定`False`のため、**既定のトロット歩行ではこの2つの関数がそもそも呼ばれない**(read_code_10で読んだ内容は「有効化したときの動作」として理解する必要がある)
  2. `compute_control`のデフォルト引数`external_wrenches`/`inertia`が、read_code_07と同じ「可変・式評価済みのデフォルト引数」の落とし穴になっている
  3. `FL/FR/RL/RR_previous_contact_sequence`が代入されるだけで一度も使われない
  4. `stance_proximity_FL`等への代入が常に`1*0=0`になっており、意図されていたはずの「接地間近では着地点最適化を弱める」機能が実質無効化されている
  5. `external_wrenches_compensation`は既定`True`だが、実際の外力推定値がどこからも供給されないため、常に0を補償するだけの空回りになっている
  6. ソルバー失敗時のフォールバックGRF(体重按分計算)が、直後に`previous_optimal_GRF`で上書きされて捨てられる
- 確認できた、実際に使われている重要な事実:
  - GRFの目標値(z成分)は「体重÷接地脚数」で計算され、`yref`へ正しく反映される(こちらは捨てられない)
  - `optimal_next_state`は既定`dt=0.02`のとき、ホライズンの2ステージ先(0.04秒後)の予測を採用する
  - ソルバー失敗時は「前回成功したGRFを使い続け、ソルバーを再構築する」という、フォールバックとして機能する経路が実際に使われる
- これで`centroidal_nmpc_nominal.py`(read_code_09・10・11)を読み終えました。次は、read_code_07で確認したGRFマスク処理の後、そのGRFを実際の関節トルクへ変換する`wb_interface.py::compute_stance_and_swing_torque`(WBC、まだ未解説)に進みます。
