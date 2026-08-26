# 02 — 速度指令 → 各脚の接地予定（歩容位相と contact_sequence）

日付: 2026-08-25
対象: `external/Quadruped-PyMPC`
関連: [01_execution_order_trace.md](01_execution_order_trace.md)（本ファイルはそのB1-1節の一部を深掘りしたもの）

対象ファイル:
- `quadruped_pympc/config.py`
- `quadruped_pympc/interfaces/wb_interface.py`
- `quadruped_pympc/helpers/periodic_gait_generator.py`

スコープ外（本ファイルでは扱わない）: Raibert着地点生成（`FootholdReferenceGenerator`）、
地形推定（`TerrainEstimator`）、NMPC内部（acados/OCP）、トルク計算
（`compute_stance_and_swing_torque`）。

---

## 変数リファレンス（shape・単位・0/1の意味）

| 変数 | shape | 単位/型 | 意味 | 定義箇所 |
|---|---|---|---|---|
| `step_freq` | スカラー | Hz（1/s） | 歩容の周期 $1/f$ の逆数。位相を1周期進める速さ | `config.py` の `gait_params[gait_name]['step_freq']`。`PeriodicGaitGenerator.step_freq` |
| `duty_factor` | スカラー | 無次元、$[0,1]$ | 1周期のうち接地（stance）している割合 | `config.py` の `gait_params[gait_name]['duty_factor']`。`PeriodicGaitGenerator.duty_factor` |
| `gait_type` | スカラー | int（`GaitType` Enumの`.value`） | `phase_offset` テーブル選択キー。config側もクラス内比較側も`.value`（int）で扱っており、Enumメンバー自体は保持しない | `config.py` の `gait_params[gait_name]['type']` = `GaitType.TROT.value` 等 |
| `phase_offset` | list[4] | 無次元（サイクル数、$\ge 0$。`% 1.0`前は1.0を含みうる） | 脚ごとの初期位相（脚順序 FL, FR, RL, RR、他モジュールと共通） | `periodic_gait_generator.py::reset()` L24–39 |
| `_phase_signal` | `np.ndarray` shape `(4,)` | 無次元、通常 $[0,1)$ | 現在の位相（サイクル内での進み具合）。プロパティ`phase_signal`で読める | `periodic_gait_generator.py` L19, L43 |
| `current_contact` | `np.ndarray` shape `(4,)` | `{0.0, 1.0}` | 1=接地(stance)、0=遊脚(swing)。脚順序 FL, FR, RL, RR | `wb_interface.py` L208–210 |
| `contact_sequence` | `np.ndarray` shape `(4, horizon)` | `{0.0, 1.0}` | 列方向が未来の時間ステップ（列0=現在、列1..horizon-1=未来）、行が脚(FL,FR,RL,RR) | `periodic_gait_generator.py::compute_contact_sequence()` 戻り値 |
| `simulation_dt` | スカラー | 秒 | MuJoCo物理積分の1ステップ時間（500 Hz相当） | `config.py::simulation_params['dt']` = `0.002` |
| `mpc_params['dt']` | スカラー | 秒 | MPCホライズンの離散化幅（`contact_sequence`の列間隔） | `config.py::mpc_params['dt']` = `0.02` |
| `mpc_frequency` | スカラー | Hz | OCPを実際に解く頻度（後述、`PeriodicGaitGenerator`内では未使用） | `config.py::simulation_params['mpc_frequency']` = `100` |
| `horizon` | スカラー | ステップ数 | `contact_sequence`の列数（既定・一様離散化時） | `config.py::mpc_params['horizon']` = `12` |

---

## 1. `step_freq`、`duty_factor`、`gait_type` の意味

`config.py` の `simulation_params['gait_params']`（読んだ値そのまま）:

```python
'gait_params': {'trot':  {'step_freq': 1.35, 'duty_factor': 0.74, 'type': GaitType.TROT.value},
                'crawl': {'step_freq': 0.5,  'duty_factor': 0.8,  'type': GaitType.BACKDIAGONALCRAWL.value},
                'pace':  {'step_freq': 1.4,  'duty_factor': 0.7,  'type': GaitType.PACE.value},
                'bound': {'step_freq': 1.8,  'duty_factor': 0.65, 'type': GaitType.BOUNDING.value},
                'full_stance': {'step_freq': 2, 'duty_factor': 0.65, 'type': GaitType.FULL_STANCE.value},
               },
```

`wb_interface.py` L45–56 がこれを取り出して `PeriodicGaitGenerator` に渡す：

```python
gait_name = cfg.simulation_params['gait']                       # 例: 'trot'
gait_params = cfg.simulation_params['gait_params'][gait_name]
gait_type, duty_factor, step_frequency = (
    gait_params['type'], gait_params['duty_factor'], gait_params['step_freq'],
)
self.pgg = PeriodicGaitGenerator(
    duty_factor=duty_factor, step_freq=step_frequency, gait_type=gait_type, horizon=horizon
)
```

- **`step_freq`**（Hz）: 位相を進める速度。`periodic_gait_generator.py::run()` L53 で
  `self._phase_signal[leg] += dt * new_step_freq` として使われる。つまり
  「1秒あたり `step_freq` 周期分だけ位相が進む」という定義であり、`dt`（秒）× `step_freq`（Hz）
  = 無次元の位相増分（サイクル数）になる。
- **`duty_factor`**（無次元、$[0,1]$）: `run()` L71 の判定 `if self._phase_signal[leg] < self.duty_factor` で
  使われる。位相がこの値未満なら接地(1)、以上なら遊脚(0)。「1周期のうち接地している割合」。
- **`gait_type`**: `reset()` L24–39 の `if/elif` で `phase_offset`（脚ごとの初期位相オフセット表）を選ぶための
  キー。値そのもの（int）は`GaitType` Enumの`.value`であり、`PeriodicGaitGenerator`内の比較も
  `self.gait_type == GaitType.TROT.value` のようにintどうしの比較になっている（Enumオブジェクトとしては
  扱われない）。

---

## 2. 各脚の位相初期値とtrotの位相差

`periodic_gait_generator.py::reset()` L22–45:

```python
def reset(self):
    if self.gait_type == GaitType.TROT.value:
        self.phase_offset = [0.5, 1.0, 1.0, 0.5]
    elif self.gait_type == GaitType.PACE.value:
        self.phase_offset = [0.8, 0.3, 0.8, 0.3]
    elif self.gait_type == GaitType.BOUNDING.value:
        self.phase_offset = [0.5, 0.5, 0.0, 0.0]
    ...
    else:
        self.phase_offset = [0.0, 0.5, 0.5, 0.0]

    self._phase_signal = np.asarray(self.phase_offset)
    self._init = [False] * len(self.phase_offset)
    self.n_contact = len(self.phase_offset)
```

配列の並びは他モジュール（`current_contact`, `swing_time`等）と共通の脚順序
**[FL, FR, RL, RR]**（`simulation.py` L273–277 の `swing_time` マッピング等から確認できる、
コードベース全体で共通の慣例）。

TROTの場合 `phase_offset = [0.5, 1.0, 1.0, 0.5]` = `[FL:0.5, FR:1.0, RL:1.0, RR:0.5]`。
`run()` は位相を `% 1.0` するため（後述）、`1.0 % 1.0 = 0.0` に正規化される。つまり実効値は
`[FL:0.5, FR:0.0, RL:0.0, RR:0.5]` となり、**対角の脚ペア（FL・RR）が位相0.5、もう一方の対角ペア
（FR・RL）が位相0.0**で、両ペアの位相差が0.5（半周期）という一般的なtrotの構造に一致する。

**精度に関する注記**: `reset()` L43 の時点では `self._phase_signal` は `phase_offset` の生値
（`% 1.0`されていない）がそのまま代入されるため、FR・RLは一時的に`1.0`という値を持つ。この値が
`0.0`に正規化されるのは最初に`run()`が呼ばれた時点（`% 1.0`演算、L56）である。実行順序上、
`WBInterface.update_state_and_reference()`は必ず`self.pgg.run(...)`（L202）を呼んでから
`phase_signal`を読み出す（例: L148の`self.wb_interface.pgg.phase_signal`）ため、実際に外部から
観測される値が`1.0`のままになることはない。

また `_init = [False] * 4` で初期化される。`run()`内には`_init[leg]`が`True`の場合の特別分岐
（L61–67、脚ごとの遅延始動を扱うためのコード）があるが、`_init`を`True`にする経路は
`set_phase_signal(..., init=...)`の呼び出しに限られ、確認した範囲（`quadruped_pympc/`,
`simulation/`）では、外部から生きている`self.pgg`インスタンスに対して`init=True`を渡す呼び出しは
見つからなかった（`srbd_batched_controller_interface.py`が使うのは歩容周波数最適化用の別インスタンス
`pgg_temp`であり、こちらも`set_phase_signal`は`init`引数なし＝`init=None`→全脚`False`で呼んでいる）。
したがって、標準（nominal）経路では`run()`のL61–67分岐は実行されず、常にL68以降の
`duty_factor`比較分岐が使われる。

---

## 3. `run()` による位相更新式

`periodic_gait_generator.py::run()` L48–76:

```python
def run(self, dt, new_step_freq):
    contact = np.zeros(self.n_contact)
    for leg in range(self.n_contact):
        self._phase_signal[leg] += dt * new_step_freq      # L53
        self._phase_signal[leg] = self._phase_signal[leg] % 1.0   # L56
        if self._init[leg]:
            ...  # 標準経路では未使用（上記2.参照）
        else:
            if self._phase_signal[leg] < self.duty_factor:  # L71
                contact[leg] = 1
            else:
                contact[leg] = 0
    return contact
```

数式化すると、脚 $i$ の位相 $\phi_i$ について

$$
\phi_i \leftarrow (\phi_i + \Delta t \cdot f) \bmod 1
$$

（$\Delta t$ = `dt`\[秒\]、$f$ = `new_step_freq`\[Hz\]）。`run()`は呼ばれるたびに
この更新を副作用として`self._phase_signal`に対して行い、更新後の接地判定
（下記4.）を戻り値として返す。`new_step_freq`は引数であり`self.step_freq`と別に渡せるが、
標準経路の呼び出し（`wb_interface.py` L202: `self.pgg.run(simulation_dt, self.pgg.step_freq)`）
では常に`self.step_freq`と同じ値が渡される。

---

## 4. 位相から接地0/1を決める条件

`run()` L68–74（標準経路、`_init[leg] == False`側）:

$$
c_i =
\begin{cases}
1 & (\phi_i < \text{duty\_factor}) \\
0 & (\phi_i \ge \text{duty\_factor})
\end{cases}
$$

つまり「1周期の前半 `duty_factor` 割合が接地、残り `1 - duty_factor` が遊脚」という単純な
しきい値判定であり、TROTの`duty_factor=0.74`であれば位相`[0, 0.74)`が接地、`[0.74, 1.0)`が遊脚
となる。

---

## 5. `compute_contact_sequence()` が未来の接地列を作る方法

`periodic_gait_generator.py::compute_contact_sequence()` L93–118（`FULL_STANCE`以外の分岐）:

```python
t_init = np.array(self._phase_signal)     # L101: 現在の位相を退避
init_init = np.array(self._init)          # L102: 現在の_initを退避

contact_sequence = np.zeros((self.n_contact, self.horizon))   # L104: shape (4, horizon)

contact_sequence[:, 0] = self.run(0.0, self.step_freq)        # L107: dt=0なので位相は進めず、"現在"の接地状態を列0に記録

j = 0
for i in range(1, self.horizon):                              # L112
    if i >= contact_sequence_lenghts[j]:
        j += 1
    dt = contact_sequence_dts[j]
    contact_sequence[:, i] = self.run(dt, self.step_freq)      # L116: dt進めながら未来の接地を予測

self.set_phase_signal(t_init, init_init)                       # L117: 退避しておいた"現在"の位相に復元
return contact_sequence
```

ポイント:
- **列0**は現在の接地状態そのもの（`dt=0`）。**列1以降**は`run()`を繰り返し呼ぶことで
  「もし今後この`step_freq`のまま進んだら」という位相を`contact_sequence_dts[j]`ずつ
  実際に先読みして進め（＝`self._phase_signal`を一時的に本当に書き換えて未来をシミュレートする）、
  各時点の接地0/1を記録している。
- 既定（`mpc_params['use_nonuniform_discretization'] = False`）では
  `contact_sequence_dts = [mpc_params['dt']]`（=0.02秒）、
  `contact_sequence_lenghts = [horizon]`（=12）であり（`wb_interface.py` L58–63）、
  ループ中`j`は常に`0`のまま、つまり列間隔は一様に`mpc_params['dt']`（0.02秒）である。
- **重要**: この先読みは`self._phase_signal`を実際に書き換えながら行うため、L117の
  `set_phase_signal(t_init, init_init)`で**呼び出し前の位相に必ず復元**している。これが
  なければ、毎シムステップ呼ぶたびに位相が「未来へ12ステップ分」余分に進んでしまう
  （実際の時間経過と無関係に）。したがって`compute_contact_sequence()`自体は
  `self._phase_signal`の実際の値を変化させない（副作用が相殺される）関数である。

---

## 6. `simulation_dt`、MPCの`dt`、`mpc_frequency`の違い

読んだ3つの時間パラメータは、**別々の場所で・別々の目的**に使われている:

| 変数 | 値（既定） | 使用箇所 | 役割 |
|---|---|---|---|
| `simulation_dt` (`config.py::simulation_params['dt']`) | 0.002秒 | `wb_interface.py` L202: `self.pgg.run(simulation_dt, self.pgg.step_freq)` | **実時間の歩容位相**を1シム物理ステップぶん進める。毎シムステップ（＝500 Hz相当）で必ず呼ばれる |
| `mpc_params['dt']` | 0.02秒 | `wb_interface.py` L59/62 経由で`contact_sequence_dts`に入り、`compute_contact_sequence()`のL115で使用 | **`contact_sequence`の列間隔**（MPCホライズンの離散化幅）。実時間の位相更新には無関係 |
| `mpc_frequency` (`config.py::simulation_params['mpc_frequency']`) | 100 Hz | `quadruped_pympc_wrapper.py` L134: `if step_num % round(1/(mpc_frequency*simulation_dt)) == 0:` | **OCPを実際に解くかどうかの間引き条件**。`PeriodicGaitGenerator`や`WBInterface.update_state_and_reference`のどこにも現れない |

コード上の事実として:

- `WBInterface.update_state_and_reference()`（`pgg.run()`と`compute_contact_sequence()`を含む）は
  `quadruped_pympc_wrapper.py::compute_actions()`の**先頭で無条件に毎シムステップ呼ばれる**
  （L114–131）。つまり`contact_sequence`は500 Hzで毎回作り直される。
- その直後にある`if step_num % round(1/(mpc_frequency*simulation_dt)) == 0:`（L134）は
  **`SRBDControllerInterface.compute_control(...)`の呼び出しだけ**を間引いている
  （既定値では`round(1/(100*0.002)) = 5`ステップに1回、つまり100 Hzで実際にOCPを解く）。

したがって「歩容位相の実時間更新」「未来ホライズンの離散化幅」「OCPを解く頻度」は
コード上完全に独立した3つの数値であり、名前が似ていても混同してはいけない
（`AGENTS.md`の「Non-obvious gotcha」に既述の内容と同じ注意点）。

---

## 7. デフォルトtrotの具体例

既定値: `step_freq = 1.35` Hz、`duty_factor = 0.74`、`phase_offset = [0.5, 1.0, 1.0, 0.5]`
（実効 `[0.5, 0.0, 0.0, 0.5]`）、`simulation_dt = 0.002` 秒。

**周期・接地時間・遊脚時間**（`wb_interface.py` L66, L73の式そのもの）:

$$
T = \frac{1}{f} = \frac{1}{1.35} \approx 0.7407\ \text{s}, \qquad
T_{stance} = \text{duty\_factor} \cdot T \approx 0.74 \times 0.7407 \approx 0.548\ \text{s}, \qquad
T_{swing} = (1-\text{duty\_factor}) \cdot T \approx 0.26 \times 0.7407 \approx 0.193\ \text{s}
$$

**1シムステップあたりの位相増分**:

$$
\Delta\phi = \Delta t \cdot f = 0.002 \times 1.35 = 0.0027\ (\text{サイクル/ステップ})
$$

**起動直後の数ステップの`current_contact`**（`FL, FR, RL, RR`の順）:

| ステップ | $\phi_{FL}$ | $\phi_{FR}$ | $\phi_{RL}$ | $\phi_{RR}$ | `current_contact` |
|---|---|---|---|---|---|
| 0（初期値、`run()`未実行） | 0.5 | 0.0 | 0.0 | 0.5 | — |
| 1（`run()`後） | 0.5027 | 0.0027 | 0.0027 | 0.5027 | `[1,1,1,1]`（全脚 `duty_factor=0.74`未満） |
| ... | ... | ... | ... | ... | `[1,1,1,1]` が続く |

`duty_factor = 0.74` が `0.5` より大きいため、**FL・RRの初期位相`0.5`もまだ接地域内**であり、
起動直後は4脚とも接地（`[1,1,1,1]`）から始まる。その後FL・RRは
$\phi=0.74$ に達するまでの残り $0.74-0.5=0.24$ サイクル
（$0.24/1.35 \approx 0.178$ 秒 $\approx 89$ ステップ）で遊脚へ切り替わり、FR・RLは
$\phi=0.0$ からスタートしているため $T_{stance}\approx0.548$秒（$\approx274$ステップ）
遊脚へ切り替わらない。この非対称な立ち上がりは`phase_offset`を生値のまま初期位相として
採用していることによる**起立直後だけの過渡的な挙動**であり、1周期経過後は定常的なtrot
（FL・RRが同時に接地/遊脚、FR・RLがその半周期ずれで同時に接地/遊脚）に収束する。

---

## データフロー（実際の関数名・変数名）

```text
config.py: simulation_params['gait_params'][gait_name]
  { step_freq, duty_factor, type(=gait_type) }
        │
        ▼ (wb_interface.py L45-56, WBInterface.__init__)
PeriodicGaitGenerator(duty_factor=..., step_freq=..., gait_type=..., horizon=...)
  → self.pgg.phase_offset (reset() L24-39)
  → self.pgg._phase_signal 初期値 = phase_offset (reset() L43)
        │
        ▼ 毎シムステップ (wb_interface.py L202, update_state_and_reference内)
self.pgg.run(simulation_dt, self.pgg.step_freq)
  → self._phase_signal[leg] = (φ + simulation_dt*step_freq) % 1.0   (run() L53,56)
  → 戻り値 contact[leg] = 1 if φ < duty_factor else 0                (run() L71-74)
        │
        ▼ 直後 (wb_interface.py L203-205)
contact_sequence = self.pgg.compute_contact_sequence(contact_sequence_dts, contact_sequence_lenghts)
  → 内部で self.run(0.0,...) を列0、self.run(mpc_dt,...) を列1..horizon-1に適用
  → 呼び出し前の位相へ set_phase_signal(t_init, init_init) で復元          (compute_contact_sequence() L101-118)
        │
        ▼ (wb_interface.py L207-210)
self.current_contact = np.array([contact_sequence[0][0], contact_sequence[1][0],
                                  contact_sequence[2][0], contact_sequence[3][0]])
        │
        ▼ update_state_and_reference() の戻り値として (wb_interface.py L305)
state_current, ref_state, contact_sequence, step_height, optimize_swing
        │
        ▼ (quadruped_pympc_wrapper.py L143-151、mpc_frequencyでゲートされた分岐内でのみ)
self.srbd_controller_interface.compute_control(
    state_current, ref_state, contact_sequence, inertia,
    self.wb_interface.pgg.phase_signal, self.wb_interface.pgg.step_freq, optimize_swing)
  → NMPC側へ contact_sequence が渡る（以降はスコープ外）
```
