# 02 — 歩容パラメータから各脚の接地予定まで

日付: 2026-08-25

対象: `external/Quadruped-PyMPC`

関連: [01_execution_order_trace.md](01_execution_order_trace.md) の B1-1 節

対象ファイル:

- `quadruped_pympc/config.py`
- `quadruped_pympc/interfaces/wb_interface.py`
- `quadruped_pympc/helpers/periodic_gait_generator.py`

スコープ外:

- Raibert則による着地点生成
- 地形推定
- NMPCの定式化とacados
- GRFから関節トルクへの変換

---

## 0. この章の結論

デフォルト経路の接地予定は、次の流れで生成される。

```text
gait_type ──→ 各脚の位相差 phase_offset
step_freq ──→ 位相を進める速さ
duty_factor → 位相から接地/遊脚を分ける境界
simulation_dt → 現在位相を1シミュレーションステップ進める
mpc dt ──────→ 現在位相から未来の接地状態を先読みする間隔

                     ↓

phase_signal → current_contact → contact_sequence → NMPC
```

### コードから確認できた事実

- `PeriodicGaitGenerator.run()` は、目標並進速度や目標角速度を受け取らない。
- 通常の接地予定は、歩容パラメータと時間から周期的に生成される。
- `contact_sequence[:, 0]` は現在の接地状態で、列1以降は未来の予測である。
- `current_contact` は `contact_sequence[:, 0]` から作られる。
- 歩容位相はシミュレーション周期で更新されるが、OCPは `mpc_frequency` に従って間引いて解かれる。

### 注意すべき解釈

「速度指令 → 接地予定」という直列の因果関係ではない。デフォルト経路では、速度指令と周期的な接地予定は別々に生成され、後段の着地点生成やNMPCで一緒に使われる。

速度指令が接地予定へ影響する可能性があるのは、`start_and_stop_activated` や歩容周波数最適化などの追加機能が有効な場合である。本章では扱わない。

---

## 1. 主要変数

脚配列の順番は、コード全体で使用される `FL, FR, RL, RR` とする。

| 変数 | shape | 単位・値 | 意味 |
|---|---:|---|---|
| `step_freq` | scalar | Hz | 位相が1秒間に何周期進むか |
| `duty_factor` | scalar | 無次元 | 1周期中に接地している割合 |
| `gait_type` | scalar | `GaitType.*.value` | 各脚の位相差を選ぶ識別子 |
| `phase_offset` | `(4,)` | cycle | 各脚の初期位相 |
| `_phase_signal` | `(4,)` | 通常 `[0,1)` | 各脚の現在位相 |
| `current_contact` | `(4,)` | 0または1 | 1=接地、0=遊脚 |
| `contact_sequence` | `(4, horizon)` | 0または1 | 各脚の現在および未来の接地予定 |
| `simulation_dt` | scalar | s | シミュレーション1ステップの時間 |
| `mpc_params['dt']` | scalar | s | 接地予定を未来へ先読みする時間間隔 |
| `mpc_frequency` | scalar | Hz | OCPを解く頻度 |

添付文書が参照したコードでは、デフォルトtrotは次の値になっている。

```python
step_freq = 1.35
duty_factor = 0.74
gait_type = GaitType.TROT.value
simulation_dt = 0.002
mpc_dt = 0.02
horizon = 12
mpc_frequency = 100
```

これらはリポジトリのバージョンや設定変更によって変わり得る。数値を利用するときは、手元の `config.py` を正とする。

---

## 2. 歩容を決める3つのパラメータ

### 2.1 `step_freq`

`step_freq` は、歩容位相が1秒間に進む周期数である。

歩容周期は次式で求められる。

$$
T_{gait}=\frac{1}{f_{step}}
$$

`f_step = step_freq` であり、単位はHzである。

コードでは、`PeriodicGaitGenerator.run()` が次の計算に使用する。

```python
self._phase_signal[leg] += dt * new_step_freq
```

### 2.2 `duty_factor`

`duty_factor` は、1周期のうち脚が接地している割合である。

$$
T_{stance}=dT_{gait}
$$

$$
T_{swing}=(1-d)T_{gait}
$$

ここで、`d = duty_factor` である。

### 2.3 `gait_type`

`gait_type` は、脚ごとの位相差 `phase_offset` を選ぶために使われる。

`PeriodicGaitGenerator.reset()` では、trotに対して次の値が設定される。

```python
self.phase_offset = [0.5, 1.0, 1.0, 0.5]
```

脚順序を加えると次のようになる。

| 脚 | FL | FR | RL | RR |
|---|---:|---:|---:|---:|
| `phase_offset` | 0.5 | 1.0 | 1.0 | 0.5 |
| 1周期で正規化した値 | 0.5 | 0.0 | 0.0 | 0.5 |

したがって、対角脚の `FL・RR` が同位相、もう一方の対角脚 `FR・RL` が同位相となり、2つの対角ペアの位相差は0.5周期である。

### 事実と解釈

**事実:** `reset()` の直後、FRとRLの内部値は一時的に `1.0` である。最初の `run()` で剰余演算が行われ、`0.0`付近へ正規化される。

**解釈:** `1.0` と `0.0` は周期上の同じ位置を表す。そのため、`1.0`を使ったこと自体が特殊な起動過渡を作っているわけではない。

---

## 3. 位相の更新

標準経路では、`WBInterface.update_state_and_reference()` が毎シミュレーションステップ、次を呼び出す。

```python
self.pgg.run(simulation_dt, self.pgg.step_freq)
```

脚 $i$ の位相を $\phi_i$ とすると、更新式は次のとおりである。

$$
\phi_{i,k+1}=(\phi_{i,k}+\Delta t_{sim}f_{step})\bmod 1
$$

| 記号 | コード | 意味 |
|---|---|---|
| $\phi_i$ | `_phase_signal[leg]` | 脚 $i$ の位相 |
| $\Delta t_{sim}$ | `simulation_dt` | シミュレーション刻み |
| $f_{step}$ | `self.pgg.step_freq` | 歩容周波数 |

デフォルト値では、1シミュレーションステップあたりの位相増分は次になる。

$$
\Delta\phi=0.002\times1.35=0.0027
$$

---

## 4. 位相から接地状態への変換

`run()` は、位相更新後に各脚の接地状態を判定する。

$$
c_i=
\begin{cases}
1 & (\phi_i<d) \\
0 & (\phi_i\ge d)
\end{cases}
$$

| 記号 | 意味 |
|---|---|
| $c_i$ | 脚 $i$ の接地状態。1=接地、0=遊脚 |
| $\phi_i$ | 更新後の脚位相 |
| $d$ | `duty_factor` |

`duty_factor = 0.74` の場合は、次の区間になる。

```text
位相 [0.00, 0.74) : 接地
位相 [0.74, 1.00) : 遊脚
```

### `_init` 分岐について

`run()` には `_init[leg] == True` の特別処理もある。

**確認できた事実:** `reset()` は全脚を `False` で初期化する。添付文書で調査した標準nominal経路では、稼働中の `self.pgg` に対して `_init=True` を設定する呼び出しは確認されていない。

したがって、本章では `_init == False` の通常分岐を対象にしている。

---

## 5. `contact_sequence` の生成

`compute_contact_sequence()` は、現在の脚位相を起点として、MPCが使用する未来の接地予定を生成する。

通常歩容でのshapeは次のとおりである。

```text
contact_sequence.shape = (4, horizon)
```

```text
行: FL, FR, RL, RR
列0: 現在
列1以降: 未来
値: 1=接地、0=遊脚
```

処理は次の順番である。

```python
# 1. 現在の内部状態を保存
t_init = np.array(self._phase_signal)
init_init = np.array(self._init)

# 2. 現在の接地状態
contact_sequence[:, 0] = self.run(0.0, self.step_freq)

# 3. 位相を未来へ進めながら接地状態を予測
for i in range(1, self.horizon):
    contact_sequence[:, i] = self.run(dt, self.step_freq)

# 4. 保存しておいた現在位相へ戻す
self.set_phase_signal(t_init, init_init)
```

一様離散化の場合、列 $j$ が表す時刻は次になる。

$$
t_j=t_{now}+j\Delta t_{mpc}
$$

`horizon = 12`、`mpc_dt = 0.02 s` なら、列0は現在、列1は0.02秒後、列11は0.22秒後を表す。

### なぜ最後に位相を復元するのか

未来予測では、`run()` を使って `_phase_signal` を一時的に未来へ進めている。そのままにすると、接地予定を計算するたびに実際の歩容位相まで未来へ進んでしまう。

そこで、計算前に保存した `t_init` と `init_init` を最後に復元している。

**コード上の結果:** 通常終了した場合、`compute_contact_sequence()` の前後で現在位相は変化しない。変化するのは、返される未来接地列だけである。

---

## 6. `current_contact`との関係

`WBInterface.update_state_and_reference()` では、まず現在位相を1シミュレーションステップ進め、続いて未来接地列を生成する。

```python
self.pgg.run(simulation_dt, self.pgg.step_freq)

contact_sequence = self.pgg.compute_contact_sequence(
    self.contact_sequence_dts,
    self.contact_sequence_lenghts,
)

self.current_contact = np.array([
    contact_sequence[0][0],
    contact_sequence[1][0],
    contact_sequence[2][0],
    contact_sequence[3][0],
])
```

最初の `run()` の戻り値は、ここでは直接 `current_contact` に代入されていない。

ただし、`compute_contact_sequence()` の列0は `run(0.0, ...)` で同じ現在位相を評価するため、通常分岐では両者の接地判定は一致する。

---

## 7. 3種類の時間パラメータ

| パラメータ | 添付文書の設定値 | 使用目的 |
|---|---:|---|
| `simulation_dt` | 0.002 s | 現在の歩容位相を進める刻み |
| `mpc_params['dt']` | 0.02 s | 未来の接地予定を並べる刻み |
| `mpc_frequency` | 100 Hz | OCPを再計算する頻度 |

デフォルトでは、次のようになる。

```text
歩容位相更新・contact_sequence生成 : 500 Hz
OCP求解                              : 100 Hz
contact_sequenceの未来時間間隔       : 0.02 s
```

これらは役割が異なり、同じ値である必要はない。

---

## 8. デフォルトtrotの具体例

```text
step_freq       = 1.35 Hz
duty_factor     = 0.74
初期位相        = [0.5, 0.0, 0.0, 0.5]
脚順序          = [FL, FR, RL, RR]
simulation_dt   = 0.002 s
```

### 8.1 周期、接地時間、遊脚時間

$$
T_{gait}=\frac{1}{1.35}\approx0.7407\ \mathrm{s}
$$

$$
T_{stance}=0.74\times0.7407\approx0.5481\ \mathrm{s}
$$

$$
T_{swing}=0.26\times0.7407\approx0.1926\ \mathrm{s}
$$

### 8.2 最初の位相更新

| 脚 | FL | FR | RL | RR |
|---|---:|---:|---:|---:|
| 更新前 | 0.5000 | 1.0000 | 1.0000 | 0.5000 |
| 更新・剰余後 | 0.5027 | 0.0027 | 0.0027 | 0.5027 |
| 接地状態 | 1 | 1 | 1 | 1 |

4脚とも位相が0.74未満なので、最初は全脚接地になる。

### 8.3 1周期中の接地パターン

| 経過時間 | FL | FR | RL | RR | 状態 |
|---|---:|---:|---:|---:|---|
| 0〜0.178 s | 1 | 1 | 1 | 1 | 全脚接地 |
| 0.178〜0.370 s | 0 | 1 | 1 | 0 | FL・RRが遊脚 |
| 0.370〜0.548 s | 1 | 1 | 1 | 1 | 全脚接地 |
| 0.548〜0.741 s | 1 | 0 | 0 | 1 | FR・RLが遊脚 |

境界時刻は連続時間の計算値である。実際の切り替えは `simulation_dt = 0.002 s` ごとの離散時刻に量子化される。

### 事実と解釈

**事実:** `duty_factor = 0.74 > 0.5` なので、2組の対角脚が同時に接地する区間が存在し、`[1,1,1,1]` になる。

**解釈:** これは高いduty factorを持つtrotの周期内に自然に生じる全脚接地区間である。「起動直後だけの異常な過渡」とは判断できない。

なぜ0.74という値を選んだかは、コードだけからは確定できない。接地時間を長くして支持余裕を増やす意図は考えられるが、これは推測であり、設計者の説明や実験結果による確認が必要である。

---

## 9. NMPCまでのデータフロー

```text
config.py: gait_params[gait_name]
  ├─ step_freq
  ├─ duty_factor
  └─ gait_type
       ↓
WBInterface.__init__()
       ↓
PeriodicGaitGenerator.reset()
  gait_type → phase_offset → _phase_signal
       ↓ 毎シミュレーションステップ
PeriodicGaitGenerator.run(simulation_dt, step_freq)
  現在位相を更新
       ↓
PeriodicGaitGenerator.compute_contact_sequence(...)
  列0     : 現在の接地状態
  列1以降 : MPC時間刻みで先読みした接地状態
       ↓
WBInterface.update_state_and_reference()
  current_contact = contact_sequence[:, 0]
       ↓
QuadrupedPyMPC_Wrapper.compute_actions()
       ↓ mpc_frequencyで間引き
SRBDControllerInterface.compute_control(..., contact_sequence, ...)
       ↓
NMPC
```

---

## 10. コード上の注意点と未確認事項

### 確認できた注意点

1. `phase_offset` の `1.0` は、`reset()`直後にはそのまま保持され、最初の`run()`で0付近に正規化される。
2. `compute_contact_sequence()` は内部状態を一時的に未来へ進め、最後に復元する。
3. `run(simulation_dt, ...)` の戻り値は、この箇所では直接使用されず、`current_contact` は `contact_sequence[:, 0]`から作られる。
4. 通常歩容の `contact_sequence` は `(4, horizon)` だが、`FULL_STANCE` 分岐ではコード上 `(4, horizon * 2)` を返す。後段が両方のshapeをどのように扱うかは、本章では未確認である。
5. ソースコードでは `contact_sequence_lenghts` と綴られている。本文でもコード変数を指す場合は、この綴りを使用する。

### 未確認事項

- 目標速度に応じて `start_and_stop_activated` を有効化する実機経路
- `optimize_step_freq=True` のときの歩容周波数更新
- 非一様離散化時の各列の具体的な時刻
- `FULL_STANCE` のshapeが後段で問題にならない理由
- デフォルト歩容パラメータの設計根拠

これらはコード上の標準的な位相生成を理解した後、必要に応じて別章で調査する。
