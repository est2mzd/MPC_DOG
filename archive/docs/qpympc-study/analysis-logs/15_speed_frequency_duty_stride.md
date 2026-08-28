# Log 15: 速度・Gait周波数・Duty factor・歩幅

対応プロンプト: フェーズ12。目標速度、`step_freq`、`duty_factor`、Foothold、Stance/Swing時間の関係。
記録日: 2026-08-23。学習資料本文と制御コードは未修正。

標準: `gait='trot'`, `gait_params['trot']={'step_freq': 1.35, 'duty_factor': 0.74}`, `optimize_step_freq=False`, `step_freq_available=[1.4, 2.0, 2.4]`, `type='nominal'`, `hip_height=0.28`。

判定の前提: 周波数が速度を決めるのではない。ユーザー指令は速度。周波数は周期とFoothold先送り量を変える。実現可能範囲はある。

---

## 1. Timing（コード照合）

コードの定義は次の3式と一致する。

\[
T=\frac{1}{f},\qquad
T_{stance}=\frac{d}{f},\qquad
T_{swing}=\frac{1-d}{f}
\]

| 項目 | 数式 | コード変数 | 生成関数 | 単位 | 使用先 |
|---|---|---|---|---|---|
| Cycle時間 | \(T=1/f\) | 明示変数なし。`(1/step_freq)` として展開 | `WBInterface.__init__` 66, 73 | s | Stance/Swing時間の共通分母 |
| Step frequency | \(f\) | `pgg.step_freq`。初期は `gait_params[gait]['step_freq']` | `WBInterface.__init__` 47–56。更新は `compute_stance_and_swing_torque` 357–358 | Hz | `PeriodicGaitGenerator.run` の位相進み \(\phi\leftarrow(\phi+\Delta t f)\bmod 1\) |
| Duty factor | \(d\) | `pgg.duty_factor`。初期は `gait_params[gait]['duty_factor']` | 同上 `__init__`。周波数更新時も**書き換えない** | — | `run()` の接地判定 \(\phi<d\)。Stance/Swing時間 |
| Stance時間 | \(T_{st}=d/f\) | `frg.stance_time` | `__init__`: `(1/pgg.step_freq)*pgg.duty_factor`。更新: `(1/f)*d` | s | FRG の `delta_ref_H = (stance_time/2)*v_H^{ref}` |
| Swing時間 | \(T_{sw}=(1-d)/f\) | `stc.swing_period` | `__init__`: `(1-d)*(1/f)`。更新: 同式のあと `regenerate_swing_trajectory_generator` | s | CubicSpline軌道、`swing_time` 上限、apex判定 |
| 候補周波数 | \(\mathcal F\) | `mpc_params['step_freq_available']` | `config.py` 既定 `[1.4, 2.0, 2.4]` | Hz | Batched / Sampling の候補。**標準Trot初期1.35は含まれない** |
| 周波数最適化フラグ | — | `mpc_params['optimize_step_freq']` | `config.py` 既定 `False` | bool | wrapperが batched を作るか。`optimize_swing` の計算可否 |
| 周波数適用ゲート | — | `optimize_swing` | `optimize_step_freq` が True なら `STC.check_touch_down_condition`、否則常に 0 | 0/1 | 候補評価と `pgg`/`frg`/`stc` 更新の両方を門番 |

標準Trot数値（最適化オフ、固定）:

| 変数 | 値 |
|---|---|
| \(f\) | 1.35 Hz |
| \(d\) | 0.74 |
| \(T\) | \(1/1.35\approx 0.7407\) s |
| \(T_{st}\) | \(0.74/1.35\approx 0.5481\) s |
| \(T_{sw}\) | \(0.26/1.35\approx 0.1926\) s |

他Gaitの `gait_params`（参照用。標準はTrotのみ使用）:

| gait | `step_freq` | `duty_factor` |
|---|---:|---:|
| trot | 1.35 | 0.74 |
| crawl | 0.5 | 0.8 |
| pace | 1.4 | 0.7 |
| bound | 1.8 | 0.65 |
| full_stance | 2 | 0.65 |

---

## 2. 速度と接地点間隔

### \(L_{footprint}=v/f\)

定常水平速度 \(v\)、周期 \(T=1/f\) のとき、**同じ脚が連続して地面に残す接地点の間隔**である。

- 胴体相対のTouchdown位置 → **違う**。それは後述の Raibert 項 \(\frac{1}{2}v T_{st}\)。
- 同じ脚の連続する地面上の接地点間隔 → **これ**。
- 1回のSwing中の足先移動距離 → 世界座標ならほぼこれ（離地点から次着地点）。胴体相対なら \(L_{stance}\)。
- Stance中の胴体相対移動量 → **違う**。それは \(L_{stance}=v T_{st}\)。

根拠: 足が接地中に滑らないなら、1周期の間に胴体は \(vT\) 進み、同じ脚の次の印は \(v/f\) 先になる。FRGはこれを直接置かない。

### \(L_{stance}=v T_{stance}\)

接地中に、固定足に対して胴体が水平に進む距離。Duty を入れると

\[
L_{stance}=v\frac{d}{f}=d\,L_{footprint}.
\]

コード対応: FRG は \(L_{stance}\) そのものではなく、その半分を Heading 水平速度で足す。

\[
\Delta p_{ref}^{H}
=
\frac{T_{st}}{2}\,v_{H}^{ref}
=
\frac{d}{2f}\,v_{H}^{ref}
\]

`foothold_reference_generator.py` 103–105。clip は \(\pm\) `hip_height*1.5` \(=\pm 0.42\) m。

したがって「脚を \(v/f\) 前へ伸ばす」はコードにも幾何にも無い。胴体相対の公称着地は hip + \(\frac{1}{2}L_{stance}\) + 誤差補正 + `hip_offset`。

### \(\bar v_{foot}=\|p_{td}-p_{lo}\|/T_{swing}\)

Swing中の足先平均速さ。\(p_{td},p_{lo}\) の取り方で意味が変わる。

| 距離の取り方 | 距離 | 平均速さ（\(d\)一定） | 周波数依存 |
|---|---|---|---|
| 同一脚の世界座標 離地→次着地 | \(\approx L_{footprint}=v/f\) | \(v/(1-d)\) | **なし** |
| 胴体相対の後方→前方（公称 Raibert） | \(\approx L_{stance}=vd/f\) | \(v\,d/(1-d)\) | **なし** |
| 鉛直ステップ高さ（固定 `step_height`） | \(0.2\times hip\_height=0.056\) m | 高さ方向は \(f\) に比例して増える | **あり** |

\(d=0.74\) なら水平平均は \(\bar v_{world}\approx 3.85\,v\)、\(\bar v_{body}\approx 2.85\,v\)。周波数を上げても水平平均は同じで、周期が短くなる分、移動距離も短くなる。

「周波数を上げると足先速度が上がり得る」が成立するのは、主に (1) 鉛直ステップを同じ高さで速く行う、(2) clip後に距離が縮まらない、(3) ピーク加速度 / 着地回数、である。水平印間隔だけを見る限り、\(d\) 一定なら平均速さは \(f\) に依らない。

---

## 3. 数値例

Duty はコードの標準Trot **0.74**。下表は幾何・時間の計算のみ。Go2で実現可能とは断定しない。

| 速度 [m/s] | Frequency [Hz] | Cycle時間 [s] | Stance時間 [s] | Swing時間 [s] | 同一脚接地点間隔 [m] | Stance相対移動量 [m] |
|---:|---:|---:|---:|---:|---:|---:|
| 0.5 | 1.4 | 0.7143 | 0.5286 | 0.1857 | 0.3571 | 0.2643 |
| 0.5 | 2.0 | 0.5000 | 0.3700 | 0.1300 | 0.2500 | 0.1850 |
| 0.5 | 2.4 | 0.4167 | 0.3083 | 0.1083 | 0.2083 | 0.1542 |
| 1.0 | 1.4 | 0.7143 | 0.5286 | 0.1857 | 0.7143 | 0.5286 |
| 1.0 | 2.0 | 0.5000 | 0.3700 | 0.1300 | 0.5000 | 0.3700 |
| 1.0 | 2.4 | 0.4167 | 0.3083 | 0.1083 | 0.4167 | 0.3083 |
| 2.0 | 1.4 | 0.7143 | 0.5286 | 0.1857 | 1.4286 | 1.0571 |
| 2.0 | 2.0 | 0.5000 | 0.3700 | 0.1300 | 1.0000 | 0.7400 |
| 2.0 | 2.4 | 0.4167 | 0.3083 | 0.1083 | 0.8333 | 0.6167 |
| 5.0 | 1.4 | 0.7143 | 0.5286 | 0.1857 | 3.5714 | 2.6429 |
| 5.0 | 2.0 | 0.5000 | 0.3700 | 0.1300 | 2.5000 | 1.8500 |
| 5.0 | 2.4 | 0.4167 | 0.3083 | 0.1083 | 2.0833 | 1.5417 |

参考（表に無い標準Trot 1.35 Hz）: \(T\approx0.741\) s、\(T_{st}\approx0.548\) s、\(T_{sw}\approx0.193\) s。\(v=1\) で \(L_{footprint}\approx0.741\) m、\(L_{stance}\approx0.548\) m。

FRG clip \(\pm0.42\) m が効き始める公称速度（\(\frac{d}{2f}v=0.42\)）:

| \(f\) [Hz] | \(v_{clip}\) [m/s] |
|---:|---:|
| 1.35 | 1.53 |
| 1.4 | 1.59 |
| 2.0 | 2.27 |
| 2.4 | 2.72 |

表の 2.0 m/s @ 1.4 Hz と 5.0 m/s の全周波数は、公称 Raibert 半歩が 0.42 m を超える。コードは先送りを切るだけで、指令速度は下げない。

同じ周波数で幾何的に「小さい」速度: \(L_{stance}/2\) が脚長スケールに収まる側。1.4 Hz ならおおよそ 1.6 m/s 未満で clip 前。これは可到達保証ではない。

---

## 4. 定常速度

区別:

| 量 | 定常水平速度（損失無視・平地） | コード上の扱い |
|---|---|---|
| 平均水平加速度 | \(\dot v\approx 0\) | 指令は速度。位置xy参照は0 |
| 平均水平GRF | \(\sum_i F_{x,i}\approx 0\) | OCPは速度誤差を罰する。平均推進は自動で小さくなる側 |
| Stance中の瞬間GRF | 鉛直はおおよそ \(mg/n_s\)。水平は Pitch・速度誤差で変動し得る | 摩擦錐 \(\mu=0.42\)、\(F_z\in[0,mg]\) は常時（接触Gateなし） |
| 足が地面に固定される相対運動 | 胴体が足の上を \(L_{stance}\) 通過する。力学コストではなく運動学拘束 | FRG/STC。OCPの足位置重み |
| Swingで足を前方へ戻す | 世界距離 \(\approx v/f\) を \(T_{sw}\) で運ぶ | STC CubicSpline。MPC足速度は実行されない |
| 着地衝撃 | 周期 \(f\) で発生。平均推進とは別 | 実接触はMuJoCo。指令側に衝撃モデルなし |
| 空気抵抗・摩擦・内部損失 | 小さな平均 \(F_x\) が必要 | モデルに陽に無い。速度重みと実滑りで間接 |

「定常速度なら歩幅を小さくできる」が成立する条件:

1. **Cadence を上げて** \(L_{footprint}=v/f\) と \(L_{stance}=vd/f\) を脚可動域内に収める。
2. 周波数を固定したまま \(L\) だけを小さくすることは、滑らない足ではできない（\(v=fL\)）。
3. 平均水平GRFが小さいことと、歩幅が小さくてよいことは同値ではない。

---

## 5. 加減速

| 制約 | 定常高速 | 急加速 |
|---|---|---|
| 必要水平GRF | 平均は損失分。瞬間は姿勢維持 | \(F_x\sim m a\)。対角2脚なら脚あたり \(ma/2\) |
| 摩擦錐 | \(\|F_{xy}\|\le\mu F_z\)。平均は余裕 | \(a\) が大きいと \(\mu F_z\) で頭打ち。\(\mu=0.42\)、\(F_z\le mg\) |
| Pitch moment | 高速でも \(F_x h\) と足配置の釣り合い | 加速で前脚/後脚の \(F_z\) 配分が偏る。OCPは角度重みで抑えるが、配置はFRG |
| Torque saturation | \(\tau=-J^\top F\)、clip \(0.9\times\)ctrlrange | 同じ写像。大きな \(F_x\) で先に飽和しやすい |
| Foot placement | 半Stance先送り。高速で clip | 加速中は現在 \(v\) と \(v^{ref}\) がずれ、誤差項 \(\sqrt{h/g}(\bar v-v^{ref})\)（±0.05 m）しか補償しない |
| Joint velocity | Swing距離は \(v/f\)、時間は \((1-d)/f\) | 加速中に目標TDが遠ざかると同じ \(T_{sw}\) で届かない |
| Swing時間 | \(d,f\) 固定なら速度に依らない | 周波数最適化がオフなら加速しても \(T_{sw}\) は伸びない |

定常高速の主制約は可動域・Swing鉛直速度・着地回数。急加速の主制約は水平力、摩擦、Pitch、トルク、配置の遅れ。両者を同じ「速さ」として評価してはいけない。

コードに加速度指令は無い。ユーザーは速度を変え、過渡が加速になる。

---

## 6. Frequency候補評価（End-to-End）

### 標準設定（ディスク `config.py`）

`optimize_step_freq=False`。

```text
ユーザー目標速度
→ VelocityModulator（姿勢危険時のみ縮小。周波数は見ない）
→ PGG は固定 f=1.35, d=0.74
→ FRG は固定 stance_time≈0.548 s
→ 単一 contact_sequence で nominal MPC
→ best_sample_freq は常に pgg.step_freq
→ optimize_swing は常に 0
→ pgg / frg / stc は更新されない
```

`SRBDBatchedControllerInterface` は作られない（wrapper 34–35: `type!='sampling' and optimize_step_freq`）。Sampling の gait adaptive も `type=='sampling'` かつ `optimize_step_freq` のときだけ import される。

### 勾配MPCでフラグを立てた場合

実装は `type!='sampling'` かつ `optimize_step_freq=True`。

```text
ユーザー目標速度（この評価中は固定。周波数は変えない）
→ 現行 f で PGG.run + compute_contact_sequence
→ 現行 stance_time で FRG（候補ごとには再計算しない）
→ 現行接触列で主MPC（100 Hz）
→ 500 Hz ごとに optimize_gait:
     optimize_swing==1 のときだけ
     各 f ∈ {1.4, 2.0, 2.4} について
       仮 PeriodicGaitGenerator(duty=現行d, step_freq=候補, gait_type=現行)
       set_phase_signal(現行位相)
       compute_contact_sequence → 候補ごとの接触列
     Acados_NMPC_GaitAdaptive.compute_batch_control
       同じ state / 同じ ref_state（同じ v^{ref}、同じ foothold）
       接触列だけ候補で変える
     cost = get_cost()
     n≠0 なら + 3*(f_n - 1.4)^2
     argmin → best_freq
→ compute_stance_and_swing_torque:
     optimize_swing==1 なら
       pgg.step_freq = best_freq
       frg.stance_time = (1/f)*d
       stc.swing_period = (1-d)/f  を再生成
→ 次の 500 Hz 周期で新しい f が位相と次の FRG に乗る
```

`optimize_swing==1` の条件（`check_touch_down_condition`, lookahead=3）:

1. 直前まで4脚接地でなく、今4脚接地（rising edge）。
2. `contact_sequence[:,0:3]` が全て1。
3. `contact_sequence[:,3]` に遊脚がある。

つまり4脚overlapの終盤、次離地の約 `3*mpc_dt=0.06` s 前。Duty 0.74 の overlap \((d-0.5)/f\) は 1.35 Hz で約 0.18 s、2.4 Hz で 0.10 s。標準Trotでは発火余地がある。

### 質問への答

| 質問 | 答 | 根拠 |
|---|---|---|
| 目標速度は候補評価で固定されるか | **固定**。`ref_state` を全候補で共有 | `optimize_gait` → `compute_batch_control(state, ref_state, contact_sequence_temp)` |
| Frequencyが目標速度を変更するか | **しない** | 速度指令経路に `step_freq` は無い |
| MPC costだけでFrequencyを選ぶか | **しない** | 勾配: `3*(f-1.4)^2`（先頭候補以外）。Sampling: `100*(f-1.3)^2` |
| 別のPenaltyがあるか | **ある**（上） | `centroidal_nmpc_gait_adaptive.py` 1232–1235。`centroidal_nmpc_jax_gait_adaptive.py` 500 |
| 標準設定で有効か | **無効** | `optimize_step_freq=False` |
| GradientとSamplingのどちらに実装されるか | **両方別実装**。標準 `nominal` は勾配batched。`type=='sampling'` は rollout内サンプリング。同時には動かない | wrapper 34–35 vs `SRBDControllerInterface` 77–83 |
| 候補ごとにDutyは変わるか | **変わらない** | 仮PGGは `duty_factor=pgg_duty_factor` をコピー |
| 選択周波数はFoothold Generatorへ反映されるか | **適用後の次周期から**。評価時点のFRGは現行 `stance_time` のまま | WBC 357–361。batchedは `ref_foot_*` を作り直さない |

資料が書いた「候補ごとのStance時間→候補ごとのFoothold」は、**選択後の反映**としては正しいが、**評価ループ内では未実装**。評価が変えるのは接触列だけである。

勾配側の追加penalty量（MPC costに加算）: \(f=2.0\) で 1.08、\(f=2.4\) で 3.0。先頭 1.4 Hz は加算ゼロ。低周波数へ寄せる。

Sampling側の追加: \(100(f-1.3)^2\)。1.4→1、2.0→49、2.4→121。内部 `PeriodicGaitGeneratorJax` は `duty_factor=0.65`, `step_freq=1.65` で初期化され、`gait_params` の 0.74 / 1.35 ではない。MPPIの周波数抽選は `optimize_swing` 乗算がコメントアウトされており、rolloutでは常に候補から選ぶ。PGG/FRGへの適用はやはり `optimize_swing==1`。

---

## 7. Foothold と Gait timing の整合

標準（固定 1.35 Hz）:

- 接触列の \(T_{st},T_{sw}\) と FRG の `stance_time`、STC の `swing_period` は同じ \(f,d\) から初期化される。
- FRGは残りSwing時間を見ない。STCは `swing_period` で軌道を切る。
- 速度が上がっても \(f\) は変わらないので \(L_{stance}\) が伸び、0.42 m で clip。

最適化オン:

- 接触列は候補 \(f\) で変わる。
- その瞬間の foothold 参照は旧 `stance_time`。
- 適用後に `stance_time` と `swing_period` が新 \(f\) に揃う。
- 主MPC（その周期）は旧接触列のまま。新周波数の接触列は次周期以降。

不整合になり得る点（事実）:

1. 候補評価時の foothold と候補接触列の \(T_{st}\) が一致しない。
2. 主MPCとbatchedが同じ周期で別接触列を見ることがある。
3. 標準Trot 1.35 Hz は候補集合に無く、初回選択で 1.4/2.0/2.4 のいずれかに跳ぶ。

---

## 8. 資料照合

### `12_Speed_Frequency_Duty_and_Stride.md`

| 箇所 | 記載 | 判定 | 理由 |
|---|---|---|---|
| §1 結論（連成、定常でも相対移動） | 速度・周波数・Duty・印間隔が連成 | 正しい | \(v=fL\) |
| §2 \(L_{footprint}=v/f\) は地面印間隔 | 胴体相対TDではない | 正しい | §2 本ログ |
| §3 5 m/s・2 Hz で \(L=2.5\) m | 印間隔の計算は正しい | 不完全 | Duty 0.65 は bound/full_stance。標準Trotは 0.74。そのとき \(L_{st}=1.85\) m（0.65なら 1.625 m） |
| §4 \(T_{st},T_{sw}\) | 式はコードと一致 | 正しい | FRG/STC |
| §4 周波数↑で足先速度↑し得る | 水平平均は \(f\) 非依存 | 不完全 | 鉛直・ピーク・clip時はあり得る。水平印間隔だけでは不成立 |
| §5 定常 vs 加速の表 | 平均GRFと運動学を分離 | 正しい | §4–5 本ログ |
| §6 \(f^*=\arg\min J_{MPC}\) | 速度固定でCadence選択 | 不完全 | 実装は \(J_{MPC}+\)周波数penalty。評価時Footholdは候補ごとでない。標準は無効 |
| §7 同一周波数の速度域 | 可動域等で \(\mathcal V(f,\ldots)\) | 正しい（理論） | コードにEnvelope計算は無い |
| §8 対応コード | batched と Sampling を列挙 | 不完全 | 標準パスではどちらも動かないことを書いていない |

### `14_MPC_and_Controller_Tuning.md`

| 箇所 | 記載 | 判定 | 理由 |
|---|---|---|---|
| §2 Frequency/Duty を優先A | 調整項目として妥当 | 正しい | 標準は手動の `gait_params` |
| §2 「速度別周波数候補」を優先C | 上位の将来項目 | 正しい | `optimize_step_freq` は既定オフ |
| §4 推奨順（低速Trotでfreq/Duty） | 運用指針 | 正しい（指針） | 実装検証対象ではない |
| 周波数最適化の中身 | 未記載 | 不完全 | penalty、Foothold非再計算、1.35∉候補 は `14` からは分からない |
| GRF rate weight を調整項目に | `input_rates` 向け | 不完全 | 標準 `nominal` に独立のGRF rate重みは無い（既存ログ 10） |

### `appendices/E_Corrections_and_Clarifications.md`

| 箇所 | 記載 | 判定 | 理由 |
|---|---|---|---|
| §5 周波数が速度を決める、の訂正 | \(v=fL\) 連成。速度が上位 | 正しい | 指令経路と一致 |
| §5 「周波数に応じてFootholdを変える」 | 適用後は `stance_time` 更新 | 不完全 | 標準は \(f\) 固定。候補評価中はFootholdを変えない |
| §6 2.5 m は印間隔 | 胴体相対伸展ではない | 正しい | 本ログ §2 |
| §7 定常高速なら小歩幅 | 低周波のままでは不可。小歩幅には高Cadence | 正しい | 本ログ §4 |
| §15 クローンを `3adfad9` と書く | wrapper HEAD | 誤り | `external/Quadruped-PyMPC` はgitではない。本ログの主題外。既存指摘 |

---

## 9. 事実 / 解釈 / 未確認

**事実**

- 標準は \(f=1.35\), \(d=0.74\) 固定。`optimize_step_freq=False`。
- Timing 3式は FRG/STC/PGG と一致する。
- FRG 先送りは \(\frac{1}{2}v T_{st}\)、clip ±0.42 m。
- 候補評価（有効時）は接触列だけ変え、速度もDutyもFootholdもその場では変えない。
- 選択後に `pgg.step_freq`, `frg.stance_time`, `stc.swing_period` を更新する。
- 勾配batchedに \(3(f-1.4)^2\)、Samplingに \(100(f-1.3)^2\)。

**解釈**

- 周波数は速度指令ではない。同じ \(f\) で出せる \(v\) には運動学上の幅がある。
- 「小歩幅で高速」は高Cadenceが条件。

**未確認**

- Go2実機/本simで各 \((v,f)\) が歩けるか（本表は幾何のみ）。
- batched を実際にONにしたときの数値的な選択頻度（実行していない）。
- Sampling Jax PGG の duty 0.65 が、外側 PGG 0.74 と同時に動いたときの接触ずれの実害。
