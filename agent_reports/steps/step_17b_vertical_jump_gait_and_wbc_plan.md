# Step 17b：その場・垂直ジャンプを「こけずに」着地させる — gait と WBC(NMPC/逆動力学)の調整計画

対象読者：この課題に初めて触れる大学院生。
前提：Step 17（[実装記録](./step_17_forward_jump_rear_leg_push.md)）で、平地・穴なしの
その場ジャンプは一度成功している（`step17_hop_sym2`：胴体 +0.226 m、四脚離地 264 ms、
着地後 2 s の |roll|,|pitch| < 0.003 rad、転倒なし）。ただしそれは
「NMPC の姿勢追従重みを上げる」「四脚対称で踏み切る」などの調整を**その場しのぎ**で
足した結果である。本書は、**なぜ歩行用の仕組み（gait）が垂直ジャンプに合わないか**を
分解し、**gait 関連と WBC（NMPC＋逆動力学）の目標をどう調整すれば堅牢になるか**の
計画を立てる。前進距離は本書の関心ではない（その場でよい）。

---

## 1. 背景 — 歩行とジャンプは「接地の入れ替え方」が根本的に違う

四足歩行は、脚を 1 本ずつ（または対で）順番に「振り出して着く」ことを周期的に
繰り返す。どの瞬間も**最低 2〜3 本は地面に着いている**。この周期パターンを **gait
（歩容）** と呼び、`period`（1 周期の長さ）、`duty`（1 本が接地している時間の割合）、
`phase_offset`（脚ごとのタイミングずれ）の 3 つで決まる。

垂直ジャンプは、これと違って**離散イベント**である。

```
全脚接地でしゃがむ  →  全脚で伸び上がる  →  四脚が同時に地面を離れる（飛翔）
                    →  四脚が同時に着く  →  衝撃を吸収して直立に戻る
```

「四脚が同時に空中」という状態は、周期 gait のどの位相にも存在しない。だから
「gait のパラメータを変える」だけではジャンプにならない。ジャンプは gait の**外**に
置くべきイベントで、その前後の一瞬だけ gait とつなぐ、という設計になる。

---

## 2. 目的

1. いまの垂直ジャンプがコード上どう流れているかを関数単位で書く。
2. 「gait」がジャンプ中・ジャンプ前後でどう振る舞い、どこが問題かを示す。
3. `hop_v0`（失敗）と `hop_sym2`（成功）の実測差から、**効いた調整・効かなかった調整**を
   切り分ける。
4. 残る不安定要因を仮説として並べる。
5. gait 関連の調整案と WBC（NMPC＋逆動力学）の目標調整案を、段階つきで計画する。

事実・実測・推測を分けて書く。

---

## 0. 現時点の到達点（2026-09-03 追記）

- **その場・垂直ジャンプは Stage A（G1 gait=実質STAND、G3 stand_pos_error 0.15、
  W1 NMPC roll/pitch 重み 20）で成立**：実行された 12 回すべてが直立着地・転倒 0・
  NMPC 失敗 0。ホップ 0.20〜0.25 m、四脚離地 238〜290 ms。
  **ユーザー目標「垂直に跳んでこけずに着地」は満たしている。**
- Stage B の naive W2（速度不連続を `dz_0` clamp で潰す）は**逆効果で撤回**（§9.2 W2 実測）。
- 残る詰め（飛翔中ピッチ ~0.3 rad、ホップ高さ ±8%）は Stage C/D と「正しい W2
  （本物のしゃがみ区間）」の追加作業。優先度はユーザー判断。

---

## 3. 結論（先に書く）

- **いまの「強制ジャンプ経路」は、実は gait をほぼバイパスしている。**
  接触スケジュールは gait の周期表ではなく **primitive（`PRELOAD/FLIGHT/SETTLE` …）**で
  上書きされ、ジャンプ後は「最後の primitive を保持 → ロボットが計画終端に近づくと
  `local_planner` が STAND へ切替」で四脚接地に戻る。だから短い強制ジャンプでは
  トロット gait は基本的に噛まない（`hop_sym2` が安定した主因の一つ）。
- **こけた `hop_v0` の原因は gait ではなく WBC 側**：
  (a) 後脚だけの踏切で前脚支持が無く、ピッチが発散（−1.4 rad）、
  (b) NMPC の姿勢（roll/pitch）追従重みが既定 0.5 で弱く姿勢を保てない、
  (c) 点質量プランの鉛直速度が不連続（踏切開始で `vz=−1.6` を要求、実機は `vz=0`）で
      NMPC の初期追従が過大 → 過剰な鉛直 GRF、
  の 3 つが重なった。`hop_sym2` は (a) を四脚対称踏切で消し、(b) を重み 20 で改善して
  成功した。(c) は残ったままで、成功はしたが「余裕は小さい」。
- **堅牢化に必要な調整は主に WBC/NMPC 側**：
  1. 姿勢追従重み（roll/pitch）を恒常的に上げる（正式パラメータ化）。
  2. 踏切の鉛直 GRF を**形状づける**（PRELOAD で上限いっぱいを出させず、なだらかに）。
  3. しゃがみ→伸び上がりの**滑らかな胴体高さ基準**を与える（速度不連続を消す）。
  4. NMPC ホライズンがジャンプ全長（≈ PRELOAD+FLIGHT+着地）を確実に覆うようにする。
  5. 飛翔中の脚に**着地姿勢を保つ Cartesian ゲイン**を入れる（現状 `swing_kp_cart=0`）。
  6. 着地の**足位置目標**を CoM 直下の安定スタンスにする。
- **gait 側で必要なのは限定的**：
  1. ジャンプ後にトロットへ戻さず、安定するまで**四脚接地の hold（STAND 相当）**を保つ。
  2. ハーネスが今トロットへ一時パッチしているのを、**duty≈1（実質 STAND）**へ変える
     ので、ホライズンにわずかに漏れる gait ステップも四脚接地になる。
  3. `stand_pos_error_threshold_` を少し広げ、着地後に STAND へ確実に落ちるようにする。

---

## 4. 用語（最小限）

| 用語 | 意味 |
|---|---|
| gait（歩容） | 脚の接地/離地の周期パターン。`period`・`duty`・`phase_offset` で決まる。`local_footstep_planner::setTemporalParams` が 1 周期ぶんの接触表 `nominal_contact_schedule_` を作る |
| 接触スケジュール `contact_schedule` | ホライズン各ステップ × 4 脚の「接地するか」の bool 表。NMPC への入力の一つ |
| primitive（プリミティブ） | 胴体プランの各点に付く動作ラベル。`CONNECT`(歩行) / `LEAP_STANCE` / `FLIGHT` / `LAND_STANCE` ＋ Step 17 追加の `PRELOAD` `REAR_PUSH` `FRONT_LAND` `SETTLE`。値は `quad_utils/primitive_ids.hpp` に一元化 |
| NMPC | `nmpc_controller`。胴体の目標軌道と**各脚の GRF**（地面反力）を最適化で出す |
| 逆動力学コントローラ | `robot_driver` の `InverseDynamicsController`。NMPC の GRF＋参照足先加速度＋接触状態から**関節トルク**を計算。接地脚は関節 PD で姿勢保持、遊脚は関節 PD＋（本来は）Cartesian PD |
| WBC | 本書では「NMPC＋逆動力学」を合わせてこう呼ぶ（Quad-SDK に独立した WBC ノードは無い） |
| GRF | 地面反力[N]。NMPC が脚ごとに出す。simple model の鉛直上限は **150 N/脚**（`go2.yaml`） |
| 強制ジャンプ経路 | Step 17b で追加。`jump_mode:=force_leap` のとき `global_body_planner` が RRT を回さず、静止後に 1 個のジャンプ action を組み立てて `body_plan` に流し続ける（`buildForcedJumpPlan` / `forcedJumpSpinOnce`） |

---

## 5. いまの垂直ジャンプはどう動いているか（関数単位）

```
global_body_planner ノード  spin()
  jump_mode==FORCE_LEAP なら callPlanner(RRT) の代わりに forcedJumpSpinOnce()
    ├ ロボットが静止（|v|<0.15）かつ reset 後 2s 経過を待つ
    ├ buildForcedJumpPlan():
    │    s0 = 現在姿勢, 高さを h_nom に, vel=0
    │    getRandomLeapAction(s0, (0,0,1), a, cfg) を最大500回試し 1個の
    │      鉛直ジャンプ action を得る（is_jump=true）
    │      - dz_impulse ∈ [dz0_min,dz0_max] を一様乱数
    │      - t_s_leap  ∈ [t_s_min,t_s_max] を一様乱数
    │      - refineAction → refineStance が GRF を高さ境界条件から解く
    │    a.grf_0[0] に前向き成分（今回 0）
    │    s_land = applyAction(s0, a, cfg)
    │    current_plan_.loadPlanData(VALID, ..., {s0,s_land}, {a}, ...)
    │       └ interpStateActionPair が a.is_jump を見て
    │            leap stance を PRELOAD→REAR_PUSH（今回は preload_fraction=1.0 で全て PRELOAD）
    │            land stance を FRONT_LAND→SETTLE（今回は front_land_fraction=0.0 で全て SETTLE）
    │            にスタンプ
    │    setPublishedTimestamp(now); forced_jump_built_ = true
    └ 以降、同じ固定プランを毎周期 body_plan / discrete_body_plan へ publish し続ける

local_planner
  ref_primitive_plan_(i) = body_plan_msg_->primitive_ids[i + current_plan_index_]
    （current_plan_index_ は「publish 時刻からの経過 / dt」で進む → プラン終端に達すると
      最後のサンプルを保持。loadPlanData の末尾 push_back で LEAP_STANCE が入る）
  LocalFootstepPlanner::computeContactSchedule(idx, body_plan, ref_primitive_plan, control_mode, out)
    control_mode==STAND → 全ステップ {1,1,1,1}
    それ以外 → nominal gait を位相タイル展開、その後 primitive で上書き:
       PRELOAD/SETTLE/LEAP_STANCE/LAND_STANCE → {1,1,1,1}
       REAR_PUSH  → {0,1,0,1}
       FRONT_LAND → {1,0,1,0}
       FLIGHT     → {0,0,0,0}
  getReferenceFromGlobalPlan: ロボットがプラン終端状態に十分近い
    （|current_state - 最終plan状態| <= stand_pos_error_threshold_）→ control_mode_ = STAND

nmpc_controller  quad_nlp
  contact_schedule を contact_sequence_(4×N) にコピー
  接地脚の GRF に u_lb/u_ub を適用（simple model 鉛直 [10,150] N/脚）
  接地脚へ名目 GRF mass*g/接地脚数 を等分
  x_weights の roll/pitch 要素（既定 0.5）で姿勢を追従

robot_driver  InverseDynamicsController
  local_plan を時間補間 → 参照姿勢, GRF は NMPC 出力の ZOH, 接触は参照状態の feet[i].contact
  computeInverseDynamics(ref_foot_acc, grf, contact_mode, tau)
  接地脚: stance_kp/kd = [60,60,60]/[4,4,4] の関節 PD
  遊脚:   swing_kp/kd = [60,60,60]/[4,4,4] の関節 PD ＋ Cartesian PD（swing_kp_cart/kd_cart = [0,0,0] ＝ 無効）
  → 関節 pos/vel/torque → MuJoCo
```

**ここから分かること**

- ジャンプ中の接触は gait 周期表ではなく **primitive で上書き**される。
- 強制ジャンプのプランは短い（状態 2 点、補間しても ≈ 0.85 s）。NMPC ホライズンは
  26 × 0.03 = **0.78 s** しかないので、**ジャンプ全体がぎりぎりホライズンに収まる**か、
  やや溢れる。
- ジャンプ後は末尾 `LEAP_STANCE` の保持で四脚接地 → ロボットが終端に近づくと STAND。
  つまり**トロット gait は短い強制ジャンプではほぼ出番がない**。
- 飛翔中、4 脚とも「遊脚」扱いだが Cartesian ゲインが 0 なので、脚は**参照関節角へ
  弱く引っぱられるだけ**。着地姿勢を能動的に作っていない。

---

## 6. 「gait」はジャンプ前後で何をしているか / なぜ噛み合わないか

### 6.1 ジャンプ中：gait は上書きされて無効

`computeContactSchedule` は先に nominal gait をタイル展開するが、`ref_primitive_plan(i)`
がジャンプ系 primitive の行はすべて上書きされる。強制ジャンプのホライズンはほぼ全部
ジャンプ primitive なので、**gait の `period/duty/phase_offset` は事実上使われない**。

### 6.2 ジャンプ直後：トロットへ戻ると危ない

もしプランが早く終わって `ref_primitive_plan` が `CONNECT` に戻り、かつ `control_mode_`
がまだ `STEP` だと、`computeContactSchedule` は**トロット**（`{1,0,0,1}`→`{0,1,1,0}` 交互）を
返す。着地直後のまだグラグラした胴体で片対角 2 脚支持に切り替えると、簡単にこける。
いまはプラン末尾 `LEAP_STANCE` 保持で四脚接地が続くため助かっているが、**設計として
明示的に「着地後は安定するまで四脚 hold」を保証していない**。

### 6.3 ジャンプ前：しゃがみ動作が gait と無関係に始まる

`forcedJumpSpinOnce` はロボットが静止したら即プランを差し込む。gait 位相との同期は
無い（同期する必要も無い）。ただし、差し込んだ瞬間の胴体高さ・脚配置が「ジャンプに
適した初期姿勢」であるとは限らない。いまは STAND 直後なので概ね良いが、保証は無い。

### 6.4 まとめ：gait 側の本質的タスクは「前後のつなぎ」だけ

- ジャンプ中は primitive 上書きで gait は消える（問題なし）。
- **ジャンプ後、安定するまで四脚接地を保証する**（現状は偶然うまくいっている）。
- **ジャンプ前、良い初期姿勢を保証する**（現状は STAND 依存）。

---

## 7. 実測：何が効いて、何が効かなかったか

平地 `flat_wide`、その場（前進なし）、2026-09-03。

| run | 主な設定 | 結果（計測） |
|---|---|---|
| `hop_v0` | dz≈1.6、`preload_fraction=0.4`（＝**後脚のみ REAR_PUSH**あり）、姿勢重み**既定 0.5** | 離地はした（胴体 +0.29 m、足先明確に浮く）。だが飛翔で **pitch −1.4 rad → roll ±3.14 rad（反転）**、NMPC **329 回失敗**、反転着地 |
| `hop_sym2` | dz∈[1.1,1.5]、`preload_fraction=1.0`（＝**四脚対称踏切**、REAR_PUSH 無し）、姿勢重み **20**、t_s∈[0.20,0.28] | **成功**。胴体 +0.226 m、四脚離地 264 ms、NMPC **失敗 0**、着地後 2 s の |roll|,|pitch| < 0.003 rad、転倒なし |
| `fwd_a` | 上記＋前向き GRF 大（vx0=0.8 → grf_0[0]≈0.88） | 前進はするが飛翔/着地で反転（max roll 3.14）、NMPC 318 回失敗 |
| build 失敗 | `dz0_min==dz0_max` に固定 | `getRandomLeapAction` が探索できず「could not build a valid jump action」 |
| 全ノード abort | `x_weights` に整数 `30` を書いた | ROS が mixed int/float 配列を弾き `RCLInvalidROSArgsError` |

**効いた調整**

1. **四脚対称踏切**（`preload_fraction=1.0` で REAR_PUSH を出さない）。
   後脚のみ踏切はピッチを支える前脚が無く、点質量プランに姿勢基準も無いので発散する。
2. **NMPC の roll/pitch 追従重みを 0.5 → 20**。姿勢を保つ最低限の権限を NMPC に与える。
3. **dz と t_s を下げすぎず、かつ範囲を残す**。範囲をつぶすと実行可能解が見つからない。

**効かなかった / 無関係だった調整**

- gait の `period/duty/phase_offset`（トロット化）。短い強制ジャンプでは primitive
  上書きで消えるので、成否にほぼ影響しなかった。
- 前向き GRF は「こけない」目的には**逆効果**（`fwd_a`）。その場ジャンプに集中する
  今回の方針では 0 にする。

---

## 8. 残る不安定要因（仮説・未検証）

垂直ジャンプは **2/2 で再現**した（`hop_sym2`：胴体 +0.226 m / `hop_rep1`：+0.219 m、
どちらも四脚離地 ≈ 260 ms、NMPC 失敗 0、転倒なし）。ただし**余裕は小さい**：
どちらも飛翔中にピッチが一時 **±0.3〜0.33 rad（≈ 19°）** まで振れてから戻っている。
外乱やパラメータのわずかな差で反転側へ倒れる余地がある。堅牢化のために潰すべき点。

1. **鉛直速度の不連続**：`getRandomLeapAction` は `a.dz_0 = getDzFromState(s) − dz_impulse`
   ＝ `−dz_impulse`（例 −1.4 m/s）を踏切開始速度にする。実機は静止（`vz=0`）。NMPC は
   この差を初期に埋めようと過大なトルク/GRF を出し、PRELOAD で胴体が跳ね上がりすぎる。
   → **しゃがみ（`vz` を 0 から負へなだらかに）を明示した滑らかな胴体高さ基準**が要る。
2. **PRELOAD の GRF が上限張り付き**：`hop_sym2` の PRELOAD で NMPC GRF が全脚 150 N/脚
   （＝合計 3.8 体重）に張り付き、鉛直加速度が過大。ホップ高さがばらつく原因。
   → **PRELOAD 区間だけ `u_ub` z を絞る**か、名目 GRF を体重付近から緩やかに増やす。
3. **飛翔中の脚制御が弱い**：`swing_kp_cart = 0` なので、飛翔中に脚が着地姿勢を能動的に
   作らない。着地の瞬間の脚角度が run ごとにばらつき、衝撃と姿勢外乱が読めない。
   → **飛翔中だけ Cartesian swing ゲインを入れ、CoM 直下へ足先を構える**。
4. **着地の足位置目標**：いま着地足は「ballistic＋弱い関節 PD」で落ちた場所。CoM 直下の
   矩形スタンスに揃えたい。
5. **NMPC ホライズン**：0.78 s はジャンプ全長（PRELOAD 0.23 ＋ FLIGHT 0.39 ＋ 着地 0.23 ≈
   0.85 s）よりわずかに短い。着地相の一部がホライズン外になり、着地直前の最適化が
   近視眼的になる。→ **horizon をジャンプ全長＋余裕（≈ 1.0〜1.2 s ぶん）に伸ばす**。
6. **着地後の gait 復帰**：7.2 のとおり、明示的な「四脚 hold」保証が無い。
7. **姿勢重み 20 の副作用**：他の追従（位置・速度）とのバランスが崩れ、別条件で
   NMPC が解けにくくなる可能性。掃引で確認が要る。

---

## 9. 計画：堅牢な「その場・垂直ジャンプ」へ

### 9.1 gait 関連の調整（小）

| # | 変更 | 目的 | 触る場所 |
|---|---|---|---|
| G1 | ハーネスの一時パッチをトロットから **duty≈0.98 / phase_offset 全 0**（実質 STAND gait）へ | ホライズンに漏れる非ジャンプステップも四脚接地にする | `scripts/trial/run_step17_jump.sh`（`go2.yaml` 一時パッチ） |
| G2 | 着地後、ロボットが安定（|v|・|ω| 小、|roll|,|pitch| 小）になるまで **明示的に四脚 hold** を維持する状態を `local_planner` か強制ジャンプ経路に追加。安定後に通常制御へ戻す | 「着地直後トロット」で崩れるのを構造的に防ぐ | `global_body_planner`（プラン末尾に十分長い `SETTLE` を足す）または `local_planner` の STAND 遷移条件 |
| G3 | `stand_pos_error_threshold_` を現状値から少し広げ、着地後に **STAND へ確実に落ちる** | ジャンプで一時的に増えた誤差で STAND 遷移を取りこぼさない | `local_planner.yaml` |

### 9.2 WBC（NMPC＋逆動力学）の目標調整（本丸）

| # | 変更 | 目的 | 触る場所 |
|---|---|---|---|
| W1 | NMPC `x_weights` の roll/pitch 要素を恒常的に引き上げ（例 0.5 → 15〜30）。ジャンプ時だけ切替える口をパラメータで持つ | 姿勢を保つ権限を NMPC に与える（`hop_sym2` で実証済み、正式化） | `go2.yaml` nmpc `x_weights`、切替は param |
| W2 | **滑らかな胴体高さ基準**を強制ジャンプ経路で自前生成：静止 → しゃがみ（z を h_nom−Δ へ、`vz` を 0 から負へ 3 次補間）→ 伸び上がり（z を上へ、`vz` を離陸速度へ）→ 弾道 → 着地 → SETTLE。`getRandomLeapAction` の点質量 action の代わり／補正に使う | 速度不連続を消し、NMPC の初期過大追従を無くす（仮説 8-1） | `global_body_planner::buildForcedJumpPlan` |
| W2 実測（naive 版・失敗） | `jump_crouch_vz` を導入し `a.dz_0 = −0.4` にして `refineAction` で再ソルブしただけの簡易版を試した。**逆効果**：`refineStance` は `dz_0` と GRF の大きさを連動させるので、`dz_0` を緩めると必要 GRF も小さくなり（`grf_0` 3.65 → 2.28、`dz_f` 1.91 → 0.73、`t_f` 0.39 → 0.18）、ホップが 0.22 m → 0.085 m の**貧弱なジャンプ**に。飛翔が浅く足先が地面を擦り、ロールが単調発散して反転（5 回中 実行 2 回とも FAIL）。→ **naive 版は撤回**（`jump_crouch_vz` は既定 0 で無効のまま残置）。正しい W2 は「同じ強い GRF/離陸速度は保ったまま、その前に本物のしゃがみ区間（別 action）を足して、踏切 action 開始時にロボットが実際に `dz_0` で降下している状態を作る」。別 action の primitive スタンプ・接続方法の設計が要る。 |
| W3 | **PRELOAD 区間だけ GRF 上限を絞る**（`u_ub` z を 150 → 例 90〜110 N/脚）／または名目 GRF を体重付近から緩やかに増やす | PRELOAD の跳ね上がりすぎを抑え、ホップ高さのばらつきを減らす（仮説 8-2） | `quad_nlp`（primitive で u_ub をゲート）または `go2.yaml` |
| W4 | NMPC **horizon_length を伸ばす**（26 → 34〜40、ジャンプ全長＋余裕）。`period` との関係（`horizon > period/dt`）も維持 | 着地相までホライズンに入れ、着地直前の最適化を近視眼的にしない（仮説 8-5） | `local_planner.yaml` |
| W5 | 飛翔中だけ **Cartesian swing ゲインを有効化**（`swing_kp_cart` を [0,0,0] → 例 [400,400,400]、`kd_cart` も）し、足先目標を CoM 直下の矩形スタンスへ | 着地姿勢を能動的に作り、着地衝撃・姿勢外乱を再現可能にする（仮説 8-3, 8-4） | `go2.yaml`、足先目標は `local_footstep_planner` の FLIGHT/FRONT_LAND 区間 |
| W6 | 逆動力学の **stance_kd を着地相で増やす**（例 4 → 8〜12）／着地検出でゲインスケジュール | 着地衝撃を吸収し、跳ね返り・二次離地を減らす | `go2.yaml`、または ID コントローラに primitive 連動ゲイン |

### 9.3 段階（各段で計測 → 判定 → 次へ）

| Stage | 内容 | 合格条件（計測） |
|---|---|---|
| A | G1+G3+W1 だけ入れて `hop_sym2` を **5 回**再走 | 5/5 で 着地後 2 s |roll|,|pitch| < 0.1 rad、転倒 0、NMPC 失敗 0。ホップ高さの分散を記録 |
| **A 実測** | — | `scripts/trial/step17_stageA.sh`。**実行された 12 回のジャンプすべてが直立着地・転倒 0・NMPC 失敗 0**（Stage A 相当設定：G1 gait=実質STAND、G3 stand_pos_error 0.15、W1 姿勢重み 20）。ホップ高さ 0.20〜0.25 m（±約 8%）、四脚離地 238〜290 ms、着地後 2 s の \|roll\|,\|pitch\| は 11/12 で < 0.1 rad。飛翔中ピッチのピークは毎回 0.26〜0.43 rad（戻る）。ジャンプ不発が 2 回あったが、いずれも起動タイミングのバグで、修正済み：(1) STAND 中に publish され body plan が無視 → GBP を `control/mode` 購読させ **WALK(2) になるまで組まない**、(2) `--once` の WALK を購読接続前に取り逃し → **`ros2 topic pub -r 10 -t N` のリピータ**に変更。**Stage A は事実上クリア**。残るばらつき（ホップ高さ ±8%・飛翔ピッチ ~0.3 rad）は Stage B/C/D の対象。 |
| B | W2（滑らかな胴体高さ基準）を追加 | PRELOAD の胴体 `vz` が単調（跳ね上がり無し）。ホップ高さ分散が Stage A より縮小 |
| **B 実測** | naive W2（`dz_0` clamp + 再ソルブ） | **失敗・撤回**（上表 W2 実測）。Stage A の強いジャンプの方が「速く綺麗に離陸する」ぶん安定していた。正しい W2（本物のしゃがみ区間）は未実装。**現時点の到達点は Stage A**：その場・垂直ジャンプが 12/12 で直立着地・転倒 0・NMPC 失敗 0。ユーザー目標「垂直に跳んでこけずに着地」は Stage A で満たしている。飛翔中ピッチ ~0.3 rad・ホップ高さ ±8% の詰めは C/D＋正しい W2 の追加作業。 |
| C | W3（PRELOAD の GRF 絞り）を追加 | PRELOAD の GRF が上限非張り付き。ホップ高さが目標 ±20% に収まる |
| D | W4（horizon 延長）＋W5（飛翔 Cartesian ゲイン）＋W6（着地 kd） | 着地時 |roll|,|pitch| < 0.05 rad、鉛直着地速度 < 1.0 m/s、二次離地なし、5/5 |
| E | G2（明示的な着地後 hold）を入れ、ジャンプ→hold→通常制御 の遷移を確認 | 着地 2 s 後に通常制御へ戻しても直立維持、5/5 |
| F | dz を段階的に上げてホップ高さを伸ばす（0.15 → 0.25 → 0.35 m）。各高さで D の条件 | 目標高さで D 合格。破綻する高さを記録（限界 Map） |

段階を飛ばさない。Step 17 の `hop_v0`（いきなり後脚踏切＋強い dz）で反転したのが教訓。

### 9.4 やらないこと（今回のスコープ外）

- 前進距離を伸ばす（前向き GRF は 0 のまま）。
- 後脚のみ踏切（`REAR_PUSH`）・前脚のみ着地（`FRONT_LAND`）を効かせる。これは GBP に
  姿勢基準と後脚接地点まわりのモーメント拘束（分析の問題 C）が要る**別課題**。まず
  四脚対称の垂直ジャンプを堅牢化してから。
- 穴シナリオ。

---

## 10. 事実 / 計測 / 推測

**事実（コードで確認）**
- 接触スケジュールは `computeContactSchedule` で primitive により上書きされる
  （`local_footstep_planner.cpp`）。強制ジャンプのホライズンはほぼ全部ジャンプ primitive。
- `local_planner` はロボットがプラン終端に近づくと `control_mode_ = STAND`（`local_planner.cpp` の
  `getReferenceFromGlobalPlan`）。STAND では接触は全脚 `{1,1,1,1}`。
- 逆動力学の Cartesian swing ゲイン `swing_kp_cart/kd_cart` は `go2.yaml` で `[0,0,0]`（無効）。
  stance/swing 関節 PD は `[60,60,60]/[4,4,4]`。
- NMPC simple model の脚別鉛直 GRF 上限は `u_ub` z = 150 N/脚（`go2.yaml`）。
- NMPC `x_weights` の roll/pitch（4,5 番目）既定は 0.5（`go2.yaml`）。
- NMPC ホライズン 26 × dt 0.03 = 0.78 s（`local_planner.yaml`）。
- `getRandomLeapAction` は `dz_impulse ∈ [dz0_min,dz0_max]`、`t_s_leap ∈ [t_s_min,t_s_max]` を
  一様乱数（`planning_utils.cpp`）。範囲をつぶすと解探索が成立しない。

**計測（2026-09-03、平地 `flat_wide`、その場）**
- `hop_sym2` / `hop_rep1`（四脚対称・姿勢重み 20、同条件 2 回）：胴体 0.318 → 0.54 m
  （+0.226 / +0.219 m）、四脚離地 264 / 256 ms、NMPC 失敗 0、着地後 2 s |roll|,|pitch| は
  `hop_sym2` で < 0.003 rad、飛翔中ピーク |pitch| は両者 0.31 / 0.33 rad、t=41 s 直立静止、
  最大関節トルク 39.7 / 40.2 Nm。
- `hop_v0`（後脚のみ踏切・姿勢重み既定）：離地するが pitch −1.4 rad → roll ±3.14 rad、
  NMPC 失敗 329、反転着地。
- `fwd_a`（前向き GRF 大）：反転、NMPC 失敗 318。
- PRELOAD 中の NMPC GRF が全脚 150 N/脚（上限）に張り付いていた（`hop_v0` のログ抜粋で確認）。

**推測（未検証）**
- 滑らかな胴体高さ基準（W2）で PRELOAD の跳ね上がりが消え、ホップ高さ分散が縮む。
- 飛翔中 Cartesian ゲイン（W5）で着地姿勢のばらつきが減り、着地衝撃が読める。
- 姿勢重みを恒常的に上げると他条件で NMPC が解けにくくなる恐れ。掃引が要る。
- horizon 延長で着地相の最適化が改善する。
- 実機のトルク/速度余裕（今回は MuJoCo のみ）。

---

## 11. 関連

- [Step 17 実装記録](./step_17_forward_jump_rear_leg_push.md)（強制ジャンプ経路・その場/前方ジャンプの計測）
- [Step 17 実装前分析](./step_17_forward_jump_code_analysis.md)（問題 A〜E、点質量 GBP の限界＝問題 C）
- コード：`global_body_planner/src/global_body_planner.cpp`（`buildForcedJumpPlan`/`forcedJumpSpinOnce`）、
  `local_planner/src/local_footstep_planner.cpp`（`computeContactSchedule`）、
  `nmpc_controller/src/quad_nlp.cpp`（`contact_sequence_`・GRF 境界）、
  `robot_driver/src/controllers/inverse_dynamics_controller.cpp`（stance/swing ゲイン）。
- ハーネス：`scripts/trial/run_step17_jump.sh`、`src/trial/quadsdk_step17_jump.py`、
  `scripts/trial/step17_analyze.py`。
