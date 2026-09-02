# Step 17（実装前分析）：Go2 前方ジャンプ — 後脚踏切パイプラインの現状と問題

対象読者：この課題に初めて触れる人。
対象コード：`external/quad-sdk` の Global Body Planner / Local Planner / NMPC / Inverse Dynamics。
位置づけ：**まだ 1 行も実装していない。** これは Step 17 課題ドキュメント第 4〜5 節で要求された「実装前のコード分析」である。
この分析の結論として、**大規模な設計変更が必要**と判明したため、実装に入る前にここへ根拠をまとめる（課題第 5 節「大規模な設計変更が必要と判明した場合は、実装前に理由を報告すること」）。

---

## 1. 背景

Go2 に穴（幅 0.30 m、深さ 1.0 m）を「またぐ」のではなく「飛び越え」させたい。
理想の一連は次の 8 つ：全脚接地で腰を落とす → 重心を後方へ → 前脚離地 → 後脚 2 本へ荷重集中 → 後脚で前上方へ踏切 → 四脚離地（飛翔）→ 前脚から着地 → 後脚着地で安定化。

Quad-SDK には既にリープ（跳躍）の枠組みがある（`LEAP_STANCE / FLIGHT / LAND_STANCE`）。
本書はその枠組みが上の 8 段階のうちどこを表現できて、どこができないかを、関数と行番号で示す。

脚順序（本書・課題共通）：`0=FL 左前, 1=RL 左後, 2=FR 右前, 3=RR 右後`。
接触ベクトル：`ALL={1,1,1,1}` `REAR_ONLY={0,1,0,1}` `NO_CONTACT={0,0,0,0}` `FRONT_ONLY={1,0,1,0}`。
（go2.yaml の `phase_offsets:[0.0,0.75,0.5,0.25]` とコメント「FL→BR→FR→BL」から index1=RL・index3=RR を確認。）

---

## 2. 目的

- 現在のジャンプ処理のデータフローを関数単位で書き出す。
- 「後脚だけで踏み切る」が現在のコードで**成立しない理由**を特定する。
- 課題第 4 節の問題 A〜E が実在するかを行番号で確認する。
- 変更予定ファイル・最小変更案・拡張案・既存歩行への影響を挙げる。
- 事実（行番号付き）と推測（未検証の見込み）を分ける。

---

## 3. 結論（先に書く）

1. **現在の「リープ」は実質「全脚接地スクワット → （運が良ければ）四脚同時飛翔 → 全脚接地」**であり、
   「後脚だけの支持（REAR_PUSH）」も「前脚だけの着地（FRONT_LAND）」も**到達不能**。
   → 課題の問題 B は実在。`local_footstep_planner.cpp:531-538` の後脚接触分岐は**デッドコード**。

2. **Global Body Planner（GBP）は点質量＋単一合力モデル**。`Action` 構造体（`planning_utils.hpp:243-251`）は
   胴体全体の合力 `grf_0 / grf_f` しか持たず、**後脚への荷重分配も接地点まわりのピッチモーメントも表現できない**。
   → 問題 C は実在。「後脚 2 本に荷重を集中して前上方へ」を GBP のまま計画するのは不可能。

3. **踏切の水平力の向きが乱数**（`planning_utils.cpp:626-631`、`refineStance()`）。
   着地点方向から必要インパルスを計算していない。
   → 問題 D は実在。

4. **通常接続がリープより先に採用される**（`rrt.cpp:21-37`、`newConfig()`）。
   `attemptConnect()` が `REACHED` を返すと即 `return true` で、リープ候補サンプリング（47-84 行）に入らない。
   → 問題 A は実在。ただし 30 cm 穴を通常歩容で「またげる」現状（Step 16）では、そもそもリープが選ばれない。

5. **NMPC も Inverse Dynamics も「計画上の接触」しか見ていない**。
   実測接触（センサ / シミュレータ接触）は torque 経路のどこにも入っていない。
   → 問題 E：現状は「計画接触オンリー」。`body_force_estimator` パッケージは存在するが ID コントローラは未使用。

6. **primitive ID が 3 ファイルに生の整数で重複定義**されている
   （`planning_utils.hpp:179` の `enum`、`local_footstep_planner.hpp:558/564`、`rviz_interface.hpp:255/257`）。
   課題第 6 節「複数ファイルへ数値で重複定義しないこと」に反する既存状態。新フェーズ追加時に共通化が要る。

7. **NMPC の脚別 GRF 上限（simple model）は鉛直 150 N/脚**（`go2.yaml:46-47`）。
   後脚 2 本で 300 N ≈ 体重の約 1.9 倍。第 8 節の踏切に必要なピーク（推定 ≈ 450〜480 N 合計）に**届かない**。
   → 上流でジャンプ可能と判定しても、NMPC 制限で実現不能になる領域がある（課題第 8 節が警告する状況が実在）。

**総括：課題が要求する `PRELOAD / REAR_PUSH / FLIGHT / FRONT_LAND / SETTLE` を「計測値で確認できるジャンプ」として
実装するには、接触スケジュールの差し替えだけでは足りず、GBP の力モデル・踏切方向の決定・NMPC の脚別 GRF 上限と
名目配分・（着地判定のための）実測接触の導入、の複数レイヤにまたがる変更が要る。**

---

## 4. データフロー（現状）

```
global_body_planner ノード (global_body_planner.cpp)
  ├ enable_leaping パラメータ読み込み  :70-74
  │   false なら num_leap_samples=0（リープ完全停止）
  └ GBPL::findPlan (gbpl.cpp:165)  … 双方向 RRT-Connect
       ├ extend() → RRT::newConfig (rrt.cpp:11)
       │    ├ (1) attemptConnect() を先に試す  :21
       │    │      REACHED なら即 return true（★問題A：リープに入らない）:34-36
       │    └ (2) 届かない時だけ getRandomLeapAction を num_leap_samples 回  :47-84
       ├ getRandomLeapAction (planning_utils.cpp:553)
       │    dz_impulse ∈ [dz0_min,dz0_max]=[1.0,2.0] m/s を一様乱数  :560
       │    t_s_leap  ∈ [t_s_min,t_s_max]=[0.12,0.25] s を一様乱数    :565
       │    t_f = 1e-6（あとで refineFlight が伸ばす）                :568
       │    → refineAction
       ├ refineAction (planning_utils.cpp:578)
       │    refineStance(LEAP_STANCE) → applyStance → refineFlight → applyFlight → refineStance(LAND_STANCE)
       ├ refineStance (planning_utils.cpp:598)
       │    ang_az        = 2π·rand()/RAND_MAX          （★問題D：水平力の向きが乱数）:626
       │    f_lateral_mag = rand()/RAND_MAX · mu                                        :627
       │    grf_stance.xy = f_lateral_mag·f_z·(cos,sin)(ang_az)                         :629-631
       │    grf_stance.z  は高さ境界条件から解く                                        :650-654
       │    friction cone（|xy| ≤ μ·z）を強制                                           :665-690
       └ 出力：state_sequence / action_sequence（Action=単一合力モデル）
              global_body_plan.cpp が primitive_id_plan_ を作る
                LEAP_STANCE を push                                    :65
                contact_state = (primitive_id != FLIGHT)              :100
              → RobotPlan.msg / BodyPlan.msg（uint32[] primitive_ids）

local_planner (local_planner.cpp)
  ref_primitive_plan_(i) = body_plan_msg_->primitive_ids[i+idx]       :514,521
  LocalFootstepPlanner::computeContactSchedule (local_footstep_planner.cpp:511)
    名目歩容をタイル展開（crawl: period 0.9s, duty 0.75×4）           :515-528
    リープ上書きループ                                                :530-545
      if ref==LEAP_STANCE:
        leading_leg_liftoff_idx = min(i, H-1)  = i                    :532
        if ref(i)==FLIGHT  → {0,1,0,1}   ← ★問題B：LEAP分岐の中なので常に偽（デッドコード）:534-535
        else               → {1,1,1,1}                                :537
      elif ref==FLIGHT     → {0,0,0,0}                                 :539-541
      elif ref==LAND_STANCE→ {1,1,1,1}                                 :542-543
  → contact_schedule（bool 4×H）

nmpc_controller (nmpc_controller.cpp:283,310 → quad_nlp.cpp:1742 updateContactSchedule)
  contact_schedule（計画接触）を contact_sequence_(4×N) にコピー       :1752-1757
  脚別 GRF 境界 u_lb/u_ub × contact でゲート                          :189-191
  名目 GRF は接地脚へ mass·g/num_contacts を等分                       :487-493 ほか（★対称配分のみ）
  simple model 脚別上限：z ∈ [10, 150] N/脚（go2.yaml:46-47）
  friction_coefficient 0.3（nmpc yaml）/ 0.6（go2 complex）/ GBP mu 0.25

robot_driver / InverseDynamicsController (inverse_dynamics_controller.cpp)
  grf_array = last_local_plan_msg_->grfs[i]（NMPC 出力の ZOH）         :96
  contact_mode[i] = ref_state_msg_.feet.feet[i].contact（★計画接触）  :120-123
  computeInverseDynamics(acc, grf, contact_mode, tau)                  :126-127
  接地脚= stance_kp/kd 位置保持、遊脚= Cartesian PD + swing gain       :163-176
  → 関節 pos/vel/torque_ff → MuJoCo / Gazebo
```

---

## 5. 各関数：入力・処理・出力・単位

| 関数（ファイル:行） | 入力 | 処理 | 出力 | 単位 |
|---|---|---|---|---|
| `RRT::newConfig` (rrt.cpp:11) | s（目標）, s_near, planner_config | まず直結 `attemptConnect`。届けば即終了。届かねばリープを乱数サンプリング | s_new, a_new（採用アクション） | 位置 m / 速度 m/s |
| `RRT::attemptConnect` (rrt.cpp:97) | s_existing, s, t_s | 3 次補間で加速度→合力。`isValidAction`→`isValidStateActionPair` | REACHED / ADVANCED / TRAPPED, Action | 合力 [体重倍] |
| `getRandomLeapAction` (planning_utils.cpp:553) | s, surf_norm | dz_impulse・t_s_leap を一様乱数、`refineAction` | Action（1 個の合力軌道） | m/s, s |
| `refineStance` (planning_utils.cpp:598) | s, phase(LEAP/LAND), a | **水平力の向き=乱数**、鉛直力=高さ境界条件、friction cone 強制、t_s / dz_0 を反復調整 | a.grf_0 or a.grf_f, t_s | N（内部は体重倍） |
| `refineFlight` (planning_utils.cpp:744) | s（離陸状態）, t_f | 弾道で高さが `h_nom+0.05` に戻る時刻まで t_f を伸ばす | t_f | s |
| `getGRF` (planning_utils.cpp:275) | a, t, phase | LEAP/LAND は時間の放物線（中点ピーク）、FLIGHT=0、CONNECT=一定 | GRF（合力） | N |
| `global_body_plan.cpp:computeState` (:76) | state, GRF, primitive_id | `contact_state=(primitive_id!=FLIGHT)`、メッセージ化 | RobotPlan/BodyPlan | — |
| `LocalFootstepPlanner::computeContactSchedule` (:511) | current_plan_index, body_plan, **ref_primitive_plan**, control_mode | 名目歩容タイル→リープ primitive で上書き | contact_schedule（bool 4×H） | — |
| `quad_nlp.cpp:updateContactSchedule` (:1742) | contact_schedule | contact_sequence_ にコピー、接触変化点で名目 GRF を再配分 | contact_sequence_, u_nom | N |
| `InverseDynamicsController::computeLegCommandArray` (:8) | robot_state, last_local_plan_msg_ | plan を時間補間、GRF は ZOH、**contact は計画値**、逆動力学で torque_ff | LegCommandArray（pos/vel/torque） | rad, rad/s, Nm |

---

## 6. 後脚踏切が成立しない原因（核心）

### 6.1 接触スケジュール上の原因（問題 B）— 実在・修正は容易

`local_footstep_planner.cpp:530-545`：

```cpp
if (ref_primitive_plan(i) == LEAP_STANCE) {
  int leading_leg_liftoff_idx = std::min(i, horizon_length_ - 1);   // = i（H-1未満のiでは常に i）
  if (ref_primitive_plan(leading_leg_liftoff_idx) == FLIGHT) {      // 外側で LEAP_STANCE と確定済み → 常に偽
    contact_schedule.at(i) = {false, true, false, true};            // REAR_ONLY … 到達不能
  } else {
    contact_schedule.at(i) = {true, true, true, true};              // 実際はいつもこちら
  }
}
```

`leading_leg_liftoff_idx == i` であり、この分岐に入る条件が既に `ref_primitive_plan(i) == LEAP_STANCE` なので、
`ref_primitive_plan(i) == FLIGHT` が同時に成り立つことは絶対にない。よって `{false,true,false,true}` は**実行されない**。
結果、リープの支持相は常に四脚接地。**前脚だけを先に離す/後脚だけで支える、という区別が存在しない。**

→ 単に `i+1` に変えるだけでも「次サンプルが FLIGHT なら現サンプルを後脚のみ」にはできるが、
**時間幅が 1 サンプル（30 ms）しかなく**、課題が求める「後脚支持 50〜150 ms・前脚を先に離地」を安定に作れない。
**明示的な時間幅つき `REAR_PUSH` フェーズ（および `PRELOAD` / `FRONT_LAND` / `SETTLE`）を primitive として持つべき。**

### 6.2 力モデル上の原因（問題 C）— 実在・修正は大きい

`Action`（`planning_utils.hpp:243-251`）は
`grf_0, grf_f`（各 `Eigen::Vector3d`＝**胴体全体の合力**）, `t_s_leap, t_f, t_s_land, dz_0, dz_f` のみ。
GBP の運動方程式は点質量（`getAcceleration = GRF/m + g`）。したがって GBP は原理的に

- 「後脚 2 本に荷重、前脚 0」という**配分**
- 後脚接地点まわりの**ピッチモーメント**（前上方に蹴ると鼻上げになる）
- 左右後脚の**差**（ロール・横滑り抑制）

を一切表現できない。「後脚だけで前上方へ踏み切る」を GBP レベルで正しく計画するのは、
`Action` に脚別 GRF（少なくとも rear L/R の Fx,Fz）と接地点、ピッチ拘束を足さない限り不可能。

### 6.3 踏切方向の原因（問題 D）— 実在

`refineStance`（`planning_utils.cpp:626-631`）は水平 GRF の**方位角 `ang_az` を一様乱数**、
**大きさも `rand()·μ`**。着地点（`s_goal.pos - s.pos` の水平方向）を全く見ていない。
ジャンプでは「必要水平インパルス `J_x = m(v_x,takeoff − v_x,start)` を着地点方向へ」出すべきで、乱数では
方向が合う確率が低く、`num_leap_samples=10` では前方ジャンプがまず当たらない。

### 6.4 選択順の原因（問題 A）— 実在（ただし副次的）

`rrt.cpp:21-37`：`attemptConnect` が `REACHED` を返すと即 `return true`。
30 cm 穴は現状の crawl 歩容で「またげる」ので通常接続が成功し、リープは**候補にすら上がらない**。
→ 「ジャンプ強制モード（FORCE_LEAP）」では、`attemptConnect` が成功しても確定させない分岐が必要。

### 6.5 実測接触を見ていない（問題 E）— 実在

- NMPC：`quad_nlp.cpp:1752` で**計画** `contact_schedule` をそのまま `contact_sequence_` に。
- ID：`inverse_dynamics_controller.cpp:120-123` で **計画** `ref_state_msg_.feet.feet[i].contact`。
- torque 経路に足裏力センサ / シミュレータ接触は入らない。
- `body_force_estimator`（外力推定パッケージ）は存在するが ID は購読していない。
→ 「四脚が実際に離地したか」「前脚が本当に先に着いたか」は**現状のコードでは判定材料にならない**。
   Stage 4/6 の成功判定（実測接触 30 ms 以上 OFF 等）には、**実測接触を購読して CSV に出す経路の新設**が要る。

---

## 7. 課題の問題 A〜E：確認結果

| 問題 | 確認 | 根拠（ファイル:行） |
|---|---|---|
| A：通常移動がジャンプより先 | **実在** | `rrt.cpp:21-37`（`REACHED` で即 return、リープループ 47-84 に未到達） |
| B：後脚だけの接触分岐が到達不能 | **実在（デッドコード）** | `local_footstep_planner.cpp:531-538`（`leading_leg_liftoff_idx==i`、LEAP分岐内で FLIGHT 判定） |
| C：GBP が四脚別 GRF を持たない | **実在** | `planning_utils.hpp:243-251`（`Action` は合力 2 本のみ）、`getAcceleration` 点質量（`planning_utils.cpp:301-305`） |
| D：水平 GRF が目標方向から決まらない | **実在** | `planning_utils.cpp:626-631`（`ang_az` 一様乱数、`f_lateral_mag=rand()·μ`） |
| E：計画接触と実接触の混同 | **現状は計画接触オンリー** | NMPC `quad_nlp.cpp:1752`、ID `inverse_dynamics_controller.cpp:120-123` |
| （追加）primitive ID の重複定義 | **実在** | `planning_utils.hpp:179` / `local_footstep_planner.hpp:558,564` / `rviz_interface.hpp:255,257` |
| （追加）NMPC 脚別 GRF 上限が低い | **実在** | `go2.yaml:46-47`（simple model：z ∈ [10,150] N/脚） |

---

## 8. 必要な離陸速度・インパルス（計算＝推定）

パラメータ：m = 16.1 kg、g = 9.81 m/s²、体重 W = mg ≈ 158 N。
同高着地の弾道近似：`t_flight = 2 v_z/g`、`L = v_x t_flight = 2 v_x v_z / g`。

目標飛距離 L（穴 0.30 m ＋ 踏切余裕 0.05〜0.10 ＋ 着地余裕 0.05〜0.10）＝ **0.40〜0.50 m**。L = 0.45 で試算。

| 量 | 式 | 値（v_x=v_z, 開始 v≈0.3 m/s, T_push=0.15 s） |
|---|---|---|
| 離陸速度（対称） | v = √(Lg/2) | **≈ 1.49 m/s**（v_x≈v_z≈1.49） |
| 飛翔時間 | 2 v_z/g | **≈ 0.30 s** |
| 水平インパルス | J_x = m(v_x,to − v_x,st) | 16.1·(1.49−0.3) ≈ **19.2 N·s** |
| 鉛直インパルス | J_z = m(v_z,to − v_z,st) + m g T_push | 16.1·1.49 + 158·0.15 ≈ **47.7 N·s** |
| 後脚合計 平均 F_x | J_x / T_push | ≈ **128 N**（片脚 ≈ 64 N） |
| 後脚合計 平均 F_z | J_z / T_push | ≈ **318 N ≈ 2.0 W**（放物線ピーク ≈ 1.5× ≈ **477 N ≈ 3.0 W**） |
| 滑らない条件 | F_x ≤ μ F_z | 128 ≤ μ·318 → **μ ≥ 0.40 必要**（現状 GBP μ=0.25 / NMPC 0.3 では滑る） |

**帰結（推定）**

- **NMPC simple model の脚別上限 150 N/脚 → 後脚 2 本で 300 N**。必要ピーク ≈ 477 N に**届かない**。
  ジャンプモードでは後脚の `u_ub` z を引き上げる（例 300〜400 N/脚）か、T_push を伸ばして必要ピークを下げる必要。
- **摩擦係数が足りない**。GBP `mu=0.25`、NMPC `friction_coefficient 0.3` では前上方推進で滑る。
  ジャンプ用に μ を 0.4〜0.6 に上げる（Go2 実足裏 ≈ 0.6 は妥当）か、v_x を落として v_z を上げる配分にする。
- Go2 の関節トルク・速度上限（`go2.yaml` complex u ±99.9、要確認）との整合は未検証。
- 鼻上げモーメント：後脚接地点より CoM が前にある状態で前上方 GRF → nose-up。
  GBP は表現不可（問題 C）。NMPC の姿勢重み＋前脚早期離地タイミングで抑えるしかなく、**要実測検証**。

---

## 9. 変更予定ファイル

### 9.1 最小変更案（接触スケジュールを「到達可能」にする。力モデルは触らない）

| ファイル | 変更 |
|---|---|
| `global_body_planner/include/global_body_planner/planning_utils.hpp` | `enum Phase` を末尾拡張：`{CONNECT, LEAP_STANCE, FLIGHT, LAND_STANCE, PRELOAD, REAR_PUSH, FRONT_LAND, SETTLE}`（既存値 0-3 は固定）。**単一の定義元にする** |
| `local_planner/include/local_planner/local_footstep_planner.hpp` `quad_utils/include/quad_utils/rviz_interface.hpp` | 生の `const int LEAP_STANCE=1...` 重複を廃し、`planning_utils` の enum（または新設の共通ヘッダ `quad_msgs` / `quad_utils` 内の `primitive_ids.hpp`）を include |
| `global_body_planner/src/global_body_plan.cpp` | リープ区間の primitive を `LEAP_STANCE` 一択（:65）から `PRELOAD→REAR_PUSH→FLIGHT→FRONT_LAND→SETTLE` の時系列で push。`computeState` の `contact_state=(primitive_id!=FLIGHT)`（:100）は新 ID も接地扱いで OK |
| `global_body_planner/src/planning_utils.cpp` | `interpStateActionPair`（:344）/ `interpStateActionPairMinimal`（:996）：離陸前 stance を時間で分割し `PRELOAD` / `REAR_PUSH` を、着地後 stance を `FRONT_LAND` / `SETTLE` をスタンプ。`getGRF`（:275）に新フェーズ分岐 |
| `local_planner/src/local_footstep_planner.cpp` | `computeContactSchedule`（:530-545）のデッドコードを、新 primitive → 接触表へのマッピングに差し替え：`PRELOAD/SETTLE→{1,1,1,1}` `REAR_PUSH→{0,1,0,1}` `FLIGHT→{0,0,0,0}` `FRONT_LAND→{1,0,1,0}` |
| `global_body_planner/src/rrt.cpp` | `jump_mode==FORCE_LEAP` では `attemptConnect`→`REACHED` でも即 return しない（:34-36 をモード分岐） |
| `global_body_planner/src/global_body_planner.cpp` / `PlannerConfig` | `global_body_planner.jump_mode` ∈ `{OFF, AUTO, FORCE_LEAP}` を追加。`enable_leaping` との関係：`OFF`⇔`enable_leaping:=false` 相当、`AUTO`=現状、`FORCE_LEAP`=新規。後方互換のため既定 `AUTO` |
| `*/config/*.yaml` | `jump_mode`（既定 AUTO）、ジャンプ用 `t_s_push`, `dz0`, 目標飛距離 `jump_target_distance`, μ override |
| `quad_utils/src/rviz_interface.cpp` | `:340-345` の primitive→色分けに新 ID |
| `global_body_planner/test/…` `local_planner/test/test_footstep_planner.cpp` | Stage 1 ユニットテスト（`PRELOAD=1111` `REAR_PUSH=0101` `FLIGHT=0000` `FRONT_LAND=1010` `SETTLE=1111` ＋ 既存 trot / STAND 回帰） |

### 9.2 拡張案（後脚踏切を物理的に成立させる）

| 対象 | 変更 |
|---|---|
| `Action`（`planning_utils.hpp`） | 後脚別 GRF（rear L/R の Fx,Fz）＋接地点＋ピッチ拘束を持つジャンプ用アクション（別構造体でも可）。GBP 検証関数群に反映 |
| `refineStance` ジャンプ分岐 | 水平力の向きを乱数（:626）から**着地点方向**へ。大きさは `J_x/T_push` から。friction cone は μ_jump で |
| `quad_nlp` / `nmpc yaml` | `REAR_PUSH` 区間で後脚 `u_ub` z を引き上げ、名目 GRF を等分（:487-493）から**後脚偏重**へ。μ_jump 反映 |
| 実測接触 | シミュレータ接触 or `body_force_estimator` を購読し、着地検出で `SETTLE` へ遷移／CSV へ measured contact を出す経路を新設 |
| Go2 トルク・速度上限 | 必要 GRF に対する関節トルク／速度の余裕を確認し、不足なら T_push・飛距離を調整 |

---

## 10. 既存歩行への影響

- 新フェーズは**ジャンプアクション採用時のみ** GBP が emit する。`computeContactSchedule` は primitive ID でしか上書きしないので、
  通常の trot / crawl / STAND は無変更。
- `enable_leaping:=false` / `jump_mode:=OFF` で `num_leap_samples=0` → ジャンプ経路は完全に無効化（現状の安全弁を維持）。
- リスク：`primitive_ids == LEAP_STANCE / LAND_STANCE / FLIGHT` を見る箇所（`rviz_interface.cpp:340-345` など）で
  新 ID を未処理にすると可視化が崩れる。grep 済みの 3 ファイル以外に `switch(primitive)` が無いことは確認済み。
- メッセージ互換：`RobotPlan.msg` / `BodyPlan.msg` の `uint32[] primitive_ids` は型不変。値の追加のみ。
- Step 14/15/16 の multistep 停止・足場差し込みは local_footstep_planner の別関数（`step12PlanSequence` 等）。今回の
  `computeContactSchedule` 変更とは独立。ただし回帰スイープ（`scripts/trial/step16_*`）で OFF 一致を再確認すべき。

---

## 11. 事実と推測の分離

**事実（コードと行番号で確認）**
- 問題 A〜E＋ID 重複＋NMPC 脚別上限 150 N は本文の行番号どおり実在。
- リープ支持相は常に四脚接地（`{false,true,false,true}` はデッドコード）。
- NMPC / ID は計画接触のみ使用。
- primitive ID は 3 ファイルに重複定義。
- パラメータ現値：m 16.1kg, μ(GBP) 0.25, μ(NMPC) 0.3, t_s [0.12,0.25]s, dz0 [1,2]m/s, u z/脚 [10,150]N, crawl period 0.9s duty 0.75。

**推測（未検証）**
- 第 8 節のインパルス値（v_x/v_z 配分・T_push・開始速度の仮定に依存）。
- 「μ を 0.4〜0.6 に上げ、後脚 u_ub を 300〜400 N に上げ、前脚早期離地タイミングを詰めれば 30 cm 穴を飛べる」――
  Stage 2〜6 の実測でしか確認できない。
- 鼻上げモーメントが NMPC 姿勢重みで抑えられるか。
- FORCE_LEAP で計画が出た後、NMPC が追従してソルバが発散しないか。

---

## 12. 未解決の前提（実装着手前に確認が要る）

1. **作業ブランチ**：課題第 2 節は「`quad_sdk` リポジトリで `devel_ros2_review` を基点に `feature/jump`」を指示。
   実際は単一リポジトリ `/home/takuya/work/mpc_dog`（ブランチ `main`）で、`external/quad-sdk` は submodule ではなく
   直接追跡のディレクトリ。`devel_ros2_review` ブランチは存在しない。Step 14/15/16 はすべて `main` にコミット済み。
   → どのブランチを基点にするか要指示。
2. **今セッションの到達目標**：フル Step 17（enum 拡張〜Stage 6 の穴越え＋GIF/CSV）はレイヤ横断の大改修＋
   複数のシミュレーション回帰を含む。分析＋最小変更（enum・接触スケジュール・FORCE_LEAP・Stage 1 ユニットテスト）
   までか、Stage 2〜3 の平地検証まで踏み込むか、フルかで作業量が大きく変わる。
