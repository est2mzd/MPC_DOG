# Step 03_1m / 04_1m(gbpl 版):global_body_planner で深い穴を渡る + Quad-SDK マニュアル調査

ブランチ `feature/apply_global_planner` の実験記録。

対象: `external/quad-sdk` の **global_body_planner(GBP-L)** モード
(`reference:=gbpl`)。main の twist モード + クロール歩容による解
(`step_03_04_1m_quadsdk_gap_crossing.md`)とは別アプローチ。

**目的**: (1) Quad-SDK 公式マニュアル(GitHub Wiki / doxygen / 論文)を読み、
穴越え・歩容・地形マップの**正しい設定方法**を整理する。(2) それに沿って
GBP-L で step03_1m / step04_1m の深い穴を渡れるか確かめる。

---

## 結論(先に)

### マニュアルが言う「正しい設定方法」

1. **穴越えの正攻法は `reference:=gbpl`(GBP-L)であり、これが既定。**
   `twist` は手動運転用のフォールバック
   ([Wiki 5. Using the Software (ROS2)])。
2. **ゴールの与え方**は 2 通り:RViz から `/clicked_point` を publish、
   または `global_body_planner.goal_state: [x, y]` パラメータ
   ([Wiki 2. Using the Software])。跳躍は `leaping:=true`(既定)。
3. **地形マップは物理ワールドとは別に用意する**。CAD で地形を作り、
   **`.stl`(Gazebo 用)と `.ply`(grid_map 用)の両方**を、
   **最大ファセットサイズ 0.20 m**、単位メートルでエクスポートし、
   モデルフォルダに置く。grid_map は `.ply` から
   `grid_map_resolution: 0.05` m で生成される
   ([Wiki: Creating Custom Terrain Map Files])。
   → **穴・段差はメッシュのジオメトリそのもので表現する**(専用のフラグや
   レイヤは無い)。「メッシュに面が無い所は穴」というのが素直な作り方。
4. **歩容パラメータ**(`period` / `duty_cycles` / `phase_offsets`)は
   `quad_utils/config/<robot>.yaml` の `local_footstep_planner:` にあり、
   `setTemporalParams(dt, period, horizon_length, duty_cycles, phase_offsets)`
   で読まれる。`duty_cycles` = 各脚の接地時間比、`phase_offsets` = 各脚の
   接地タイミング([LocalFootstepPlanner class ref])。
   **「ステップ周波数」という単独ノブは無く、自動適応もしない**
   ── 固定の接触スケジュール表。
5. **新しい(小型)ロボットを足すとき**に触るのは
   `quad_utils/config/<robot>.yaml`、`local_planner/local_planner.yaml` の
   `desired_height`、`Quad_KD.h` の関節速度・トルク上限
   ([Wiki: Adding a New Type of Robot])。
   **GBP-L の跳躍パラメータを小型ロボット向けに調整する手順は
   マニュアルに無い**(= 研究レベルの作業)。
6. **GBP-L のプランナ内部**(doxygen):
   - `anytime_horizon` … 1 回の探索に使う時間。超えたら restart。
   - `horizon_expansion_factor: 1.2` … replan 時に horizon を伸ばす倍率。
   - `planning_rate_estimate: 16.0` … 初期の想定移動速度 [m/s]。
   - `global_body_planner.yaml`:`max_planning_time`(既定 **1.0 s**)、
     `num_leap_samples`(既定 **10**)、`traversability_threshold: 0.3`、
     `t_s_min/max: 0.12/0.25`、`dz0_min/max: 1.0/2.0`。
7. **公式の但し書き**:「すべてのワールドファイルでのシームレスな動作は
   保証しない」「一部の環境は aspirational で、コミュニティの協力が必要」
   ([Wiki 5])。

### 実験結果(この地形・go2 で GBP-L を回した結論)

- GBP-L は **1 m 深・0.3 m 幅の穴 1 本**を、近いゴール(x=2.0)+
  `max_planning_time` を 1 → 10 s に延ばせば **跳んで渡れる**
  (gbpl_run5、CSV + GIF 確認済み。ゴールで直立静止)。
- **穴 2 本(x=4.0)**:GBP-L は積極的な複合 leap を出し、ロボットは
  x≈0.2→2.9 を一気に跳んで**両方の穴を越える**が、**着地で背面へ転倒**
  (gbpl_run7)。local NMPC が go2 でこの跳躍を追従しきれない。
- **穴 3 本(x=6.0)**:`max_planning_time` 18 s でも **1 本目を跨ぐ leap
  すら張れず**、x≈0.55 で停止 → 転落(gbpl_run6)。ゴールが遠いほど
  RRT-Connect の探索が発散して近傍の leap を見つけられない。
- 穴 1 本を渡れた run5 でも**横スイングが大きい**(y が一時 −1.9 m)。
  「横位置中央を保って渡る」要件は満たさない。

→ **step03/04 の課題(穴の連なる区間を姿勢を保って渡り切る)は GBP-L では
未達**。main の **twist + クロール解**(0.15〜0.5 m/s で連続 5〜6 本を
姿勢を保って踏破)が引き続き実用解。

本ブランチ `feature/apply_global_planner` は、**「GBP-L を試したが未達」
という調査結果 + 工程別のボトルネック分析(5 節)+ 実験ハーネス**を残す
ものとして `--no-ff` で main にマージする(`external/quad-sdk` への
設定変更は一切コミットしていない ── ハーネスが一時パッチして trap で
必ず戻すため、追加されるのは実験ハーネスと本 doc、README リンクのみ)。
どの工程で精度が落ちるかは 5 節にまとめた。

---

## 1. GBP-L モードとは

- **GBP-L**(Global Body Planner for Legged Robots)= RRT-Connect ベースの
  大域プランナ。2.5D 地形マップ上で start→goal の点間プランを、
  **walk / leap(stance→flight→land)混合プリミティブ**で作る
  (Norby & Johnson, "Fast global motion planning for dynamic legged
  robots," IROS 2020)。
- ゴール点 `[x, y]` で駆動。GBP-L がプランを publish → local_planner の
  NMPC が追従(`computeContactSchedule` に `LEAP_STANCE` / `FLIGHT` /
  `LAND_STANCE` 分岐あり)。
- go2 用パラメータは `quad_utils/config/go2.yaml` 冒頭の
  `/**/global_body_planner:` ブロック
  (`h_max: 0.375`, `h_nom: 0.3`, `mass: 16.1`, `robot_l: 0.3`,
  `grf_max: 5.0` bw ...)。
- 跳躍まわりは `global_body_planner/config/global_body_planner.yaml`。

## 2. ハーネス `scripts/trial/run_quadsdk_gap_gbpl.sh`

`run_quadsdk_gap_1m.sh`(twist)からの差分:

- `quad_plan.py` に `reference:"gbpl"` + `goal_state:[GOAL_X, GOAL_Y]` を渡す
  (robot_configs JSON 経由 → `global_body_planner.goal_state` param)。
  `leaping:=true`。
- `cmd_vel` は送らない。STAND → プランナ起動 → WALK → ゴールへ歩かせて待つ。
- **実験用に config を一時パッチ**して trap(EXIT/TERM/INT)で必ず戻す
  (このブランチのコミット内容は twist 解と同一に保つ):
  - `go2.yaml`: `period` / `duty_cycles` / `phase_offsets` /
    `foothold_search_radius` を素のトロットへ
    (env: `GBPL_GAIT_*`, `GBPL_FOOTHOLD_RADIUS`)
  - `local_planner.yaml`: `horizon_length` を 26 へ(env: `GBPL_HORIZON`)
  - `global_body_planner.yaml`: `max_planning_time` / `num_leap_samples`
    (env: `GBPL_MAX_PLANNING_TIME`, `GBPL_NUM_LEAP_SAMPLES`)

実行例:
```bash
GAP_WORLD=flat_gaps_2m.xml GAP_TAG=step03_1m_gbpl GOAL_X=2.0 DURATION_S=50 \
  PLAN_STARTUP_S=14 GBPL_MAX_PLANNING_TIME=10 GBPL_NUM_LEAP_SAMPLES=30 \
  bash scripts/trial/run_quadsdk_gap_gbpl.sh
```

## 3. 試行ログ

| run | world | goal x | max_plan_t | 歩容 | 結果 |
|---|---|---|---|---|---|
| 1 | flat_gaps_2m | 12.0 | 1.0 | クロール(period 0.9) | 初手で横倒れ(roll→π)。GBP-L「trapped」連発 |
| 2 | flat_gaps_2m | 12.0 | 1.0 | 素トロット | x≈0.6 まで歩いて停止 → 遅れて転倒。GBP-L「partially valid」のみ、stamp 更新されず |
| 3 | flat_gaps_2m | 2.0 | 1.0 | 素トロット | 同上。x≈0.56 で停止 → 転倒 |
| 4 | gap_40cm(素材) | 1.7 | 1.0 | 素トロット | 一歩も動かず。「Start is sufficiently close to goal」即時(素材 PLY の座標が壊れており地形マップが退化) |
| 5 | flat_gaps_2m | 2.0 | 10.0 | 素トロット | **成功(2 回中 1 回)。** 穴 1 本を渡り goal (2.0, 0) で直立静止。ただし途中の横スイング y=−1.9→0 |
| 6 | flat_gaps_2m | 6.0 | 18.0 | 素トロット | **失敗。** 1 本目を跨ぐ leap も張れず、x≈0.55 で停止 → 転落。GBP-L「partially valid」1 回のみ |
| 7 | flat_gaps_2m | 4.0 | 22.0 | 素トロット | **不成立。** 複合 leap で x 0.2→2.9 を一気に跳び穴 2 本を越えるが、**着地で背面へ転倒**(roll→π) |
| 8 | flat_gaps_2m | 2.0 | 10.0 | 素トロット | **失敗(run5 と同一条件)。** 跳躍が制御不能に発散、goal を大きく行き過ぎて x≈3.6 で転倒。→ **run5 の成功は非決定的**(2 回中 1 回) |

### run5 の軌跡(CSV、成功)

```
t=10   x=-0.01  z=0.318  (起立、待機)
t=12   x= 0.34  z=0.316  vx=0.78   (発進)
t=14   x= 1.76  z=0.303            (穴 x=0.85 を越えた)
t=16   x= 2.37  y=-0.67 z=0.310    (行き過ぎ + 横スイング)
t=18   x= 1.84  y=-1.86 z=0.313    (大きく横へ、だが直立維持)
t=22   x= 2.00  y=-0.00 z=0.306 roll=0 pitch=0  (goal に静止)
t=22..70  x=2.00 で直立静止
```

### run7 の軌跡(CSV、穴 2 本・着地失敗)

```
t=17.5 x= 0.19  z=0.328  vx=0.36
t=20   x= 2.94  z=0.457  vx=1.59 pitch=+0.13   (跳躍中。穴 x=1, x=3 を通過)
t=22.5 x= 3.52  z=0.058  roll=-π               (着地で背面転倒)
```

## 4. マニュアルと実験を突き合わせた考察

- **設定方法はマニュアル通りにできた**:`reference:=gbpl` + `goal_state`
  param + 本物のメッシュ穴(facet ≤ 0.20 m、grid_map 0.05 m)。GBP-L は
  地形マップを受領し、穴帯(`traversability` = NaN)を非踏破と判定して
  leap を要求する動作まで確認できた。
- **既定値では渡れない理由**:
  - `max_planning_time: 1.0 s` は、6 本の穴が並ぶ 12 m コースに対して
    RRT-Connect の探索時間として足りない。マニュアルは「全ワールドで
    シームレスは保証しない」と明記しており、これは既知の限界。
  - go2 は spirit より小型・低出力。GBP-L の leap パラメータ
    (`dz0 1.0–2.0 m/s`、`grf_max 5.0` bw)は spirit 前提で、
    **小型ロボット向けの調整手順はマニュアルに無い**。run7 の着地失敗は
    local NMPC が go2 でこの跳躍を追従しきれないことを示す。
  - 歩容は自動適応しない(固定表)。twist 用のクロールを入れたまま gbpl を
    使うと NMPC が GBP-L プランを追従できず即転倒(run1)。gbpl では
    素トロットに戻す必要がある。
- **結論**:GBP-L は「近いゴール + 潤沢な計画時間」で穴 1 本なら跳べるが、
  step03/04 の「穴の連なる区間を姿勢を保って踏破」には、GBP-L の
  RRT チューニング(goal bias / connect radius / anytime_horizon)と
  go2 向け leap パラメータ調整、さらに NMPC の leap 追従の頑健化が要る。
  これはパラメータ数点では届かない研究レベルの作業。実用解は main の
  twist + クロール。

## 5. パイプラインのどの工程で精度が落ちるか(シナリオ別)

**問い**: センシング → foot plan(GBP-L + local_footstep_planner)→ MPC(NMPC)
→ WBC(逆動力学)→ トルク のどこで精度が落ちて転ぶのか。
CSV(`plan_nmpc_cost` / `plan_nmpc_iterations` / `plan_compute_time_ms` /
`plan_age_s` / 接触フラグ / 各脚 GRF)とノードログ(NMPC fail、
robot_driver の effort 超過警告、GBP-L のプラン状態)で工程ごとに切り分けた。

### 各工程で見た指標

| 工程 | 見た指標 | 健全時の値(平地歩行〜穴手前) |
|---|---|---|
| センシング | `state/ground_truth`(シムなので真値) | 誤差なし(ground truth) |
| 大域プラン(GBP-L) | 「New plan published」の stamp が更新されるか / VALID or PARTIAL | 1 本渡りは 1 プラン発行、複数穴は「partially valid」のまま |
| foot plan / 接触スケジュール | `computeFootPlan` DIAG、接触フラグ列、LEAP/FLIGHT/LAND への切替 | トロットの TFFT/FTTF が規則的に交替、snap_calls 正常 |
| MPC(NMPC) | `plan_nmpc_cost`、`plan_nmpc_iterations`、`plan_compute_time_ms`、`plan_age_s` | cost ≈ 0.005〜0.03、iter = 1、compute ≈ 6 ms、age ≈ 0 |
| WBC(逆動力学) | 各脚 GRF、robot_driver「total effort … exceeds threshold」警告数 | GRF ≈ 70〜80 N/脚、effort 警告 = 0 |
| トルク → シム | 胴体 z / roll / pitch | z ≈ 0.31、roll・pitch < 0.02 rad |

### シナリオ 1:穴 3 本(goal x=6.0、run6)── **大域プラン(GBP-L)で破綻**

- **GBP-L**: 「New plan published」の stamp は **34.882 のまま一度も更新されず**、
  毎回「partially valid and closer to the goal」= **ゴールまでの完全なプランを
  一度も出せない**。「Planner was unable to make any progress, start state
  likely trapped」も出る。→ RRT-Connect が `max_planning_time`(18 s に
  延長しても)内で穴を跨ぐ leap 枝を張れていない。
- **下流(foot plan / MPC / WBC)**: 上流から届くのは x≈0.55 までの部分プラン
  だけ。ロボットはそこまで**普通に歩き**(cost・iter・age 正常、GRF 70〜80 N、
  effort 警告 0)、プラン終端(穴の手前)で止まる。
- **転倒の直接原因**: プラン終端で前進指令が尽き、穴の縁で静止 → バランスを
  失って 1 本目の穴へ転落。NMPC fail(580 回)と effort 警告(40 回)は
  **転落後**に出る従属現象。
- **結論**: この工程図では **foot plan の一段目(= 大域プラン)** が
  ボトルネック。MPC/WBC は健全な参照を与えられていない。

### シナリオ 2:穴 2 本(goal x=4.0、run7 / および goal x=2.0 の run8)── **MPC(NMPC)で破綻、WBC が増幅**

- **GBP-L**: プランは出る(積極的な複合 leap)。stamp は 1 回だけ更新。
  `replanning: true` でも**空中に出た後は再計画されず**、局所側は固定参照を
  握り続ける(`plan_age_s` が後で 1〜2 s まで伸びる)。
- **foot plan / 接触**: LEAP_STANCE → FLIGHT → LAND_STANCE を素直に展開。
  ただし LAND を **降下速度 vz ≈ −2 m/s の瞬間に TTTT 固定**で当てる
  (緩衝なし)。
- **MPC(NMPC)── ここが最初に崩れる**:
  - 離陸前、胴体がまだ水平で接地しているのに
    **`plan_nmpc_cost` が 0.01 → 0.99 → 6.6 と急上昇**(run7/run8 とも
    t≈17.5〜18.0)。= 与えられた参照(GBP-L の body plan + 目前の leap
    foot schedule)を、**go2 では高コストでしか追従できない**。
  - 着地衝撃の後、**NMPC が収束しなくなる**:
    `iter` 1 → 14 → 46 → 53、`compute_time` 6 ms → 38 → 90 → **116 ms**
    (replan 周期 ≈ 30 ms を大きく超過)→ `plan_age_s` 0 → **2.3 s**
    (プランが完全に陳腐化)。`cost` → 130 → **1100+**。
- **WBC(逆動力学)── 増幅役**:
  - 崩れた/陳腐化した NMPC の GRF を受けて、GRF を **±150 N 上限に貼り付け、
    (10, 150, 10, 150) のバンバン制御**に。
  - robot_driver の「total effort exceeds threshold(33.5 / 50 Nm)」警告が
    **run7 で 2158 回、run8 で 212 回**。WBC は**誤差の発生源ではなく、
    NMPC の破綻をそのままトルクに変換して増幅している**。
- **トルク → シム**: 飽和したバンバントルク → roll が発散(+0.4 → +1 →
  +2 → π)→ 背面着地、x≈3.5 で反転して停止。
- **結論**: 破綻の起点は **MPC(NMPC)**。原因は上流(GBP-L の leap 参照が
  go2 には過大 + 空中で再計画しない)だが、**精度が最初に落ちて不可逆に
  なるのは NMPC の工程**。WBC はそれを飽和トルクへ忠実に写すだけ。

### シナリオ 3:穴 1 本・成功時(goal x=2.0、run5)── 全工程が辛うじて健全、ただし foot plan/MPC の横方向が甘い

- 発進〜穴手前(t≈15.4〜17.2):**cost 0.004〜0.27、iter 1、compute 6 ms、
  age 0** = 完全に健全。
- 渡り(t≈17.5〜):cost が一時 6 まで上がる(シナリオ 2 と同じ入口)が、
  この試行では NMPC が収束を保ち、胴体 z・roll・pitch は制御下。
- ただし **y が一時 −1.9 m まで振れて**からゴール (2.0, 0) に戻る。
  = foot plan(Raibert + スナップ)と NMPC の**横方向トラッキングが
  大きくオーバーシュート**。「横位置中央を保って渡る」要件は未達。
- run8(同一条件)は同じ入口(cost → 6)からシナリオ 2 と同じ発散をたどって
  転倒 → **成功は 2 回に 1 回。NMPC がこの参照で収束できるかどうかが
  紙一重**。

### まとめ

| シナリオ | 精度が最初に落ちる工程 | 下流の効果 |
|---|---|---|
| 穴 3 本(run6) | **大域プラン(GBP-L RRT)** ── 完全なプランを出せない | foot plan/MPC/WBC は健全な参照を与えられず、プラン終端で失速・転落 |
| 穴 2 本(run7 / run8) | **MPC(NMPC)** ── leap 参照が go2 に過大で cost 発散、着地後は非収束(compute 116 ms ≫ 予算、age 2.3 s) | **WBC** が飽和 GRF/トルク(effort 警告 200〜2000 回)へ増幅、roll→π で反転 |
| 穴 1 本・成功時(run5) | **foot plan / MPC の横方向** ── y が ±1.9 m オーバーシュート(姿勢は保持) | 復帰してゴールに静止。ただし同条件の run8 は同じ入口から発散・転倒(成功 1/2) |

**共通の根**: センシングは常に健全。**WBC は誤差の発生源ではなく増幅器**。
上流の **GBP-L(大域プラン)がボトルネック** ── 解けない(3 本)か、
go2 の NMPC+WBC が実現できない過大な leap 参照を出す(1〜2 本)。
`global_body_planner.yaml` の leap パラメータ(`dz0 1–2 m/s`, `t_s`,
`grf_max 5` bw)は spirit 前提で、go2 向けの再チューニング手順は
マニュアルに無い(3 節)。

## 6. 参考(Quad-SDK 公式)

- [Wiki: 5. Using the Software (ROS2)](https://github.com/robomechanics/quad-sdk/wiki/5.-Using-the-Software-(ROS2))
- [Wiki: 2. Using the Software](https://github.com/robomechanics/quad-sdk/wiki/2.-Using-the-Software)
- [Wiki: Tutorial: Creating Custom Terrain Map Files](https://github.com/robomechanics/quad-sdk/wiki/Tutorial:-Creating-Custom-Terrain-Map-Files)
- [Wiki: Tutorial: Adding a New Type of Robot to Quad SDK](https://github.com/robomechanics/quad-sdk/wiki/Tutorial:-Adding-a-New-Type-of-Robot-to-Quad-SDK)
- [doxygen: GBPL Class Reference](https://robomechanics.github.io/quad-sdk/classGBPL.html)
- [doxygen: LocalFootstepPlanner Class Reference](https://robomechanics.github.io/quad-sdk/classLocalFootstepPlanner.html)
- [GitHub: robomechanics/quad-sdk](https://github.com/robomechanics/quad-sdk)

## 7. 関連(本リポジトリ)

- `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md`(main、twist + クロール、成功・姿勢きれい)
- `scripts/trial/run_quadsdk_gap_gbpl.sh`
- `agent_reports/quadsdk_step01_gait_and_mpc.md`
