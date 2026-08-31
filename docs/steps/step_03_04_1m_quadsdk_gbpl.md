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
姿勢を保って踏破)が実用解。**このブランチは main へマージしない**
(`external/quad-sdk` への設定変更は一切コミットしていない。追加物は
実験ハーネスと本 doc のみ)。

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
| 5 | flat_gaps_2m | 2.0 | 10.0 | 素トロット | **成功。** 穴 1 本を渡り goal (2.0, 0) で直立静止。ただし途中の横スイング y=−1.9→0 |
| 6 | flat_gaps_2m | 6.0 | 18.0 | 素トロット | **失敗。** 1 本目を跨ぐ leap も張れず、x≈0.55 で停止 → 転落。GBP-L「partially valid」1 回のみ |
| 7 | flat_gaps_2m | 4.0 | 22.0 | 素トロット | **不成立。** 複合 leap で x 0.2→2.9 を一気に跳び穴 2 本を越えるが、**着地で背面へ転倒**(roll→π) |

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

## 5. 参考(Quad-SDK 公式)

- [Wiki: 5. Using the Software (ROS2)](https://github.com/robomechanics/quad-sdk/wiki/5.-Using-the-Software-(ROS2))
- [Wiki: 2. Using the Software](https://github.com/robomechanics/quad-sdk/wiki/2.-Using-the-Software)
- [Wiki: Tutorial: Creating Custom Terrain Map Files](https://github.com/robomechanics/quad-sdk/wiki/Tutorial:-Creating-Custom-Terrain-Map-Files)
- [Wiki: Tutorial: Adding a New Type of Robot to Quad SDK](https://github.com/robomechanics/quad-sdk/wiki/Tutorial:-Adding-a-New-Type-of-Robot-to-Quad-SDK)
- [doxygen: GBPL Class Reference](https://robomechanics.github.io/quad-sdk/classGBPL.html)
- [doxygen: LocalFootstepPlanner Class Reference](https://robomechanics.github.io/quad-sdk/classLocalFootstepPlanner.html)
- [GitHub: robomechanics/quad-sdk](https://github.com/robomechanics/quad-sdk)

## 6. 関連(本リポジトリ)

- `docs/steps/step_03_04_1m_quadsdk_gap_crossing.md`(main、twist + クロール、成功・姿勢きれい)
- `scripts/trial/run_quadsdk_gap_gbpl.sh`
- `agent_reports/quadsdk_step01_gait_and_mpc.md`
