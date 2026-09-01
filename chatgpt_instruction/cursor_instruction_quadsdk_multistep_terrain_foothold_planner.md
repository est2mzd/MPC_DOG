# Quad-SDK 複数歩先 Terrain-aware Foothold Planner 実装指示書

## 0. この指示書の目的

Quad-SDK + Go2 + MuJoCo の既存歩行を壊さずに、視野範囲内の地形から、

- この先を歩けるか
- どの脚を、どの順序で、どこへ置けば歩けるか
- 何歩目で足場列が成立しなくなるか
- 危険地点の何歩手前で減速・停止すべきか

を判定する機能を、段階的に追加する。

最初から既存の足場計画やNMPCを置換しない。新機能は既定OFFの並列経路として追加し、`shadow mode`で計算結果を記録してから制御へ接続する。

各Stepは独立してレビュー・回帰・revertできる単位にする。実験したStepでは必ずMarkdown、CSV、代表GIFを作成し、代表的な成功GIFと失敗GIFをREADMEへ掲載する。

---

## 1. 背景

### 1.1 現在できていること

現行のQuad-SDK twist経路は、おおむね次の処理を行う。

1. 上位から`cmd_vel`を受ける。
2. 固定された`period / duty_cycles / phase_offsets`から接触スケジュールを作る。
3. Raibert則で各脚の名目着地点を作る。
4. `getNearestValidFootholdResult()`が、名目着地点の周囲を`foothold_search_radius`以内で探索し、`traversability > foothold_obj_threshold`のセルへ足場をスナップする。
5. 接触スケジュール、胴体参照、足場列をNMPCへ渡す。
6. NMPCのGRFと遊脚軌道を下位制御が関節トルクへ変換する。

Phase 2A〜4として、無効足場の状態化、graceful stop latch、前方lookahead、穴縁判定、IK距離判定が追加されている。ただし、これらは主に「一つの足場の局所修正」または「前方に通過不能な穴があるか」の判定であり、視野範囲全体について複数歩の足場列を作ってはいない。

### 1.2 現在不足していること

現在は以下が分離している。

- 広い範囲を見る処理：`safe_stop_lookahead`で危険の有無を調べる。
- 一歩を決める処理：名目足場周囲を`foothold_search_radius`で探索する。

この間に必要な、次の処理が存在しない。

> 視野範囲内にある安全領域を使い、現在の脚状態と歩容から、FL/FR/RL/RRの複数歩分の実現可能な足場列を作る。

そのため、向こう岸が見えていても、実際に脚が届くか、支持状態を維持できるか、何歩で到達できるかを確認せずに「通過可能」と判定し得る。

### 1.3 距離の役割を混同しない

最低でも次の距離を別々に管理する。

| 名称 | 意味 | 現行の参考値 |
|---|---|---:|
| `map_reliable_range` | センサまたはMapを信頼して使える距離 | 実測して決定 |
| `terrain_planning_distance` | 複数歩足場列を探索する前方距離 | 初期値2.5 m候補 |
| `foothold_search_radius` | 1個の名目足場を局所修正する半径 | 0.7 m |
| `ik_max_reach` | midstance hipから足先までの許容距離 | 0.45 m |
| `nmpc_horizon_distance` | NMPC時間ホライズン中に進む距離 | `velocity × timestep × horizon_length` |
| `stopping_distance` | 現在速度から安全に停止するための距離 | 実測＋式で算出 |

`foothold_search_radius`をLiDAR/カメラ視野の代わりに使ってはいけない。また、遠方まで見えるからといって、一歩の探索半径を広げてはいけない。

### 1.4 歩容情報の優先順位

複数歩計画には、次の優先順位で歩容・状態を使う。

1. **現在の実状態**：現在の接触脚、遊脚、実足位置、胴体姿勢・速度、現在位相。
2. **現在選択中の歩容**：`period / duty_cycles / phase_offsets`から得られる未来の離着地順序。
3. **過去の実績**：着地誤差、スナップ誤差、横ドリフト、停止減速度などの安全余裕推定。
4. **平均的な歩容**：実状態または履歴が取得できない場合だけ使うフォールバック。

過去平均だけで将来の脚順序を決めてはいけない。実際の位相がずれていると、最初に動かす脚を誤るためである。

---

## 2. 最終的に作る機能

### 2.1 入力

| 入力 | 主な内容 | 単位・形式 |
|---|---|---|
| Terrain Map | `z`、加工済み高さ、危険度、観測状態 | grid、m、0〜1、enum |
| Robot state | 胴体位置・姿勢・速度 | m、rad、m/s、rad/s |
| Foot state | 各脚の実足位置、接触状態、遊脚状態 | FL/FR/RL/RR、m、bool |
| Gait state | 現在位相、周期、duty、phase offset | s、0〜1 |
| User command | 目標並進速度・yaw速度 | m/s、rad/s |
| Capability | IK範囲、縁余裕、最大step、停止性能 | m、m/s² |

### 2.2 出力

| 出力 | 内容 |
|---|---|
| `terrain_feasible` | 視野範囲内で足場列が成立するか |
| `max_feasible_progress` | 現在位置から何m先まで足場列が成立したか |
| `first_blocked_step` | 最初に成立しない未来着地番号 |
| `first_blocked_leg` | その着地を担当する脚 |
| `planned_footholds[]` | 脚、着地時刻、x/y/z、margin、reach、status |
| `required_stop_steps` | 危険地点までに確保すべき停止歩数 |
| `safe_velocity_ref` | 上位指令を制限した速度 |
| `behavior_mode` | PASS / SLOW / STOP_REQUEST / STOPPED / UNKNOWN |
| `reason` | NO_SAFE_CELL / IK / EDGE / UNKNOWN / SUPPORT / MAP_END等 |

### 2.3 基本判定

各未来着地について、少なくとも次を満たす候補だけを残す。

1. Map内である。
2. 観測済みセルである。`unknown`を自動的にsafeと扱わない。
3. 足裏を含む領域が安全である。
4. 穴・段差縁から必要距離以上離れている。
5. 予測midstance hipからの距離が`ik_max_reach`以下である。
6. 前後左右の到達範囲内である。球距離だけで完結させない。
7. その足を動かしている間の支持脚集合が成立する。
8. 前回の同脚足場からのstep長が許容範囲内である。
9. 選択した足場をNMPCが追従できる見込みがある。

初期版では7と9を簡易近似してよいが、未実装の判定を「確認済み」と記載しない。

### 2.4 停止歩数

ユーザー設定`stop_margin_steps = M`を持たせる。ただし固定Mだけで止めず、現在速度から必要停止距離も計算する。

初期式：

```text
d_stop = v_forward^2 / (2 * a_safe) + v_forward * t_delay + distance_margin
```

ここで、`a_safe`はシミュレーションの減速試験から得る保守的な減速度、`t_delay`はMap更新・計画・ラッチ・歩容遷移の合計遅れである。

歩容から未来着地点間の前進量を推定し、`d_stop`を歩数へ変換する。

```text
required_stop_steps = ceil(d_stop / conservative_step_progress)
final_stop_steps = max(stop_margin_steps, required_stop_steps)
```

足場列が`k`歩目で破綻し、`k <= final_stop_steps`なら即座に`STOP_REQUEST`。それより先なら`SLOW`として速度を下げ、毎周期再計画する。

---

## 3. 後方互換性の絶対条件

1. 新機能は既定OFFにする。
2. OFF時は既存関数の戻り値、`cmd_vel`、足場、NMPC入力、停止挙動を変更しない。
3. 既存のStep 03/04/05/05b/06/07/08用ファイルを上書きしない。
4. 既存パラメータの既定値を、説明なく変更しない。
5. `reference:=twist`を維持する。GBP-Lへの置換は本タスク外。
6. 新しい足場列を最初からNMPCへ入力しない。必ずshadow modeを先に通す。
7. user changesを破棄しない。`git reset --hard`、`git checkout --`を使わない。
8. 各Step開始時と終了時に、対象repoとsubmoduleのbranch、HEAD、`git status --short`を記録する。

推奨設定形：

```yaml
terrain_foothold_planner:
  enabled: false
  shadow_mode: true
  terrain_planning_distance: 2.5
  stop_margin_steps: 2
  use_current_gait_state: true
  use_history_margin: false
  unknown_is_safe: false
  apply_velocity_limit: false
  apply_planned_footholds: false
```

実際のparameter namespaceとファイル位置は、コードを確認してから決定する。上記名を無確認で貼り付けない。

---

## 4. 実施前にAgentが必ず調査すること

コード変更前に以下を調査し、最初のStepのMarkdownへ記録する。

1. Terrain Map publisherから`local_planner`までのtopic、message型、frame、更新周期。
2. Mapの全layer名、生成順序、単位、NaNの意味。
3. `z`、`z_inpainted`、hole mask、`traversability`の正確な式。
4. `getNearestValidFootholdResult()`の入力、探索順、閾値、戻り値。
5. 現在位相と未来の脚順序を保持する変数・関数。
6. 未来midstance hipとnominal footholdを作る関数。
7. NMPCへ渡るfoot planのshape、脚順、時刻対応。
8. Phase 2Bのstop latchからSTEP→STANDまでの状態遷移。
9. `safe_stop_lookahead`、`safe_stop_horizon`、`max_crossable_gap`の使用箇所。
10. Mapの範囲外、未観測、穴がコード上で区別されているか。

推測でファイル名・関数名・挙動を書かない。必ず`rg`、呼び出し元、実行ログで確認する。

---

## 5. 段階的な実装Step

## Step 09：Mapと足場判断の定量計測（制御変更なし）

### 目的

15/30/35/50/100 cmの穴について、どのセルが踏める・踏めないになり、どの足場が選ばれたかを確定する。

### 実施内容

- Map中央断面をCSV化する。
- 各未来touchdownについて、名目足場、選択足場、脚、時刻、状態を記録する。
- 制御挙動は一切変更しない。

最低限のCSV列：

```text
time, map_stamp, frame, cell_i, cell_j, x, y,
z_raw, z_inpainted, z_smooth,
hole_mask, hole_mask_filtered, traversability,
observed, inside_map, binary_safe,
leg, touchdown_index, touchdown_time,
nominal_x, nominal_y, selected_x, selected_y, selected_z,
snap_distance, hip_distance, foothold_status
```

### 完了条件

- 50 cm穴について「穴セルをsafeと誤認した」のか「穴セルはunsafeだが向こう岸へスナップした」のかを数値で確定する。
- 既存の18シナリオで制御結果が変更されていない。

### 成果物

- `agent_reports/steps/step_09_terrain_grid_and_foothold_measurement.md`
- CSVと断面図。
- 15/30/50/100 cmの代表GIF。
- READMEに成功例30 cmと失敗例50 cmを掲載。

---

## Step 10：現在歩容から未来の脚順序を再構成（shadow mode）

### 目的

現在の実位相から、視野範囲内で将来どの脚がどの順番で着地するかを再構成する。

### 実施内容

- 現在の接触状態とgait phaseを取得する。
- `period / duty_cycles / phase_offsets`から未来touchdown event列を生成する。
- 既存`computeContactSchedule()`または相当処理との一致を確認する。
- 平均歩容で位相を0から作り直さず、現在位相から開始する。
- 既存制御には入力せず、shadow出力だけを記録する。

### 完了条件

- 各脚の予測touchdown時刻と実際の接触遷移の誤差を記録できる。
- 平地、30 cm穴、連続15 cm穴で脚順序が一致する。

### 成果物

- `agent_reports/steps/step_10_future_gait_event_prediction.md`
- 予測接触と実接触を重ねたCSV/図。
- 脚名・予測着地を画面に重ねた代表GIF。
- READMEに最も分かりやすい成功GIFを掲載。

---

## Step 11：1歩の可到達領域と安全足場候補生成（shadow mode）

### 目的

単純な半径0.7 m探索ではなく、各脚について実際に届く安全セルを列挙する。

### 実施内容

- 予測midstance hipを基準に候補セルを評価する。
- 最低限、`ik_max_reach`、前後左右範囲、縁距離、足裏半径、Map状態を判定する。
- 既存の`getNearestValidFootholdResult()`と新候補生成器の結果を並べて記録する。
- Phase 4と判定式を重複実装せず、可能なら共通関数化する。ただしOFF時の挙動を変えない。

### 完了条件

- 30 cm穴では有効候補が残る。
- 50/100 cm穴で、届かない向こう岸候補を`IK_UNREACHABLE`として除外できる。
- 平地で既存名目足場付近の候補が残る。

### 成果物

- `agent_reports/steps/step_11_reachable_safe_foothold_candidates.md`
- 各脚のreachable regionと候補セルを重ねた図・GIF。
- 成功例と代表的な候補なし例をREADMEへ掲載。

---

## Step 12：複数歩足場列の探索（shadow mode）

### 目的

視野範囲内について、未来の脚順序に沿った足場列が成立するか判定する。

### 初期アルゴリズム

最初は複雑な最適化を使わず、beam searchまたは幅制限付きグラフ探索でよい。

各node：

```text
body prediction
four foot positions
contact/support state
next touchdown leg
time/index
accumulated cost
minimum safety margin
```

各edge：次の脚を候補セルへ移す操作。

初期cost候補：

```text
distance_from_nominal
+ previous_foothold_change
+ edge_risk
+ reach_margin_penalty
+ lateral_deviation
+ low_map_confidence_penalty
```

### 判定結果

- `FEASIBLE_TO_RANGE`
- `BLOCKED_AT_STEP_K`
- `UNKNOWN_BEFORE_RANGE`
- `SEARCH_TIMEOUT`

`SEARCH_TIMEOUT`を`FEASIBLE`として扱わない。

### 完了条件

- 平地：視野末端まで成立。
- 15 cm連続穴：既存成功範囲で成立。
- 30 cm穴：成立。
- 50/100 cm穴：成立しない、または能力外として分類。
- 計算時間を記録し、local planner周期内に収める必要があるか、別周期で動かすかを判断する。

### 成果物

- `agent_reports/steps/step_12_multistep_foothold_sequence_shadow.md`
- 予定足場列を脚別色で重ねたGIF。
- 30 cm成功列、50 cmブロック列をREADMEへ掲載。

---

## Step 13：停止余裕M歩の推定とshadow判定

### 目的

足場列が成立しなくなる地点のM歩手前で安全に停止要求を出す。

### 実施内容

1. 平地で速度0.15/0.30/0.50 m/sから停止試験を行う。
2. stop latch発火から速度5%以下、全脚接地、STAND安定までの時間と距離を測る。
3. 保守的な`a_safe`と`t_delay`を算出する。
4. 必要停止距離を未来着地event数へ変換する。
5. `stop_margin_steps`との大きい方を採用する。
6. まだ制御へ反映せず、実際に止めていたらどこで止まったかをshadow表示する。

### 完了条件

- 速度が上がるほど必要停止距離・歩数が増える。
- 50/100 cm穴で、物理縁より十分手前にSTOP_REQUEST予定点が出る。
- 30 cm穴では不要なSTOP_REQUESTを出さない。

### 成果物

- `agent_reports/steps/step_13_step_margin_and_stopping_distance.md`
- 速度別停止距離表、停止位置を表示したGIF。
- 最短停止成功と停止余裕不足の代表GIFをREADMEへ掲載。

---

## Step 14：速度制限・graceful stopへの接続（opt-in）

### 目的

shadow判定を既存Phase 2Bのgraceful stopへ接続する。最初は足場列をNMPCへ変更せず、速度制限と停止だけを反映する。

### 実施内容

- `apply_velocity_limit:false`、`apply_stop_request:false`を既定とする。
- opt-in時だけ`PASS/SLOW/STOP_REQUEST`を`cmd_vel`制限と既存stop latchへ接続する。
- local planのpublishを突然止めない。
- 遊脚を着地させ、減速し、全脚接地後にSTANDへ遷移する既存経路を使う。
- latch理由とfirst blocked step/legをログへ出す。

### 完了条件

- 50/100 cm穴：M歩以上手前で直立停止、3回中3回転倒なし。
- 15/30/35 cm：既存成功シナリオを不要停止せず通過。
- feature OFF：Step 08の既存結果と一致。

### 成果物

- `agent_reports/steps/step_14_multistep_planner_safe_stop_integration.md`
- 成功・失敗・回帰結果CSV。
- 30 cm通過、50 cm停止、100 cm停止のGIFをREADMEへ掲載。

---

## Step 15：計画足場列を既存Local Planner/NMPCへ接続（opt-in）

### 着手条件

Step 14までの判定と停止が安定してから着手する。先に実装しない。

### 目的

複数歩計画で得た足場列の直近部分を、既存Local Footstep PlannerとNMPCへ渡す。

### 実施内容

- 全視野分を一度に固定せず、receding horizonで毎周期更新する。
- 直近のtouchdownだけ、またはNMPCホライズン内だけを適用する。
- 既存スナップ処理は最終的な局所微修正として残す。
- Map更新、状態ずれ、着地誤差があれば再計画する。
- 計画が消えた場合は名目足場へ黙って戻さず、SLOWまたはSTOP_REQUESTへ遷移する。

### 完了条件

- 15 cm連続穴と30 cm穴で、計画足場と実着地の対応がログで追える。
- 足場追従誤差、NMPC cost、iteration、compute time、plan ageが許容範囲。
- 50/100 cm穴では無理な足場をNMPCへ渡さない。
- feature OFFの回帰が完全に維持される。

### 成果物

- `agent_reports/steps/step_15_multistep_foothold_nmpc_integration.md`
- 計画足場・実足場・誤差を表示したGIF。
- 代表成功GIFと代表失敗GIFをREADMEへ掲載。

---

## Step 16：全回帰と限界Map作成

### 目的

穴幅、平地幅、穴数、速度、歩容を掃引し、「通過・減速・停止・失敗」の境界を定量化する。

### 最低限の掃引

- 穴幅：15/25/30/35/40/50/75/100 cm。
- 平地幅：15/25/35/50 cm。
- 穴数：N=1〜6。
- 速度：0.15/0.30/0.50 m/s。
- 歩容：現在採用中のクロール、元のトロット。可能なら現在位相を変えた複数開始条件。
- feature：OFF / shadow / stop-only / foothold-apply。

### 完了条件

- 各条件を最低3回。非決定性が出た条件は追加試行する。
- 危険な穴へ落下しないことを最優先とする。
- 通過可能領域、停止領域、未確認領域を明確に分ける。

### 成果物

- `agent_reports/steps/step_16_multistep_terrain_planner_full_regression.md`
- 全試行CSV、成功率表、計算時間表、限界Map。
- READMEにはGIFを増やしすぎず、次の4本を代表として掲載する。
  1. 平地または既存回帰。
  2. 30 cm通過成功。
  3. 50 cm手前停止成功。
  4. 代表的な未解決失敗。

---

## 6. 各Stepで必須の実験記録

各StepのMarkdownは、最低限次の章を持つ。

```text
# Step XX: タイトル
## 1. 背景
## 2. 目的
## 3. 変更前のコード経路
## 4. 事実 / 推測 / 未確認
## 5. 変更計画
## 6. 変更ファイルと変更理由
## 7. 入出力・単位・座標系
## 8. 実験条件
## 9. 試行結果
## 10. 失敗原因
## 11. 後方互換性確認
## 12. GIF・CSV・ログ
## 13. 次Stepへ進む条件
```

### 1試行ごとの必須記録

```text
trial_id
git commit / submodule commit
feature flags
world / map tag
velocity command
gait parameters and initial phase
map reliable range
planning distance
stop margin steps
predicted feasible distance
first blocked step / leg / reason
selected footholds
actual touchdown positions
stop request time / position
final position / pose
min body z / max roll / max pitch
NMPC cost / iterations / compute time / plan age
PASS / SAFE_STOP / FELL / UNKNOWN
```

### GIF要件

実際にシミュレーションを行った試行群では必ずGIFを作る。GIFには可能な範囲で次を重ねる。

- 時刻、指令速度、実速度。
- 現在の脚状態。
- 名目足場と選択足場。
- 複数歩の予定足場。脚ごとに色を固定する。
- Map上のunsafe/unknown領域。
- first blocked step。
- `PASS / SLOW / STOP_REQUEST / STOPPED`。
- 危険地点までの距離と必要停止歩数。

大量のGIFをREADMEへ貼らない。各Stepの詳細GIFはStep Markdownから参照し、READMEには代表的な成功・失敗だけを掲載する。

---

## 7. 成否判定

### 通過成功

- テスト区間終端を胴体が通過。
- 足が物理穴へ落ちていない。
- `z >= 0.15 m`かつ`|roll|, |pitch| < 0.8 rad`。
- 選択足場がMap、IK、縁距離条件を満たす。
- NMPC計算が実時間予算を継続的に超えない。

### 安全停止成功

- 危険地点より必要停止距離以上手前でSTOP_REQUEST。
- 最終的に全脚接地または既存STAND状態。
- 前進速度が十分小さい状態を3秒以上維持。
- 穴へ落下せず、直立を維持。
- plan publish停止による遊脚凍結を起こさない。

### 失敗

- 穴への落下、転倒。
- 無効・到達不能足場をNMPCへ渡す。
- Map未観測領域をsafeとして通過する。
- SEARCH_TIMEOUTを通過可能として扱う。
- 危険検知後に必要停止距離を確保できない。
- feature OFFで既存挙動が変わる。

---

## 8. Agentの作業ルール

1. Stepを飛ばさない。
2. 各Step開始時に、調査結果と変更計画を先に提示する。
3. 変更ファイル、変更理由、既存挙動への影響、検証方法を表にする。
4. 大きな設計判断が必要な場合は、実装前にユーザーへ確認する。
5. 事実、推測、未確認を明確に分ける。
6. 成功判定を最終xだけで行わない。GIF、足位置、姿勢、Map値、ログを突き合わせる。
7. 失敗を隠さない。再現率と非決定性を記録する。
8. パラメータを一時変更する場合、変更前値を保存し、正常終了・異常終了の両方で復元する。
9. 制御変更と計装変更を同じcommitへ混ぜない。
10. 1回の依頼で複数Stepを一気に実装しない。1 Stepを調査・実装・検証・文書化して停止する。

---

## 9. 最初にAgentへ渡す実行指示

```text
添付の「Quad-SDK 複数歩先 Terrain-aware Foothold Planner 実装指示書」を読んでください。

今回はStep 09だけを実施してください。Step 10以降は実装しないでください。

目的は、15/30/35/50/100 cm穴について、Terrain Mapの各レイヤと足場選択結果をセル単位で記録し、50 cm穴が

A. 穴セル自体をtraversableと誤認している
B. 穴セルはinvalidだが、向こう岸のvalidセルへスナップして通過可能と判定している
C. AとBの両方

のどれかを、推測ではなく数値で確定することです。

作業前に、対象branch、HEAD、git status、Map生成からgetNearestValidFootholdResultまでのコード経路、変更計画を提示してください。計装以外の制御挙動を変更しないでください。

実験後は、指定されたMarkdown、CSV、断面図、代表GIFを作成し、READMEに30 cm成功GIFと50 cm失敗GIFを掲載してください。既存Step 03〜08のファイルと設定を上書きしないでください。
```

