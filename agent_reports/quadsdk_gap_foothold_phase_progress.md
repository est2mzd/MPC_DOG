# Quad-SDK 穴対応 Foot Placement 改善:フェーズ実施ログ

読者は制御の大学院初心者を想定。まず **背景・目的・結論** を書き、
そのあと用語と各コミットの詳細を続ける。

---

## 背景

- MPC_DOG では、四足ロボット **Go2** を Quad-SDK で走らせている。
  以前の作業で、シミュレーション上で **深さ 1 m・幅 0.3 m の溝を、
  足を溝に入れずに複数本連続で渡れる**ようになった
  (`agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md` §5。
  GIF: `artifacts/gifs/quadsdk_step03_1m_v0p15.gif` /
  `artifacts/gifs/quadsdk_step04_1m_v0p3.gif`、いずれも git 追跡対象)。
- ただしそれは「**静的で位置ずれの無い既知の地形**」+「**手作業で付けた
  安全マージン**」に頼った成功であり、実センサ・実機で安全とは言えない。
- そこで指示書
  (`chatgpt_instruction/cursor_instruction_quadsdk_gap_foothold_analysis.md`)
  で、**まずコードを精読して「足の置き場所を決める処理(Foot Placement)と
  MPC の連携」を数式とコードで正確に把握し、そのうえで不足している安全機能を
  小さな変更で順に足す**ことになった。

## 目的

1. **理解**:センサ → 足場計画 → MPC → 逆動力学 → トルク の各段が、
   何を入力に何を出力し、穴に対して誰が責任を持つのかを、
   推測でなくコードの関数・行から確定する。
2. **訂正**:以前の作業メモに残っていた、コードと食い違う記述を直す。
3. **改善**:「有効な足場が無い(`status != FootholdStatus::VALID`)のに名目の足場で歩き続ける」等の
   危険な挙動を、**既存の成功挙動を壊さずに**、1 コミット = 1 目的で
   段階的に潰す。各フェーズは着手前に変更計画を出し、確認を取る。

## 結論(現時点)

- **解析は完了。** 一番大事な確定事項:
  **Go2 の MPC は足場を最適化していない。** 足の置き場所は
  Foot Placement(`local_footstep_planner`)が地図から決め、MPC はその足場を
  **固定パラメータ**として胴体軌道と地面反力だけを最適化する。
  MPC の制約は運動方程式と摩擦だけで、「脚が届くか(逆運動学)」の制約は無い。
- **Phase 0 完了(ドキュメント訂正のみ、コード不変)。** 以前のメモの 3 つの
  誤り(「MPC の脚可到達制約が破れた」/「horizon はギャップ周期より長い必要が
  ある」/「実センサでも穴は自然に欠測値になる」)を、コード事実と推測に
  分けて書き直した。
- **Phase 1 完了(コード変更あり、ただし挙動は不変)。** 足場選択関数を、
  「位置だけ返す」から「**成功/失敗の種類 + 診断値**も返す」型へ拡張した。
  返す位置は従来と 1 バイトも変わらない。付随して、以前から red だった
  無関係なテスト(`horizon_length` 期待値の古い 26)を実値 40 へ揃えた結果、
  **`local_planner` の全 29 テストが green**。
- **Phase 2 は「2A(NMPC へ無効足場を渡さない)」と「2B(遊脚を考慮した停止
  シーケンス設計)」に分割。**
- **Phase 2A 完了(コード変更あり)。** `computeFootPlan()` が
  `FootPlanResult{ok,...}` を返し、無効足場が 1 つでもあれば
  `computeLocalPlan()` が `stop_on_invalid_foothold`(既定 ON)のとき
  local plan を publish しない → 既存の 0.1 s タイムアウト経由で
  `robot_driver` が起立姿勢へ PD ホールド。**`status==VALID` の経路では
  挙動は不変**。`local_planner` テスト **31/31 green**(既存 + 新規 3 本)。
  詳細は下記「Phase 2A — 実装完了」。
- **Phase 2B〜6 は未着手。** 次は Phase 2A のシミュレーション検証
  (幅 10 m の穴の手前で 3 秒停止できるか)。

---

## 用語(最低限)

- **Foot Placement / 足場計画** … 遊脚(いま空中にある脚)を次にどこへ着地
  させるかを決める処理。Quad-SDK では `local_footstep_planner`。
  古典的な **Raibert 則**(速度差から着地点を幾何的に決める式)で仮の
  着地点を出し、地図を見て安全な場所へ「スナップ」する。
- **terrain map / 地形マップ** … 周囲の地面を格子で持ったもの。
  各セルに高さ `z`、傾き `slope`、通行可能度 `traversability`(0〜1、
  1=平ら安全、0/欠測=崖・穴)などの層がある。
- **`getNearestValidFoothold()`** … 仮の着地点のまわりを渦巻き状に探索し、
  `traversability` が閾値(0.6)を超える一番近いセルへ着地点を移す関数。
  今回いじっているのはここ。
- **NMPC(非線形モデル予測制御)** … 未来数十ステップ分の胴体の動きと
  各接地脚の地面反力(GRF)を、運動方程式・摩擦・入力上下限を満たしながら
  最適化する。Quad-SDK の `nmpc_controller`。
- **決定変数 / パラメータ** … 最適化が「動かせる」量が決定変数、
  「固定で外から与える」量がパラメータ。**足場は Go2 では決定変数ではなく
  パラメータ**、というのが本作業の核心。
- **逆動力学(inverse dynamics)/ WBC 相当** … MPC が決めた GRF と足先軌道を
  実際の関節トルクへ変換する下流の段(`robot_driver`)。
- **診断値のみ(diagnostics only)** … コードは足すが、**ロボットの動きを
  変える判断には一切使わない**変更。ログや戻り値の新フィールドだけ。
  「悪さをしていない」ことを既存テストで示せる。
- **1 コミット = 1 目的** … 変更の原因と効果を後から追えるように、
  1 回のコミットで 1 つのことだけ変える。

---

## なぜ「段階的」にやるのか(初心者向け)

穴渡りが失敗するとき、原因は「地図が悪い」「足場計画が悪い」「MPC が悪い」
「逆動力学が悪い」のどれか、あるいは複合。いきなり大きく変えると、
**直ったのか・別の所を壊したのか**が分からなくなる。

そこで:

1. まず **観測できるようにする**(Phase 1)。足場選択が「見つかった/
   見つからなかった/地図の外/高さが変」のどれで終わったか、
   どれだけ着地点をずらしたか、を戻り値に持たせる。**動きは変えない。**
2. 次に **その観測をもとに安全側へ倒す**(Phase 2)。有効足場が無ければ
   減速・停止する。ここで初めて挙動が変わる。
3. 以降、縁の余裕(Phase 3)、逆運動学の可到達性(Phase 4)、大補正時の
   減速(Phase 5)、地図の鮮度(Phase 6)を 1 つずつ。

---

## フェーズ表

| Phase | 目的 | 状態 | コミット |
|---|---|---|---|
| 解析 | 資料 ⇔ コード照合、terrain map / foot placement / NMPC を数式とコードで整理 | ✅ | `3a6c705` `c9cf853` |
| 0 | 解析で判明した資料の 3 誤りを訂正。**コード変更なし** | ✅ | `6e089e1` |
| 1 | 足場選択器を「位置だけ」→「成功/失敗 + 診断値」を返す型へ。**挙動不変** | ✅ | `484ea13` `88605aa` `6282643` |
| 2A | NMPC へ無効足場(穴上・地図外・高さ非有限)を渡さない | ⬜ 変更計画提示済み・未実装 | ― |
| 2B | 遊脚を考慮した安全な減速・停止シーケンスを設計 | ⬜ 2A 完了後 | ― |
| 3 | 穴縁からの安全距離を地図上で明示判定 | ⬜ | ― |
| 4 | 逆運動学の可到達性で候補を絞る | ⬜ | ― |
| 5 | 大きな足場補正時の減速/刻み歩行 | ⬜ | ― |
| 6 | 地図の鮮度・未観測セルの扱い | ⬜ | ― |

---

## 各コミットの詳細

### `3a6c705` — 解析レポート新規作成(コード変更なし)

`agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md` を作成。
確定した主なこと:

- `reference:=twist` でも terrain map による足場補正は動く。
  Global Body Planner だけが `reference:=gbpl` 限定。
- gait は地形で自動変更されない(`period`/`duty_cycles`/`phase_offsets` は
  起動時 1 回だけ読む固定表)。
- 穴対応の実効メカニズム:メッシュに実穴 → 生 `z`=欠測(NaN) →
  補間した `z_inpainted` と食い違う → `traversability` が穴帯で NaN →
  `getNearestValidFoothold` が却下。段差/スロープでは発火しない。
- 地形表現は 2 系統:足場選択用(穴=NaN が残る)と、胴体の高さ・傾き参照用
  (穴を埋めた版)。凸条を水平・同一高さに保てば偽のピッチ指令が出ない。
- **Go2 の NMPC は 12 状態の簡易モデルのみ。足場はパラメータ**
  (GRF のモーメントアームとして運動方程式に入る)。
- **NMPC の制約は 運動方程式 + 摩擦ピラミッドのみ。** 関節角・逆運動学
  可到達性の制約は無い。地面反力の上下限 `f_z ∈ [10, 150] N`、
  実効摩擦係数 μ = 0.6(`go2.yaml` が `nmpc_controller.yaml` の 0.3 を
  launch 順で上書き)。
- **有効足場が無いと現コードは名目足場をそのまま返す**(警告はマクロの
  引数バグで実質出ない)→ 成功/失敗が下流へ伝わらない(← Phase 1/2 の的)。
- 下流の安全側フォールバック:local plan が 0.1 s 以上古い/時刻窓外なら
  逆動力学コントローラが false を返し、`robot_driver` が起立姿勢へ PD ホールド。

### `c9cf853` — 解析レポートにレビュー指摘 7 点を反映(コード変更なし)

| # | 訂正 |
|---|---|
| 1 | `foot_positions_body_` は body フレームではない(`foot_world − body_pos`、姿勢回転なし) |
| 2 | Raibert 名目の速度は「並進=world 系 / 角速度=body 系」。RobotState 側の並進速度フレームは未確認 |
| 3 | 実効摩擦係数 μ は **0.6**(launch 順で go2.yaml が上書き)。要ライブ `ros2 param get` |
| 4 | 「NMPC 失敗後に壊れた GRF がそのままトルク化」は誤り。(a) 失敗継続→起立ホールド と (b) 求解成功だが低品質解→飽和 GRF 実行 を分離(転倒は (b)) |
| 5 | 地図外は `getNearestValidFoothold` の手前で `continue` される。現フローでは `getNearestValidFoothold` に到達しないため `FootholdResult.status` に `NOMINAL_OUTSIDE_MAP` が入らない(Phase 2A で `continue` 地点にフックを足す) |
| 6 | 「遠い足場 → NMPC cost 増大 → 非収束」の因果は**推測**へ格下げ(cost 内訳・slack・制約違反・IPOPT status を記録するまで) |
| 7 | `filter_chain.yaml` の `traversability_mask` 閾値 0.5 と `foothold_obj_threshold` 0.6 の不一致。足場選択は `traversability` レイヤを読むため現挙動への直接影響なし |

### `6e089e1` — Phase 0:資料の 3 誤りを訂正(ドキュメントのみ)

`agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md`:

- **§3.4「NMPC 内の脚可到達制約が破れた」** → 削除。
  「go2 簡易 NMPC の制約は 運動方程式 + 摩擦錐のみで、脚可到達制約は不在」を
  **コード事実**、モーメントアーム変化→GRF 実現困難→スラック増大 の筋を
  **推測(要ログ)**に分離。
- **§3.4「`horizon_length > period_` が必須」** → 「26→40 で追従改善」は
  実験事実、「剰余ラップなのでコード上の必須条件ではない」を分けて記述。
- **§2「実センサでも穴は自然に NaN」** → **未確認**へ格下げ
  (この repo に LiDAR/深度 → 地図 `z` の処理は無い)。

### `484ea13` — Phase 1:`FootholdResult` / `FootholdStatus`(コード、挙動不変)

**変更ファイル**

| ファイル | 変更 |
|---|---|
| `local_footstep_planner.hpp` | `enum class FootholdStatus` + `struct FootholdResult` + 新関数 `getNearestValidFootholdResult()` 宣言 |
| `local_footstep_planner.cpp` | `getNearestValidFoothold()` を薄いラッパ化。探索本体を新関数へ移し status を設定。DIAG ログに status/snap 追記 |
| `test/test_footstep_planner.cpp` | 合成地形ヘルパ 2 個 + テスト 5 本 |

**追加した型**(Phase 1 はこの 4 値だけ。`EDGE_TOO_CLOSE` / `IK_UNREACHABLE` /
`MAP_STALE` はそれを計算するコードと一緒に後の Phase で追加する)

```cpp
enum class FootholdStatus {
  VALID,                     // 通行可能な候補を選べた
  NOMINAL_OUTSIDE_MAP,       // 仮の着地点が地図の外
  NO_TRAVERSABLE_CANDIDATE,  // 探索半径内に閾値を超えるセルが無い
  NONFINITE_HEIGHT,          // 採用セルの補間高さが有限でない
};

struct FootholdResult {
  Eigen::Vector3d position;        // 選ばれた足場(world)。従来の戻り値と同一
  FootholdStatus  status;
  double traversability_nominal;   // 仮の着地点の通行可能度
  double traversability_selected;  // 採用セルの通行可能度
  double snap_distance;            // ||selected.xy − nominal.xy||(どれだけずらしたか)
};
```

`edge_clearance` / `reachable` は**入れていない**(ユーザー指示。原因と効果を
追いやすくするため、それらは計算コードと同時に導入する)。

**挙動が変わっていないこと**

- `getNearestValidFoothold()` は `getNearestValidFootholdResult(...).position` を
  返すだけ。唯一の呼び出し(`computeFootPlan`)は行を変えていない。
- `NO_TRAVERSABLE_CANDIDATE`:従来どおり「仮の着地点の x/y + 補間高さ」を返す。
- `NONFINITE_HEIGHT`:従来どおり NaN 高さを含む位置をそのまま返す(ラベルだけ付ける)。
- `NOMINAL_OUTSIDE_MAP`:防御的な早期 return。**現フローでは `computeFootPlan`
  が手前で `continue` するので到達しない**。Phase 2 で地図外処理を移すときに生きる。

**検証**(`colcon test --packages-select local_planner`)

- 新規 5 本すべて pass:
  `FootholdResultReportsValidOnFlatTerrain` /
  `...ReportsNoTraversableCandidate` / `...ReportsNominalOutsideMap` /
  `...ReportsNonfiniteHeight` / `...SnapDistanceMatchesSelection`
- **既存の挙動不変テスト 2 本**
  (`FootholdSearchUsesValidTerrainAndToeRadius`、
  `FootholdSearchFallsBackWhenTerrainInvalid`)も pass
  → 足場選択の結果は変わっていない。
- `LocalFootstepPlannerTest` **17/17 pass**。

### `88605aa` — ドキュメント語の統一(コードなし)

`found==false`(bool 1 個の言い方)を **`status != FootholdStatus::VALID`** に
統一。理由:Phase 1 で「失敗」は `NOMINAL_OUTSIDE_MAP` / `NO_TRAVERSABLE_CANDIDATE` /
`NONFINITE_HEIGHT` の 3 種になったので、下流へ伝えるべき条件は「`VALID` でない」。
単体試験表には「広い穴 → `NO_TRAVERSABLE_CANDIDATE`」「地図外 → `NOMINAL_OUTSIDE_MAP`」
と具体名を書いた。

### `6282643` — 先行バグの修正(テストのみ、制御コードなし)

`test_local_planner.cpp:175` の `EXPECT_EQ(planner.N_, 26)` を **40** へ。
`local_planner.yaml` の `horizon_length` は `9ccd639`(クロール歩容)以来 40 で、
このアサートは古い値を追っていただけ。Phase 1 とは無関係。

結果:`colcon test --packages-select local_planner` →
`Summary: 29 tests, 0 errors, 0 failures`。
`LocalFootstepPlannerTest` 17 + `LocalPlannerTest` 12 = **29/29 green**
(以前 red だった `ConstructorLoadsYamlConfigurationAndInterfaces` も green)。

### `45720a5` `ea14d7b` — 本フェーズ実施ログの作成・書き直し + README リンク

### Phase 2A — 実装完了(コミット 2 本:制御コード + テスト)

**目的:穴上・地図外・高さ非有限の足場を NMPC へ絶対に渡さない。**
STAND 遷移・`cmd_vel`→0・Map 期限切れ・edge clearance・IK は **入れていない**
(それぞれ Phase 2B / 3 / 4)。停止自体は既存の
「local plan 0.1 s タイムアウト → `robot_driver` が起立姿勢へ PD ホールド」に委ねる。

**ユーザーからの回答で確定した設計判断(2026-08-31)**:

- 戻り値は **`struct FootPlanResult{ok, worst_status, failed_leg,
  failed_touchdown_index, failed_count}`**(bare `bool` ではなく診断つき)。
- **`stop_on_invalid_foothold` パラメータ(既定 `true`)** で切れる。
  `false` で Phase 2A 前の挙動(最後の有効足場を保持して計画続行)へ戻る。
- 失敗記録は **全件カウント(`failed_count`)+ 最初の失敗の詳細**。

**実装内容**:

| # | 変更ファイル | 実装 |
|---|---|---|
| 2A-1 | `local_footstep_planner.hpp` / `.cpp` | `computeFootPlan()` 戻り値 `void` → `FootPlanResult`。touchdown ループにラムダ `record_foothold_failure(status, leg, idx)` を追加(最初の失敗を保持 + `failed_count++`)。**関数末尾に `return plan_result;` を追加**(下記「ハマった点」) |
| 2A-2 | `local_footstep_planner.cpp` 地図外 `continue`(現 `:271-281`) | 裸の `continue` の直前に `record_foothold_failure(NOMINAL_OUTSIDE_MAP, j, i)`。`continue` 自体は不変 |
| 2A-3 | `local_footstep_planner.cpp` foothold 書き込み(現 `:291-306`) | `getNearestValidFoothold()` → `getNearestValidFootholdResult()` へ。`status == VALID` なら従来どおり `foothold.position` を書く。`status != VALID` は **穴上の名目/NaN を書かず `getFootData(foot_positions, i-1, j)`(直前値)を踏襲** + 失敗記録 |
| 2A-4 | `local_planner.cpp` `computeLocalPlan()`(現 `:536-558`) | `computeFootPlan()` の戻り値を受け、`stop_on_invalid_foothold_ && !ok` なら **NMPC を呼ばず `return false`**。`spin()` は `publishLocalPlan()` を呼ばず、local plan が古くなり `robot_driver` が PD ホールド。`RCLCPP_WARN_THROTTLE` に `[safe-stop]` ログ |
| 2A-5 | `local_planner.cpp` コンストラクタ + `local_planner.hpp` + `local_planner.yaml` | `stop_on_invalid_foothold_`(既定 `true`)を `loadROSParamDefault` で読む。yaml にキー追加(既定安全値、コメントつき) |

**検証(テストコミット)**:

- 新規 3 本、すべて green:
  - `LocalFootstepPlannerTest.ComputeFootPlanReportsOkOnFlatTerrain`
    (平地は `ok==true` / `failed_count==0`、= 回帰ガード)
  - `LocalFootstepPlannerTest.ComputeFootPlanReportsInvalidOverHole`
    (全面非通行地形で `ok==false` / `failed_leg==0` / `failed_touchdown_index==3`
    / `worst_status==NO_TRAVERSABLE_CANDIDATE`、失敗 touchdown 行は穴 nominal
    (x≈0.37)でなく直前値 `{0.20,0.12,0.02}`、`feet.allFinite()`)
  - `LocalPlannerTest.StopOnInvalidFootholdParamDefaultsOnAndCanBeDisabled`
    (既定 `true`、param で `false` にできる)
- **回帰:既存の `local_planner` テストは全 green のまま**
  (`colcon test --packages-select local_planner` → **31 tests / 0 failures**。
  Phase 1 時点の 29 + 今回の 2… ローカル実測 `gtest.xml` は
  `tests="31" failures="0"`)。
- 0.15/0.3/0.5 m/s の既存溝渡り走行での回帰(sim)は **未実施**(次イテレーション。
  ただし平地・VALID 経路では挙動は 1 バイトも変わらない設計:`status==VALID`
  のとき書き込む値は従来と同一)。

**ハマった点(記録)**:`computeFootPlan()` を `void` → `FootPlanResult` に
変えたのに **末尾の `return` を書き忘れた**。`-Wall -Wextra` でも
「control reaches end of non-void function」は **warning 止まり**でビルドは通り、
実行時に**未定義動作**として `ComputeFootPlanHandlesSwingAndTouchdown` が
無限ループ的にハング(ctest 60 s タイムアウト、DIAG ログが数百万行)。
`git stash` で stock に戻して原因を切り分け、`return plan_result;` 追加で解消。

### 次にやること:Phase 2A のシミュレーション検証

ユーザー指定の安全停止シナリオ:**進行方向に幅 10 m の穴**を用意し、
**穴の手前で 3 秒静止できれば OK**(落ちない・転ばない・無効足場を NMPC へ
渡さない)。これは Step 05(15 cm 連続穴)より先に実施する。地形生成器と
ランナーの追加が必要(変更計画は
`agent_reports/steps/step_05_quadsdk_repeated_15cm_gaps.md` の表を流用)。

### そのあと Phase 2B(未設計)

WALKING / STOP_REQUESTED / WAITING_FOR_ALL_CONTACT / STAND / FAILURE_LATCHED
の状態遷移表を作る。遊脚がある時点で失敗を検出した場合の扱い、新 liftoff の
禁止タイミング、`cmd_vel`→0 のタイミング、全脚接地判定に計画値/実測値の
どちらを使うか、最後の有効 plan を何秒使うか、`planner_failed` に購読者が
いない現状への対応、Map 回復時の自動復帰、robot_driver の 0.1 s タイムアウト
との整合、を明確にする。**コード変更前に設計表を提示する。**

## 関連

- `agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md`(解析本体)
- `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md`(twist+クロール成功記録、Phase 0 で訂正)
- `agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md`(gbpl 実験 + 工程別分析)
