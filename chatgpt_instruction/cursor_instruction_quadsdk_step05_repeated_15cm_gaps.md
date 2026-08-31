# Cursor / Coding Agent 指示書

## Quad-SDK Step05：15 cm平地・15 cm穴の連続区間（N=2〜5）

## 0. 指示の目的

Quad-SDKのGo2＋MuJoCo環境で、進行方向に次のパターンを繰り返す地形を作り、Terrain Map、Foot Placement、NMPCが連続した穴に対してどこまで機能するかを検証してください。

```text
15 cm平地 → 15 cm穴 → 15 cm平地 → 15 cm穴 → ...
```

穴の数`N`を`2, 3, 4, 5`へ段階的に増やします。

このStepの目的は、単に全ケースを強引に渡らせることではありません。

1. 現在のFoot Placementが連続した狭い支持面を正しく選べるか確認する。
2. 穴縁の危険帯を除くと、実際に何cmの接地可能領域が残るか確認する。
3. どの`N`まで連続通過できるか測定する。
4. 通過できない場合、転倒ではなく安全停止へ移行できるか確認する。
5. 失敗原因をTerrain Map、足場選択、IK可到達性、NMPC、下位制御に分離する。

## 1. 絶対条件

- 推測とコード・ログで確認した事実を明確に分けてください。
- 最初に既存コード、既存Step、現在の設定値を読んでください。
- 読む前に実装やパラメータ変更を始めないでください。
- 既存のStep01〜Step04の地形、スクリプト、ログを上書きしないでください。
- 既存のユーザー変更を削除、巻き戻し、整理しないでください。
- `git reset --hard`、`git checkout -- <file>`などの破壊的操作は禁止です。
- 1コミットを1目的にしてください。
- 比較中は一度に複数の制御パラメータを変更しないでください。
- 成功させるために、穴や危険帯を無断で狭くしないでください。
- `N=2`を確認せず、いきなり`N=5`を実行しないでください。
- 実装着手前に、調査結果と変更計画を提示してください。

## 2. 作業対象

- リポジトリルート：`/home/takuya/work/mpc_dog`
- Quad-SDK：`external/quad-sdk`
- ROS 2 workspace：`ros2_ws`
- ロボット：Go2
- シミュレータ：MuJoCo
- Planner入力：`reference:=twist`
- 対象：Local Footstep Planner＋Terrain Map＋NMPC＋Robot Driver
- Global Body Planner：対象外

`reference:=twist`でもTerrain Map、Local Footstep Planner、NMPCは使用します。無効になるのはGlobal Body Plannerです。

## 3. 最初に読む資料

存在するファイルを`rg --files`で確認してから、少なくとも次を読んでください。

1. `agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md`
2. `agent_reports/steps/step_03_04_1m_quadsdk_gap_crossing.md`
3. `agent_reports/steps/step_03_04_1m_quadsdk_gbpl.md`
4. `docs/quad_sdk_step01_investigation.md`
5. `docs/quad_sdk_step01_changes_and_usage.md`
6. `chatgpt_instruction/cursor_instruction_quadsdk_gap_foothold_analysis.md`
7. 現在のStep03／Step04実行スクリプト
8. 現在使用中の穴地形のMuJoCo world／Xacro／PLY
9. `external/quad-sdk/local_planner/src/local_footstep_planner.cpp`
10. `external/quad-sdk/local_planner/src/local_planner.cpp`
11. `external/quad-sdk/quad_utils/config/filter_chain.yaml`
12. `external/quad-sdk/nmpc_controller/src/nmpc_controller.cpp`
13. `external/quad-sdk/nmpc_controller/src/quad_nlp.cpp`
14. `external/quad-sdk/robot_driver/src/controllers/inverse_dynamics_controller.cpp`

資料名や配置が異なる場合は、類似名を推測して進めず、`rg --files`と`rg`で正しいファイルを特定してください。

## 4. 最初に確認するGit状態

次を確認し、レポート冒頭へ記録してください。

```bash
cd /home/takuya/work/mpc_dog
git status --short
git branch --show-current
git rev-parse HEAD
git -C external/quad-sdk status --short
git -C external/quad-sdk branch --show-current
git -C external/quad-sdk rev-parse HEAD
```

未コミット変更がある場合は、誰の変更か不明でも削除しないでください。Step05と重なるファイルが変更済みなら、着手前に報告してください。

## 5. 地形の定義

### 5.1 基本形状

地面上面の高さを`z_ground`、パターン開始位置を`x0`とします。

各`i = 0 ... N-1`について、次の区間を作ります。

\[
x_{support,start}(i)=x_0+0.30i
\]

\[
x_{support,end}(i)=x_0+0.30i+0.15
\]

\[
x_{gap,start}(i)=x_0+0.30i+0.15
\]

\[
x_{gap,end}(i)=x_0+0.30i+0.30
\]

したがって、テスト区間長は次です。

\[
L_{test}=0.30N
\]

| N | 穴の数 | テスト区間長 |
|---:|---:|---:|
| 2 | 2 | 0.60 m |
| 3 | 3 | 0.90 m |
| 4 | 4 | 1.20 m |
| 5 | 5 | 1.50 m |

テスト区間の前には十分な助走平面、最後の穴の後には十分な着地・停止平面を置いてください。助走長と着地長は既存Stepの初期位置、立ち上がり時間、記録時間を確認して決め、採用値と根拠を報告してください。

### 5.2 横方向

- 15 cm平地と15 cm穴は、同じ横幅を持たせてください。
- 穴はロボットが横へ迂回できないよう、既存テスト通路の有効横幅全体を横断させてください。
- 横幅を新しく推測しないでください。既存の穴試験world／PLYから値を読み、同じ値を使用してください。
- 使用した`y_min`、`y_max`、幅をレポートへ記録してください。

### 5.3 深さ

- 穴の深さは地面上面から`1.0 m`とします。
- 物理シミュレーションでは、必要なら穴底を`z_ground - 1.0 m`に配置してください。
- Terrain Map用PLYでは、穴の上面をまたぐ三角形を作らないでください。
- 穴底をTerrain Map用PLYへ含めた結果、穴セルが`z=-1.0 m`の通常地形として扱われないようにしてください。
- MuJoCoの衝突形状とTerrain Map用メッシュを分ける必要があるか、既存実装を読んで判断してください。

## 6. 危険帯の成立性を実装前に計算する

既存の穴縁マージンをコード・PLY・生成スクリプトから確認してください。期待値は片側`m=0.05 m`ですが、確認前に事実として扱わないでください。

物理穴が15 cmで、片側マージンが5 cmなら、1つの穴に対する接地禁止帯は次です。

\[
L_{forbidden}=0.05+0.15+0.05=0.25\ \mathrm{m}
\]

連続する穴の間にある15 cm平地は、左右の穴から5 cmずつ侵食されるため、安全領域は次です。

\[
L_{safe\ support}=0.15-0.05-0.05=0.05\ \mathrm{m}
\]

```text
進行方向 →

   穴15 cm       平地15 cm        穴15 cm
┈┈┈┈┈┈┈┈┈┈│←5→│←─ 安全5 cm ─→│←5→│┈┈┈┈┈┈┈┈┈┈
            ↑                      ↑
       左の穴の余裕           右の穴の余裕
```

この5 cmが本当に足場として成立するか、次を実装前に確認してください。

1. Go2の足先衝突形状の直径または有効接触幅
2. Footstep Plannerで使用している`toe_radius`
3. `toe_radius`が水平安全判定に使われているか、Z補正だけか
4. Terrain Mapのresolution
5. 5 cm領域に有効セルが何列残るか
6. IKで到達可能か
7. crawlの一歩あたり前進量と足場間隔が整合するか
8. 4脚すべてについて支持多角形が成立するか

### 必須判断

実装前の調査結果として、次のいずれかを明記してください。

- `成立可能性あり`：足先寸法、Map resolution、IKの観点で候補が存在する。
- `境界条件`：セル数や足先寸法に対して余裕がほぼない。
- `幾何学的に成立困難`：安全領域が足先より狭い、または有効セルが残らない。

成立困難でも、穴やマージンを勝手に変更しないでください。この場合のStep05の主目的は、通過ではなく正しく失敗を検出して安全停止できることになります。

## 7. Terrain Mapの事前検証

ロボットを歩かせる前に、`N=2,3,4,5`の各地形でTerrain Mapを検証してください。

最低限、各区間の中央と境界付近で次を記録してください。

- `z`
- `z_inpainted`
- `z_smooth`
- `traversability`
- Map内／Map外
- finite／NaN
- 接地可／接地禁止の最終判定

### 期待する区別

- 物理穴15 cm
- 穴縁マージン5 cm
- 平地中央の安全領域5 cm
- 通常平地

PLYの見た目だけで正しいと判断しないでください。実際にLocal Footstep Plannerが購読する`terrain_map`の数値で確認してください。

### 必須出力

- 上面図またはRViz画像
- x方向の`traversability`断面
- 各穴の物理範囲
- 各穴の接地禁止範囲
- 各平地の残存安全幅
- Map resolutionと有効セル数

## 8. Step05のベースライン条件

最初の比較では、現在のStep03／Step04で最も再現性が高かった設定を使用してください。

最低限、次の実値を既存ファイルから読み、レポートへ記録してください。

- `cmd_vel`
- gait名
- `period`
- `duty_cycles`
- `phase_offsets`
- `ground_clearance`
- `horizon_length`
- NMPC時間刻み
- `foothold_search_radius`
- `foothold_obj_threshold`
- `obj_fun_layer`
- 摩擦係数の実行時パラメータ
- 使用するIPOPT linear solver
- 穴縁マージン
- Terrain Map resolution

ベースライン比較中は、`N`以外の値を固定してください。

## 9. 実験順序

### Stage A：地形だけを検証

1. `N=2`の物理地形を表示する。
2. MuJoCoのcollision形状を確認する。
3. Terrain Mapの各レイヤーを確認する。
4. 穴底が接地可能面として扱われていないことを確認する。
5. 15 cm平地に残る安全領域の幅とセル数を確認する。
6. 同じ確認を`N=3,4,5`へ展開する。

### Stage B：Foot Placementだけを検証

可能ならロボット本体の制御試験前に、合成した名目足場列を入力して次を記録してください。

- 名目足場
- 探索した候補
- 選択足場
- `FootholdStatus`
- `traversability_nominal`
- `traversability_selected`
- `snap_distance`
- 脚番号
- 対象接地時刻
- Map内／外
- IK成立／不成立（IK判定がまだ未実装なら「未判定」と書く）

既存Phase 1の`FootholdResult`を利用してください。存在しない診断値を成功値として捏造しないでください。

### Stage C：Nを順番に増やす

次の順番で実行してください。

```text
N=2 → N=3 → N=4 → N=5
```

- まず各`N`を1回実行する。
- 安全に通過できた`N`は、同一条件で合計3回実行して再現性を確認する。
- 転倒または危険な足場選択が発生した`N`では、自動的に次の`N`へ進まない。
- 失敗原因を分析し、変更計画を提示する。
- ユーザー承認なしに複数パラメータをまとめて変更しない。

### Stage D：安全停止を検証

その`N`が幾何学的または運動学的に通過不能な場合、次を確認してください。

1. 無効足場をNMPCへ渡さない。
2. 新しい離脚を開始しない。
3. すでに遊脚中の脚を安全に着地させる。
4. `cmd_vel`を段階的にゼロへ近づける。
5. 全脚接地を確認する。
6. STANDまたは失敗ラッチ状態へ移る。
7. ロボットが穴へ落ちない。

Phase 2が未完成なら、「安全停止できなかった」をStep05の結果として記録し、先にPhase 2を実装すべきと結論してください。名目足場へフォールバックして歩行を継続しないでください。

## 10. 成功・失敗の判定

### 10.1 通過成功

最低限、次をすべて満たした場合だけ通過成功としてください。

- ロボット胴体が最後の穴を越え、着地平面上の所定位置へ到達した。
- 転倒していない。
- 足の実接触点が物理穴または危険帯へ入っていない。
- `FootholdStatus != VALID`の足場を実行していない。
- 非finiteな足場をNMPCへ渡していない。
- NMPC／Local Planの重大な連続失敗がない。
- 最後に安全に停止できる。

単にbaseのx座標がテスト区間を越えただけでは成功にしないでください。ジャンプ、滑落、穴底への接触、危険帯への接触を区別してください。

### 10.2 安全停止成功

通過不能でも次を満たす場合は、安全停止成功として別分類してください。

- 無効足場を選択しなかった。
- 穴へ落ちなかった。
- 転倒しなかった。
- 新規離脚を停止した。
- 全脚または設計上定義した安全接触状態で停止した。
- 失敗理由をログへ残した。

### 10.3 失敗

- 穴または危険帯へ接触した。
- 穴底へ落下した。
- 転倒した。
- 無効足場をNMPCへ渡した。
- `NO_TRAVERSABLE_CANDIDATE`後に名目足場で歩行を継続した。
- Map外を黙って`continue`した。
- 非finite値がLocal Plan、GRF、トルクへ伝播した。
- プロセス異常終了により判定不能になった。

## 11. 記録するデータ

### 11.1 試行単位

- 試行ID
- `N`
- 乱数seed（使用する場合）
- 速度指令
- gait設定
- horizon
- search radius
- Map resolution
- 危険帯マージン
- 開始時刻・終了時刻
- 最終base位置
- 通過成功／安全停止成功／失敗
- 転倒時刻
- 最初の足場失敗時刻
- 最初のNMPC失敗時刻
- 失敗分類

### 11.2 時系列

- base位置・姿勢・速度
- 各脚の実位置
- 各脚の接触状態
- 各脚のGRF
- 名目足場
- 選択足場
- `FootholdStatus`
- `snap_distance`
- 選択地点のtraversability
- 接触点から最寄り穴縁までの距離
- Local Plan age
- NMPC計算時間
- NMPC iteration
- IPOPT status（取得可能なら）
- constraint violation／slack（取得可能なら）
- control mode／停止状態

## 12. Nごとの比較表

最終レポートに次の表を作成してください。

| N | 区間長 | 安全領域幅 | 有効セル数 | 通過回数/試行数 | 安全停止 | 最大snap | NMPC失敗 | 最終結果 | 主原因 |
|---:|---:|---:|---:|---:|---|---:|---:|---|---|
| 2 | 0.60 m | 実測 | 実測 |  |  |  |  |  |  |
| 3 | 0.90 m | 実測 | 実測 |  |  |  |  |  |  |
| 4 | 1.20 m | 実測 | 実測 |  |  |  |  |  |  |
| 5 | 1.50 m | 実測 | 実測 |  |  |  |  |  |  |

## 13. 失敗原因の分類

失敗時は、少なくとも次へ分類してください。

| 分類 | 確認内容 |
|---|---|
| 地形生成 | MuJoCo地形とPLYが一致しているか |
| Map生成 | 穴、穴縁、安全領域が正しい値か |
| Map解像度 | 5 cm安全領域に有効セルが残っているか |
| Foot Placement | 有効候補を探索・選択できたか |
| 評価関数 | 前回足場への近さが不適切な候補を優先していないか |
| IK可到達性 | 選択足場へ各脚が届くか |
| 支持多角形 | crawl中の重心が支持領域内か |
| NMPC | 固定された足場とGRFで運動方程式・摩擦制約を満たせるか |
| Solver | IPOPT status、反復数、constraint violation |
| 下位制御 | 計画足場と実足先の追従誤差 |
| 安全遷移 | 足場失敗後に新規離脚を止めたか |

「遠い足場だからNMPCが失敗した」と推測だけで断定しないでください。Go2 simple NMPCにはIK可到達制約がないため、因果を主張する場合はログで示してください。

## 14. 実装方法の方針

### 推奨

- Step05専用の地形と実行スクリプトを追加する。
- `N`を引数で変更できる方式を優先する。
- MuJoCo worldとTerrain Map用PLYの形状を同じ定義から生成し、手作業の座標不一致を避ける。
- 既存のStep03／Step04地形は変更しない。
- 生成物とソースを区別する。

### 想定ファイル名

実際のリポジトリ構成を確認して調整して構いませんが、役割が分かる名前にしてください。

```text
scripts/trial/run_quadsdk_step05_repeated_gaps.sh
src/trial/quadsdk_step05_repeated_gaps.py
scripts/terrain/generate_repeated_gap_terrain.py
agent_reports/steps/step_05_quadsdk_repeated_15cm_gaps.md
artifacts/logs/quadsdk_step05/n02/
artifacts/logs/quadsdk_step05/n03/
artifacts/logs/quadsdk_step05/n04/
artifacts/logs/quadsdk_step05/n05/
```

既存の地形生成方式が別にある場合は、重複実装を作らず再利用してください。

## 15. 実装前に提示する変更計画

コード変更前に、次の形式で提示してください。

| # | 変更ファイル | 現状 | 変更内容 | 必要な理由 | 制御挙動への影響 | 検証方法 |
|---:|---|---|---|---|---|---|
| 1 |  |  |  |  |  |  |

さらに、次を明記してください。

- 変更しないファイル
- 再ビルドが必要なROS 2 package
- 地形生成だけで済むか、C++変更が必要か
- Phase 2の安全停止実装がStep05より先に必要か
- 1コミットごとの目的

この変更計画を提示した時点で一度停止し、ユーザーの承認を待ってください。

## 16. コミットの分割例

実際の差分に応じて調整してください。

1. `Add parameterized repeated-gap terrain generator`
2. `Add Step05 repeated-gap map validation`
3. `Add Step05 foothold diagnostics and recorder`
4. `Add Step05 staged N=2..5 runner`
5. `Document Step05 repeated-gap results`

Phase 2の安全停止が未実装なら、地形追加コミットへ混ぜず別目的・別コミットにしてください。

## 17. 最初の回答で行うこと

最初の回答では、まだコード変更をしないでください。次だけを実施してください。

1. Git状態を確認する。
2. 指定資料と実コードを読む。
3. 現在のStep03／Step04成功条件とパラメータを特定する。
4. 現在の穴縁マージンを特定する。
5. Go2足先寸法、Terrain Map resolution、残存安全幅を計算する。
6. 15 cm平地が足場として成立するか判定する。
7. Phase 2の安全停止が実装済みか確認する。
8. 事実、未確認、推測を分けて報告する。
9. 変更計画表を提示する。
10. ユーザーの承認を待つ。

## 18. 最初の回答フォーマット

```markdown
# Step05 事前調査結果

## 結論

## コードで確認した事実

## 現在の実験パラメータ

## 15 cm平地＋危険帯の幾何学的成立性

## Terrain Map解像度との整合

## Go2足先寸法との整合

## IK・支持多角形との整合

## Phase 2安全停止の実装状態

## 未確認事項

## 推測・仮説

## 変更計画

## ユーザー判断が必要な項目
```

## 19. 最終的に答えるべき問い

1. 15 cmの物理穴はTerrain Map上で正しく接地禁止になっているか。
2. 片側5 cmの危険帯を考慮した結果、15 cm平地には何cm・何セル残るか。
3. その領域へGo2の足先を物理的に置けるか。
4. その領域へ各脚がIK上到達できるか。
5. `N=2,3,4,5`のどこまで再現性を持って通過できるか。
6. 通過回数が増えると、足場補正誤差やNMPC誤差が蓄積するか。
7. 通過不能時に危険な名目足場を使わず安全停止できるか。
8. 限界を決めているのはMap、Foot Placement、IK、NMPC、下位制御のどこか。

Step05の価値は、`N=5`を無理に成功させることではありません。連続穴に対する現在の方式の成立範囲と、安全に失敗できる境界をコードとログで明らかにすることです。
