# Cursorへの指示：Quad-SDKの穴対応・Foot Placement・NMPC連携の解析と改善

## 0. この指示の目的

`MPC_DOG`内のQuad-SDKについて、平坦路歩行ではなく、**穴がある平面で「穴に足を入れない」制御**を対象にする。

渡された検証資料を最初に読み、その記載を実コードと照合すること。そのうえで、Foot Placement Controlが何を出力し、NMPCがそれをどのように使用しているかを、コードと数式に基づいて説明する。

さらに、現在の穴対応に不足している安全機能を、既存挙動を壊さない小さな変更として順番に実装・検証する。

## 1. 最重要ルール

1. 推測で回答しない。
2. 「資料に記録された実験結果」「コードから確認した事実」「推測・仮説」「提案」を明確に分ける。
3. コード上の主張には、必ずファイルパス、関数名、該当箇所を付ける。
4. 先に資料を読み、その後に実コードを読む。資料だけを要約して終わらない。
5. 一度に大規模変更しない。解析、計装、単体試験、安全停止、候補評価の順に進める。
6. 各フェーズの終了時に結果を報告し、次のフェーズへ進む前にユーザーへ確認を求める。
7. Global Body Plannerの改良は今回の対象外とする。
8. `reference="twist"`でもLocal Footstep Planner、terrain map、NMPCは動作する。`twist`は「地図を使わない」という意味ではない。
9. 平坦路Step 01の再調査は行わない。平坦路歩行は成立済みとして扱う。
10. 成功動画だけで安全性を判断しない。入力、選択足場、失敗条件、MPC入力を数値で確認する。

## 2. 対象リポジトリと基準

- 自リポジトリ：`/home/takuya/work/mpc_dog`
- Quad-SDK：`/home/takuya/work/mpc_dog/external/quad-sdk`
- ROS 2 workspace：`/home/takuya/work/mpc_dog/ros2_ws`
- upstream：`robomechanics/quad-sdk`
- upstream基準コミット：`a3591a9f9e84aa9be3534ee0be107f0829ceb868`
- 実行対象：Go2、MuJoCo、`reference:=twist`

作業開始時に現在のブランチ、コミット、未コミット差分を確認すること。既存のユーザー変更を勝手に破棄、上書き、resetしないこと。

## 3. 最初に読む資料

最初に以下を読む。

1. `docs/steps/step_03_04_1m_quadsdk_gap_crossing.md`
2. `docs/steps/step_03_04_1m_quadsdk_gbpl.md`
3. `agent_reports/quadsdk_step01_control_pipeline.md`
4. `agent_reports/quadsdk_step01_terrain_map.md`
5. `agent_reports/quadsdk_step01_mpc.md`

特に1番の資料から、以下を抽出する。

- 穴の寸法
- 成功した速度、回数、距離
- gait設定
- horizon設定
- foothold探索半径
- PLYと物理ワールドの違い
- `traversability`の実測値
- nominal footholdとselected footholdの実測値
- `found=0`の件数
- 資料内で「確認済み」と書かれた事項
- 資料内で「推測」と書かれた事項

## 4. 資料から確認済みの現状

以下は、資料に記録された実験結果として扱う。コードから自動的に保証される事実とは区別すること。

- 深さ1 m、幅0.3 m、横幅5 mの穴を複数連続で通過した。
- `reference:=twist`を使用した。
- trotからcrawlへ変更した。
- `period: 0.9`
- `duty_cycles: [0.75, 0.75, 0.75, 0.75]`
- `phase_offsets: [0.0, 0.75, 0.5, 0.25]`
- `horizon_length: 40`
- `foothold_search_radius: 0.7`
- `ground_clearance: 0.1`
- 地形PLY上の穴を物理穴より左右0.05 mずつ広く作成した。
- 穴部分の生`z`はNaNになった。
- 穴上の`traversability`はNaNになった。
- 名目足場が穴上に来たとき、平面セルへ足場が移動した。
- 例：nominal `x=1.006, traversability=NaN`からselected `x=1.146`。
- 例：nominal `x=0.940, traversability=NaN`からselected `x=0.890`。
- 記録した実験では、有効候補なしを示す`found=0`は0件だった。

この結果から直ちに「実センサでも安全」と結論してはいけない。現在の成功範囲は、静的な既知PLY、正確な位置合わせ、手作業の安全マージンを使用したシミュレーションである。

## 5. 必ず確認する実コード

最低限、以下のファイルを読む。

### Terrain Map

- `external/quad-sdk/quad_utils/config/filter_chain.yaml`
- `external/quad-sdk/quad_utils/src/mjcf_to_grid_map_converter.cpp`
- `external/quad-sdk/quad_utils/include/quad_utils/fast_terrain_map.hpp`
- `external/quad-sdk/quad_utils/src/fast_terrain_map.cpp`

### Local Planner / Foot Placement

- `external/quad-sdk/local_planner/src/local_planner.cpp`
- `external/quad-sdk/local_planner/src/local_footstep_planner.cpp`
- `external/quad-sdk/local_planner/include/local_planner/local_footstep_planner.hpp`
- `external/quad-sdk/local_planner/config/local_planner.yaml`
- `external/quad-sdk/quad_utils/config/go2.yaml`

### NMPC

- `external/quad-sdk/nmpc_controller/src/nmpc_controller.cpp`
- `external/quad-sdk/nmpc_controller/src/quad_nlp.cpp`
- `external/quad-sdk/nmpc_controller/include/nmpc_controller/quad_nlp.hpp`
- `external/quad-sdk/nmpc_controller/scripts/dynamicsModel.m`

### 下流制御

- `external/quad-sdk/robot_driver/`
- `external/quad-sdk/quad_utils/src/quad_kd2.cpp`
- 足先位置、GRF、逆運動学、関節トルクへ変換している関数

## 6. 最初に作成する解析レポート

コード変更前に、次の内容を1つのMarkdownへまとめる。

出力先：

`agent_reports/quadsdk_gap_foothold_mpc_code_analysis.md`

### 6.1 資料とコードの照合表

次の列を持つ表を作る。

| 資料の主張 | 分類 | コード上の根拠 | 判定 | 補足 |
|---|---|---|---|---|
| 主張 | 実験事実／コード事実／推測 | ファイル・関数・行 | 一致／一部一致／不一致／未確認 | 理由 |

少なくとも次を判定する。

1. `reference="twist"`ではGlobal Body Plannerを使わない。
2. `twist`でもterrain mapによる足場補正を使う。
3. gaitは地形に応じて自動変更されない。
4. 穴上のNaNは候補判定で無効になる。
5. `foothold_search_radius`内で有効セルを探索する。
6. 足場の評価関数は距離だけであり、IK可到達性を含まない。
7. toe半径は水平安全距離の判定には使われない。
8. 有効足場がない場合、名目足場を返す。
9. Go2のNMPCでは足場位置を最適化しない。
10. Go2のNMPCには脚のIK可到達制約がない。
11. `horizon_length > period_`はコード上の必須条件か。
12. 実センサの穴が必ずNaNになる保証があるか。

## 7. Terrain Mapの処理を数式とコードで説明する

`filter_chain.yaml`の順番に沿って、各レイヤの入力、出力、単位、目的を表にする。

標準コードでは概ね以下を計算する。

### 7.1 高さ補間

生の高さを`z`、穴埋め後を`z_inpainted`とする。

### 7.2 傾斜と粗さ

\[
\mathrm{slope}=\arccos(n_z)
\]

\[
\mathrm{roughness}=|z_{\mathrm{inpainted}}-z_{\mathrm{smooth}}|
\]

### 7.3 形状によるtraversability

\[
T_{\mathrm{shape}}
=
0.5\left(1-\frac{\mathrm{roughness}}{0.1}\right)
+
0.5\left(1-\frac{\mathrm{slope}}{0.4}\right)
\]

### 7.4 穴マスク

コード上の式は、

\[
H=1-|z_{\mathrm{finite}}-z_{\mathrm{inpainted}}|
\]

である。

最終的に、

\[
T=(T_{\mathrm{shape}}+0.02)H
\]

を計算する。

ただし、「NaNが各フィルタでどのように伝播するか」は式だけで断定しない。Grid Map Filterの実装または実行ログで確認すること。

## 8. Foot Placement Controlの入出力を追跡する

### 8.1 接触スケジュール

`LocalFootstepPlanner::setTemporalParams()`と`computeContactSchedule()`を確認する。

各脚`j`、時刻`k`について、

\[
c_{j,k}\in\{0,1\}
\]

- `1`：接地脚
- `0`：遊脚

`period`、`duty_cycles`、`phase_offsets`から起動時に固定テーブルを作ることを確認する。

`reference="twist"`では、地形に応じてこの接触スケジュールを変更する処理が存在するかを検索し、結果を明記する。

### 8.2 名目足場

`LocalFootstepPlanner::computeFootPlan()`から、実際の式を抽出する。

コード上の名目足場は概ね、

\[
p_{\mathrm{nom}}
=p_{\mathrm{hip,midstance}}
+\frac{h}{g}(v_{\mathrm{body}}\times\omega_{\mathrm{ref}})
+\sqrt{\frac{h}{g}}(v_{\mathrm{body}}-v_{\mathrm{ref}})
\]

である。

符号、使用している現在値／参照値、座標系をコードから確認すること。

### 8.3 地図による足場補正

`LocalFootstepPlanner::getNearestValidFoothold()`を確認する。

候補集合を、

\[
\mathcal{P}
=
\left\{
p\mid
\|p-p_{\mathrm{nom}}\|\le R,
T(p)>T_{\min}
\right\}
\]

とすると、選択式は、

\[
p^*=\arg\min_{p\in\mathcal{P}}
\left[
\|p-p_{\mathrm{nom}}\|
+0.5\|p-p_{\mathrm{previous}}\|
\right]
\]

である。

高さは、

\[
p_z^*=z_{\mathrm{inpainted}}(p_x^*,p_y^*)+r_{\mathrm{toe}}
\]

となる。

次が評価されていないことをコードで確認する。

- 足裏の面積
- 穴の縁までの距離
- Map位置誤差
- IK可到達性
- 関節角度限界
- 関節速度限界
- 支持多角形
- 地図の時刻・鮮度

### 8.4 Foot Placement Controlの出力

以下について、変数名、型、配列形状、座標系、単位、publish先を表にする。

- `contact_schedule`
- `foot_positions_world`
- `foot_positions_body`
- `foot_velocities_world`
- `foot_accelerations_world`
- discrete foot plan
- continuous foot plan

遊脚軌道について、x/yの三次Hermite補間、zの上昇／下降2区間補間をコードから説明する。

## 9. NMPCへの受け渡しを追跡する

`LocalPlanner::computeLocalPlan()`から`NMPCController::computeLegPlan()`、`quadNLP::update_solver()`、`quadNLP::eval_g()`まで追う。

各関数について次を表にする。

| 関数 | 入力 | 出力 | 足場の扱い | 地図の扱い |
|---|---|---|---|---|

Go2では`enable_mixed_complexity_`が無効になる条件を確認する。

### 9.1 Go2のsimple model

状態を、

\[
x_k=
\begin{bmatrix}
p_b & \theta_b & v_b & \omega_b
\end{bmatrix}^{T}
\in\mathbb{R}^{12}
\]

入力を、

\[
u_k=
\begin{bmatrix}
f_{FL} & f_{BL} & f_{FR} & f_{BR}
\end{bmatrix}^{T}
\in\mathbb{R}^{12}
\]

として、コード上の次元と一致することを確認する。

### 9.2 足場位置の役割

足場位置が決定変数ではなく、動力学のパラメータとして渡されることを確認する。

`dynamicsModel.m`に基づき、離散運動方程式を示す。

\[
q_{k+1}-q_k-\Delta t\dot q_{k+1}=0
\]

\[
M(x_{k+1})(v_{k+1}-v_k)
+\Delta t\left[h(x_{k+1})-J_u(p_f)u_k\right]=0
\]

物理的な意味は概ね、

\[
m\ddot p_b=\sum_j c_j f_j+mg
\]

\[
I\dot\omega+\omega\times I\omega
=\sum_j c_j(p_{f,j}-p_b)\times f_j
\]

である。

足場`p_f`はGRFのモーメントアームとして使われるが、Go2のsimple NMPC自身は足場を移動しないことを明記する。

### 9.3 接触と摩擦制約

接触スケジュールが0の脚では、入力上下限を0にして、

\[
f_j=0
\]

に固定するコードを示す。

接地脚について、Go2設定の、

\[
10\le f_z\le150\ \mathrm{N}
\]

および摩擦ピラミッド、

\[
|f_x|\le\mu f_z,\qquad |f_y|\le\mu f_z
\]

を確認する。

### 9.4 目的関数

コードから、

\[
J=
\frac12\sum_k
\left[
(x_k-x_k^{ref})^TQ(x_k-x_k^{ref})
+(u_k-u_k^{nom})^TR(u_k-u_k^{nom})
\right]
+J_{\mathrm{slack}}
\]

を確認する。

Go2のsimple modelの目的関数や制約に`traversability`が直接含まれるかを検索し、結果を明記する。

## 10. 資料中で再判定が必要な主張

以下は必ずコードと照合し、断定を修正する。

### 10.1 「NMPCの脚可到達制約が破れた」

Go2のsimple modelに関節角度、足位置、IK可到達性の制約が含まれるか確認する。

含まれない場合、正確には次のように整理する。

- 遠い足場はGRFのモーメントアームを変える。
- 胴体参照とGRFの動力学的整合性が悪化する可能性がある。
- 後段のIKや逆動力学が追従できない可能性がある。
- しかし「NMPC内の脚可到達制約違反」とは断定できない。

### 10.2 「horizon_lengthはperiod_より大きい必要がある」

`computeFutureBodyPlan()`によるホライズン外予測を確認する。

- `26 → 40`で実験が改善したことは実験事実。
- `horizon_length > period_`がコード上の必須条件かは別問題。

両者を分離して記載する。

### 10.3 「実機の穴も自然にNaNになる」

現在のリポジトリに、実LiDARまたは深度カメラから`z`レイヤを作る処理があるか確認する。

無い場合は、次を未確認事項とする。

- no-return点の扱い
- occlusionの扱い
- 未観測セルの表現
- 古いセルの表現
- 穴と単なる未観測領域の区別

## 11. コード解析後に行う改善

解析レポートを完成させ、ユーザー確認を受けてから実装する。

### Phase 1：足場選択結果を構造化する

現在の`getNearestValidFoothold()`は位置だけを返す。有効候補がない場合も名目足場を返すため、成功／失敗を下流へ伝えられない。

以下に相当する結果型を追加する案を作る。

```cpp
struct FootholdResult {
  Eigen::Vector3d position;
  bool found;
  double traversability;
  double snap_distance;
  double edge_clearance;
  bool reachable;
};
```

既存APIへの影響範囲を列挙し、最小変更案を提示する。ユーザー確認前に実装しない。

### Phase 2：有効足場なし時の安全停止

`found == false`の場合、名目足場を返して歩行を継続してはいけない。

次の動作を設計する。

1. 新しい一歩を確定しない。
2. `cmd_vel`を0へ減速する。
3. 安全に全脚接地可能ならSTANDへ移行する。
4. `planner_failed`へ失敗理由を通知する。
5. Map外、未観測、Map期限切れを区別する。

急停止で転倒する可能性があるため、即座にトルク0へ切り替える変更は行わない。

### Phase 3：穴縁からの安全距離

PLYの穴を手作業で広げる方法から、地図上で明示的に安全距離を判定する方法へ移行する。

候補足場`p`について、

\[
d_{\mathrm{edge}}(p)
>
r_{\mathrm{toe}}+e_{\mathrm{map}}+m_{\mathrm{safety}}
\]

を要求する。

- `r_toe`：足先半径
- `e_map`：地図とロボット位置の誤差
- `m_safety`：追加安全幅

実装候補を比較する。

1. 危険セルからの2D距離変換レイヤ
2. 候補周囲の円内に無効セルがないことを確認
3. traversabilityマスクのモルフォロジー膨張／安全領域の収縮

計算量、Grid Mapとの統合しやすさ、単体試験のしやすさから推奨案を出す。

### Phase 4：IK可到達性判定

予測着地時の胴体位置・姿勢と候補足場から、脚座標系の候補位置を計算する。

\[
p_f^{leg}=R_{wb}^{T}(p_f-p_{hip})
\]

以下を満たす候補だけを有効にする。

- IK解が存在する。
- 関節角度がGo2の上下限内。
- 必要なら関節速度が上限内。
- 脚の左右を跨ぐ不自然な候補ではない。

既存の`QuadKD2`で利用できる関数を先に探す。新しいIKを重複実装しない。

### Phase 5：大きな足場補正時の減速

\[
d_{\mathrm{snap}}=\|p^*-p_{\mathrm{nom}}\|
\]

を記録する。

大きな補正時には、探索半径を広げて遠い足場を選ぶだけでなく、減速、刻み歩行、停止を選べる設計にする。

閾値は実測前に断定しない。初期候補値を置く場合は「提案値」と明記し、パラメータ化する。

### Phase 6：Map鮮度と不確実性

最低限、次を追加する案を作る。

- terrain mapのheader stamp保存
- 現在時刻との差
- 最大許容Map age
- Map frameとrobot pose frameの整合
- 未観測セルと危険セルの区別
- Map更新停止時の減速・停止

## 12. 最初に作る単体試験

MuJoCo全体試験の前に、足場選択器だけのテストを作る。

| 試験 | 入力 | 期待結果 |
|---|---|---|
| 平面 | 名目足場が平面中央 | ほぼ移動しない |
| 穴中央 | 名目足場が穴中央 | 安全領域へ移動 |
| 穴の縁 | 名目足場が縁付近 | 必要安全距離を確保 |
| 広い穴 | 安全セルが探索半径外 | `found=false` |
| Map外 | 名目足場がMap外 | `found=false` |
| 未観測 | 候補が未観測セル | `found=false` |
| 到達不能 | traversabilityは高いがIK範囲外 | `reachable=false` |
| 古いMap | stampが許容時間超過 | 計画停止 |

各試験で次を記録する。

- 脚番号
- touchdown indexと時刻
- nominal foothold `[m]`
- selected foothold `[m]`
- nominal traversability
- selected traversability
- snap distance `[m]`
- edge clearance `[m]`
- IK可否
- found
- 失敗理由

## 13. 閉ループ試験

単体試験が通った後に、現在成功している静的PLY穴越えを回帰試験として使う。

最低限、次を比較する。

- 変更前と変更後の到達距離
- 穴へ入った足の数
- 最小穴縁距離
- 最大snap distance
- NMPC成功率
- NMPC cost
- roll、pitch、base height
- GRFの最大値
- planner failure回数
- 停止時の転倒有無

最初から実センサMapへ進まない。静的Mapで安全失敗処理を成立させた後に、オンラインMapへ進む。

## 14. 実装前に提示する変更計画

コード変更前に、次の形式でユーザーへ提示する。

| 順番 | 変更ファイル | 変更関数 | 変更内容 | 必要な理由 | 既存挙動への影響 | 検証方法 |
|---|---|---|---|---|---|---|

一度の変更は1つの目的に限定する。

例：

1. 診断値追加のみ
2. 戻り値に`found`追加
3. `found=false`の単体試験
4. 安全停止への伝播
5. edge clearance追加
6. IK可到達性追加

## 15. 禁止事項

- 平坦路Step 01の再実装
- Global Body Plannerの改造
- いきなり足場位置をNMPC決定変数へ追加する大規模変更
- `foothold_search_radius`を根拠なくさらに拡大すること
- 有効足場なし時に名目足場で継続すること
- 動画だけで成功判定すること
- PLYの安全マージンだけで一般的な安全性を主張すること
- 実センサの未観測セルが必ずNaNになると仮定すること
- ユーザーの既存差分を破棄すること
- 関係ないファイルを整理・フォーマットすること
- 大量の修正を1コミットへまとめること

## 16. 最終的に説明すべき役割分担

最終レポートでは、次を初心者にも分かるように説明する。

| モジュール | 主入力 | 主出力 | 穴への責任 |
|---|---|---|---|
| Terrain Map | 高さ・未観測セル | `traversability`など | 穴／危険領域を表現する |
| Contact Schedule | gait設定 | 接地／遊脚時刻 | いつ足を上げるか決める。twistでは地形適応しない |
| Footstep Planner | 胴体予測、Map、現在足場 | 着地点、足先軌道 | 穴を避けて足場を選ぶ |
| NMPC | 胴体参照、固定足場、接触時刻 | 胴体軌道、GRF | 与えられた足場で動力学を成立させる |
| Robot Driver | 胴体・足先・GRF計画 | 関節指令／トルク | 計画を実機またはMuJoCoで実行する |

必ず次の結論を明記する。

> 現在のGo2構成では、MPCが穴を避けて足場を最適化しているのではない。Local Footstep Plannerが地図から足場を決定し、NMPCはその足場を固定パラメータとして胴体軌道とGRFを最適化する。

## 17. 現時点の技術的結論

### コードと資料から確認できる事実

- 静的PLYの穴をNaN／低traversabilityとして表現できれば、Local Footstep Plannerは穴上の名目足場を近傍の有効セルへ移動できる。
- `reference="twist"`でもterrain mapによる足場補正は動作する。
- gaitは地形から自動変更されない。
- Go2のsimple NMPCは足場を最適化しない。
- NMPCは足場をGRFのモーメントアームとして使用する。
- 有効足場がない場合、現コードは警告後に名目足場を返す。
- 足場選択には足裏面積、縁距離、IK可到達性、Map鮮度が含まれない。

### まだ確認できていないこと

- 実LiDAR／深度カメラMapで穴が確実にNaNまたは低traversabilityになること。
- 地図誤差を含めても足が穴の縁から十分離れること。
- 0.7 m探索範囲内の候補がGo2の脚で到達可能であること。
- 有効足場がない状況で安全に停止できること。

### 次の最優先対策

`getNearestValidFoothold()`を、位置だけでなく成功／失敗と診断値を返す足場選択器へ変更し、有効足場がない場合に名目足場を返さず、安全に減速・停止できる経路を作る。

その後に、穴縁からの安全距離、IK可到達性、Map鮮度の順で追加する。

## 18. 最初の返答形式

この指示を受けたら、まだコードを変更せず、次の4点だけを返す。

1. 読んだ資料とコードファイル一覧
2. 資料の穴対応状況の要約
3. 資料の主張とコードの一致／不一致の重要項目
4. 最初に作成する解析レポートの章立て

不明点がなければ解析レポート作成まで進め、コード変更前に停止してユーザー確認を求めること。

