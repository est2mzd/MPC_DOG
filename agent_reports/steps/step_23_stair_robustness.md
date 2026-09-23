# 階段歩行のロバスト性

## 第1章 概要

- Step 23は、高さ 0.15 m、踏面奥行き 0.30 m、4段上り下りの同一worldを反復し、変更前後の成功率を比較する
- 基準試験は、足先支持判定を `shadow` に保ち、同一条件を5回実行した
- 基準の有効試行は5回で、2回は最上面付近の転倒、2回は下り2段目のピッチ転倒、1回は下り2段目のヨー増加後の横ずれであり、その前の5回は起動異常である
- 第1変更は、`foothold_support_check_mode` だけを `enforce` へ変え、同一条件の反復を始めた
- 変更後の1回目は、最上面の前端でロールが崩れて失敗した
- 変更後の2回目は、接近平地でヨーが増え、最初の蹴上の手前でロールが崩れて失敗した
- 変更後の3回目は、最上面の前端 x=4.068 m まで到達したあと、x=3.861 m でピッチが崩れて失敗した
- 変更後の4回目も、最上面の前端 x=4.060 m まで到達したあと、x=3.822 m でピッチが崩れて失敗した
- 変更後の5回目は、上り最終段の x=3.611 m でピッチが崩れ、最上面には乗らずに失敗した
- 変更後の5回はすべて失敗し、下りを始めた試行は無い
- 第2変更は、`foothold_edge_inset_mode` だけを `enforce` にし、足先支持判定は `shadow` に戻したままである
- 段端後退の1回目は、最上面の前端 x=4.019 m でロールが崩れて失敗した
- 段端後退の2回目は、最上面の前端 x=4.087 m まで乗ったあと、x=3.848 m でピッチが崩れて失敗した
- 段端後退の3回目は、最上面の前端でヨーが増え、下り1段目 x=5.073 m でロールが崩れて失敗した
- 段端後退の4回目は、下り2段目 x=5.324 m でロールが崩れて失敗した
- 段端後退の5回目は、最上面の前端 x=3.873 m でロールが崩れて失敗した
- 段端後退の5回はすべて失敗し、転倒前に下りへ入ったのは2回である
- 第3変更は、`support_triangle_shift_mode` だけを `enforce` にする
- 支持三角形の1回目は、`foothold_edge_inset_mode` の無引用 `off` を bool と読んで `local_planner_node` が死に、最大 x=0.998 m の起動異常である
- 支持三角形の2回目は、接近平地 x=2.697 m でロールが崩れて失敗した
- 採用候補を組み合わせた確認試験は5回とも歩行を開始したが、完走は0回、下り到達も0回である
- 採用候補の確認試験は、2回が最上面前端へ到達したあと上り最終段へ後退してロール転倒し、3回が最初の蹴上付近でピッチ転倒した
- 同一設定で結果が分岐する主因は、階段への到達脚位相を固定せず、足先支持を `shadow` で観測するだけにして、支持余裕を制御へ反映していない構成にある
- 計画周期の処理時間は基準群と確認群で近いため、計画処理時間の増加だけでは今回の分岐を説明できない
- ロバスト性を上げるには、支持可能領域から着地点を生成し、複数歩の着地点と胴体支持余裕を同時に整合させ、着地点が無い位相では遊脚を開始しない構成が必要である
- 成功判定は、退出位置、横ずれ、胴体高さ、ロール、ピッチを同時に要求する
- 結論は、個別対策の採用判断だけでなく、採用候補を組み合わせた5回の確認試験に基づいて出す
- 第3章から第6章と第9章の速度変更は、0.15 m/s、0.20 m/s、0.30 m/s を追加し、完走を増やしていない
- 第12章は、上り 0.30 m/s のあと下りだけ 0.10 m/s にした5回も完走していない
- 第13章は、最上段の平らな面で頭を 0.10 rad 下げた5回も、後ろ向きの転倒を消していない
- 第14章は、前足を次の1段の奥の踏める位置へ置く実行で、最初の蹴上の手前から失敗している
- 第15章は、転倒前に5区間の出口を超えた回数で、基準 0.10 m/s と速度を含む条件を比べる
- 第16章は、基準状態が Quad-SDK の main から変えた歩容、先読み、速度、振り足、平滑法線を示す
- 第17章は、基準状態の形状確認の進行を表に残し、試行のたびに同じ表を更新する

## 第2章 固定条件

### 概要

- 地形は、高さ 0.15 m、踏面奥行き 0.30 m、4段上り下り、最上面奥行き 1.00 mに固定する
- 歩行は、速度 0.10 m/sと既存のクロール歩容に固定する
- 実行環境は、同じRelease overlayと同じworldに固定する
- 判定は、最大 xと終了姿勢に加えて横ずれを使う

### 詳細

- 地形は、高さ 0.15 m、踏面奥行き 0.30 m、4段上り下り、最上面奥行き 1.00 mに固定する
  - worldは `jp_stair_matrix_h15_d30_n04.xml` である
  - 最初の蹴上は x=3.00 mである
  - 下り終端は x=5.80 mである
- 歩行は、速度 0.10 m/sと既存のクロール歩容に固定する
  - 歩行周期は 0.90 sである
  - 各脚のデューティ比は 0.75である
  - 速度指令時間は90 sである
- 実行環境は、同じRelease overlayと同じworldに固定する
  - overlayは `/tmp/mpc_dog_stack_release_install/setup.bash` である
  - 振り足地形判定は `enforce` に固定する
  - 足先IK判定は `shadow` に固定する
- 判定は、最大 xと終了姿勢に加えて横ずれを使う
  - 最大 xは 7.30 m以上を要求する
  - 終了時の yの絶対値は 0.50 m未満を要求する
  - 終了時の zは 0.20 mより大きいことを要求する
  - 終了時のロールとピッチの絶対値は 0.50 rad未満を要求する

## 第3章 基準試験

### 概要

- 基準試験は、`foothold_support_check_mode: shadow` のまま5回実行した
- 1回目は、最上面の前端でロールが崩れて失敗した
- 2回目から5回目は、胴体が初期位置の直立を維持し、`local_planner_node` の周期ログを残さなかった
- 追加の1回も、最大 x=0.998 m のまま直立し、起動異常である
- 共有メモリ削除後の1回目は歩行を再開し、下り2段目の x=5.366 m でピッチが崩れて失敗した
- 共有メモリ削除後の2回目も、下り2段目の x=5.373 m でピッチが崩れて失敗した
- 共有メモリ削除後の3回目は、下り2段目でヨーが 0.60 rad まで増え、退出平地を斜めに歩いて終了 y=1.152 m で失敗した
- 共有メモリ削除後の4回目は、最上面の前端 x=4.063 m まで到達したあと後退し、上り最終段 x=3.732 m でピッチが崩れて失敗した
- 起動異常の5回は、階段上のロバスト性の分母に入れない

### 詳細

- 基準試験は、`foothold_support_check_mode: shadow` のまま5回実行した

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash bash scripts/trial/run_quadsdk_stair_robustness.sh baseline_shadow 5
```

- 1回目は、最上面の前端でロールが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| 試行 | `baseline_shadow` の1回目 | 変更前の設定で、階段上の転倒位置を記録する |
| `GAP_WORLD` | `jp_stair_matrix_h15_d30_n04.xml` | 高さ 0.15 m、踏面奥行き 0.30 m、4段上り下り、最上面奥行き 1.00 m の同一地形で、変更前後の到達を比べる |
| `FORWARD_VEL_MPS` | `0.10` | 速度を要因にせず、既存クロールの指令速度を固定する |
| `local_footstep_planner.period` | `0.9` | クロールの歩行周期を固定する |
| `local_footstep_planner.duty_cycles` | `[0.75, 0.75, 0.75, 0.75]` | 常に3脚以上を接地させるクロールの接地比を固定する |
| `local_footstep_planner.phase_offsets` | `[0.0, 0.75, 0.5, 0.25]` | 横方向のクロール順を固定する |
| `DURATION_S` | `90` | 退出平地の先まで前進指令を続ける |
| `HOLD_S` | `5` | 指令のあと零速度を送り、終了姿勢を残す |
| `PLAN_STARTUP_S` | `3` | プランナ起動後に WALK を送るまでの待ちは、スクリプトの既定 3 s を使う |
| `foothold_support_check_mode` | `shadow` | 足先全体の支持可否を記録するが着地点は拒否せず、変更前の歩行を残す |
| `foothold_ik_check_mode` | `shadow` | 足先IKによる着地点の拒否を、この比較の要因にしない |
| `swing_terrain_check_mode` | `enforce` | 振り足の頂点を中間地形より上へ上げる既存設定を固定する |
| `stair_tread_snap_max_run` | `0.0` | 短い踏面を中央へ移す処理を無効にし、中央化を要因にしない |
| `foothold_edge_inset_mode` | `未設定` | この実行ファイルにはこの変数が無く、螺旋探索の着地点を段端から動かさない |
| `local_planner.toe_radius` | `0.022` | 段端から内側へ引く距離の元になる足先半径である |
| `foothold_support_margin` | `0.0` | 足先半径の外に支持判定の余裕を足さない |
| `multistep_planner.enabled` | `true` | 平地と溝と階段を一つの先読みで扱う |
| `multistep_planner.apply_foothold` | `false` | 先読みの着地点列を名目着地点へ戻さず、既存の最近傍選択を残す |

  - 最上面は x=3.90 m から 4.90 m である
  - 最前進は x=4.061 m である
  - 転倒開始は時刻 53.23 s、x=3.851 m、z=0.787 m、ロール 0.801 rad、ピッチ -0.452 rad である
  - この位置は、高さ 0.15 m、奥行き 0.30 m、4段の単発試行の転倒開始 x=3.876 m と近い
  - NMPC失敗は461回で、転倒開始後の継続である

![基準試験の1回目](../../artifacts/logs/quadsdk_step23_baseline_shadow_r01/trial.gif)

- 2回目から5回目は、胴体が初期位置の直立を維持し、`local_planner_node` の周期ログを残さなかった

  - 1回目から変えた変数は無い

  - 4回とも最大 x は 0.998 m、終了 z は 0.318 m、ロールは 0 rad 付近である
  - 起動ログは `WALK` と `cmd_vel` の送信まで進んでいる
  - `local_planner_node` のログは起動警告で止まり、`LocalPlanner took` を残していない
  - 1回目の同じログは歩行周期の処理時間を残している
  - 転倒開始は4回とも記録されていない

![基準試験の2回目](../../artifacts/logs/quadsdk_step23_baseline_shadow_r02/trial.gif)

![基準試験の3回目](../../artifacts/logs/quadsdk_step23_baseline_shadow_r03/trial.gif)

![基準試験の4回目](../../artifacts/logs/quadsdk_step23_baseline_shadow_r04/trial.gif)

![基準試験の5回目](../../artifacts/logs/quadsdk_step23_baseline_shadow_r05/trial.gif)

- 追加の1回も、最大 x=0.998 m のまま直立し、起動異常である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |

  - `foothold_support_check_mode` は `shadow` のままである
  - 終了時の z は 0.318 m、ロールは 0 rad 付近である
  - `local_planner_node` は `Loaded Pinocchio` と `LocalPlanner took` を残していない
  - NMPC失敗は0回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh shadow06 1
```

![追加基準の起動異常](../../artifacts/logs/quadsdk_step23_shadow06_r01/trial.gif)

- 共有メモリ削除後の1回目は歩行を再開し、下り2段目の x=5.366 m でピッチが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |

  - `foothold_support_check_mode` は `shadow` のままである
  - `local_planner_node` は `Loaded Pinocchio` と `LocalPlanner took` を記録し、周期は約 8 ms である
  - 最上面は x=3.90 m から 4.90 m、下り2段目は x=5.20 m から 5.50 m、高さ 0.30 m である
  - 転倒開始は時刻 69.05 s、x=5.366 m、y=-0.041 m、z=0.761 m、ピッチ 0.801 rad、ロール -0.293 rad である
  - 転倒後の終了姿勢は x=6.917 m、z=0.155 m、ロール -1.587 rad である
  - 記録上の最大 x=7.023 m は転倒後の移動なので、完走には数えない
  - NMPC失敗は453回で、転倒開始後の継続である

```bash
rm -f /dev/shm/fastrtps_* && QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh shadow07 1
```

![共有メモリ削除後の1回目](../../artifacts/logs/quadsdk_step23_shadow07_r01/trial.gif)

- 共有メモリ削除後の2回目も、下り2段目でピッチが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |

  - 転倒開始は時刻 72.02 s、x=5.373 m、y=0.211 m、z=0.789 m、ピッチ 0.802 rad、ロール 0.368 rad である
  - 転倒前の最大 x は 5.372 m で、直前の有効試行の x=5.366 m と 0.007 m しか違わない
  - 転倒後の終了姿勢は x=6.393 m、y=0.330 m、z=0.155 m、ロール 1.587 rad である
  - 記録上の最大 x=6.460 m は転倒後の移動なので、完走には数えない
  - NMPC失敗は446回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh shadow08 1
```

![共有メモリ削除後の2回目](../../artifacts/logs/quadsdk_step23_shadow08_r01/trial.gif)

- 共有メモリ削除後の3回目は、下り2段目でヨーが増えて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |

  - `foothold_support_check_mode` は `shadow` のままである
  - 立ち上がり後に、z が 0.22 m 未満、またはロールかピッチの絶対値が 0.80 rad を超える転倒開始は出ていない
  - ヨー 0.204 rad は時刻 66.95 s、x=5.169 m、y=-0.170 m、z=0.783 m、ピッチ 0.668 rad である
  - 時刻 67.01 s には x=5.195 m、ヨー 0.404 rad である
  - 時刻 67.07 s には x=5.221 m、ヨー 0.605 rad である
  - 最大ロールは時刻 67.39 s、x=5.284 m、ロール 0.590 rad、ピッチ 0.647 rad、ヨー 0.742 rad で、0.80 rad には達していない
  - 下り2段目は x=5.20 m から 5.50 m、高さ 0.30 m である
  - 退出 x=5.800 m ではヨー 0.632 rad、y=-0.077 m、z=0.398 m である
  - 終了姿勢は x=7.565 m、y=1.152 m、z=0.309 m、ロール -0.000 rad、ピッチ 0.000 rad、ヨー 0.686 rad である
  - 最大 x=7.574 m は直立歩行中の値で、終了 y の絶対値 1.152 m が 0.50 m 未満を外している
  - NMPC失敗は0回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh shadow09 1
```

![共有メモリ削除後の3回目](../../artifacts/logs/quadsdk_step23_shadow09_r01/trial.gif)

- 共有メモリ削除後の4回目は、最上面の前端まで到達したあと後退してピッチが崩れた

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |

  - `foothold_support_check_mode` は `shadow` のままである
  - 最前進は時刻 59.04 s、x=4.063 m、y=-0.092 m、z=0.898 m、ロール -0.023 rad、ピッチ -0.136 rad、ヨー -0.023 rad である
  - 最上面は x=3.90 m から 4.90 m、高さ 0.60 m なので、この時点では前端に乗っている
  - ヨー 0.201 rad は時刻 59.45 s、x=3.980 m である
  - ヨー 0.402 rad は時刻 60.55 s、x=3.747 m、ピッチ -0.642 rad である
  - 転倒開始は時刻 64.91 s、x=3.732 m、y=-0.212 m、z=0.800 m、ピッチ -0.806 rad、ロール -0.020 rad、ヨー 0.464 rad である
  - x=3.732 m は上り最終段で、その段は x=3.60 m から 3.90 m、高さ 0.45 m である
  - 終了姿勢は x=2.775 m、y=-0.459 m、z=0.155 m、ロール 1.587 rad である
  - 記録上の最大 x=4.063 m は転倒前の到達で、下りは始めていない
  - NMPC失敗は518回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh shadow10 1
```

![共有メモリ削除後の4回目](../../artifacts/logs/quadsdk_step23_shadow10_r01/trial.gif)

- 起動異常の5回は、階段上のロバスト性の分母に入れない
  - 有効な基準試行は5回である
  - 1回は最上面の前端のロール転倒、1回は最上面到達後の後退とピッチ転倒、2回は下り2段目のピッチ転倒、1回は下り2段目のヨー増加後の横ずれである
  - 次の試行前にも `/dev/shm/fastrtps_*` を削除する
  - 基準5回が揃うまで `foothold_support_check_mode` は `shadow` のままで、その後に `enforce` へ変えた

### 速度変更

#### 概要

- 0.15 m/s の5回は、歩行した3回が失敗し、2回は起動異常で、完走は0回である
- 0.20 m/s は、起動異常の1回のあと歩行した5回が失敗し、下りまで届いたのは1回である
- 0.30 m/s の歩行5回は完走0回で、転倒前に最上面へ届いたのは4回、下りまで届いたのは3回である

#### 詳細

- 0.15 m/s の5回は、歩行した3回が失敗し、2回は起動異常で、完走は0回である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.15` | 基準の 0.10 m/s より速い指令で、同じ階段を歩けるかを見る |
| `foothold_support_check_mode` | `shadow` | 基準と同じく、足先支持の不足を記録するだけで着地点は拒否しない |
| `PLAN_STARTUP_S` | `8` | 計画周期へ入る前に WALK を送ることを避ける |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.15 bash scripts/trial/run_quadsdk_stair_robustness.sh speed_base_v15 5
```

- 1回目は、最上面の前端 x=3.862 m でピッチ -0.801 rad、遊脚 FL の転倒である
- 最大 x は 4.065 m で、下り x=4.90 m には届いていない
- NMPC失敗は400回である

![基準0.15 m/sの1回目](../../artifacts/logs/quadsdk_step23_speed_base_v15_r01/trial.gif)

- 2回目は、最大 x=0.998 m のまま直立し、起動異常である
- 終了 z は 0.319 m、ロールは 0 rad 付近である
- NMPC失敗は480回である

![基準0.15 m/sの2回目](../../artifacts/logs/quadsdk_step23_speed_base_v15_r02/trial.gif)

- 3回目も、最大 x=1.303 m のまま直立し、起動異常である
- 終了 z は 0.311 m、ロールは 0.004 rad である
- NMPC失敗は1回である

![基準0.15 m/sの3回目](../../artifacts/logs/quadsdk_step23_speed_base_v15_r03/trial.gif)

- 4回目は、最上面の前端 x=3.896 m でロール 0.802 rad、遊脚 FL の転倒である
- 最大 x は 4.062 m で、下りには届いていない
- NMPC失敗は130回である

![基準0.15 m/sの4回目](../../artifacts/logs/quadsdk_step23_speed_base_v15_r04/trial.gif)

- 5回目は、上り1段目 x=3.278 m でロール -0.806 rad、遊脚 FL の転倒である
- 最大 x は 3.302 m で、最上面には届いていない
- NMPC失敗は536回である

![基準0.15 m/sの5回目](../../artifacts/logs/quadsdk_step23_speed_base_v15_r05/trial.gif)

- 0.20 m/s は、起動異常の1回のあと歩行した5回が失敗し、下りまで届いたのは1回である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.20` | 0.15 m/s の次の速度として、基準と同じ支持判定のまま到達を比べる |

- 起動異常の1回は、最大 x=0.965 m、終了ロール -3.142 rad で、階段へ入っていない
- NMPC失敗は885回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.20 bash scripts/trial/run_quadsdk_stair_robustness.sh speed_base_v20 1
```

![基準0.20 m/sの起動異常](../../artifacts/logs/quadsdk_step23_speed_base_v20_r01/trial.gif)

- 歩行した1回目は、最初の蹴上の手前 x=2.824 m でロールが反転した
- 最大 x は 3.031 m で、最上面には届いていない
- NMPC失敗は491回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.20 bash scripts/trial/run_quadsdk_stair_robustness.sh baseline_v020 1
```

![基準0.20 m/sの1回目](../../artifacts/logs/quadsdk_step23_baseline_v020_r01/trial.gif)

- 2回目は、最初の蹴上の手前 x=2.940 m でロール -0.885 rad、遊脚 FR の転倒である
- 最大 x は 3.072 m である
- NMPC失敗は570回である

![基準0.20 m/sの2回目](../../artifacts/logs/quadsdk_step23_baseline_v020_more_r01/trial.gif)

- 3回目は、最上面へ x=4.071 m まで乗ったあと、x=3.831 m でピッチ -0.802 rad、遊脚 BR の転倒である
- 下り x=4.90 m には届いていない
- NMPC失敗は699回である

![基準0.20 m/sの3回目](../../artifacts/logs/quadsdk_step23_baseline_v020_more_r02/trial.gif)

- 4回目は、下り x=5.354 m でピッチ 0.804 rad、遊脚 BR の転倒である
- 最大 x は 6.888 m で、転倒後の移動を含む
- NMPC失敗は622回である

![基準0.20 m/sの4回目](../../artifacts/logs/quadsdk_step23_baseline_v020_more_r03/trial.gif)

- 5回目は、最上面へ x=4.072 m まで乗ったあと、x=3.891 m でピッチ -0.802 rad、遊脚 FL の転倒である
- 下りには届いていない
- NMPC失敗は702回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.20 bash scripts/trial/run_quadsdk_stair_robustness.sh baseline_v020_more 4
```

![基準0.20 m/sの5回目](../../artifacts/logs/quadsdk_step23_baseline_v020_more_r04/trial.gif)

- 0.30 m/s の歩行5回は完走0回で、転倒前に最上面へ届いたのは4回、下りまで届いたのは3回である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.30` | 0.20 m/s よりさらに速い指令で、最上段と下りへの到達が増えるかを見る |

- 1回目は、下り x=5.572 m でロール -0.800 rad、遊脚 BL の転倒である
- 最大 x は 6.324 m で、完走の x=7.30 m と終了姿勢の条件は満たしていない
- NMPC失敗は58回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 bash scripts/trial/run_quadsdk_stair_robustness.sh baseline_v030 1
```

![基準0.30 m/sの1回目](../../artifacts/logs/quadsdk_step23_baseline_v030_r01/trial.gif)

- 2回目は、下り x=5.443 m でピッチ 0.805 rad、遊脚 FL の転倒である
- 最大 x は 7.349 m だが、終了 z は 0.059 m、終了ロールは 3.121 rad で失敗である
- NMPC失敗は648回である

![基準0.30 m/sの2回目](../../artifacts/logs/quadsdk_step23_baseline_v030_more_r01/trial.gif)

- 3回目は、上り x=3.260 m でピッチ -0.800 rad、遊脚 BL の転倒である
- 最大 x は 3.431 m で、最上面には届いていない
- NMPC失敗は745回である

![基準0.30 m/sの3回目](../../artifacts/logs/quadsdk_step23_baseline_v030_more_r02/trial.gif)

- 4回目は、最上面の前端 x=3.971 m でロール -0.804 rad、遊脚 FR の転倒である
- 最大 x は 4.086 m で、下りには届いていない
- NMPC失敗は766回である

![基準0.30 m/sの4回目](../../artifacts/logs/quadsdk_step23_baseline_v030_more_r03/trial.gif)

- 5回目は、下り x=5.883 m でロール -0.811 rad、遊脚 BL の転倒である
- 最大 x は 7.768 m、終了 y は -1.301 m で、横ずれの条件を外している
- NMPC失敗は475回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 bash scripts/trial/run_quadsdk_stair_robustness.sh baseline_v030_more 4
```

![基準0.30 m/sの5回目](../../artifacts/logs/quadsdk_step23_baseline_v030_more_r04/trial.gif)

- 基準の有効5回は 0.10 m/s で完走0回、下り到達3回である
- 0.15 m/s、0.20 m/s、0.30 m/s も完走は0回である
- 0.30 m/s の下り到達は3回で、基準の3回と同じである
- 基準試行と比較して
  - 改善無し
- 速度を上げた設定は、階段の成功設定として採用しない

## 第4章 足先支持判定の適用

### 概要

- 第1変更は、`foothold_support_check_mode` だけを `shadow` から `enforce` へ変える
- 変更後試験は、基準の有効試行を揃えたあとで5回実行する
- 変更後の1回目は、最上面の前端 x=4.021 m でロールが崩れて失敗した
- 変更後の2回目は、接近平地の x=1.446 m でヨーが 0.518 rad になり、最初の蹴上の手前でロールが崩れて失敗した
- 変更後の3回目は、最上面の前端 x=4.068 m まで到達したあと、x=3.861 m でピッチが崩れて失敗した
- 変更後の4回目も、最上面の前端 x=4.060 m まで到達したあと、x=3.822 m でピッチが崩れて失敗した
- 変更後の5回目は、上り最終段の x=3.611 m でピッチが崩れ、最上面には乗らずに失敗した
- 変更後の5回はすべて失敗し、下りを始めた試行は無い

### 詳細

- 第1変更は、`foothold_support_check_mode` だけを `shadow` から `enforce` へ変える
  - `shadow` は足先全体の支持可否を記録するが、着地点を拒否しない
  - `enforce` は足先全体を支持できない着地点を拒否し、別の候補を探索する
  - 足先IK判定、振り足地形判定、速度、歩容、地形は変更しない
- 変更後試験は、基準の有効試行を揃えたあとで5回実行する
  - 基準試験と同じ成功条件を使う
  - 成功率だけでなく、停止、転倒、起動異常を区別して記録する
- 変更後の1回目は、最上面の前端 x=4.021 m でロールが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_support_check_mode` | `enforce` | 1回目の shadow から enforce に変え、足先全体を支持できない着地点を拒否する |

  - `foothold_support_check_mode` は `enforce`、足先IK判定は `shadow`、振り足地形判定は `enforce` である
  - `local_planner_node` は `Loaded Pinocchio` と `LocalPlanner took` を記録している
  - 最前進は時刻 60.76 s、x=4.106 m、y=-0.029 m、z=0.920 m、ロール -0.015 rad、ピッチ -0.070 rad、ヨー 0.408 rad である
  - 転倒開始は時刻 61.37 s、x=4.021 m、y=-0.162 m、z=0.870 m、ロール 0.806 rad、ピッチ -0.251 rad、ヨー 0.361 rad である
  - 最上面は x=3.90 m から 4.90 m、高さ 0.60 m なので、転倒は前端の上である
  - 終了姿勢は x=3.933 m、y=-0.328 m、z=0.679 m、ロール 1.875 rad である
  - 下りは始めていない
  - NMPC失敗は526回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh enforce01 1
```

![足先支持判定enforceの1回目](../../artifacts/logs/quadsdk_step23_enforce01_r01/trial.gif)

- 変更後の2回目は、接近平地でヨーが増えて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_support_check_mode` | `enforce` | 1回目の shadow から enforce に変え、足先全体を支持できない着地点を拒否する |

  - `foothold_support_check_mode` は `enforce` のままである
  - ヨー 0.518 rad は時刻 28.07 s、x=1.446 m、y=0.205 m、z=0.310 m である
  - 最初の蹴上は x=3.00 m なので、この時点は接近平地である
  - x=2.000 m では y=0.534 m、ヨー 0.511 rad である
  - x=3.000 m では y=1.117 m、ヨー 0.560 rad である
  - 最前進は時刻 50.50 s、x=3.136 m、y=1.195 m、z=0.448 m、ヨー 0.543 rad である
  - 階段の横半幅は 1.50 m なので、y=1.195 m は踏面の内側である
  - 転倒開始は時刻 53.51 s、x=2.794 m、y=0.852 m、z=0.353 m、ロール 0.805 rad、ピッチ -0.474 rad、ヨー -0.956 rad である
  - 終了姿勢は x=2.764 m、y=0.731 m、z=0.306 m、ロール 0.052 rad、ピッチ -0.012 rad、ヨー -1.378 rad である
  - 段の上には乗っていない
  - NMPC失敗は105回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh enforce02 1
```

![足先支持判定enforceの2回目](../../artifacts/logs/quadsdk_step23_enforce02_r01/trial.gif)

- 変更後の3回目は、最上面の前端まで到達したあとピッチが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_support_check_mode` | `enforce` | 1回目の shadow から enforce に変え、足先全体を支持できない着地点を拒否する |

  - `foothold_support_check_mode` は `enforce` のままである
  - 最前進は時刻 57.25 s、x=4.068 m、y=-0.030 m、z=0.904 m、ロール -0.007 rad、ピッチ -0.157 rad、ヨー -0.002 rad である
  - 最上面は x=3.90 m から 4.90 m、高さ 0.60 m なので、この時点では前端に乗っている
  - 転倒開始は時刻 58.81 s、x=3.861 m、y=0.121 m、z=0.913 m、ピッチ -0.801 rad、ロール -0.414 rad、ヨー 0.386 rad である
  - x=3.861 m は上り最終段の終端で、その段は x=3.60 m から 3.90 m、高さ 0.45 m である
  - 終了姿勢は x=2.446 m、y=-0.307 m、z=0.169 m、ロール -2.679 rad である
  - 記録上の最大 x=4.068 m は転倒前の到達で、下りは始めていない
  - NMPC失敗は487回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh enforce03 1
```

![足先支持判定enforceの3回目](../../artifacts/logs/quadsdk_step23_enforce03_r01/trial.gif)

- 変更後の4回目も、最上面の前端まで到達したあとピッチが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_support_check_mode` | `enforce` | 1回目の shadow から enforce に変え、足先全体を支持できない着地点を拒否する |

  - `foothold_support_check_mode` は `enforce` のままである
  - 最前進は時刻 61.77 s、x=4.060 m、y=0.042 m、z=0.899 m、ロール 0.048 rad、ピッチ -0.168 rad、ヨー -0.182 rad である
  - 転倒開始は時刻 62.64 s、x=3.822 m、y=0.065 m、z=0.877 m、ピッチ -0.806 rad、ロール 0.258 rad、ヨー -0.106 rad である
  - x=3.822 m は上り最終段の終端で、その段は x=3.60 m から 3.90 m、高さ 0.45 m である
  - 終了姿勢は x=2.790 m、y=-0.279 m、z=0.155 m、ロール 1.587 rad である
  - 記録上の最大 x=4.060 m は転倒前の到達で、下りは始めていない
  - NMPC失敗は518回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh enforce04 1
```

![足先支持判定enforceの4回目](../../artifacts/logs/quadsdk_step23_enforce04_r01/trial.gif)

- 変更後の5回目は、上り最終段でピッチが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_support_check_mode` | `enforce` | 1回目の shadow から enforce に変え、足先全体を支持できない着地点を拒否する |

  - `foothold_support_check_mode` は `enforce` のままである
  - 最前進は時刻 49.87 s、x=3.791 m、y=-0.003 m、z=0.794 m、ロール -0.053 rad、ピッチ -0.467 rad、ヨー 0.010 rad である
  - 転倒開始は時刻 50.81 s、x=3.611 m、y=0.048 m、z=0.755 m、ピッチ -0.801 rad、ロール -0.029 rad、ヨー 0.027 rad である
  - 上り最終段は x=3.60 m から 3.90 m、高さ 0.45 m で、最上面の開始 x=3.90 m には届いていない
  - 終了姿勢は x=2.000 m、y=0.307 m、z=0.155 m、ロール 1.587 rad である
  - NMPC失敗は557回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh enforce05 1
```

![足先支持判定enforceの5回目](../../artifacts/logs/quadsdk_step23_enforce05_r01/trial.gif)

- 変更後の5回はすべて失敗し、下りを始めた試行は無い
  - 1回目は最上面の前端 x=4.021 m のロール転倒である
  - 2回目は接近平地でヨーが約 0.52 rad になり、最初の蹴上の手前でロール転倒した
  - 3回目は x=4.068 m まで乗ったあと x=3.861 m のピッチ転倒、4回目は x=4.060 m まで乗ったあと x=3.822 m のピッチ転倒である
  - 5回目は上り最終段 x=3.611 m のピッチ転倒で、最上面に乗っていない
  - 基準の有効試行では2回が下り2段目まで到達し、1回は退出平地を直立で歩いた
  - 足先支持判定の `enforce` は、その到達を延ばしていない

### 速度変更

#### 概要

- 0.20 m/s の5回は完走0回で、転倒前に最上面へ届いたのは3回、下りまで届いたのは1回である
- 0.30 m/s の5回は完走0回で、転倒前に最上面へ届いたのは3回、下りまで届いたのは1回である

#### 詳細

- 0.20 m/s の5回は完走0回で、転倒前に最上面へ届いたのは3回、下りまで届いたのは1回である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.20` | 第4章の `enforce` のまま、基準より速い指令の到達を見る |
| `foothold_support_check_mode` | `enforce` | 支持不足の候補を拒否する第4章の設定を固定する |
| `PLAN_STARTUP_S` | `8` | 計画周期へ入ってから WALK を送る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.20 bash scripts/trial/run_quadsdk_stair_robustness.sh enforce_v020 5
```

- 1回目は、最上面の前端 x=3.992 m でロール 0.802 rad、遊脚 FL の転倒である
- 最大 x は 4.127 m で、下りには届いていない
- NMPC失敗は692回である

![支持判定0.20 m/sの1回目](../../artifacts/logs/quadsdk_step23_enforce_v020_r01/trial.gif)

- 2回目は、最上面へ x=4.065 m まで乗ったあと、x=3.813 m でロール -0.802 rad、遊脚 BL の転倒である
- NMPC失敗は655回である

![支持判定0.20 m/sの2回目](../../artifacts/logs/quadsdk_step23_enforce_v020_r02/trial.gif)

- 3回目は、下り x=5.761 m でピッチ 0.801 rad、遊脚 BL の転倒である
- 最大 x は 6.390 m である
- NMPC失敗は656回である

![支持判定0.20 m/sの3回目](../../artifacts/logs/quadsdk_step23_enforce_v020_r03/trial.gif)

- 4回目は、上り x=3.150 m でロール -0.802 rad、遊脚 BR の転倒である
- 最大 x は 3.343 m である
- NMPC失敗は730回である

![支持判定0.20 m/sの4回目](../../artifacts/logs/quadsdk_step23_enforce_v020_r04/trial.gif)

- 5回目は、上り x=3.266 m でロール 0.807 rad、遊脚 FR の転倒である
- 最大 x は 3.313 m である
- NMPC失敗は669回である

![支持判定0.20 m/sの5回目](../../artifacts/logs/quadsdk_step23_enforce_v020_r05/trial.gif)

- 0.30 m/s の5回は完走0回で、転倒前に最上面へ届いたのは3回、下りまで届いたのは1回である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.30` | `enforce` のまま、0.20 m/s より速い指令の到達を見る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 bash scripts/trial/run_quadsdk_stair_robustness.sh enforce_v030 5
```

- 1回目は、最上面の前端 x=4.038 m でロール 0.800 rad、遊脚 BR の転倒である
- 最大 x は 4.097 m である
- NMPC失敗は102回である

![支持判定0.30 m/sの1回目](../../artifacts/logs/quadsdk_step23_enforce_v030_r01/trial.gif)

- 2回目は、最上面へ x=4.072 m まで乗ったあと、x=3.617 m でピッチ -0.804 rad、遊脚 BL の転倒である
- NMPC失敗は747回である

![支持判定0.30 m/sの2回目](../../artifacts/logs/quadsdk_step23_enforce_v030_r02/trial.gif)

- 3回目は、上り x=3.241 m でロール 0.801 rad、遊脚 FR の転倒である
- 最大 x は 3.538 m である
- NMPC失敗は671回である

![支持判定0.30 m/sの3回目](../../artifacts/logs/quadsdk_step23_enforce_v030_r03/trial.gif)

- 4回目は、上り x=3.228 m でロール -0.800 rad、遊脚 FL の転倒である
- 最大 x は 3.280 m である
- NMPC失敗は640回である

![支持判定0.30 m/sの4回目](../../artifacts/logs/quadsdk_step23_enforce_v030_r04/trial.gif)

- 5回目は、下り x=5.721 m でピッチ 0.800 rad、遊脚 FR の転倒である
- 最大 x は 6.859 m である
- NMPC失敗は648回である

![支持判定0.30 m/sの5回目](../../artifacts/logs/quadsdk_step23_enforce_v030_r05/trial.gif)

- 第4章の 0.10 m/s は下り到達0回、完走0回である
- 0.20 m/s と 0.30 m/s も完走は0回で、下り到達は各1回である
- 基準試行と比較して
  - 改善無し
- `foothold_support_check_mode: enforce` は、速度を上げても採用しない

## 第5章 段端からの足先後退

### 概要

- 第2変更は、`foothold_edge_inset_mode` だけを `off` から `enforce` へ変える
- 足先支持判定は `shadow` のまま、踏面中央への移動は無効のままである
- 段端後退の1回目は、最上面の前端 x=4.019 m でロールが崩れて失敗した
- 段端後退の2回目は、最上面の前端 x=4.087 m まで乗ったあと、x=3.848 m でピッチが崩れて失敗した
- 段端後退の3回目は、最上面の前端でヨーが増え、下り1段目 x=5.073 m でロールが崩れて失敗した
- 段端後退の4回目は、下り2段目 x=5.324 m でロールが崩れて失敗した
- 段端後退の5回目は、最上面の前端 x=3.873 m でロールが崩れて失敗した
- 段端後退の5回はすべて失敗し、転倒前に下りへ入ったのは2回である

### 詳細

- 第2変更は、`foothold_edge_inset_mode` だけを `off` から `enforce` へ変える
  - `enforce` は、踏面の高さ変化を段端とみなし、足の x を段端から `toe_radius` 0.022 m だけ内側へ移す
  - 探索半径の端と地図の端は段端にしないので、平地の足は動かさない
  - 踏面が両側の後退量より狭いときだけ、足の x を測定区間の中間へ移す
- 足先支持判定は `shadow` のまま、踏面中央への移動は無効のままである
  - `foothold_support_check_mode` は `shadow` である
  - `stair_tread_snap_max_run` は 0 m である
  - 足先IK判定は `shadow`、振り足地形判定は `enforce` である
- 段端後退の1回目は、最上面の前端 x=4.019 m でロールが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_edge_inset_mode` | `enforce` | 1回目は未設定で着地点を動かさなかったので、この試行は段端から足先半径だけ内側へ足の x を引く |

  - `local_planner_node` は `Loaded Pinocchio` と `LocalPlanner took` を記録している
  - 最前進は時刻 56.42 s、x=4.060 m、y=-0.003 m、z=0.899 m、ロール 0.031 rad、ピッチ -0.150 rad、ヨー -0.038 rad である
  - 転倒開始は時刻 57.34 s、x=4.019 m、y=-0.291 m、z=0.858 m、ロール 0.800 rad、ピッチ -0.226 rad、ヨー -0.231 rad である
  - 最上面は x=3.90 m から 4.90 m、高さ 0.60 m なので、転倒は前端の上である
  - 終了姿勢は x=2.743 m、y=-1.088 m、z=0.155 m、ロール -1.587 rad である
  - 下りは始めていない
  - NMPC失敗は548回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh inset01 1
```

![段端後退の1回目](../../artifacts/logs/quadsdk_step23_inset01_r01/trial.gif)

- 段端後退の2回目は、最上面の前端まで乗ったあとピッチが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_edge_inset_mode` | `enforce` | 1回目は未設定で着地点を動かさなかったので、この試行は段端から足先半径だけ内側へ足の x を引く |

  - `foothold_edge_inset_mode` は `enforce`、`foothold_support_check_mode` は `shadow` である
  - 最前進は時刻 57.91 s、x=4.087 m、y=-0.195 m、z=0.898 m、ロール 0.023 rad、ピッチ -0.098 rad、ヨー -0.114 rad である
  - 転倒開始は時刻 58.82 s、x=3.848 m、y=-0.293 m、z=0.900 m、ピッチ -0.802 rad、ロール 0.470 rad、ヨー -0.498 rad である
  - x=3.848 m は上り最終段の終端で、その段は x=3.60 m から 3.90 m、高さ 0.45 m である
  - 終了姿勢は x=2.904 m、y=-0.379 m、z=0.155 m、ロール 1.587 rad である
  - 記録上の最大 x=4.087 m は転倒前の到達で、下りは始めていない
  - NMPC失敗は526回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh inset02 1
```

![段端後退の2回目](../../artifacts/logs/quadsdk_step23_inset02_r01/trial.gif)

- 段端後退の3回目は、最上面の前端でヨーが増えてから下り1段目でロールが崩れた

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_edge_inset_mode` | `enforce` | 1回目は未設定で着地点を動かさなかったので、この試行は段端から足先半径だけ内側へ足の x を引く |

  - `foothold_edge_inset_mode` は `enforce`、`foothold_support_check_mode` は `shadow` である
  - ヨー 0.201 rad は時刻 58.25 s、x=4.039 m、y=-0.012 m、z=0.924 m である
  - ヨー 0.602 rad は時刻 63.64 s、x=4.071 m、y=0.067 m、z=0.975 m である
  - 下り開始 x=4.900 m では時刻 80.52 s、y=1.141 m、ヨー 1.207 rad、ロール 0.459 rad である
  - 転倒開始は時刻 81.47 s、x=5.073 m、y=1.085 m、z=0.934 m、ロール 0.802 rad、ピッチ -0.632 rad、ヨー 1.377 rad である
  - 下り1段目は x=4.90 m から 5.20 m、高さ 0.45 m である
  - 転倒前の最大 x は 5.073 m である
  - 記録上の最大 x=5.826 m は転倒後の移動なので、完走には数えない
  - 終了姿勢は x=5.817 m、y=0.691 m、z=0.229 m、ロール -1.482 rad、ピッチ 0.398 rad である
  - NMPC失敗は10回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh inset03 1
```

![段端後退の3回目](../../artifacts/logs/quadsdk_step23_inset03_r01/trial.gif)

- 段端後退の4回目は、下り2段目でロールが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_edge_inset_mode` | `enforce` | 1回目は未設定で着地点を動かさなかったので、この試行は段端から足先半径だけ内側へ足の x を引く |

  - `foothold_edge_inset_mode` は `enforce`、`foothold_support_check_mode` は `shadow` である
  - 下り開始 x=4.900 m は時刻 65.81 s、y=0.021 m、z=0.857 m、ピッチ 0.592 rad、ヨー 0.548 rad である
  - x=5.200 m は時刻 66.12 s、ヨー 1.756 rad、ピッチ 0.701 rad である
  - 転倒開始は時刻 66.23 s、x=5.324 m、y=0.048 m、z=0.668 m、ロール 0.802 rad、ピッチ 0.471 rad、ヨー 2.000 rad である
  - 下り2段目は x=5.20 m から 5.50 m、高さ 0.30 m である
  - 転倒前の最大 x は 5.324 m である
  - 記録上の最大 x=6.522 m は転倒後の移動なので、完走には数えない
  - 終了姿勢は x=6.495 m、y=-0.015 m、z=0.155 m、ロール 1.617 rad である
  - NMPC失敗は470回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh inset04 1
```

![段端後退の4回目](../../artifacts/logs/quadsdk_step23_inset04_r01/trial.gif)

- 段端後退の5回目は、最上面の前端でロールが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `foothold_edge_inset_mode` | `enforce` | 1回目は未設定で着地点を動かさなかったので、この試行は段端から足先半径だけ内側へ足の x を引く |

  - `foothold_edge_inset_mode` は `enforce`、`foothold_support_check_mode` は `shadow` である
  - 最前進は時刻 60.39 s、x=4.039 m、y=0.019 m、z=0.892 m、ロール -0.026 rad、ピッチ -0.173 rad、ヨー 0.049 rad である
  - 転倒開始は時刻 61.00 s、x=3.873 m、y=-0.292 m、z=0.779 m、ロール 0.801 rad、ピッチ -0.414 rad、ヨー 0.051 rad である
  - x=3.873 m は上り最終段の終端で、最上面の開始は x=3.90 m である
  - 終了姿勢は x=3.216 m、y=-1.132 m、z=0.344 m、ロール 2.680 rad である
  - 下りは始めていない
  - NMPC失敗は419回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh inset05 1
```

![段端後退の5回目](../../artifacts/logs/quadsdk_step23_inset05_r01/trial.gif)

- 段端後退の5回はすべて失敗し、転倒前に下りへ入ったのは2回である
  - 1回目は最上面の前端 x=4.019 m のロール転倒、5回目は x=3.873 m のロール転倒である
  - 2回目は x=4.087 m まで乗ったあと x=3.848 m のピッチ転倒である
  - 3回目は最上面でヨーが 0.60 rad まで増えたあと、下り1段目 x=5.073 m のロール転倒である
  - 4回目は下り2段目 x=5.324 m のロール転倒で、基準の下り2段目ピッチ転倒と同じ段である
  - 成功は0回で、基準の有効試行にも退出平地を直立で歩いた記録がある
  - `toe_radius` 0.022 m の段端後退は、その到達を完走へ変えていない

### 速度変更

#### 概要

- 0.20 m/s は起動異常1回と失敗4回で、完走は0回、下りまで届いたのは1回である
- 0.30 m/s の5回は完走0回で、転倒前に最上面へ届いたのは2回、下りまで届いたのは1回である

#### 詳細

- 0.20 m/s は起動異常1回と失敗4回で、完走は0回、下りまで届いたのは1回である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.20` | 段端後退 `enforce` のまま、速い指令の到達を見る |
| `foothold_edge_inset_mode` | `enforce` | 第5章の段端後退を固定する |
| `foothold_support_check_mode` | `shadow` | 支持判定の拒否は戻したままにする |
| `PLAN_STARTUP_S` | `8` | 計画周期へ入ってから WALK を送る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.20 bash scripts/trial/run_quadsdk_stair_robustness.sh inset_v020 5
```

- 1回目は、最大 x=0.967 m のまま直立し、起動異常である
- 終了 z は 0.081 m、ピッチは -0.092 rad である
- NMPC失敗は19回である

![段端後退0.20 m/sの1回目](../../artifacts/logs/quadsdk_step23_inset_v020_r01/trial.gif)

- 2回目は、最上面へ x=4.058 m まで乗ったあと、x=3.755 m でピッチ -0.804 rad、遊脚 FL の転倒である
- NMPC失敗は709回である

![段端後退0.20 m/sの2回目](../../artifacts/logs/quadsdk_step23_inset_v020_r02/trial.gif)

- 3回目は、下り x=5.153 m でロール -0.800 rad、遊脚 BR の転倒である
- 最大 x は 6.044 m である
- NMPC失敗は646回である

![段端後退0.20 m/sの3回目](../../artifacts/logs/quadsdk_step23_inset_v020_r03/trial.gif)

- 4回目は、上り x=3.151 m でピッチ -0.806 rad、遊脚 BL の転倒である
- 最大 x は 3.367 m である
- NMPC失敗は709回である

![段端後退0.20 m/sの4回目](../../artifacts/logs/quadsdk_step23_inset_v020_r04/trial.gif)

- 5回目は、最上面の前端 x=3.914 m でロール -0.809 rad、遊脚 FR の転倒である
- 最大 x は 4.066 m である
- NMPC失敗は718回である

![段端後退0.20 m/sの5回目](../../artifacts/logs/quadsdk_step23_inset_v020_r05/trial.gif)

- 0.30 m/s の5回は完走0回で、転倒前に最上面へ届いたのは2回、下りまで届いたのは1回である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.30` | 段端後退のまま、0.20 m/s より速い指令の到達を見る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 bash scripts/trial/run_quadsdk_stair_robustness.sh inset_v030 5
```

- 1回目は、上り x=3.508 m でロール 0.801 rad、遊脚 FR の転倒である
- 最大 x は 3.567 m である
- NMPC失敗は777回である

![段端後退0.30 m/sの1回目](../../artifacts/logs/quadsdk_step23_inset_v030_r01/trial.gif)

- 2回目は、上り x=3.209 m でロール -0.802 rad、遊脚 FL の転倒である
- 最大 x は 3.290 m である
- NMPC失敗は774回である

![段端後退0.30 m/sの2回目](../../artifacts/logs/quadsdk_step23_inset_v030_r02/trial.gif)

- 3回目は、最上面の前端 x=4.055 m でロール -0.801 rad、遊脚 BR の転倒である
- 最大 x は 4.126 m である
- NMPC失敗は741回である

![段端後退0.30 m/sの3回目](../../artifacts/logs/quadsdk_step23_inset_v030_r03/trial.gif)

- 4回目は、下り x=5.545 m でロール -0.804 rad、遊脚 BL の転倒である
- 最大 x は 6.155 m である
- NMPC失敗は703回である

![段端後退0.30 m/sの4回目](../../artifacts/logs/quadsdk_step23_inset_v030_r04/trial.gif)

- 5回目は、上り x=3.542 m でピッチ -0.802 rad、遊脚 BR の転倒である
- 最大 x は 3.711 m である
- NMPC失敗は761回である

![段端後退0.30 m/sの5回目](../../artifacts/logs/quadsdk_step23_inset_v030_r05/trial.gif)

- 第5章の 0.10 m/s は完走0回、下り到達2回である
- 0.20 m/s の下り到達は1回、0.30 m/s の下り到達は1回で、完走はどちらも0回である
- 基準試行と比較して
  - 改善無し
- `foothold_edge_inset_mode: enforce` は、速度を上げても採用しない

## 第6章 支持三角形への胴体参照

### 概要

- 第3変更は、`support_triangle_shift_mode` だけを `enforce` にする
- 支持三角形の1回目は、`foothold_edge_inset_mode` の無引用 `off` を bool と読んで `local_planner_node` が死に、最大 x=0.998 m の起動異常である
- 支持三角形の2回目は、接近平地 x=2.697 m でロールが崩れて失敗した

### 詳細

- 第3変更は、`support_triangle_shift_mode` だけを `enforce` にする
  - `enforce` は、接地が3脚のとき、胴体参照の xy をその3脚の重心へ最大 0.05 m 寄せる
  - 接地が3脚でないときは、胴体参照の xy を動かさない
  - 足先支持判定は `shadow`、段端後退は文字列の `off` に戻す
- 支持三角形の1回目は、`foothold_edge_inset_mode` の無引用 `off` を bool と読んで `local_planner_node` が死に、最大 x=0.998 m の起動異常である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `PLAN_STARTUP_S` | `8` | 1回目の 3 s から 8 s に延ばし、計画周期へ入る前に WALK を送ることを避ける |
| `support_triangle_shift_mode` | `enforce` | 1回目にはこの変数が無く、この試行は接地3脚の重心へ胴体参照の xy を最大 0.05 m 寄せる |
| `foothold_edge_inset_mode` | 無引用の `off` | YAML が `off` を bool と読み、文字列パラメータへの代入で `local_planner_node` が終了する |

  - `local_planner_node` は `Loaded Pinocchio` のあと、`InvalidParameterTypeException` で終了している
  - 終了時の最大 x は 0.998 m、NMPC失敗は0回である
  - この1回は階段上の転倒ではなく、計画器が周期へ入る前の起動異常である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh tri01 1
```

![支持三角形の1回目](../../artifacts/logs/quadsdk_step23_tri01_r01/trial.gif)

- 支持三角形の2回目は、接近平地でロールが崩れて失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `foothold_edge_inset_mode` | 文字列の `"off"` | 1回目の型異常を除き、支持三角形への胴体参照だけを実行する |

  - `local_planner_node` は `Loaded Pinocchio` と `LocalPlanner took` を記録し、計画周期へ入っている
  - 転倒開始は時刻 47.55 s、x=2.697 m、y=0.095 m、z=0.277 m、ロール -0.807 rad、ピッチ 0.118 rad、ヨー 0.594 rad である
  - 最初の蹴上は x=3.00 m なので、この転倒は接近平地で発生している
  - 最前進は x=2.745 m で、階段には到達していない
  - 終了姿勢は x=2.523 m、y=0.740 m、z=0.058 m、ロール -3.142 rad、ヨー -2.141 rad である
  - NMPC失敗は542回で、転倒開始後の継続である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 bash scripts/trial/run_quadsdk_stair_robustness.sh tri02 1
```

![支持三角形の2回目](../../artifacts/logs/quadsdk_step23_tri02_r01/trial.gif)

- 支持三角形への胴体参照は、階段へ着く前の平地歩行を崩したので、`support_triangle_shift_mode` を文字列の `"off"` に戻した

### 速度変更

#### 概要

- 0.20 m/s の1回は、上り x=3.214 m のピッチ転倒で、最上面には届いていない
- 0.30 m/s の1回は、上り x=3.412 m のピッチ転倒で、最上面には届いていない

#### 詳細

- 0.20 m/s の1回は、上り x=3.214 m のピッチ転倒で、最上面には届いていない

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.20` | 支持三角形 `enforce` の有効試行と同じ1回を、速い指令で見る |
| `support_triangle_shift_mode` | `enforce` | 3脚の重心方向へ胴体参照を最大 0.05 m 動かす |
| `foothold_edge_inset_mode` | `"off"` | 段端後退を混ぜず、支持三角形だけを見る |
| `foothold_support_check_mode` | `shadow` | 支持判定の拒否は戻したままにする |
| `PLAN_STARTUP_S` | `8` | 計画周期へ入ってから WALK を送る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.20 bash scripts/trial/run_quadsdk_stair_robustness.sh tri_v020 1
```

- 転倒開始は x=3.214 m、ピッチ -0.805 rad、遊脚 BR である
- 最大 x は 3.322 m で、最上面 x=3.90 m には届いていない
- NMPC失敗は711回である

![支持三角形0.20 m/s](../../artifacts/logs/quadsdk_step23_tri_v020_r01/trial.gif)

- 0.30 m/s の1回は、上り x=3.412 m のピッチ転倒で、最上面には届いていない

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.30` | 支持三角形のまま、0.20 m/s より速い指令を1回見る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 bash scripts/trial/run_quadsdk_stair_robustness.sh tri_v030 1
```

- 転倒開始は x=3.412 m、ピッチ -0.805 rad、遊脚 FR である
- 最大 x は 3.590 m で、最上面には届いていない
- NMPC失敗は242回である

![支持三角形0.30 m/s](../../artifacts/logs/quadsdk_step23_tri_v030_r01/trial.gif)

- 第6章の 0.10 m/s は接近平地 x=2.697 m のロール転倒である
- 0.20 m/s と 0.30 m/s は最初の段には入ったが、完走は0回である
- 基準試行と比較して
  - 改善無し
- `support_triangle_shift_mode: enforce` は、速度を上げても採用しない

## 第7章 全試行と失敗モードの対応

### 概要

- この章は、Step 23の全27試行を1試行ずつ分け、設定、到達、失敗モード、転倒時の遊脚を対応させる
- 表の遊脚は、転倒開始時に `contact_*` が0だった脚を示す
- 表の最大 x は転倒後の移動を含むので、転倒開始 x と分けて読む
- 表は確認できた結果だけを示し、次策を決めない

### 詳細

- 設定群は、各試行で有効だった変更を示す
  - 基準群は、`foothold_support_check_mode: shadow` を使い、着地点を支持判定で拒否しない
  - 足先支持群は、`foothold_support_check_mode: enforce` だけを基準から変える
  - 段端後退群は、足先支持を `shadow` に戻し、`foothold_edge_inset_mode: enforce` だけを加える
  - 支持三角形群は、段端後退を `"off"` に戻し、`support_triangle_shift_mode: enforce` だけを加える
  - 採用候補確認群は、個別試験で残した設定を組み合わせ、試行前のFast DDS共有メモリ削除を毎回実行する

#### 基準群

| 試行 | 1回目から変えた変数 | 最大 x | 失敗位置 | 失敗モード | 転倒時の遊脚 | 試行ごとの分析 |
| --- | --- | ---: | --- | --- | --- | --- |
| `baseline_shadow_r01` | 無い | 4.061 m | 上り最終段 x=3.851 m | ロール 0.801 rad | FL | 最上面前端へ到達したあとFL遊脚中に左側へ崩れ、上り最終段へ後退した |
| `baseline_shadow_r02` | 無い | 0.998 m | 初期位置 | 起動異常 | 無い | `local_planner_node` が計画周期へ入らず、階段上の失敗を評価できない |
| `baseline_shadow_r03` | 無い | 0.998 m | 初期位置 | 起動異常 | 無い | `local_planner_node` が計画周期へ入らず、階段上の失敗を評価できない |
| `baseline_shadow_r04` | 無い | 0.998 m | 初期位置 | 起動異常 | 無い | `local_planner_node` が計画周期へ入らず、階段上の失敗を評価できない |
| `baseline_shadow_r05` | 無い | 0.998 m | 初期位置 | 起動異常 | 無い | `local_planner_node` が計画周期へ入らず、階段上の失敗を評価できない |
| `shadow06_r01` | `PLAN_STARTUP_S: 3` から `8` | 0.998 m | 初期位置 | 起動異常 | 無い | 待ち時間を延ばしても計画周期へ入らず、共有メモリ残留の影響を分離できる |
| `shadow07_r01` | `PLAN_STARTUP_S: 3` から `8` | 7.023 m | 下り2段目 x=5.366 m | ピッチ 0.801 rad | FL | 転倒後にxが伸びた試行で、転倒前の到達は下り2段目までである |
| `shadow08_r01` | `PLAN_STARTUP_S: 3` から `8` | 6.460 m | 下り2段目 x=5.373 m | ピッチ 0.802 rad | FR | `shadow07` と0.007 m差の位置で反対側の前脚が遊脚になり、同じ段で前後に崩れた |
| `shadow09_r01` | `PLAN_STARTUP_S: 3` から `8` | 7.574 m | 退出平地 | 転倒なし、終了 y=1.152 m | 無い | 下り2段目から増えたヨーを残したまま直立で退出し、横ずれ条件だけを外した |
| `shadow10_r01` | `PLAN_STARTUP_S: 3` から `8` | 4.063 m | 上り最終段 x=3.732 m | ピッチ -0.806 rad | BR | 最上面前端へ到達したあとBR遊脚中に後退し、上り最終段で前後に崩れた |

#### 足先支持群

| 試行 | 1回目から変えた変数 | 最大 x | 失敗位置 | 失敗モード | 転倒時の遊脚 | 試行ごとの分析 |
| --- | --- | ---: | --- | --- | --- | --- |
| `enforce01_r01` | `PLAN_STARTUP_S: 8`、`foothold_support_check_mode: enforce` | 4.106 m | 最上面 x=4.021 m | ロール 0.806 rad | BL | 最上面前端でBL遊脚中に横へ崩れ、基準1回目と近い位置で失敗した |
| `enforce02_r01` | `PLAN_STARTUP_S: 8`、`foothold_support_check_mode: enforce` | 3.136 m | 接近平地 x=2.794 m | ロール 0.805 rad | FL | 階段前から y=0.852 m、ヨー -0.956 rad までずれ、最上段とは別の失敗になった |
| `enforce03_r01` | `PLAN_STARTUP_S: 8`、`foothold_support_check_mode: enforce` | 4.068 m | 上り最終段 x=3.861 m | ピッチ -0.801 rad | BL | 最上面前端へ到達したあとBL遊脚中に後退し、上り最終段で前後に崩れた |
| `enforce04_r01` | `PLAN_STARTUP_S: 8`、`foothold_support_check_mode: enforce` | 4.060 m | 上り最終段 x=3.822 m | ピッチ -0.806 rad | FR | `enforce03` と近い位置だがFR遊脚中に崩れ、後ろ脚だけの失敗ではない |
| `enforce05_r01` | `PLAN_STARTUP_S: 8`、`foothold_support_check_mode: enforce` | 3.791 m | 上り最終段 x=3.611 m | ピッチ -0.801 rad | BR | 最上面へ届く前にBR遊脚中に前後へ崩れ、足先支持群で最短の到達になった |

#### 段端後退群

| 試行 | 1回目から変えた変数 | 最大 x | 失敗位置 | 失敗モード | 転倒時の遊脚 | 試行ごとの分析 |
| --- | --- | ---: | --- | --- | --- | --- |
| `inset01_r01` | `PLAN_STARTUP_S: 8`、`foothold_edge_inset_mode: enforce` | 4.060 m | 最上面 x=4.019 m | ロール 0.800 rad | FL | 最上面前端でFL遊脚中に横へ崩れ、基準1回目と同じ遊脚と位置で失敗した |
| `inset02_r01` | `PLAN_STARTUP_S: 8`、`foothold_edge_inset_mode: enforce` | 4.087 m | 上り最終段 x=3.848 m | ピッチ -0.802 rad | FR | 最上面前端へ到達したあとFR遊脚中に後退し、前脚遊脚でも前後に崩れた |
| `inset03_r01` | `PLAN_STARTUP_S: 8`、`foothold_edge_inset_mode: enforce` | 5.826 m | 下り1段目 x=5.073 m | ロール 0.802 rad | BL | 最上面でヨーが増え、y=1.085 m、ヨー1.377 radの状態で下り1段目へ入って横に崩れた |
| `inset04_r01` | `PLAN_STARTUP_S: 8`、`foothold_edge_inset_mode: enforce` | 6.522 m | 下り2段目 x=5.324 m | ロール 0.802 rad | FL | ヨー2.000 radまで回った状態で下り2段目へ入り、転倒後の移動で最大xが伸びた |
| `inset05_r01` | `PLAN_STARTUP_S: 8`、`foothold_edge_inset_mode: enforce` | 4.039 m | 上り最終段 x=3.873 m | ロール 0.801 rad | BR | 最上面前端へ到達したあとBR遊脚中に横へ崩れ、後ろ脚遊脚の失敗に該当する |

#### 支持三角形群

| 試行 | 1回目から変えた変数 | 最大 x | 失敗位置 | 失敗モード | 転倒時の遊脚 | 試行ごとの分析 |
| --- | --- | ---: | --- | --- | --- | --- |
| `tri01_r01` | `PLAN_STARTUP_S: 8`、`support_triangle_shift_mode: enforce`、無引用の `foothold_edge_inset_mode: off` | 0.998 m | 初期位置 | 型異常による起動失敗 | 無い | YAMLの `off` をboolと読み、`local_planner_node` が計画周期前に終了した |
| `tri02_r01` | `PLAN_STARTUP_S: 8`、`support_triangle_shift_mode: enforce`、`foothold_edge_inset_mode: "off"` | 2.745 m | 接近平地 x=2.697 m | ロール -0.807 rad | FL | 型異常を直したあとも支持三角形への胴体参照が接近平地を崩し、階段へ到達しなかった |

#### 採用候補確認群

| 試行 | 1回目から変えた変数 | 最大 x | 失敗位置 | 失敗モード | 転倒時の遊脚 | 試行ごとの分析 |
| --- | --- | ---: | --- | --- | --- | --- |
| `adopted_final_r01` | 無い | 4.064 m | 上り最終段 x=3.873 m | ロール -0.807 rad | BL | 最上面前端へ到達したあとBL遊脚中に後退し、上り最終段で横へ崩れた |
| `adopted_final_r02` | 無い | 3.234 m | 最初の蹴上付近 x=2.970 m | ピッチ -0.801 rad | BL | 1段目へ進んだあとBL遊脚中に後退し、最初の蹴上の手前で前後に崩れた |
| `adopted_final_r03` | 無い | 4.065 m | 上り最終段 x=3.863 m | ロール 0.801 rad | FL | 最上面前端へ到達したあとFL遊脚中に後退し、上り最終段で横へ崩れた |
| `adopted_final_r04` | 無い | 3.156 m | 最初の蹴上付近 x=2.885 m | ピッチ -0.800 rad | BR | 1段目へ進んだあとBR遊脚中に後退し、最初の蹴上の手前で前後に崩れた |
| `adopted_final_r05` | 無い | 3.172 m | 最初の蹴上付近 x=2.868 m | ピッチ -0.800 rad | FL | 1段目付近まで進んだあとFL遊脚中に後退し、最初の蹴上の手前で前後に崩れた |

### 全試行から確認した事実

- 起動異常6回は、階段上の運動失敗から分ける
- 接近平地の転倒2回は、最上段の足運びとは別の失敗である
- 最初の蹴上付近の転倒3回は、採用候補確認群で新たに記録した
- 上り最上段付近の11回は、FL遊脚3回、FR遊脚2回、BL遊脚3回、BR遊脚3回に分かれる
- 下りの4回は、下り1段目1回と下り2段目3回に分かれ、ピッチ転倒2回とロール転倒2回を含む
- `shadow09_r01` は転倒せず、退出後の横ずれだけで失敗する
- 後ろ足の失敗は複数の試行にあるが、前脚遊脚中にも同じ上り最終段付近で失敗している
- 現在のCSVは足先実位置と計画着地点を持たないので、踏み外した脚と段端からの距離は未確認である
- この章は各試行の結果を対応させるだけで、次策を決めない

## 第8章 失敗モードと対策効果の対応

### 概要

- 起動異常は、Fast DDS共有メモリを削除すると歩行を再開した
- 最初の蹴上までの到達は、平滑法線半径を 0.40 m から 0.10 m へ減らすと最大 x が 2.77 m から 3.07 m へ伸びた
- 振り足頂点の地形対応は、頂点を地形より上へ上げたが、最初の蹴上の転倒を解消しなかった
- 足先支持の強制、段端後退、支持三角形への胴体参照は、上り最上段、下り、横ずれの失敗率を改善しなかった
- 段端後退は、下り2段目のピッチ転倒をロール転倒へ置き換えたが、下りの転倒自体を解消しなかった
- この章は確認済みの改善と未改善を分け、未試験の次策を含めない

### 詳細

| 失敗モード | 対策前 | 失敗原因と対策の考え方 | 対策後 | 以前より改善 |
| --- | --- | --- | --- | --- |
| 計画周期へ入らない起動異常 | `baseline_shadow_r02` から `r05` と `shadow06_r01` の5回が最大 x=0.998 mで停止した | - 失敗原因：残留したFast DDS共有メモリが `local_planner_node` の初期化を止めたと想定する<br>- 対策の考え方：起動前に `/dev/shm/fastrtps_*` を削除すればDDSの起動状態を初期化でき、以前の通信状態を再利用せずに計画周期へ入れると考える | `shadow07_r01` から `shadow10_r01` の4回はすべて歩行を開始した | 改善あり 起動異常5回から歩行開始4回へ変わった |
| 最初の蹴上まで届かない失敗 | Step 19の変更前は最大 x=2.77 mで転倒した | - 失敗原因：0.40 mの平滑法線が蹴上と踏面の境界を広い範囲へぼかしたと想定する<br>- 対策の考え方：平滑法線半径を 0.10 m へ縮めれば蹴上近傍の法線を局所的に保てるため、段へ着く前から胴体参照が大きく傾く状態を避けられると考える | Step 19の変更後は最大 x=3.07 mまで伸びたが転倒した | 改善あり 到達が約 0.30 m伸びたが転倒は残った |
| 振り足が途中地形へ近づく問題 | 端点だけで振り足頂点を決めた | - 失敗原因：端点だけの頂点計算が振り足経路の途中にある高い地形を見落としたと想定する<br>- 対策の考え方：`swing_terrain_check_mode: enforce` が経路上の最大地形高さへクリアランスを加えれば、足先を途中地形より上へ運び、接触を避けられると考える | 途中地形に応じて振り足頂点が上がったが、最初の蹴上で転倒した | 改善あり 振り足頂点は改善したが階段転倒は残った |
| 足先全体を支持できない候補 | 基準の有効5回では3回が下りへ到達し、そのうち1回が退出平地まで直立した | - 失敗原因：足先中心は踏面上でも足先円の一部が段端から外れる着地点を選んだと想定する<br>- 対策の考え方：`foothold_support_check_mode: enforce` が足先円全体を支持できない候補を拒否すれば、足先全体が同じ高さへ載る着地点だけを使えると考える | 足先支持群5回は下り到達0回、接近平地転倒1回になった | 改善なし 到達と安定性が悪化した |
| 上り最上段付近の転倒 | 基準の有効5回では上り最上段付近の転倒が2回である | - 失敗原因：上り最終段または最上面の段端に近すぎる着地点を選んだと想定する<br>- 対策の考え方：`foothold_edge_inset_mode: enforce` が着地点を段端から `toe_radius` だけ内側へ移せば、足先が段端をまたぐ状態を避けられると考える | 段端後退群5回では上り最上段付近の転倒が3回である | 改善なし 転倒回数が2回から3回へ増えた |
| 下り2段目のピッチ転倒 | 基準群は下り2段目のピッチ転倒を2回記録した | - 失敗原因：下りの段端に近い着地点が前後方向の支持余裕を減らしたと想定する<br>- 対策の考え方：`foothold_edge_inset_mode: enforce` が下りでも足先を段端から内側へ移せば、前後方向の支持余裕を確保してピッチ転倒を避けられると考える | 下りのピッチ転倒は0回になったが、下りのロール転倒を2回記録した | 改善なし ピッチ失敗をロール失敗へ置き換えただけである |
| 下りまでの到達 | 基準の有効5回では3回が下りまたは退出平地へ到達した | - 失敗原因：段端に近い着地点が上りから下りへ移る前の支持を不安定にしたと想定する<br>- 対策の考え方：`foothold_edge_inset_mode: enforce` が各踏面の内側へ足先を置けば、最上面までの支持を保ち、下りへ移れる試行を増やせると考える | 段端後退群5回では下り到達2回、退出平地の直立歩行0回になった | 改善なし 下り到達が3回から2回へ減った |
| ヨー増加と横ずれ | `shadow09_r01` は終了 y=1.152 m、ヨー0.686 radで直立した | - 失敗原因：左右脚が段端から異なる距離へ着地し、左右非対称な支持がヨーモーメントを生んだと想定する<br>- 対策の考え方：`foothold_edge_inset_mode: enforce` が左右脚をそれぞれ段端から同じ足先半径だけ内側へ置けば、左右の支持条件を揃えてヨー増加を抑えられると考える | `inset03_r01` はヨー1.377 rad、`inset04_r01` はヨー2.000 radで転倒した | 改善なし ヨーが増えて転倒を加えた |
| 3脚支持中の胴体位置 | 基準の有効5回に接近平地転倒は無い | - 失敗原因：1脚を振り上げたときに胴体参照が残り3脚の支持三角形の中心から離れたと想定する<br>- 対策の考え方：`support_triangle_shift_mode: enforce` が胴体参照を接地3脚の重心へ最大 0.05 m 寄せれば、重心投影と支持三角形の境界との余裕を増やして転倒を避けられると考える | 型異常を除いた `tri02_r01` は接近平地 x=2.697 mでロール転倒した | 改善なし 階段到達前の平地安定性が悪化した |

- 基準試行と比較して
  - 改善有り
    - Fast DDS共有メモリ削除は、起動異常5回のあとに4回連続で歩行を再開した
  - 改善無し
    - `foothold_support_check_mode: enforce` は、基準の下り到達3回を0回へ減らした
    - `foothold_edge_inset_mode: enforce` は、基準の下り到達3回を2回へ減らし、上り最上段付近の転倒を2回から3回へ増やした
    - `foothold_edge_inset_mode: enforce` は、下りのピッチ転倒2回をロール転倒2回へ置き換えただけである
    - `support_triangle_shift_mode: enforce` は、基準では無かった接近平地転倒を発生させた
- 基準試行より前の試行と比較して
  - 改善有り
    - 平滑法線半径 0.10 mは、変更前より最初の蹴上までの最大 x を約 0.30 m伸ばした
    - `swing_terrain_check_mode: enforce` は、変更前より振り足頂点を途中地形に応じて上げた

### 実施済み試行に基づく確認試験への採用判断

- 確認試験へ採用する対策
  - 試行前のFast DDS共有メモリ削除は、歩行開始を4回連続で回復したため確認試験へ採用する
  - 平滑法線半径 0.10 mは、転倒を解消しないが最初の蹴上までの到達を約 0.30 m伸ばしたため確認試験へ採用する
  - `swing_terrain_check_mode: enforce` は、階段転倒を解消しないが途中地形を考慮した振り足頂点を実現したため確認試験へ採用する
  - `foothold_support_check_mode: shadow` は、`enforce` より下り到達を維持したため確認試験へ採用する
  - `foothold_edge_inset_mode: "off"` は、`enforce` が成功数と下り到達数を増やさなかったため確認試験へ採用する
  - `support_triangle_shift_mode: "off"` は、`enforce` が接近平地を崩したため確認試験へ採用する
- 採用すべきでない対策
  - 現在の `foothold_support_check_mode: enforce` は、下り到達を3回から0回へ減らしたため採用しない
  - 現在の `foothold_edge_inset_mode: enforce` は、上り最上段付近と下りの失敗を改善しなかったため採用しない
  - 現在の `support_triangle_shift_mode: enforce` は、階段へ到達する前の平地歩行を崩したため採用しない

## 第9章 採用候補を組み合わせた確認試験

### 概要

- 確認試験は、第8章で採用候補にした設定を同時に使い、同一条件を5回実行する
- 5回とも歩行を開始したので、Fast DDS共有メモリ削除は起動異常の再発を防いだ
- 5回とも完走せず、基準の有効5回にあった下り到達3回を再現しなかった
- 2回は最上面前端まで到達したあと上り最終段へ後退してロール転倒した
- 3回は最初の蹴上付近まで進んだあと後退してピッチ転倒した

### 詳細

- 確認試験は、第8章で採用候補にした設定を同時に使い、同一条件を5回実行する

```bash
PLAN_STARTUP_S=8 QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash bash scripts/trial/run_quadsdk_stair_robustness.sh adopted_final 5
```

#### 確認試験の1回目

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| 試行 | `adopted_final_r01` | 個別試験で残した対策を組み合わせ、階段全体の成功へつながるか確認する |
| `GAP_WORLD` | `jp_stair_matrix_h15_d30_n04.xml` | 高さ 0.15 m、踏面奥行き 0.30 m、4段上り下り、最上面奥行き 1.00 mの基準地形を使う |
| `FORWARD_VEL_MPS` | `0.10` | 速度を比較要因にせず、基準試験と同じクロール指令を使う |
| `local_footstep_planner.period` | `0.9` | 基準試験と同じ歩行周期を使う |
| `local_footstep_planner.duty_cycles` | `[0.75, 0.75, 0.75, 0.75]` | 基準試験と同じ3脚以上の接地比を使う |
| `local_footstep_planner.phase_offsets` | `[0.0, 0.75, 0.5, 0.25]` | 基準試験と同じ脚順を使う |
| `DURATION_S` | `90` | 退出平地の先まで前進指令を続ける |
| `HOLD_S` | `5` | 指令後の終了姿勢を記録する |
| `PLAN_STARTUP_S` | `8` | 起動待ちを基準の追加試行と同じ長さにする |
| Fast DDS共有メモリ | 試行前に削除 | 残留通信状態による起動異常を比較から除く |
| `smooth_surface_normals.radius` | `0.10` | Step 19で最初の蹴上までの到達を伸ばした値を維持する |
| `foothold_support_check_mode` | `shadow` | `enforce` が下り到達を減らしたため、支持可否は記録だけにする |
| `foothold_edge_inset_mode` | `"off"` | `enforce` が上りと下りの失敗を改善しなかったため無効にする |
| `support_triangle_shift_mode` | `"off"` | `enforce` が接近平地を崩したため無効にする |
| `foothold_ik_check_mode` | `shadow` | 足先IK判定を比較要因にせず、基準設定を維持する |
| `swing_terrain_check_mode` | `enforce` | Step 20で確認した途中地形に応じた振り足頂点を維持する |
| `stair_tread_snap_max_run` | `0.0` | 踏面中央化を無効にし、未試験の処理を加えない |
| `foothold_support_margin` | `0.0` | 足先支持判定の余裕を加えない |
| `multistep_planner.enabled` | `true` | 基準試験と同じ先読みを使う |
| `multistep_planner.apply_foothold` | `false` | 先読み着地点を制御へ適用しない基準設定を維持する |

| 結果項目 | 値 | 判定 |
| --- | ---: | --- |
| 最大 x | 4.064 m | 最上面前端まで到達した |
| 転倒開始 | x=3.873 m、ロール -0.807 rad | 上り最終段へ後退してロール転倒した |
| 転倒時の遊脚 | BL | 後ろ脚遊脚中の失敗である |
| NMPC失敗 | 557回 | 転倒後の継続を含む |
| 完走判定 | 失敗 | 下りへ到達していない |

![採用候補確認の1回目](../../artifacts/logs/quadsdk_step23_adopted_final_r01/trial.gif)

#### 確認試験の2回目

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| 1回目から変更した変数 | 無い | 反復差だけを確認する |

| 結果項目 | 値 | 判定 |
| --- | ---: | --- |
| 最大 x | 3.234 m | 1段目まで進んだ |
| 転倒開始 | x=2.970 m、ピッチ -0.801 rad | 最初の蹴上付近へ後退してピッチ転倒した |
| 転倒時の遊脚 | BL | 後ろ脚遊脚中の失敗である |
| NMPC失敗 | 602回 | 転倒後の継続を含む |
| 完走判定 | 失敗 | 上りを継続できていない |

![採用候補確認の2回目](../../artifacts/logs/quadsdk_step23_adopted_final_r02/trial.gif)

#### 確認試験の3回目

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| 1回目から変更した変数 | 無い | 反復差だけを確認する |

| 結果項目 | 値 | 判定 |
| --- | ---: | --- |
| 最大 x | 4.065 m | 最上面前端まで到達した |
| 転倒開始 | x=3.863 m、ロール 0.801 rad | 上り最終段へ後退してロール転倒した |
| 転倒時の遊脚 | FL | 前脚遊脚中にも同じ位置で失敗した |
| NMPC失敗 | 561回 | 転倒後の継続を含む |
| 完走判定 | 失敗 | 下りへ到達していない |

![採用候補確認の3回目](../../artifacts/logs/quadsdk_step23_adopted_final_r03/trial.gif)

#### 確認試験の4回目

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| 1回目から変更した変数 | 無い | 反復差だけを確認する |

| 結果項目 | 値 | 判定 |
| --- | ---: | --- |
| 最大 x | 3.156 m | 1段目付近まで進んだ |
| 転倒開始 | x=2.885 m、ピッチ -0.800 rad | 最初の蹴上付近へ後退してピッチ転倒した |
| 転倒時の遊脚 | BR | 後ろ脚遊脚中の失敗である |
| NMPC失敗 | 637回 | 転倒後の継続を含む |
| 完走判定 | 失敗 | 上りを継続できていない |

![採用候補確認の4回目](../../artifacts/logs/quadsdk_step23_adopted_final_r04/trial.gif)

#### 確認試験の5回目

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| 1回目から変更した変数 | 無い | 反復差だけを確認する |

| 結果項目 | 値 | 判定 |
| --- | ---: | --- |
| 最大 x | 3.172 m | 1段目付近まで進んだ |
| 転倒開始 | x=2.868 m、ピッチ -0.800 rad | 最初の蹴上付近へ後退してピッチ転倒した |
| 転倒時の遊脚 | FL | 前脚遊脚中にも同じ位置で失敗した |
| NMPC失敗 | 544回 | 転倒後の継続を含む |
| 完走判定 | 失敗 | 上りを継続できていない |

![採用候補確認の5回目](../../artifacts/logs/quadsdk_step23_adopted_final_r05/trial.gif)

### 基準試行との比較

| 比較項目 | 基準の有効5回 | 採用候補の確認5回 | 以前より改善 |
| --- | ---: | ---: | --- |
| 歩行開始 | 5回 | 5回 | 変化なし |
| 完走 | 0回 | 0回 | 改善なし |
| 下りまたは退出平地への到達 | 3回 | 0回 | 改善なし |
| 上り最上段付近の転倒 | 2回 | 2回 | 変化なし |
| 最初の蹴上付近の転倒 | 0回 | 3回 | 悪化 |

- 採用候補の確認5回は、基準試行より成功数を増やしていない
- 採用候補の確認5回は、基準試行より下り到達を3回から0回へ減らした
- 個別試験で確認した局所効果は、組み合わせた設定の階段全体のロバスト性を保証しない

### 速度変更

#### 概要

- 0.20 m/s は起動異常1回と失敗4回で、完走は0回、下り到達は0回である
- 0.30 m/s は起動異常1回のあと、歩行した4回がすべて下りで転倒し、完走は0回である

#### 詳細

- 0.20 m/s は起動異常1回と失敗4回で、完走は0回、下り到達は0回である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.20` | 第9章の確認設定のまま、速い指令の到達を見る |
| `foothold_support_check_mode` | `shadow` | 確認試験と同じく、支持不足は記録するだけで拒否しない |
| `foothold_edge_inset_mode` | `"off"` | 段端後退は使わない |
| `support_triangle_shift_mode` | `"off"` | 支持三角形への胴体移動は使わない |
| `swing_terrain_check_mode` | `enforce` | 振り足の頂点を中間地形より上へ上げる設定を固定する |
| `PLAN_STARTUP_S` | `8` | 計画周期へ入ってから WALK を送る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.20 bash scripts/trial/run_quadsdk_stair_robustness.sh adopted_v020 1
```

- 起動異常の1回は、最大 x=0.998 m、終了 z=0.310 m のまま直立している
- NMPC失敗は0回である

![確認設定0.20 m/sの起動異常](../../artifacts/logs/quadsdk_step23_adopted_v020_r01/trial.gif)

- 歩行した1回目は、上り x=3.146 m でロール -0.806 rad、遊脚 FR の転倒である
- 最大 x は 3.616 m である
- NMPC失敗は691回である

![確認設定0.20 m/sの1回目](../../artifacts/logs/quadsdk_step23_adopted_v020_more_r01/trial.gif)

- 2回目は、上り x=3.271 m でピッチ -0.800 rad、遊脚 BL の転倒である
- 最大 x は 3.631 m である
- NMPC失敗は679回である

![確認設定0.20 m/sの2回目](../../artifacts/logs/quadsdk_step23_adopted_v020_more_r02/trial.gif)

- 3回目は、最上面の前端 x=4.027 m でロール 0.809 rad、遊脚 FL の転倒である
- 最大 x は 4.077 m である
- NMPC失敗は675回である

![確認設定0.20 m/sの3回目](../../artifacts/logs/quadsdk_step23_adopted_v020_more_r03/trial.gif)

- 4回目は、最上面へ x=4.073 m まで乗ったあと、x=3.734 m でピッチ -0.801 rad、遊脚 BR の転倒である
- NMPC失敗は701回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.20 bash scripts/trial/run_quadsdk_stair_robustness.sh adopted_v020_more 4
```

![確認設定0.20 m/sの4回目](../../artifacts/logs/quadsdk_step23_adopted_v020_more_r04/trial.gif)

- 0.30 m/s は起動異常1回のあと、歩行した4回がすべて下りで転倒し、完走は0回である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.30` | 確認設定のまま、0.20 m/s より速い指令の到達を見る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 bash scripts/trial/run_quadsdk_stair_robustness.sh adopted_v030 1
```

- 起動異常の1回は、最大 x=1.022 m、終了ロール -3.142 rad である
- NMPC失敗は21回である

![確認設定0.30 m/sの起動異常](../../artifacts/logs/quadsdk_step23_adopted_v030_r01/trial.gif)

- 歩行した1回目は、下り x=5.453 m でピッチ 0.800 rad、遊脚 FR の転倒である
- 最大 x は 6.579 m である
- NMPC失敗は641回である

![確認設定0.30 m/sの1回目](../../artifacts/logs/quadsdk_step23_adopted_v030_more_r01/trial.gif)

- 2回目は、下り x=5.461 m でピッチ 0.800 rad、遊脚 FR の転倒である
- 最大 x は 7.374 m、終了ロールは -2.665 rad で失敗である
- NMPC失敗は594回である

![確認設定0.30 m/sの2回目](../../artifacts/logs/quadsdk_step23_adopted_v030_more_r02/trial.gif)

- 3回目は、下り x=5.465 m でピッチ 0.800 rad、遊脚 FL の転倒である
- 最大 x は 6.486 m である
- NMPC失敗は687回である

![確認設定0.30 m/sの3回目](../../artifacts/logs/quadsdk_step23_adopted_v030_more_r03/trial.gif)

- 4回目は、下り x=5.581 m でロール -0.803 rad、遊脚 BL の転倒である
- 最大 x は 6.494 m である
- NMPC失敗は682回である

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 bash scripts/trial/run_quadsdk_stair_robustness.sh adopted_v030_more 4
```

![確認設定0.30 m/sの4回目](../../artifacts/logs/quadsdk_step23_adopted_v030_more_r04/trial.gif)

- 第9章の 0.10 m/s は完走0回、下り到達0回である
- 0.30 m/s は歩行した4回が下りまで届いたが、完走は0回である
- 完走回数が同数の 0 なので、その後の頭下げと前足の着地は、最上段と下りへ届く回数が多かった 0.30 m/s で行う
- 基準試行と比較して
  - 改善無し
- 速度を上げた確認設定は、階段の成功設定として採用しない

## 第10章 再現性とロバスト性を失う理由

### 概要

- 基準の有効5回と採用候補の確認5回は、階段歩行へ作用する設定が同じなので、確認群の悪化を新しい設定変更の因果とは判定しない
- 現設定は、階段への到達脚位相を地形へ同期せず、足先支持の不足を検出しても着地点を変えず、複数歩の着地点列も制御へ適用しない
- 同じ設定でも階段到達時の遊脚と姿勢が変わり、支持余裕を制御へ反映しないため、その小さい差が最初の蹴上、最上段、下りの異なる失敗へ拡大する
- 計画処理時間は基準群と確認群で近く、全確認試行に共通する計画停止も無いので、計算遅延を共通原因とは判定しない
- 現ログは足先実位置、計画着地点、段端距離、NMPC失敗時刻を同じCSVへ記録しないため、最後に支持を失った接触現象までは確定できない

### 詳細

#### 基準群と確認群の設定差

| 比較項目 | 基準の有効5回 | 採用候補の確認5回 | 分析 |
| --- | --- | --- | --- |
| `foothold_support_check_mode` | `shadow` | `shadow` | 足先支持不足を記録するだけで着地点を拒否しない |
| `foothold_edge_inset_mode` | `"off"` | `"off"` | 段端から着地点を離す処理を使わない |
| `support_triangle_shift_mode` | `"off"` | `"off"` | 3脚支持に合わせた胴体参照移動を使わない |
| `swing_terrain_check_mode` | `enforce` | `enforce` | 振り足高さだけを地形へ適用する |
| `multistep_planner.apply_foothold` | `false` | `false` | 複数歩の着地点列を名目着地点へ適用しない |
| `stair_tread_snap_max_run` | `0.0` | `0.0` | 踏面中央へ着地点を移さない |
| Fast DDS共有メモリ削除 | 有効基準の後半4回で実施 | 5回で実施 | 起動異常を防ぐが階段上の制御則を変えない |

- 基準群と確認群の階段制御は同じなので、下り到達が3回から0回へ減った事実は、設定変更による平均性能低下ではなく、同じ制御の試行間変動を示す
- Fast DDS共有メモリ削除と `PLAN_STARTUP_S: 8` は歩行開始を安定させるが、階段上の支持余裕を増やさない

#### 階段到達時の脚位相が固定されない

| 試行群 | x=3.00 mへ初めて到達したときの遊脚 | 確認できる差 |
| --- | --- | --- |
| 基準の有効5回 | BLが4回、BRが1回 | 1種類へ偏るが完全には固定されない |
| 採用候補の確認5回 | BLが2回、BRが2回、FRが1回 | 同じ設定でも3種類へ分かれる |

- 接地スケジュールは `current_plan_index % period` で決まり、`current_plan_index` はプランナ開始時刻から進む
- 接地スケジュールは階段の蹴上位置へ同期しないので、起動と前進のわずかな時間差が、蹴上へ到達するときの遊脚を変える
- 確認2回目は x=2.80 mでヨー -0.322 radまでずれた一方、基準5回の同位置でのヨー絶対値は最大 0.045 radである
- 確認4回目は x=3.00 mで y=0.002 m、ロール -0.010 rad、ピッチ -0.445 rad、ヨー0.014 radであり、基準群と近い姿勢でも直後に最初の蹴上付近で転倒した
- 遊脚と姿勢の差だけで結果が一意に決まらないことは、現制御の安定余裕が小さく、記録していない足先接触差も結果へ影響することを示す

#### 支持不足を検出しても制御へ反映しない

| 試行 | 転倒前に記録した `support=0` | 転倒前の該当候補 x 範囲 |
| --- | ---: | --- |
| `adopted_final_r01` | 28回 | 2.969 mから3.656 m |
| `adopted_final_r02` | 4回 | 2.897 mから2.949 m |
| `adopted_final_r03` | 36回 | 2.890 mから3.719 m |
| `adopted_final_r04` | 12回 | 2.886 mから3.069 m |
| `adopted_final_r05` | 2回 | 2.882 mから2.887 m |

- `support=0` は、評価した足先円を同じ高さで支持できない候補を示す
- `foothold_support_check_mode: shadow` は `support=0` を記録しても候補を `NO_SUPPORTED_CANDIDATE` に変えない
- 5回すべてが転倒前に `support=0` を記録したので、現設定は支持余裕が不足する候補を使いうる
- ただし `enforce` の個別試験は下り到達を0回へ減らしたので、単純な候補拒否は代替着地点と胴体運動を同時に成立させず、ロバスト性を回復しなかった

#### 階段全体を調整する機能が制御へ入っていない

- `swing_terrain_check_mode: enforce` は振り足頂点の高さを変えるが、着地点 xとy、段端距離、接地後の支持多角形を変えない
- `multistep_planner.enabled: true` でも `apply_foothold: false` なので、先読みした着地点列は名目着地点へ戻らない
- `stair_tread_snap_max_run: 0.0`、`foothold_edge_inset_mode: "off"`、`support_triangle_shift_mode: "off"` なので、踏面中央化、段端余裕、3脚支持中の胴体位置は制御へ入らない
- この構成は振り足の途中接触を減らしても、最初の蹴上、最上段遷移、下りで必要な接地後の支持余裕を作らない

#### 共通原因と判定しない要因

| 要因 | 基準の有効5回 | 採用候補の確認5回 | 判定 |
| --- | ---: | ---: | --- |
| 転倒前の計画処理時間中央値 | 8.65 ms | 8.95 ms | 差が小さく、共通原因とは判定しない |
| 転倒前の計画処理時間95パーセンタイル | 30.76 ms | 34.26 ms | 両群に処理時間の揺れがあり、確認群だけの原因ではない |
| 100 msを超える最大計画処理時間 | 121.80 ms | 120.84 ms | 両群にあり、失敗位置の違いを単独で説明しない |
| NMPC失敗回数 | 転倒後の継続を多く含む | 転倒後の継続を多く含む | 回数だけでは原因と結果を分けられない |

- 確認5回では転倒前の `plan_age_s` が0.5 sを超えていないので、全試行に共通する長時間の計画停止は確認していない
- 関連するプランナと実行スクリプトには階段試験をランダム化する処理を確認していない
- ROS 2のプロセス起動順、トピック到着時刻、MuJoCo接触計算の微小差は固定しておらず、どの差が最初の分岐を作ったかは現ログだけでは未確認である

### 分析結果

- 再現性がない直接の理由は、同じ地形位置へ同じ脚位相で到達する条件を固定していないことである
- ロバスト性がない直接の理由は、位相と姿勢の小さい差を許容する足先支持余裕と複数歩の着地点調整を制御へ適用していないことである
- `support=0` を全確認試行で転倒前に観測した事実は支持余裕不足を示すが、最後に滑った脚や段端距離を確定する証拠ではない
- 現設定は同じ制御則から複数の失敗モードへ分岐するため、基準の下り到達3回も安定した性能ではなく、偶発的に長く進めた結果として扱う

## 第11章 ロバスト性を上げる設計

### 概要

- 最優先は、足先円と安全余裕を踏面から差し引いた支持可能領域を作り、名目着地点をその領域へ投影することである
- 単純な候補拒否は代替着地点を作らず悪化したので、支持可能領域が無いときは別候補、速度低下、接地維持を一つの経路として扱う
- 次に、前脚と後ろ脚を独立に動かさず、次の4着地と胴体位置を複数歩で整合させる
- 最上段では、前脚を最上面の奥へ置いてから胴体を前へ進め、後ろ脚の安全な着地点を確保する系列を候補に含める
- 最初の蹴上、最上段、下りで着地点が成立しない位相では、時間だけで遊脚を進めず、接地を維持して位相と速度を調整する
- 評価は一つの到達位相へ固定して成功を作るのではなく、4種類の到達脚位相を分けて反復し、すべての位相で成功率を確認する

### 詳細

#### 改善策の優先順位

| 優先 | 改善策 | 回避する失敗原因 | 現在の方式との差 | 採否を決める試験 |
| ---: | --- | --- | --- | --- |
| 0 | 足先実位置と計画着地点の記録 | 最後に支持を失った脚と段端距離を判定できない | 胴体姿勢と接触だけでなく、各脚の計画 x y z、実 x y z、支持余裕を同じ時刻で残す | 1試行で全着地を脚別に再構成できることを確認する |
| 1 | 支持可能領域からの着地点生成 | `support=0` の候補を使う | 候補を拒否するだけでなく、安全領域内で名目点に最も近い着地点を作る | 4到達位相で `support=0` の適用着地を0回にし、接近平地を崩さないことを確認する |
| 2 | 着地点が無いときの接地維持と減速 | 代替候補が無いまま遊脚を開始する | 無効候補を使うか停止要求だけを出す代わりに、現在の支持脚を維持して再計画する | 着地点を意図的に狭めても転倒せず、直立停止または再開することを確認する |
| 3 | 4脚分の複数歩着地点を制御へ適用 | 各脚の局所着地が次の脚の支持余裕を減らす | `apply_foothold: false` を前提にせず、連続する着地点と胴体進行を一組で評価する | 最初の蹴上、最上段、下りの各遷移で4脚の系列が成立することを確認する |
| 4 | 支持余裕を使った胴体参照と速度調整 | 3脚支持中に重心投影が支持境界へ近づく | 3脚の単純重心へ一律に寄せず、支持多角形の最小余裕を制約または費用にする | 接近平地を維持し、階段上の最小支持余裕が基準より増えることを確認する |
| 5 | 接触完了に基づく歩容位相更新 | 時刻差で蹴上到達時の遊脚が変わる | `current_plan_index % period` だけで進めず、着地確認と次着地点成立を位相遷移条件にする | 4到達位相のどれでも遊脚開始時に有効着地点があることを確認する |
| 6 | 着地誤差の閉ループ回復 | 計画点と実足位置のずれを次の歩へ持ち越す | 接触後の実足位置を次の支持領域と胴体計画へ戻す | 着地誤差を与えても次の2歩以内に支持余裕を回復することを確認する |

- 優先0は計測追加であり、それ自体は歩行を改善しないが、優先1以降の採否を客観的に決めるために先に必要である
- 優先1と2は一組で実装し、支持候補を消すだけの `foothold_support_check_mode: enforce` を再利用しない
- 優先3は優先1が安全な着地点を生成できたあとに適用し、安全でない着地点列を複数歩へ広げない
- 優先4は優先3の着地点列から支持多角形を作り、接近平地から一律に胴体を動かした既存の `support_triangle_shift_mode: enforce` を再利用しない

#### 支持可能領域から着地点を生成する

- 各踏面の同じ高さで連結した領域を抽出し、その境界を `toe_radius + safety_margin` だけ内側へ縮める
- 縮めた領域を足先中心が置ける支持可能領域とし、螺旋探索で得た名目着地点を領域内の最近点へ投影する
- x方向だけを段端から引くのではなく、xとyの両方で足先円が同じ踏面へ収まることを要求する
- 同じ踏面に複数の領域がある場合は、名目点からの距離、IK余裕、前後左右の段端余裕、次の脚の到達可能性を費用にする
- 支持可能領域が無い場合は、遠い候補へ急に飛ばさず、現在の接地を維持して速度を下げ、次の遊脚候補を再計画する

| 比較項目 | 単純な `support enforce` | 支持可能領域からの生成 |
| --- | --- | --- |
| 支持不足候補 | 拒否する | 安全領域内の最近点へ置き換える |
| 代替点が無い場合 | 探索結果または停止処理へ依存する | 接地維持と減速を明示する |
| 段端余裕 | 支持可否の二値で扱う | 境界までの距離を連続量で最大化する |
| 次の脚との整合 | 評価しない | 複数歩費用で評価する |
| 実施済み試験との関係 | 下り到達を3回から0回へ悪化させた | 未実施であり、同じ対策とは扱わない |

#### 最上段の前脚と後ろ脚を複数歩で整合させる

- 最上段では、前脚を単に前へ伸ばす量や後ろ脚を単に前へ伸ばす量を固定値で増やさない
- 次の4着地候補について、各足先の支持可能領域、IK余裕、支持多角形余裕、胴体の前進可能量を同時に評価する
- 前脚が最上面前端に近い場合は、前脚を最上面の奥へ置く追加着地を候補にし、その接地を確認してから胴体を前へ進める
- 後ろ脚が最後の踏面から最上面へ届かない場合は、後ろ脚を無理に振らず、前脚支持と胴体前進で後ろ脚の到達可能領域ができるまで接地を維持する
- 前脚遊脚中にも最上段付近で転倒しているため、後ろ脚専用モードにはせず、どの脚が次の遊脚でも同じ支持余裕条件を使う
- 最上段の判定はworld名や段数で固定せず、前方の支持面が同じ高さで長く続き、後方に低い連続面が残る地形形状から判定する

| 最上段の候補系列 | 適用条件 | 回避する失敗 |
| --- | --- | --- |
| 前脚を最上面の奥へ追加着地する | 前脚が前端に近く、奥側に支持可能領域がある | 前脚支持が前端へ偏ったまま後ろ脚を上げる失敗 |
| 胴体を低速で前へ進める | 前脚2本と少なくとも1本の後ろ脚で支持余裕を確保できる | 後ろ脚の到達距離不足 |
| 後ろ脚を最上面へ順に移す | 各後ろ脚の着地点と3脚支持余裕が成立する | 後ろ脚遊脚中の後退とロール |
| 接地を維持して再計画する | 次の安全領域または支持余裕が成立しない | 無効着地点のまま遊脚を開始する失敗 |

#### 最初の蹴上と下りを別の境界条件として扱う

- 最初の蹴上では、平地から高い踏面へ初めて移る脚の支持可能領域と、その直後に残る3脚支持を評価する
- 確認群の3回は最初の蹴上付近で後退したため、最上段だけを直しても全体のロバスト性は上がらない
- 下りでは、低い踏面の奥側へ着地点を置き、接地確認前に胴体を前へ進めすぎない
- 上りと下りで同じ固定オフセットを使わず、次の支持面の高さと進行方向に対する前後余裕を費用へ入れる
- ヨーと横ずれが増えた場合は、左右脚の支持余裕差を使って前進速度とヨー参照を下げる

#### 歩容位相を固定する試験と位相へ耐える制御を分ける

- 原因分離試験では、WALK開始時の `current_plan_index` または階段手前の位相を固定し、同じ遊脚で蹴上へ入る条件を作る
- ロバスト性試験では、FL、BL、FR、BRが次の遊脚になる4条件を意図的に作り、すべての条件を通す
- 一つの成功位相だけへ同期する機能は再現性を上げても、他の位相へ耐えるロバスト性を証明しない
- 最終制御は、到達位相を選ぶだけでなく、選んだ位相で安全な着地点が無い場合に接地を維持して別位相へ移れる必要がある

#### 実装と検証の順序

| 段階 | 実施内容 | 次へ進む条件 |
| ---: | --- | --- |
| 1 | 計画着地点、実足位置、支持可能領域境界、段端距離、支持多角形余裕、NMPC状態をCSVへ追加する | 各転倒の直前2歩を脚別に再構成できる |
| 2 | 支持可能領域への投影を `shadow` で計算し、現着地点との差だけを記録する | 全脚で投影点が同じ踏面内にあり、IK余裕を満たす |
| 3 | 投影点、接地維持、減速を一組で適用し、2段階段と4段階段を1回ずつ診断する | 接近平地転倒と最初の蹴上転倒を増やさず、適用着地の `support=0` を0回にする |
| 4 | 4種類の到達脚位相を各5回、合計20回試す | 転倒0回、完走18回以上、各位相4回以上完走し、未完走は直立停止する |
| 5 | 複数歩の最上段遷移を適用し、同じ20回を試す | 基準より下り到達を増やし、最上段付近の転倒を0回にする |
| 6 | 下りの着地点と支持余裕を適用し、同じ20回を試す | 完走18回以上を維持し、下り転倒と終了横ずれを0回にする |
| 7 | 平地、溝、10 cm、15 cm、20 cmの階段へ回帰する | 既存の平地と溝を悪化させず、階段高ごとの結果を分けて記録する |

- 段階4以降は1回の成功で採用せず、到達脚位相ごとの成功率を残す
- 完走できない場合でも直立停止できた試行は安全性では合格候補にするが、完走成功へは数えない
- 各段階は直前段階から変えた変数だけを表へ残し、失敗位置、遊脚、計画点、実足位置、支持余裕を試行ごとに対応させる

#### 採用しない短絡策

| 短絡策 | 採用しない理由 |
| --- | --- |
| 前脚の歩幅だけを一律に増やす | 最初の蹴上、前脚遊脚中の最上段転倒、下り、横ずれを同時に扱えず、段端とIK余裕を減らしうる |
| 後ろ脚の歩幅だけを一律に増やす | 後ろ脚以外の遊脚でも失敗し、支持可能領域が無い状態で到達距離だけを増やす |
| `foothold_support_check_mode: enforce` だけを再度有効にする | 実施済み5回で下り到達を3回から0回へ減らした |
| `foothold_edge_inset_mode: enforce` だけを再度有効にする | x方向の段端後退だけでは成功数を増やさず、下りのピッチ転倒をロール転倒へ置き換えた |
| `support_triangle_shift_mode: enforce` だけを再度有効にする | 一律の3脚重心移動が接近平地を崩した |
| 平滑半径または振り足高さだけを再調整する | 局所効果は確認済みだが、着地点と接地後の支持余裕を変えない |
| 一つの到達脚位相だけへ固定する | 成功を再現できても、他の位相へのロバスト性を証明しない |

### 分析結果

- 最初に実施すべき制御変更は、支持不足候補の単純拒否ではなく、支持可能領域内へ着地点を生成し、生成できないときに接地を維持する処理である
- 次に実施すべき変更は、安全な着地点を次の4歩へ接続し、最上段で前脚支持、胴体前進、後ろ脚移動を一つの系列として評価する処理である
- 胴体参照は3脚の幾何中心へ一律に動かさず、複数歩の着地点から得た支持多角形余裕を使って速度と位置を制約する
- ロバスト性の判定は、4種類の到達脚位相を各5回試し、全体成功率だけでなく各位相の最低成功数と転倒数で行う
- この順序は、最上段の後ろ足失敗だけでなく、最初の蹴上、前脚遊脚中の最上段、下り、ヨー横ずれを同じ支持余裕の枠組みで扱う

## 第12章 上りを速くし下りだけを遅くする

### 概要

- 上りは 0.30 m/s、胴体 x が 4.90 m を超えたあと下りは 0.10 m/s である
- 5回は完走0回で、下りまで届いたのは3回、最初の蹴上の手前で止まったのは2回である

### 詳細

- 上りは 0.30 m/s、胴体 x が 4.90 m を超えたあと下りは 0.10 m/s である

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `FORWARD_VEL_MPS` | `0.30` | 最上段と下りへ届く回数が多かった速度を上りに使う |
| `DESCENT_VEL_MPS` | `0.10` | 下りだけ基準の速度へ戻し、下り中の転倒が減るかを見る |
| `DESCENT_SWITCH_X_M` | `4.90` | 下りの開始 x で指令速度を切り替える |
| `foothold_support_check_mode` | `shadow` | 基準と同じ支持判定のまま速度だけを変える |
| `foothold_edge_inset_mode` | `"off"` | 段端後退は使わない |
| `support_triangle_shift_mode` | `"off"` | 支持三角形への胴体移動は使わない |
| `PLAN_STARTUP_S` | `8` | 計画周期へ入ってから WALK を送る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 DESCENT_VEL_MPS=0.10 DESCENT_SWITCH_X_M=4.90 bash scripts/trial/run_quadsdk_stair_robustness.sh descent_v030_v010 5
```

- 5回は完走0回で、下りまで届いたのは3回、最初の蹴上の手前で止まったのは2回である
- 1回目は、下り x=5.461 m でピッチ 0.807 rad、遊脚 BL の転倒である
- 最大 x は 6.171 m である
- NMPC失敗は732回である

![下り減速の1回目](../../artifacts/logs/quadsdk_step23_descent_v030_v010_r01/trial.gif)

- 2回目は、最初の蹴上 x=2.990 m でピッチ -0.801 rad、遊脚 BL の転倒である
- 最大 x は 3.240 m で、速度の切替 x=4.90 m には届いていない
- NMPC失敗は762回である

![下り減速の2回目](../../artifacts/logs/quadsdk_step23_descent_v030_v010_r02/trial.gif)

- 3回目は、接近平地 x=2.855 m でピッチ -0.801 rad、遊脚 BL の転倒である
- 最大 x は 3.239 m で、速度の切替には届いていない
- NMPC失敗は752回である

![下り減速の3回目](../../artifacts/logs/quadsdk_step23_descent_v030_v010_r03/trial.gif)

- 4回目は、転倒閾値を超えず最大 x=7.049 m まで進んだ
- 終了 y は -1.195 m、終了 z は 0.307 m、終了ロールは 0 rad 付近で、横ずれの条件を外している
- NMPC失敗は2回である

![下り減速の4回目](../../artifacts/logs/quadsdk_step23_descent_v030_v010_r04/trial.gif)

- 5回目は、退出側 x=6.157 m でロール -0.807 rad、遊脚 BR の転倒である
- 最大 x は 6.563 m である
- NMPC失敗は652回である

![下り減速の5回目](../../artifacts/logs/quadsdk_step23_descent_v030_v010_r05/trial.gif)

- 基準の 0.10 m/s 有効5回は完走0回、下り到達3回である
- この5回も完走は0回である
- 基準試行と比較して
  - 改善無し
- 下りだけを 0.10 m/s にする設定は採用しない

## 第13章 最上段で頭を下げる

### 概要

- 平らな最上段にいるとき、胴体ピッチの参照を 0.10 rad の頭下げへ置き換える
- 5回は完走0回で、後ろ向きのピッチ転倒が残っている

### 詳細

- 平らな最上段にいるとき、胴体ピッチの参照を 0.10 rad の頭下げへ置き換える

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `top_nose_down_pitch_rad` | `0.10` | 最上段で後ろ向きに倒れることが多いので、平らな上面のピッチ参照を頭下げにする |
| `FORWARD_VEL_MPS` | `0.30` | 最上段と下りへ届く回数が多かった速度を使う |
| `foothold_support_check_mode` | `shadow` | 基準と同じ支持判定のまま、ピッチ参照だけを変える |
| `foothold_edge_inset_mode` | `"off"` | 段端後退は使わない |
| `support_triangle_shift_mode` | `"off"` | 支持三角形への胴体移動は使わない |
| `PLAN_STARTUP_S` | `8` | 計画周期へ入ってから WALK を送る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 bash scripts/trial/run_quadsdk_stair_robustness.sh nosedown_v030 5
```

- 5回は完走0回で、後ろ向きのピッチ転倒が残っている
- 1回目は、上り x=3.524 m でロール -0.804 rad、ピッチ -0.768 rad、遊脚 FL の転倒である
- 最大 x は 3.988 m である
- NMPC失敗は737回である

![頭下げの1回目](../../artifacts/logs/quadsdk_step23_nosedown_v030_r01/trial.gif)

- 2回目は、上り x=3.452 m でピッチ -0.802 rad、遊脚 BL の転倒である
- 最大 x は 3.583 m である
- NMPC失敗は720回である

![頭下げの2回目](../../artifacts/logs/quadsdk_step23_nosedown_v030_r02/trial.gif)

- 3回目は、下り x=5.302 m でピッチ 0.823 rad、遊脚 FL の転倒である
- 最大 x は 6.223 m である
- NMPC失敗は700回である

![頭下げの3回目](../../artifacts/logs/quadsdk_step23_nosedown_v030_r03/trial.gif)

- 4回目は、上り x=3.568 m でピッチ -0.802 rad、遊脚 BL の転倒である
- 最大 x は 3.816 m である
- NMPC失敗は695回である

![頭下げの4回目](../../artifacts/logs/quadsdk_step23_nosedown_v030_r04/trial.gif)

- 5回目は、上り最終段 x=3.738 m でロール 0.802 rad、遊脚 BR の転倒である
- 最大 x は 4.072 m である
- NMPC失敗は744回である

![頭下げの5回目](../../artifacts/logs/quadsdk_step23_nosedown_v030_r05/trial.gif)

- 基準の 0.30 m/s も完走0回である
- 頭下げ 0.10 rad の5回にも、ピッチが負の転倒が残っている
- 基準試行と比較して
  - 改善無し
- `top_nose_down_pitch_rad: 0.10` は採用しない

## 第14章 前足を次の段の奥へ置く

### 概要

- モードが off のままの5回と、計画器が停止した1回と、奥端を踏破スコアで棄却した3回は、前足の着地を次の段の奥へ動かしていない
- 前足だけを、今の足から 0.45 m 以内で次の1段の奥の踏める位置へ置いた5回は、すべて最初の蹴上の手前で失敗した

### 詳細

- モードが off のままの5回と、計画器が停止した1回と、奥端を踏破スコアで棄却した3回は、前足の着地を次の段の奥へ動かしていない
- `frontfar_v030` の5回は、実行時の `front_next_tread_mode` が `"off"` のままで、着地規則は動いていない
- 1回目の最大 x は 3.548 m である

![前足着地が無効の1回目](../../artifacts/logs/quadsdk_step23_frontfar_v030_r01/trial.gif)

- 2回目の最大 x は 6.441 m である

![前足着地が無効の2回目](../../artifacts/logs/quadsdk_step23_frontfar_v030_r02/trial.gif)

- 3回目の最大 x は 3.386 m である

![前足着地が無効の3回目](../../artifacts/logs/quadsdk_step23_frontfar_v030_r03/trial.gif)

- 4回目の最大 x は 5.816 m である

![前足着地が無効の4回目](../../artifacts/logs/quadsdk_step23_frontfar_v030_r04/trial.gif)

- 5回目の最大 x は 3.580 m である

![前足着地が無効の5回目](../../artifacts/logs/quadsdk_step23_frontfar_v030_r05/trial.gif)

- `frontfar2_v030` の1回は、`local_planner_node` が地図範囲外の参照で停止し、着地規則の試験になっていない

![前足着地の計画器停止](../../artifacts/logs/quadsdk_step23_frontfar2_v030_r01/trial.gif)

- `frontfar3_v030` の3回は、次の段の奥端を計算したあと、そのセルの踏破スコアが 0.02 で閾値 0.6 を下回り、着地 x を書き戻していない
- 1回目は下り x=5.401 m でロール -0.825 rad、遊脚 FR の転倒、最大 x は 7.224 m である

![奥端を棄却した1回目](../../artifacts/logs/quadsdk_step23_frontfar3_v030_r01/trial.gif)

- 2回目は下り x=5.579 m でロール -0.804 rad、遊脚 BL の転倒、最大 x は 6.779 m である

![奥端を棄却した2回目](../../artifacts/logs/quadsdk_step23_frontfar3_v030_r02/trial.gif)

- 3回目は下り x=5.107 m でロール 0.804 rad、遊脚 FR の転倒、最大 x は 6.893 m である

![奥端を棄却した3回目](../../artifacts/logs/quadsdk_step23_frontfar3_v030_r03/trial.gif)

- 前足だけを、今の足から 0.45 m 以内で次の1段の奥の踏める位置へ置いた5回は、すべて最初の蹴上の手前で失敗した

| 変数名 | 値 | 背景・目的 |
| --- | --- | --- |
| `front_next_tread_mode` | `enforce` | 前足 FL と FR だけを、次の1段の奥の踏める位置へ置き、その次の段には置かない |
| `FORWARD_VEL_MPS` | `0.30` | 最上段と下りへ届く回数が多かった速度を使う |
| `foothold_support_check_mode` | `shadow` | 支持判定の拒否は混ぜない |
| `foothold_edge_inset_mode` | `"off"` | 段端後退は使わない |
| `support_triangle_shift_mode` | `"off"` | 支持三角形への胴体移動は使わない |
| `top_nose_down_pitch_rad` | `0.0` | 頭下げは使わない |
| `PLAN_STARTUP_S` | `8` | 計画周期へ入ってから WALK を送る |

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash PLAN_STARTUP_S=8 FORWARD_VEL_MPS=0.30 bash scripts/trial/run_quadsdk_stair_robustness.sh frontfar5_v030 5
```

- 1回目は、最大 x=2.723 m で終了し、終了ロールは -3.142 rad、終了 z は 0.058 m である
- 最初の蹴上 x=3.00 m には届いていない
- NMPC失敗は806回である

![前足を次の段の奥へ置いた1回目](../../artifacts/logs/quadsdk_step23_frontfar5_v030_r01/trial.gif)

- 2回目は、接近平地 x=2.339 m でロール -0.803 rad、遊脚 BL の転倒である
- 最大 x は 2.407 m である
- NMPC失敗は802回である

![前足を次の段の奥へ置いた2回目](../../artifacts/logs/quadsdk_step23_frontfar5_v030_r02/trial.gif)

- 3回目は、最大 x=2.579 m、終了 z=0.321 m、終了ロールは約 0 rad で、最初の蹴上には届いていない
- NMPC失敗は689回である

![前足を次の段の奥へ置いた3回目](../../artifacts/logs/quadsdk_step23_frontfar5_v030_r03/trial.gif)

- 4回目は、接近平地 x=2.590 m でロール -0.802 rad、遊脚 BL の転倒である
- 最大 x は 2.779 m である
- NMPC失敗は792回である

![前足を次の段の奥へ置いた4回目](../../artifacts/logs/quadsdk_step23_frontfar5_v030_r04/trial.gif)

- 5回目は、接近平地 x=2.296 m でピッチ -0.802 rad、遊脚 BL の転倒である
- 最大 x は 2.505 m である
- NMPC失敗は781回である

![前足を次の段の奥へ置いた5回目](../../artifacts/logs/quadsdk_step23_frontfar5_v030_r05/trial.gif)

- ログの `x_used` は、前足が段に近いとき次の1段の内側へ移り、その次の段の x にはなっていない
- 基準の 0.30 m/s は5回中4回が最上面へ届いた
- この5回は5回とも最初の蹴上の手前で失敗し、完走は0回である
- 基準試行と比較して
  - 改善無し
- `front_next_tread_mode: enforce` は採用しない

## 第15章 区間ごとの通過回数

### 概要

- 転倒前の最大 x が区間の出口を超えた回数で、基準 0.10 m/s と、それ以上に見える条件を比べる
- 階段前の平地と上りは、基準 0.10 m/s が 5/5 である
- 最上段を抜けた回数は、確認設定 0.30 m/s の 4/5 が基準の 3/5 より多い
- 下りを抜けた回数は、上り 0.30 m/s から下り 0.10 m/s の 2/5 が基準の 1/5 より多い
- 階段下の平地を進んだのは、基準 0.10 m/s の1回と下り減速の1回で、どちらも横ずれのため完走ではない
- 支持判定の `enforce`、支持三角形、頭下げ、前足の奥置きは、どの区間でも基準 0.10 m/s を超えていない

### 詳細

- 転倒前の最大 x が区間の出口を超えた回数で、基準 0.10 m/s と、それ以上に見える条件を比べる
  - 階段前の平地の出口は x=3.00 m である
  - 上りの出口は最上段の開始 x=3.90 m である
  - 最上段の出口は下りの開始 x=4.90 m である
  - 下りの出口は x=5.80 m である
  - 階段下の平地を進んだ判定は、転倒前の最大 x が 6.50 m 以上である
  - 完走は、最大 x が 7.30 m 以上で、終了の横ずれ、高さ、ロール、ピッチを同時に満たすことである
  - この表の条件は、完走が 0 回である

| 条件 | 速度 | 試行 | 1 階段前の平地 | 2 上り | 3 最上段 | 4 下り | 5 階段下の平地 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 基準、支持判定は shadow、段端後退は off、支持三角形は off | 0.10 m/s | 5 | 5 | 5 | 3 | 1 | 1 |
| 同じ基準設定 | 0.30 m/s | 5 | 5 | 4 | 3 | 1 | 0 |
| 確認設定、中身は基準と同じ | 0.30 m/s | 5 | 4 | 4 | 4 | 0 | 0 |
| 上り 0.30 m/s、x=4.90 m から下り 0.10 m/s | 0.30 から 0.10 | 5 | 5 | 3 | 3 | 2 | 1 |
| `foothold_edge_inset_mode: enforce`、他は基準 | 0.10 m/s | 5 | 5 | 5 | 2 | 0 | 0 |

- 階段前の平地と上りは、基準 0.10 m/s が 5/5 である
  - 段端後退 0.10 m/s も、この2区間は 5/5 である
  - 基準 0.30 m/s は、平地が 5/5、上りが 4/5 である
- 最上段を抜けた回数は、確認設定 0.30 m/s の 4/5 が基準の 3/5 より多い
  - その4回は下り途中の x が約 5.5 m で転倒し、下りの出口 x=5.80 m は超えていない
  - 確認設定の5回のうち1回は起動異常で、階段前の平地を抜けていない
- 下りを抜けた回数は、上り 0.30 m/s から下り 0.10 m/s の 2/5 が基準の 1/5 より多い
  - この条件の上りを抜けた回数は 3/5 で、基準 0.10 m/s の 5/5 より少ない
  - 基準 0.30 m/s の下りを抜けた回数は 1/5 で、階段下の平地の 6.50 m には届いていない
- 階段下の平地を進んだのは、基準 0.10 m/s の1回と下り減速の1回で、どちらも横ずれのため完走ではない
  - 基準 0.10 m/s のその1回は、終了 y=1.15 m である
  - 下り減速のその1回は、最大 x=7.05 m、終了 y=-1.20 m である
- 支持判定の `enforce`、支持三角形、頭下げ、前足の奥置きは、どの区間でも基準 0.10 m/s を超えていない
  - 前足を次の段の奥へ置く 0.30 m/s は、5回とも x=3.00 m の手前で止まっている
  - 頭下げ 0.10 rad の 0.30 m/s は、最上段を抜けたのが 1/5 である

## 第16章 Quad-SDK main と基準状態の差

### 概要

- 第16章は、基準状態が Quad-SDK の main から変えた歩容、先読み、速度、振り足、平滑法線を示す
- NMPC の運動方程式と胴体のコスト重みは、main のままである
- 段端後退、支持三角形、前足の奥置き、頭下げは、基準ではオフである

### 詳細

- 第16章は、基準状態が Quad-SDK の main から変えた歩容、先読み、速度、振り足、平滑法線を示す
  - 比較先の main は、`robomechanics/quad-sdk` の main で、ロボットは Spirit、歩容はトロットである
  - 基準状態は、Step 23 で選んだ Go2 の 0.10 m/s である

| 項目 | Quad-SDK main | 基準状態 |
| --- | --- | --- |
| ロボット | Spirit、胴体高さ 0.27 m、足先半径 0.02 m | Go2、胴体高さ 0.30 m、質量 16.1 kg、足先半径 0.022 m |
| 歩容 | トロット、周期 0.36 s、接地割合 0.5、位相 `[0, 0.5, 0.5, 0]` | クロール、周期 0.90 s、接地割合 0.75、位相 `[0, 0.75, 0.5, 0.25]` |
| 足場探索 | 半径 0.25 m、遊脚高さ 0.07 m | 半径 0.70 m、遊脚高さ 0.10 m |
| 先読み | `horizon_length` 26、0.78 s | `horizon_length` 40、1.20 s |
| 速度 | 大域計画 `gbpl`、公称 0.75 m/s | `reference:=twist` の `cmd_vel` 0.10 m/s |
| 歩き始め | `stand_cmd_vel_threshold` 0.1 m/s | 0.05 m/s |
| 線形ソルバ | IPOPT の MA27 | MUMPS |
| 振り足 | 端点だけで頂点を決める | `swing_terrain_check_mode: enforce` が、経路上の地形より上へ上げる |
| 平滑法線 | `smooth_normal_vectors` の半径 0.40 m | 半径 0.10 m |
| 無効な足場 | 計画へ渡す | `stop_on_invalid_foothold` と複数歩の先読みが、渡れない列の手前で速度を 0 にする |
| 着地点の寄せ | 名目の着地点のまま | `apply_foothold: false` で、名目の着地点のまま |
| ヨーの境界 | ±π rad | ±10 rad |

- NMPC の運動方程式と胴体のコスト重みは、main のままである
  - 剛体の運動方程式、摩擦円錐、胴体の `x_weights` は main と同じである
  - 変えているのは、先読みの長さ、線形ソルバ、ヨーの境界、Go2 の関節と質量である
- 段端後退、支持三角形、前足の奥置き、頭下げは、基準ではオフである
  - `foothold_edge_inset_mode` は `"off"` である
  - `support_triangle_shift_mode` は `"off"` である
  - `front_next_tread_mode` は `"off"` である
  - `top_nose_down_pitch_rad` は 0 である
  - `foothold_support_check_mode: shadow` は支持不足を記録するだけで、着地点の選択は main の螺旋探索のままである

## 第17章 形状確認の進行

### 概要

- 第17章は、基準状態の形状確認の進行を表に残し、試行のたびに同じ表を更新する
- 失敗した条件は、その後に1回成功したら残りの回を行わない
- 5回とも失敗した条件だけ、5回で終える
- 進行表の下には、成功した回の GIF と、成功が無い条件の1回目の失敗 GIF を残す

### 詳細

- 第17章は、基準状態の形状確認の進行を表に残し、試行のたびに同じ表を更新する
  - 成功は、転倒前の最大 x が下り終端の 1.50 m 先以上で、終了の横ずれが 0.50 m 未満、高さが 0.20 m より大きく、ロールとピッチの絶対値が 0.50 rad 未満であることである
  - 高さ 0.15 m、踏面 0.30 m、4段は、第3章の基準5回を使う
  - 数値一覧は `artifacts/logs/step24_baseline_shape_summary.csv` に保存する
- 失敗した条件は、その後に1回成功したら残りの回を行わない
- 5回とも失敗した条件だけ、5回で終える

<!-- shape-progress-start -->

| 順番 | 高さ | 踏面 | 段数 | ここまで | 結果 |
| ---: | ---: | ---: | ---: | --- | --- |
| 1 | 10 cm | 39 cm | 上り4＋下り4 | 1回 | 成功 |
| 2 | 10 cm | 30 cm | 上り4＋下り4 | 1回 | 成功 |
| 3 | 12.5 cm | 39 cm | 上り4＋下り4 | 1回 | 成功 |
| 4 | 12.5 cm | 30 cm | 上り4＋下り4 | 2回目で成功 | 終了 |
| 5 | 15 cm | 60 cm | 上り4＋下り4 | 5回失敗 | 終了 |
| 6 | 15 cm | 39 cm | 上り4＋下り4 | 5回失敗 | 終了 |
| 7 | 15 cm | 30 cm | 上り1＋下り1 | 3回目で成功 | 終了 |
| 8 | 15 cm | 30 cm | 上り2＋下り2 | 2回目で成功 | 終了 |
| 9 | 15 cm | 30 cm | 上り4＋下り4 | 以前の基準5回 | この判定では成功 0 |
| 10 | 15 cm | 30 cm | 上り6＋下り6 | 2回目で成功 | 終了 |
| 11 | 15 cm | 39 cm | 上り6＋下り6 | 5回失敗 | 終了 |

<!-- shape-progress-end -->

- 進行表の下には、成功した回の GIF と、成功が無い条件の1回目の失敗 GIF を残す
  - 1-1
    - この回は成功である
    - 条件は、高さ 10 cm、踏面 39 cm、上り4段と下り4段である

![1-1](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h10_d39_n04/trial.gif)

  - 2-1
    - この回は成功である
    - 条件は、高さ 10 cm、踏面 30 cm、上り4段と下り4段である

![2-1](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h10_d30_n04/trial.gif)

  - 3-1
    - この回は成功である
    - 条件は、高さ 12.5 cm、踏面 39 cm、上り4段と下り4段である

![3-1](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h12p5_d39_n04/trial.gif)

  - 4-2
    - この回は成功である
    - 条件は、高さ 12.5 cm、踏面 30 cm、上り4段と下り4段である

![4-2](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h12p5_d30_n04_r02/trial.gif)

  - 4-3
    - この回は成功である
    - 条件は、高さ 12.5 cm、踏面 30 cm、上り4段と下り4段である

![4-3](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h12p5_d30_n04_r03/trial.gif)

  - 5-1
    - この回は失敗である
    - 条件は、高さ 15 cm、踏面 60 cm、上り4段と下り4段である

![5-1](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h15_d60_n04/trial.gif)

  - 6-1
    - この回は失敗である
    - 条件は、高さ 15 cm、踏面 39 cm、上り4段と下り4段である

![6-1](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h15_d39_n04/trial.gif)

  - 7-3
    - この回は成功である
    - 条件は、高さ 15 cm、踏面 30 cm、上り1段と下り1段である

![7-3](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h15_d30_n01_r03/trial.gif)

  - 8-2
    - この回は成功である
    - 条件は、高さ 15 cm、踏面 30 cm、上り2段と下り2段である

![8-2](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h15_d30_n02_r02/trial.gif)

  - 9-1
    - この回は失敗である
    - 条件は、高さ 15 cm、踏面 30 cm、上り4段と下り4段である

![9-1](../../artifacts/logs/quadsdk_step23_baseline_shadow_r01/trial.gif)

  - 10-2
    - この回は成功である
    - 条件は、高さ 15 cm、踏面 30 cm、上り6段と下り6段である

![10-2](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h15_d30_n06_r02/trial.gif)

  - 11-1
    - この回は失敗である
    - 条件は、高さ 15 cm、踏面 39 cm、上り6段と下り6段である

![11-1](../../artifacts/logs/quadsdk_step24_jp_stair_matrix_h15_d39_n06/trial.gif)

## 結論

### 概要

- 完走した条件は2つあり、どちらも5回中1回で、100%ではない
- 階段を最後まで歩く設定としては、基準の 0.10 m/s を選ぶ
- 成績は5区間の通過率の和で、選んだ基準 0.10 m/s が 3.00 で1位である
- 着地点を段へ合わせる対策は4つ試し、いずれも完走を5/5にしていない

### 詳細

- 完走した条件は2つあり、どちらも5回中1回で、100%ではない
  - ここでの完走は、転倒せずに階段下の平地 x=6.50 m 以上まで歩いたことである
  - 基準 0.10 m/s の `shadow09_r01` は、転倒せず最大 x=7.574 m、終了 z=0.309 m まで歩いた
  - その回の終了 y は 1.152 m で、横ずれ 0.50 m 未満は外している
  - 上り 0.30 m/s から x=4.90 m で下り 0.10 m/s へ切り替えた4回目は、転倒せず最大 x=7.049 m、終了 z=0.307 m まで歩いた
  - その回の終了 y は -1.195 m で、横ずれ 0.50 m 未満は外し、最大 x も 7.30 m 未満である
  - この2回以外に、階段下の平地まで転倒せず届いた試行は無い
  - 基準 0.30 m/s の2回目の最大 x=7.349 m は転倒後の値で、完走には数えない

- 階段を最後まで歩く設定としては、基準の 0.10 m/s を選ぶ
  - 完走は、基準 0.10 m/s が 1/5、速度切替が 1/5 である
  - 基準 0.10 m/s は、平地前 5/5、上り 5/5、最上段 3/5、下り 1/5、階段下 1/5 である
  - 速度切替は下り 2/5 と階段下 1/5 で、上りは 3/5 である
  - 完走回数が同じ1回のなかで、手前の区間をより多く残す基準 0.10 m/s を選ぶ
  - 確認設定 0.30 m/s は最上段 4/5 で、下りの出口は 0/5 である
  - 段端後退 0.10 m/s は上りまで 5/5 で、下りは 0/5 である
  - 0.10 m/s では基準が階段下 1/5 まで残り、0.15 m/s と 0.20 m/s は階段下 0 である
  - 0.30 m/s の基準は下り 1/5、階段下 0/5 で、選んだ 0.10 m/s より短い

### 成績順

- 成績は5区間の通過率の和で、選んだ基準 0.10 m/s が 3.00 で1位である
- 1区間を全試行が抜けると 1.00 で、5区間すべてを全試行が抜けると 5.00 である
- 成績が同じときは、より先の区間を多く抜けた行を上にする
- それも同じ行は、同じ順位にする

| 順位 | 条件 | 速度 | 成績 | 平地前 | 上り | 最上段 | 下り | 階段下 |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 基準 | 0.10 m/s | 3.00 | 5/5 | 5/5 | 3/5 | 1/5 | 1/5 |
| 2 | 上り 0.30 m/s、下り 0.10 m/s | 切替 | 2.80 | 5/5 | 3/5 | 3/5 | 2/5 | 1/5 |
| 3 | 基準 | 0.30 m/s | 2.60 | 5/5 | 4/5 | 3/5 | 1/5 | 0/5 |
| 4 | 確認設定、基準と同じ制御 | 0.30 m/s | 2.40 | 4/5 | 4/5 | 4/5 | 0/5 | 0/5 |
| 5 | 段端後退 enforce | 0.10 m/s | 2.40 | 5/5 | 5/5 | 2/5 | 0/5 | 0/5 |
| 6 | 基準 | 0.20 m/s | 1.80 | 5/5 | 3/5 | 1/5 | 0/5 | 0/5 |
| 6 | 支持判定 enforce | 0.20 m/s | 1.80 | 5/5 | 3/5 | 1/5 | 0/5 | 0/5 |
| 6 | 支持判定 enforce | 0.30 m/s | 1.80 | 5/5 | 3/5 | 1/5 | 0/5 | 0/5 |
| 6 | 頭下げ 0.10 rad | 0.30 m/s | 1.80 | 5/5 | 3/5 | 1/5 | 0/5 | 0/5 |
| 10 | 段端後退 enforce | 0.20 m/s | 1.60 | 4/5 | 3/5 | 1/5 | 0/5 | 0/5 |
| 11 | 段端後退 enforce | 0.30 m/s | 1.60 | 5/5 | 2/5 | 1/5 | 0/5 | 0/5 |
| 12 | 支持判定 enforce | 0.10 m/s | 1.60 | 5/5 | 3/5 | 0/5 | 0/5 | 0/5 |
| 13 | 確認設定、基準と同じ制御 | 0.10 m/s | 1.40 | 5/5 | 2/5 | 0/5 | 0/5 | 0/5 |
| 14 | 確認設定、基準と同じ制御 | 0.20 m/s | 1.20 | 4/5 | 2/5 | 0/5 | 0/5 | 0/5 |
| 15 | 基準 | 0.15 m/s | 1.00 | 3/5 | 2/5 | 0/5 | 0/5 | 0/5 |
| 16 | 支持三角形 enforce | 0.20 m/s | 1.00 | 1/1 | 0/1 | 0/1 | 0/1 | 0/1 |
| 16 | 支持三角形 enforce | 0.30 m/s | 1.00 | 1/1 | 0/1 | 0/1 | 0/1 | 0/1 |
| 18 | 支持三角形 enforce | 0.10 m/s | 0.00 | 0/1 | 0/1 | 0/1 | 0/1 | 0/1 |
| 18 | 前足を次の段の奥 | 0.30 m/s | 0.00 | 0/5 | 0/5 | 0/5 | 0/5 | 0/5 |

### 条件の目線

- 数字は、転倒前にその区間の出口を超えた回数である
- 確認設定は、支持判定 shadow、段端後退 off、支持三角形 off で、基準と同じ制御である
- 確認設定が基準と違うのは、別の試行群であることだけである

| 条件 | 速度 | 平地前 | 上り | 最上段 | 下り | 階段下 | 基準 0.10 m/s との関係 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 基準 | 0.10 m/s | 5/5 | 5/5 | 3/5 | 1/5 | 1/5 | 選ぶ設定、完走 1/5 |
| 基準 | 0.15 m/s | 3/5 | 2/5 | 0/5 | 0/5 | 0/5 | 同じ条件の 0.10 m/s より短い |
| 基準 | 0.20 m/s | 5/5 | 3/5 | 1/5 | 0/5 | 0/5 | 同じ条件の 0.10 m/s より短い |
| 基準 | 0.30 m/s | 5/5 | 4/5 | 3/5 | 1/5 | 0/5 | 最上段と下りの回数は同じで、階段下は届かない |
| 段端後退 enforce | 0.10 m/s | 5/5 | 5/5 | 2/5 | 0/5 | 0/5 | 上りまでは同じで、最上段と下りが短い |
| 段端後退 enforce | 0.20 m/s | 4/5 | 3/5 | 1/5 | 0/5 | 0/5 | 基準 0.10 m/s より短い |
| 段端後退 enforce | 0.30 m/s | 5/5 | 2/5 | 1/5 | 0/5 | 0/5 | 基準 0.10 m/s より短い |
| 支持判定 enforce | 0.10 m/s | 5/5 | 3/5 | 0/5 | 0/5 | 0/5 | 上りの途中までで、下りは 0 |
| 支持判定 enforce | 0.20 m/s | 5/5 | 3/5 | 1/5 | 0/5 | 0/5 | 基準 0.10 m/s より短い |
| 支持判定 enforce | 0.30 m/s | 5/5 | 3/5 | 1/5 | 0/5 | 0/5 | 基準 0.10 m/s より短い |
| 支持三角形 enforce | 0.10 m/s | 0/1 | 0/1 | 0/1 | 0/1 | 0/1 | 平地の途中で転倒する |
| 支持三角形 enforce | 0.20 m/s | 1/1 | 0/1 | 0/1 | 0/1 | 0/1 | 上りに入る前で転倒する |
| 支持三角形 enforce | 0.30 m/s | 1/1 | 0/1 | 0/1 | 0/1 | 0/1 | 上りに入る前で転倒する |
| 確認設定、基準と同じ制御 | 0.10 m/s | 5/5 | 2/5 | 0/5 | 0/5 | 0/5 | 同じ制御の別群で、基準の 5 回より短い |
| 確認設定、基準と同じ制御 | 0.20 m/s | 4/5 | 2/5 | 0/5 | 0/5 | 0/5 | 基準 0.10 m/s より短い |
| 確認設定、基準と同じ制御 | 0.30 m/s | 4/5 | 4/5 | 4/5 | 0/5 | 0/5 | 最上段だけ多く、下りは 0 |
| 上り 0.30、下り 0.10 | 切替 | 5/5 | 3/5 | 3/5 | 2/5 | 1/5 | 下りと階段下は多いが、上りは 3/5 |
| 頭下げ 0.10 rad | 0.30 m/s | 5/5 | 3/5 | 1/5 | 0/5 | 0/5 | 同じ 0.30 m/s の基準より短い |
| 前足を次の段の奥 | 0.30 m/s | 0/5 | 0/5 | 0/5 | 0/5 | 0/5 | 平地の出口 x=3.00 m に届かない |

### 速度の目線

- 0.10 m/s では、基準が平地 5/5、上り 5/5、最上段 3/5、下り 1/5、階段下 1/5 で最も先まで残る
- 0.10 m/s の段端後退は上りまで基準と同じ 5/5 で、最上段は 2/5、下りは 0/5 である
- 0.15 m/s は基準だけで、平地 3/5、上り 2/5、最上段以降は 0/5 である
- 0.20 m/s は、どの条件も下りを抜けていない
- 0.20 m/s で相対的に先なのは基準で、最上段 1/5 までである
- 0.30 m/s の基準は、最上段 3/5、下り 1/5 で、階段下は 0/5 である
- 0.30 m/s の確認設定は最上段 4/5 だが、下りは 0/5 である
- 上り 0.30 m/s から下り 0.10 m/s は、下り 2/5、階段下 1/5 で、上りは 3/5 である
- 0.30 m/s の頭下げと前足の奥置きは、同じ速度の基準より短い
- 完走があるのは、基準 0.10 m/s の 1/5 と、上り 0.30 m/s から下り 0.10 m/s の 1/5 だけである

### 着地点を段へ合わせる対策

- 着地点を段へ合わせる対策は4つ試し、いずれも完走を5/5にしていない
  - 支持判定 `enforce` は支持不足の候補を拒否し、基準 0.10 m/s の下り到達 3/5 を 0/5 へ減らした
  - 段端後退 `enforce` は足先を段端から内側へ置き、0.10 m/s の完走は 0/5 である
  - 支持三角形 `enforce` は3脚の重心側へ胴体を寄せ、0.10 m/s では平地の途中で転倒した
  - 前足を次の段の奥へ置く `enforce` は前脚だけを次の踏面の奥へ置き、0.30 m/s の5回は最初の蹴上 x=3.00 m の手前で止まった
  - 基準 0.10 m/s の有効5回は、x=3.00 m の遊脚が BL 4回、BR 1回でも、完走は 1/5 である
  - 到達遊脚が段の位置と同期していないことだけでは、完走が 1/5 に留まることを説明しない
  - 足先の実位置と計画着地点は同じ CSV に無いので、踏み外した脚と段端からの距離は未確認である
