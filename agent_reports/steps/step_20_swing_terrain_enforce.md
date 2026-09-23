# 地形対応の振り足頂点

## 第1章 概要

- Step 20 は、振り経路上の最大地形高さから求めた頂点を、到達可能な振り足軌道へ適用する
- 平地は地形対応頂点と従来頂点が一致するため、Step 20 は平地の振り足高さを変えない
- 単体試験は、途中に高さ 0.18 m の踏面がある経路で、適用後の頂点が従来頂点より高くなることを確認した
- 第1試行は、胴体中心を最初の蹴上より 0.035 m 先まで進めたが、その後に転倒したため失敗である
- 第1試行は、適用前の最大 x=3.074 m を超えなかったため、振り足頂点の適用だけでは階段失敗を解消しない
- 次の変更は制御値を変えず、従来頂点、地形対応頂点、適用頂点を脚ごとに記録する

## 第2章 実装

### 概要

- `swing_terrain_check_mode` は `enforce` を受け付ける
- `evaluateSwingClearance` は、振り経路の `z_inpainted` 最大値へ `ground_clearance` を足して地形対応頂点を求める
- `computeSwingApex` は、地図が有限で地形対応頂点が股関節下の上限以内なら、その頂点を返す
- `computeSwingApex` は、地図が欠ける場合または上限を超える場合に従来頂点を返す

### 詳細

- `swing_terrain_check_mode` は `enforce` を受け付ける
  - `off` は従来頂点だけを使う
  - `shadow` は地形対応頂点を計測するが従来頂点を使う
  - `enforce` は成立条件を満たす地形対応頂点を使う
- `evaluateSwingClearance` は、振り経路の `z_inpainted` 最大値へ `ground_clearance` を足して地形対応頂点を求める
  - 始点と終点の間を地図解像度の半分以下の間隔で標本化する
  - 各標本の `z_inpainted` を比較して最大地形高さを求める
  - 地形対応頂点は、従来頂点と最大地形高さ加算値の大きい方である
- `computeSwingApex` は、地図が有限で地形対応頂点が股関節下の上限以内なら、その頂点を返す
  - 股関節下の上限は股関節高さから `hip_clearance` を引いた値である
  - 成立条件は `required_apex <= hip_ceiling` である
- `computeSwingApex` は、地図が欠ける場合または上限を超える場合に従来頂点を返す
  - 地図外または非数値の標本は `path_finite=false` にする
  - 上限を超える地形対応頂点は `feasible=false` にする
  - 不成立経路を無理に高く上げず、次の段階で足場変更または停止へ伝える

## 第3章 単体試験

### 概要

- 新しい単体試験は、高さ 0.18 m の途中地形で `enforce` が地形対応頂点を返すことを確認した
- 既存単体試験は、`shadow` が従来頂点を維持することを確認した
- `local_planner_test` は失敗0件で完了した

### 詳細

- 新しい単体試験は、高さ 0.18 m の途中地形で `enforce` が地形対応頂点を返すことを確認した
  - 始点の z は 0.02 m である
  - 終点の z は 0.02 m である
  - 途中地形の z は 0.18 m である
  - 地形対応頂点は従来頂点より高い
- 既存単体試験は、`shadow` が従来頂点を維持することを確認した
  - 影計測は制御出力を変更しない
  - 平地は地形対応頂点と従来頂点が一致する
- `local_planner_test` は失敗0件で完了した
  - Release ビルドは `local_planner` を再構築した
  - CTest は対象試験を0.94秒で完了した

## 第4章 第1試行

### 概要

- 第1試行の目的は、地形対応頂点の適用が最初の蹴上での転倒を解消するか確認することである
- 第1試行は、胴体 x を 0.965 m から最大 3.035 m まで進めた
- 第1試行は、時刻 50.00 s にロールが 0.50 rad を超え、時刻 50.49 s に胴体 z が 0.20 m を下回った
- 第1試行の最大 x は、適用前の第6試行の最大 x=3.074 m より 0.039 m 小さい
- 第1試行の映像は、機体前部が最初の蹴上で持ち上がった後に姿勢を崩したことを示した
- 第1試行は失敗なので、地形対応頂点だけを階段成功の原因にしない

### 詳細

- 第1試行の目的は、地形対応頂点の適用が最初の蹴上での転倒を解消するか確認することである
  - 速度指令は 0.10 m/s である
  - 速度指令時間は50秒である
  - 共通平滑法線半径は 0.10 m である
- 第1試行は、胴体 x を 0.965 m から最大 3.035 m まで進めた
  - 最初の蹴上は x=3.00 m である
  - 胴体 z の最大値は 0.451 m である
  - 終了時の胴体 x は 2.571 m である
- 第1試行は、時刻 50.00 s にロールが 0.50 rad を超え、時刻 50.49 s に胴体 z が 0.20 m を下回った
  - ロール判定時の x は 2.726 m である
  - 胴体低下判定時の x は 2.644 m である
  - 終了時のロールは -1.587 rad である
- 第1試行の最大 x は、適用前の第6試行の最大 x=3.074 m より 0.039 m 小さい
  - 試行間の歩容位相差があるため、0.039 m の差だけで悪化とは断定しない
  - 少なくとも地形対応頂点の適用は転倒を解消していない
- 第1試行の映像は、機体前部が最初の蹴上で持ち上がった後に姿勢を崩したことを示した
  - 35秒のフレームは接近平地にいる状態を示した
  - 45秒のフレームは機体前部が最初の段に掛かった状態を示した
  - 50秒のフレームは最初の蹴上付近で姿勢を崩した状態を示した
- 第1試行は失敗なので、地形対応頂点だけを階段成功の原因にしない
  - 次は実際に適用した頂点を脚ごとに記録する
  - 次は足場位置と脚トルク警告の時刻を同じ記録で比較する

## 第5章 再実行コマンドと映像

### 概要

- Step 20は、ReleaseビルドとCTestの後に短時間階段試行を実行する
- 各階段試行は、同じworld、速度、継続時間を使い、振り足診断の追加だけを比較する
- GIFは、各試行の歩行区間を4 fps、幅480 pxで保存する

### 詳細

- `local_planner` は、地形対応頂点の適用と単体試験をReleaseで構築する

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && source /tmp/mpc_dog_stack_release_install/setup.bash && colcon --log-base /tmp/mpc_dog_stack_release_log build --base-paths external/quad-sdk --packages-select local_planner --build-base /tmp/mpc_dog_stack_release_build --install-base /tmp/mpc_dog_stack_release_install --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3 && ctest --test-dir /tmp/mpc_dog_stack_release_build/local_planner --output-on-failure
```

- ReleaseビルドとCTestは成功した

- 地形対応頂点の適用試行は、速度指令50秒と停止保持3秒で実行する

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash GAP_TAG=step20_swing_enforce_stair_short DURATION_S=50 HOLD_S=3 bash scripts/trial/run_quadsdk_jp_stair.sh
```

- 適用試行は最初の蹴上で失敗し、GIFを `artifacts/logs/quadsdk_step20_swing_enforce_stair_short/trial.gif` に保存する

![地形対応頂点の適用試行](../../artifacts/logs/quadsdk_step20_swing_enforce_stair_short/trial.gif)

- 適用頂点の診断試行は、速度指令42秒と停止保持2秒で実行する

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash GAP_TAG=step20_swing_diag_stair_short DURATION_S=42 HOLD_S=2 bash scripts/trial/run_quadsdk_jp_stair.sh
```

- 診断試行は最初の蹴上で失敗し、GIFを `artifacts/logs/quadsdk_step20_swing_diag_stair_short/trial.gif` に保存する

![地形対応頂点の診断試行](../../artifacts/logs/quadsdk_step20_swing_diag_stair_short/trial.gif)

- 呼出経路の短時間確認は、平地で速度指令5秒を実行する

```bash
QUADSDK_OVERLAY_SETUP=/tmp/mpc_dog_stack_release_install/setup.bash GAP_WORLD=flat_wide.xml GAP_TAG=step20_swing_call_probe DURATION_S=5 HOLD_S=0 bash scripts/trial/run_quadsdk_jp_stair.sh
```

- 呼出経路は `swing terrain mode=enforce` と `computeSwingApex first call mode=enforce` を記録し、GIFを `artifacts/logs/quadsdk_step20_swing_call_probe/trial.gif` に保存する

![振り足頂点の呼出確認](../../artifacts/logs/quadsdk_step20_swing_call_probe/trial.gif)

## 結論

- 地形対応の振り足頂点は、有限で股関節下の上限以内の経路へ適用できる
- 単体試験は、途中の高い地形に対して適用頂点が上がることを確認した
- 階段試行は最初の蹴上で転倒したため、地形対応頂点の適用だけでは不十分である
- 次の技術的な積み上げは、各脚へ適用した頂点を記録し、足場位置と脚トルクの関係を確定することである
