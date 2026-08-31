# 引き継ぎ資料(2026-08-30時点)

作成方法についての注記: 本資料は会話記憶だけでなく、`git status`・`git log`・
`git diff`・関連ファイルの実際の内容・過去の実行ログを再確認した上で作成した。

## 1. 最終目的

Quadruped-PyMPCとQuad-SDKの2つの制御実装を、MPC_DOGプロジェクト内で
Step単位(Step 01=前進歩行確認、以降のStepは未着手)で検証し、記録(CSV・GIF)を
残しながら比較していくこと。今回のセッションは主にQuad-SDK側のStep 01
(0.3 m/s・10m以上の前進歩行確認)に集中していた。

## 2. 現在の作業範囲

- Quad-SDK版Step 01: 前進歩行の実現・再現性検証・速度スイープ(0.1〜1.1 m/s)
- 上記に付随する目視検証手段(GIF)の改善
- リポジトリ整理(quad-sdkの`.git`を外して本体リポジトリで管理、`.gitignore`整備)
- ドキュメント整備、コミット・push・main へのマージ

**制約(現在も有効)**: CoinHSL導入・MA27への変更・MPCゲイン調整は、明確な
必要性が確認されるまで行わない、という制約が調査の初期にユーザーから
課されている。最終的にMUMPSのままで成功条件を達成できたため、この制約に
抵触する変更は一切行っていない。

## 3. 完了したこと

1. **Quad-SDK版Step 01の前進歩行を実現**(0.3 m/s・10m以上、転倒なし)。
   根本原因は「起動シーケンスの固定sleepが`joint_controller`(ros2_control、
   関節へトルクを伝えるコントローラ)のアクティブ化より短かったこと」で、
   `controller_manager_msgs/srv/ListControllers`をポーリングして実際に
   `active`になるまで待つよう修正した。
2. **10m規模歩行の地面サイズ問題を解決**: 既定の`flat.xml`(地面9m弱)は
   狭すぎて端から落ちる、より広い`big_flat.xml`(詳細メッシュ地形)は
   原因不明の不安定化を招いたため、`flat.xml`と同じ単純な直方体プリミティブ
   構造のまま範囲を拡大した`flat_wide.xml`を新規作成し、10.08m前進を達成した。
3. **プロセス残留(ゾンビプロセス)問題を発見・緩和**: `trap cleanup`の
   プロセスグループkillが一部の子ノード(`grid_map_visualization`・
   `topic_tools/relay`・`robot_state_publisher`・`static_transform_publisher`・
   `controller_manager`のspawner等)を終了しきれず残留し、最大121プロセス・
   load average 134超まで悪化していたことを発見。名前パターンでの`pkill -9`
   保険を追加し緩和(根本原因は未調査のまま、6-4節参照)。
4. **0.1 m/sが歩行しない仕様を発見・修正**: `stand_cmd_vel_threshold`(既定0.1)
   とコード側の厳密な`>`比較により、0.1 m/s指令時に歩行へ移行できない仕様を
   発見。ユーザー指示により`0.05`へ変更し解消。
5. **GIF目視検証手段を、ユーザーとの多数回のやり取りを経て確立**: 追従カメラ→
   固定カメラ、時刻(小数点1桁)・ファイル名の焼き込み、地面の5m目盛り線、
   カメラ距離・lookat位置の実測較正(可視幅約13m)。
6. **0.1〜1.1 m/s(0.2刻み、6速度)の速度スイープを最終設定で完走**
   (`velocity_sweep5`、下記4節「今後の検証データ」参照)。全速度で
   絶対座標(`base_pos_x_m`)の前進を確認済み。
7. **`external/quad-sdk`の`.git`を削除し、このリポジトリ本体で直接管理する
   方式へ変更**。未使用ロボットモデル(a1/a2/b2/go1/spirit/spot/vision60/
   underbrush、約554MB)を除外し、go2/go2wのみ追跡(約260MB)。
8. **ドキュメント整備**:
   - `agent_reports/step01/quad_sdk_step01_investigation.md`(1317行、全経緯の時系列ログ)
   - `agent_reports/step01/quad_sdk_step01_changes_and_usage.md`(要点まとめ)
   - `agent_reports/step01/quadsdk_step01_baseline_py_structure.md`(記録ハーネスの構造説明)
   - `agent_reports/step01/pympc_step01_changes_and_usage.md`・
     `agent_reports/step01/step_01_baseline_py_structure.md`(PyMPC側の同等ドキュメント。
     PyMPC本体には一切コード変更がないことを確認済み)
   - `README.md`から上記全てへリンク
9. **`.gitignore`整備**: `artifacts/gifs/*.gif`・`artifacts/logs/quadsdk_step01/`・
   `/backup/`・quad-sdk内未使用ロボットモデルを追加。`archive/`は履歴保持の
   意図的設計のため対象外のまま維持(ユーザー確認済み)。`ros2_ws/.gitignore`
   (`build/install/log`除外、既存だが未追跡だった)も追跡対象にした。
10. **コミット・push・マージ完了**: `feature/step-01-reference-baseline`ブランチへ
    2コミット(`6447a6c`quad-sdk本体、`d3d2f20`ハーネス・ドキュメント一式)、
    origin へpush、`main`へfast-forwardマージしてpush済み。
    working treeはクリーン(`git status`で確認済み)。
11. `git config merge.ff false`をこのリポジトリのローカル設定に追加
    (今後のマージでマージコミットを必ず作成し、git graphに分岐線を残す)。

## 4. 未完了のこと

- **10m規模試験の再現性が未確認**(1回のみ成功、`trials_summary.csv`のid=09、
  walk_dist_x_m=10.078m)。短時間試行(0.3 m/s)では5回連続成功を確認済みだが、
  10m規模での複数回連続成功はまだ試していない。
- **`fall_time_s`の誤検出ロジックが未修正**: 記録開始をSTAND送信前へ前倒しした
  ことで、STAND完了前の受動的な沈み込みが`FALL_HEIGHT_THRESHOLD_M`(0.15m)を
  一時的に下回り、実際には転倒していないのに`fall_time_s`が記録される
  (`trials_summary.csv`のid=04/06等で発生)。初期沈み込み区間を除外する
  ロジックは未実装。
- **プロセスグループkillが一部の子ノードを終了しきれない根本原因は未調査**
  (緩和策の`pkill -9`保険のみ実施済み)。
- **`big_flat.xml`(詳細メッシュ地形)特有の不安定化の根本原因は未確認**
  (`flat_wide.xml`で目的を達成したため優先度低として保留)。
- **WALK移行直後(t≈17〜18秒)の一時的な姿勢の跳躍の原因は未調査**
  (`flat_wide.xml`では実害なく回復するため放置)。
- **IPOPTの詳細ログ(`print_level=5`のまま)の出力内容を精査していない**
  (NMPC失敗がほぼ0件になったため必要性が下がった)。
- 速度スイープの結果、**0.5〜1.1 m/sの安定性はまだ心許ない**
  (このセッションの初期に0.5 m/sが2/2転倒→プロセス残留修正後は改善したが、
  1/2で「一度も起立しない」新しい非決定的失敗パターンを確認しており、
  完全に安定とは言い切れない。詳細は`agent_reports/step01/quad_sdk_step01_investigation.md`
  「進捗ログ 19:00時点」参照)。
- **Step 02以降は完全に未着手**(`scripts/trial/run_step_02.sh`・
  `src/trial/step_02_frequency.py`はファイルとして存在するが、このセッションの
  作業対象外だった。中身は未確認)。

## 5. 変更・追加したファイルと変更内容

### `external/quad-sdk`(元リポジトリからの変更、`git diff`で確認済み)

- `nmpc_controller/src/nmpc_controller.cpp`: `linear_solver`を`"ma27"`→
  `"mumps"`(HSL未導入のため)、`print_level`を`0`→`5`(診断用、そのまま残存)
- `local_planner/config/local_planner.yaml`: `stand_cmd_vel_threshold`を
  `0.1`→`0.05`
- `quad_utils/launch/quad_visualization.py`: `rviz`既定値を`true`→`false`
- `quad_utils/launch/quad_mujoco.py`: `camera_track_robot`・
  `camera_distance`・`camera_lookat_x`・`camera_lookat_y`をlaunch引数化
- `quad_utils/src/mujoco_recorder_node.cpp`: 上記`camera_lookat_x/y`
  パラメータを新規追加(C++、再ビルド済み・`strings`で反映確認済み)
- `quad_utils/CMakeLists.txt`・`quad_utils/package.xml`: Pinocchioの
  CMakeターゲットexport修正(このセッションより前、ビルド準備作業)
- 新規ファイル: `quad_simulator/quad_sim_scripts/worlds/flat_wide.xml.xacro`
  (単純な直方体プリミティブ地面、x∈[-3,15]・y∈[-5,5]、5m間隔目盛り線付き)、
  `quad_simulator/quad_sim_scripts/models/flat_wide/meshes/flat_wide.ply`
  (地形マップ用メッシュ)

### MPC_DOG側(新規作成)

- `scripts/trial/run_quadsdk_step01_baseline.sh`: メイン実行スクリプト
  (現在の既定値: `CAMERA_DISTANCE_M=8.72`・`CAMERA_LOOKAT_X_M=2.0`・
  `STAND_SETTLE_S=8`・`PLAN_STARTUP_S=3`・`JOINT_CONTROLLER_WAIT_TIMEOUT_S=40`)
- `src/trial/quadsdk_step01_baseline.py`: CSVロガー(ROS2ノード)
- `scripts/trial/make_gif.sh`: mp4→GIF変換(時刻・ファイル名焼き込み)
- `scripts/trial/run_reference_baseline.sh`→`scripts/trial/run_step_01.sh`、
  `src/trial/record_step01_baseline.py`→`src/trial/step_01_baseline.py`
  へリネーム(PyMPC側、Step 02ファイルとの命名統一のため。このリネーム自体は
  今回のセッション開始時点で既に行われていたものをコミットした)
- `scripts/trial/build_acados.sh`・`scripts/trial/install_quadruped_pympc.sh`:
  PyMPC用ビルド・インストールスクリプト
- ドキュメント新規5件(3節参照)、`README.md`更新
- `.gitignore`更新(3節参照)

## 6. 実行済みコマンドと結果

- **速度スイープ最終実行**(`velocity_sweep5`、ローカルディスクのみ、
  gitignore対象で未追跡): 各速度`DURATION_S=15`で実行、絶対x座標の変化
  (`base_pos_x_m`、先頭行→最終行):
  - 0.1 m/s: -0.037→0.017(閾値修正前の初回。閾値修正後の再検証では0.90m前進を確認)
  - 0.3 m/s: -0.037→2.941(Δx=2.98m)
  - 0.5 m/s: -0.036→4.802(Δx=4.84m)
  - 0.7 m/s: -0.007→6.826(Δx=6.83m)
  - 0.9 m/s: -0.036→8.586(Δx=8.62m)
  - 1.1 m/s: -0.026→7.434(Δx=7.46m)
  - CSV: `artifacts/logs/quadsdk_step01/velocity_sweep5/state_log_v{速度}.csv`
    (ローカルディスクのみ、gitignore対象)
  - グラフ: 同ディレクトリの`x_vs_time_all.png`
  - GIF: `artifacts/gifs/quadsdk_sweep5_v{速度}.gif`(ローカルディスクのみ、
    gitignore対象、ファイル存在は確認済み)
- **10m規模成功試行**: `trials_summary.csv`(こちらもgitignore対象、
  ローカルのみ)のid=09、walk_dist_x_m=10.078m、fall_time_s=0.0(誤検出、
  4節参照)
- **git操作**: `git add`/`git commit`(2回)/`git push -u origin
  feature/step-01-reference-baseline`/`git checkout main`/
  `git merge --ff-only`/`git push origin main`、全て成功。最終確認
  (このセッション末尾): `git status` → "nothing to commit, working tree clean"、
  `git log -1` → `d3d2f20`が`main`の最新コミット。

## 7. 発生中のエラー

なし。`git status`はクリーン、常駐プロセス・ゾンビプロセスも現在ゼロ
(`pgrep`で確認済み)、load averageも正常範囲(1.84、20コア環境)。

## 8. 事実として確認できたこと

- Quad-SDK版Step 01の成功条件(0.3 m/s・10m以上・転倒なし)は**達成した**
  (1回のみ、再現性は未確認)。
- 「MUMPSの数値精度不足」が転倒の原因だという証拠は、調査全体を通じて
  **一度も見つからなかった**。実際の根本原因は起動シーケンス(コントローラ
  起動待ち)と地面サイズだった。
- `external/Quadruped-PyMPC`本体には**一切コード変更がない**
  (`git diff --stat`で確認済み、空)。
- `artifacts/logs/quadsdk_step01/`(680MB超、mp4録画・CSV)と
  `artifacts/gifs/*.gif`は**ローカルディスクにのみ存在し、gitには一切
  含まれていない**(意図的にgitignore対象)。次のセッションが別環境・
  別ディスクの場合、これらの生データ・GIFは失われる。
- `agent_reports/`ディレクトリは以前アーカイブ済み(`archive/`へ移動)で、
  リポジトリルートには存在しなかった(本資料作成のため新規作成した)。

## 9. 推測・未確認事項

- 0.5〜1.1 m/sでの歩行安定性の限界がどこにあるか(既定のMPCゲイン・歩容
  パラメータがこの速度帯向けにチューニングされていない可能性があるが未検証。
  ゲイン調整は制約により未実施)。
- プロセスグループkillが一部の子ノードに効かない理由(ROS2/DDSのプロセス
  グループ・シグナルハンドリングの何らかの癖と推測されるが未調査)。
- `big_flat.xml`の詳細メッシュ地形が不安定化を招いた具体的メカニズム
  (メッシュ衝突判定の数値的脆さ、等と推測しているが未確認)。

## 10. 次に行う作業を1ステップだけ

**10m規模試験(0.3 m/s・DURATION_S=40程度)を、現在の最終設定
(`flat_wide.xml`・`joint_controller`起動待ち・カメラ設定込み)のまま
最低3回連続実行し、再現性(連続成功率)を確認する。** 実行前後で
プロセス残留(6-3節のパターン)とload averageを必ず確認すること。

## 11. 次のチャットで最初に読むべきファイル

1. `agent_reports/step01/quad_sdk_step01_investigation.md` — 全経緯・根拠となる実測値
2. `agent_reports/step01/quad_sdk_step01_changes_and_usage.md` — 変更点・実行方法の要点
3. `scripts/trial/run_quadsdk_step01_baseline.sh` — 現在の実行スクリプト本体
4. 本ファイル(`agent_reports/handoff/quadsdk_step01_handoff.md`)
