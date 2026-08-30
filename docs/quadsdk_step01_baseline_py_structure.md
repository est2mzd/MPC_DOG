# `src/trial/quadsdk_step01_baseline.py` の構造

対象ファイル: `/home/takuya/work/mpc_dog/src/trial/quadsdk_step01_baseline.py`

## 役割

- `scripts/trial/run_quadsdk_step01_baseline.sh` から呼ばれるROS2ノード。
- quad-sdkが公開するトピックを購読し、Quadruped-PyMPC版の`state_log.csv`に近い
形式でCSVへ記録する。GIF/動画の記録はこのスクリプトの範囲外で、
`quad_mujoco.py`の`recording:=true`機能(mp4出力)が別途担当する。

- system python3 + ROS2(`source /opt/ros/jazzy/setup.bash`)が必要
- プロジェクトの`.venv`(uv管理)には`rclpy`が無いため、`uv run`では実行できない

## 全体構成

ファイルは大きく5つのブロックからなる。

1. モジュール定数(`LEG_ORDER`, `FALL_HEIGHT_THRESHOLD_M`)
2. ヘルパー関数(`_next_trial_id`)
3. ROS2ノード本体(`Step01Recorder`クラス)
4. エントリーポイント(`main`関数)
5. `if __name__ == "__main__":` 呼び出し

## 1. モジュール定数

- **`LEG_ORDER = ["FL", "BL", "FR", "BR"]`**
  quad-sdkの脚順序(`quad_msgs/MultiFootState.msg`のコメントに明記)。
  Quadruped-PyMPC版の順序(`FL, FR, RL, RR`)とは異なる点に注意。

- **`FALL_HEIGHT_THRESHOLD_M = 0.15`**
  転倒判定のしきい値[m]。go2の立位base高さ(約0.3m)の半分程度。
  quad-sdkには公式の`is_terminated`相当の仕組みが見つからなかったための簡易代用。

## 2. `_next_trial_id(summary_path)` — ヘルパー関数

`trials_summary.csv`の既存行を読み、次の連番id(最大id+1)を返す。
ファイルが無い/空なら1を返す。Quadruped-PyMPC版と同じ採番方式。

## 3. `Step01Recorder` クラス(`rclpy.node.Node`を継承)

### 購読するトピック(`/{robot_ns}/`配下)

- `control/grfs`(`quad_msgs/GRFArray`) — 実測GRF(接地力)
- `local_plan`(`quad_msgs/RobotPlan`) — NMPCの計算時間・反復数などの診断情報。
  NMPC解が失敗した回は publish されないため、**このメッセージが来ないこと自体が
  失敗の間接シグナル**になる
- `state/ground_truth`(`quad_msgs/RobotState`) — base位置・姿勢・速度など。
  記録の主軸で、他の2つはこのコールバック内でキャッシュ値として付加される

### `__init__`

上記3つの`create_subscription`に加えて、`duration_s`秒後に1回だけ発火する
タイマー(`create_timer`)を作る。**ノード自身のROS時計(`use_sim_time`)には
合わせていない**点が実装上のポイント: `quad_mujoco.py`の`/clock`配信タイミング
次第で`get_clock().now()`が不連続にジャンプし、タイマーが直後に発火してしまう
不具合が実際に発生したため。各行の`sim_time_s`はメッセージ自身の
`header.stamp`から計算する(`_on_state`参照)。

### `_on_grf(msg)` / `_on_plan(msg)`

直近受信したメッセージをインスタンス変数(`self._latest_grf` /
`self._latest_plan`)へキャッシュするだけ。記録行の生成は行わない。

### `_on_state(msg)` — 記録の中心処理

`state/ground_truth`受信のたびに1行分の`dict`を組み立て、`self._rows`へ追加する。

1. **時刻計算**: `msg.header.stamp`から`sim_time_s`を算出。最初に受信した
   メッセージの時刻を基準(`self._t0_msg_s`)として、以降は相対時間で記録する。
2. **base状態の抽出**: `msg.body.pose`(位置+クォータニオン姿勢)と
   `msg.body.twist`(並進+角速度)から、位置・roll/pitch/yaw(scipyでクォータニオン
   →オイラー角変換)・速度を取り出し、`row`辞書へ格納する。
3. **local_plan由来の診断値の付加**: `self._latest_plan`があれば
   `plan_age_s`(キャッシュの経過時間。値が伸び続ける=NMPC失敗が続いている
   間接指標)・`plan_compute_time_ms`・`plan_nmpc_iterations`・
   `plan_nmpc_cost`を追加。無ければ全て`None`。
4. **GRF/接地状態の付加**: `LEG_ORDER`の4脚について、`contact_{脚}`(bool)と
   `grf_{脚}_{x,y,z}_N`を追加。**`self._latest_grf`が`None`でも既定値
   (False/0.0)で必ず全列を埋める**(でないと`csv.DictWriter`が
   行ごとの列不一致で例外を出すため)。
5. **転倒時刻の記録**: `pos.z`が`FALL_HEIGHT_THRESHOLD_M`を初めて下回った
   時刻を`self._fall_time_s`へ記録(以降は上書きしない)。

### `_on_timeout()` — 終了処理

`duration_s`経過後に1回だけ呼ばれる。

1. タイマーを止め、`self._rows`を`csv_path`へ書き出す
2. `trials_summary.csv`(`summary_csv_path`)へ1行追記する:
   `id`(2桁ゼロ埋め)・`velocity_mps`・`sim_time_s`(最終行の値)・
   `walk_dist_x_m`/`walk_dist_y_m`(最終行−先頭行の位置差)・`fall_time_s`
3. `rclpy.shutdown()`でノードを終了する

## 4. `main()` — エントリーポイント

`argparse`で以下を受け取り、`Step01Recorder`を生成して`rclpy.spin()`する:

- `--robot-ns`(既定`"robot_1"`)
- `--duration-s`(既定`10.0`)
- `--csv-path`(必須)
- `--summary-csv-path`(必須)
- `--velocity-mps`(必須。ROS2からは読み取れないため呼び出し側から明示的に渡す)

`rclpy.spin(node)`は`_on_timeout`が`rclpy.shutdown()`を呼ぶまでブロックし続ける。

## 既知の注意点(コード内コメントに明記されているもの)

- `LEG_ORDER`のquad-sdk側の並びは`MultiFootState.msg`のコメントで確認済みだが、
  `GRFArray`自体の`vectors`/`points`/`contact_states`の並びが同じ前提かどうかは
  **未確認・要検証**とコメントされている
- `FALL_HEIGHT_THRESHOLD_M`は簡易的な代用指標であり、公式の転倒判定とは異なる
