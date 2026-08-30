# `src/trial/step_01_baseline.py` の構造

対象ファイル: `/home/takuya/work/mpc_dog/src/trial/step_01_baseline.py`

## 役割

- `scripts/trial/run_step_01.sh` から呼ばれる、単一プロセスのPythonスクリプト
  (quad-sdk版と異なりROS2ノードではない)。
- `simulation.py`の`run_simulation()`内側ループ(commit `cc145a2`時点、
  169〜327行目)を、**呼び出す関数・引数の順序を一切変えずに**そのまま
  呼び出しながら、(a) CSVへの状態・GRF・関節トルク・MPC計算時間の記録、
  (b) オフスクリーンレンダリングによるGIF用フレーム取得、を行う。
- PyMPC自体の計算式(MPC/WBC本体)はこのファイルに一切含まれていない。

## 全体構成

ファイルは大きく3つのブロックからなる。

1. 記録パラメータ定数(MPC_DOG側の設定)
2. ヘルパー関数(`_next_trial_id`)
3. `main()` 関数(記録ループ本体)

quad-sdk版(`quadsdk_step01_baseline.py`)がROS2の非同期コールバック
(`create_subscription`/`create_timer`)で組み立てられているのに対し、
こちらは`for step in range(n_steps):`の**同期的なforループ**が中心になる。
これはPyMPCが単一プロセスの同期シミュレーションループを使う設計であるため。

## 1. 記録パラメータ定数

- `NUM_SECONDS = 10` — 記録するシミュレーション実時間[秒]
- `INITIAL_FORWARD_VEL_MPS = 1.1` — 前進速度指令[m/s]の固定値
- `GIF_FPS = 10` / `GIF_MAX_WIDTH = 480` / `GIF_MAX_HEIGHT = 270` — GIF出力設定
- `OVERLAY_FONT_SIZE` / `OVERLAY_FONT` / `OVERLAY_COLOR` — GIF内テキスト
  オーバーレイの見た目
- `REPO_ROOT` / `LOG_DIR` / `GIF_DIR` / `SUMMARY_CSV_PATH` — 出力先パス

これらは**quad-sdk版と違って環境変数で上書きできない**。速度や記録時間を
変える場合はファイルを直接編集する。

## 2. `_next_trial_id(summary_path)` — ヘルパー関数

`trials_summary.csv`の既存行から次の連番id(最大id+1)を返す。
quad-sdk版の同名関数と全く同じロジック(先にこちらが作られ、quad-sdk版が
これに合わせた形)。

## 3. `main()` — 記録ループ本体

### 3-1. 初期化(simulation.py 55〜139行目相当)

- `QuadrupedEnv`を生成。**公式コードからの変更点が1つ**: `base_vel_command_type`を
  既定の`"human"`(キー入力待ち、速度0のまま)ではなく`"forward"`にし、
  `ref_base_lin_vel=INITIAL_FORWARD_VEL_MPS`を渡すことで、非対話実行でも
  前進速度指令が入るようにしている
- 重力設定・`env.reset(random=False)`はsimulation.py同一行を踏襲
- `QuadrupedPyMPC_Wrapper`を生成(MPC+WBCの実体)
- 関節トルク上限を90%に抑える`tau_limits`を計算(simulation.py同様)
- オフスクリーンレンダラ(`mujoco.Renderer`)とカメラ(`distance=2.2`,
  `elevation=-20`, `azimuth=120`)を設定。**MPC_DOG側の追加**で、
  `external/`は関与しない

### 3-2. `for step in range(n_steps):` ループ本体

各ステップで以下を順に行う(コメントに`simulation.py`の対応行番号が
明記されている):

1. **状態取得**(simulation.py 172〜205行目相当): `feet_pos`/`feet_vel`/
   `hip_pos`/`base_lin_vel`/`base_ang_vel`/`base_ori_euler_xyz`/`base_pos`/
   `com_pos`等をenvから取得。引数は1つずつ、順序も公式実装と同一
2. **PyMPCコントローラ本体の呼び出し**(simulation.py 208〜236行目相当):
   `quadrupedpympc_wrapper.compute_actions(...)`を**未変更のまま**呼び出し、
   関節トルク`tau`を得る。計算時間を`time.perf_counter()`で計測
3. **トルク制限とMuJoCoへの入力**(simulation.py 238〜251行目相当):
   `tau`を`tau_limits`でクリップし、`env.step(action=action)`でシミュレーション
   を1ステップ進める
4. **観測値取得**(simulation.py 254行目相当): `quadrupedpympc_wrapper.get_obs()`
   でMPC計算GRF(`nmpc_GRFs`)を、`env.feet_contact_state(...)`で実測接地状態と
   実測GRFを取得
5. **記録処理(MPC_DOG側の追加)**: 上記全てを1行の`dict`にまとめ
   `log_rows`へ追加。列は「base位置・姿勢・速度」「目標速度」「4脚の接地bool」
   「MPC計算GRF・実測GRF(各脚xyz)」「関節トルク(各脚3関節)」
   「`compute_actions_time_s`」。単位・座標系はsimulation.pyと同一
   (world座標系、m, m/s, rad, rad/s, N, N・m)、脚順序は`FL, FR, RL, RR`固定
   (quad-sdk版の`FL, BL, FR, BR`とは異なる)
6. **GIF用フレーム取得**: `frame_stride`ステップごとに間引いてレンダリング。
   カメラの`lookat`をロボットのxy位置へ追従させ(zは地面レベル固定)、
   経過時間・目標速度・実速度を焼き込んだ画像を`frames`へ追加
7. **転倒時のリセット処理**: コード中に存在するが**コメントアウトされている**
   (`is_terminated`/`is_truncated`時に`env.reset(random=True)`する処理)。
   実行されないため、転倒してもリセットされない

### 3-3. ループ終了後の後処理

1. `env.close()`
2. `log_rows`を`state_log.csv`へ書き出し
3. `frames`を`GIF_MAX_WIDTH`/`GIF_MAX_HEIGHT`以下に縮小してから
   `imageio.mimsave(...)`でGIFへ書き出し(無限ループ、`optimize=True`)
4. 書き出したGIFを`imageio.get_reader`で読み直し、実フレーム数・解像度・
   ファイルサイズを検証して`gif_meta.json`へ保存
5. `trials_summary.csv`へ1行追記: `id`(2桁ゼロ埋め)・`velocity_mps`・
   `sim_time_s`・`walk_dist_x_m`/`walk_dist_y_m`(最終行−先頭行の位置差)・
   `fall_time_s`(常に`None`。上記3-2-7が無効化されているため一度も
   設定されない)

## quad-sdk版との主な違い

| 項目 | pympc版(本ファイル) | quad-sdk版(`quadsdk_step01_baseline.py`) |
|---|---|---|
| 実行方式 | 単一プロセス、同期forループ | ROS2ノード、非同期コールバック |
| 速度・時間の指定 | ファイル内定数を直接編集 | 環境変数(`FORWARD_VEL_MPS`/`DURATION_S`) |
| GIF生成 | このファイル内で完結(`imageio`) | 別スクリプト(`make_gif.sh`、ffmpeg)が録画mp4から変換 |
| 脚順序 | `FL, FR, RL, RR` | `FL, BL, FR, BR` |
| 転倒時リセット | コード上は存在するがコメントアウトで無効 | リセット処理自体が無い(1試行=1回きりの記録) |
| `external/`への変更 | なし | あり(承認を得た上で複数箇所、詳細は`docs/quad_sdk_step01_changes_and_usage.md`) |
