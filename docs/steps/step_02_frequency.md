# Step 02：歩容周波数(step frequency)と前進速度の関係(平面マップ)

対象commit: `external/Quadruped-PyMPC` = Step 01 と同じ
`cc145a2d353db4c39df4b49e6624959acc4b87b0`。`external/` 配下は**一切変更していない**
(Step 01 と同じ制約)。歩容周波数の上書きは公式コードと同じく
`qpympc_cfg.simulation_params['gait_params'][gait]['step_freq']` の dict を
`QuadrupedPyMPC_Wrapper` 構築**前**に書き換える方式。

## 1. 目的

平面マップ入力(`scene="flat"`、`visual_foothold_adaptation="blind"` =
地形の起伏を見ない盲目歩行)の上で、**前進トロットが転倒せず成立するか**を
確認する。Step 01 は `ref` 速度ゼロの静止立位の基準記録だった。Step 02 は
前進速度指令を入れ、歩容周波数 `step_freq` との組み合わせを見る。

ハーネス: `src/trial/step_02_frequency.py`(実行: `bash scripts/trial/run_step_02.sh`)。
`run_simulation()` の内側ループ(`simulation.py` 169–327行、commit cc145a2)を
**制御ロジックは変更せず同じ順序で**呼び出し、CSV・GIF・成否判定を出す。
環境変数 `STEP02_VEL` / `STEP02_FREQ` / `STEP02_SECONDS` で上書き可能。

## 2. 事実:速度と歩容周波数のスイープ結果(平面・8s)

`compute_actions()` 呼び出し列は Step 01 と同一のまま、
`(ref 前進速度, step_freq)` を振った(転倒判定 = base 高さ < 0.15 m
または |roll|/|pitch| > 0.8 rad が一度でも成立):

| ref 速度 [m/s] | step_freq [Hz] | 結果 | 前進距離 [m] | min z [m] | max tilt [rad] |
|---|---|---|---|---|---|
| 0.3 | 1.4 | OK | 2.39 | 0.290 | 0.04 |
| 0.5 | 1.4 | OK | 3.79 | 0.290 | 0.06 |
| 0.7 | 1.4 | OK | 4.91 | 0.284 | 0.19 |
| 0.9 | 1.4 | 際どい | 4.24 | 0.264 | 0.38 |
| **1.1** | **1.4** | **転倒(0.57 s)** | 0.79 | 0.079 | 3.14 |
| 0.5 | 1.0 | 転倒(1.21 s) | 0.08 | 0.083 | 3.14 |
| 0.5 | 2.0 | OK | 3.75 | 0.290 | 0.04 |
| 0.7 | 2.0 | OK | 5.17 | 0.290 | 0.04 |
| 1.0 | 2.5 | OK | 7.29 | 0.290 | 0.04 |
| **1.1** | **2.5** | **OK** | 7.94 | 0.290 | 0.05 |

**要点**: 当初ハーネスの既定 `(1.1 m/s, 1.4 Hz)` は 0.57 s で転倒する。
`step_freq` が速度に対して低すぎると 1 歩が長くなりすぎて足を置ききれない。
**速度に合わせて `step_freq` を上げると安定する**(`(0.5, 1.0)` の転倒も
「速度に対して周波数が低い」側の失敗)。

## 3. 事実:採用した既定値と本記録(10 s)

既定を **`INITIAL_FORWARD_VEL_MPS = 1.1`、`GAIT_STEP_FREQ_HZ = 2.5`** に変更した
(当初は `1.4`)。この設定で 10 s 記録:

- `verdict`: **PASS**(転倒なし、指令速度の 6 割以上前進)
- `walk_dist_x` = **9.95 m**(目標 `1.1 × 10 = 11.0 m` の約 90%)
- `fall_time_s` = なし
- base 高さ z: min 0.290 / mean 0.301 / max 0.322 m(公称 ≈ 0.30、沈み込みなし)
- |roll| max 0.016 rad(≈ 0.9°)、|pitch| max 0.048 rad(≈ 2.7°)
- 整定後の実 vx ≈ 1.01 m/s(指令 1.1 の約 92%)、y ドリフト 0.40 m / 10 m
- `compute_actions()` 実測時間: mean 2.2 ms / max 5.9 ms
- 生成物:
  - `artifacts/logs/step_02/state_log.csv`(4999 行、Step 01 と同じ列 + `gait_step_freq_hz`)
  - `artifacts/logs/step_02/trials_summary.csv`(`id, velocity_mps, gait_step_freq_hz,
    sim_time_s, walk_dist_x_m, walk_dist_y_m, fall_time_s, verdict`)
  - `artifacts/logs/step_02/gif_meta.json`
  - `artifacts/gifs/step_02_{id}.gif`(固定 fps=10、480×270、時刻・周波数焼き込み)
  - CSV / GIF は `.gitignore` 対象(ローカルのみ)

## 4. ハーネスの変更点(MPC_DOG 側のみ、`external/` 不変)

`src/trial/step_02_frequency.py`:

- docstring・GIF オーバーレイ・GIF ファイル名を "Step 01" → "Step 02" に修正
  (コピー由来の誤記だった)
- `NUM_SECONDS` / `INITIAL_FORWARD_VEL_MPS` / `GAIT_STEP_FREQ_HZ` を
  環境変数(`STEP02_SECONDS` / `STEP02_VEL` / `STEP02_FREQ`)で上書き可能に
- 既定 `GAIT_STEP_FREQ_HZ` を `1.4` → `2.5`(2 節の理由)
- 転倒判定を追加(reset はしない。最初に閾値を割った sim 時刻だけ記録)
  - `FALL_HEIGHT_THRESHOLD_M = 0.15`、`FALL_TILT_THRESHOLD_RAD = 0.8`
- 実行末尾で PASS/FAIL を判定し、終了コード(0/1)と `trials_summary.csv` の
  `verdict` 列に反映
- `compute_actions()` の**引数・順序・呼び出しは Step 01 と完全に同一**
  (制御ロジックは触っていない)

## 5. 推測・未確認

- **`step_freq` を上げれば任意速度まで安定するわけではない**はず(1 歩が短く
  なりすぎる/計算が間に合わない上限があるはず)。1.1 m/s 超は未検証。
- go2 の `trot` の他パラメータ(duty factor、swing height 等)は既定のまま。
  これらと `step_freq` の相互作用は未検証。
- `blind`(地形非考慮)前提。起伏マップ入力での挙動は Step 02 の対象外。
- 転倒判定はしきい値ベースの簡易版で、公式の `is_terminated` とは基準が異なる。

## 6. 関連

- 歩容(接触スケジュール・着地位置)と MPC の関係の理論・コード:
  `agent_reports/quadsdk_step01_gait_and_mpc.md`(Quad-SDK 側の解説だが
  役割分担の考え方は共通)
- MPC の基礎:`agent_reports/quadsdk_step01_mpc.md` の付録
