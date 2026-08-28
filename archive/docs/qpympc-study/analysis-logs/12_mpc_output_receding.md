# Log 12: MPC出力と Receding Horizon

対応プロンプト: `solver.get` → u0 / x / GRF / foothold / mask / 保持 / shift。本文未修正。

## 内部解

| Solver変数 | Shape | 内容 | Horizon方向 |
|---|---|---|---|
| `x` stage 0..N | (30,) | 状態。出力は `[0:24]` を主に使う | 0=現在（固定）、N=終端 |
| `u` stage 0..N-1 | (24,) | 足速度12 + GRF12 | 0=即時入力 |
| `p` stage 0..N-1 | (29,) | 接触・μ・proximity・base・wrench・I・m | 段ごと固定 |
| `yref` 0..N-1 | (54,) | x参照+u参照 | 段ごと |
| `yref` N | (30,) | x参照のみ | 終端 |

acadosは全段を保持する。明示的な系列shift関数は `use_warm_start=False` のため呼ばれない。

## 外部出力

| 出力変数 | Shape | 単位 | Frame | 抽出元 | Mask | 次の使用先 |
|---|---|---|---|---|---|---|
| `control` / `u0` | (24,) | 混在 | W* | `get(0,"u")` | なし | GRFだけ下流。足速度は捨てる |
| `optimal_GRF` | (12,) | N | W* | `u0[12:]` | 後で脚ごと | interface |
| `nmpc_GRFs.*` | (3,)×4 | N | W | 上 | `* c_0` | `-J.T @ F` |
| `nmpc_footholds.*` | (3,)×4 | m | W | 下節 | なし（clipあり） | STC TD / 立脚des_foot |
| `nmpc_predicted_state` | (24,) | 混在 | W | `get(k,"x")[0:24]`, `k=2`（dt≤0.02） | 足部分をfootholdで上書き | IK代入はコメントアウト |
| `nmpc_joints_*` | None | — | — | nominal | — | 未使用 |
| `best_sample_freq` | scalar | Hz | — | 入力 `pgg.step_freq` | — | 標準では周波数更新せず |
| `status` | int | — | — | `solve()` | — | 1/4でfallback |

\*OCP内部は原点相対。GRFは並進不変。footholdは `+ initial_base_position`。

## 特に確認した8点

1. **実行される入力はどれか?** 先頭 `u0` のGRF12成分だけ。`u1...u_{N-1}` は実行しない。足速度12は低レベルへ渡さない。
2. **足先速度の最初の入力はどこで使われるか?** OCP内部の \(\dot p_i\) だけ。Swing actuatorには使わない。Swingは lift-off→`nmpc_footholds` のCartesian軌道。
3. **`nmpc_footholds` の抽出元?**
   - 現在立脚: scaling後の現在足（=実足を原点相対にしたもの）をdecenter
   - 現在遊脚: horizon上で `c_j != c_{j-1}` となる最初の予測状態 `x_j` の足位置。無ければ参照
   - `x1` 固定ではない。`dt<=0.02` の `k=2` は `nmpc_predicted_state` 用
4. **立脚と遊脚でFoothold出力が違うか?** はい。立脚=現在足（最適化しない）。遊脚=次TD予測（±0.15 m clip）または参照。
5. **100 Hz / 500 Hz は正しいか?** はい。`step_num % round(1/(100*0.002)) == 0` → 5stepに1回。低レベルは毎0.002 s。
6. **非更新周期の保持?** wrapperメンバ `self.nmpc_GRFs`, `self.nmpc_footholds` をそのまま `compute_stance_and_swing_torque` へ。Zero-order hold。
7. **Jacobianは毎周期更新か?** はい。`simulation.py` が毎step `env.feet_jacobians` を取り、トルクを再計算する。MPC解は保持、Jは更新。
8. **Solver failureで前回解か?** GRFは `previous_optimal_GRF`。solverは `reset()`。遊脚footholdは参照に戻す。前回の最適足系列全体は使わない。死文の `mg/n_s` 代入あり。

## `09` 照合

| 記載 | 判定 | 差分 |
|---|---|---|
| 実行は先頭u | 正しい | 足速度未使用を明記済み |
| `perform_scaling` は平行移動 | 正しい | なし |
| 遊脚teleport | 正しい | なし |
| mask `F*=c_0` | 正しい | なし |
| 100/500 と `% 5` | 正しい | なし |
| `nmpc_predicted_state` は k=2 | 正しい | IK未使用も記載済み |
| Failure | 「前回GRFや基準鉛直」と07が曖昧 | `09`はmask正本でfailure詳細は07。実装は前回GRF |
| 明示warm start | 標準オフ | `09`はshiftを課題にしている。実装に自動shift関数なし |

## Receding の実体

各MPC周期:

1. 新しい測定で `x0` を固定
2. 新しい `contact_sequence` と `yref` を全段に書き込み
3. SQP 1回
4. `u0` GRF と次TDを出力
5. 4回のsim stepは同じGRF/TD、新しいJでトルク

これが固定schedule receding horizon の実装である。
