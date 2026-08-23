# Log 04: `02` / `16` / `D` 照合

対応プロンプト: Call graph調査を学習資料3ファイルと比較する。本文未修正。

判定: 正しい / 不完全 / 誤り / コードから確認不能

| 資料 | 記載内容 | コード上の事実 | 判定 | 必要な修正 |
|---|---|---|---|---|
| `02` 全体フロー | 速度指令→Gait/Foothold/参照→MPC→立脚遊脚→トルク→MuJoCo | 一致。標準経路 | 正しい | なし |
| `02` mermaid | `_sample_ref_vel` / `_key_callback` → `target_base_vel` → `compute_actions` | 一致。キーはrender時のみ | 正しい | なし |
| `02` TerrainEstimator位置 | `update_state_and_reference` 先頭 | 一致 | 正しい | なし |
| `02` 周期 | MPC 100 Hz、低レベル/sim 500 Hz | `mpc_frequency=100`, `dt=0.002`, `% 5` | 正しい | なし |
| `16` 主要ファイル表 | simulation / wrapper / wb / srbd / PGG / FRG / TE / VFA / model / nmpc | 標準経路と一致 | 正しい | なし |
| `16` 無効経路 | VFA, start/stop, optimize_step_freq, RTI, DDP, integrators, foothold constraints, stability, reflex, sampling系 | 設定条件と一致 | 正しい | なし |
| `16` TerrainEstimator末尾 | 過去の誤り。現行は先頭と記載 | `wb_interface.py` 先頭呼び出し | 正しい（訂正済み） | なし |
| `D` `reset` / `_set_ground_friction` / `com` | 索引にある | `quadruped_env.py` に存在 | 正しい | なし |
| `02`/`16` PyMPC commit | 一部で `3adfad9` をPyMPCと書く可能性 | `3adfad9` は wrapper | 不完全 | Baselineログ参照。PyMPCはgit外 |
| `16` 関節PD | コメントアウトと書く必要 | wrapper 197–203行がコメント | 不完全 | 「実装されているが無効」と明記するとよい |
| `D` 観測キー typo | wrapperは `ref_foot_FL_constraints` を読む箇所あり | `ref_state` 実キーは `ref_foot_constraints_FL`。観測名 `ref_feet_constraints` は typo キーを参照 | 誤り（観測分岐のみ） | wrapper観測分岐を直すか、資料で「観測コードのtypo」と分離 |
| `02` mermaid全辺の型 | 概ね一致 | 一部edgeのshape省略 | 不完全 | 必要ならAへリンク |

総合: 3資料は標準経路の骨格としては使える。残るのはcommit取り違えと観測キーtypoの分離。
