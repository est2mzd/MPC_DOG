# Log 25: 最終データフロー確定

対応プロンプト: ユーザー指令からMuJoCo Feedbackまでの最終データフロー。新しい推測なし。制御コード未変更。
記録日: 2026-08-23。

本文反映: `docs/qpympc-study/02_System_Architecture_and_Dataflow.md`（実行順22段、境界表、分割Mermaid、周期、最終確認10問）。

リンクのみ: `00`, `03`, `09`, `11`, `19`。

根拠は `simulation.py`、`quadruped_pympc_wrapper.py`、`wb_interface.py`、`srbd_controller_interface.py`、`config.py`。周期と保持値はコードの条件式だけ。
