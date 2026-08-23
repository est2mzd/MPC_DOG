# Conversation Coverage Map

会話の質問がどの章へ入ったかの監査表である。本文の正本は各章である。

照合コミット: `external/legged_control` の `a7f381c0367e98e31c01336e678eef47e304d40d`。

| ID | 質問 | 統合先 | 状態 |
|---|---|---|---|
| Q1 | データ流れを、サイズ付き・可能ならmermaid・各ブロックの処理付きで整理する | [02](02_System_Architecture_and_Dataflow.md)。周期は[01](01_Packages_and_Control_Loop.md)。変数表は[A](appendices/A_Variable_Dictionary.md) | 済 |
| Q2 | 各ブロックの背景・目的・数式・ロジックをコードと紐付けて解説する | ①② [03](03_User_Command_and_Reference.md)、③ [04](04_State_Estimation.md)、④ [05](05_NMPC.md)、⑤ [06](06_WBC.md)、⑥⑦ [07](07_Joint_Control_and_Hardware.md) | 済 |
| Q3 | 速習PPTとじっくりPPT | [slides/01_quickstart](slides/01_quickstart_legged_control.pptx)、[slides/02_deep_dive](slides/02_deep_dive_legged_control.pptx) | 済 |

ユーザー原文は [analysis-logs/00_user_chat_prompts.md](analysis-logs/00_user_chat_prompts.md) に変更せず置いた。
