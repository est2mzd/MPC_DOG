# Cursor Analysis Workflow

## 1. 目的

Cursorを、数式・コード・実験を同期させる分析環境として使う。資料の入口は[00](00_README.md)。境界は[02](02_System_Architecture_and_Dataflow.md)。関数木は[16](16_Code_Map_and_Call_Graph.md)。

## 2. 最初に渡すContext

1. `00_README.md`
2. `02_System_Architecture_and_Dataflow.md`
3. `16_Code_Map_and_Call_Graph.md`
4. 対象章1つ
5. その章の対応コードだけ（[D](appendices/D_File_Function_Index.md)）

リポジトリ全体を毎回渡さない。

## 3. 今回有効だった分析手順

この学習資料を作るときに繰り返した順である。飛ばすと推測が増える。

| 段 | 作業 | 何をするか | 成果の置き場 |
|---|---|---|---|
| 1 | Baseline固定 | wrapper HEAD、PyMPC tree識別、`config.py`の標準フラグ、gym版、XML経路を記録する | [00](00_README.md) §3、[analysis-logs/01](analysis-logs/01_baseline.md) |
| 2 | Call graph | `run_simulation`から`mj_step`まで、実際に呼ばれる関数だけを辿る。無効経路は条件と一緒に切る | [16](16_Code_Map_and_Call_Graph.md) |
| 3 | 変数追跡 | 各境界で生成元、shape、単位、frame、周期、次の使用先を書く | [02](02_System_Architecture_and_Dataflow.md)、[A](appendices/A_Variable_Dictionary.md) |
| 4 | 数式再構成 | コード行を式にする。符号、Mask、clip、frameを省略しない | 正本章 + [B](appendices/B_Equation_Index.md) |
| 5 | 制約分類 | Hard / Soft / Costのみ / 出力後処理。接触`c=0`で何が残るかを分ける | [07](07_MPC_Formulation.md)、[09](09_MPC_Output_and_Receding_Horizon.md) §6 |
| 6 | Default/Optional分類 | ディスク既定でONか、フラグOFFか、コメントアウトか、未実装か | [16](16_Code_Map_and_Call_Graph.md) §4、[C](appendices/C_Parameter_Index.md) |
| 7 | 解析ログ | 照合表は`analysis-logs/`へ。本文へ転記しない | [analysis-logs](analysis-logs/README.md) |
| 8 | 修正計画 | Critical → High → データフロー → 数式 → Medium。正本を先に決める | [19](19_Conversation_Coverage_Map.md)、[E](appendices/E_Corrections_and_Clarifications.md) |
| 9 | Markdown更新 | コードを正とする。誤りは置換。旧理由はE。未確定はF。重複は正本リンク | 各本文 |
| 10 | 機械検証 | リンク、コード参照、Mermaid区切り、表列数 | `python3 docs/qpympc-study/scripts/verify_study_docs.py` |
| 11 | 実験設計 | 主変数1群、Baseline比、目標GRFと実GRFを分離。合格数値は根拠があるものだけ | [18](18_Experiments_and_Research_Roadmap.md) |

プロンプト例（対象を埋める）:

```text
Markdownを正解にしないでください。コードを正本にしてください。
実装事実 / 理論 / 推奨改善 / 未確認 を分けてください。
推測で穴を埋めないでください。
```

変数追跡・数式・制約は上表の3–5を、対象の変数名または関数名を入れて依頼する。

## 4. 更新規則

- 識別子は wrapper の `mpc_dog` HEAD と、`external/Quadruped-PyMPC` の tree 識別（git外）を記録する。[00](00_README.md) §3
- 監査中の照合は `analysis-logs/` に残す。ログを本文へ転記しない
- 同じ数式を複数章へコピーせず、正本へリンクする
- 実装にない改善は「推奨改善」と書く
- 標準OFFを標準動作として書かない
- 制御コードは、ユーザーが明示するまで変更しない
- Mermaidは横長にしない。Edgeに意味・変数・単位を書く。閉ループ図の正本は`02`

## 5. Git運用

- 1テーマ1Branch
- 1実験1Config
- Markdownと制御コードを同じCommitに混ぜる場合は理由を書く
- 自動生成acados codeはレビューから分離
- ユーザーが依頼するまでcommitしない

## 6. 完了条件

対象章について、次に答えられればその章は手の内化できている。

- 入力は誰が作るか
- 何を解くか
- 出力は何か
- 次に誰が使うか
- どの数式がどの関数か
- どのパラメータを変えると何が変わるか（[C](appendices/C_Parameter_Index.md)）
