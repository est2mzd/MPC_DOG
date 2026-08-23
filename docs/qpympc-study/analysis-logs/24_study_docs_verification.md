# Log 24: 修正後 study docs の機械・意味検証

対応プロンプト: 修正後 `docs/qpympc-study/` のリンク、コード参照、Mermaid、数式、表、重複を検証。制御コード未変更。Git commitなし。
記録日: 2026-08-23。

## スクリプト

- ファイル: `docs/qpympc-study/scripts/verify_study_docs.py`
- 実行: `python3 docs/qpympc-study/scripts/verify_study_docs.py`
- 範囲: `00`–`19` と appendices A–F（26ファイル）
- 依存: 標準ライブラリのみ。既存 `.venv` の `gym_quadruped` があれば索引に含める

最終実行: exit 0。links 248 bad=0。code refs 579、hard missing 0、expected-missing 2（Menagerie `unitree_go2/go2.xml`）。LaTeX区切り対応。表列数一致。I/Oヘッダ欠落なし。Mermaid 2図とも構文ok、未ラベルEdgeなし。

## 検出できないもの

数式の物理意味、記号の章間差、Scalar/Vector混同、Mermaidの意味的入出力、章間の長い重複、変数の役割（同名異義）。

## 今回直した本文

- `00`/`02` Mermaid: 接触記号とNode ID衝突、`current_contact`欠落、各脚shape
- `08` 並進式を06と揃える（`F_ext`、`g`ベクトル）
- `10` Stanceは \(F^{cmd}\)（Mask後`nmpc_GRFs`）
- `11` \(F^{MPC}\) / \(F^{cmd}\) / \(\lambda\) を分離
- `19` 遊脚GRFの正本を09 §6へ
- I/O表に単位・Shape・Frame・Optionalを補完（01, 04, 05, 06, 07, 08, 09, 10, 11, 13, 18, C）

## 意味判定の要約

正本分割は維持。残る未確認変数は `xfrc_applied`（MuJoCoフィールド。実験案）のみ機械的に「未確認」。`ref_foot_constraints_FL` はキーワード引数としてコードに存在。
