# Study-docs verification

機械検証。制御コードは読取のみ。新規パッケージは不要。既存の `.venv` に `gym_quadruped` があれば索引に含める（インストールはしない）。

```text
python3 docs/qpympc-study/scripts/verify_study_docs.py
```

## 検査範囲

`docs/qpympc-study/00`–`19` と `appendices/A`–`F`。本ディレクトリの `verify_study_docs.py` も索引に含める。

`analysis-logs/` はリンク先としての存在確認だけ。制御コードは変更しない。

## 検査項目

1. Markdown相対リンクと同一ファイル内 `#anchor`
2. バッククォート内の `.py` / `.xml` / `func()` / Config key。クラス名は PascalCase の一部
3. Mermaid の未閉じ引用、Node ID重複、1行あたりの過剰Node、ラベル無しEdge
4. LaTeX `\(` `\[` `$` の対応
5. 表の列数。I/O・パラメータ表ヘッダの単位 / Shape / Frame / Optional 欠落（警告）

## 検出できない問題

- 数式の物理的意味、同一記号の章間意味差、Scalar/Vector/Frameの省略
- 変数名の曖昧一致（単純文字列では「未確認」）
- Mermaidの意味的データフロー（前段出力と次段入力）
- 章間の長い重複説明
- Menagerie `unitree_go2/go2.xml` は本treeに無いため `expected-missing`（本文で既記）
