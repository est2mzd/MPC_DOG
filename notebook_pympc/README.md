# PyMPC 理論・コード実行Notebook

大学院初心者が `external/Quadruped-PyMPC` を、数式・実装・小実験の順に理解する教材です。
最終到達点は、症状に応じたチューニングと、テスト可能な数式変更です。

## 起動

リポジトリ直下で:

```bash
source .env.workshop
uv run --extra workshop jupyter lab
```

## 順序

`00_curriculum_map.ipynb` から番号順に進めます。`00` で背景・目的・結論と
ASCIIデータフローを把握し、`01`–`12` で処理ブロックの式とコメント付きコードを対応させます。
`16` の最後には実リポジトリをEasy / Normal / Hard各10条件で動かした性能評価があります。

## 原則

- 現行コードと学習用単純化を混同しない
- shape・単位・座標系・脚順を常に書く
- 上流コードは `14` まで変更しない
- 1 trialで変更するパラメータ群は1つ
- 動画だけでなく、内部ログと制約marginで判断する
- 数式変更は式・実装・単体テスト・閉ループ比較を同時に設計する

## 30シナリオ性能評価

集計結果は `16_equation_modification_capstone.ipynb` の後半、機械可読な実測値は
`benchmark_results/scenario_results.json` にあります。

```bash
source .env.workshop
.venv/bin/python scripts/run_pympc_curriculum_benchmark.py
```

## 対応する詳細資料

補足説明と索引は `docs/qpympc-study/`、既存の実習は `docs/pympc_2day/` にあります。
本ディレクトリは、その内容を「自分で実行して確かめる」ための段階的な入口です。
