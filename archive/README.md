# archive/

過去の作業で生成された、現在は上位互換のファイルに置き換わった中間生成物
(下書き・使い捨てスクリプト)を、削除せずに隔離しておく場所です。

- 中身は**参照用**として残しています。日常的に開く必要はありません。
- `git mv`で移動したものだけなので、各ファイルの変更履歴(`git log --follow`)は
  そのまま追跡できます。

## 2026-08-29:`.py`/`.ipynb`/`.sh`全ファイルの一括archive

`external/`(submodule)を「ゼロから」検証し直す前提として、ユーザーの明示的な
指示により、`external/`・`archive/`自身を除く**すべての`.py`/`.ipynb`/`.sh`**
(122ファイル)を`archive/`配下へ移動しました。対象:`notebook/`・
`notebook_pympc/`・`notebook_legged/`の全`.ipynb`、`scripts/`・`tests/`・
`src/`の全`.py`/`.sh`、`docs/pympc_2day/notebooks/`・
`docs/legged_control/slides/`・`docs/qpympc-study/{scripts,slides}/`の
`.py`/`.ipynb`。

**既知の影響(意図的に未修正)**:

- `pyproject.toml`の`[tool.setuptools.packages.find] where=["src"]`が
  `.py`を含まなくなったため、`pip install -e .`はパッケージを見つけられない
- `[tool.pytest.ini_options] testpaths=["tests"]`が空になったため、
  `pytest`は収集0件になる
- `notebook/`・`notebook_pympc/`・`notebook_legged/`・`docs/`配下には、
  `.py`/`.ipynb`/`.sh`以外のファイル(`README.md`・`assets/`・
  `benchmark_results/`等)がそのまま残っている(今回の指示範囲外のため)

## `archive/notebook/`(方針A、旧)

`notebook/`にあった`_append_*`・`_exec_*`・`_patch_*`・`_probe_*`・`_run_*`
という使い捨てスクリプト群(26個)。ノートブック(`00`〜`14`)を作る過程の
試行錯誤の産物で、正式な成果物は各ノートブックに統合済みです(この直後の
一括archiveで、統合先のノートブック自体も`archive/notebook/`へ移動済み)。

## `archive/agent_reports/quadruped_pympc_onboarding/`

`agent_reports/quadruped_pympc_onboarding/`にあった、版違い(`_v1`/`_v2`/`_v3`)
の下書きファイルと、方向性が採用されなかった初期案(`06_existing_docs_synthesis.md`)。
いずれも、同ディレクトリの`read_code_*.md`シリーズ(逐次コード読解の正式版)、
および`07_code_reading_order_v3.md`(アーキテクチャ概要、正式に採用された版)
に統合・置き換え済みです。
