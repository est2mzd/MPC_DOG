# archive/

過去の作業で生成された、現在は上位互換のファイルに置き換わった中間生成物
(下書き・使い捨てスクリプト)を、削除せずに隔離しておく場所です。

- 中身は**参照用**として残しています。日常的に開く必要はありません。
- `git mv`で移動したものだけなので、各ファイルの変更履歴(`git log --follow`)は
  そのまま追跡できます。

## `archive/notebook/`

`notebook/`にあった`_append_*`・`_exec_*`・`_patch_*`・`_probe_*`・`_run_*`
という使い捨てスクリプト群(26個)。ノートブック(`00`〜`14`)を作る過程の
試行錯誤の産物で、正式な成果物は各ノートブックに統合済みです。

## `archive/agent_reports/quadruped_pympc_onboarding/`

`agent_reports/quadruped_pympc_onboarding/`にあった、版違い(`_v1`/`_v2`/`_v3`)
の下書きファイルと、方向性が採用されなかった初期案(`06_existing_docs_synthesis.md`)。
いずれも、同ディレクトリの`read_code_*.md`シリーズ(逐次コード読解の正式版)、
および`07_code_reading_order_v3.md`(アーキテクチャ概要、正式に採用された版)
に統合・置き換え済みです。
