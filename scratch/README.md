# scratch/

使い捨ての試行錯誤(検証スクリプト、途中経過のログ、ちょっとした実験)を
置く場所です。**このディレクトリの中身はデフォルトでgit管理対象外**
(`.gitignore`で`scratch/*`を無視、この`README.md`だけ例外的に追跡)です。

## ルール

1. 新しい検証・実験は、まずここで自由に試す。ファイル名や構成に迷わない。
2. 試行錯誤の結果、正式な成果物として残す価値が出たら、該当する場所へ
   **昇格**させる:
   - 恒久的なノートブック → `notebook/`・`notebook_pympc/`・`notebook_legged/`
   - 分析結果・作業ログ → `agent_reports/<group-name>/`
     (`AGENTS.md`の「Work logs」セクション参照)
   - 恒久的なスクリプト・ツール → `scripts/`・`src/`
3. 昇格させたら、`scratch/`側の元ファイルは削除してよい(gitignore対象なので
   削除してもコミット履歴を汚さない)。
4. 逆に、`notebook/`・`agent_reports/`等の正式な場所には、使い捨てスクリプト
   (`_append_*`、`_exec_*`、`_patch_*`、`_probe_*`、`_run_*`のような命名)を
   直接置かない。かつて`notebook/`直下にこの種のファイルが26個溜まって
   `archive/notebook/`へ退避する羽目になった、という反省による(詳細は
   `archive/README.md`)。

## `scratch/external/`

`external/`(Quadruped-PyMPC・legged_control)配下のコードをゼロから検証する
作業は、`scratch/external/<トピック名>/`に1トピック1フォルダで進める。
詳細は`scratch/external/README.md`参照。
