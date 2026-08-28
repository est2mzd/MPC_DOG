# archive/

過去の作業で生成された、現在は上位互換のファイルに置き換わった中間生成物
(下書き・使い捨てスクリプト)を、削除せずに隔離しておく場所です。

- 中身は**参照用**として残しています。日常的に開く必要はありません。
- `git mv`で移動したものだけなので、各ファイルの変更履歴(`git log --follow`)は
  そのまま追跡できます。

## 2026-08-29(さらに続き):`scratch/`・`AGENTS.md`・`README.md`もarchive

ユーザーの指示により、直前のarchiveで新設したばかりの`scratch/`
(使い捨て作業の置き場、`README.md`2枚のみ追跡)と、ルート直下の
`AGENTS.md`・`README.md`も`archive/root/`・`archive/scratch/`へ移動しました。
これでリポジトリのルートは`agent_reports/`・`archive/`・`external/`・`papers/`
と最小限のプラムビング(`.gitignore`・`.gitmodules`・`pyproject.toml`・
`uv.lock`・`logs/`)だけになっています。**この`archive/README.md`が現時点で
唯一のルート近傍の説明文書**です。

- `archive/root/README.md`・`archive/root/AGENTS.md`:元のリポジトリ直下の
  `README.md`・`AGENTS.md`(パスの衝突を避けるため`root/`配下に格納)
- `archive/scratch/`:方針Bで作った`scratch/README.md`・
  `scratch/external/README.md`(中身のルール自体はまだ有効な考え方だが、
  一旦保留)
- `.gitignore`の`scratch/*`関連ルールは今回削除していません(該当パスが
  存在しないだけで無害)

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

## 2026-08-29(続き):残り全ファイル・空フォルダの一括archive

上記の`.py`/`.ipynb`/`.sh`移動で空になった、あるいは元から`.md`等しか
持たなかったディレクトリを、ユーザーの指示(「サブの不要なフォルダ、不要
ファイル、`.md`・`.ipynb`も対象」)により丸ごと`archive/`へ移動しました
(393ファイル)。対象:`docs/`(168)・`notebook_pympc/`(93、
`benchmark_results*/`・`README.md`)・`notebook_legged/`(68、`assets/`・
`README.md`)・`notebook/`(43、`assets/`)・`configs/`(19、
`pympc_presets/*.yaml`)・`prompts/`(1)・`src/legged_control_mujoco/models/a1.xml`(1)。

移動後、`notebook/`・`notebook_pympc/`・`notebook_legged/`・`configs/`・
`prompts/`・`src/`・`scripts/`・`tests/`はgit管理対象が0件になったため、
ディレクトリ自体を削除しました(`__pycache__`・`*.egg-info`等の
gitignore対象ビルド生成物のみが残っていたことを確認済み)。

`docs/`配下には、ビルドキャッシュ(`_eqcache/`・`_figcache/`・
`__pycache__`)やワークショップの中間生成物
(`.gitignore`に明記の`assets/frames_*/`・`tuning_lab_results.json`等、
「スクリプトで再生成可能」と既に注記されていたもの)、LibreOfficeの
ロックファイルなど、**未追跡(untracked)の生成物のみ**が3191ファイル
(147MB)残っていたため、`git status`で追跡外であることを確認した上で
`docs/`ごと削除しました。

`papers/`(翻訳成果物`Nonlinear_Model_Predictive_Control_for_Quadrupedal.md`+
元論文PDF)は、トライアル・分析の副産物ではなく明示的に依頼・保持されている
参照文書のため、今回のarchive対象から**除外**しています。ルート直下の
`README.md`・`AGENTS.md`も、現役の案内文書として除外しています。

**追加の既知の影響**:上記に加え、`README.md`・`AGENTS.md`が参照している
`notebook_pympc/`・`docs/`配下のノートブック・ガイド類へのリンク・パスは、
すべて`archive/`配下へ移動済みのため無効になっています(今回は未修正)。

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
