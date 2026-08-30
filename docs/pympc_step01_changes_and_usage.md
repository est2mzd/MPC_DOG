# Quadruped-PyMPC Step 01: 元リポジトリからの変更点と実行方法

対象: `external/Quadruped-PyMPC`(Quadruped-PyMPC本体)。詳細な調査経緯は
`docs/steps/step_01_reference_baseline.md`を参照。本ドキュメントは要点のみ。

## 1. 元リポジトリ(`external/Quadruped-PyMPC`)から変更する必要があったもの

**なし。** `git status`/`git diff`で確認した通り、`external/Quadruped-PyMPC`
(および入れ子の`quadruped_pympc/acados`submodule)には一切コード変更を
加えていない。MPC本体・WBC・歩容生成・状態推定などの制御ロジックは
公式実装のまま。

ビルド時のcmakeオプションのみ、READMEの既定値と異なる選択をしている:

- `ACADOS_WITH_SYSTEM_BLASFEO`を`ON`(README記載値)ではなく`OFF`にした
  - 理由: `ON`のままだとシステムに`blasfeo`パッケージが見つからずcmake
    configureが失敗する(本環境はconda/pixiではなく`uv`管理の`.venv`のため、
    Quadruped-PyMPC側が想定するconda環境提供のシステムblasfeoが存在しない)。
    `OFF`はacados同梱のソースからビルドする設定で、acados自身のCMakeLists.txtが
    「開発者が実際にテストしているのはOFFの場合のみ」と明記している値でもある
  - これは`external/`のコード変更ではなく、ビルド時のCMakeオプション選択

## 2. 実行に必要なコード(MPC_DOG側、新規作成)

- **`scripts/trial/build_acados.sh`**
  acados(C実装、blasfeo/hpipm/acados本体)をビルドし、`acados_template`を
  editableインストールする。**1回実行すれば以後は不要**

- **`scripts/trial/install_quadruped_pympc.sh`**
  Quadruped-PyMPC本体を`uv pip install -e`でeditableインストールする
  (README_install.md手順7相当)。acadosビルド後に実行する

- **`scripts/trial/run_step_01.sh`**
  メイン実行スクリプト。`ACADOS_SOURCE_DIR`/`LD_LIBRARY_PATH`を設定し、
  `src/trial/step_01_baseline.py`を`uv run python`で実行する

- **`src/trial/step_01_baseline.py`**
  記録用ハーネス本体。`simulation.py`の`run_simulation()`内側ループ
  (呼び出す関数・引数の順序を変えずに)そのまま呼び出しながら、CSV記録と
  GIF用フレーム取得を行う。構造の詳細は
  [`docs/step_01_baseline_py_structure.md`](./step_01_baseline_py_structure.md)を参照

### 実行方法

初回のみ(ビルド・インストール、1回で以後は不要):

```bash
cd /home/takuya/work/mpc_dog
bash scripts/trial/build_acados.sh
bash scripts/trial/install_quadruped_pympc.sh
```

本実行:

```bash
bash scripts/trial/run_step_01.sh
```

出力:
- CSV: `artifacts/logs/step_01/state_log.csv`(毎回上書き)
- 試行サマリ: `artifacts/logs/step_01/trials_summary.csv`(毎回追記)
- GIF: `artifacts/gifs/step_01_{2桁連番}.gif`
- GIFメタデータ: `artifacts/logs/step_01/gif_meta.json`

速度や記録時間を変えたい場合は、`src/trial/step_01_baseline.py`冒頭の
`NUM_SECONDS`・`INITIAL_FORWARD_VEL_MPS`等の定数を直接編集する
(quad-sdk版のような環境変数化はされていない)。

### 前提条件

- `cmake`/`make`/`gcc`/`g++`が導入済みであること
- 上記2つのビルド・インストールスクリプトを実行済みであること
- `uv run`(このプロジェクトの`.venv`)で実行する。quad-sdk版と異なり、
  system python3は不要

### 既知の注意点

- `scripts/trial/run_step_01.sh`冒頭のコメントは、リネーム前のファイル名
  `record_step01_baseline.py`のまま更新されていない(実際に呼んでいるのは
  `step_01_baseline.py`)。動作には影響しないが、読む際は注意
