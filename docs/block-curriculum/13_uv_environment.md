# 実装環境（uv）

進める順番の正本は [00](00_README.md) である。本章は **uv で環境を作る手順** である。

## 1. 前提

- Python **3.11**（`pyproject.toml` の `requires-python`）
- リポジトリ根: `mpc_dog`
- シミュレータ: MuJoCo。Go2 モデルは `gym-quadruped` 同梱 XML

## 2. 作成

リポジトリ根で:

```text
uv sync --extra workshop
uv pip install -e .
```

`--extra workshop` は Jupyter、`imageio`、カーネル用である。学習 Notebook に使う。  
`-e .` は `src/mpc_dog` を import できるようにする。

確認:

```text
uv run python -c "import mpc_dog, mujoco, gym_quadruped; print('ok')"
```

ヘッドレス描画（Notebook / GIF）では、先に OpenGL バックエンドを決める。

```text
export MUJOCO_GL=egl
```

`egl` が無いマシンでは `osmesa` を試す。コード側でも未設定ならこの順で探す（`mpc_dog.viz.gl`）。

## 3. Notebook の実行

```text
cd docs/block-curriculum/notebook
uv run jupyter nbconvert --to notebook --execute 00_mujoco_go2_demo.ipynb --inplace
```

または JupyterLab でセルを順に実行する。成功したら [00](00_README.md) §7 どおり origin の `main` に push する。
