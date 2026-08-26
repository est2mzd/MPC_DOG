# 08 — acadosの「2段階ビルド」のやり方まとめ

日付: 2026-08-27
対象: `external/Quadruped-PyMPC/quadruped_pympc/acados`(2026-08-27に`submodule`として
新規cloneしたばかりの版、まだビルドしていない状態)

凡例: **事実** = 本パスでコードを直接読んで確認した内容(ファイル・行番号つき)。
**手順** = 実行すればよいコマンド(まだ実行していない、必要になったら実行する)。

---

## 0. 背景:なぜ「2段階」なのか

MPC(`controllers/gradient/nominal/centroidal_nmpc_nominal.py`)は、数値最適化の実体を
**acados**(C言語で書かれた外部ソルバー)に委譲している。C言語のソースコードはそのままでは
実行できず、コンパイルして共有ライブラリ(`.so`)に変換する「ビルド」が要る。この
「ビルド」が実は2つの独立した層に分かれている、というのが本ファイルの主題。

| | 大きいビルド | 小さいビルド |
|---|---|---|
| 対象 | acados本体(HPIPM・BLASFEO等の数値計算コア) | このMPCモデル専用の`c_generated_code/`(状態30・入力24・パラメータ29の、あなたが`centroidal_model_nominal.py`で書いた式そのもの) |
| いつ要るか | 環境構築時に1回(acadosのバージョンを上げない限り再実行不要) | MPCの状態・入力・コスト・制約の式を変えるたび |
| 誰が実行するか | 人間が手動でコマンドを打つ | `acados_template`(pipパッケージ)が**シミュレーション起動のたびに自動で**実行 |
| 実体 | `cmake` + `cmake --build --target install` | `make clean_ocp_shared_lib` + `make ocp_shared_lib` |

---

## 1. 大きいビルド:acados本体

### 1.1 上流の一般手順(`external/Quadruped-PyMPC/README_install.md` 39–45行)

pixiまたはcondaで依存関係を入れたあと:

```bash
cd quadruped_pympc/acados/
mkdir build
cd build
cmake -DACADOS_WITH_SYSTEM_BLASFEO:BOOL=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
make install -j4
pip install -e ./../interfaces/acados_template
```

さらに`.bashrc`(or `.zshrc`)へ、acadosの場所を教える環境変数を追加する必要がある
(README_install.md 47–58行):

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/path_to_acados/lib"
export ACADOS_SOURCE_DIR="/path_to_acados"
```

### 1.2 このリポジトリ(`mpc_dog`)独自の手順(**事実**、`scripts/setup_uv_workshop.sh`)

`mpc_dog`はpixi/condaではなく**uv**を使うため、実際に使うべきコマンドは上と少し違う。
`scripts/setup_uv_workshop.sh`37–58行を直接読んで確認した内容:

```bash
# acadosがすでにビルド済みなら再ビルドしない(38行の存在チェック)
if [[ ! -f "$ACADOS_DIR/lib/libacados.so" ]]; then
  mkdir -p "$ACADOS_DIR/build"
  cmake -S "$ACADOS_DIR" -B "$ACADOS_DIR/build" \
    -DACADOS_WITH_SYSTEM_BLASFEO:BOOL=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_INSTALL_PREFIX="$ACADOS_DIR"
  cmake --build "$ACADOS_DIR/build" --target install -j"$(nproc)"
fi

uv pip install -e "$ACADOS_DIR/interfaces/acados_template"
uv pip install -e "$PYMPC"   # Quadruped-PyMPC本体
```

環境変数は、手動で`.bashrc`に書く代わりに`.env.workshop`というファイルへ自動生成される
(52–58行)。使うときは`source`するだけでよい:

```bash
source .venv/bin/activate && . .env.workshop
```

### 1.3 上流の手順との違い(**事実**、両ファイルを比較して確認)

| 項目 | 上流`README_install.md` | `mpc_dog`の`setup_uv_workshop.sh` |
|---|---|---|
| `ACADOS_WITH_SYSTEM_BLASFEO` | `ON`(システムのBLASFEOを使う) | `OFF`(acados同梱のBLASFEOをビルドする) |
| pipコマンド | `pip install -e ...` | `uv pip install -e ...` |
| 再ビルド判定 | 記載なし(常に手動実行を想定) | `lib/libacados.so`の有無で自動スキップ(38行) |
| 環境変数の設定場所 | `.bashrc`/`.zshrc`に手動追記 | `.env.workshop`を自動生成、`source`するだけ |

### 1.4 今すぐ実行するなら(**手順**、まだ実行していない)

新規cloneした`external/Quadruped-PyMPC`はまだビルドされていない
(`quadruped_pympc/acados/lib/libacados.so`が存在しない)。このリポジトリの流儀に沿うなら:

```bash
cd /home/takuya/work/mpc_dog
./scripts/setup_uv_workshop.sh
```

これ1つで、依存インストール・acadosビルド・`.env.workshop`生成まで一括で行われる
(体感30〜90分、大半はacadosのコンパイル時間)。

---

## 2. 小さいビルド:このMPCモデル専用のコード生成+コンパイル

### 2.1 呼び出し元(**事実**、`centroidal_nmpc_nominal.py` 57–61行)

```python
self.acados_ocp_solver = AcadosOcpSolver(
    self.ocp, json_file=self.ocp.code_export_directory + "/centroidal_nmpc" + ".json"
)
```

`build=False`・`generate=False`が指定されていない。これらのフラグが指定されている箇所は
同ファイルの`reset()`(553行付近)だけであり、そちらは「今あるコードをそのまま使う」動作。
つまり**通常の起動経路(`__init__`)では、毎回コード生成とコンパイルをやり直す**のが
デフォルト。

### 2.2 内部で何が起きるか(**事実**、`acados_template`のソースを直接読んで確認)

`external/Quadruped-PyMPC/quadruped_pympc/acados/interfaces/acados_template/acados_template/`
配下、`AcadosOcpSolver.__init__`(`acados_ocp_solver.py`198行)は既定で`build=True`,
`generate=True`。`build()`メソッド(142–167行)の中身:

```python
if cmake_builder is not None:
    cmake_builder.exec(code_export_dir, verbose)
else:
    verbose_system_call([make_cmd, 'clean_ocp_shared_lib'], verbose)
    verbose_system_call([make_cmd, 'ocp_shared_lib'], verbose)
```

`centroidal_nmpc_nominal.py`は`cmake_builder`を渡していないため`else`側が実行される。
つまり実際に走るのは`c_generated_code/`ディレクトリの中での

```bash
make clean_ocp_shared_lib
make ocp_shared_lib
```

というコマンドで、これを`verbose_system_call()`(`utils.py`、内部で`subprocess`を使用)が
**Pythonから自動で**呼んでいる。この`Makefile`自体も、直前の`generate()`ステップ
(`acados_ocp.py::render_templates`)がCasADiの式からテンプレート展開して生成したもの。

### 2.3 大きいビルドとの関係(**事実**)

この小さいビルドがリンクする先は、1章で作った`lib/libacados.so`等(大きいビルドの成果物)。
大きいビルドが済んでいない状態でシミュレーションを起動すると、この`make`コマンド自体は
走るが、リンクの時点で`libacados.so`が見つからずエラーになる(**解釈**、実際にエラー
メッセージを再現して確認したわけではない)。

---

## 3. 実務上のまとめ

- **環境構築で1回だけ**やること: `./scripts/setup_uv_workshop.sh`(大きいビルドを含む)
- **MPCの式(`centroidal_model_nominal.py`)を変えるたび**: 何もしなくてよい。次に
  `simulation/simulation.py`を実行した瞬間、`AcadosOcpSolver`が自動で
  `c_generated_code/`を作り直し、`make`でコンパイルし直す。体感は数秒〜1分程度
  (式の複雑さによる、**解釈**・実測はしていない)。
- **acados自体のバージョンを上げたとき**(`quadruped_pympc/acados`のsubmodule参照先を
  更新したときなど)は、`lib/libacados.so`を消すか`setup_uv_workshop.sh`の存在チェックを
  無視して大きいビルドをやり直す必要がある(**解釈**、38行の存在チェックのロジックからの推論)。

## 関連

- [07_code_reading_order_v3.md](07_code_reading_order_v3.md) — このMPCのコードを読む順序(全体)
- `external/Quadruped-PyMPC/README_install.md` — 上流の一般的なインストール手順
- `scripts/setup_uv_workshop.sh` — このリポジトリ独自のuvベースのセットアップ手順
