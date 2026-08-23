# Log 02: `00_README.md` 対象コード節との照合

対応プロンプト: Baselineを`docs/qpympc-study/00_README.md`の「対象コード」と照合する。本文は未修正。修正案だけを残す。

現行`00_README.md` §2 の記載:

- Quadruped-PyMPC: `external/Quadruped-PyMPC` のコミット `3adfad9f814c499fb996cf046c8fb4ac3a574e55`
- gym-quadruped: 1.1.5
- Go2: gym-quadruped同梱 `go2.xml`（`nq=19`, `nv=18`, `nu=12`）
- 初版記録は`cc145a2`。差分理由は Appendix E §15
- Menagerieは上流参照、実行時未ロード

## 分類

### 1. READMEと一致

- gym-quadruped 1.1.5
- 実行時XMLはgym-quadruped同梱Go2
- `nq=19`, `nv=18`, `nu=12`
- Menagerieは実行時未ロード

### 2. READMEが古い / 誤り

- 「照合した対象は `external/Quadruped-PyMPC` のコミット `3adfad9`」は誤り。`3adfad9` は wrapper リポジトリ `mpc_dog` の HEAD。`external/Quadruped-PyMPC` に `.git` はない。
- Appendix E §15 も同じ取り違えを残している。

### 3. READMEに記録されていない

- Python 3.11.16
- MuJoCo 3.11.0
- CasADi 3.7.2
- acados_template 0.5.1
- wrapper remote / branch
- 標準Configキー（`type='nominal'`, `gait='trot'`, `dt=0.002`, `mpc_frequency=100`）
- 未Commit差分の存在

### 4. 現在の環境では確認不能

- PyMPC 展開元の正確な git commit（zip comment `cc145a2` は過去記録。現行treeとの同一性は未再ハッシュ）
- Menagerie HEAD と gym-quadruped `go2.xml` の同一性
- acados Cライブラリの上流commit

## 修正案（未適用）

```diff
- Quadruped-PyMPC: `external/Quadruped-PyMPC` のコミット `3adfad9f814c499fb996cf046c8fb4ac3a574e55`
+ wrapperリポジトリ `mpc_dog` HEAD: `3adfad9f814c499fb996cf046c8fb4ac3a574e55`
+ Quadruped-PyMPC: `external/Quadruped-PyMPC`（git管理外の展開ディレクトリ。初版記録の zip comment は `cc145a2`）
+ Python 3.11.16 / MuJoCo 3.11.0 / CasADi 3.7.2 / acados_template 0.5.1
+ 標準: `type='nominal'`, `gait='trot'`, `simulation_dt=0.002`, `mpc_frequency=100`
```
