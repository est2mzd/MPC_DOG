# Step 01：参照実装の固定と公式動作の再現

対象commit: `external/Quadruped-PyMPC` = `cc145a2d353db4c39df4b49e6624959acc4b87b0`(branch `main`、`origin` = `https://github.com/iit-DLSLab/Quadruped-PyMPC.git`)。
以下の行番号・引用はすべてこのcommitに対して確認したものである。

## 1. 目的

Quadruped-PyMPCを変更せずに実行し、以後のStepで比較に使う基準動作を保存する。MPC_DOG独自の制御ロジックはこのStepでは実装しない。

## 2. 今回は実施しないこと

- `external/`配下のコード変更(実施していない、後述「事実」で無変更を確認)
- MPC_DOG独自の状態推定・MPC・WBCの実装
- 前進歩行の検証(Step 5より後)
- 実際のシミュレーション実行・ログ収集・GIF作成 — **本Stepの前提条件(後述)を満たせず未達成**。詳細は「10. 結果」「14. 未解決事項」参照。

## 3. 前提の食い違い(先に報告)

指示書2節は「現在は`external/`のみ存在する」としているが、実際のリポジトリ直下には以下が存在する(2026-08-29時点)。

```text
archive/               ← 過去の分析・ノートブック・スクリプトの隔離場所(git履歴保持)
chatgpt_instruction/    ← 本指示書ファイルが置かれているディレクトリ(未追跡)
external/               ← 前提通り存在(Quadruped-PyMPC, legged_control のsubmodule)
papers/                 ← 論文PDFとその翻訳(1件、明示的に保持されている参照資料)
.gitignore / .gitmodules / pyproject.toml / uv.lock / .venv/ 等のプラムビング
```

`archive/`・`papers/`はいずれも過去のセッションで意図的に整理・保持されたものであり、`external/`の内容や本Stepの作業には影響しない。事実として報告した上で、指示書4節の「推奨ディレクトリ構成」に従って`src/mpc_dog/`・`scripts/`・`tests/`・`configs/`・`docs/steps/`・`artifacts/`を`external/`と並べて新規作成する方針で進めた。

## 4. `external/`の実際の構成(調査結果)

```text
external/
├── legged_control/            (別submodule、本Stepの対象外)
├── Quadruped-PyMPC/            ← 本Stepの対象
└── Quadruped-PyMPC.zip-backup/ (gitignore対象、旧zip展開版のローカル残骸、git管理外)
```

`external/Quadruped-PyMPC`はgit submoduleとして登録されている(`.gitmodules`にて`path = external/Quadruped-PyMPC`、`url = https://github.com/iit-DLSLab/Quadruped-PyMPC.git`)。

| 項目 | 値 |
|---|---|
| remote URL | `https://github.com/iit-DLSLab/Quadruped-PyMPC.git` |
| branch | `main`(`origin/main`を追跡) |
| commit SHA | `cc145a2d353db4c39df4b49e6624959acc4b87b0` |
| commit日時 | 2026-08-11 22:46:02 +0200 |
| commitメッセージ | `Update mamba_environment.yml` |
| LICENSE | BSD 3-Clause License、`Copyright (c) 2025, DLS Lab at Istituto Italiano di Tecnologia, Italy`(`external/Quadruped-PyMPC/LICENSE`、1〜4行) |

さらに、`external/Quadruped-PyMPC`自身が`quadruped_pympc/acados`をsubmoduleとして持つ(`.gitmodules`内`[submodule "quadruped_pympc/acados"]`、`url = https://github.com/acados/acados.git`)。`git submodule status`で確認したpin先commitは`5d358fe80c1037a0feeb8ba1021fcd354f1be8c2`(タグ`v0.4.3-32-g5d358fe80`)。このacadosは初期化済み(ソースは存在する)だが、後述の通り**ビルドされていない**。

## 5. READMEと実行設定(公式が想定する最小実行方法)

`external/Quadruped-PyMPC/README_install.md`(1〜126行)より、公式が想定するセットアップ手順を確認した(**事実**、引用は同ファイルより)。

1. Pixi または Conda で環境を作る(`pixi install && pixi shell`、または`conda env create -f mamba_environment.yml && conda activate quadruped_pympc_env`)
2. `git submodule update --init --recursive`
3. acadosをビルドする:
   ```
   cd quadruped_pympc/acados/
   mkdir build && cd build
   cmake -DACADOS_WITH_SYSTEM_BLASFEO:BOOL=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
   make install -j4
   pip install -e ./../interfaces/acados_template
   ```
4. `LD_LIBRARY_PATH`・`ACADOS_SOURCE_DIR`を環境変数として設定する
5. `pip install -e .`(Quadruped-PyMPC自身)
6. 実行:`python3 simulation/simulation.py`(README_install.md 88行)

つまり公式が想定する最小のMuJoCo実行方法は、`external/Quadruped-PyMPC`ディレクトリで`python3 simulation/simulation.py`を実行することである(**事実**)。ロボット種別・MPCタイプ・歩容等は`quadruped_pympc/config.py`で設定する(README_install.md 91行の記載通り)。

## 6. 主要な呼び出し経路(実行入口→MuJoCo入力)

`simulation/simulation.py`の`run_simulation`関数を起点に、以下の経路を実コードから確認した(**事実**、すべて上記commitでの行番号)。

```text
simulation/simulation.py (134行) QuadrupedPyMPC_Wrapper 生成
  ↓ 183行  env.target_base_vel() ─────────────────── 速度・yaw角速度指令の取得元
  ↓ 208行  quadrupedpympc_wrapper.compute_actions(...)
      quadruped_pympc_wrapper.py (50行) compute_actions
        ↓ 115行  self.wb_interface.update_state_and_reference(...)
            interfaces/wb_interface.py (108行) update_state_and_reference
              202行  self.pgg.run(simulation_dt, self.pgg.step_freq)         ← 歩容位相の更新
              203行  self.pgg.compute_contact_sequence(...)                  ← gait/contact sequence
              213行  self.frg.update_lift_off_positions(...)
              222行  self.frg.update_touch_down_positions(...)
              231行  self.frg.compute_footholds_reference(...)               ← 遊脚の着地目標(X/Y)
              (54行のコンストラクタで self.pgg=PeriodicGaitGenerator, 67行で
               self.frg=FootholdReferenceGenerator, 77行で self.stc=SwingTrajectoryController を生成)
        ↓ 143行  self.srbd_controller_interface.compute_control(...)
            interfaces/srbd_controller_interface.py (27〜30行) self.type=="nominal" のとき
              controllers/gradient/nominal/centroidal_nmpc_nominal.py
                (1138行 compute_control 内、1445/1450行で) self.acados_ocp_solver.solve() ← MPC本体
                                                             (戻り値にGRF=nmpc_GRFsを含む)
        ↓ 172行  self.wb_interface.compute_stance_and_swing_torque(...)
            interfaces/wb_interface.py (307行)
              372〜375行  tau.FL = -feet_jac.FL[:, legs_qvel_idx.FL].T @ nmpc_GRFs.FL  ← GRF→関節トルク(立脚)
              (遊脚側はSwingTrajectoryControllerによるカルテシアン空間PD制御、307行の関数内で別途計算)
  ↓ 251行  env.step(action=action) ───────────────── MuJoCoへの入力(action=関節トルク)
```

対応表:

| 求められている経路(指示書4節) | 対応するコード |
|---|---|
| 速度・姿勢指令 | `simulation.py:183` `env.target_base_vel()` |
| 状態取得 | `simulation.py`内、`env`(`gym_quadruped.QuadrupedEnv`)から`feet_pos`・`base_lin_vel`等を取得する一連の行(`compute_actions`呼び出し直前) |
| gait/contact sequence | `wb_interface.py:202-203`(`PeriodicGaitGenerator.run`/`compute_contact_sequence`) |
| MPC | `centroidal_nmpc_nominal.py`の`compute_control`(1138行、`acados_ocp_solver.solve()`) |
| GRF | `centroidal_nmpc_nominal.py`の`compute_control`戻り値(`nmpc_GRFs`) |
| 遊脚軌道 | `wb_interface.py`内`self.stc`(`SwingTrajectoryController`、307行の`compute_stance_and_swing_torque`から呼ばれる) |
| 関節指令 | `wb_interface.py:372-375`(立脚 `tau = -J^T F`)+遊脚側PD |
| MuJoCoへの入力 | `simulation.py:251` `env.step(action=tau)` |

## 7. 関連する理論

このStepでは新規の理論導入はない。上記経路が示す通り、セントロイダルSRBD(Single Rigid Body Dynamics)モデルに基づくgradient-based NMPC(acados/SQP)がGRFを計算し、立脚は\(\tau=-J^\top F\)、遊脚はカルテシアン空間PD制御という2種類の制御則がWBC相当の役割を担う。

## 8. 元コードの対応箇所

上記6節の表・呼び出し経路の通り。個別クラスの内部実装(`PeriodicGaitGenerator`・`FootholdReferenceGenerator`・`SwingTrajectoryController`・`Acados_NMPC_Nominal`等)はこのStepでは深追いしていない(Step 1は経路の特定までが範囲)。

## 9. 変更内容と変更理由

`external/`配下は無変更(後述「12. 事実」で`git status`により確認)。MPC_DOG側で新規作成したのは、本ドキュメントおよび`scripts/run_reference_baseline.sh`(公式手順をそのまま呼び出す起動スクリプト)のみ。ロジックの実装は行っていない。

## 10. 入出力・shape・単位・座標系

`config.py`で確認した既定値(**事実**、`robot='go2'`時、9行目/85-109行目/213-241行目):

| 項目 | 値 | 出典 |
|---|---|---|
| ロボット | `go2` | `config.py:9` |
| MPCタイプ | `nominal` | `config.py:85` |
| MPCホライズン | `12`ステップ、`dt=0.02`秒 | `config.py:92-93` |
| 摩擦係数`mu` | `0.5` | `config.py:98` |
| GRF上限`grf_max` | `mass * gravity_constant`(ロボット質量依存) | `config.py:96` |
| 既定歩容 | `trot`(`step_freq=1.4`Hz、`duty_factor=0.65`) | `config.py:215-216` |
| シミュレーション`dt` | `0.002`秒 | `config.py:213` |
| 速度指令モード | `human`(キーボード入力) | `config.py:231` |
| 地形 | `flat` | `config.py:241` |

座標系・脚順序(`FL/FR/RL/RR`)・単位(m, m/s, rad, N, N·m, s)は、`external/Quadruped-PyMPC`側のコード規約にそのまま従う(このStepでは変更していないため独自定義なし)。

## 11. 実行方法

`scripts/run_reference_baseline.sh`を参照。中身は公式`README_install.md`の手順をそのまま呼び出すラッパーであり、**現時点では実行環境が未整備のため未検証**(後述12節)。

## 12. 評価指標

- `sum(実行できたか)`:実行の成否(バイナリ)
- Base状態・接触状態・GRF・関節指令・MPC計算時間のログ有無
- GIFの実時間・解像度・フレーム数・ファイルサイズ

## 13. 結果

**未達成。実行環境のセットアップ段階でブロックされ、シミュレーションを1回も実行できていない。**

環境調査で確認した事実(このマシン上):

| 項目 | 状態 |
|---|---|
| `.venv`(uv管理、Python 3.11) | 存在する |
| `mujoco` | インポート可能(v3.11.0) |
| `casadi` | インポート可能(v3.7.2) |
| `gym_quadruped` | インポート可能 |
| `acados_template` | **インポート失敗**:`SyntaxError: unknown encoding: future_fstrings`(`acados_template/utils.py`) |
| `quadruped_pympc/acados`(submodule) | 初期化済み(ソースは存在、pin先 `5d358fe80c1037a0feeb8ba1021fcd354f1be8c2`) だが**未ビルド**(`build/`ディレクトリなし、コンパイル済み`.so`なし) |
| `cmake` / `make` / `gcc` / `g++` / `cc` / `clang` | **すべて未インストール**(`command -v`で検出不可) |
| `sudo` | パスワードが必要(非対話的に利用不可、`apt`等でのツールチェイン導入も不可) |

README_install.mdの手順5(acadosのビルド)には`cmake`・`make`・Cコンパイラが必須だが、このマシンには**C/C++ビルドツールチェインが一切存在せず、権限の都合で新規インストールもできない**。したがって、公式手順通りの`python3 simulation/simulation.py`実行は現状不可能である。

Base状態・GRF・関節指令・MPC計算時間のログ、および20秒以上のGIFは、いずれも**作成できていない**。

## 14. GIF

未作成(理由:上記13節のブロッカーによりシミュレーションが1回も実行できていないため)。

## 15. 事実

- `external/Quadruped-PyMPC`はcommit `cc145a2`で固定されたgit submoduleであり、本Step作業前後で`git status`(submodule内)に差分がないことを確認した(無変更)。
- 6節の呼び出し経路・行番号は、すべて上記commitに対して`grep`/`Read`で直接確認したものである。
- README_install.mdが要求する`cmake`・`make`・Cコンパイラが本マシンには存在せず、非対話的`sudo`も使えないため、acadosのビルドが実行不可能である。
- `acados_template`のインポートは、ビルド前の状態でも`future_fstrings`エンコーディング関連の`SyntaxError`で失敗する。

## 16. 推測

- `future_fstrings`のエラーは、`future-fstrings`というPyPIパッケージ(ソースエンコーディング宣言`# -*- coding: future_fstrings -*-`を解釈するためのcodec登録を行う)が現在の`.venv`にインストールされていないために発生していると考えられる。acadosを正式にビルドする過程(`pip install -e ./../interfaces/acados_template`)でこの依存関係も解決される可能性が高いが、未検証のため断定はしない。
- ホスト環境にC/C++ツールチェインが意図的に置かれていない(コンテナ/サンドボックス環境の制約である)可能性が高いが、根拠となる情報はこのセッション内には無く確証はない。

## 17. 未解決事項

1. **acadosがビルドできない**:C/C++ツールチェイン(`cmake`・`make`・gcc/g++)が本マシンに存在せず、`sudo`も非対話的に使えない。ユーザー側での対応(ツールチェインの導入、または既にビルド済みのacados/コンテナ環境の提供)が必要。
2. 上記が解決するまで、Step 1の実行(5〜8項目)・ログ収集・GIF作成が行えない。
3. `acados_template`の`future_fstrings`エラーの根本原因は未検証。

## 18. 次のStepへ進める条件

- 上記「未解決事項1」が解消され、`python3 simulation/simulation.py`が公式手順通りに実行できること。
- 20秒以上のGIFと、Base状態・接触状態・GRF・関節指令・MPC計算時間のログが`artifacts/`配下に保存されていること。
- 完了条件(指示書「Step 1 完了条件」)をすべて満たすまで、Step 2以降には着手しない。
