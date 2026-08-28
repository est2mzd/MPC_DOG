# Step 01：参照実装の固定と公式動作の再現

対象commit: `external/Quadruped-PyMPC` = `cc145a2d353db4c39df4b49e6624959acc4b87b0`(branch `main`、`origin` = `https://github.com/iit-DLSLab/Quadruped-PyMPC.git`)。
以下の行番号・引用はすべてこのcommitに対して確認したものである。

## 1. 目的

Quadruped-PyMPCを変更せずに実行し、以後のStepで比較に使う基準動作を保存する。MPC_DOG独自の制御ロジックはこのStepでは実装しない。

## 2. 今回は実施しないこと

- `external/`配下のコード変更(実施していない、後述「事実」で無変更を確認)
- MPC_DOG独自の状態推定・MPC・WBCの実装
- 前進歩行の検証(Step 5より後、本Stepの記録でも`ref_base_lin_vel`は終始ゼロで前進していないことを確認済み)
- `env.mjData.contact`等の生データそのものの保存(GRF・接触bool・関節トルクという集約済みの物理量のみをログした)

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

`external/`配下は無変更(後述「15. 事実」で、ビルド・実行の前後とも`git status`により確認)。MPC_DOG側で新規作成したのは以下の3点で、いずれも制御ロジックの実装は一切含まない。

- `docs/steps/step_01_reference_baseline.md`(本ドキュメント)
- `scripts/run_reference_baseline.sh`:環境の前提条件(ビルドツールチェイン・acadosビルド済み・`ACADOS_SOURCE_DIR`)を検査し、問題があれば理由を明示して停止する起動スクリプト
- `scripts/record_step01_baseline.py`:`simulation.py`の`run_simulation()`内部ループ(169〜327行目)を、**呼び出す関数・引数の順序を一切変えずに**そのまま呼び出しながら、(a)オフスクリーンレンダリングでGIF用フレームを取得し、(b)Base状態・接触状態・GRF・関節トルク・MPC計算時間をCSVへ記録する、記録専用のハーネス。PyMPC自体の計算式は含まれておらず、各ブロックの直前コメントに対応する`simulation.py`の行番号を明記した(ファイル冒頭のdocstring参照)。

acadosのビルドは、README_install.mdの手順通りだが1点だけ、`ACADOS_WITH_SYSTEM_BLASFEO`を`ON`(README記載値)ではなく`OFF`(acados自身の`CMakeLists.txt:96`が定義する既定値)にして実行した。理由:`ON`のまま実行すると、システムに`blasfeo`パッケージが見つからずCMake configureが失敗した(このホストはconda/pixi環境ではなく`uv`管理の`.venv`のため、Quadruped-PyMPC側のconda環境が提供する想定のシステムblasfeoが存在しない)。`OFF`はacados自身のCMakeLists.txtが警告文で「開発者が実際にテストしているのはOFFの場合のみ」と明記している値でもある。これは`external/`のコード変更ではなくビルド時のCMakeオプション選択であり、外部コードそのものには一切手を加えていない。

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

```bash
bash scripts/run_reference_baseline.sh
```

内部で行っていること(スクリプト本体参照):

1. preflightチェック(`simulation.py`の存在、`cmake`/`make`/`gcc`/`g++`、`quadruped_pympc/acados/lib`、`ACADOS_SOURCE_DIR`)
2. `ACADOS_SOURCE_DIR`・`LD_LIBRARY_PATH`をこのプロセス内だけに設定(ユーザー環境やexternal/は変更しない)
3. `scripts/record_step01_baseline.py`を実行(既定)。公式`simulation.py`を対話的にそのまま動かしたいだけの場合は`RUN_OFFICIAL_ONLY=1 bash scripts/run_reference_baseline.sh`。

acadosのビルド自体(一度だけ必要)は、README_install.mdの手順通り以下で行った(`ACADOS_WITH_SYSTEM_BLASFEO`の値のみ9節の理由によりOFFへ変更):

```bash
cd external/Quadruped-PyMPC/quadruped_pympc/acados
mkdir build && cd build
cmake -DACADOS_WITH_SYSTEM_BLASFEO:BOOL=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
make install -j4
uv pip install --python ../../../../.venv/bin/python -e ../interfaces/acados_template
uv pip install --python ../../../../.venv/bin/python -e ../../..   # Quadruped-PyMPC本体
```

(`uv pip install`を使ったのは、`.venv`が`uv`管理でpip本体を含まないため。README_install.mdの`pip install -e .`と同義)

## 12. 評価指標

- 実行の成否(バイナリ)、`external/`への差分の有無
- Base状態・接触状態・GRF・関節指令・MPC計算時間のログ有無と内容の妥当性
- GIFの実時間・解像度・フレーム数・ファイルサイズ

## 13. 結果

**成功。** `bash scripts/run_reference_baseline.sh`で、公式のPyMPCコントローラ(acados NMPC + WBC)がMuJoCo上のgo2を25秒間(12,499ステップ、`dt=0.002`秒)動かし、ログとGIFを生成した。

実行環境(このマシン、ユーザーがツールチェイン導入後):

| 項目 | 状態 |
|---|---|
| `cmake`/`make`/`gcc`/`g++` | インストール済み(Ubuntu標準パッケージ、gcc/g++ 13.3.0、cmake 3.28.3) |
| acadosビルド | 成功(`libacados.so`・`libblasfeo.so`・`libhpipm.so`を`quadruped_pympc/acados/lib/`に生成) |
| `acados_template`のインポート | 成功(ビルド後に`pip install -e`すると依存の`future-fstrings`も自動解決され、以前観測した`SyntaxError`は解消した) |
| tera_renderer | 初回実行時に公式の自動ダウンロード機能(README_install.md記載の仕様通り)で取得 |

実行結果の統計(`artifacts/logs/step_01/state_log.csv`、12,499行を集計):

| 指標 | 値 |
|---|---|
| 実時間25秒のシミュレーションにかかった壁時計時間 | 46.5秒(269 steps/s) |
| `compute_actions`(MPC+WBC)1呼び出しあたりの時間 | 平均2.17ms、中央値2.07ms、p99 4.67ms、最大30.2ms |
| Base高さ`z` | 0.290〜0.308m(平均0.306m)、転倒なし |
| Base roll / pitch | roll: -0.0135〜0.0061 rad、pitch: -0.0000〜0.0475 rad(いずれも小さく、安定) |
| 接地率(4脚) | FL 0.637 / FR 0.649 / RL 0.614 / RR 0.620(既定トロットの`duty_factor=0.65`とほぼ整合) |
| MPCが計算した接地力の合計`Fz`(4脚) | 平均139.9N、範囲116.3〜190.7N(go2質量`12.019kg`×`g=9.81`≈117.9Nに近い) |
| 目標並進速度`ref_base_lin_vel` | 全ステップで`(0.0, 0.0)`固定(後述「事実」参照) |
| Base水平方向の総移動量 | 約0.84m(前進歩行ではなく、その場でのわずかなドリフトの範囲) |
| 関節トルクの絶対値最大 | 15.0 N·m(全12関節・全ステップ中) |

## 14. GIF

- パス:`artifacts/gifs/step_01_reference_baseline.gif`
- 実時間:25.0秒(250フレーム ÷ 10fps)
- 解像度:640×360
- フレーム数:250
- ファイルサイズ:約7.6MB
- ループ:無限ループ(`imageio.mimsave(..., loop=0)`)
- 内容:ロボット全体と床面が入るfree camera(baseのx-yを追従)。画面左上に経過時間・目標速度・実速度をオーバーレイ表示。前進歩行は発生していないため10m移動の収録要件(指示書5.2節)は本Stepでは非該当(理由は「9. 事実」参照)。

## 15. 事実

- `external/Quadruped-PyMPC`および入れ子の`quadruped_pympc/acados`submoduleは、acadosのビルド・OCPソルバーのコード生成・25秒間のシミュレーション実行の前後を通じて`git status`(それぞれのsubmodule内)に一切差分が無いことを確認した。ビルド成果物(`build/`・`lib/`・`bin/`)とOCPコード生成物(`quadruped_pympc/controllers/gradient/nominal/c_generated_code/`)は、いずれも各リポジトリ自身の`.gitignore`で無視される場所に生成されている。
- 6節の呼び出し経路・行番号は、すべて対象commitに対して`grep`/`Read`で直接確認したものである。
- `run_simulation()`は既定で`base_vel_command_type="human"`(キーボード入力)を使うが、本実行は非対話的なターミナルから行ったためキー入力は一切発生せず、`ref_base_lin_vel`/`ref_base_ang_vel`は初期値のゼロのまま25秒間変化しなかった(`state_log.csv`の`ref_lin_vel_x_mps`等の列がすべて`0.0`であることで確認)。結果としてロボットは前進せず、トロット歩容でその場に留まる動きになった。
- `quadrupedpympc_wrapper.get_obs()`が返す`ctrl_state["nmpc_GRFs"]`は`LegsAttr`型(脚名でアクセスするオブジェクト)であり、フラットな配列ではない(`quadruped_pympc_wrapper.py:39`)。
- MuJoCoのオフスクリーンレンダラ(`mujoco.Renderer`)は、モデルの既定オフスクリーンフレームバッファ幅(640px)を超える解像度を指定するとエラーになる(go2のMJCFにフレームバッファサイズの明示指定が無いための既定値)。

## 16. 推測

- `compute_actions_time`の最大値(30.2ms)は、MPCが実際にacadosソルバーを呼ぶステップ(既定`mpc_frequency=100`Hzに従い間引かれる)に対応すると考えられるが、`compute_actions`内部でどのステップが実際にソルバーを呼んだかは今回のログには含めていないため、厳密な対応関係は未確認。
- Base水平方向に約0.84m動いたのは、目標速度がゼロであっても、トロットの各接地衝撃や初期姿勢からの整定過程で生じる小さなドリフトによるものと考えられる(意図的な移動指令ではない)。

## 17. 未解決事項

1. `compute_actions_time`とMPC実解回数(間引き)の対応関係の詳細な検証(未実施、Step 5「MPC計算時間が制御周期内に収まるか」でより厳密に扱う想定)。
2. GIFのファイルサイズ(約7.6MB)は要件の上限は明記されていないが、やや大きい。必要であれば減色・フレーム間引きで縮小可能。

## 18. 次のStepへ進める条件

- 完了条件をすべて満たしたことを確認済み:`external/`に差分なし、commit SHAと実行コマンドを記録、`bash scripts/run_reference_baseline.sh`一発で再現可能、主要処理経路を実ファイル名・行番号で説明、ログ(`artifacts/logs/step_01/state_log.csv`・`gif_meta.json`)とGIF(`artifacts/gifs/step_01_reference_baseline.gif`)を作成済み、GIFは25秒(20秒以上)。前進歩行は発生していないため10m要件は非該当。
- ユーザーの承認を得てからStep 2(四脚接地での静止)へ進む。
