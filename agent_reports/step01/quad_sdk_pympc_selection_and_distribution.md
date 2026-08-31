# Quad-SDKとQuadruped-PyMPCの選定・Solver・配布判断

## 1. 背景

四脚ロボットGo2／Go2Wに対し、次の機能を持つ制御系を構築することを検討した。

- 速度指令に従って安定して歩行する
- 地形Mapから足を置ける場所を判断する
- 名目足場が危険な場合は、安全な近傍へ足場を補正する
- 安全な足場が存在しない場合は減速または停止する
- MPCで胴体運動と接地反力を計算する
- WBC／Robot Driverを通して関節指令へ変換する
- 将来、LiDARや深度カメラからオンライン地形Mapを生成する
- チーム内共有や将来の製品配布が可能なライセンス構成にする

当初は、すでに平面歩行が比較的容易に動作していた`Quadruped-PyMPC`を使用していた。一方、地形Map、足場選択、MPC、WBC、ROS 2、実機Driverまで接続された実装を探した結果、`robomechanics/quad-sdk`も候補になった。

Quad-SDKは機能範囲が広い一方、依存ライブラリ、ROS 2パッケージ、submodule、起動モード、Solver設定が複雑であり、実際に歩行するまでに複数の問題を解決する必要があった。

## 2. 目的

本調査の目的は、次の二点を明確にすることである。

1. Go2／Go2W向けの地形対応足制御の本線として、Quad-SDKとQuadruped-PyMPCのどちらを採用するか判断する。
2. チーム共有・ソース公開・製品配布を想定したとき、IPOPT、MA27、MUMPS、Coin-HSL、CasADi、acados、HPIPMなどの役割とライセンス上の問題を整理する。

## 3. 結論

### 3.1 現時点の技術方針

現在の本線は、次の構成を維持するのが合理的である。

```text
Quad-SDK
  + Local Footstep Planner
  + 独自NMPC
  + IPOPT
  + MUMPS
  + WBC / Robot Driver
  + ROS 2 / MuJoCo
```

理由は次のとおりである。

- 実際にGo2を歩行させるところまで到達している。
- MUMPSへ変更後は、連続していたNMPC失敗が解消した。
- Quad-SDKには地形Mapを利用するLocal Footstep Plannerが存在する。
- MPC、WBC、Robot Driver、ROS 2、MuJoCoがすでに接続されている。
- MUMPSは再配布可能なCeCILL-Cライセンスであり、Coin-HSLの無料ライセンスのような再配布禁止条件ではない。

したがって、**配布を理由にQuadruped-PyMPCへ移行する必要はない**。

### 3.2 Coin-HSL／MA27の扱い

無料版Coin-HSLに含まれるMA27は、個人向けかつ再配布不可という制約が強いため、チーム共有や製品配布の標準構成には含めない。

MA27を検討するのは、次の条件を満たす場合に限定する。

- MUMPSでは計算時間または収束性が要求を満たさない。
- 同一条件の計測で、MA27による明確な改善が確認できる。
- 会社が必要な商用・組み込みライセンスを取得できる。

### 3.3 Quadruped-PyMPCを選ぶ条件

Quadruped-PyMPCを選ぶ合理的な理由は、配布の可否ではなく、次の技術要件が生じた場合である。

- Python／CasADiでMPC定式化を頻繁に変更したい。
- acadosのSQP／RTIとHPIPMを使って高速化したい。
- 足場位置をMPCの決定変数として最適化したい。
- JAXによるSampling MPCを利用したい。
- 地形Plannerや大規模なROS 2統合を含まない、軽量なMPC基盤が必要である。
- Quad-SDKのNMPCが実機上で周期・収束性の要件を満たさない。

## 4. 結論に至るまでの詳細

### 4.1 Quad-SDKを候補にした理由

Quadruped-PyMPCは平面歩行のMPC実験を開始しやすく、実際に速度指令に対して良好に動作した。一方、当初の最終要件は単なる平面歩行ではなく、地形を認識して安全な足場を選ぶことであった。

Quad-SDKには次の機能が含まれている。

- 2.5D GridMap
- traversability評価
- Local Footstep Planner
- NMPC
- WBC／Robot Driver
- MuJoCo／Gazeboシミュレーション
- ROS 2による各ノードの接続

このため、地形対応足制御の統合基盤としてQuad-SDKを調査することにした。

### 4.2 Global Body Plannerを対象外にした意味

Quad-SDKには、大域的な胴体経路を生成するGlobal Body Plannerと、短期の胴体軌道・足場・接地力を計算するLocal Plannerがある。

今回の目的は大域経路計画ではなく、速度指令に対する足制御MPCである。そのため、設定を次のようにした。

```text
reference = "twist"
```

この設定では、`cmd_vel`から短期参照軌道を生成し、Global Body Plannerの経路は使用しない。

「Global Body Plannerを使用していない」という説明の意図は、今回確認できた歩行性能や問題が、Quad-SDK全機能の評価ではなく、主に次の範囲の評価であることを明確にするためである。

```text
cmd_vel
  → Local Planner
  → Local Footstep Planner
  → NMPC
  → Robot Driver / WBC
  → MuJoCoまたは実機
```

### 4.3 Quad-SDKの足場選択が必要とする情報

Local Footstep Plannerが直接利用するのは、LiDAR点群やカメラ画像そのものではなく、処理済みの2.5D GridMapとロボット状態である。

代表的なMap情報は次のとおりである。

| 情報 | 用途 |
|---|---|
| 地面高さ | 足先接地高さ、遊脚軌道 |
| 平滑化高さ | 胴体目標高さ |
| 地表面法線 | 地形姿勢の把握 |
| 平滑化法線 | 胴体roll／pitch目標 |
| traversability | 足場候補の可否判定 |

ロボット状態として、胴体位置・姿勢・速度、関節角度・速度、FKから得る足先位置などが必要になる。

ただし、`devel_ros2_review`には、実LiDAR点群からオンライン地形Mapを完成させる一連の処理が十分には含まれていない。また、次の安全機能は追加が必要である。

- Mapの更新時刻監視
- 未観測セルの排除
- Map外判定
- 有効な足場がない場合の停止

標準の足場探索は、有効な候補が見つからない場合に安全性未確認の名目足場を返すため、そのまま実機へ適用すべきではない。

### 4.4 採用したQuad-SDKの版

対象は次の版で固定した。

- Repository: `https://github.com/robomechanics/quad-sdk`
- Branch: `devel_ros2_review`
- Commit: `a3591a9f9e84aa9be3534ee0be107f0829ceb868`

リポジトリを`external/quad-sdk`へcloneし、submoduleも再帰的に取得した。ROS 2ワークスペースからは次の位置を参照する構成にした。

```text
/home/takuya/work/mpc_dog/external/quad-sdk
           ↑
/home/takuya/work/mpc_dog/ros2_ws/src/quad_sdk
```

最初のシンボリックリンクは`./external/quad-sdk`を指定したため、`ros2_ws/src`基準では存在しない場所を指していた。正しい相対リンクは`../../external/quad-sdk`である。

### 4.5 導入時に発生した主な問題

#### submoduleが空だった

`rbdl-orb`と`unitree_sdk2`のディレクトリが存在していたが、中身が完全に取得されておらず、`CMakeLists.txt`が見つからなかった。

最終的に`git submodule update --init --recursive`を正常に完了させ、指定コミットの取得を確認した。

#### rosdepが別のバックアップリポジトリまで走査した

上位ディレクトリから`rosdep`を実行した結果、Quadruped-PyMPC本体とバックアップの同名ROSパッケージを検出し、重複エラーになった。依存解決対象はQuad-SDKまたは正しいROS 2ワークスペースに限定する必要がある。

#### ROS 2がuvのPythonを参照した

CMakeが`~/.local/bin/python3.11`を選択し、そのPython環境には`catkin_pkg`が存在しなかったため、`ament_cmake`が失敗した。

ROS 2 JazzyはUbuntuのsystem Pythonとaptパッケージを前提としているため、ROS 2パッケージのビルドでは次を明示した。

```text
-DPython3_EXECUTABLE=/usr/bin/python3
```

uvはPythonアプリケーションの依存固定には有効だが、aptで導入されたROS 2 Pythonパッケージを自動的には引き継がない。したがって、ROS 2ワークスペース全体をuvへ移すことが必ずしも安定化にはならない。

#### Pinocchioの依存がexportされていなかった

`quad_utils`自身のビルド後、`quad_utils`の公開ヘッダを利用する`gazebo_scripts`などで、`pinocchio/multibody/model.hpp`が見つからなかった。

原因は、Pinocchioが`quad_utils`内部のビルドだけで利用可能になっており、下流パッケージへ正しくexportされていなかったことである。`quad_utils/CMakeLists.txt`と`package.xml`で、PinocchioをCMake／ROS依存として公開する修正を行った。

修正後、次の構成で全36パッケージのビルドに成功した。

```bash
colcon build --symlink-install \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_BUILD_TYPE=Release
```

### 4.6 ビルド成功後も歩かなかった原因

ビルド成功は、歩行制御が正しく起動することを保証しなかった。前進歩行の検証では、複数の独立した原因が見つかった。

#### 原因1：`reference="twist"`が指定されていなかった

既定値ではGlobal Body Plannerからの目標を待つため、`cmd_vel`を送ってもLocal Plannerが速度指令として使用しなかった。

対応として、`robot_configs`へ`reference="twist"`を明示した。

#### 原因2：WALKモードへ切り替えていなかった

制御モードには概ね次の区別がある。

| mode | 動作 |
|---:|---|
| 0 | SAFETY：トルクを出さない |
| 1 | STAND：PD制御で立位を保持する |
| 2 | WALK：Local Planを追従する |

STANDのままでは、Plannerが軌道を計算してもRobot Driverは立位保持を続ける。起動スクリプトへWALKモードへの切り替えを追加した。

#### 原因3：IPOPTが利用できないMA27を指定していた

Quad-SDKのNMPC実装では、IPOPTの線形Solverが次のように固定されていた。

```cpp
SetStringValue("linear_solver", "ma27");
```

MA27はCoin-HSLに含まれる。対象PCにはCoin-HSLが導入されていなかったため、NMPCが連続して失敗し、ロボットが転倒した。

実際に利用可能なSolverであるMUMPSへ変更した。

```cpp
SetStringValue("linear_solver", "mumps");
```

変更後はNMPC失敗が0件になり、ロボットが接触を確立して安定して立てるようになった。

なお、`nmpc_controller`は静的ライブラリとして`local_planner_node`へリンクされる。そのため、`nmpc_controller`だけを再ビルドしても実行ファイルが再リンクされず、古い設定が残った。`nmpc_controller`と`local_planner`の両方を再ビルドする必要があった。

#### 原因4：cmd_vel Publisherが起動直後に異常終了していた

`cmd_vel_publisher_node`へ必須のparameter fileを渡していなかったため、`topics.cmd_vel`が初期化されず、ノードが起動直後に終了していた。

つまり、それ以前の試行では速度指令が実際には配信されていなかった。公式例にある`cmd_vel_publisher_topics.yaml`を渡すよう修正した。

#### 原因5：歩行開始閾値と速度指令が同値だった

コードは次の厳密な比較で歩行開始を判定していた。

```text
cmd_vel.norm() > stand_cmd_vel_threshold
```

速度指令が`0.1 m/s`、閾値も`0.1`の場合、条件は成立しない。閾値を`0.05`へ変更し、低速指令でも歩行へ移行できるようにした。

これらを修正した結果、Quad-SDKで実際に前進歩行できる状態まで到達した。

### 4.7 CasADi、IPOPT、MA27、MUMPSの役割

Quad-SDKのSolver関係は次のように整理できる。

| 要素 | 役割 |
|---|---|
| Quad-SDK NMPC | 状態、入力、コスト、制約、予測ホライズンを定義する |
| CasADi | 数式の自動微分や評価コード生成に利用される |
| IPOPT | 非線形最適化問題全体を反復的に解く |
| MA27またはMUMPS | IPOPT内部で生じる大規模な疎線形方程式を解く |

したがって、CasADiとMA27／MUMPSは競合するSolverではない。CasADiは主に数式処理・微分・コード生成を担当し、MA27／MUMPSはIPOPT内部の数値線形代数を担当する。

Quad-SDKではCasADiが実行時にNMPC全体を直接解いている、という理解は正しくない。生成された評価関数をQuad-SDK／IPOPTが実行時に利用する構成である。

### 4.8 MA27とCoin-HSLの評価

MA27は、大規模な疎対称連立方程式を解くHSLの著名な直接法Solverであり、IPOPTで広く使われてきた。性能・収束性の面で有用な可能性はある。

ただし、無料Coin-HSLのライセンスには次の制約がある。

- 利用者個人に付与される。
- 譲渡・共有・再配布ができない。
- ソース形式、バイナリ形式ともに配布できない。
- チーム内共有や製品への同梱には適さない。

したがって、「有名で高性能な可能性がある」ことと、「今回の配布構成へ採用すべき」ことは別問題である。

### 4.9 MUMPSの位置付けとライセンス

MUMPSは、MA27と同様にIPOPT内部の疎線形方程式を解くSolverとして利用できる。

公式配布版はCeCILL-Cライセンスで提供される。これはライブラリ向けの弱いコピーレフトであり、Coin-HSLの個人利用ライセンスとは異なり、条件を守れば再配布できる。

配布時には少なくとも次を行う必要がある。

- MUMPSの著作権表示とCeCILL-Cライセンス文を同梱する。
- MUMPS本体を変更して配布する場合は、その変更部分についてCeCILL-Cの義務を確認する。
- Quad-SDK、IPOPT、その他依存ライブラリについても、それぞれのライセンス表示を保持する。

MUMPSで実際にNMPCが安定して解けている現状では、Coin-HSLを追加する必然性はない。

### 4.10 Quadruped-PyMPCでは何を使うか

Quadruped-PyMPCには勾配型とSampling型がある。

#### 勾配型

```text
CasADiでモデル・制約・コストを定義
  → acadosがSQP／RTIを実行
  → 各反復のQPをHPIPMが解く
  → BLASFEOが行列演算を高速化
```

対応関係は次のようになる。

| Quad-SDK | Quadruped-PyMPC勾配型 |
|---|---|
| IPOPT | acadosのSQP／RTI |
| MA27／MUMPS | HPIPM |
| 数値線形代数ライブラリ | BLASFEO |
| CasADiによる数式処理 | CasADiによる問題定義・コード生成 |

HPIPMはMA27の完全な置き換えではない。MA27／MUMPSは一般的な非線形最適化のKKT線形系を解くのに対し、HPIPMはMPCに現れる構造化QPを解く。

#### Sampling型

Sampling型はJAXを使い、多数の制御候補をCPU／GPUで並列評価する。この方式では通常、IPOPT、MA27、MUMPS、HPIPMを使用しない。

### 4.11 配布性の比較

| 構成 | 配布上の評価 |
|---|---|
| Quad-SDK＋IPOPT＋無料Coin-HSL | 無料Coin-HSLを再配布できないため不適切 |
| Quad-SDK＋IPOPT＋MUMPS | ライセンス表示等を守れば配布候補になる |
| Quadruped-PyMPC＋acados＋HPIPM＋BLASFEO | BSD系ライセンスが中心で配布しやすい |
| Quadruped-PyMPC Sampling型＋JAX | Apache-2.0系依存を含み、一般に配布可能 |

Quadruped-PyMPCのSolver依存は扱いやすいが、それだけでQuad-SDKから移行すべきとは結論できない。Quad-SDK＋MUMPSも配布可能だからである。

### 4.12 両者の技術的な選択基準

| 判断項目 | Quad-SDK＋MUMPS | Quadruped-PyMPC |
|---|---|---|
| 現在のGo2歩行 | 動作確認済み | 以前から動作確認済み |
| ROS 2統合 | 広範囲 | 比較的軽量 |
| 地形Map連携 | 既存機能あり | 別途統合が必要 |
| Local Footstep Planner | あり | 同等機能は標準の中心ではない |
| MPC変更の容易さ | C++／IPOPTで比較的重い | Python／CasADiで変更しやすい |
| 足場のMPC最適化 | 基本はPlannerで事前決定 | オプションあり |
| Solver配布 | MUMPSなら可能 | 比較的容易 |
| 導入・起動の容易さ | 複雑 | 比較的容易 |

今回の目的では、すでに動作しているQuad-SDK＋MUMPSを本線とし、Quadruped-PyMPCは次の用途で保持するのが妥当である。

- MPC定式化の比較対象
- acados／RTIの性能評価
- 足場最適化の参考実装
- Sampling MPCの研究
- Quad-SDK NMPCの性能が不足した場合の移行候補

## 5. 事実・判断・未確認事項

### 確認済みの事実

- Quad-SDKは指定コミットからビルドできた。
- 全36パッケージのビルドに成功した。
- MA27指定かつCoin-HSL未導入の状態ではNMPCが連続失敗した。
- MUMPSへ変更後、NMPC失敗が解消した。
- 必須parameter fileを渡すまで`cmd_vel` Publisherは起動直後に終了していた。
- WALKモード、`reference="twist"`、速度閾値修正などを反映後、前進歩行できた。
- MUMPSはCeCILL-Cで配布されている。
- 無料Coin-HSLは再配布を認めていない。
- Quadruped-PyMPC勾配型はCasADiとacadosを使用する。
- acadosはHPIPMとBLASFEOを利用する。

### 現時点の技術判断

- 配布のみを理由にQuadruped-PyMPCへ移行する必要はない。
- 現在はQuad-SDK＋MUMPSを本線とする方が、地形対応足制御の目的に近い。
- Coin-HSL／MA27は標準依存にせず、必要性を計測で確認してからライセンス取得を検討する。

### 未確認・今後の評価項目

- 実機Go2／Go2WでのMUMPSの最悪計算時間と制御周期余裕
- MUMPSと正規ライセンス版MA27の同一条件ベンチマーク
- MUMPS構成で長時間歩行した場合の収束失敗率
- 実LiDARから生成した地形Mapによる足場選択
- 有効足場なし、Map期限切れ、未観測領域での安全停止
- Quad-SDKとQuadruped-PyMPCの同一モデル・同一歩容・同一周期での性能比較
- 製品配布時の全依存ライブラリのライセンス一覧と同梱物の法務確認

## 6. 参照URL

- Quad-SDK: https://github.com/robomechanics/quad-sdk
- Quadruped-PyMPC: https://github.com/iit-DLSLab/Quadruped-PyMPC
- Quadruped-PyMPC installation: https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/README_install.md
- acados documentation: https://docs.acados.org/
- HPIPM license: https://github.com/giaf/hpipm/blob/master/LICENSE.txt
- BLASFEO: https://github.com/giaf/blasfeo
- CasADi documentation: https://web.casadi.org/docs/
- CasADi code-generation information: https://web.casadi.org/get/
- Ipopt documentation: https://coin-or.github.io/Ipopt/
- MUMPS official download and license: https://mumps-solver.org/index.php?page=dwnld
- CeCILL-C license: https://www.cecill.info/licences/Licence_CeCILL-C_V1-en.html
- HSL licensing: https://www.hsl.rl.ac.uk/licensing.html
- Coin-HSL Archive license: https://www.hsl.rl.ac.uk/download/coinhsl-archive/2021.05.05/

## 7. 最終方針

現時点では、以下を採用する。

1. Quad-SDK `devel_ros2_review`の固定コミットを基準にする。
2. Global Body Plannerは使わず、`reference="twist"`でLocal Planner系を評価する。
3. IPOPTの線形SolverにはMUMPSを使用する。
4. 無料Coin-HSL／MA27を配布構成へ含めない。
5. Local Footstep PlannerへMap鮮度、未観測領域、有効足場なし時の停止処理を追加する。
6. 実機でMUMPSの周期・収束性が不足した場合に限り、MA27の商用ライセンスまたはacadosへの移行を比較する。
7. Quadruped-PyMPCは比較・研究用の基準実装として保持する。

最終的な判断は、**Quad-SDKかQuadruped-PyMPCかという二者択一ではなく、現在必要な統合機能を持つQuad-SDKをMUMPSで運用し、PyMPCを次世代MPCの比較候補として残す**、という構成である。
