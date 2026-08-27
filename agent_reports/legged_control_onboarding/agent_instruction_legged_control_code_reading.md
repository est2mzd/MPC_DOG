# legged_control 逐次コード読解の運用ルール

`external/Quadruped-PyMPC`向けに確立した
`agent_reports/quadruped_pympc_onboarding/agent_instruction_quadruped_pympc_code_reading.md`
の方法論を、`external/legged_control`(C++、ROS1/catkin、OCS2ベース)向けに
そのまま適用する。以下はpympc側で試行錯誤を経て確定した最終ルールを、
このリポジトリの言語・ビルド系(C++/ROS1)に合わせて読み替えたものである。
pympc側の instruction ファイルにある「なぜこのルールに至ったか」という
経緯の記述は繰り返さない(結論だけをここに書く)。

## 1. このリポジトリの前提(pympcとの違い)

- **言語・実行系がPythonではなくC++/ROS1(catkin)である**。`simulation.py`の
  ような「これ1つを読めば実行の全体が追える単一スクリプト」は存在しない。
  実行の起点は`legged_examples/legged_unitree/legged_unitree_hw/src/legged_unitree_hw.cpp`
  の`main()`(実機・Gazebo共通で使う`legged_hw/LeggedHWLoop`を起動する)、
  または`legged_gazebo`パッケージのGazeboプラグインロードである
- **MPCソルバー本体(OCS2)がこのリポジトリに含まれない外部ライブラリである**。
  `external/legged_control`が持っているのは「OCS2に渡す問題定義
  (動力学・コスト・制約、`legged_interface`パッケージ)」と「OCS2の解を
  受け取ってWBC・ロボットへ渡す部分(`legged_wbc`、`legged_controllers`)」
  であり、OCS2内部のSQP/QP求解ロジック自体は対象外(pympcにおける
  acadosの内部実装を深追いしなかったのと同じ扱い)。OCS2側の関数呼び出しは
  シグネチャ・型・返り値までは確認するが、内部実装は「このリポジトリの
  対象外、未確認」と明記する
- **`config.py`に相当する単一ファイルが無い**。数値パラメータは以下に
  分散している。数値を書く前に必ず該当ファイルを直接読んで確認すること
  (pympcでの`step_freq=1.35`誤り事件と同じ轍を踏まない):
  - `legged_controllers/config/<robot>/task.info`(OCS2形式、コスト重み・
    制約パラメータ・MPC設定)
  - `legged_controllers/config/<robot>/reference.info`(目標速度・姿勢生成)
  - `legged_controllers/config/<robot>/gait.info`(歩容スケジュール定義)
  - `legged_controllers/config/controllers.yaml`(ros_controlのコントローラ
    登録)
  - `legged_examples/legged_unitree/legged_unitree_hw/config/<robot>.yaml`
    (実機HWのループ周波数・安全閾値等)
  - `legged_gazebo/config/default.yaml`(Gazebo側センサ・接触設定)
  - ロボットタイプは環境変数`ROBOT_TYPE`(`a1`/`aliengo`/`go1`/`laikago`)で
    選択され、pympcの`cfg.robot='go2'`のような単一のPythonデフォルトは
    存在しない。具体的な数値例を示すときは、断りなく`a1`の設定値を代表例
    として使う(使うときは「a1の設定では」と明記する)

## 2. ファイル名と番号

- `read_code_連番2桁_ファイル内容.md`、格納先は
  `agent_reports/legged_control_onboarding/`
- 連番は呼び出しの上流→下流(実行ループの仕組み→ハードウェア抽象化→
  具体的なハードウェア実装(実機/Gazebo)→コントローラ本体→状態推定→
  OCS2向け問題定義→WBC→補助/テレオペ)の意味的な順序で振る
- 保留ファイルは`NN_ファイル内容.md`とし、後で正式な連番に差し替える
- リネーム時は他のread_codeファイルからの相互参照を全て更新する

## 3. 各ファイル冒頭に必須の2セクション

1. **実行への結びつき(呼び出し連鎖)**
   - `simulation.py`に相当する単一の起点が無いため、
     「`legged_unitree_hw.cpp`の`main()`」または「Gazeboプラグインの
     ロード(`legged_hw_sim_plugins.xml`経由)」を起点とした呼び出し連鎖を
     `→`矢印で書く
   - 呼び出し頻度(毎制御周期/起動時1回/条件付き)を明記する
   - 実行から一切呼ばれない経路(テストコード、未使用ロボットタイプ専用等)
     はこの節を省略してよい
2. **このファイル/クラスの役割(全体の中での位置づけ)**
   - 何を担当し、何を担当しないかを、他ファイルを参照せず単体で完結させる

## 4. 情報は直接書く

- 過去の分析結果や他の`.md`(read_code以外)を参照するだけで済ませない。
  事実は直接書き下す。同シリーズ内の`read_code_NN`への相互参照のみ許可
- OCS2側の型・関数(`ocs2::...`名前空間)については、シグネチャ・意味は
  書けるが、内部実装は「対象リポジトリの外、未確認」と明記する

## 5. フォーマット

- 長い地の文で複数の事実を並べない。箇条書き・表に分解する
- コード数行の引用→直後に短い説明、を繰り返す
- 具体的な数値例(実際の設定ファイルの値を代入した計算)を積極的に入れる
- 事実と解釈を区別する(「〜と考えられる」「〜の可能性が高い」等の言葉で
  推測であることを示す)

## 6. 変数・メンバ変数の記載ルール

すべての変数(関数引数、メンバ変数、戻り値)について、次を省略しない。

1. **型**:C++は型が言語の一部なので必ず書く(`double`、`Eigen::Vector3d`、
   `scalar_t`(OCS2の型エイリアス、実体は`double`)、`vector_t`
   (`Eigen::VectorXd`)等)
2. **単位**:m、m/s、rad、rad/s、秒、Hz、N、N·m、無次元等
3. **値**:
   - デフォルト引数があればその値
   - 無い場合、実際にこのワークスペースの設定ファイル(1節参照)で使われて
     いる具体的な値を、該当ファイルを実際に`grep`・`Read`して確認してから
     書く。値が不明なまま書かない

変数の意味は「変数名:意味(型、単位)。」という短い1文で止める。
用途・更新タイミング等は別の箇条書きに分ける(pympc側ルール18.1と同じ)。

## 7. 関数・メソッドごとの役割

新しい関数・メソッドの解説を始めるときは、コードブロックを貼る前に、
その関数が何をする関数なのかを1行で先に書く(pympc側ルール19と同じ)。

## 8. 進め方

- 1ファイルずつ、丁寧に進める
- 1ファイル書き終えたら、次に読むべきファイル(呼び出し連鎖上、自然に
  つながる先)を末尾で示し、ユーザーの指示を待ってから次へ進む
- ユーザーが「最後まで一気に」等、明示的に連続実行を指示した場合のみ、
  複数ファイルを連続して作成してよい

## 9. 新しいルール・訂正が判明した場合

pympc側ルール18.3と同じく、今書いている1ファイルだけでなく、既存の
`read_code_*.md`シリーズ全体を対象に`grep`等で横断的に確認し、見つかった
箇所をすべて直す。

## 10. バグ・危険箇所の集約

pympcシリーズの`09_bugs_and_risks_summary.md`と同様、legged_control側でも
一定量のread_codeファイルが揃った段階で、横断的なバグ・危険箇所の集約
ファイルを作成する(ユーザーからの指示があった時点、またはシリーズが
一区切りついた時点で提案する)。
