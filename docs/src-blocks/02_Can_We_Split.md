# ブロックに分けられるのか

## 1. 結論

**3段階で答えが違う。**

| 段階 | 答え | 区分 |
|---|---|---|
| A. 概念・呼出順として層があるか | **はい** | 実装事実 |
| B. 現行ファイルをそのまま再利用ライブラリにできるか | **一部だけ** | 実装事実（依存）+ 推測（工数） |
| C. 両リポの部品をレゴのように差し替えられるか | **今は不可。境界を自分で置けば一部可** | 方針 / 推測 |

分けられない、ではない。**今の結合の仕方のままでは、自分の `src/` に「ブロック」として置けない**。

## 2. 実装事実 — すでに層は分かれている

### 2.1 PyMPC の1ループ（標準）

`run_simulation` → `QuadrupedPyMPC_Wrapper.compute_actions` の順（[qpympc-study/02](../qpympc-study/02_System_Architecture_and_Dataflow.md)）:

1. ユーザー速度（Heading）
2. World 変換
3. Plant getters
4. `TerrainEstimator`
5. `VelocityModulator`
6. `PeriodicGaitGenerator`（位相、接地列）
7. `FootholdReferenceGenerator`
8. `ref_state` 組立
9. 5ステップに1回 `Acados_NMPC_Nominal.compute_control`
10. GRF mask
11. Stance `-J^T F` / Swing PD
12. clip → `env.step`

関数・クラスは層ごとに存在する。

### 2.2 legged_control の1ループ（標準）

`LeggedController::update`（[legged_control/01](../legged_control/01_Packages_and_Control_Loop.md)）:

1. `updateStateEstimation` → rbd `(36,)` → centroidal `(24,)`
2. `setCurrentObservation`
3. `updatePolicy` / `evaluatePolicy` → `x* (24,)`, `u* (24,)`, mode
4. `WeightedWbc::update` → `(42,)`、使うのは `τ (12,)`
5. `SafetyChecker`
6. `setCommand(q*, dq*, Kp=0, Kd=3, τ)` ×12

NMPC 本体は別スレッド 100 Hz の `advanceMpc()`。

### 2.3 共通する層（実装事実）

両方とも次をこの順で持つ。

```text
指令 → 参照 → （接地スケジュール） → NMPC → 下位トルク → 関節 → プラント
```

違いは中身である。推定の有無、NMPC の状態定義、下位が Jacobian 転置か QP か。

## 3. 実装事実 — そのまま切れない理由

### 3.1 PyMPC

| 障害 | 根拠 |
|---|---|
| グローバル `config.py` | コントローラ・ヘルパ・モデルがモジュール import で読む。コンストラクタ注入ではない |
| dict 契約 | `state_current` / `ref_state` のキーが `wb_interface` と NMPC の暗黙 API |
| `LegsAttr` | gym-quadruped の型。NumPy 配列だけの公開 API ではない |
| acados codegen | `c_generated_code/` とビルド済みソルバ。純粋関数ではない |
| IK が Plant を所有 | `InverseKinematicsNumeric` が内部で `QuadrupedEnv` を作る |
| 代替経路が同居 | sampling / lyapunov / kinodynamic が同一インタフェースにぶら下がる |

### 3.2 legged_control

| 障害 | 根拠 |
|---|---|
| OCS2 型 | `vector_t`, `SystemObservation`, `TargetTrajectories` が全層に浸透 |
| NMPC 本体が tree 外 | ダイナミクス AD、SQP、ゲイト本体が無い |
| ros-control | `LeggedController` はプラグイン。アルゴリズムと I/O が同一クラス |
| Pinocchio + 配置順 | rbd `(36,)` と pinocchio 配置の軸順が違う（コードが入れ替え） |
| qpOASES | WBC だけだが C++ 依存 |

### 3.3 両リポ間

| 項目 | PyMPC | legged_control | 互換? |
|---|---|---|---|
| NMPC 状態 | CoM 位置速度 + rpy + ω + 足12（+積分6） | 正規化 centroidal 運動量6 + ベース6 + 関節12 | **非互換** |
| NMPC 入力 | 足速度12 + GRF12 | GRF12 + 関節速度12 | 後半の意味は近いが全体は違う |
| 参照 | 毎周期の dict。位置参照の xy は 0 | 2点軌道（現在と最大 1 s 先） | **非互換** |
| 接地 | `(4,)` と `(4,12)` の 0/1 | OCS2 mode 番号 + `ModeSchedule` | 変換が要る |
| 下位入力 | mask 済み GRF と foothold | `x*(24)`, `u*(24)`, rbd `(36,)`, mode | **非互換** |
| 下位出力 | `τ (12,)` のみ（標準） | `τ` + `q*` + `dq*` + Kd | 出力は近い |
| ロボット | Go2、脚順 FL/FR/RL/RR | A1 系、コントローラ脚順 LF/LH/RF/RH | 脚順が違う |

脚順の差は実装事実である（PyMPC は gym-quadruped の `LegsAttr`、legged_control README / `contactNames`）。変換を書かないと混成できない。

## 4. 推測 — 切りやすさ

工数は未実測。依存の向きからの見積である。

### 4.1 切りやすい（純計算、入出力が狭い）

| 候補 | 理由 |
|---|---|
| `PeriodicGaitGenerator` | NumPy。位相と 0/1。設定を引数にすれば独立 |
| `TerrainEstimator` | 幾何。`roll_activated=False` 等のフラグを引数化 |
| `FootholdReferenceGenerator` | ヒューリスティック。`LegsAttr` と重力定数だけ |
| Swing 軌道生成（scipy / explicit） | 多項式。制御ループから分離可能 |
| `cmdVelToTargetTrajectories` 一式 | ROS を外すと約 100 行の幾何 |
| `SafetyChecker` | roll 判定のみ |
| 関節則（clip / `Kp=0, Kd=3, ff=τ`） | 数行 |

### 4.2 中くらい（運動学または QP、ソルバは置換可）

| 候補 | 理由 |
|---|---|
| `WeightedWbc` + `WbcBase` | タスク定義は明確。Pinocchio と型の橋が要る |
| 線形 KF | 予測・更新は標準。観測の作り方が Pinocchio FK 依存 |
| Stance `-J^T F` | 式は1行。`J` の供給元（MuJoCo or Pinocchio）を境界にする |

### 4.3 切りにくい（方針: 包むか、後回し）

| 候補 | 理由 |
|---|---|
| acados NMPC 一式 | codegen、warm-start、パラメータ `p` の並び |
| OCS2 NMPC 全体 | この workspace にソースが無い。Python `src/` へ移植は非現実 |
| `WBInterface` / `LeggedController` | ブロックではなく配線 |
| `simulation.py` / `LeggedHWLoop` | プラントと I/O |

## 5. 混成の可否（推測 + 方針）

レゴとして意味がある組み合わせと、意味が薄い組み合わせがある。

| 組み合わせ | 判定 | 理由 |
|---|---|---|
| PyMPC ゲイト + PyMPC SRBD-MPC + PyMPC Stance/Swing | **現行そのもの** | すでに動く |
| PyMPC ゲイト + PyMPC MPC + LC の WeightedWbc | **狙う価値あり** | 下位だけ QP に上げる。要: GRF/状態 → WBC 入力の変換 |
| LC の KF + PyMPC の残りのスタック | **狙う価値あり** | sim 真値を実機相当の推定に置換。要: rbd → PyMPC `state_current` |
| LC の NMPC + PyMPC の Swing | **変換コスト大** | 状態定義が違う。先に片方の NMPC を選ぶべき |
| 両方の NMPC を同時に同じロボットへ | **非推奨** | 同じ層の二重実装。比較実験以外は無駄 |
| OCS2 を Python で再実装 | **非推奨** | 方針は[09](09_Extraction_Policy.md)。SRBD 側を自分で持つ |

## 6. 「ブロック」の定義（方針）

このノートでは、次を満たすものをブロックと呼ぶ。

1. **役割が1つ**（ゲイトだけ、KF だけ、など）
2. **入出力が配列と小さな dataclass** で書ける（ROS メッセージやグローバル config に依存しない）
3. **内部状態があってもよい**が、所有者が明示されている（位相、ソルバ warm-start）
4. **他ブロックを import しない**か、依存は下流方向だけ
5. **単体テスト**で、固定入力に対する出力を元コードと照合できる

現行の `WBInterface` と `LeggedController` はブロックではない。**オーケストレータ**である。`src/` では薄く残す。

## 7. 最終判定

- **分けられる**: 概念層は両リポがすでに分けている。
- **今のままでは使いまわせない**: 設定・型・ソルバ・ROS が接着剤になっている。
- **自分の `src/` で分けられる**: 境界契約（[06](06_Interfaces_and_Contracts.md)）を先に置き、純計算から再実装または薄く包む。

次はブロックの名前と範囲である。[03](03_Unified_Block_Catalog.md)。
