# legged_control → ブロック対応

## 1. 結論

legged_control から `src/` に価値がある核は **B02 の2点参照、B06 線形 KF、B09 WeightedWbc、B12 Safety** である。B07/B08（NMPC 本体）と B03（ゲイト本体）は OCS2 にあり、Python へ移植しない。

理論の正本: [docs/legged_control](../legged_control/00_README.md)。

## 2. 実装事実 — ファイル → ID

| ファイル | ID | 核 / 配線 |
|---|---|---|
| `legged_controllers/.../TargetTrajectoriesPublisher.cpp` の自由関数 | B01/B02 | 核（`main` と ROS は配線） |
| `legged_estimation/.../LinearKalmanFilter.cpp` | B06 | 核 |
| `legged_estimation/.../StateEstimateBase.cpp` | B06 | 核（rbd 36 の組立） |
| `legged_estimation/.../FromTopicEstimate.cpp` | B06 cheater | sim 専用配線 |
| `legged_wbc/.../WbcBase.cpp` | B09 | 核 |
| `legged_wbc/.../WeightedWbc.cpp` | B09 | 核。**配線済み** |
| `legged_wbc/.../HierarchicalWbc.cpp`, `HoQp.cpp` | B09 代替 | **未配線** |
| `legged_wbc/.../Task.h` | B09 の部品 | 核 |
| `legged_controllers/.../SafetyChecker.h` | B12 | 核 |
| `legged_interface/.../SwingTrajectoryPlanner.cpp` | B10 | 核に近い。OCS2 mode 依存 |
| `legged_interface/.../FrictionConeConstraint.cpp` 他 | B08 の制約仕様 | 仕様として読む |
| `legged_interface/.../LeggedInterface.cpp` | B08 工場 | OCS2 配線 |
| `legged_controllers/.../LeggedController.cpp` | ループ | 配線 |
| `legged_hw/`, `legged_gazebo/`, `legged_unitree_hw/` | B13 / デプロイ | 配線 |
| `legged_common/HybridJointInterface.h` | B11 の型 | ros-control 配線 |

## 3. 実装事実 — 標準経路の核メソッド

| ID | 関数 | 計算するもの |
|---|---|---|
| B01/B02 | `cmdVelToTargetTrajectories` | Twist `(4,)` + obs → 2点 centroidal 参照 |
| B01/B02 | `goalToTargetTrajectories` | Pose 6 → 2点参照（運動量 0） |
| B06 | `KalmanFilterEstimate::update` | IMU 予測 + 足 FK 観測。並進だけ |
| B06 | `CentroidalModelRbdConversions::...` | rbd 36 → x 24。**OCS2。tree 外** |
| B08 | `mpcMrtInterface_->evaluatePolicy` | 現在時刻の 1点 \(x^*,u^*,mode\)。本体は外部 |
| B09 | `WeightedWbc::update` | QP → \(x_{wbc}(42,)\)、\(\tau=\) tail 12 |
| B12 | `SafetyChecker::check` | base roll ∈ \([-π/2, π/2]\) |
| B11 | `HybridJointHandle::setCommand` | \(q^*, \dot q^*, K_p=0, K_d=3, \tau_{ff}\) |

`HierarchicalWbc` はソースにあるが `LeggedController::init` は呼ばない。

## 4. 実装事実 — tree 内に無いブロック

| ID | 無いもの | あるもの |
|---|---|---|
| B03 | `GaitSchedule` の更新ロジック | 端末から gait 名を送る起動だけ |
| B07 | `LeggedRobotDynamicsAD` | `centroidalModelType=0` という設定値 |
| B08 ソルバ | `SqpMpc` + HPIPM | `task.info` の horizon / dt / 反復 |
| B04 xy | 着地点ヒューリスティック全体 | swing の z 軌道 |

したがって LC を「NMPC ブロックのソース」としては使えない。使えるのは **推定・WBC・参照の作り方・安全** である。

## 5. 推測 — 抽出の具体手順（LC 側）

1. **B02** 自由関数を Python 化。入力を `(4,)` と現在ポーズ/速度にする。OCS2 `TargetTrajectories` 型は使わない。
2. **B12** roll チェックを数行で移植。
3. **B11** ハイブリッド則を関数化。sim では μjoco が τ のみでも、実機寄りのテスト用に残す。
4. **B06** KF を NumPy で再実装。18 状態、28 観測。FK は Pinocchio Python（`pin` は `pyproject.toml` にある）。
5. **B09** タスク行列を NumPy で組み立て、QP は `qpsolvers` / OSQP に置換。qpOASES は必須にしない。
6. rbd 36 → 自分の状態レイアウトは **B00 の変換関数** として独立させる。OCS2 の変換を呼ばない。
7. `LeggedInterface` の制約式は B08 を自分で書くときの **仕様メモ** にする。C++ をバインドしない。

## 6. Python `src/` へ持ち込むときの言語ギャップ（実装事実 + 推測）

| 事実 | 含意（推測） |
|---|---|
| 本体が C++ | コピーではなく **再実装** が基本 |
| Pinocchio は Python バインディングがある | B06/B09 は C++ なしで再現しうる |
| OCS2 に公式の薄い Python 全面移植は無い（本 workspace 未確認） | B07/B08 は PyMPC 側を正とする |
| `task.info` に数値がある | 重み・KF 分散は移植時の照合入力になる |

## 7. LC にあってカタログに薄いもの

| 機能 | 扱い（方針） |
|---|---|
| Self-collision 制約 / 可視化 | B08 のオプション。Phase 1 対象外 |
| `FromTopicStateEstimate` | B06 の `ground_truth` バックエンドとして参考 |
| 視覚オドメトリで位置上書き | B06 拡張。対象外 |
| Gazebo 指令遅延 9 ms | B13 の realist オプション。後回し |

## 8. 次

境界の数値契約。[06](06_Interfaces_and_Contracts.md)。
