# 共通ブロックカタログ（B00–B13）

## 1. 結論

両リポをリポ名ではなく **14個の役割** に再分割する。これが将来の `src/mpc_dog/` のモジュール境界である。ファイル対応は[04](04_PyMPC_to_Blocks.md)、[05](05_LeggedControl_to_Blocks.md)。配置は[08](08_Src_Layout.md)。

ID は方針である。上流 OSS のクラス名ではない。

## 2. カタログ

凡例:

- **抽出**: 自分の `src/` に置く現実度（方針）
- **混成**: 両リポの実装を差し替えられるか（推測）

| ID | ブロック | 何をするか | PyMPC | LC | 抽出 | 混成 |
|---|---|---|---|---|---|---|
| B00 | 型と契約 | 脚順、frame、配列レイアウト | `LegsAttr`、dict | OCS2 型、rbd 36 | **新規定義** | 必須 |
| B01 | ユーザー指令 | 速度 / ゴールを内部コマンドへ | `_sample_ref_vel`, キー | `/cmd_vel`, goal | 容易 | 可 |
| B02 | 参照生成 | NMPC が追う参照 | `ref_state` dict | 2点 `TargetTrajectories` | 容易 | 要変換 |
| B03 | ゲイト | 位相と接地列 \(c_{i,k}\) | `PeriodicGaitGenerator` | OCS2 Gait（外部） | PyMPC を採用 | LC は後回し |
| B04 | 足場参照 | 着地点ヒューリスティック | `FootholdReferenceGenerator` | swing z のみ近い | 中 | 部分 |
| B05 | 地形 | 地形角・高さ | `TerrainEstimator` | 標準で高さ0 | 容易 | PyMPC のみ |
| B06 | 状態推定 | センサ → 剛体状態 | なし（真値） | 線形 KF | 中（LC） | 補完 |
| B07 | 予測モデル | \(\dot x = f(x,u,p)\) | CasADi SRBD | OCS2 FullCentroidal（外部） | 中〜難 | **二者択一** |
| B08 | NMPC | OCP を解いて先頭 \(u\) | acados nominal | `SqpMpc`（外部） | 難 | **二者択一** |
| B09 | 下位制御 | GRF/目標 → \(\tau\) | `-J^T F` | `WeightedWbc` | 中 | **バックエンド2種** |
| B10 | 遊脚軌道 | 足の Cartesian 軌道と PD | `SwingTrajectoryController` | WBC 内 swing タスク / `SwingTrajectoryPlanner` | 中 | 部分 |
| B11 | 関節指令 | clip、ハイブリッド則 | `np.clip` + `action` | `setCommand(Kp=0,Kd=3)` | 容易 | 可 |
| B12 | 安全 | 非常停止条件 | なし | `SafetyChecker` | 容易 | 補完 |
| B13 | プラント | 次状態を進める | MuJoCo | Gazebo / 実機 | アダプタ | I/O のみ |

B00 は他のすべての前提である。B00 なしに混成はできない。

## 3. ブロックの3分類（方針）

### 3.1 共通化して1実装にする（採用元を決める）

| ID | 採用の方針 | 理由 |
|---|---|---|
| B00 | **自分で新規定義** | 上流の型を残すと永遠に非互換 |
| B01 | 小さい共通 dataclass | 中身は \(v_x,v_y,\dot\psi\) 程度 |
| B03 | **PyMPC の周期ゲイト** | ソースが tree 内。LC のゲイトは OCS2 |
| B04 | **PyMPC の foothold** | LC は着地点 xy をこの形では持たない |
| B05 | **PyMPC の TerrainEstimator** | LC 標準は地形高さ 0 |
| B11 | 共通の小さい関数 | clip と PD 則 |
| B12 | **LC の SafetyChecker** | PyMPC に相当物が無い |
| B13 | **MuJoCo アダプタを先に** | 本 repo の sim がここにある |

### 3.2 同じ穴に差し込む代替実装（プラグイン）

| ID | 実装 A | 実装 B | いつ選ぶか |
|---|---|---|---|
| B07+B08 | PyMPC SRBD + acados | （将来）自分で書いた SRBD + 別ソルバ | まず A。OCS2 は bind しない |
| B09 | `jacobian_transpose`（PyMPC） | `weighted_qp`（LC） | 平坦は A、拘束が要るとき B |
| B10 | 独立 Swing PD（PyMPC） | WBC の swing タスク（LC） | B09 の選択に追従 |
| B06 | `ground_truth`（sim） | `linear_kf`（LC） | sim 検証は前者、実機寄りは後者 |

B07 と B08 は対である。予測モデルと OCP を別リポから混在させない（方針）。

### 3.3 オーケストレータ（ブロックに数えない）

| 名前 | 現行 | `src/` での扱い |
|---|---|---|
| ループ | `simulation.py`, `LeggedController::update` | `loop/` に薄い合成関数を1本 |
| 配線 | `WBInterface`, `QuadrupedPyMPC_Wrapper` | 分解して消す。残さない |

## 4. データが流れる向き（方針）

依存は一方向にする。矢印の先だけを import してよい。

```text
B13 プラント
  ↑
B11 関節 ← B12 安全
  ↑
B09 下位 ← B10 遊脚
  ↑
B08 NMPC ← B07 予測モデル
  ↑
B02 参照 ← B01 指令
  ↑
B03 ゲイト ← B04 足場 ← B05 地形
  ↑
B06 推定
  ↑
B00 型
```

**実装事実との差:** 現行 PyMPC は `wb_interface` が B03–B05 と B02 と B09 を同一クラスで持つ。現行 LC は `LeggedController` が B06・B08読取・B09・B11・B12 を同一メソッドで持つ。分割後はそれを禁じる。

## 5. 各ブロックの入出力（要約）

次元の正本は[06](06_Interfaces_and_Contracts.md)。ここは役割確認である。

| ID | 主な入力 | 主な出力 |
|---|---|---|
| B01 | ジョイスティック / キー / Twist | `UserCommand`（速度、旋回） |
| B02 | command + 現在状態 + 地形 | `Reference`（NMPC 用） |
| B03 | dt, step_freq, duty, offset | `contact (4,)`, `contact_seq (4,N)` |
| B04 | 速度, hip, stance 時間, lift-off | 着地点 `(4,3)` |
| B05 | 足位置, yaw, 接地 | terrain roll/pitch/height |
| B06 | IMU, 関節, 接地 | `RbdState` または直接 `CentroidalState` |
| B07 | \(x,u,p\) | \(\dot x\) または離散 \(x_+\) |
| B08 | \(x_0\), ref, \(c_{i,k}\), 慣性 | 先頭 GRF、必要なら足・予測状態 |
| B09 | GRF または \(x^*,u^*\) + 運動学 | \(\tau (12,)\) |
| B10 | foothold, 現在足, 位相 | 遊脚 \(\tau\) または足加速度タスク |
| B11 | \(\tau\), 任意で \(q^*,\dot q^*\) | アクチュエータ指令 |
| B12 | 状態 | ok / stop |
| B13 | 指令 | 次の \(q,\dot q\), センサ |

## 6. 「ブロックに分けられない」もの（実装事実）

次はブロックにしない。理由は単一役割を持たない、または外部巨大依存である。

| 対象 | 理由 |
|---|---|
| `config.py` 全体 | 全ブロックのパラメータ袋。dataclass に分解する |
| `LeggedInterface.cpp` 全体 | OCS2 OCP 工場。仕様として読む。移植しない |
| acados サブモジュール | ソルバ実装。依存として残す |
| `ros2/`, `legged_hw`, Gazebo プラグイン | デプロイ。Phase 1 の `src/` に入れない |
| VFA / `virall.vfa` | 標準 OFF。非公開依存の記述あり |

## 7. 推測 — 最小の「歩ける」合成

最初に組み合わせて意味がある最小セットは次である。

```text
B01 + B02 + B03 + B04 + B07/B08(acados) + B09(jacobian_transpose) + B11 + B13(mujoco)
```

これは現行 PyMPC 標準経路の再配置である。新しい制御アルゴリズムではない。

その次に足すと差分が見えるもの:

1. B06 `ground_truth` → `linear_kf`（推定を入れたときの劣化）
2. B09 `jacobian_transpose` → `weighted_qp`（下位を QP にしたときの差）
3. B05 を ON（地形角。標準 PyMPC でも roll は無効）

## 8. 次

ファイルとクラスを ID に落とす。[04](04_PyMPC_to_Blocks.md)、[05](05_LeggedControl_to_Blocks.md)。
