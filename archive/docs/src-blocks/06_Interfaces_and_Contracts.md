# 境界契約 — 型・次元・frame

## 1. 結論

ブロックを組み合わせる条件は、関数名の一致ではなく **配列の意味が一致すること** である。上流2リポのレイアウトは互いに非互換なので、`src/` では **自分の正本レイアウト** を1つ決める。上流との差分は変換関数に閉じる。

本章の「上流の数」は実装事実。「自分の正本」は方針である。

## 2. 実装事実 — 上流の次元

### 2.1 PyMPC（標準 nominal）

正本: [qpympc-study Appendix A](../qpympc-study/appendices/A_Variable_Dictionary.md)、[06](../qpympc-study/06_Centroidal_SRBD_Model.md)。

| 信号 | Shape | 内容 | Frame |
|---|---|---|---|
| MPC 状態 \(x\)（基本） | `(24,)` | CoM \(p,v\), rpy, \(\omega\), 足4×3 | 主に W。\(\omega\) は Base |
| acados 状態 | `(30,)` | + 積分6。既定 `use_integrators=False` | — |
| MPC 入力 \(u\) | `(24,)` | 足速度12 + GRF12 | 足速度・GRF は W |
| 接地瞬間 | `(4,)` | 0/1 | なし |
| 接地列 | `(4, 12)` | horizon 12 | なし |
| GRF ベクトル | `(12,)` | FL,FR,RL,RR × xyz | W |
| 関節トルク | `(12,)` | 脚×3 | 関節 |
| `qpos` / `qvel` | `(19,)`, `(18,)` | Go2 MuJoCo | MuJoCo |
| 脚順 | FL, FR, RL, RR | `LegsAttr` | — |

### 2.2 legged_control（a1 既定）

正本: [legged_control/02](../legged_control/02_System_Architecture_and_Dataflow.md)、[04](../legged_control/04_State_Estimation.md)、[06](../legged_control/06_WBC.md)。

| 信号 | Shape | 内容 | Frame |
|---|---|---|---|
| NMPC \(x\) | `(24,)` | \(h_{com}/m\) (6), \(p_b\) (3), ZYX (3), \(q_j\) (12) | README / task.info |
| NMPC \(u\) | `(24,)` | GRF 12 + 関節速度 12 | GRF は W |
| rbd | `(36,)` | ZYX, \(p\), \(q_j\), \(\omega\), \(v\), \(\dot q_j\) | 推定出力 |
| KF \(\hat x\) | `(18,)` | \(p_b, v_b\), 足位置12 | — |
| KF 観測 | `(28,)` | 足 pos/vel 12+12 + 高さ4 | — |
| WBC \(x\) | `(42,)` | \(\ddot q(18), F_c(12), \tau(12)\) | — |
| 参照 | t:2, x:24×2, u:24×2 | 2点だけ | — |
| コントローラ脚順 | LF, LH, RF, RH | 各 HAA, HFE, KFE | — |
| 接触名コメント（OCS2） | LF, RF, LH, RH | `modelSettings` | コントローラと異なる可能性 |

接触名の2通りは [legged_control/00](../legged_control/00_README.md) §6 が注意している。混成時は **脚インデックス表を1枚持つ**（方針）。

## 3. 方針 — 自分の正本（第一版）

第一版は **PyMPC 標準に寄せる**。理由: 本 repo の sim が動く経路であり、B07/B08 を自分で持てる。LC 由来の B06/B09 は変換して接続する。

### 3.1 Frame

| 記号 | 意味 |
|---|---|
| W | world / odom |
| B | base |
| H | heading（yaw のみ） |

ZYX は yaw-pitch-roll。PyMPC の SciPy `xyz` と LC の ZYX は **同じ3角でもライブラリ規約が違う**。変換は B00 にテストを置く（方針）。未確認の一致は推測であり、数値テストで決める。

### 3.2 脚順（方針）

内部は **FL, FR, RL, RR**（PyMPC / gym-quadruped）。LC の LF/LH/RF/RH はアダプタで並べ替える。

```text
FL = LF
FR = RF
RL = LH
RR = RH
```

この対応は命名からの推測である。URDF とコントローラ配列を1回照合してから固定する。

### 3.3 推奨 dataclass（方針）

名前は仮。実装時に `src/mpc_dog/types/` へ置く。

| 型 | 中身（最小） |
|---|---|
| `UserCommand` | `v_lin_H: (3,)`, `yaw_rate: float` |
| `ContactFlags` | `c: (4,)` 0/1 |
| `ContactSchedule` | `seq: (4, N)` |
| `Footholds` | `p: (4, 3)` W |
| `Grf` | `F: (4, 3)` W |
| `JointTorque` | `tau: (12,)` |
| `CentroidalState` | PyMPC 24 と同じ並び |
| `RbdState` | LC 36 と同じ並び（KF 用）。NMPC には直接渡さない |
| `Reference` | `CentroidalState` + `Footholds` + 速度。2点版は `times: (2,)` を追加 |
| `PlantState` | `qpos, qvel` または抽象センサ |

dict キー文字列をブロック間 API にしない（方針）。現行 PyMPC の dict はアダプタの内側に閉じる。

## 4. 実装事実 — 周期

| 処理 | PyMPC 標準 | LC 標準 |
|---|---|---|
| Plant / HW | 500 Hz（dt=0.002） | 500 Hz |
| 推定 | なし（毎周期真値） | 500 Hz |
| ゲイト位相 | 500 Hz | gait ノード（イベント） |
| NMPC | 100 Hz（5 sim step に1回） | 100 Hz スレッド |
| 下位トルク | 500 Hz（MPC 出力は保持） | 500 Hz（policy を現在時刻で評価） |
| 参照 | 500 Hz で組立 | 指令イベント。観測待ち |

自分のループも **下位 500 Hz、NMPC 間引き 100 Hz** を第一の契約にする（方針）。LC のように policy 補間するか、PyMPC のようにゼロ次ホールドするかはループの実装選択である。第一版は **ゼロ次ホールド**（動かしている経路に合わせる）。

## 5. 接続表（方針）

| 上流ブロック | 下流 | 渡す型 |
|---|---|---|
| B01 | B02, B04 | `UserCommand` |
| B05 | B02 | terrain roll/pitch/height |
| B03 | B08, B09 | `ContactFlags`, `ContactSchedule` |
| B04 | B02, B10 | `Footholds` |
| B06 または Plant | B02, B08, B09 | `CentroidalState` または `RbdState` |
| B02 | B08 | `Reference` |
| B08 | B09, B10 | `Grf`, `Footholds`（任意で予測状態） |
| B09+B10 | B11 | `JointTorque` |
| B11 | B13 | アクチュエータ配列 |
| B12 | ループ | bool |

LC の `WeightedWbc` を B09 に挿すときは、アダプタが `CentroidalState` + `Grf` + `RbdState` → WBC 入力 `(24,)+(24,)+(36,)+mode` を作る。このアダプタは推測が多く、**数値照合が必須**である。

## 6. 実装事実 — 単位

| 量 | 単位 |
|---|---|
| 位置 | m |
| 速度 | m/s |
| 角度 | rad |
| 角速度 | rad/s |
| 力（GRF） | N |
| トルク | N·m |
| 質量 | kg |

両リポとも SI。ここは互換である。

## 7. 未確認（推測に落とすな）

次は [qpympc-study F](../qpympc-study/appendices/F_Open_Questions.md) および LC ノートの「OCS2 未照合」と重なる。契約を固定する前にテストで決める。

- PyMPC の `env.com` が物理 CoM と一致するか
- SciPy `xyz` Euler と OCS2 ZYX の数値一致
- LC 接触名順とコントローラ関節順の完全対応
- WBC に渡す `mode` と `(4,)` 接地の写像

## 8. 次

式とブロックの対応。[07](07_Equation_to_Block.md)。
