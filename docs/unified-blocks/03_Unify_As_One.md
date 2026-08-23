# 1クラスでよい層

## 1. 結論

次は数式と考え方が一致するので、**実装を1つ書けば両リポの役割を覆う**。上流の関数をコピーする必要はない。

対象: `UserCommand`, `PeriodicGait` / `ContactSchedule`, `FrictionLimits`, `net_force`, `map_jt`, `JointCommand`（飽和）, `SafetyGate`, `RecedingHold`。

## 2. UserCommand

**理論:** 指令は平面の速度である。

**実装事実:** 両リポとも前進・横・旋回を読む。

**新規（推測）:**

- フィールドは `vx, vy, yaw_rate` だけ
- frame は `heading` を既定にする（人が「前」と言う基準）
- World へ回すのは参照ビルダの仕事。指令オブジェクトは回さない

LC のゴール Pose は、同じ `UserCommand` にしない。`TwoPointHorizonReference` が Pose を直接受ければよい。

## 3. ContactSchedule と PeriodicGait

**理論:**

\[
\phi_i^{+}=\bigl(\phi_i+\Delta t\,f\bigr)\bmod 1,\qquad
c_i=[\phi_i<d]
\]

ホライズンは同じ更新を \(N\) 回した列である。

**実装事実:** この式は PyMPC にある。LC は mode 列だが、意味は「時刻 → どの脚が接地か」。

**新規（推測）:**

1. 正本は `c ∈ {0,1}^4` と `C ∈ {0,1}^{4×N}`
2. `PeriodicGait` は上記の式だけを持つ。duty, freq, offset はコンストラクタ引数
3. LC の mode は表で `c` に落とす。表はデータ、クラスを増やさない
4. MPC も TorqueResolver も `c` だけを見る。ゲイト内部を見ない

これで「LC の gait と PyMPC の gait は同じブロック」になる。ソースが OCS2 にあるかどうかは関係ない。

## 4. FrictionLimits

**理論:**

\[
F_z\ge 0,\quad |F_x|\le \mu F_z,\quad |F_y|\le \mu F_z
\]

5本の線形不等式（ピラミッド）で書くと、QP にも SQP にも渡せる。

**実装事実:** 両リポとも線形錐を使う。\(\mu\) の数値だけ違う（0.3 と 0.42）。PyMPC は Focchi の接線・法線形、LC WBC は軸揃えピラミッド。水平床ではほぼ同じ集合である。

**新規（推測）:**

```text
FrictionLimits(mu).as_pyramid() -> (D, f)   # D F <= f
```

斜面では法線を `TerrainPlane` から受けて回転する。それはこのクラスの拡張であり、別ブロックにしない。

MPC 用と WBC 用でクラスを分けない。\(\mu\) を2つ持ってよい（予測用と下位用）。それはパラメータ2つであり、式は1つ。

## 5. net_force（並進の共通核）

**理論:**

\[
F_{\mathrm{net}} = \sum_i c_i F_i + mg
\]

**実装事実:** PyMPC の \(\dot v\) がこの形。LC の \(\dot h_{com}\) も接触力の合力が駆動項（OCS2 側。詳細未照合）。

**新規（推測）:** 独立した純関数にする。`SingleRigidBody` も `FullCentroidal` もこれを呼ぶ。  
ここに関節を混ぜない。

## 6. map_jt（仮想仕事の共通核）

**理論:**

\[
\tau_i = -J_i^\top F_i
\]

**実装事実:** PyMPC の stance がこの1行。LC の EoM は \(-J^\top F_c\) を決定変数の項として含む。

**新規（推測）:**

```text
map_jt(J_leg: (3,3) or (3,nv), F: (3,)) -> tau
```

- `MapJT` リゾルバは4脚分これを足す
- `InstantQp` は行列 \(M, J\) を組み、同じ転置を制約に使う

「WBC と \(J^\top F\) は違う技術」ではなく、**後者は前者の核**である。1関数を共有する。

符号は「\(F\) は足が床から受ける力」か「足が床を押す力」かで逆になる。自分の型で \(F\) の向きを1つに決め、テストで固定する（方針）。上流2リポの符号規約を混在させない。

## 7. JointCommand と SafetyGate

**理論:** なし（工学的な箱）。

**実装事実:** 両方とも最終的に12トルクを出す。LC は加えて速度 D 項。LC だけ roll で止める。

**新規（推測）:**

- `clip_torque(tau, limit)` は常に使う
- ハイブリッド PD は `JointCommand` のオプションフィールド
- `SafetyGate` は `bool`。MPC の制約にしない

LC にあって PyMPC に無いものは、**無い側が欠けている**ので、1クラスとして足してよい。

## 8. RecedingHold

**理論:** 後退ホライズン。実行は \(u_0\)。

**実装事実:** 両方 100 Hz 求解、500 Hz 適用。

**新規（推測）:** カウンタまたは時刻で `should_solve` を返す小さなオブジェクト。GRF を保持する。  
MPC ソルバと合体させない。

## 9. 1クラスにしたあと消えるもの

上流の次の名前は、新規設計では不要である。

| 捨ててよい名前 | 吸収先 |
|---|---|
| `LegsAttr` | `(4,3)` 配列 + 脚順定数 |
| OCS2 `mode` 整数 | `ContactFlags` |
| `/cmd_vel` | `UserCommand` |
| `SafetyChecker` クラス名 | `SafetyGate` |

ファイルを対応させる必要はない。式が残ればよい。
