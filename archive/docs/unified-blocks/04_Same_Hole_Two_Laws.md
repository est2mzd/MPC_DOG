# 同型層 — 穴は同じ、法則は2つ

## 1. 結論

次の層は **入出力の意味が同じ** なのでブロック化できる。中の式は1つにできない。  
「概ね同じ」は Protocol の話であり、中身を平均する話ではない。

## 2. 図

```text
         同じ穴（型）
ReferenceBuilder ──┬── HoldVelocityReference      ← PyMPC の考え方
                   └── TwoPointHorizonReference   ← LC の考え方

CentroidalModel  ──┬── SingleRigidBody            ← PyMPC
                   └── FullCentroidal             ← LC（後回し可）

GrfMpc           ──┬── ShortHorizonSrbd           ← PyMPC
                   └── LongHorizonCentroidal      ← LC（後回し可）

StateSource      ──┬── SimTruthSource             ← PyMPC
                   └── LinearKalmanSource         ← LC

TorqueResolver   ──┬── MapJT                      ← PyMPC
                   └── InstantQp                  ← LC

SwingEffort      ──┬── CartesianPdEffort          ← PyMPC
                   └── SwingAsQpTask              ← LC（InstantQp と組）
```

縦棒が「ブロック化できそう」の根拠である。左右はリポ由来の **考え方** であり、ファイルではない。

## 3. ReferenceBuilder

**同じ穴:**

```text
in:  UserCommand, 今の胴体, ContactFlags, 任意で FootPlacement
out: そのモデル用の MpcReference
```

**2法則:**

| | HoldVelocity（PyMPC） | TwoPoint（LC） |
|---|---|---|
| 何を追わせるか | 速度・高さ・姿勢。xy 位置は追わない | 1 s 先の点（速度指令ならそこへ進んだポーズ） |
| ホライズン上の参照 | ほぼ一定の速度参照 | 2点を補間 |
| 合うモデル | 位置 xy コストが 0 の SRBD | 位置も状態に入る centroidal |

**推測:** 参照レシピと予測モデルは対である。HoldVelocity を FullCentroidal に、TwoPoint を「xy コスト0の SRBD」に無理に繋ぐと、式は動いても意図が壊れる。組み合わせの禁止は[06](06_Compose.md)。

## 4. CentroidalModel

**同じ穴:**

```text
in:  x, u, contact, extras(inertia, mass, wrench)
out: xdot  または  x_{k+1}
```

**2法則:**

| | SingleRigidBody | FullCentroidal |
|---|---|---|
| \(x\) に足 | 足位置12 | 関節12 |
| 関節の慣性 | 無視 | 運動量に残す |
| 並進 | `net_force` と同じ | 合力は同じ族 |
| 回転 | \(I\dot\omega=\) 接触モーメント \(-\omega\times I\omega\) | centroidal \(L\) の式（OCS2） |

共有してよいのは `net_force` と「接触は \(c_i\) でゲート」だけである。状態ベクトルを共通配列にしない。

**推測:** 自分の第一モデルは SingleRigidBody で足りる。FullCentroidal は「穴を空けておく」。OCS2 を再実装しない。

## 5. GrfMpc

**同じ穴:**

```text
in:  x0, MpcReference, ContactSchedule, extras
out: HorizonSolution
        .grf0: (4,3)
        .optional: footholds, x_pred
```

**2法則:** ホライズン長、\(f\)、制約の書き方（ゲートをダイナミクスで消すか ZeroForce 制約か）。

公開してよい出力は **今出す力** である。内部の `u∈R^{24}` を TorqueResolver に見せない。  
Resolver が関節速度まで欲しい（LC WBC のベース加速度タスク）なら、`HorizonSolution` に optional 欄を足す。必須にしない。

**推測:** ソルバ名（acados / HPIPM）をクラス名にしない。中で呼べばよい。

## 6. StateSource

**同じ穴:**

```text
in:  Sensors（IMU, q, dq, 接地, 任意で真値）
out: 今のモデルが食える x0
```

**2法則:** 真値の並べ替え vs 線形 KF。

KF の式は LC の考え方を新規に書いてよい。出力を LC の36次元に合わせる必要はない。  
**推測:** KF は「World の \(p,v\)」を出し、モデルが自分の \(x\) に組み立てる方がきれい。36と24を KF の公開型にしない。

## 7. TorqueResolver

**同じ穴:**

```text
in:  Grf, 運動学(J, 任意で M, nle, 足位置), ContactFlags
out: tau (12,)
```

**2法則:**

```text
MapJT:      tau = 脚ごとの map_jt
InstantQp:  min  タスク
            s.t. EoM, 摩擦, 立脚 J qdd = -Jdot v, 遊脚 F=0
```

InstantQp の接触力追従タスクは

\[
F_c \approx F^{MPC}
\]

である。つまり **穴の入力は同じ GRF** である。QP はそれを硬等号にしない。

**推測:** 遊脚の扱いを Resolver に寄せると、PyMPC 形は「遊脚 τ を SwingEffort が上書き」、LC 形は「遊脚は QP タスク」。  
ループは次のどちらか一方にする。

- MapJT + CartesianPdEffort
- InstantQp（swing をタスクとして内包）

3つ同時（MapJT + PD + QP）は穴が二重になる。

## 8. SwingMotion

**同じ穴の前半（軌道）:** 位相または残り時間 → \(p^*,v^*,a^*\)。

これは1法則でよい（スプライン）。両リポの数値（step height, kp, kd）はパラメータ。

**同じ穴の後半（実現）:** 上の2行（MapJT 組 or QP 組）。

## 9. 「概ね同じ」の判定基準（方針）

次をすべて満たせば同型ブロックとする。

1. 入力の物理量の意味が同じ（力、接地、運動学）
2. 出力の物理量の意味が同じ（参照、\(\dot x\)、\(\tau\)）
3. 両リポの標準経路で、その層を抜くと歩行が壊れる
4. 中の式を1つにすると、どちらかの論文的意図が消える → 2法則にする

4が無いものは[03](03_Unify_As_One.md)へ移す。
