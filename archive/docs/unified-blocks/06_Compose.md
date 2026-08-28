# 組み合わせ例

## 1. 結論

ブロック化できる、の意味は「LC 風と PyMPC 風を、同じ `walking_step` の引数差し替えで作れる」ことである。  
全部を同時に足す必要はない。

以下は推測（設計）。動いた事実ではない。

## 2. 組 A — PyMPC と同じ考え方

```text
UserCommand
PeriodicGait
HoldVelocityReference
SimTruthSource
SingleRigidBody + ShortHorizonSrbd
MapJT + SplineSwing + CartesianPdEffort
clip_torque
MujocoGo2
```

意味: ゲイトが接地を決め、短い SRBD-MPC が GRF を出し、\(J^\top F\) と遊脚 PD で τ にする。推定しない。

LC の部品は使っていないが、P1–P3, P8–P13 の **穴** は LC と同じである。

## 3. 組 B — 下位だけ LC の考え方

```text
（組 A の上側はそのまま）
TorqueResolver = InstantQp     # MapJT を置換
SwingEffort    = （QP 内タスク。CartesianPd は外す）
```

意味: 「MPC は力、今この瞬間の拘束は QP」という LC の文章。予測モデルは SRBD のまま。

**推測:** これが一番旨みのある混成である。状態定義を変えずに、下位だけ厚くできる。  
組 A と組 B で変えるクラスは Resolver（と遊脚の所属）だけ、が成功条件。

## 4. 組 C — 推定だけ LC の考え方

```text
（組 A）
StateSource = LinearKalmanSource
```

意味: \(x_0\) が真値ではなく濾波される。MPC / 下位は知らない。

**推測:** 2番目に旨みがある。実機に近づける実験が、MPC を触らずにできる。

## 5. 組 D — 参照だけ LC の考え方

```text
ReferenceBuilder = TwoPointHorizonReference
```

**推測:** SRBD で位置 xy コストが 0 だと、2点軌道の「1 s 先の位置」は効きにくい。  
組 D は `SingleRigidBody` のコストを「位置も追う」に変えるか、モデルを変えるときだけ意味がある。  
第一版では組に入れない。

## 6. 組 E — やってはいけない混成

| 混成 | 理由 |
|---|---|
| SRBD の \(x\) を LC の24次元配列に入れて MPC | 同じ長さでも物理が違う。[07](07_Do_Not_Force.md) |
| MapJT と InstantQp を同じ周期に直列 | トルクが二重定義 |
| CartesianPd と QP swing タスクを同時 | 遊脚が二重 |
| FullCentroidal の \(x\) に PyMPC の足位置を連結 | 状態の意味が壊れる |
| 両方の NMPC を1ループで解く | 同じ穴に2つの力 |

## 7. 「概ね同じでブロック化できそう」の最短文

- **ゲイト:** LC の mode と PyMPC の \(c_i\) は、`ContactFlags` にすれば同じ
- **指令:** Twist とキー入力は `UserCommand` にすれば同じ
- **摩擦:** 両錐は `FrictionLimits` にすれば同じ
- **MPC:** 両方とも `GrfMpc.solve → Grf`。中の \(f\) だけ違う
- **下位:** 両方とも `TorqueResolver.resolve(Grf) → τ`。1行か QP かだけ違う
- **推定:** 両方とも `StateSource → x0`。真値か KF かだけ違う
- **遊脚:** 軌道は同じ。τ にするか QP タスクにするかだけ違う

## 8. 性能比較（同じ穴に実装を並べる）

組 A と組 B は設計の話である。測るときは [block-curriculum/12](../block-curriculum/12_Block_Performance_Comparison.md) に従い、**1穴だけ** 挿し替える。

例: 組 A のまま `ShortHorizonSrbd` を `EqualShare` に戻すと、MPC の有無の比較になる。下位の `MapJT` は動かさない。
 InstantQp に替える比較は、MPC を固定した別表である。
