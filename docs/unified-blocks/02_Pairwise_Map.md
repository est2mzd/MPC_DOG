# 対応表 — LC ここ × PyMPC ここ → 新規クラス

## 1. 読み方

各節は同じ型である。

- **考え方**: 両者が共有する問
- **数式**: 一致 / 同型 / 不一致
- **LC のここ** / **PyMPC のここ**: 実装事実。クラスを移植する指定ではない
- **こう設計する**: 新規クラス。import しない
- **判定**: 同一 / 同型 / 片方 / 合わせない

「こう設計する」は推測〜方針である。

## 2. 一覧

| # | 考え方 | 判定 | 新規クラス（仮） |
|---|---|---|---|
| P1 | 胴体速度を指令する | **同一** | `UserCommand` |
| P2 | 指令を MPC 参照にする | **同型** | `ReferenceBuilder` |
| P3 | 接地は MPC の外で決める | **同一**（符号化が違う） | `ContactSchedule` + `PeriodicGait` |
| P4 | 次の足はどこへ置くか | **片方に厚い** | `FootPlacement` |
| P5 | 床の傾き | **片方** | `TerrainPlane` |
| P6 | 今の状態 \(x_0\) | **同型** | `StateSource` |
| P7 | 重心まわりの予測 \(\dot x=f\) | **同型** | `CentroidalModel` |
| P8 | ホライズン上で GRF を最適化 | **同型** | `GrfMpc` |
| P9 | 摩擦を守る | **同一**（線形化の差は小さい） | `FrictionLimits` |
| P10 | GRF を関節トルクへ | **同型** | `TorqueResolver` |
| P11 | 遊脚を着地点へ運ぶ | **同型** | `SwingMotion` |
| P12 | モータへ出す直前 | **同一** | `JointCommand` + `SafetyGate` |
| P13 | 遅い MPC と速い下位 | **同一** | `RecedingHold` |
| P14 | プラントを1歩進める | **同型**（I/O） | `Plant` |

## 3. 各ペア

### P1 指令 — 同一

**考え方:** 人は「前へ・横へ・回れ」を出す。関節角は出さない。

**数式:** 一致。内部表現は

\[
u_{\mathrm{cmd}} = (v_x,\; v_y,\; \dot\psi)
\quad [\mathrm{m/s},\;\mathrm{m/s},\;\mathrm{rad/s}]
\]

LC は \(v_z\) も読めるが、ゲームパッド既定では出ない（実装事実）。

| | 場所（実装事実） |
|---|---|
| LC | `cmdVelCallback` が読む Twist。中身 `(vx,vy,vz,ψ̇)` |
| PyMPC | `_ref_base_lin_vel_H` と `_ref_base_ang_yaw_dot` |

**こう設計する:**

```text
UserCommand(vx, vy, yaw_rate, frame="heading")
```

I/O 装置（キー、ROS、ゲームパッド）は `CommandSource` の外側。核は3スカラー。

---

### P2 参照 — 同型

**考え方:** 生のジョイスティックを OCP に直接載せない。胴体が今からどう動いてほしいかを、MPC の状態空間へ書く。

**数式:** 役割は同じ。レシピが違う。

| | レシピ（実装事実） |
|---|---|
| LC | 現在と最大 1 s 先の **2点**。速度指令なら両端の運動量先頭を \(v_W\) にする |
| PyMPC | 毎周期1つの `ref_state`。位置 xy の参照は 0。速度と高さ・姿勢・足を書く |

**こう設計する:**

```text
ReferenceBuilder.build(command, state_now, contact, footholds) -> MpcReference
```

実装を2つ置く。

- `HoldVelocityReference` — PyMPC 形。位置 xy を追わせない
- `TwoPointHorizonReference` — LC 形。\(t\in\{t_0, t_0+T\}\) の2点

どちらも戻りは `MpcReference`（そのモデルの \(x^{ref}, u^{ref}\)）。  
**推測:** 1つの関数に if で両レシピを混ぜない。差し替えにする。

---

### P3 接地スケジュール — 同一

**考え方:** 「今どの足が床にいるべきか」はゲイトが決める。MPC はそれを破らない。

**数式:** 周期ゲイトとしては PyMPC が明示している。

\[
\phi_i \leftarrow (\phi_i + \Delta t\, f)\bmod 1,\quad
c_i = [\phi_i < d]
\]

LC は mode 番号（立脚脚の組合せ）とイベント時刻である。中身は同じ4ビットの時系列である。

| | 場所（実装事実） |
|---|---|
| LC | OCS2 `ModeSchedule` / `GaitReceiver`（tree 外）。コントローラは mode を読む |
| PyMPC | `PeriodicGaitGenerator.run` と `compute_contact_sequence` → `(4,)` と `(4,N)` |

**こう設計する:**

```text
ContactFlags      # c: (4,) の 0/1。脚順は自分で固定
ContactSchedule   # seq: (4, N)
PeriodicGait.step(dt) -> ContactFlags
PeriodicGait.horizon(N, dt_mpc) -> ContactSchedule
```

LC の mode 整数は `flags_from_mode(mode) -> ContactFlags` で吸収する。  
ゲイト生成器は Periodic 以外（イベント表）も `ContactSchedule` さえ出せばよい。

**判定: 同一。** 符号化の差だけなので、配列 `(4,)` を正本にする。

---

### P4 足の置き場 — 片方に厚い

**考え方:** 遊脚は「次にどこへ着くか」を持つ。ゲイトは時刻だけ決め、場所は別である。

**数式:** PyMPC は Raibert 系ヒューリスティック（速度 × stance 時間 + 誤差補償）。LC 標準は xy 足場器を持たず、swing の **z** 軌道と、NMPC 関節が足を動かす。

| | 場所（実装事実） |
|---|---|
| LC | `SwingTrajectoryPlanner`（主に z）。xy は NMPC の関節最適化側 |
| PyMPC | `FootholdReferenceGenerator.compute_footholds_reference` |

**こう設計する:**

```text
FootPlacement.suggest(command, hips, lift_off, stance_time) -> points (4,3)
```

第一実装は PyMPC の考え方を自分の式で書く。LC 経路では `None`（MPC に足位置を任せる）でもブロックとしては成立する。

**判定: 片方。** 無理に LC と同一視しない。役割だけ空ける。

---

### P5 地形平面 — 片方

**考え方:** 床が傾いていれば、指令速度と高さ参照を床に合わせたい。

| | 場所（実装事実） |
|---|---|
| LC | 標準で地形高さ 0 |
| PyMPC | `TerrainEstimator`。標準は roll 無効、pitch/高さはフィルタ |

**こう設計する:** `TerrainPlane(roll, pitch, height)` を optional 入力にする。無ければ水平。LC 相当はゼロ平面。

---

### P6 現在状態 — 同型

**考え方:** MPC は \(x(t_0)=x_{\mathrm{meas}}\) が要る。センサ生値ではない。

| | 場所（実装事実） |
|---|---|
| LC | IMU + 関節 + 接地 → 線形 KF（並進）→ rbd 36 → centroidal 24 |
| PyMPC | MuJoCo の CoM / 足 / 姿勢をそのまま `state_current` にする。KF 無し |

**こう設計する:**

```text
StateSource.read(sensors) -> model が要求する MpcState
```

- `SimTruthSource` — プラントの真値を並べる（PyMPC 形）
- `LinearKalmanSource` — LC 形。並進だけ濾波

どちらも **そのループが選んだモデルの状態** を返す。36と24を世界標準にしない。

---

### P7 予測モデル — 同型

**考え方:** 床反力の合力・合モーメントが、重心の並進と角運動量を動かす。関節トルクはここには出さない。

**数式:** 並進は同じ族。

\[
m\dot v = \sum_i c_i F_i + mg
\]

回転と「何を状態に残すか」が違う。

| | 状態に残すもの（実装事実） |
|---|---|
| LC | \(h_{com}\)、ベース、**関節角**。Full centroidal（既定 type 0） |
| PyMPC | CoM、姿勢、ω、**足位置**。関節なし。SRBD |

**こう設計する:**

```text
CentroidalModel
    dim_x, dim_u
    deriv(x, u, contact, extras) -> xdot
```

- `SingleRigidBody` — 足を点、胴を1剛体
- `FullCentroidal` — 関節を状態に残す（書くなら後。OCS2 を呼ばない）

**判定: 同型。1式に潰さない。** 並進式だけ共通関数 `net_force(contact, grf, mass)` に抜く。

---

### P8 GRF の NMPC — 同型

**考え方:** 既知の \(x_0\)、参照、接地列のもとで、未来の胴体と GRF を二次コストで解く。トルクは変数にしない。先頭 \(u\) だけ使う。

**数式:** 骨格は同じ。

\[
\min \sum_k \|x_k-x_k^{ref}\|_Q^2 + \|u_k-u_k^{ref}\|_R^2
\quad\text{s.t.}\quad
x_{k+1}=f(x_k,u_k,c_k),\;
\text{摩擦},\;
x_0=x_{\mathrm{meas}}
\]

中身の差（実装事実）:

| | PyMPC | LC |
|---|---|---|
| \(f\) | SRBD、ERK、N=12、0.02 s（0.24 s） | Full centroidal、1.0 s、0.015 s |
| 参照 | 毎周期 dict | 2点をホライズンへ伸ばす |
| 遊脚力 | ダイナミクスは \(c_i\) で消す。摩擦箱は残る | ZeroForce 制約 |
| ソルバ | acados SQP | OCS2 SQP + HPIPM |

**こう設計する:**

```text
GrfMpc.solve(x0, reference, schedule, inertia) -> HorizonSolution
HorizonSolution.first_grf() -> Grf   # (4,3)
```

ソルバ（acados 等）は `GrfMpc` の内部である。公開戻りは **力と、必要なら足・予測状態**。  
`u` 全体の24並べを公開契約にしない。

**推測:** 第一実装は SRBD + 短いホライズン。LC の 1 s / 関節入りは第2法則として空けるだけでよい。

---

### P9 摩擦 — 同一

**考え方:** 足は床を引き抜けない。接線は \(\mu F_z\) 以内。

**数式:** 本質は同じ。線形化が少し違う。

\[
F_z\ge 0,\quad |F_x|\le\mu F_z,\quad |F_y|\le\mu F_z
\]

| | 場所（実装事実） |
|---|---|
| LC WBC | ピラミッド5式。\(\mu=0.3\) |
| LC NMPC | 軟制約の錐（詳細は OCS2 側） |
| PyMPC NMPC | Focchi 線形錐。\(\mu=0.42\)。脚あたり5式 |

**こう設計する:**

```text
FrictionLimits(mu).pyramid(F) -> inequalities
```

MPC も WBC もこれを呼ぶ。\(\mu\) はブロックの外から渡す。  
**判定: 同一。** 係数の差はパラメータであり、クラスを分けない。

---

### P10 トルクへ落とす — 同型

**考え方:** MPC は力を出した。ロボットは関節トルクしか出せない。仮想仕事で脚へ写す。必要なら今この瞬間の拘束（EoM、摩擦、遊脚加速度）をもう一度解く。

**数式:**

共通核:

\[
\tau = -J^\top F \quad \text{（符号は「床を押す」の定義に合わせる）}
\]

LC はこれを単独では終わらせず、

\[
M\ddot q - J^\top F_c - S^\top \tau + nle = 0
\]

の下で \(\ddot q, F_c, \tau\) を QP する。\(F_c\) は MPC の力に **近づける**（重みが小さいとずらしてよい）。

| | 場所（実装事実） |
|---|---|
| LC | `WeightedWbc::update`。42変数。使うのは \(\tau\) 12 |
| PyMPC | `compute_stance_and_swing_torque` の \(-J^\top F\)。QP なし |

**こう設計する:**

```text
TorqueResolver.resolve(grf, kinematics, contact, extras) -> JointTorque
```

- `MapJT` — 上の1行。PyMPC 形
- `InstantQp` — LC 形。EoM + 摩擦 + タスク。ソルバは何でもよい

共通部品として `MapJT` を関数で持ち、`InstantQp` の EoM 項がそれを含む、と見る。

**判定: 同型。** 「WBC」という1クラスに両方を入れ、中で if しない。

---

### P11 遊脚 — 同型

**考え方:** 遊脚は力を出さない。足先を次の着地点（または NMPC が望む位置）へ、軌道を描いて運ぶ。

**数式:** 軌道（スプライン）+ PD。LC の WBC では加速度タスク

\[
J\ddot q = k_p(p^*-p)+k_d(v^*-v)-\dot J v
\]

PyMPC では同等の足加速度を脚トルクへ直接書く。

| | 場所（実装事実） |
|---|---|
| LC | `SwingTrajectoryPlanner`（z）+ WBC `formulateSwingLegTask` |
| PyMPC | `SwingTrajectoryController` + scipy/explicit 生成器 |

**こう設計する:** 2段に分ける。

```text
SwingPath.at(phase) -> p*, v*, a*     # 軌道だけ。トルクを知らない
SwingEffort.apply(...)                # 実現方法
```

- `CartesianPdEffort` — 脚 PD（PyMPC）
- `SwingAsQpTask` — `InstantQp` に渡すタスク（LC）

経路の式は1つ。力への落とし方だけ差し替える。

---

### P12 関節直前 — 同一

**考え方:** 計算トルクをアクチュエータが飲める形にする。危険なら止める。

| | 場所（実装事実） |
|---|---|
| LC | `setCommand(q*, dq*, Kp=0, Kd=3, τ)`。`SafetyChecker` は roll |
| PyMPC | `clip(τ, 0.9 τ_lim)`。安全クラスなし |

**こう設計する:**

```text
JointCommand.from_torque(tau, limits)           # 必須
JointCommand.from_hybrid(q, dq, kp, kd, tau)    # 実機寄り optional
SafetyGate.ok(state)                            # roll など
```

飽和と安全は MPC に入れない。

---

### P13 後退ホライズンの使い方 — 同一

**考え方:** 予測は長く、実行は今の一瞬。MPC は下位より遅い。

**こう設計する:**

```text
RecedingHold
    maybe_solve(...)   # 100 Hz のときだけ GrfMpc
    current_grf()      # 500 Hz は前回を返す
```

LC の「軌道を現在時刻で評価」は `RecedingHold` の別実装 `PolicyInterpolate` でよい。第一はゼロ次ホールド。

---

### P14 プラント — 同型（I/O）

**考え方:** トルクを受けて、次のセンサを返す。制御ブロックではない。

```text
Plant.step(joint_cmd) -> Sensors
```

MuJoCo / Gazebo / 実機はここだけ差し替える。中の物理を制御クラスに漏らさない。

## 4. 一文で

> LC の「ゲイトは別・MPC は GRF・WBC が τ」と、PyMPC の「ゲイトは別・MPC は GRF・\(J^\top F\) が τ」は、**同じ文章の下位だけが違う**。  
> 新規設計ではその文章をクラス境界にし、下位と予測モデルだけを2法則にする。

詳細な式は[03](03_Unify_As_One.md)、[04](04_Same_Hole_Two_Laws.md)。
