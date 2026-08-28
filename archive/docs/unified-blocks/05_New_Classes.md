# 新規クラスの形

## 1. 結論

上流の型名を使わない。物理量と不変条件でクラスを決める。  
以下は方針である。まだ `src/` には無い。

脚順は内部で FL, FR, RL, RR に固定する（方針）。変換は端だけ。

## 2. 値オブジェクト（状態を持たない）

| クラス | 中身 | 不変条件 |
|---|---|---|
| `UserCommand` | `vx, vy, yaw_rate` | SI。frame は heading または world を明示 |
| `ContactFlags` | `(4,)` の 0/1 | それ以外の値を持たない |
| `ContactSchedule` | `(4, N)` | 各列が `ContactFlags` |
| `Grf` | `(4, 3)` | 向きを文書で1つに固定（床から足） |
| `Footholds` | `(4, 3)` | World |
| `JointTorque` | `(12,)` | 脚3×4。順は脚順定数 |
| `TerrainPlane` | roll, pitch, height | 水平ならゼロ |
| `Sensors` | IMU, q, dq, 接地, 任意真値 | プラント依存をここに閉じる |

`MpcState` と `MpcReference` は Protocol である。SRBD 用と FullCentroidal 用でフィールドが違う。

## 3. 同一層のクラス

### `PeriodicGait`

- 状態: 位相 `(4,)`
- 引数: `freq, duty, offset`
- `step(dt) -> ContactFlags`
- `horizon(N, dt) -> ContactSchedule`

不変: MPC も Resolver もこの内部位相を読まない。`Contact*` だけ読む。

### `FrictionLimits`

- 引数: `mu`
- `pyramid(normal=None) -> 不等式`

不変: MPC / QP の両方から呼べる。グローバル設定を読まない。

### `RecedingHold`

- 状態: 最後の `Grf`（と任意の `HorizonSolution`）
- `due(now) -> bool`
- `store(solution)` / `grf() -> Grf`

### `SafetyGate`

- `ok(roll, ...) -> bool`

### 純関数

```text
net_force(flags, grf, mass, g) -> (3,)
map_jt(J, F) -> tau_leg
clip_torque(tau, limit) -> tau
```

## 4. 同型層の Protocol

名前は「何を返すか」にする。ソルバ名にしない。

```text
class ReferenceBuilder:
    def build(self, cmd, body, flags, footholds, terrain) -> MpcReference

class StateSource:
    def read(self, sensors) -> MpcState

class CentroidalModel:
    dim_x: int
    dim_u: int
    def deriv(self, x, u, flags, extras) -> xdot

class GrfMpc:
    def solve(self, x0, ref, schedule, extras) -> HorizonSolution

class TorqueResolver:
    def resolve(self, grf, kin, flags, extras) -> JointTorque

class SwingPath:
    def at(self, phase_or_time) -> (p, v, a)

class Plant:
    def step(self, cmd: JointTorque) -> Sensors
```

第一実装（推測、歩くために必要な最小）:

| Protocol | 第一実装 | 由来する考え方 |
|---|---|---|
| `ReferenceBuilder` | `HoldVelocityReference` | PyMPC |
| `StateSource` | `SimTruthSource` | PyMPC |
| `CentroidalModel` | `SingleRigidBody` | PyMPC |
| `GrfMpc` | `ShortHorizonSrbd` | PyMPC（中身は自分で書いてよい） |
| `TorqueResolver` | `MapJT` | PyMPC |
| `SwingPath` | `SplineSwing` | 両方 |
| `SwingEffort` | `CartesianPdEffort` | PyMPC |
| `Plant` | `MujocoGo2` | PyMPC の Plant |

第二実装（穴だけ先に空ける）:

| Protocol | 第二実装 | 由来する考え方 |
|---|---|---|
| `ReferenceBuilder` | `TwoPointHorizonReference` | LC |
| `StateSource` | `LinearKalmanSource` | LC |
| `TorqueResolver` | `InstantQp` | LC |
| `CentroidalModel` | `FullCentroidal` | LC。書かなくてよい |
| `GrfMpc` | 長ホライズン | LC。書かなくてよい |

## 5. オーケストレータは1関数で足りる

クラスにしない。ブロックを引数で受け取る。

```text
def walking_step(blocks, sensors, cmd, clock):
    x        = blocks.state.read(sensors)
    flags    = blocks.gait.step(clock.dt)
    terrain  = blocks.terrain.or_flat()
    feet     = blocks.placement.or_none(...)
    ref      = blocks.reference.build(cmd, x, flags, feet, terrain)
    if clock.mpc_due:
        blocks.hold.store(blocks.mpc.solve(x, ref, blocks.gait.horizon(...)))
    tau      = blocks.resolver.resolve(blocks.hold.grf(), sensors.kin, flags)
    tau      = blocks.swing.maybe_override(tau, flags, ...)
    if not blocks.safety.ok(x):
        return stop
    return blocks.plant.step(clip_torque(tau))
```

これが「組み合わせて使える」の実体である。  
`WBInterface` も `LeggedController` も再現しない。

## 6. 1ファイル1役割（方針）

| ファイル案 | 中身 |
|---|---|
| `types.py` | 値オブジェクト |
| `gait.py` | PeriodicGait, Contact* |
| `friction.py` | FrictionLimits |
| `command.py` | UserCommand |
| `reference.py` | Protocol + 2ビルダ |
| `state_source.py` | Protocol + 2ソース |
| `dynamics_srbd.py` | SingleRigidBody, net_force |
| `mpc.py` | GrfMpc, HorizonSolution, RecedingHold |
| `torque.py` | map_jt, MapJT, InstantQp |
| `swing.py` | SwingPath, Effort |
| `joint.py` | clip, SafetyGate |
| `loop.py` | walking_step |

上流パスをファイル名にしない（`centroidal_nmpc_nominal.py` を置かない）。
