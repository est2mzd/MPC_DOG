# `src/` 配置案

## 1. 結論

パッケージ名は `mpc_dog`（`pyproject.toml` の `mpc-dog` に対応）。リポ名のサブフォルダ（`pympc/`, `legged_control/`）は作らない。**役割ディレクトリ = ブロック ID** にする。

本章は方針である。ディスク上の `src/` は現在空である（実装事実）。

## 2. 目標ツリー

```text
src/mpc_dog/
├── __init__.py
├── types/                 # B00
│   ├── frames.py
│   ├── layouts.py         # 脚順、状態インデックス
│   └── signals.py         # dataclass
├── command/               # B01
│   └── user_command.py
├── reference/             # B02
│   └── body_reference.py
├── gait/                  # B03
│   └── periodic.py
├── foothold/              # B04
│   └── heuristic.py
├── terrain/               # B05
│   └── plane_estimator.py
├── estimation/            # B06
│   ├── ground_truth.py
│   └── linear_kf.py
├── dynamics/              # B07
│   └── srbd.py
├── mpc/                   # B08
│   ├── protocol.py        # 共通 Protocol
│   └── acados_srbd.py     # 上流を包む。最初は薄い
├── wbc/                   # B09
│   ├── protocol.py
│   ├── jacobian_transpose.py
│   └── weighted_qp.py
├── swing/                 # B10
│   ├── trajectory.py
│   └── cartesian_pd.py
├── joint/                 # B11
│   ├── clip.py
│   └── hybrid.py
├── safety/                # B12
│   └── roll_limit.py
├── plant/                 # B13
│   ├── protocol.py
│   └── mujoco_go2.py
├── loop/                  # オーケストレータ
│   └── walking_loop.py
└── adapters/              # 上流との境界。本番ループからは隠す
    ├── pympc_config.py
    ├── pympc_dicts.py
    └── lc_index.py
```

テストは `tests/` に同じ相対パスで置く（既存の `tests/` を使う）。

```text
tests/
├── test_apply_pympc_preset.py   # 既存。触らない
├── types/
├── gait/
├── foothold/
└── ...
```

## 3. 置かないもの（方針）

| 置かない | 理由 |
|---|---|
| `src/mpc_dog/external_copy/` | 上流の丸コピーは管理対象を増やすだけ |
| ROS ノード、launch、Gazebo プラグイン | デプロイ。後の別パッケージ |
| acados 生成 C コード | ビルド成果物。gitignore |
| JAX sampling / Lyapunov / kinodynamic | 標準ブロックを汚さない |
| `HierarchicalWbc` | 上流でも未配線 |
| Notebook | `docs/` に残す |

## 4. 1ブロックの中身の型（方針）

各核モジュールは次の形に揃える。

```text
# 例: gait/periodic.py

@dataclass
class PeriodicGaitParams:
    step_freq: float
    duty_factor: float
    phase_offset: tuple[float, float, float, float]

class PeriodicGaitGenerator:
    def __init__(self, params: PeriodicGaitParams): ...
    def reset(self, phase: np.ndarray | None = None) -> None: ...
    def step(self, dt: float) -> ContactFlags: ...
    def contact_sequence(self, horizon: int, dt: float) -> ContactSchedule: ...
```

禁止（現行 PyMPC の真似をしない）:

- `from mpc_dog import config` のようなグローバル
- `dict` を公開戻り値の主型にする
- コンストラクタで MuJoCo 環境や ROS ノードを作る

## 5. オーケストレータ（方針）

`loop/walking_loop.py` だけがブロックを組み立てる。擬似コード:

```text
cmd      = command.read()
state    = estimation.update(sensors)          # or plant.observe()
terrain  = terrain.estimate(feet, yaw)
cmd      = command.modulate(cmd, feet, hips)   # optional
contact  = gait.step(dt)
footholds = foothold.compute(cmd, hips, contact)
ref      = reference.build(cmd, state, terrain, footholds)
if due_mpc:
    grf, footholds_mpc = mpc.solve(state, ref, gait.sequence())
tau = wbc.stance(grf, J, contact)
tau = swing.override(tau, contact, footholds_mpc, feet)
if not safety.ok(state):
    stop()
plant.step(joint.clip(tau))
```

これは現行 PyMPC 標準経路の再配置である（推測ではなく、[qpympc-study/02](../qpympc-study/02_System_Architecture_and_Dataflow.md) の番号付けに対応）。新しいアルゴリズムをここで発明しない。

## 6. 上流コードの扱い（方針）

| やり方 | いつ使う |
|---|---|
| **再実装**（短い、式が閉じている） | B03, B05, B11, B12, B02 の核、B09 の \(J^T F\) |
| **再実装 + 数値照合** | B04, B10, B06, B07（NumPy） |
| **薄いラップ** | B08 acados、B13 `QuadrupedEnv` |
| **仕様だけ読んで自分で書く** | B09 QP（ソルバは OSQP） |
| **移植しない** | OCS2 全体、`LeggedController`, `WBInterface` |

ラップする場合、`adapters/` が上流の dict / `LegsAttr` を変換する。核モジュールは上流を import しない。

## 7. `pyproject.toml` との関係（方針）

パッケージ発見を後で足す。今は書かないが、想定は次である。

```text
[tool.setuptools.packages.find]
where = ["src"]
```

依存は既存のまま使えるものが多い（numpy, scipy, casadi, mujoco, gym-quadruped, pin）。qpOASES は足さない。QP が要るときだけ OSQP または `qpsolvers` を extra にする。

## 8. 推測 — 最初の PR に入れる最小セット

1コミットで全部作らない。最初にディレクトリと B00 + B03 + テストだけ、が扱いやすい。

| 優先 | パス | 照合相手 |
|---|---|---|
| 1 | `types/`, `gait/periodic.py` | `PeriodicGaitGenerator.run` の固定 dt |
| 2 | `terrain/plane_estimator.py` | 固定4足位置 |
| 3 | `foothold/heuristic.py` | 直進トロットの1ケース |
| 4 | `wbc/jacobian_transpose.py` | `-J.T @ F` の数値 |
| 5 | `joint/clip.py` | 既知の limit |
| 6 | `loop/` + `plant/` + `mpc/` | 既存 headless と同じ seed で転倒しない |

6 が「ブロックを組み合わせて歩ける」最初のマイルストーンである。

## 9. 次

抽出のルールと段階。[09](09_Extraction_Policy.md)。
