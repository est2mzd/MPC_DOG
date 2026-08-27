# 上流の計画④ helpers/velocity_modulator.py 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.wb_interface.update_state_and_reference(...)
          → self.vm.modulate_velocities(...)   ← 本ファイル(activatedがTrueのときだけ)
```

`self.vm`は`WBInterface.__init__`の中で`VelocityModulator()`として生成されるインスタンスです。`config.py`を確認すると`'velocity_modulator': True`となっており、**既定で有効**です(`visual_foothold_adaptation`や`optimize_step_freq`のような「既定OFF」の機能とは違い、この機能は標準の歩行で実際に毎周期動いています)。

## このクラスの役割(全体の中での位置づけ)

`VelocityModulator`が担当するのは、「**脚が伸びきりそうなときに、目標速度を強制的にゼロへ落とす**」という安全機構です。

- 入力：目標並進速度・目標角速度、4本の足位置、4本の股関節位置
- 出力：(必要なら書き換えられた)目標並進速度・目標角速度
- MPCの外側にある、いわば「非常ブレーキ」に相当する処理。MPCの最適化そのものには一切関与せず、MPCへ渡す**前**の目標速度を書き換えるだけ
- 歩容(いつ)・着地点(どこ)・地形推定(傾き)のいずれとも独立した、目標速度に対する安全側の補正レイヤー

対象は `external/Quadruped-PyMPC/quadruped_pympc/helpers/velocity_modulator.py`(46行)です。

---

## 1〜16行：import とコンストラクタ

```python
import numpy as np
from quadruped_pympc import config as cfg

class VelocityModulator:
    def __init__(self):
        self.activated = cfg.simulation_params['velocity_modulator']

        if cfg.robot == "aliengo":
            self.max_distance = 0.2
        elif cfg.robot == "go1" or cfg.robot == "go2":
            self.max_distance = 0.2
        else:
            self.max_distance = 0.2
```

- `self.activated`：この機能を使うかのフラグ(`bool`)。デフォルト値はなく`config.py`の値を保持する(既定`True`)
- `self.max_distance`：股関節から足先までの許容距離(m)。ロボットの種類によって値を分けようとする`if`/`elif`/`else`構造になっているが、**すべての分岐が同じ`0.2`という値になっている**
  - つまり現時点では、ロボットの種類による違いは何もない
  - 分岐の形だけは用意されているので、将来的にロボットごとに異なる値を設定する予定だった(あるいはその名残)と考えられる(**設計上の解釈**)

---

## 18〜45行：`modulate_velocities`

```python
def modulate_velocities(self, ref_base_lin_vel, ref_base_ang_vel, feet_pos, hip_pos):
    distance_FL_to_hip_xy = np.sqrt(
        np.square(feet_pos.FL[0] - hip_pos.FL[0]) + np.square(feet_pos.FL[1] - hip_pos.FL[1])
    )
    distance_FR_to_hip_xy = np.sqrt(
        np.square(feet_pos.FR[0] - hip_pos.FR[0]) + np.square(feet_pos.FR[1] - hip_pos.FR[1])
    )
    distance_RL_to_hip_xy = np.sqrt(
        np.square(feet_pos.RL[0] - hip_pos.RL[0]) + np.square(feet_pos.RL[1] - hip_pos.RL[1])
    )
    distance_RR_to_hip_xy = np.sqrt(
        np.square(feet_pos.RR[0] - hip_pos.RR[0]) + np.square(feet_pos.RR[1] - hip_pos.RR[1])
    )
```

- 引数の意味(すべてデフォルト値はなく、呼び出し元が毎回渡す):
  - `ref_base_lin_vel`：目標並進速度(m/s)
  - `ref_base_ang_vel`：目標角速度(rad/s)
  - `feet_pos`：4脚の現在位置(m、world座標系)
  - `hip_pos`：4脚の股関節位置(m、world座標系)
- `distance_FL_to_hip_xy`等(m)：各脚について、足先と股関節の水平(x,y平面)距離を計算する
- Z成分(高さ)は無視され、水平方向の広がりだけを見ている
- ここで渡される`feet_pos`は、呼び出し元(`update_state_and_reference`)を見ると**その瞬間の実際の足位置**(離地位置ではない)である点が、前章の地形推定とは違う

```python
    if(ref_base_lin_vel[0] < 0.01 and ref_base_lin_vel[1] < 0.01):
        # If the robot is not moving, we don't need to modulate the velocities
        return ref_base_lin_vel, ref_base_ang_vel
```

**実装上の問題点(強い疑い)**：

- コメントは「ロボットが動いていないなら、速度を修正する必要はない」という意図
- しかし条件式は`ref_base_lin_vel[0] < 0.01`であり、`np.abs(...)`が付いていない
- 目標速度が**後退方向(負の値)**のとき、たとえば`ref_base_lin_vel[0] = -2.0`(後ろへ2 m/sで動けという指令)だったとしても、`-2.0 < 0.01`は真になる
- つまりこの条件は「動いていない」ではなく、「前進方向にほぼ動いていない、または後退方向に動いている」を意味してしまっている
- 結果として、**後退方向の速度指令に対しては、この関数の本来の目的である脚の伸びきりチェック(下記)が一切実行されず、常にそのまま返される**ことになる
- 前進方向(正の値)のときだけ意図通りに安全チェックが働き、後退方向では素通りする、という左右非対称な挙動になっている可能性が高い(コードを読んで論理的に導いた指摘であり、実際に後退動作を試して確認したわけではない)

```python
    if (
        distance_FL_to_hip_xy > self.max_distance
        or distance_FR_to_hip_xy > self.max_distance
        or distance_RL_to_hip_xy > self.max_distance
        or distance_RR_to_hip_xy > self.max_distance
    ):
        ref_base_lin_vel = ref_base_lin_vel * 0.0
        ref_base_ang_vel = ref_base_ang_vel * 0.0

    return ref_base_lin_vel, ref_base_ang_vel
```

- 4本のうち**どれか1本でも**、股関節からの水平距離が`max_distance`(0.2m)を超えていたら、目標並進速度・目標角速度を**両方ともゼロへ**書き換える
- 中間的な減速はなく、「全開」か「完全ゼロ」かの二値的な制御になっている
- この安全機構が働くと、直後に呼ばれる歩容・着地点の計算には「目標速度ゼロ」が渡ることになる(ただし歩容自体の接地スケジュールは止まらない。read_code_02で読んだ`PeriodicGaitGenerator.update_start_and_stop`とは別の仕組みで、こちらは目標速度を書き換えるだけで、全脚接地への切り替えは行わない)

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `if(ref_base_lin_vel[0] < 0.01 and ref_base_lin_vel[1] < 0.01):`という早期リターン条件に`abs()`が付いておらず、後退方向の速度指令に対して安全チェックが素通りする可能性が高い
  2. `max_distance`のロボット別分岐が、実質すべて同じ値(0.2)になっており、分岐として機能していない
- この安全機構はMPCの外側で動き、目標速度をMPCへ渡す前に書き換えるだけである点が重要。MPCの制約や状態には一切現れない
- 次は、ここまで読んできた4つの上流コンポーネント(歩容・着地点・地形推定・速度変調)を実際にまとめ上げる`interfaces/wb_interface.py`の`update_state_and_reference`本体を読みます。
