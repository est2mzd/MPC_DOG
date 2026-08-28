# MuJoCo Go2 Plant Model

本章はGo2 Plant（自由度、アクチュエータ、接触、センサ、質量）の正本である。閉ループでの`action`組立と`mj_step`は[11](11_Joint_Torque_and_MuJoCo_Closed_Loop.md)。型付きデータ契約は[02](02_System_Architecture_and_Dataflow.md)。

## 1. 結論

標準シミュレーションがロードするGo2は、gym-quadruped 1.1.5同梱の`gym_quadruped/robot_model/go2/go2.xml`である。MuJoCo MenagerieのURLは上流参照として[00](00_README.md)に残すが、このワークスペースではMenagerieをcheckoutしておらず、実行時XMLとの同一性は未確認である。

このXMLは浮遊胴体、12関節、リンク質量・慣性、トルク`<motor>`、Visual/Collision分離を持つ全身剛体Plantである。モータ電気系、通信遅延、実機PD、バックラッシュ、実測足裏ゴム、センサノイズは含まれない。

XMLにはIMUと12`jointpos`の計16センサがある。標準`run_simulation()`は`sensors=`を渡さず、`mjData.sensordata`も読まない。状態は`qpos`/`qvel`と`mj_jac`等から直接取る。

対応コード: `gym_quadruped/robot_cfgs.py` の `get_robot_config('go2')`、`gym_quadruped/quadruped_env.py` の `QuadrupedEnv.__init__()` / `step()` / `reset()`、`simulation/simulation.py` の `run_simulation()`。

## 2. 自由度と状態ベクトル

`get_robot_config('go2')`の`hip_height`は0.28 m。`MjModel`実測は`nq=19`、`nv=18`、`nu=12`、`nbody=14`（world + 13リンク）、`nsensor=16`。

| 項目 | 次元 |
|---|---:|
| Base位置 | 3 |
| Base姿勢Quaternion (`wxyz`) | 4 |
| 関節角 | 12 |
| `nq` | 19 |
| Base並進・角速度 | 6 |
| 関節速度 | 12 |
| `nv` | 18 |
| Torque actuator | 12 |

各脚はHip abduction、Thigh pitch、Calf/Knee pitchの3関節である。前脚thighと後脚thighの可動域は別classである。

### 2.1 `qpos`（`(19,)`）

| index | 内容 | 単位 | frame |
|---|---|---|---|
| 0:3 | Base位置 | m | W |
| 3:7 | Base Quaternion `wxyz` | 無次元 | W |
| 7:10 | FL hip, thigh, calf | rad | 関節 |
| 10:13 | FR hip, thigh, calf | rad | 関節 |
| 13:16 | RL hip, thigh, calf | rad | 関節 |
| 16:19 | RR hip, thigh, calf | rad | 関節 |

`freejoint`は無名。対応コード: `go2.xml` の `<freejoint/>` と各`<joint name="*_joint">`。読取は`mjData.qpos`。

### 2.2 `qvel`（`(18,)`）

| index | 内容 | 単位 | Frame |
|---|---|---|---|
| 0:3 | Base並進速度 | m/s | W。`base_lin_vel(frame='world')` |
| 3:6 | Base角速度 | rad/s | B（コード上の解釈）。`base_ang_vel(frame='base')`は`qvel[3:6]`をそのまま返す。MuJoCo freejoint公式との一致は[F](appendices/F_Open_Questions.md) |
| 6:9 | FL関節速度 | rad/s | 関節 |
| 9:12 | FR | rad/s | 関節 |
| 12:15 | RL | rad/s | 関節 |
| 15:18 | RR | rad/s | 関節 |

対応コード: `QuadrupedEnv.base_lin_vel()`、`QuadrupedEnv.base_ang_vel()`。

### 2.3 関節可動域

XML default class（`autolimits="true"`）。

| class | 適用 | range [rad] |
|---|---|---|
| `abduction` | 全hip | −1.0472 … 1.0472 |
| `front_hip` | FL/FR thigh | −1.5708 … 3.4907 |
| `back_hip` | RL/RR thigh | −0.5236 … 4.5379 |
| `knee` | 全calf | −2.7227 … −0.83776 |

対応コード: `go2.xml` の `<default class="abduction|front_hip|back_hip|knee">`。

## 3. 全身運動方程式

理論として、MuJoCoは概念的に次を解く。

\[
M(q)\ddot q+h(q,\dot q)=S^\mathsf T\tau+J_c(q)^\mathsf T\lambda
\]

- \(\tau\)：12関節トルク。`mjData.ctrl`へ入る指令
- \(\lambda\)：MuJoCo接触ソルバーが求める接触力
- \(h\)：重力、コリオリ、遠心、受動損失など

MPCのGRFをXMLへ直接入力するのではない。MPCのGRFを関節トルクへ変換し、そのトルクと接触からMuJoCoが\(\lambda\)を求める。

対応コード: MuJoCoエンジン。制御側の代入は`QuadrupedEnv.step()`の`self.mjData.ctrl = action`のあと`mujoco.mj_step(self.mjModel, self.mjData)`。[11](11_Joint_Torque_and_MuJoCo_Closed_Loop.md)。

## 4. 質量と慣性

### 4.1 XMLリンク質量（実装事実）

`go2.xml`の`<inertial mass=...>`合計は15.206408 kg。world body質量は0。

| body | mass [kg] |
|---|---:|
| `base` | 6.921 |
| 各`*_hip` | 0.678 |
| 各`*_thigh` | 1.152 |
| 各`*_calf` | 0.241352 |

`base`の`diaginertia`は`0.107027 0.0980771 0.0244531`（XML値、kg·m²）。慣性主軸QuaternionもXMLにある。

### 4.2 MPCが使う質量・慣性（別物）

`quadruped_pympc/config.py`のGo2分岐は`mass = 15.019`と、XML`diaginertia`とは異なる3×3 `inertia`を持つ。標準では`simulation_params['use_inertia_recomputation']=True`であり、`QuadrupedEnv.get_base_inertia()`の`mj_fullM`ブロック`[3:6,3:6]`をflattenした`(9,)`をMPCパラメータにする。

`get_base_inertia()`のDocstringはworld frameと書く。厳密なframe意味は[F](appendices/F_Open_Questions.md)。

`QuadrupedEnv.com`は全bodyについて`body_mass[i] * subtree_com[i]`を足して割る。これがMuJoCoの物理的総CoMと一致するかは未検証。[F](appendices/F_Open_Questions.md)。

| 量 | 生成元 | shape | 単位 | Frame | 使用先 |
|---|---|---|---|---|---|
| XML合計質量 | `go2.xml` inertial | scalar | kg | なし | Plantのみ。MPC`mass`キーではない |
| `config.mass` | `config.py` Go2分岐 | scalar | kg | なし | SRBD。15.019 |
| `inertia` | `get_base_inertia().flatten()`（再計算ON時） | `(9,)` | kg·m² | コード上は`mj_fullM[3:6,3:6]`。[F](appendices/F_Open_Questions.md) | MPC parameter。100 Hzで読む |
| `com_pos` | `QuadrupedEnv.com` | `(3,)` | m | W | `state_current['position']` |

## 5. アクチュエータ

標準`go2.xml`の`<motor>`はトルク入力である。実行時`gainprm[0]=1`、`biasprm=0`、`gear=1`。

\[
u_i=\tau_i
\]

対応コード: `go2.xml` の `<motor class=... name="FL_hip" joint="FL_hip_joint"/>` 等。実行時値は`mjModel.actuator_*`。

`actuator_ctrlrange`:

| actuator | ctrlrange [N·m] |
|---|---|
| `*_hip`, `*_thigh` | ±23.7 |
| `*_calf` | ±45.43 |

`actuator_forcerange`は`[0,0]`、`forcelimited=False`である。Simulationのclipは`ctrlrange * 0.9`を使う。

\[
\tau^{limited}=\operatorname{clip}(\tau,0.9\tau_{min},0.9\tau_{max})
\]

対応コード: `simulation/simulation.py` の `run_simulation()`（`tau_soft_limits_scalar = 0.9`）。[11](11_Joint_Torque_and_MuJoCo_Closed_Loop.md)。

位置目標を与える内部PDではない。`ctrl=0`のままなら初期姿勢から崩れる。閉ループ姿勢保持は外部コントローラが必要である。

実機風の低レベル制御

\[
\tau=K_p(q_d-q)+K_d(\dot q_d-\dot q)+\tau_{ff}
\]

は **推奨改善** / 実機低レベル構成であり、標準Wrapperではコメントアウト。[10](10_Stance_and_Swing_Control.md)。

### 5.1 `action`境界

| 入力 | shape | 単位 | frame | 出力 | shape | 単位 | frame |
|---|---|---|---|---|---|---|---|
| `tau.FL/FR/RL/RR` | 各`(3,)` | N·m | 関節 | `action` | `(12,)` | N·m | actuator |

アクチュエータ順はFL, FR, RL, RR × hip, thigh, calf。`env.step(action)`が`mjData.ctrl`へ代入する。周期500 Hz。

## 6. 関節損失

XMLの各関節は`damping`、`armature`、`frictionloss`を持つ。電気モータモデルではなく、簡略化された受動損失である。数値は関節ごとに異なり、左右対称ではない。

理論近似:

\[
\tau_{loss}\approx-b\dot q-\tau_c\operatorname{sgn}(\dot q)
\]

MuJoCoが`qfrc_passive`へ集約する。標準立脚・遊脚のあと`tau -= legs_qfrc_passive`する。[10](10_Stance_and_Swing_Control.md)。

| joint | damping | armature | frictionloss |
|---|---:|---:|---:|
| `FL_hip_joint` | 0.1531 | 0.0010 | 0.2884 |
| `FL_thigh_joint` | 0.2406 | 0.0197 | 0.1739 |
| `FL_calf_joint` | 0.1450 | 0.0356 | 1.0151 |
| `FR_hip_joint` | 0.2628 | 0.0129 | 0.1835 |
| `FR_thigh_joint` | 0.1420 | 0.0135 | 0.2386 |
| `FR_calf_joint` | 0.1728 | 0.0371 | 0.4912 |
| `RL_hip_joint` | 0.2243 | 0.0075 | 0.1550 |
| `RL_thigh_joint` | 0.2352 | 0.0205 | 0.1388 |
| `RL_calf_joint` | 0.1201 | 0.0228 | 1.3573 |
| `RR_hip_joint` | 0.2989 | 0.0081 | 0.2488 |
| `RR_thigh_joint` | 0.1983 | 0.0139 | 0.2484 |
| `RR_calf_joint` | 0.1000 | 0.0332 | 0.7893 |

対応コード: `go2.xml` の各`<joint ... damping= armature= frictionloss=>`。読取は`QuadrupedEnv.legs_qfrc_passive`。

## 7. 足と接触

### 7.1 VisualとCollision

| class | group | 接触 |
|---|---|---|
| `visual` | 2 | `contype=0` `conaffinity=0`。接触なし |
| `collision` | 3 | 接触あり。default `friction="0.6"` `margin="0.001"` |

表示Meshと物理Collision geomは別である。見た目の接触点と物理接触点が一致しない場合がある。

### 7.2 足geom

足先Collisionは球である。4脚とも`size="0.022"`、`priority="1"`、`condim="6"`、XML摩擦`0.8 0.02 0.01`。

他のcollision geomは`condim`未指定のためMuJoCo既定（3）である。`condim=6`は足だけである。

`<option cone="elliptic" impratio="100"/>`。楕円摩擦錐と大きい`impratio`であり、単純なピラミッドCoulombそのものではない。

理論上よく使うCoulomb近似

\[
\sqrt{F_x^2+F_y^2}\leq\mu F_z
\]

はMPC側の`config.mpc_params['mu']`（標準0.42）と、Plant接触の実行時\(\mu\)の両方に出てくるが、値も更新経路も別である。

### 7.3 実行時摩擦の上書き

`QuadrupedEnv.reset()`は毎回`_set_ground_friction(tangential_coeff=...)`を呼ぶ。対象は名前が`ground`/`floor`/`hfield`/`terrain`のgeom、**または** `_feet_geom_id`に入る足geomである。足のXML摩擦`0.8 0.02 0.01`は、reset後の実行値ではない。

`run_simulation()`の既定`friction_coeff=(0.5, 1.0)`なので、接線摩擦は区間一様乱数、ねじり0.005、転がり0.0へ上書きされる。固定値を渡した場合はその値になる。

対応コード: `QuadrupedEnv._set_ground_friction()`、`QuadrupedEnv.reset()`、`simulation/simulation.py` の `run_simulation(friction_coeff=...)`。

## 8. センサ

XML `<sensor>` は次の16個である。

| 名前 | 種類 | dim |
|---|---|---:|
| `imu_acc` | accelerometer @ site `imu` | 3 |
| `imu_gyro` | gyro @ site `imu` | 3 |
| `imu_pos` | framepos | 3 |
| `imu_quat` | framequat | 4 |
| `*_hip/thigh/calf_joint_pos` × 12 | jointpos | 各1 |

`imu` site位置は`pos="-0.02557 0 0.04232"`（base相対、m）。

標準経路では`QuadrupedEnv(..., sensors=None)`であり、カスタム`Sensor`インスタンスは作られない。`state_obs_names=[]`。`simulation.py`は`env.com`、`base_pos`、`qpos`、`qvel`、`feet_*`、`get_base_inertia()`を使う。`sensordata`への参照はない。

したがって「XMLにセンサがない」は誤りである。「標準制御ループはXMLセンサを状態推定に使わない」が実装事実である。

## 9. 初期姿勢

`go2.xml` keyframe `home`:

```
qpos = 0 0 0.27  1 0 0 0  0 0.9 -1.8  0 0.9 -1.8  0 0.9 -1.8  0 0.9 -1.8
```

`run_simulation()`は`env.reset(random=False)`を呼ぶ。`random=False`かつ`qpos is None`の分岐では、XML keyframe 0（`home`）を起点にする。その後、必要なら高さ補正と`mj_step`で接触を安定化する。zを`hip_height`へ置き換える分岐は`random=True`側である。

対応コード: `go2.xml` の `<key name="home">`、`QuadrupedEnv.reset()`。

## 10. Plant境界の入出力

制御→Plant、Plant→制御の契約。次段は同じ変数を受け取る。

| 方向 | 変数 | shape | 単位 | frame | 生成元 | 使用先 | 周期 |
|---|---|---|---|---|---|---|---|
| 制御→Plant | `action` | `(12,)` | N·m | actuator順 | `run_simulation()` | `QuadrupedEnv.step()` → `mjData.ctrl` | 500 Hz |
| Plant内部 | \(\lambda\) | 接触ごと | N | 接触frame→W | MuJoCo接触 | 次`qacc`。指令GRFではない | `mj_step`内 |
| Plant→制御 | `qpos` | `(19,)` | 混在 | §2.1 | `mjData.qpos` | IK、姿勢 | 500 Hz |
| Plant→制御 | `qvel` | `(18,)` | 混在 | §2.2 | `mjData.qvel` | 速度、Jacobian | 500 Hz |
| Plant→制御 | `com` | `(3,)` | m | W | `QuadrupedEnv.com` | `state_current['position']` | 500 Hz |
| Plant→制御 | `feet_pos.*` | 各`(3,)` | m | W | `geom_xpos` | Gait、Foothold、MPC、Swing | 500 Hz |
| Plant→制御 | `feet_jac.*` | 各`(3,18)` | 混在 | W | `mj_jac` | Stance/Swing | 500 Hz |
| Plant→制御 | `legs_qfrc_passive.*` | 各`(3,)` | N·m | 関節 | `qfrc_passive` | `tau -= passive` | 500 Hz |
| Plant→制御 | `get_base_inertia()` | `(3,3)` | kg·m² | [F] | `mj_fullM[3:6,3:6]` | MPC `inertia` | 500 Hz計算 |
| XMLセンサ | `sensordata` | 25 | 混在 | sensor | MuJoCo | 標準ループ未使用 | — |

実接触力をログする場合は`QuadrupedEnv.feet_contact_state(ground_reaction_forces=True)`。標準トルク計算はこの戻りを使わない。viewer表示で呼ぶ。

## 11. MPCとのモデル分担

| モデル | 役割 |
|---|---|
| MuJoCo Go2 | 全身Plant、関節、接触、実際の次状態 |
| Centroidal/SRBD | MPC内部の簡略予測モデル。質量15.019 kg、状態は足先点と胴体 |

これは自動車で、Full vehicle modelをPlant、二輪モデルをMPC内部モデルに使う関係に近い。SRBDの状態・入力は[06](06_Centroidal_SRBD_Model.md)。

## 12. MJX版

この照合対象（`external/Quadruped-PyMPC` 展開tree、gym-quadruped 1.1.5。識別子は[00](00_README.md)）に、Go2のMJX XML切替や`mjx`呼び出しはない。通常MuJoCo版とMJX版の接触差を表にすることは、本スタックでは根拠がない。上流Menagerieや将来のgym-quadruped版との差は[F](appendices/F_Open_Questions.md)。

## 13. 実装事実 / 理論 / 推奨改善 / 未確認

**実装事実**

- 実行時XMLはgym-quadruped同梱`go2.xml`。`nq=19`、`nv=18`、`nu=12`。
- `<motor>`はgain 1のトルク入力。clipは`0.9 * ctrlrange`。
- 足は`condim=6`の球。resetが床と足の`geom_friction`を上書きする。
- XMLセンサ16個は存在するが、標準ループは読まない。
- Plant質量合計15.206 kg。MPC`mass`は15.019。

**理論**

- 全身剛体＋接触の\(M\ddot q+h=S^\top\tau+J_c^\top\lambda\)。
- MPC GRFとMuJoCo \(\lambda\)は別物。

**推奨改善**

- 実験ログでXML摩擦とreset後摩擦、MPC`mu`、実GRFを分けて残す。制御式は変えない。
- 実機低レベルPDは標準経路に入れない。コメントアウトのままBaselineとする。

**未確認**

- 当該`go2.xml`とMenagerie HEADの差分。
- `env.com`と`qvel[3:6]`と`get_base_inertia()`の厳密な物理定義。[F](appendices/F_Open_Questions.md)。
- MJX接触との差。本スタックにMJX経路がない。

## 14. Cursor確認課題

1. Menagerie `unitree_go2/go2.xml`を別途取得し、gym-quadruped同梱XMLとのdiffを[F](appendices/F_Open_Questions.md)へ記録する。
2. `env.com`を`mjData.subtree_com[1]`または公式CoMと比較する。
3. Collision geomだけを可視化し、Visual meshとの差を確認する。
4. `friction_coeff`を固定したときの`geom_friction`をreset直後にログする。
