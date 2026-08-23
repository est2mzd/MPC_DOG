# Log 05: MuJoCo Go2 Plant

対応プロンプト: 実行時Go2モデルの自由度・関節・接触・センサを確定する。本文未修正。
実行時XML: `.venv/.../gym_quadruped/robot_model/go2/go2.xml`。Menagerieは未ロード。

## A. Joint / Actuator

`nq=19`, `nv=18`, `nu=12`。freejoint（無名）が qpos 0:7 / qvel 0:6。脚順 FL, FR, RL, RR。各脚 hip / thigh / calf。

| 脚 | Joint | qpos | qvel | Actuator | action | ctrlrange（XML） |
|---|---|---:|---:|---|---:|---|
| FL | `FL_hip_joint` | 7 | 6 | `FL_hip` motor | 0 | 要XML（hip） |
| FL | `FL_thigh_joint` | 8 | 7 | `FL_thigh` | 1 | 要XML（front thigh class） |
| FL | `FL_calf_joint` | 9 | 8 | `FL_calf` | 2 | 要XML（calf） |
| FR | `FR_hip_joint` | 10 | 9 | `FR_hip` | 3 | 同上 |
| FR | `FR_thigh_joint` | 11 | 10 | `FR_thigh` | 4 | 同上 |
| FR | `FR_calf_joint` | 12 | 11 | `FR_calf` | 5 | 同上 |
| RL | `RL_hip_joint` | 13 | 12 | `RL_hip` | 6 | 同上 |
| RL | `RL_thigh_joint` | 14 | 13 | `RL_thigh` | 7 | rear thigh class（前脚と可動域が違う） |
| RL | `RL_calf_joint` | 15 | 14 | `RL_calf` | 8 | 同上 |
| RR | `RR_hip_joint` | 16 | 15 | `RR_hip` | 9 | 同上 |
| RR | `RR_thigh_joint` | 17 | 16 | `RR_thigh` | 10 | rear thigh |
| RR | `RR_calf_joint` | 18 | 17 | `RR_calf` | 11 | 同上 |

種類はすべてトルク `<motor>`。`run_simulation` は `0.9 * actuator_ctrlrange` でclipして `action[legs_tau_idx]` に入れる。

数値の完全表は `01_MuJoCo_Go2_Plant_Model.md` に既掲。本ログは「実行時XMLはgym-quadruped同梱」とindex対応を確定する。

## B. Link物性

XMLの `<body mass>` / `<inertial>` がPlant。MPC `config.mass=15.019` と XML合計質量は一致しない（MPCは簡略値）。詳細数値は `01` 本文。

## C. Contact

足は collision capsule/sphere。`condim` はXML定義。`QuadrupedEnv.reset()` の `_set_ground_friction()` が床と足geom摩擦を上書きする。`run_simulation` 既定 `friction_coeff=(0.5, 1.0)`。XML初期摩擦は実行時摩擦ではない。

## D. モデルに含まれない実機要素

実装事実（XML/コードに無い）:

- モータ電気系、通信遅延、バックラッシュ
- 実機関節PD（wrapperのPD加算はコメントアウト）
- センサノイズ（センサ定義はあるが標準経路は未読）
- 足裏ゴムの実測コンプライアンス

一般論（コード根拠なし）: 実機同定誤差、温度、摩耗。

## E. `01_MuJoCo_Go2_Plant_Model.md` 照合

| 節 | 判定 |
|---|---|
| 実行時XMLはgym-quadruped同梱 | 正しい |
| Menagerie未ロード | 正しい |
| nq/nv/nu | 正しい |
| 12 motor | 正しい |
| センサ16個あるが標準未使用 | 正しい |
| 摩擦上書き | 正しい |
| MJX切替なし | 正しい |
| XML質量 vs MPC質量の差 | 正しい（値の再ハッシュは本ログでは未再実行） |
| Menagerieとの差分 | 未確認 |

## 実機にないもの（再掲）

実装事実と一般論は上D。`01` 本文のこの区別は維持してよい。
