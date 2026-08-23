# MPC and Controller Tuning

## 1. 結論

平地低速Trotは既定値から始められるが、不整地・高速・実機ではMPC重み、Gait timing、Foothold、Swing、低レベルが連成する。仕様書で決まる質量、慣性、定格値は本章の調整対象から除外する。

## 2. ユーザー調整項目

| 優先 | レイヤー | 項目 | 主な影響 |
|---:|---|---|---|
| A | 運用 | 目標速度上限 | Gaitで実現可能な速度域 |
| A | Gait | Gait type | 脚順・支持様式 |
| A | Gait | Step frequency | 必要歩幅、接触切替 |
| A | Gait | Duty factor | Stance/Swing時間 |
| A | Swing | Step height | 障害物余裕と上下運動 |
| A | MPC | 高さ重み | 鉛直支持・沈み込み |
| A | MPC | 速度重み | 指令追従 |
| A | MPC | Roll/Pitch重み | 胴体水平性 |
| A | MPC | 角速度重み | 姿勢振動減衰 |
| A | MPC | GRF重み | 力の大きさと追従の妥協 |
| A | Swing | Position/velocity gain | 遊脚追従と振動 |
| B | MPC | Foot position weight | Foothold忠実度。`set_weight()` |
| B | MPC | Foot velocity weight | 足運びの滑らかさ。既定 `[1e-4,1e-4,1e-5]` |
| B | MPC | Horizon/dt | 予見性と計算量 |
| B | Foothold | 速度補正・Clip | 歩幅と速度誤差補償 |
| B | 制約 | Foothold / Stability | **標準OFF**。`use_foothold_constraints` / stability フラグ |
| D | MPC | GRF rate weight | **`type='input_rates'` 専用**。nominal未実装。[E](appendices/E_Corrections_and_Clarifications.md) §22 |
| C | 上位 | 速度別周波数候補 | 標準OFF。`optimize_step_freq`。[12](12_Speed_Frequency_Duty_and_Stride.md) §6 |
| C | 上位 | Gait切替点 | **パラメータ未実装**。運用ルールもない |
| C | 上位 | 減速・停止規則 | **未実装**。`start_and_stop` はsimオフ |
| C | MPC | Integral gain/cap | 標準OFF。`use_integrators` |
| C | MPC | Soft constraint penalty | 標準ではslack未構築 |
| C | 接触 | Early stance/reflex | 標準OFF。`reflex_trigger_mode=False` |
| C | 低レベル | Joint impedance | **実装あり・標準無効**（コメントアウト） |

## 3. 症状からの逆引き

| 症状 | 最初に確認 | 次に調整 |
|---|---|---|
| 胴体が沈む | GRF saturation、Height reference | Height/GRF重み |
| Roll/Pitch振動 | State estimate、接触列 | Angle/rate重み |
| 速度追従が遅い | Torque/GRF saturation | VelocityとGRF重み |
| 接触時Torqueが急変 | GRF時系列 | 標準ではGRF重み。rate重みは`input_rates`のみ |
| 遊脚が振動 | Swing軌道 | Swing Kp/Kd |
| 着地点に届かない | Swing残時間・IK | Frequency、Foothold、gain |
| 足が滑る | 実\(\mu\)、実GRF | \(\mu\)の保守化、速度・GRF |
| 穴の縁へ着く | Terrain map | 標準blindでは保証なし。[13](13_Feasibility_on_Rough_Terrain.md) |
| Solver infeasible | Hard constraints | 標準は摩擦のみ。安定/足箱を足さない |

## 4. 推奨調整順

1. Full stanceで高さ・Roll/Pitch・外乱復帰。
2. Swing単体で位置・速度Gain。
3. 低速Trotでfrequency、Duty、Step height。
4. 速度・Foot・GRF重み。
5. Horizon。Foothold制約は標準OFFのまま評価してからONにする。
6. 速度域ごとのFrequency/Gait（`optimize_step_freq`は標準OFF）。
7. 地形はblindの失敗を先に記録。[13](13_Feasibility_on_Rough_Terrain.md)
8. 実機前にIntegral/Reflex/Impedanceを**個別に**有効化する（標準OFF）。

数値表の正本は[07](07_MPC_Formulation.md) §4。実験段階の正本は[18](18_Experiments_and_Research_Roadmap.md)。

## 5. ADAS MPCとの対応

| ADAS操舵MPC | 四脚MPC |
|---|---|
| 横偏差・Yaw誤差重み | 速度・姿勢重み |
| 操舵角・操舵速度重み | GRF・Foot velocity（GRF rateは`input_rates`のみ） |
| 摩擦円 | 足の摩擦錐 |
| 操舵範囲 | Foothold/関節到達域 |
| \(\mu\)変動 | 足裏摩擦・地形 |
| Actuator delay | Motor・通信・推定遅延 |

四脚固有の難しさは、利用可能な接触脚が時間で切り替わる点である。

## 6. 対応コード

- `config.py`
- `centroidal_nmpc_nominal.py:set_weight()`
- `swing_trajectory_controller.py`
- `foothold_reference_generator.py`
- `visual_foothold_adaptation.py`

## 7. Cursor確認課題

全調整値をYAML等へ外出しし、実験ID、Git commit、値、評価指標を自動記録する変更計画を作る。