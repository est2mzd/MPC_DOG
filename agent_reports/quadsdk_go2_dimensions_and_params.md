# go2 の寸法・質量・関節・パラメータ(Quad-SDK モデル + 公称スペック)

作成: 2026-08-31。`external/quad-sdk` の MuJoCo モデル・URDF・YAML から
実際に読み取った値。**【事実】=リポジトリのファイルで確認済み**、
**【参考】=Unitree 公称スペック(一般情報、リポジトリ外)** として分ける。
関連: `agent_reports/quadsdk_step01_mpc.md`、`agent_reports/quadsdk_step01_wbc.md`、
`agent_reports/quadsdk_step01_simple_model_terrain_and_gaps.md`。

---

## 1. 【事実】MuJoCo モデルの寸法

出所: `quad_simulator/go2_description/models/go2/go2_mjc/go2.xml`
(Unitree 公式 MJCF)。

### 脚配置(胴体 COM 基準)

- **前後の股関節間(ホイールベース)**
  - FL/FR 股関節 `pos x = +0.1934`、RL/RR 股関節 `pos x = -0.1934`
  - → **前後スパン = 0.3868 m**
- **左右の股関節間**
  - 股関節 `pos y = ±0.0465`
  - 腿(thigh)の回転軸は股から `y = ±0.0955` さらに外 → `y = ±0.141`
  - → 股関節間 **約 0.093 m**、腿間 **約 0.282 m**
- **リンク長**
  - 腿(thigh):`0.213 m`(calf ボディが thigh から `z = -0.213`)
  - すね(calf):`0.213 m`(foot が calf から `z = -0.213`)
  - → 脚の全伸長(股 → 足先)は約 **0.426 m**
- **足先(toe)半径**:`0.022 m`(`go2.yaml: toe_radius`、mjcf の foot geom `size 0.022` と一致)

### 胴体

- コリジョン box:half-extent `size = "0.1881 0.04675 0.057"`
  - → 判定用の胴体寸法 **0.376 × 0.094 × 0.114 m**(L×W×H)
  - 実際の外殻(visual メッシュ)はこれより一回り大きい
- 追加のコリジョン: 前方に円柱 `size 0.05 0.045 @ pos (0.285,0,0.01)`、
  球 `size 0.047 @ pos (0.293,0,-0.06)`(頭部・センサ部の当たり)

### 立位姿勢(mjcf の `home` keyframe)

- `qpos = "0 0 0.27  1 0 0 0  0 0.9 -1.8  0 0.9 -1.8  0 0.9 -1.8  0 0.9 -1.8"`
  - **base 高さ z = 0.27 m**
  - 各脚の関節角 `[abad, hip, knee] = [0, 0.9, -1.8]` rad
- Step 01 の起動姿勢は `init_pose: "-x 0.0 -y 0.0 -z 0.5"`(スポーン時)
  → 落として着地させる

---

## 2. 【事実】質量・慣性

出所: `go2.xml` の `<inertial>`、`quad_utils/config/go2.yaml`、
`nmpc_controller/scripts/main.m`(コメント)。

- **MuJoCo モデルの inertial 合計**
  - 胴体(base_link):`mass = 6.921 kg`
  - 股(hip)× 4:`0.678 kg` 各
  - 腿(thigh)× 4:`1.152 kg` 各
  - すね(calf)× 4:`0.241 kg` 各
  - → 合計 ≈ 6.921 + 4 × (0.678 + 1.152 + 0.241) ≈ **15.2 kg**
- **Quad-SDK の planner/MPC が使う値**(`go2.yaml`)
  - `global_body_planner.mass: 16.1 kg`(NMPC の `u_nom` = 体重項に使用。
    inertial 合計より少し大きめ)
- **`nmpc_controller/scripts/main.m`(コメント値、go2 用)**
  - 胴体単体質量 `7.279 kg`、1脚あたり `2.242 kg`
  - 股オフセット `hip_offset = [0.2263, 0.07, 0]`(絶対値、胴体 COM から)

---

## 3. 【事実】関節の可動域とトルク・速度限界

出所: `go2.xml` の `<default>`、`quad_utils/config/go2.yaml`。

### 関節可動域(mjcf `range`)

- **abad(股ロール, `abduction`)**:`-1.0472 .. 1.0472` rad(±60°)
- **hip(腿ピッチ, `hip`)**
  - 前脚(`front_hip`):`-1.5708 .. 3.4907` rad
  - 後脚(`back_hip`):`-0.5236 .. 4.5379` rad
- **knee(すねピッチ, `knee`)**:`-2.7227 .. -0.83776` rad

### トルク限界

- **mjcf のアクチュエータ `ctrlrange`**
  - abad / hip:`±23.7 N·m`
  - knee:`±45.43 N·m`
- **Quad-SDK の `go2.yaml: motor_limits.torque`**(planner/controller が使う値)
  - `[33.5, 33.5, 50.0] N·m`([abad, hip, knee])
  - mjcf の値と一致していない(Quad-SDK 側がやや大きめ。
    `agent_reports/quadsdk_step01_wbc.md` 参照)

### 速度限界

- **`go2.yaml: motor_limits.speed`**:`[30.0, 30.0, 20.06] rad/s`
- NMPC の関節速度ソフト境界(go2.yaml `nmpc_controller.joints`)も
  これに合わせて `±30 / ±30 / ±15.7` に絞られている
  (`agent_reports/quadsdk_step01_mpc.md`)

---

## 4. 【事実】歩行・プランナの幾何パラメータ(`go2.yaml`)

- 公称胴体高さ:`h_nom = desired_height = 0.30 m`
- 胴体高さ範囲:`h_max = 0.375 m`、対地最小クリアランス `h_min = 0.075 m`
- 脚ベース〜胴体底:`robot_h = 0.05 m`
- 胴体寸法(プランナ用):`robot_l = 0.3 m`、`robot_w = 0.3 m`
- 最大速度:`v_max = 2.0 m/s`、公称速度:`v_nom = 0.75 m/s`
- 歩容(トロット):`period = 0.36 s`、`duty_cycles = [0.5]×4`、
  `phase_offsets = [0, 0.5, 0.5, 0]`、`ground_clearance = 0.07 m`
- 足場探索半径:`foothold_search_radius = 0.25 m`
- 摩擦係数(NMPC):`friction_coefficient = 0.6`(go2.yaml が
  nmpc_controller.yaml の 0.3 を上書き)
- GRF 鉛直成分の範囲(NMPC):`10 .. 150 N`/脚(`nmpc_controller.body.u_ub`)

---

## 5. 【参考】Unitree Go2 公称スペック(リポジトリ外・一般情報)

- 立位寸法:約 **0.70 × 0.31 × 0.40 m**(全長 × 全幅 × 全高)
  - 全長 0.70 m は「鼻先〜尻」で、上の「股関節間 0.387 m」より当然大きい
- 質量:約 **15 kg**(base モデル)
- 定格ペイロード:約 3 kg、最大 ~7–8 kg
- 最高歩行速度:公称 3–5 m/s(モデルによる)
- 関節数:12(3 自由度 × 4 脚)

> 公称スペックはモデル/ロット/公開資料で差があるため目安。制御・
> シミュレーションで使うのは 1〜4 節のリポジトリ内の値。

---

## 6. ソース早見表

- MuJoCo モデル(寸法・質量・関節・アクチュエータ)
  - `external/quad-sdk/quad_simulator/go2_description/models/go2/go2_mjc/go2.xml`
- URDF(制御・運動学用)
  - `external/quad-sdk/quad_simulator/go2_description/models/go2/urdf/go2.urdf.xacro`
- Quad-SDK のパラメータ(planner / MPC / driver が読む値)
  - `external/quad-sdk/quad_utils/config/go2.yaml`
- NMPC のモデル生成(コメントに質量・慣性・hip_offset)
  - `external/quad-sdk/nmpc_controller/scripts/main.m`
