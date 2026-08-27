# 逆運動学 helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.wb_interface.compute_stance_and_swing_torque(...)  (read_code_12)
          → self.ik.compute_solution(...)   ← 本ファイル、毎周期(既定タイプで)
```

`self.ik`は`WBInterface.__init__`(read_code_06)の中で`InverseKinematicsNumeric()`
として生成される。`compute_solution`は、`read_code_12`で見た
`if cfg.mpc_params['type'] != 'kinodynamic':`の分岐内で呼ばれる。既定の`'nominal'`型は
この条件を満たすため、**毎周期(既定500Hz相当)実際に呼ばれている**。

## このクラスの役割(全体の中での位置づけ)

`InverseKinematicsNumeric`が担当するのは、「**望ましい足先の3次元位置(デカルト座標)を、
それを実現する12個の関節角度に変換する**」ことです。`read_code_12`で見た
`des_foot_pos`(MPC/フットホールド計画が出した目標足先位置)を`des_joints_pos`
(関節角度指令)へ変換する、WBCパイプラインの最終段の一つです。

内部でMuJoCoの順運動学(`mj_fwdPosition`)とヤコビアン(`feet_jacobians`)を使い、
反復的にニュートン型の数値解法で解く。目標軌道の生成(`read_code_13`)や早期接地の検知
(`read_code_14`)は行わない。

対象は
`external/Quadruped-PyMPC/quadruped_pympc/helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py`
(201行)です。

---

## 1〜37行:モジュールレベルの定数とインポート

```python
IT_MAX = 5
DT = 1e-2
damp = 1e-3
damp_matrix = damp * np.eye(12)
```

- `IT_MAX`(無次元、反復回数)：IK反復ループの最大回数。`5`固定(クラス外の定数、`config.py`には存在しない)
- `DT`(秒)：関節速度を関節角度へ積分する際の仮想時間刻み。`1e-2`固定
- `damp`(無次元)：減衰最小二乗法(damped least squares)の減衰係数。`1e-3`固定
- `damp_matrix`：12×12の単位行列に`damp`を掛けたもの。関節数12(4脚×3関節)分の減衰項

**事実**：`IT_MAX`/`DT`/`damp`はいずれもモジュールレベルの定数としてハードコードされており、
`config.py`をgrepしても該当キーは存在しない。実行時に変更する手段はない。

---

## 42〜55行:`__init__`

この関数の役割:IK計算専用の、内部だけで使うMuJoCo環境(`QuadrupedEnv`)を1つ生成する。

```python
def __init__(self) -> None:
    robot_name = cfg.robot

    # Create the quadruped robot environment ---------------------
    self.env = QuadrupedEnv(
        robot=robot_name,
    )
```

- `robot_name`：`config.py`の`cfg.robot`。既定`'go2'`
- `self.env`：`gym_quadruped.quadruped_env.QuadrupedEnv`の新規インスタンス。`robot`以外は全て
  デフォルト値(`scene='flat'`、`sim_dt=0.002`秒 など)

**事実**：`simulation.py`本体が持つ物理シミュレーション用の`env`とは**別の、IK専用の
MuJoCoインスタンス**である。`mj_step`(物理積分)は一切呼ばれず、`mj_fwdPosition`
(姿勢から運動学量を再計算するだけの関数)のみが使われる。つまりこのMuJoCoインスタンスは
「動かない計算機」として、順運動学とヤコビアンの計算だけに使われている。

---

## 57〜122行:`compute_solution`

この関数の役割:初期関節角度と4本の目標足先位置から、減衰最小二乗法によるニュートン反復で
目標を実現する12個の関節角度を求める。

### 79〜81行:初期姿勢のセット

```python
self.env.mjData.qpos = q
mujoco.mj_fwdPosition(self.env.mjModel, self.env.mjData)
```

- `q`(`np.ndarray`、長さ19)：`[base位置3, baseクォータニオン4, 関節角度12]`の初期姿勢。
  `read_code_12`から渡される値は`copy.deepcopy(qpos)`(現在のbase位置・姿勢はそのまま、
  関節角度も現在値)。単位はm(位置)/無次元(クォータニオン)/rad(関節角)
- `mj_fwdPosition`：MuJoCoの姿勢依存の運動学量(前進運動学、ヤコビアン等)だけを`qpos`から
  再計算する関数。積分は行わない

### 83〜121行:反復ループ本体(`IT_MAX=5`回固定)

```python
for j in range(IT_MAX):
    feet_pos = self.env.feet_pos(frame='world')

    FL_foot_actual_pos = feet_pos.FL
    FR_foot_actual_pos = feet_pos.FR
    RL_foot_actual_pos = feet_pos.RL
    RR_foot_actual_pos = feet_pos.RR

    err_FL = FL_foot_target_position - FL_foot_actual_pos
    err_FR = FR_foot_target_position - FR_foot_actual_pos
    err_RL = RL_foot_target_position - RL_foot_actual_pos
    err_RR = RR_foot_target_position - RR_foot_actual_pos

    # Compute feet jacobian
    feet_jac = self.env.feet_jacobians(frame='world', return_rot_jac=False)

    J_FL = feet_jac.FL[:, 6:]
    J_FR = feet_jac.FR[:, 6:]
    J_RL = feet_jac.RL[:, 6:]
    J_RR = feet_jac.RR[:, 6:]

    total_jac = np.vstack((J_FL, J_FR, J_RL, J_RR))
    total_err = 100*np.hstack((err_FL, err_FR, err_RL, err_RR))

    # Solve the IK problem
    damped_pinv = np.linalg.inv(total_jac.T @ total_jac + damp_matrix) @ total_jac.T
    dq = damped_pinv @ total_err

    q_joint = self.env.mjData.qpos.copy()[7:]
    q_joint += dq * DT
    self.env.mjData.qpos[7:] = q_joint

    mujoco.mj_fwdPosition(self.env.mjModel, self.env.mjData)

return q_joint
```

- `err_FL`など(m)：目標足先位置と現在の足先位置の差(world座標系)。`feet_pos(frame='world')`は
  world座標系での並進足先位置(`gym_quadruped`側の実装、`read_code_01`で確認済み)
- `feet_jac`：`feet_jacobians(frame='world', return_rot_jac=False)`で得る各脚の並進ヤコビアン
  (回転成分は含まない)。1脚あたり3行×自由度列
- `J_FL = feet_jac.FL[:, 6:]`：ヤコビアン全列のうち先頭6列(base自由度、free jointの6自由度)を
  除いた、関節角速度に対応する列だけを抜き出す。1脚は3関節なので`J_FL`は3×3
- `total_jac`：4脚分の`J_FL/FR/RL/RR`を縦に積んだ12×12行列(`vstack`)
- `total_err`：4脚分の位置誤差を横に並べた長さ12のベクトル(`hstack`)に、係数`100`を掛けたもの
- `damped_pinv`：`(J^T J + damp*I)^-1 J^T`という減衰最小二乗法の擬似逆行列(damped
  pseudo-inverse)。特異点(ヤコビアンが低ランクになる姿勢)近くでも数値的に安定して解ける
- `dq`(rad/DT、実質的には各反復の関節角度増分)：`damped_pinv @ total_err`
- `q_joint`：現在の関節角度(`qpos[7:]`、先頭7要素はbase位置3+クォータニオン4なので除く)に
  `dq * DT`を加算し、次の反復の初期値として`self.env.mjData.qpos[7:]`へ書き戻す
- ループ後、`mj_fwdPosition`で姿勢を再計算してから次の反復へ進む。これを`IT_MAX=5`回繰り返し、
  最後の`q_joint`(rad、長さ12)を返す

**事実**：コメントアウトされた2つの代替式(109行目のヤコビアン転置ベースの式、120行目の
`mj_step`/`mj_kinematics`呼び出し)が残っている。現在有効なのは`(J^T J + damp*I)^-1 J^T`の
形と`mj_fwdPosition`のみ。

**設計上の解釈**：`total_err`に`100`という係数が掛けられている理由はコードにコメントがなく
不明(**未確認**)。誤差(m単位、通常は数cm〜数mmオーダー)を人為的に拡大することで、
`dq*DT`のステップ幅を実質的に大きくし、少ない反復回数(`IT_MAX=5`)でも収束を早めるための
経験的なスケーリングと推測されるが、根拠となるコメントや文献参照はコード中にない。

**実装上の注意点**：反復回数は`IT_MAX=5`固定であり、収束判定(誤差がしきい値以下になったら
打ち切る、といった処理)は存在しない。つまりこの関数は「常にちょうど5回反復する」動作をし、
早期収束時の計算節約や、未収束時の警告は行わない。

---

## 125〜201行:`if __name__ == "__main__":`

この関数の役割:モジュール単体で動かした際に、ランダムな姿勢からIKを解き、収束結果を
MuJoCoビューアで目視確認するためのデモスクリプト。

- `robot_cfg`ごとのXMLパスを`gym_quadruped_path`から組み立て、ランダムな関節角・base姿勢を
  持つ`MjModel`/`MjData`を作り、その足先ジオメトリ位置(`geom_xpos`)を目標位置とする
- 新しい`InverseKinematicsNumeric`インスタンスを作り、**別のランダム初期関節角**から
  `compute_solution`を呼んで、目標位置にどれだけ近づけたかを`print`で比較する
- 最後に`mujoco.viewer.launch_passive`でビューアを開き、無限ループで表示し続ける
  (`while True: viewer.sync()`、終了条件なし)

**事実**：このブロックは`read_code_13`の`swing_trajectory_controller.py`の`__main__`同様、
`WBInterface`経由の通常のパイプラインからは呼ばれない、独立した検証用デモである。
`go1`ロボットの分岐(128行目)は`if`(129行目`elif`ではない)になっており、`go2`の`if`文
(126〜127行目)と合わせて2つの独立した`if`が並ぶ構造になっている。`cfg.robot`が`'go2'`の
場合、127行目で`xml_filename`が確定した後、128行目の`if cfg.robot == 'go1':`は素通りするだけ
なので実害はないが、意図としては`elif`にすべき書き方(軽微なコードの書き方の乱れ、
**未確認**なほどには重要ではないが指摘しておく)。

---

## この章のまとめ

- 見つかった実装上の特徴・注意点:
  1. `IT_MAX`/`DT`/`damp`は`config.py`に存在しない、モジュールレベルのハードコード定数
  2. 収束判定なしの固定5回反復。誤差が5回で収束しきらない場合も打ち切られてそのまま返る
  3. 誤差ベクトルに掛かる`100`倍のスケーリングの根拠はコード中に説明がなく未確認
  4. IK専用に、物理シミュレーションとは別の`QuadrupedEnv`インスタンスを1つ保持している
     (`mj_step`は使わず`mj_fwdPosition`のみ使用)
- 確認できた重要な事実:
  - 既定の`'nominal'`型では、`WBInterface.compute_stance_and_swing_torque`から
    **毎周期**呼ばれる、既定で有効な処理である(`read_code_14`の`EarlyStanceDetector`と
    同様、「既定OFF」が多いこの周辺コードの中では例外)
  - これで、`read_code_06`の`WBInterface.__init__`で確認した主要コンポーネント
    (`pgg`, `frg`, `stc`, `terrain_computation`, `vm`, `esd`, `ik`)は全て読み終えました
- 次に読む候補(まだ未指定、提案):
  1. ROS2の二重構成(`ros2/run_controller.py`、`ros2/run_simulator.py`) — 概要レベルでは
     既存の別ドキュメントで触れたが、`read_code`形式の逐次解説はまだない
  2. Visual Foothold Adaptation(`helpers/visual_foothold_adaptation.py`、既定OFF)
  3. サンプリング/JAXベースのMPC経路(既定OFF)
  4. `SRBDBatchedControllerInterface`(既定OFF)
  5. スイング軌道生成器の実体(`swing_generators/scipy_swing_trajectory_generator.py`、
     `explicit_swing_trajectory_generator.py` — `read_code_13`で参照したが中身は未読)
