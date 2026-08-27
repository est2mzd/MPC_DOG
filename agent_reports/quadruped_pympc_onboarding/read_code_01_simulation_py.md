# 単体プロセス版① simulation/simulation.py 逐次解説

対象は現在の `external/Quadruped-PyMPC/simulation/simulation.py`(365行)です。先頭から末尾まで、コードを数行ずつ引用しながら順番に説明します。

---

## 1〜25行：import

```python
import pathlib
import time
from os import PathLike
from pprint import pprint
import copy
import numpy as np
import mujoco
```

- `pathlib`：後で記録用のファイルパスを組み立てるために使用
- `numpy` / `mujoco`：数値計算とMuJoCo公式Python bindingの標準的な依存
- それ以外(`time`, `PathLike`, `pprint`, `copy`)：補助的なユーティリティ

```python
from gym_quadruped.quadruped_env import QuadrupedEnv
from gym_quadruped.utils.mujoco.visual import render_sphere, render_vector
from gym_quadruped.utils.quadruped_utils import LegsAttr
from tqdm import tqdm
```

- `gym_quadruped`：Quadruped-PyMPCとは**別のPythonパッケージ**(`.venv/lib/python3.11/site-packages/gym_quadruped/`)
- `QuadrupedEnv`：MuJoCoの生の`mjModel`/`mjData`を、Gymっぽいインターフェースでラップしたクラス
- `LegsAttr`：`FL`/`FR`/`RL`/`RR`の4値を持つ入れ物のクラス。このファイルのいたるところで使われる
- `render_sphere`, `render_vector`：Viewer上への描画ヘルパー(制御には無関係)
- `tqdm`：進捗バー表示用。制御には関係しない

```python
from quadruped_pympc.helpers.quadruped_utils import plot_swing_mujoco
from quadruped_pympc.quadruped_pympc_wrapper import QuadrupedPyMPC_Wrapper
```

- ここからがQuadruped-PyMPC自身のコード
- `QuadrupedPyMPC_Wrapper`：歩容生成・着地点生成・MPC・トルク変換をすべてまとめた「制御ロジック全体の入口」
- `simulation.py`自身はMPCの数式を一切知らず、このラッパーに投げるだけ

---

## 28〜42行：関数シグネチャとNumPy設定

```python
def run_simulation(
    qpympc_cfg,
    process=0,
    num_episodes=500,
    num_seconds_per_episode=60,
    ref_base_lin_vel=(0.0, 4.0),
    ref_base_ang_vel=(-0.4, 0.4),
    friction_coeff=(0.5, 1.0),
    base_vel_command_type="human",
    seed=0,
    render=True,
    recording_path: PathLike = None,
):
```

- `qpympc_cfg`だけが必須引数、残りはすべてデフォルト値あり
- `qpympc_cfg`には通常`quadruped_pympc/config.py`モジュールがそのまま渡される(365行目の`__main__`で確認できる)
- `num_episodes=500`・`num_seconds_per_episode=60`：デフォルトでは「60秒のエピソードを500回」の意味だが、実際には終了条件(転倒など)で早期に打ち切られることが多い

```python
np.set_printoptions(precision=3, suppress=True)
np.random.seed(seed)
```

- `np.set_printoptions(...)`：numpyの表示桁数を3桁に、指数表記をオフにする(ターミナル出力を見やすくするだけ)
- `np.random.seed(seed)`：乱数を固定し再現性を確保
- ただしMuJoCo Viewerの描画タイミングはリアルタイムクロック(`time.time()`)に依存するため、完全な再現性があるのは物理・乱数部分のみ

---

## 44〜49行：設定値を短い変数名に移す

```python
robot_name = qpympc_cfg.robot
```

`config.py`で選んだロボット名。Go2なら`"go2"`のような文字列。

```python
hip_height = qpympc_cfg.hip_height
```

- ロボットの基準となる腰の高さ。単位はm
- 59行目で、速度指令の範囲をロボットのサイズに合わせてスケーリングするために使う

```python
robot_leg_joints = qpympc_cfg.robot_leg_joints
robot_feet_geom_names = qpympc_cfg.robot_feet_geom_names
```

- `robot_leg_joints`：脚を構成する関節名の辞書
- `robot_feet_geom_names`：MuJoCo上の足先形状の名前
- この`simulation.py`の中では、この2つはこのあと**一度も参照されない**(代入したのに使われない変数)

```python
scene_name = qpympc_cfg.simulation_params["scene"]
```

使用するMuJoCoシーンの名前。平坦路・段差・傾斜路などを表す文字列。

```python
simulation_dt = qpympc_cfg.simulation_params["dt"]
```

- MuJoCo物理シミュレーションの時間刻み。単位は秒
- 既定値は`0.002`で、

$$
f_{\mathrm{sim}} = \frac{1}{0.002} = 500\ \mathrm{Hz}
$$

- この`simulation_dt`と、後で出てくるMPCの計算周波数(既定100Hz)は別物
- 500Hzのうち5ステップに1回だけMPCを解く、という間引き処理が`quadrupedpympc_wrapper`の内部で行われる(134行目、後述)

---

## 51〜52行：env.step()から返す観測項目

```python
state_obs_names = []  # list(QuadrupedEnv.ALL_OBS)  # + list(IMU.ALL_OBS)
```

- `QuadrupedEnv`が通常の観測`state`として返す項目名のリスト
- 現在は空リストなので、後の`state, reward, is_terminated, is_truncated, info = env.step(...)`の`state`には実質何も入らない
- 誤解しやすい点：これはMPCに必要な状態が取れていない、という意味ではない
  - MPCが実際に使う値は、この`state`ではなく`env.feet_pos(...)`・`env.base_lin_vel(...)`・`env.mjData.qpos`のように`env`から個別に直接取得する(173行目以降)
- コメントアウトされている`list(QuadrupedEnv.ALL_OBS)`を使えば、環境が提供できる全観測をこの`state`に含めることもできるが、既定では使われていない

---

## 54〜64行：MuJoCo環境の生成

```python
env = QuadrupedEnv(
    robot=robot_name,
    scene=scene_name,
    sim_dt=simulation_dt,
    ref_base_lin_vel=np.asarray(ref_base_lin_vel) * hip_height,
    ref_base_ang_vel=ref_base_ang_vel,
    ground_friction_coeff=friction_coeff,
    base_vel_command_type=base_vel_command_type,
    state_obs_names=tuple(state_obs_names),
)
```

この`env`が、単体プロセス版における「ロボットの身体+センサー+物理シミュレータ」に相当する。各引数の意味:

- `robot=robot_name`
  - 使用するロボットモデルの指定
  - `gym_quadruped`側が対応するMuJoCo XMLモデルを読み込む
- `scene=scene_name`
  - 地面や障害物などの環境を決める
  - 注意：シーンに凹凸があることと、コントローラがその凹凸を認識できることは別問題。後者にはHeightMap(95行目以降)のような仕組みが要る。既定設定(`visual_foothold_adaptation='blind'`)では、シーンを段差ありに変えてもコントローラはその凹凸を「見て」いない
- `sim_dt=simulation_dt`
  - MuJoCoの物理計算を1回進める時間刻み
  - これはMPCの計算周期とは限らない。たとえばMuJoCoが500Hz、MPCが100Hzなら、MuJoCoを5ステップ進めるごとにMPCを1回だけ更新する、という間引きが起きる
- `ref_base_lin_vel=np.asarray(ref_base_lin_vel) * hip_height`
  - 胴体の目標並進速度の**範囲**(現在の速度指令そのものではない)を設定
  - デフォルト引数`ref_base_lin_vel=(0.0, 4.0)`に`hip_height`を掛ける
  - 例：`hip_height=0.3`なら$[0.0,4.0]\times0.3=[0.0,1.2]$ → 目標速度の範囲は0〜1.2 m/s
  - 現在の値そのものは、あとで`env.target_base_vel()`(183行目)から別に取得する
- `ref_base_ang_vel=ref_base_ang_vel`
  - 目標角速度の範囲。デフォルトは`(-0.4, 0.4)`、単位はrad/s
  - 主にyaw方向(旋回速度)の指令に使われる
- `ground_friction_coeff=friction_coeff`
  - 地面の摩擦係数。デフォルトは`(0.5, 1.0)`
  - これは**MuJoCoが実際に使う地面摩擦**であり、MPC内部(`centroidal_nmpc_nominal.py`の摩擦錐制約)の摩擦係数とは別物
  - 両者が大きく食い違うと、「MPCは滑らないと予測しているのに、MuJoCo上では実際に滑る」というモデル誤差が起きうる
- `base_vel_command_type=base_vel_command_type`
  - 速度指令の与え方の種類。`gym_quadruped`側のdocstringによれば:
    - `"forward"`：前進固定方向
    - `"random"`：ランダムな方向
    - `"forward+rotate"`：前進+旋回
    - `"human"`：キーボード操作(このファイルの既定)
- `state_obs_names=tuple(state_obs_names)`
  - 52行目で作った空リストをタプルに変換して渡すだけ。結果は`()`

この呼び出しが終わると、内部でMuJoCoの`mjModel`(変更されにくいモデル定義)と`mjData`(時々刻々変化する状態)が生成される。

---

## 65〜76行：環境の初期設定

```python
pprint(env.get_hyperparameters())
```

作成された環境の設定値をターミナルへ表示するだけ。制御処理には影響しない。

```python
env.mjModel.opt.gravity[2] = -qpympc_cfg.gravity_constant
```

- MuJoCoのz方向の重力加速度を上書き
- 例：`gravity_constant=9.81` → `gravity=[0,0,-9.81]`(z軸上向きの座標系なので負方向)
- この値は、MPC側の力学モデル(`centroidal_model_nominal.py`)が使う`self.gravity_constant = config.gravity_constant`と**同じ`config.py`由来の値**
- MuJoCo側とMPC側の重力定数をここで一致させている

```python
if qpympc_cfg.qpos0_js is not None:
    env.mjModel.qpos0 = np.concatenate((env.mjModel.qpos0[:7], qpympc_cfg.qpos0_js))
```

- `qpos0_js`(`js`=joint space)がconfigで指定されていれば、MuJoCoの初期姿勢`qpos0`を作り直す
- `qpos0[:7]`：浮遊ベースの状態(位置3+姿勢クォータニオン4=7要素)
- `qpympc_cfg.qpos0_js`：関節の初期角度
- 概念的には`qpos0 = [ベース位置3, ベース姿勢4, 関節角度12]`という初期姿勢を組み直している

```python
env.reset(random=False)
```

- 環境を初期状態に戻す
- `random=False`なので、最初のエピソードでは初期状態をランダム化しない(後でエピソードが切り替わるとき、318行目付近では`random=True`が使われる)

```python
if render:
    env.render()
    env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
    env.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
```

- 画面表示が有効な場合だけ実行
- `env.render()`：MuJoCo Viewerを生成(この最初の呼び出しで`env.viewer`が作られる)
- 影・反射の描画を無効化：描画負荷を減らすだけで、物理計算や制御には無関係

---

## 78〜93行：関節トルクと制限値、脚順序

```python
tau = LegsAttr(*[np.zeros((env.mjModel.nv, 1)) for _ in range(4)])
```

- 4脚分のトルク変数`tau`をゼロで初期化
- `LegsAttr`は`tau.FL`・`tau.FR`・`tau.RL`・`tau.RR`のように脚名でアクセスできる入れ物(FL=Front Left、FR=Front Right、RL=Rear Left、RR=Rear Rightの略)
- 初期配列のサイズが各脚3要素ではなく`(env.mjModel.nv, 1)`(ロボット全体の一般化速度の数)になっているのはやや不自然
  - 実際には最初の`compute_actions()`呼び出しの中で、各脚3関節分のトルクへ置き換わる
  - 動作上どうしても必要というより、型・shapeの初期化がやや粗い実装

```python
tau_soft_limits_scalar = 0.9
tau_limits = LegsAttr(
    FL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FL] * tau_soft_limits_scalar,
    FR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.FR] * tau_soft_limits_scalar,
    RL=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RL] * tau_soft_limits_scalar,
    RR=env.mjModel.actuator_ctrlrange[env.legs_tau_idx.RR] * tau_soft_limits_scalar,
)
```

- モーターの最大トルクを100%使わず、90%までに制限するための上下限を、脚ごとに作る
- 例：MuJoCoモデル上の制限が`[-40,40] Nm`なら、実際に使う制限は`[-36,36] Nm`
- ここではまだトルクをクリップしていない。あとで制御ループの中で`np.clip()`するときに使う上下限を、あらかじめ準備しているだけ(238行目)

```python
feet_traj_geom_ids, feet_GRF_geom_ids = None, LegsAttr(FL=-1, FR=-1, RL=-1, RR=-1)
legs_order = ["FL", "FR", "RL", "RR"]
```

- `feet_traj_geom_ids` / `feet_GRF_geom_ids`：描画用の図形IDの初期値。`-1`は「まだViewer上に図形が作られていない」の意味。制御計算には使われない
- `legs_order`：コード全体で使う脚の並び順を固定する変数
  - この順序は重要：MPCが返す12個の力(4脚×3成分)も「FLのFx,Fy,Fz → FRのFx,Fy,Fz → RLのFx,Fy,Fz → RRのFx,Fy,Fz」という順番として解釈される

---

## 95〜117行：HeightMapの生成

```python
if qpympc_cfg.simulation_params["visual_foothold_adaptation"] != "blind":
    from gym_quadruped.sensors.heightmap import HeightMap
    resolution_heightmap = 0.04
    num_rows_heightmap = 7
    num_cols_heightmap = 7
```

- 着地点補正のモードが`"blind"`(地形を見ず平坦と仮定して歩くモード)でない場合だけ、足元の高さ情報を作る準備をする
- 既定設定ではこのモードは`"blind"`なので、**この節自体が実行されない**
- 格子の仕様:
  - 格子間隔：`0.04`m(4cm)
  - サイズ：縦7点×横7点(1脚あたり49点)
  - 端から端までの物理サイズ：$(7-1)\times0.04=0.24\ \mathrm{m}$、つまりおよそ24cm四方の局所地形

```python
heightmaps = LegsAttr(
    FL=HeightMap(num_rows=num_rows_heightmap, num_cols=num_cols_heightmap,
                 dist_x=resolution_heightmap, dist_y=resolution_heightmap,
                 mj_model=env.mjModel, mj_data=env.mjData),
    ... # FR, RL, RR も同様
)
```

- 4本の足それぞれに独立した`HeightMap`を作る
- `gym_quadruped/sensors/heightmap.py`を実際に読むと分かる特徴:
  - あらかじめ用意されたグローバルな地図から切り出す方式では**ない**
  - `update_height_map(center, yaw)`が呼ばれるたびに、その中心座標を基準にMuJoCoの`mj_ray`(公式のレイキャストAPI)で49点をその場で再計算する
  - 実センサーとの違い：視野角の制限、オクルージョン(遮蔽物による死角)、ノイズがなく、MuJoCoが知っている真の地形形状に直接アクセスしている
- 各足で別々に持つ理由：足ごとに次の着地点候補の位置が違うため

```python
else:
    heightmaps = None
```

`"blind"`モード(既定)なら地形高さは取得せず、後の`compute_actions()`には`None`が渡される。

---

## 119〜139行：MPC制御器の初期化

```python
quadrupedpympc_observables_names = (
    "ref_base_height",
    "ref_base_angles",
    "ref_feet_pos",
    "nmpc_GRFs",
    "nmpc_footholds",
    "swing_time",
    "phase_signal",
    "lift_off_positions",
)
```

これは**MPCへの入力ではなく**、制御器の内部から記録・描画のために取り出したい値の名前リスト。

| 名前 | 意味 |
|---|---|
| `ref_base_height` | 目標胴体高さ(m) |
| `ref_base_angles` | 目標roll/pitch/yaw角(rad) |
| `ref_feet_pos` | Raibertヒューリスティック等が計算した各足の基準着地点 |
| `nmpc_GRFs` | MPCが計算した各足の地面反力(N) |
| `nmpc_footholds` | 実際に使われる目標着地点(着地点最適化が有効なら`ref_feet_pos`と一致しない場合がある) |
| `swing_time` | 各脚が今の遊脚に入ってからの経過時間(秒) |
| `phase_signal` | 各脚の歩容位相(0〜1の無次元値) |
| `lift_off_positions` | 各脚が地面から離れた瞬間の足位置 |

```python
quadrupedpympc_wrapper = QuadrupedPyMPC_Wrapper(
    initial_feet_pos=env.feet_pos,
    legs_order=tuple(legs_order),
    feet_geom_id=env._feet_geom_id,
    quadrupedpympc_observables_names=quadrupedpympc_observables_names,
)
```

- MPC制御系全体をまとめたラッパーを生成
  - この中で`WBInterface`・`SRBDControllerInterface`(歩容生成器・着地点生成器・遊脚軌道制御器を含む)が初期化される
  - `simulation.py`自身はMPCの数式を直接解くことは一切なく、このラッパーを通して共通の制御ロジックを呼び出すだけ
- `initial_feet_pos=env.feet_pos`は注意が要る箇所
  - これは`env.feet_pos(frame="world")`という**呼び出し結果**ではなく、`env.feet_pos`という**関数そのもの**を渡している
  - ラッパー側は内部で`initial_feet_pos(frame='world')`という形で改めて呼び出す設計
  - 変数名は値のように見えるが、実体は「足先位置を取得する関数」
- `feet_geom_id=env._feet_geom_id`も注意が要る箇所
  - 先頭が`_`のついた、本来`QuadrupedEnv`の内部用属性を外部から直接読み出している
  - 動くには動くが、`gym_quadruped`側の内部実装が変わると壊れやすい書き方

---

## 141〜160行：データ記録の準備

```python
if recording_path is not None:
    from gym_quadruped.utils.data.h5py import H5Writer
    root_path = pathlib.Path(recording_path)
    root_path.mkdir(exist_ok=True)
    dataset_path = (
        root_path
        / f"{robot_name}/{scene_name}"
        / f"lin_vel={ref_base_lin_vel} ang_vel={ref_base_ang_vel} friction={friction_coeff}"
        / f"ep={num_episodes}_steps={int(num_seconds_per_episode // simulation_dt):d}.h5"
    )
    h5py_writer = H5Writer(file_path=dataset_path, env=env, extra_obs=None)
    print(f"\n Recording data to: {dataset_path.absolute()}")
else:
    h5py_writer = None
```

- `recording_path`が指定されたときだけ、シミュレーション結果をHDF5形式で保存する準備をする
- 365行目の既定の呼び出し(`run_simulation(qpympc_cfg=qpympc_cfg)`)では`recording_path`は渡されないため、**このブロックは既定では実行されない**
- 保存先パスの階層構成:
  - `ロボット名/シーン名/`
  - `速度指令と摩擦係数/`
  - `エピソード数_ステップ数.h5`
- `int(num_seconds_per_episode // simulation_dt)`：1エピソードのステップ数。既定値なら$60\div0.002=30000$ステップ(`//`は切り捨て除算)
- `root_path.mkdir(exist_ok=True)`：既にディレクトリがあってもエラーにしないが、`parents=True`が指定されていないため、親ディレクトリまで丸ごと存在しない場合は失敗する

---

## 162〜171行：ループの設定と多エピソードループの開始

```python
RENDER_FREQ = 30
N_EPISODES = num_episodes
N_STEPS_PER_EPISODE = int(num_seconds_per_episode // simulation_dt)
last_render_time = time.time()

state_obs_history, ctrl_state_history = [], []
for episode_num in range(N_EPISODES):
    ep_state_history, ep_ctrl_state_history, ep_time = [], [], []
    for _ in tqdm(range(N_STEPS_PER_EPISODE), desc=f"Ep:{episode_num:d}-steps:", total=N_STEPS_PER_EPISODE):
```

- `RENDER_FREQ=30`：描画の目標フレームレート(Hz)。後の描画ブロック(264行目以降)で「30fps相当の間隔でしか描画しない」という間引きに使う
  - 物理計算自体は`simulation_dt`(既定0.002秒=500Hz)ごとに毎回進むが、画面更新はそれより粗い頻度でよい、という設計
- `N_STEPS_PER_EPISODE`：1エピソードのステップ数(既定30000)
- 2重ループ構造:
  - 外側`for episode_num in range(N_EPISODES)`：エピソードのループ
  - 内側`for _ in tqdm(...)`：1エピソード内の制御ステップのループ(`tqdm`は進捗バー表示のみ)
- `state_obs_history` / `ctrl_state_history`：全エピソード分の履歴を貯める入れ物としてここで初期化されるが、`recording_path`が`None`の既定実行では**最後まで使われない**(323〜324行目で追加されるだけで、参照される場所がない)

---

## 173〜182行：足・胴体の観測取得

```python
feet_pos = env.feet_pos(frame="world")
feet_vel = env.feet_vel(frame='world')
hip_pos = env.hip_positions(frame="world")
base_lin_vel = env.base_lin_vel(frame="world")
base_ang_vel = env.base_ang_vel(frame="base")
base_ori_euler_xyz = env.base_ori_euler_xyz
base_pos = copy.deepcopy(env.base_pos)
com_pos = copy.deepcopy(env.com)
```

ここから内側ループの本体。1制御周期の最初に、MuJoCoの現在状態から必要な観測量をすべて取得する。

- `feet_pos`：ワールド座標系での4本の足先位置
  - `gym_quadruped`側の実装では`mjData.geom_xpos`(足のgeom要素のワールド座標、MuJoCoが順運動学で計算済みの値)をそのまま読んでいるだけで、独自の計算はしていない

`base_lin_vel(frame="world")`と`base_ang_vel(frame="base")`は、一見frameの指定が非対称に見えるが理由がある:

- `gym_quadruped`側の実装:
  - `base_lin_vel(frame='world')`は`mjData.qvel[0:3]`を変換なしでそのまま返す
  - `base_ang_vel(frame='base')`は`mjData.qvel[3:6]`を変換なしでそのまま返す
- 推測：MuJoCoの浮遊ベース関節(free joint)は、並進速度成分`qvel[0:3]`をワールド座標系、回転速度成分`qvel[3:6]`をベース座標系で内部的に保持している(MuJoCo本体の一次資料までは確認できていない)
- このコードはそれぞれ変換コストのかからない、MuJoCoのネイティブ表現をそのまま使っているだけと考えられる
- 「world/baseが混在していておかしい」というより、MuJoCo側の仕様に素直に従った結果と解釈できる

`base_pos`と`com_pos`は似ているが別物:

| 変数 | 中身 |
|---|---|
| `base_pos`(`env.base_pos`) | `mjData.qpos[0:3]`。XML上でベースリンクとして定義されている1点の位置 |
| `com_pos`(`env.com`) | ロボットを構成する**全body**(脚も含む)の質量加重平均位置。`gym_quadruped`側では`nbody`個のボディの`subtree_com`を質量で加重平均する実装 |

- ベースリンクの原点とロボット全体の重心は、たまたま一致することはあっても一般には別の点
- `copy.deepcopy`しているのは、あとで値を書き換えても`env`側の内部状態に影響しないようにするため

---

## 183行：目標速度の取得

```python
ref_base_lin_vel, ref_base_ang_vel = env.target_base_vel()
```

- 現在の目標速度を取得する
- 中身：`self._ref_base_lin_vel_H`(ロボットの向きに対する相対的な速度指令)を、現在のyaw角で回転させてワールド座標系に変換したもの

59行目の疑問に戻る:

- `QuadrupedEnv`生成時に渡した`ref_base_lin_vel=(0.0,4.0)*hip_height`は「生成される範囲」
- この183行目で取れる値は「今この瞬間の目標速度」
- 既定の`base_vel_command_type="human"`では、この値はキーボード操作(`env`側の`_key_callback`)によって更新される
- 速度の範囲を指定する`base_lin_vel_range`という乱数サンプル用の設定は、`"human"`モードでは実質使われない

- この`ref_base_lin_vel`/`ref_base_ang_vel`は、あとで`WBInterface`の中で`ref_state`という辞書に格納され、MPCのコスト関数の目標値(参照値)として使われる
- 重要な点：MPCの制約(「これを超えてはいけない」)ではなく、コスト(「これに近づけたい」)として使われる

---

## 186〜189行：慣性の取得

```python
if qpympc_cfg.simulation_params["use_inertia_recomputation"]:
    inertia = env.get_base_inertia().flatten()
else:
    inertia = qpympc_cfg.inertia.flatten()
```

- `use_inertia_recomputation`が有効：今の関節配置に応じて胴体まわりの慣性を毎回再計算
- 無効：`config.py`に書かれた固定値を使う
- トレードオフの背景：4脚の姿勢(立脚・遊脚、関節角度)によって、胴体から見た「反映された慣性」は変化しうる。毎回計算し直すか、固定近似で済ませるか、という選択

`env.get_base_inertia()`の中身を確認すると:

- MuJoCoの全身質量行列を`mj_fullM`で展開し、その`[3:6, 3:6]`ブロック(ベースの回転自由度に対応する部分)を抜き出している
- docstringには「world frame」と書かれている
- しかし`base_ang_vel(frame='base')`が`qvel[3:6]`を無変換でベース座標系として扱っていたことから考えると、対になる質量行列のこのブロックも本当はベース座標系の値である可能性が高い
- docstringの記載とやや食い違っているように見えるが、確証が持てるところまでは追い切れていない

---

## 191〜196行：関節の状態

```python
qpos, qvel = env.mjData.qpos, env.mjData.qvel
legs_qvel_idx = env.legs_qvel_idx
legs_qpos_idx = env.legs_qpos_idx
joints_pos = LegsAttr(FL=legs_qvel_idx.FL, FR=legs_qvel_idx.FR, RL=legs_qvel_idx.RL, RR=legs_qvel_idx.RR)
```

- `qpos`/`qvel`：MuJoCoの生の関節角度・速度配列そのもの
- `legs_qvel_idx`/`legs_qpos_idx`：脚ごとに「`qvel`/`qpos`の何番目の要素が対応するか」を示すインデックス配列

名前と中身が一致していない箇所:

- `joints_pos`という変数名なら「関節角度の値」を想像するが、実際に代入されているのは`legs_qvel_idx`、つまり**インデックスの配列**
- この`joints_pos`はこのあと`compute_actions`へそのまま渡り、`WBInterface`の中で`state_current['joint_FL'] = joints_pos.FL`という形で状態辞書にそのまま格納される
- つまりMPCに渡される「状態」の`joint_FL`は、関節角度の値ではなくインデックスの配列になっている
- MPCの状態ベクトル自体には関節角度に相当する成分が見当たらないため、この値が実際にOCPの計算で数値として使われているかは疑わしく、使われていない可能性がある(変換処理の中身までは追い切れていない)

---

## 198〜201行：脚の力学量

```python
legs_mass_matrix = env.legs_mass_matrix
legs_qfrc_bias = env.legs_qfrc_bias
legs_qfrc_passive = env.legs_qfrc_passive
```

- `legs_mass_matrix`：全身の質量行列から各脚の自由度に対応する部分だけを切り出したもの
- `legs_qfrc_bias`：遠心力・コリオリ力・重力の合計(MuJoCoの`qfrc_bias`)
- `legs_qfrc_passive`：関節の摩擦などの受動的な力

- これらは後で遊脚のトルク計算(フィードバック線形化や摩擦補償)に使われる
- MPC自体(OCP)はこれらを直接使わず、あくまでトルク変換の段階(ステップ9)で使われる値

---

## 203〜205行：足のヤコビアン

```python
feet_jac = env.feet_jacobians(frame='world', return_rot_jac=False)
feet_jac_dot = env.feet_jacobians_dot(frame='world', return_rot_jac=False)
```

- 各足について、関節速度から足先速度への変換行列(ヤコビアン)を計算する
- `gym_quadruped`側は`mujoco.mj_jac`(MuJoCo公式API)を、脚ごとに4回呼び出している
- 返る行列は`(3, mjModel.nv)`、つまり**全身**の自由度に対するヤコビアン
- 実際にトルク変換で使うときは、この中から該当する脚の3列だけを抜き出して使う(`feet_jac.FL[:, legs_qvel_idx.FL]`のような形)
- 全身分計算してから一部だけ使う、というやや遠回りな作りだが、致命的な問題ではない

---

## 207〜236行：`compute_actions`の呼び出し

```python
tau = quadrupedpympc_wrapper.compute_actions(
    com_pos, base_pos, base_lin_vel, base_ori_euler_xyz, base_ang_vel,
    feet_pos, hip_pos, joints_pos, heightmaps, legs_order, simulation_dt,
    ref_base_lin_vel, ref_base_ang_vel, env.step_num, qpos, qvel,
    feet_jac, feet_jac_dot, feet_vel, legs_qfrc_passive, legs_qfrc_bias,
    legs_mass_matrix, legs_qpos_idx, legs_qvel_idx, tau, inertia,
    env.mjData.contact,
)
```

- ここまでに集めた観測量を、すべて`compute_actions`へまとめて渡す
- この関数の内部で行われる処理の流れ:
  1. 歩容更新
  2. 着地点計算
  3. 状態辞書の組み立て
  4. MPCのOCPを解く
  5. 先頭ステップだけ取り出す
  6. 立脚・遊脚のトルクへ変換
- `simulation.py`自身はこの中身を知らず、「まとめて渡して、トルクを受け取る」という関係だけを持つ
- 最後の引数`env.mjData.contact`：MuJoCoが検出した現在の接触情報。`WBInterface`のどこでこれが使われているかは、このファイルの範囲だけでは特定できていない
- 戻り値の`tau`：4脚分の関節トルク

---

## 238〜247行：トルクのクリップとaction配列の組み立て

```python
for leg in ["FL", "FR", "RL", "RR"]:
    tau_min, tau_max = tau_limits[leg][:, 0], tau_limits[leg][:, 1]
    tau[leg] = np.clip(tau[leg], tau_min, tau_max)
```

- 89行目付近で用意した`tau_limits`(モーター上限の90%)で、各脚のトルクをクリップする
- 注意点：MPCのOCP自体には関節トルクの上下限という制約は入っていない(OCPが持つ制約は接地反力に対する摩擦錐だけ)
- つまりMPCは「実行可能」と判断したGRFを返しているのに、それをトルクへ変換した後にここで問答無用にクリップされる可能性があり、MPCの想定とここでの現実の間にズレが生まれうる構造になっている

```python
action = np.zeros(env.mjModel.nu)
action[env.legs_tau_idx.FL] = tau.FL
action[env.legs_tau_idx.FR] = tau.FR
action[env.legs_tau_idx.RL] = tau.RL
action[env.legs_tau_idx.RR] = tau.RR
```

- `LegsAttr`形式の`tau`を、MuJoCoのアクチュエータ順序に従って1本のベクトル`action`へ詰め替える
- ここでも`["FL","FR","RL","RR"]`という同じ並び順のリテラルが(93行目の`legs_order`とは別に)直接書かれている
- 今回確認した範囲では値は一致しているが、同じ並び順が複数箇所に個別に書かれていること自体は、将来どちらか一方だけ変更されるとズレる、という潜在的なリスク

---

## 250〜251行：MuJoCoを1歩進める

```python
state, reward, is_terminated, is_truncated, info = env.step(action=action)
```

`env.step()`の中身:

- `self.mjData.ctrl = action`でMuJoCoの制御入力へ代入
- `mujoco.mj_step(self.mjModel, self.mjData)`で物理を1ステップ進める
- 実機であれば、この行に相当するのは「計算したトルク指令を実際のモータードライバへ送信する」という処理(`ros2/run_controller.py`側では`/control_signal`トピックのpublishがこれに対応)
- `step_num`はこの中で1つ増える
- `base_vel_command_type`に`'reset'`という文字列が含まれる場合は、一定ステップごとに目標速度を再サンプルする処理も入っているが、既定の`"human"`ではこの分岐は使われない

---

## 253〜262行：観測の記録

```python
ctrl_state = quadrupedpympc_wrapper.get_obs()
base_poz_z_err = ctrl_state["ref_base_height"] - base_pos[2]
ctrl_state["base_poz_z_err"] = base_poz_z_err

ep_state_history.append(state)
ep_time.append(env.simulation_time)
ep_ctrl_state_history.append(ctrl_state)
```

- 119行目で指定した`quadrupedpympc_observables_names`に対応する値を取り出す
- 目標高さと実際の高さの差(`base_poz_z_err`)を追加で計算
- このエピソードの履歴に積み立てる
- この履歴は、エピソード終了時(318行目以降)に全体の履歴へまとめて追加されるが、`recording_path`が`None`の既定実行では、その全体の履歴自体が結局どこにも使われない

---

## 264〜316行：描画ブロック

```python
if render and (time.time() - last_render_time > 1.0 / RENDER_FREQ or env.step_num == 1):
```

- `RENDER_FREQ=30`で決めた間隔(約33ミリ秒に1回)、または最初のステップでだけ、この中の描画処理を実行する
- 中で行っていること:
  - 遊脚の軌道の描画(`plot_swing_mujoco`)
  - HeightMapの球体表示(`visual_foothold_adaptation`が`blind`でないときだけ)
  - 各脚の地面反力の矢印描画

```python
_, _, feet_GRF = env.feet_contact_state(ground_reaction_forces=True)
...
feet_GRF_geom_ids[leg_name] = render_vector(
    env.viewer, vector=feet_GRF[leg_name], pos=feet_pos[leg_name], ...
)
```

- 注意点：この`feet_GRF`は**MPCが計算した目標値(`nmpc_GRFs`)ではない**
- `env.feet_contact_state(...)`が返す、MuJoCoの接触ソルバーが実際に計算した接触力
- 見た目には同じ「GRF」の矢印だが、由来が違う。理想的には近い値になるはずだが、一致する保証はない
- この描画ブロック全体は物理計算やMPCの計算結果に一切フィードバックしない

---

## 318〜327行：エピソード終了処理

```python
if env.step_num >= N_STEPS_PER_EPISODE or is_terminated or is_truncated:
    if is_terminated:
        print("Environment terminated")
    else:
        state_obs_history.append(ep_state_history)
        ctrl_state_history.append(ep_ctrl_state_history)

    env.reset(random=True)
    quadrupedpympc_wrapper.reset(initial_feet_pos=env.feet_pos(frame="world"))
```

- 規定ステップ数に達するか、転倒などで異常終了すると、このブロックに入る
- 分岐の中身:
  - `is_terminated`(転倒などの異常終了)のとき：`print`するだけで、このエピソードの履歴(`ep_state_history`等)は全体の履歴に**追加されない**
  - それ以外(規定ステップ数到達)のとき：履歴を全体の履歴へ追加する
- つまり失敗したエピソードのデータは記録上捨てられる設計になっている
  - 意図的な設計(失敗データをデータセットに混ぜたくない)である可能性が高い
  - ただし「なぜ失敗したか」を後から分析したい場合には不便
- `env.reset(random=True)`：環境をランダムに初期化し直す
- `quadrupedpympc_wrapper.reset(...)`：歩容の位相や着地点の履歴といった制御側の内部状態もリセットする

---

## 329〜336行：記録の保存と終了

```python
if h5py_writer is not None:
    ep_obs_history = collate_obs(ep_state_history)
    ep_traj_time = np.asarray(ep_time)[:, np.newaxis]
    h5py_writer.append_trajectory(state_obs_traj=ep_obs_history, time=ep_traj_time)

env.close()
if h5py_writer is not None:
    return h5py_writer.file_path
```

- `recording_path`が指定されているときだけ、このエピソードの履歴をHDF5へ追記する
- 既定実行では`h5py_writer`が`None`なので、このブロックはほぼスキップされる
- 最後に`env.close()`だけが呼ばれて関数を抜ける(戻り値なし)

---

## 339〜352行：`collate_obs`関数

```python
def collate_obs(list_of_dicts) -> dict[str, np.ndarray]:
    if not list_of_dicts:
        raise ValueError("Input list is empty.")
    keys = list_of_dicts[0].keys()
    collated = {key: np.stack([d[key] for d in list_of_dicts], axis=0) for key in keys}
    collated = {key: v[:, None] if v.ndim == 1 else v for key, v in collated.items()}
    return collated
```

- 「1ステップ=1辞書」という形式のリストを、キーごとにスタックした配列の辞書に変換するユーティリティ
- 1次元配列は`(N,)`から`(N,1)`へ整形される
- これもH5Writer用の前処理であり、MPCの計算そのものには関係しない

---

## 355〜365行：`__main__`ブロック

```python
if __name__ == "__main__":
    from quadruped_pympc import config as cfg
    qpympc_cfg = cfg
    pass
    run_simulation(qpympc_cfg=qpympc_cfg)
```

- このファイルを直接実行したときの入口
- `quadruped_pympc.config`モジュールをそのまま`qpympc_cfg`として渡し、他はすべてデフォルト値で`run_simulation`を呼ぶ
- `pass`の行：実行前にここへ`config.py`のパラメータ上書きコードを書き込むための空のプレースホルダと考えられる(コメント「Custom changes to the config here」がその上にある)

---

以上で`simulation.py`の先頭から末尾までを一通り読み終えました。続きとして`WBInterface`や`SRBDControllerInterface`の内部を、同じ粒度で読み進めることもできます。
