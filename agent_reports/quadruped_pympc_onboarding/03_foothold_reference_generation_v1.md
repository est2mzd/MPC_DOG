# 03 — 接地状態から着地点参照（Raibert則）まで

日付: 2026-08-25
対象: `external/Quadruped-PyMPC`
関連: [01_execution_order_trace_v2.md](01_execution_order_trace_v2.md) の B1-1 節（表内の順番7, 8, 9）、
[02_gait_and_contact_sequence_v2.md](02_gait_and_contact_sequence_v2.md)（`current_contact`/`previous_contact`の生成元）

対象ファイル:
- `quadruped_pympc/interfaces/wb_interface.py`
- `quadruped_pympc/helpers/foothold_reference_generator.py`
- `quadruped_pympc/config.py`（関連設定値）

スコープ外（本ファイルでは扱わない）: 地形推定（`TerrainEstimator`）、
Visual Foothold Adaptation（`VisualFootholdAdaptation`）、NMPC内部（acados/OCPの数式・実装）、
遊脚軌道（`SwingTrajectoryController`の軌道生成）、トルク計算。

本ファイルの記述は次の3種類に分けて明記する。

- **事実**: 読んだコードにそのまま書かれている内容
- **解釈**: 事実から導かれる、コード上は明示されていない理論的な意味づけ（推測を含む場合は明記）
- **不明**: コードだけでは確認できない事項

---

## 0. 全体データフロー（結論）

```text
previous_contact, current_contact
        │
        ├─→ FootholdReferenceGenerator.update_lift_off_positions()   → lift_off_positions
        └─→ FootholdReferenceGenerator.update_touch_down_positions() → touch_down_positions
        │
current base_position, base_ori_euler_xyz, base_lin_vel, ref_base_lin_vel, hips_position, ref_z
        │
        ▼
FootholdReferenceGenerator.compute_footholds_reference()
        │  (Raibert則。z成分だけは lift_off_positions から流用)
        ▼
ref_feet_pos (world frame, LegsAttr)
        │
        ▼ (wb_interface.py, ref_state辞書へ格納)
ref_state['ref_foot_FL'] 等 (shape (1,3))
        │
        ▼ (quadruped_pympc_wrapper.py::compute_actions, mpc_frequencyでゲート)
SRBDControllerInterface.compute_control(..., ref_state, ...)
        │
        ▼ (NMPC内部・yrefの一部として使用。詳細は01のB1-2節、本ファイルの対象外)
Acados_NMPC_Nominal → 着地点を状態として最適化 → nmpc_footholds
```

**事実**: `ref_feet_pos`（Raibert参照）と`nmpc_footholds`（NMPC最適化結果）は別変数であり、
前者はOCPの参照（コスト関数側）、後者はOCPが実際に返す解（状態変数側）である
（詳細は8節）。

---

## 1. `previous_contact` と `current_contact` から離地・着地を判定する条件

`wb_interface.py::update_state_and_reference()` L202–210:

```python
self.pgg.run(simulation_dt, self.pgg.step_freq)
contact_sequence = self.pgg.compute_contact_sequence(
    contact_sequence_dts=self.contact_sequence_dts, contact_sequence_lenghts=self.contact_sequence_lenghts
)

self.previous_contact = copy.deepcopy(self.current_contact)
self.current_contact = np.array(
    [contact_sequence[0][0], contact_sequence[1][0], contact_sequence[2][0], contact_sequence[3][0]]
)
```

**事実**: この関数は毎シミュレーションステップ呼ばれる（`02`参照）。したがって
`previous_contact`は「1シミュレーションステップ前の`current_contact`」、
`current_contact`は「今ステップの接地状態」であり、両者は`simulation_dt`間隔で比較される。

`foothold_reference_generator.py::update_lift_off_positions()` L166–178 /
`update_touch_down_positions()` L187–199 が、脚ごとに次の4通りの遷移を判定する:

| `previous_contact[i]` | `current_contact[i]` | 意味 | `update_lift_off_positions`の分岐 | `update_touch_down_positions`の分岐 |
|---|---|---|---|---|
| 1 | 0 | **離地イベント**（stance→swing） | L172–174: 実行 | 該当分岐なし（何もしない） |
| 0 | 0 | 遊脚継続（swing中） | L176–178: 実行 | 該当分岐なし（何もしない） |
| 0 | 1 | **着地イベント**（swing→stance） | 該当分岐なし（何もしない） | L193–195: 実行 |
| 1 | 1 | 接地継続（stance中） | 該当分岐なし（何もしない） | L197–199: 実行 |

**事実**: `gait_type == GaitType.FULL_STANCE.value` のときは、上記の遷移判定をせず、
毎回無条件に`lift_off_positions[leg] = feet_pos[leg]`（L167–169）、
`touch_down_positions[leg] = feet_pos[leg]`（L188–190）で現在の実測足位置に上書きする
（`continue`で以降の分岐をスキップ）。

---

## 2. `lift_off_positions` と `touch_down_positions` の更新・保持タイミング

### 2.1 離地時（`lift_off_positions`, L172–174）

```python
if previous_contact[leg_id] == 1 and current_contact[leg_id] == 0:
    self.lift_off_positions[leg_name] = feet_pos[leg_name]
    self.lift_off_positions_h[leg_name] = R_W2H @ (self.lift_off_positions[leg_name] - base_position)
```

**事実**: 離地の瞬間、実測の足先ワールド座標`feet_pos[leg_name]`をそのまま
`lift_off_positions`へ記録し、同時にその位置を「ベース位置からの相対ベクトルを
horizontal frameで回転させたもの」として`lift_off_positions_h`にキャッシュする
（座標変換は6節）。

### 2.2 遊脚継続時（`lift_off_positions`, L176–178）

```python
elif previous_contact[leg_id] == 0 and current_contact[leg_id] == 0:
    self.lift_off_positions[leg_name] = R_W2H.T @ self.lift_off_positions_h[leg_name] + base_position
```

**事実**: 遊脚中は毎ステップ、離地時にキャッシュした`lift_off_positions_h`（horizontal frame
でのオフセット）を、**その時点の**`base_position`・`yaw`で再度ワールド座標へ逆変換して
`lift_off_positions`を上書きする。

**解釈（推測を含む）**: `R_W2H`のz成分は恒等（3節参照）なので、Z座標については
「離地時の（足高さ − ベース高さ）」という差分が遊脚中一定に保たれ、
`lift_off_positions[leg].z = 定数 + 現在のbase_position.z`という式になる。
X,Yについても同様に、離地時の「ベースから見た相対水平位置」を保ったまま、
ベースの並進・yawの変化分だけ引きずられる。つまり`lift_off_positions`は
遊脚中、実測の足位置を再取得しているのではなく、離地時点の相対位置を
ベースに追従させた**推定値**である。これが実装された理由（例えば足位置センサの
値を毎回読みたくないため、あるいは`compute_footholds_reference()`のZ成分
（4節）用に「ベース相対高さ」を安定して保持するためなど）はコードのコメントからは
読み取れず、**推測**にとどまる。

### 2.3 着地時（`touch_down_positions`, L193–195）／接地継続時（L197–199）

構造は2.1・2.2と対称で、遷移条件が「0→1」「1→1」に変わるだけである
（コードは前節参照）。

**事実（`touch_down_positions`の利用先について）**: リポジトリ全体を`grep`した結果、
`self.touch_down_positions`（および`_h`キャッシュ）は`foothold_reference_generator.py`
内部の読み書き以外に、**外部から読み出している箇所が見つからなかった**
（`wb_interface.py` L407に`# des_foot_pos[leg_name] = self.frg.touch_down_positions[leg_name]`
というコメントアウトされた行が1つあるのみ）。一方、`self.esd.update_detection(...)`
（L365）の`touch_down`引数には`self.frg.touch_down_positions`ではなく`nmpc_footholds`
（NMPC側の出力）が渡されている。したがって`touch_down_positions`は現行コードパスでは
**計算されるが読まれない状態**であるとコード上確認できる。

### 2.4 `lift_off_positions` の利用先（事実）

- `wb_interface.py` L156: `terrain_computation.compute_terrain_estimation(feet_pos=self.frg.lift_off_positions, ...)`（地形推定、スコープ外）
- `wb_interface.py` L365: `esd.update_detection(lift_off=self.frg.lift_off_positions, ...)`（早期接地検知、スコープ外）
- `wb_interface.py` L394: `stc.compute_swing_control_cartesian_space(lift_off=self.frg.lift_off_positions[leg_name], ...)`（遊脚軌道、スコープ外）
- `foothold_reference_generator.py` L151: `compute_footholds_reference()`内でZ成分にのみ使用（4節）
- `quadruped_pympc_wrapper.py` L237: 観測値として`get_obs()`経由で外部公開（ロギング用）

---

## 3. `compute_footholds_reference()` の入力・出力

`foothold_reference_generator.py` L53–61（シグネチャそのまま）:

```python
def compute_footholds_reference(
    self,
    base_position: np.ndarray,        # (3,)  ワールド座標 [m]
    base_ori_euler_xyz: np.ndarray,    # (3,)  ワールド座標系でのroll,pitch,yaw [rad]
    base_xy_lin_vel: np.ndarray,       # (2,)  ワールド座標系でのベース並進速度 x,y [m/s]
    ref_base_xy_lin_vel: np.ndarray,   # (2,)  ワールド座標系での目標並進速度 x,y [m/s]
    hips_position: LegsAttr,           # 脚ごと(3,) ワールド座標 [m]（xyのみ使用）
    com_height_nominal: np.float32,    # スカラー [m]
) -> LegsAttr:
```

**事実**: 引数`base_xy_lin_vel`・`ref_base_xy_lin_vel`はL83–85のassertで
`shape == (2,)`が強制されている。戻り値`ref_feet`（=呼び出し側での`ref_feet_pos`）は
`LegsAttr`で、各脚`(3,)`のワールド座標（4節で導出）。

**事実（呼び出し元）**: `wb_interface.py` L231–238:

```python
ref_feet_pos = self.frg.compute_footholds_reference(
    base_position=base_pos,
    base_ori_euler_xyz=base_ori_euler_xyz,
    base_xy_lin_vel=base_lin_vel[0:2],
    ref_base_xy_lin_vel=ref_base_lin_vel[0:2],
    hips_position=hip_pos,
    com_height_nominal=cfg.simulation_params['ref_z'],
)
```

`com_height_nominal`には`config.py`の`simulation_params['ref_z']`
（`= hip_height * 1.08`、`hip_height`はロボットごとに`gym_quadruped`から取得）が渡される。
「現在の実測CoM高さ」ではなく「目標の基準高さ」が使われている点に注意。

---

## 4. 実装されているRaibert着地点計算式（コードからの復元）

`compute_footholds_reference()` L87–151 を、行の出現順に数式化する。

### 4.1 world → horizontal frame の速度変換（L88–94）

```python
yaw = base_ori_euler_xyz[2]
R_W2H = np.array([np.cos(yaw), np.sin(yaw), -np.sin(yaw), np.cos(yaw)]).reshape((2, 2))
base_lin_vel_H = R_W2H @ base_xy_lin_vel
ref_base_lin_vel_H = R_W2H @ ref_base_xy_lin_vel
```

$$
R_{W\to H}=\begin{bmatrix}\cos\psi & \sin\psi\\-\sin\psi & \cos\psi\end{bmatrix},\qquad
v_H = R_{W\to H}\,v_{xy}^{W},\qquad v_H^{ref} = R_{W\to H}\,v_{xy}^{ref,W}
$$

($\psi$ = `yaw`)

### 4.2 移動平均とRaibertの速度補正項（L96–111）

```python
self.base_vel_hist.append(base_lin_vel_H)          # deque(maxlen=20)
base_vel_mvg = np.mean(list(self.base_vel_hist), axis=0)

delta_ref_H = (self.stance_time / 2.0) * ref_base_lin_vel_H
delta_ref_H = np.clip(delta_ref_H, -self.hip_height * 1.5, self.hip_height * 1.5)
vel_offset = np.concatenate((delta_ref_H, np.zeros(1)))

error_compensation = np.sqrt(com_height_nominal / self.gravity_constant) * (base_vel_mvg - ref_base_lin_vel_H)
error_compensation = np.clip(error_compensation, -0.05, 0.05)   # 実装はnp.whereを2回だが結果はclipと同一
error_compensation = np.concatenate((error_compensation, np.zeros(1)))
```

$$
\bar v_H = \text{mean}(\text{直近最大20サンプルの } v_H)
$$

$$
\Delta_{ref}=\text{clip}\!\Big(\frac{T_{stance}}{2}\,v_H^{ref},\ \pm 1.5\,h_{hip}\Big)
\qquad\text{(標準Raibert項 } \tfrac{T_{stance}}{2}v^{ref}\text{ に相当)}
$$

$$
e = \text{clip}\!\Big(\sqrt{\tfrac{h_{com}}{g}}\,(\bar v_H - v_H^{ref}),\ \pm 0.05\ \text{m}\Big)
\qquad\text{(倒立振子ゲイン } \sqrt{h/g}\text{ による速度誤差フィードバック)}
$$

**事実**: `error_compensation`の符号は `(実測移動平均速度 − 目標速度)` であり、
教科書的なRaibert則でよく見る `(目標 − 実測)` とは**符号が逆**である。
これはコードにそのまま書かれている事実であり、一般的なRaibert則の式を
そのまま転記すると符号を誤る。

### 4.3 hip位置基準の水平面参照（L114–129）

```python
ref_feet.FL[0:2] = R_W2H @ (hips_position.FL[0:2] - base_position[0:2])
...
ref_feet.FL[1] += self.hip_offset   # FL, RL: +0.1
ref_feet.FR[1] -= self.hip_offset   # FR, RR: -0.1
```

$$
p_{H,i} = R_{W\to H}\,(p_{hip,i}^{xy} - p_{base}^{xy}) + o_i,\qquad
o_i = \begin{cases}(0,\ +h_{off})^T & i\in\{FL,RL\}\\(0,\ -h_{off})^T & i\in\{FR,RR\}\end{cases}
$$

（`self.hip_offset = 0.1`（L44、config値ではなくクラス内ハードコード。コード中のTODOコメントで
「configから渡すべき」と明記されている））

### 4.4 速度補正の加算とworld frameへの逆変換（L131–138）

```python
ref_feet += vel_offset + error_compensation
ref_feet.FL[0:2] = R_W2H.T @ ref_feet.FL[:2] + base_position[0:2]
...
```

$$
p_{H,i} \leftarrow p_{H,i} + \Delta_{ref} + e
\qquad\qquad
p_{W,i}^{xy} = R_{W\to H}^{T}\,p_{H,i} + p_{base}^{xy}
$$

$R_{W\to H}$は直交行列（回転行列）なので$R_{W\to H}^{T}=R_{W\to H}^{-1}=R(\psi)$である。

**解釈**: `vel_offset`・`error_compensation`は全脚共通の値であり、`R_{W\to H}^T R_{W\to H} = I`
なので、**クリップが作動しない範囲では**この往復変換は打ち消し合い、world frame上の式は

$$
p_{W,i}^{xy} \approx p_{hip,i}^{xy} + R_{W\to H}^{T} o_i + \frac{T_{stance}}{2} v_{xy}^{ref,W} + \sqrt{\frac{h_{com}}{g}}\,(\bar v^{W} - v_{xy}^{ref,W})
$$

という、hip位置起点の標準的なRaibert則に一致する（$R_{W\to H}^T o_i$はyawだけ回転した
左右オフセット）。**ただし**`delta_ref_H`・`error_compensation`のクリップ処理は
`R_W2H`で回転した**horizontal frame上の各軸ごと**に行われるため、クリップが作動している
場合はworld frame側で単純な標準Raibert式には一致しない（軸ごとのクリップはyawに依存する）。
これは4.2・4.4の式から導かれる**解釈**であり、コードにこの等価性が明示されているわけではない。

### 4.5 CoMオフセットの加算（L140–146、スコープ外の補足）

```python
R_B2W = Rotation.from_euler("xyz", base_ori_euler_xyz).as_matrix()   # roll,pitch,yawすべて使う真のbody→world回転
self.com_pos_offset_w = R_B2W @ self.com_pos_offset_b
ref_feet.FL[0:2] += self.com_pos_offset_w[0:2]
```

**事実**: ここで使われる`R_B2W`は、`R_W2H`（yawのみ）と異なり roll・pitch・yaw を
含む真のbody→world回転行列である（6節）。`com_pos_offset_b`はコード上
`np.zeros((3,))`で初期化されており（L32）、本ファイルで読んだ範囲では
これを非ゼロに設定する箇所は見つかっていない（**不明**: 外部から設定される経路が
別にある可能性は排除できない）。

### 4.6 Z成分の決定（L150–151）

```python
for leg_id in ['FL', 'FR', 'RL', 'RR']:
    ref_feet[leg_id][2] = self.lift_off_positions[leg_id][2]
```

**事実**: 着地参照のZ座標は、4.1–4.5のRaibert計算（X,Yのみ）とは無関係に、
**`lift_off_positions`のZ成分をそのままコピー**している。Raibert則の式では
Z座標は導出されない。

---

## 5. 現在速度・目標速度・stance time・yaw rate・hip位置が着地点へ与える影響

4節の式に基づき整理する。

| 入力 | 影響する項 | 影響の向き（事実／解釈） |
|---|---|---|
| `ref_base_xy_lin_vel`（目標並進速度） | `delta_ref_H`（4.2）に直接比例 | 目標速度が大きいほど、着地点を進行方向へ`stance_time/2`倍だけ前方（または後方）にずらす（**事実**：コードの式そのもの） |
| `base_xy_lin_vel`（現在速度、移動平均経由） | `error_compensation`（4.2）に寄与 | 移動平均速度が目標より速い場合、着地点を減速方向へ補正する（**解釈**：`√(h/g)`ゲインを介した速度誤差フィードバックとして機能） |
| `stance_time`（`= (1/step_freq)*duty_factor`、`wb_interface.py` L66で計算されコンストラクタへ渡される） | `delta_ref_H`の係数 | stance時間が長いほど、目標速度1単位あたりの着地点オフセットが大きくなる（**事実**：式に`stance_time/2`が直接掛かる） |
| **yaw rate（`ref_base_ang_vel`）** | **どの項にも現れない** | **事実**: `compute_footholds_reference()`のシグネチャに`ref_base_ang_vel`（角速度）は存在せず、関数内でも参照されていない。ヨーレートによる着地点補正はこの関数には実装されていない（`foothold_reference_generator.py` L64–67のTODOコメントに「yaw_dotによる速度誤差補正を将来追加すべき」と明記されており、**未実装であることがコード上のコメントからも確認できる**） |
| `hips_position`（hip位置） | 4.3の基準点そのもの | 各脚の参照着地点は、常に対応するhipのXY位置を基準（原点）として計算される（**事実**） |
| `yaw`（ヨー角そのもの、角速度ではない） | `R_W2H`全体 | 現在のヨー角だけ回転した座標系で速度補正・hipオフセットを計算してからworld frameへ戻す（**事実**） |

---

## 6. world / body / horizontal frame 間の座標変換

**事実**: `foothold_reference_generator.py`内には、実質2種類の回転行列しか登場しない。

| 変数 | 定義 | 使用する角度 | 呼称（コード上の変数名/コメントより） |
|---|---|---|---|
| `R_W2H`（2×2、L89） | $\begin{bmatrix}\cos\psi&\sin\psi\\-\sin\psi&\cos\psi\end{bmatrix}$ | yawのみ（$\psi$） | "world to horizontal frame (hip-centric)"（L87のコメント） |
| `R_W2H`（3×3、L163, L184） | 上記2×2にz軸の恒等変換を追加 | yawのみ | 同上（`update_lift_off_positions`/`update_touch_down_positions`内） |
| `R_B2W`（3×3、L141） | `Rotation.from_euler("xyz", base_ori_euler_xyz)` | roll・pitch・yawすべて | コメントなし。変数名から本ファイルでは"body to world"と呼ぶ |

**解釈**: コード中の「horizontal frame」は、ロボットの重心/ヒップまわりで**yaw角だけ**
world frameを回転させた座標系であり、roll・pitchは含まない
（コメント"hip-centric"が示す通り、水平面内の向きだけを揃えるための frame）。
一方「body frame」（真の胴体固定座標系）は`R_B2W`が対応するroll・pitch・yawをすべて
含む回転で表現されるが、本ファイルで`R_B2W`が使われるのは4.5節の
CoMオフセット変換のみであり、Raibert着地点計算そのもの（4.1–4.4節）は
**horizontal frameだけ**で行われている。両者は名前が似ているが**別の回転行列**であり、
混同しないことが重要である。

**事実（3×3版の性質）**: `update_lift_off_positions`/`update_touch_down_positions`で使う
3×3の`R_W2H`はz軸に対して恒等（`R_W2H[2,2]=1`、他のz行・z列は0）なので、
Z成分の変換は単純な平行移動（`- base_position`または`+ base_position`）だけになる
（2節で述べたZ座標の挙動はここから導かれる）。

---

## 7. `ref_feet_pos` が `ref_state` へ入り、NMPCへ渡されるまで

`wb_interface.py::update_state_and_reference()` の該当箇所を呼び出し順に示す。

1. L231–238: `ref_feet_pos = self.frg.compute_footholds_reference(...)`（3・4節の計算結果）
2. L241–256: VFA有効時は`ref_feet_pos`がここで上書きされる可能性がある
   （**事実として存在するが、本ファイルのスコープ外につき詳細は扱わない**）。
   VFA無効（既定の`'blind'`）の場合、`ref_feet_pos`はそのまま次工程へ渡る。
3. L278–294（`mpc_params['type'] != 'kinodynamic'`のとき）:
   ```python
   ref_state = {}
   ref_state |= dict(
       ref_foot_FL=ref_feet_pos.FL.reshape((1, 3)),
       ref_foot_FR=ref_feet_pos.FR.reshape((1, 3)),
       ref_foot_RL=ref_feet_pos.RL.reshape((1, 3)),
       ref_foot_RR=ref_feet_pos.RR.reshape((1, 3)),
       ...
   )
   ```
   **事実**: `(3,)`だった`ref_feet_pos.FL`等が`(1,3)`にreshapeされて`ref_state`へ格納される。
4. L305: `return state_current, ref_state, contact_sequence, self.step_height, optimize_swing`
5. `quadruped_pympc_wrapper.py::compute_actions()` L114–131でこの`ref_state`を受け取る。
6. L134の`mpc_frequency`ゲート内（L143–151）で
   `self.srbd_controller_interface.compute_control(state_current, ref_state, contact_sequence, inertia, ...)`
   に渡される。
7. `SRBDControllerInterface.compute_control()`（nominal型）は
   `self.controller.compute_control(state_current, ref_state, contact_sequence, inertia=inertia, ...)`
   （`srbd_controller_interface.py` L210）へそのまま委譲する。
8. `Acados_NMPC_Nominal.compute_control()`内で`reference["ref_foot_FL"][idx_ref_foot_to_assign[0]]`
   等として読み出され、acadosの`yref`（各ステージの参照値）の一部に組み込まれる
   （`centroidal_nmpc_nominal.py` L1171、`01_execution_order_trace_v2.md`のB1-2節「2-b」で既述。
   これ以降のNMPC内部処理は本ファイルのスコープ外）。

---

## 8. Raibert着地点参照とNMPCが最適化した着地点の違い

**事実（変数としての違い）**:

| | `ref_feet_pos` / `ref_state['ref_foot_*']` | `nmpc_footholds` |
|---|---|---|
| 生成元 | `FootholdReferenceGenerator.compute_footholds_reference()`（幾何ヒューリスティック、Raibert則） | `Acados_NMPC_Nominal.compute_control()`内のOCP求解（`01_execution_order_trace_v2.md`のB1-2節「2-h」） |
| OCP内での役割 | コスト関数の**参照値**（`yref`の一部） | OCPの**決定変数**（状態`x`の一部）の求解結果 |
| 生成頻度 | 毎シミュレーションステップ（`update_state_and_reference`が毎回呼ばれるため） | `mpc_frequency`でゲートされた頻度のみ更新 |

**事実（`use_foothold_optimization`設定、`config.py`）**:
```python
# if this is off, the mpc will not optimize the footholds and will
# use only the ones provided in the reference
'use_foothold_optimization':               True,
```

**解釈**: このコメントから、`use_foothold_optimization=True`（既定）の場合、NMPCは
Raibert参照を**出発点/コスト目標**としつつ、接地・摩擦・安定性などの制約の範囲内で
実際の着地点を最適化した結果として`nmpc_footholds`を返す。一方`False`の場合は
「参照で与えたものをそのまま使う」とコメントされており、その場合は
`nmpc_footholds`が`ref_feet_pos`（Raibert参照）と一致する設計であると読み取れる
（**解釈**：このコメントの記述に基づく推測であり、`False`時の実装分岐の詳細な
コードパスそのものは本ファイルでは追っていない）。

**不明**: `use_foothold_optimization=False`時に実際に`nmpc_footholds`が
`ref_feet_pos`と数値的に完全一致するかどうかは、NMPC内部（`centroidal_nmpc_nominal.py`の
制約定式化、スコープ外）を読まないと確認できない。

---

## 9. まとめ表（主要変数・shape・座標系・単位）

| 変数 | shape | 座標系 | 単位 | 定義箇所 |
|---|---|---|---|---|
| `previous_contact` / `current_contact` | `(4,)` | — | `{0,1}` | `wb_interface.py` L207–210 |
| `feet_pos` | 脚ごと`(3,)` | world | m | 呼び出し元（`simulation.py`）から渡される実測値 |
| `lift_off_positions` | 脚ごと`(3,)` | world（内部で`_h`にhorizontal frameキャッシュを保持） | m | `foothold_reference_generator.py` L28, L168/173/178 |
| `touch_down_positions` | 脚ごと`(3,)` | world（同上） | m | `foothold_reference_generator.py` L29, L189/194/199（現状コード上の消費者なし、2.3節） |
| `base_position` | `(3,)` | world | m | `compute_footholds_reference`引数 |
| `base_ori_euler_xyz` | `(3,)` | world基準のroll,pitch,yaw | rad | 同上 |
| `base_xy_lin_vel` / `ref_base_xy_lin_vel` | `(2,)` | world | m/s | 同上（assertでshape強制） |
| `hips_position` | 脚ごと`(3,)` | world（xyのみ使用） | m | 同上 |
| `com_height_nominal` | スカラー | — | m | `config.py::simulation_params['ref_z']` |
| `stance_time` | スカラー | — | s | `wb_interface.py` L66（`(1/step_freq)*duty_factor`） |
| `hip_offset` | スカラー | horizontal frame, y軸 | m | `foothold_reference_generator.py` L44（ハードコード`0.1`） |
| `ref_feet_pos`（戻り値） | 脚ごと`(3,)` | world | m | `compute_footholds_reference`戻り値 |
| `ref_state['ref_foot_*']` | `(1,3)` | world | m | `wb_interface.py` L281–284 |

---

## 10. 未確認事項（まとめ）

- `com_pos_offset_b`が非ゼロに設定される経路（本ファイルで読んだ範囲では見つからず）
- `use_foothold_optimization=False`時に`nmpc_footholds`が`ref_feet_pos`と厳密に一致するかどうか（NMPC内部の確認が必要、スコープ外）
- yawレート（角速度）による着地点補正が将来追加される場合の設計意図（TODOコメントに記載があるのみで未実装）
- VFA（`visual_foothold_adaptation`）有効時に`ref_feet_pos`がどう上書きされるか（スコープ外）
- `lift_off_positions`を遊脚中に「ベース追従」させる設計上の理由（2.2節、コード上明記なし）
