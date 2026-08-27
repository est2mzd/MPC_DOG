# 早期接地検知 helpers/early_stance_detector.py 逐次解説

## simulation.py との結びつき(呼び出し連鎖)

```text
simulation.py (run_simulationのループ)
  → quadrupedpympc_wrapper.compute_actions(...)
      → self.wb_interface.compute_stance_and_swing_torque(...)  (read_code_12)
          → self.esd.update_detection(...)   ← 本ファイル、毎周期
              → (結果を) self.stc.compute_swing_control_cartesian_space(...) (read_code_13)へ渡す
```

`self.esd`は`WBInterface.__init__`(read_code_06)の中で`EarlyStanceDetector(feet_geom_id)`
として生成される。`update_detection`は毎周期(既定500Hz相当)呼ばれる。

**本ファイルは、これまで読んだ多くの機能と違い、既定で有効な機能です**(`reflex_trigger_mode`が
`config.py`で既定`'tracking'`、`False`ではない)。read_code_10・11で見た「既定OFF」の
パターンとは対照的に、ここは通常のトロット歩行でも毎周期動いている処理です。

## このクラスの役割(全体の中での位置づけ)

`EarlyStanceDetector`が担当するのは、「**遊脚中の足が、予定より早く地面(または障害物)に
接触したことを検知する**」ことです。段差や凸凹地形で、計画した着地予定時刻より前に足が
何かに当たった場合、それを検知して`read_code_13`のスイング制御(を経由して軌道生成器)へ
伝える、簡易的な反射(リフレックス)機構です。

対象は`external/Quadruped-PyMPC/quadruped_pympc/helpers/early_stance_detector.py`
(147行)です。

---

## 1〜33行:`__init__`

この関数の役割:検知モードを`config.py`から読み込み、検知用のしきい値と内部状態を初期化する。

```python
def __init__(self, feet_geom_id=None, legs_order=('FL', 'FR', 'RL', 'RR')):
    self.legs_order = legs_order
    self.feet_geom_id = feet_geom_id
    self.early_stance = LegsAttr(FL=False, FR=False, RR=False, RL=False)
    self.hitmoments = LegsAttr(FL=-1.0, FR=-1.0, RR=-1.0, RL=-1.0)
    self.hitpoints = LegsAttr(FL=None, FR=None, RR=None, RL=None)

    if(cfg.mpc_params['type'] == 'sampling'):
        self.activated = False # TO FIX

    self.trigger_mode = cfg.simulation_params['reflex_trigger_mode']
    if(self.trigger_mode == False):
        self.activated = False
    else:
        self.activated = True

    self.early_stance_time_threshold = 0.07
    self.relative_tracking_error_threshold = 0.3
    self.absolute_min_distance_error_threshold = 0.1

    self.gait_cycles_after_reflex_height_enanchement = -1
    self.use_reflex_next_steps_height_enhancement = cfg.simulation_params['reflex_next_steps_height_enhancement']
    self.max_gait_cycles_height_enhancement = 6
```

- `feet_geom_id`：MuJoCo上の足geomのID(`LegsAttr`)。デフォルト`None`
- `legs_order`：脚名の並び順。デフォルト`('FL','FR','RL','RR')`
- `self.early_stance`：4脚分の`bool`。初期値すべて`False`
- `self.hitmoments`(秒)：早期接地を検知した瞬間の`swing_time`。初期値すべて`-1.0`(未検知を表す)
- `self.hitpoints`(m)：早期接地を検知した瞬間の足位置。初期値すべて`None`
- `self.trigger_mode`：`config.py`の`simulation_params['reflex_trigger_mode']`。既定`'tracking'`(他に`'geom_contact'`、`False`で無効化)
- `self.activated`：**既定`True`**(`trigger_mode`が`False`でない限り有効)。read_code_10・11で見た多くの機能とは異なり、この機能は既定のトロット歩行で実際に動いている
- `self.early_stance_time_threshold`(秒)：`0.07`固定
- `self.relative_tracking_error_threshold`(無次元、比率)：`0.3`固定
- `self.absolute_min_distance_error_threshold`(m)：`0.1`固定
- `self.use_reflex_next_steps_height_enhancement`：`config.py`の`simulation_params['reflex_next_steps_height_enhancement']`。既定`False`
- `self.max_gait_cycles_height_enhancement`(無次元、周期数)：`6`固定

**実装上の問題点(開発者自身が`# TO FIX`と認めているバグ)**：`cfg.mpc_params['type'] == 'sampling'`のとき`self.activated = False`を17〜18行目で設定しているが、その直後(20〜24行目)で`self.trigger_mode`の値だけを見て`self.activated`を**無条件に再設定**している。`trigger_mode`が既定`'tracking'`(truthy)である限り、たとえ`sampling`タイプであっても最終的に`self.activated`は`True`になり、17〜18行目の意図(サンプリングMPCではこの機能を無効化する)は**常に上書きされて無効になる**。既定の`'nominal'`タイプではこの17行目の`if`自体に入らないため、この章で扱う標準経路には直接影響しない

---

## 36〜129行:`update_detection`

この関数の役割:遊脚中の足の実測軌道が計画からどれだけ外れているかを見て、早期接地(または障害物への衝突)を検知し、その情報を記録する。

### 53〜57行:無効化時のリセット

```python
if not self.activated:
    for leg_id, leg_name in enumerate(self.legs_order):
        self.early_stance[leg_name] = False
        self.hitmoments[leg_name] = -1.0
        self.hitpoints[leg_name] = None
```

- `self.activated`が`False`(`reflex_trigger_mode=False`のとき)なら、4脚すべてを「未検知」状態にリセットするだけ。既定では`self.activated=True`のためこの分岐には入らない

### 59〜89行:`'tracking'`モード(既定)

```python
if self.trigger_mode == 'tracking':
    for leg_id, leg_name in enumerate(self.legs_order):
        disp = touch_down[leg_name] - lift_off[leg_name]
        if current_contact[leg_id] == 1:
            self.early_stance[leg_name] = False
            continue
        elif self.early_stance[leg_name] == False and swing_time[leg_id] > swing_period - self.early_stance_time_threshold:
            self.early_stance[leg_name] = False
            continue
        else:
            local_disp = (des_feet_pos[leg_name] - feet_pos[leg_name]).squeeze()
            if self.early_stance[leg_name] == False:
                if (np.linalg.norm(local_disp)/np.linalg.norm(disp)) > self.relative_tracking_error_threshold and np.linalg.norm(local_disp) > self.absolute_min_distance_error_threshold:
                    self.hitpoints[leg_name] = feet_pos[leg_name].copy()
                    self.hitmoments[leg_name] = swing_time[leg_id]
                    self.early_stance[leg_name] = True
                    self.gait_cycles_after_reflex_height_enanchement = 0
                    break
                else:
                    self.early_stance[leg_name] = False
                    self.hitmoments[leg_name] = -1.0
                    self.hitpoints[leg_name] = None
        if self.early_stance[leg_name] == False:
            self.hitmoments[leg_name] = -1.0
            self.hitpoints[leg_name] = None
```

- `disp`(m)：離陸位置から着地目標までの、計画上の総移動量
- 判定は3段階:
  1. 今すでに接地中(`current_contact==1`)の脚は「計画上の接地」であり、早期接地ではないので単純に`False`にリセット
  2. 遊脚の残り時間が`early_stance_time_threshold`(0.07秒)未満(=もうすぐ着地予定)の脚も、`False`のまま(スイング終盤は正常な着地との区別が難しいための除外と考えられる)
  3. それ以外の、まだスイング中盤の脚について：`local_disp`(目標軌道位置と実測位置の差)のノルムが、計画上の総移動量`disp`のノルムの**30%を超え**、かつ絶対値でも**0.1mを超えている**場合、「早期接地(または障害物への衝突)」と判定する
- 判定された瞬間の足位置(`hitpoints`)と経過時間(`hitmoments`)を記録し、`self.gait_cycles_after_reflex_height_enanchement`を`0`にリセットする(次のブロックで使われる)
- コメントアウトされた行(74行目)から、当初は変位の**向き**(内積の角度、60度)で判定する設計だったが、現在は変位の**大きさの比**で判定する方式に変更されていることが分かる(過去の実装の名残)

### 90〜115行:`'geom_contact'`モード(既定では選ばれない)

```python
elif self.trigger_mode == 'geom_contact':
    self.contact = mujoco_contact
    for leg_id, leg_name in enumerate(self.legs_order):
        contact_points = self.contact_points(leg_name)
        disp = touch_down[leg_name] - lift_off[leg_name]
        for contact_point in contact_points:
            if swing_time[leg_id] < self.early_stance_time_threshold or swing_time[leg_id] > swing_period - self.early_stance_time_threshold:
                self.early_stance[leg_name] = False
                break
            else:
                local_disp = (contact_point - feet_pos[leg_name]).squeeze()
                if self.early_stance[leg_name] == False:
                    if np.arccos(np.dot(disp, local_disp) / (np.linalg.norm(disp) * np.linalg.norm(local_disp))) < np.pi/3:
                        self.hitpoints[leg_name] = contact_point.copy()
                        self.hitmoments[leg_name] = swing_time[leg_id]
                        self.early_stance[leg_name] = True
                        self.gait_cycles_after_reflex_height_enanchement = 0
                        break
```

- `config.py`の既定`reflex_trigger_mode='tracking'`のため、このブロックは既定では実行されない
- `'tracking'`モードが「目標軌道からの実測位置のずれ」という**運動学的な推定**で早期接地を判定するのに対し、こちらは`mujoco_contact`(実際の接触情報、read_code_01で確認したMuJoCoの接触データ)を使い、足のgeomが実際に何かと接触した点を直接調べる、**センサー情報に基づく判定**。角度差(接触点への変位方向と、計画上の移動方向の内積)が60度(`π/3`)未満なら早期接地と判定する
- こちらは元々のコメントアウトされていない、角度ベースの判定方式がそのまま残っている(`'tracking'`モードとは異なる基準)

### 118〜128行:着地後の脚上げ強化リフレックス(既定OFF)

```python
if(self.use_reflex_next_steps_height_enhancement):
    for leg_id, leg_name in enumerate(self.legs_order):
        if current_contact[leg_id] == 1 and previous_contact[leg_id] == 0 and self.gait_cycles_after_reflex_height_enanchement >= 0:
            self.gait_cycles_after_reflex_height_enanchement += 1
            break
    if(self.gait_cycles_after_reflex_height_enanchement >= 0 and self.gait_cycles_after_reflex_height_enanchement < self.max_gait_cycles_height_enhancement):
        stc.swing_generator.reflex_next_steps_height_enhancement = True
    else:
        stc.swing_generator.reflex_next_steps_height_enhancement = False
        self.gait_cycles_after_reflex_height_enanchement = -1
```

- `self.use_reflex_next_steps_height_enhancement`は既定`False`(`config.py`)のため、このブロックは既定では実行されない
- 有効な場合の動作:早期接地が検知された(`gait_cycles_after_reflex_height_enanchement`が`0`以上になった)あと、遊脚→接地の遷移が起きるたびにこのカウンタを1つ増やし、`max_gait_cycles_height_enhancement`(6周期)に達するまでの間、`stc.swing_generator`(read_code_13、未読の軌道生成器本体)へ`reflex_next_steps_height_enhancement=True`というフラグを立てる。**この関数自体はスイング高さを直接変更せず、フラグを立てるだけ**で、実際に足を高く上げる処理は`swing_generator`(未読)側にあると考えられる(**設計上の解釈**)。障害物に一度つまずいたら、その後数歩は足を高めに上げて再発を防ぐ、という「反射」の意図が読み取れる

---

## 131〜147行:`contact_points`

この関数の役割:MuJoCoの接触情報の中から、指定した脚のgeomに関わる接触点だけを抽出する。

```python
def contact_points(self, leg_name):
    contact_points = []
    contact_id = np.where(np.any(self.contact.geom == self.feet_geom_id[leg_name], axis=1))
    if contact_id[0].size > 0:
        for i in range(contact_id[0].size):
            contact_points.append(self.contact.pos[contact_id[0][i]])
    return contact_points
```

- `leg_name`(文字列)：デフォルト値はなく必須引数
- `self.contact`(`update_detection`内で`mujoco_contact`から代入される)の`geom`配列(各接触ペアのgeom ID)を調べ、対象の脚のgeom IDが含まれる接触だけを`contact.pos`(m、接触点位置)のリストとして返す
- `'geom_contact'`モードでのみ使われる(既定では呼ばれない)

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `# TO FIX`とコメントされた通り、`sampling`タイプ向けの無効化(`self.activated=False`)が、直後の`trigger_mode`判定で常に上書きされてしまう
- 確認できた重要な事実(既定で有効な、数少ない機能):
  - `reflex_trigger_mode`は既定`'tracking'`で、`self.activated=True`。この検知処理自体は既定のトロット歩行でも毎周期動いている
  - 判定方式は「目標軌道からの実測位置のずれが、計画移動量の30%かつ絶対値0.1mを超えたら早期接地」という運動学的な推定
  - 検知結果(`hitmoments`/`hitpoints`)はread_code_13の`compute_swing_control_cartesian_space`へ渡されるが、実際にそれがスイング軌道をどう変えるかは軌道生成器(`swing_generator`、未読)側の実装次第
  - 高さ強化リフレックス(`reflex_next_steps_height_enhancement`)自体は既定`False`で無効
- 次は、逆運動学を担当する`InverseKinematicsNumeric`(`helpers/inverse_kinematics/inverse_kinematics_numeric_mujoco.py`)に進みます。これで、read_code_06の`WBInterface.__init__`で確認した主要コンポーネント(`pgg`, `frg`, `stc`, `terrain_computation`, `vm`, `esd`, `ik`)のうち、`ik`だけが未読として残ります。
