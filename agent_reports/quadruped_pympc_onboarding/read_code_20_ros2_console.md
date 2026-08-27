# ROS2 対話コンソール ros2/console.py 逐次解説

## 通信上の位置づけ

```text
ROS2トピックの購読・配信は無し。
```

`console.py`はROS2ノードではありません。[read_code_18](read_code_18_ros2_controller_init.md)
で確認した通り、`Quadruped_PyMPC_Node.__init__`が`Console(controller_node=self)`
として生成し、別のPythonスレッド(`thread_console`)として起動するだけの、
**同一プロセス内のヘルパークラス**です。`controller_node`引数として
`Quadruped_PyMPC_Node`インスタンス自身を受け取り、その属性
(`wb_interface`、`env`、`srbd_controller_interface`等)を直接読み書きします。
ROS2のpub/subは一切使いません。

## このファイルの役割(全体の中での位置づけ)

`Console`が担当するのは、「**ターミナルからのキーボード入力で、実行中の
`Quadruped_PyMPC_Node`の各種パラメータやモードを対話的に変更する**」ことです。
歩容の開始/停止、歩容タイプの切り替え、スイング/スタンスのゲイン調整、
CoMの高さ・ピッチ・オフセットの調整、起立/伏せ動作のトリガーなどを、
プログラムを再起動せずに実行時に変更できるデバッグ・運用ツールです。
制御ロジックの計算自体は一切行わず、既存のクラス
(`WBInterface`、`SRBDControllerInterface`、`PeriodicGaitGenerator`等の
インスタンス変数)を書き換えるだけです。

対象は`external/Quadruped-PyMPC/ros2/console.py`(415行)です。

---

## 14〜46行:`Console.__init__`

この関数の役割:`controller_node`への参照を保持し、起立/伏せの初期状態と
タブ補完を設定する。

```python
def __init__(self, controller_node):
    self.controller_node = controller_node
    self.walking = False
    self.isDown = True
    self.height_delta = -cfg.simulation_params['ref_z']
    self.pitch_delta = 0
    self.step_height_holder = cfg.simulation_params['step_height']

    self.commands = [ "help", "stw", "ooo", "ictp", "narrowStance", "wideStance",
                       "goUp", "goDown", "setGaitTimer", "setupGaitTimer",
                       "setupLegsGains", "setupGeneral" ]
    readline.set_completer(self.complete)
    readline.parse_and_bind("tab: complete")
```

- `self.walking`(`bool`)：初期値`False`。現在歩行中かどうかのフラグ
  (このクラス内だけで使う状態で、`WBInterface`側には伝わらない)
- `self.isDown`(`bool`)：初期値`True`。ロボットが伏せ姿勢かどうか。
  read_code_19の`compute_control_callback`が`self.console.isDown`を参照して
  PD目標を上書きする条件に使われる
- `self.height_delta`(m)：初期値`-cfg.simulation_params['ref_z']`。
  `ref_z`はhip_height由来(go2の既定`0.28`m)なので、初期値は`-0.28`m。
  これは「基準の目標CoM高さから、伏せている分だけ低くする」オフセットを
  意味する(read_code_19の`ref_state["ref_position"][2] += height_delta`)
- `self.pitch_delta`(rad)：初期値`0`
- `self.step_height_holder`(m)：`cfg.simulation_params['step_height']`
  (`0.2*hip_height`、go2既定で`0.056`m)。スイング軌道の目標最大高さ
  ([read_code_13](read_code_13_swing_trajectory_controller.md)の
  `step_height`引数)をコンソール側で保持しておくための変数
- `readline`のタブ補完設定：ターミナルで`Tab`キーを押すと`self.commands`
  リストにある12個のコマンド名を補完候補として提示する

---

## 49〜55行:`complete`

この関数の役割:`readline`のタブ補完コールバックとして、入力途中の文字列に
前方一致するコマンド名を返す。

```python
def complete(self, text, state):
    options = [cmd for cmd in self.commands if cmd.startswith(text)]
    if state < len(options):
        print(options[state])
        return options[state]
    else:
        return None
```

- `text`(文字列)：ユーザーが入力途中の文字列
- `state`(整数)：`readline`が同じ`text`に対して複数候補がある場合に
  `0,1,2,...`と順に呼び出す番号
- `options[state]`が存在すればその候補を返し、無ければ`None`を返して
  補完候補が尽きたことを`readline`に伝える

---

## 58〜397行:`interactive_command_line`

この関数の役割:標準入力から1行ずつコマンド文字列を読み取り、対応する処理を
実行する無限ループ(このメソッド自体が`thread_console`スレッドのターゲット)。

```python
def interactive_command_line(self, ):
    self.print_all_commands()
    while True:
        input_string = input(">>> ")
        try:
            if(input_string == "stw"):
                ...
        except Exception as e:
            print("Error: ", e)
            print("Invalid Command")
            self.print_all_commands()
```

- 全体が`try`/`except Exception`で囲まれており、コマンド処理中に何らかの
  例外(不正な数値入力等)が起きても、**このスレッド自体は落ちずに**
  エラーメッセージを表示してループを継続する

以下、各コマンド分岐を見ていきます。

### `"stw"`(84〜72行目):歩行開始

```python
if(self.isDown == True):
    print("The robot is down, please go up before starting to walk")
else:
    if(self.walking):
        print("The robot is already walking")
    print("Starting Walking")
    self.walking = True
    self.controller_node.wb_interface.pgg.gait_type = self.controller_node.wb_interface.pgg.previous_gait_type
    self.controller_node.wb_interface.pgg.reset()
```

- `self.isDown`(伏せ状態)のままでは歩行開始を拒否する安全策
- `pgg.gait_type`を`pgg.previous_gait_type`(直前に選ばれていた歩容タイプ、
  read_code_02の`PeriodicGaitGenerator`が持つ属性)へ戻し、`pgg.reset()`
  (read_code_02で確認済みのメソッド、位相をリセットする)を呼ぶ。
  [read_code_18](read_code_18_ros2_controller_init.md)で見た起動直後の
  `gait_type=7`(`FULL_STANCE`)強制設定から、実際の歩行タイプへ復帰する
  導線がここにある
- **実装上の問題点**:「`self.walking`が既に`True`」の場合でも
  `print`するだけで、後続の`self.walking = True`等の処理は`if`文の外に
  あるため**そのまま再実行されます**(`continue`していない)。実害は
  無い(同じ値を再代入し、`pgg.reset()`が再度呼ばれるだけ)が、
  意図としては早期`continue`すべき書き方に見える

### `"ooo"`(75〜78行目):歩行停止

```python
self.walking = False
self.controller_node.wb_interface.pgg.gait_type = 7 # FULL_STANCE
```

- 歩容タイプを`7`(`FULL_STANCE`)に戻すだけで、`pgg.reset()`は呼ばれない
  (`"stw"`との非対称性。**設計上の解釈**:全脚接地への切り替えは位相の
  連続性を保ったまま行いたい、という意図かもしれないが根拠となるコメントは
  無く**未確認**)

### `"narrowStance"` / `"wideStance"`(81〜88行目):左右の足幅調整

```python
self.controller_node.wb_interface.frg.hip_offset -= 0.03  # narrowStance
self.controller_node.wb_interface.frg.hip_offset += 0.03  # wideStance
```

- `frg.hip_offset`(m)：[read_code_03](read_code_03_foothold_reference_generator.md)
  で確認した`FootholdReferenceGenerator`のハードコード既定値`0.1`m を、
  コマンド1回につき`±0.03`mずつ増減させる。上限・下限のクランプは無い
  (**実装上の問題点**、繰り返し入力すると`hip_offset`が際限なく増減し
  続けられる)

### `"setGaitTimer"`(91〜141行目):歩容タイプの選択

```python
gait_type = int(input("Gait Type: >>> "))
if(gait_type == 7): gait_name = "full_stance"
elif(gait_type == 0): gait_name = "trot"
elif(gait_type == 1): gait_name = "pace"
elif(gait_type == 2): gait_name = "bound"
elif(gait_type == 3): gait_name = "crawl"
elif(gait_type == 4): gait_name = "crawl"
elif(gait_type == 5): gait_name = "crawl"
elif(gait_type == 6): gait_name = "crawl"

if(gait_type >= 0 and gait_type <= 7):
    gait_params = cfg.simulation_params['gait_params'][gait_name]
    gait_type, duty_factor, step_frequency = gait_params['type'], gait_params['duty_factor'], gait_params['step_freq']
    self.controller_node.wb_interface.pgg.step_freq = step_frequency
    self.controller_node.wb_interface.pgg.duty_factor = duty_factor
    self.controller_node.wb_interface.pgg.gait_type = gait_type
    self.controller_node.wb_interface.pgg.previous_gait_type = gait_type
    self.controller_node.wb_interface.pgg.reset()
    self.controller_node.wb_interface.frg.stance_time = (1 / step_freq) * duty_factor
    swing_period = (1 - duty_factor) * (1 / step_freq)
    self.controller_node.wb_interface.stc.regenerate_swing_trajectory_generator(step_height=self.step_height_holder, swing_period=swing_period)
```

**実装上の問題点(表示メニューと実際の動作の不一致)**:表示メニューは
`3: CIRCULARCRAWL`、`4: BFDIAGONALCRAWL`、`5: BACKDIAGONALCRAWL`、
`6: FRONTDIAGONALCRAWL`という**4種類の異なるクロール歩容**を提示しますが、
コード上は`3`・`4`・`5`・`6`のどれを選んでも同じ`gait_name="crawl"`に
写像されます。`config.py`の`cfg.simulation_params['gait_params']`には
`'crawl'`というキーが1つしか無く(実体は`GaitType.BACKDIAGONALCRAWL.value`、
`config.py`で確認済み)、結局`gait_type`は`gait_params['type']`で
**上書きされる**ため、ユーザーがメニューで`3`(CIRCULARCRAWL)や`6`
(FRONTDIAGONALCRAWL)を選んでも、**実際に適用される歩容タイプは常に
BACKDIAGONALCRAWLになります**。メニュー表示と実装が一致していない、
実運用上わかりにくいバグです。

- `gait_params['type']`/`['duty_factor']`/`['step_freq']`：
  `config.py`の`gait_params`辞書(read_code_02・[read_code_16](read_code_16_ros2_communication_overview.md)
  で既出)からロードする、無次元のenum値・無次元の比率・Hz
- `frg.stance_time`・`swing_period`の再計算式は
  [read_code_02](read_code_02_periodic_gait_generator.md)・
  [read_code_03](read_code_03_foothold_reference_generator.md)で確認した
  式と同一(`stance_time=(1/step_freq)*duty_factor`、
  `swing_period=(1-duty_factor)*(1/step_freq)`)。トロット既定値
  (`step_freq=1.4`、`duty_factor=0.65`)なら`stance_time≈0.464`秒、
  `swing_period=0.25`秒(read_code_02で確認済みの数値と同じ)
- `stc.regenerate_swing_trajectory_generator`：
  [read_code_13](read_code_13_swing_trajectory_controller.md)で読んだ
  メソッドそのもの。歩容切り替え時に、スイング軌道生成器を新しい
  `step_height`/`swing_period`で作り直す

### `"setupGaitTimer"`(144〜171行目):ステップ周波数・デューティ比・発進停止機能の個別調整

```python
temp = max(0.4, min(float(temp), 2.0))   # step_freq のクランプ
...
temp = max(0.4, min(float(temp), 0.9))   # duty_factor のクランプ
...
self.controller_node.wb_interface.pgg.start_and_stop_activated = True/False
```

- `step_freq`(Hz)：入力値を`[0.4, 2.0]`の範囲にクランプしてから適用
- `duty_factor`(無次元)：入力値を`[0.4, 0.9]`の範囲にクランプしてから適用
- 各入力後、`"setGaitTimer"`と同様に`frg.stance_time`・
  `stc.regenerate_swing_trajectory_generator`を再計算する
- `pgg.start_and_stop_activated`(`bool`)：
  [read_code_02](read_code_02_periodic_gait_generator.md)で確認した
  `PeriodicGaitGenerator.update_start_and_stop`(既定`False`で無効)を、
  この対話コマンドから`True`に切り替えられる。ROS2経路はこの機能を
  有効化する唯一のコード上の経路です(`simulation.py`経路にはこの機能を
  切り替えるUIが無い)
- 各入力欄は空文字列(単に`Enter`を押す)ならスキップされ、既存の値が
  維持される

### `"setupLegsGains"`(173〜194行目):スイング/スタンスのPDゲイン調整

```python
self.controller_node.wb_interface.stc.position_gain_fb = float(temp)  # スイングKp
self.controller_node.wb_interface.stc.velocity_gain_fb = float(temp)  # スイングKd
self.controller_node.impedence_joint_position_gain = np.ones(12)*float(temp)  # スタンスKp
self.controller_node.impedence_joint_velocity_gain = np.ones(12)*float(temp)  # スタンスKd
```

- `stc.position_gain_fb`/`velocity_gain_fb`：
  [read_code_13](read_code_13_swing_trajectory_controller.md)で確認した
  スイング脚のフィードバックPDゲイン(無次元、既定`swing_position_gain_fb=500`・
  `swing_velocity_gain_fb=10`、`config.py`)を上書きする
- `controller_node.impedence_joint_position/velocity_gain`：
  [read_code_18](read_code_18_ros2_controller_init.md)で見た、
  `/trajectory_generator`の`kp`/`kd`として送信される12関節分の
  インピーダンスゲイン(既定`10.0`/`2.0`)を上書きする。値の入力範囲の
  クランプは無い

### `"setupGeneral"`(196〜264行目):CoM高さ・ステップ高さ・各種フラグの調整

```python
height_delta_temp = float(height_temp) - cfg.simulation_params['ref_z']
self.height_delta = max(-0.1, min(height_delta_temp, 0.1))
...
self.step_height_holder = max(0.05, min(float(step_height_temp), 0.25))
self.controller_node.wb_interface.stc.regenerate_swing_trajectory_generator(self.step_height_holder, swing_period_temp)
...
self.controller_node.wb_interface.stc.use_feedback_linearization = True/False
self.controller_node.wb_interface.stc.use_friction_compensation = True/False
self.controller_node.srbd_controller_interface.controller.use_integrators = True/False
self.controller_node.wb_interface.frg.com_pos_offset_b[0/1/2] = float(temp)  # クランプ [-0.1, 0.1]
self.controller_node.wb_interface.esd.activated = True/False
```

- CoM高さの入力値は`ref_z`からの差分に変換され、`[-0.1, 0.1]`mの範囲に
  クランプされる
- ステップ高さは`[0.05, 0.25]`mにクランプされる
- `stc.use_feedback_linearization`/`use_friction_compensation`：
  [read_code_13](read_code_13_swing_trajectory_controller.md)で確認した、
  本来ハードコード`True`固定だったフラグを、この対話コマンドからは
  **実行時に`False`へ切り替えられます**。つまり
  「コード上のデフォルトはTrue固定」という以前の指摘は、`simulation.py`
  経路にのみ当てはまり、ROS2経路では実行時に変更可能です
- `srbd_controller_interface.controller.use_integrators`：
  [read_code_11](read_code_11_compute_control.md)で確認したMPCの
  積分器(既定OFF)を実行時にON/OFFできる
- `frg.com_pos_offset_b`(m、3成分、`[-0.1,0.1]`クランプ)：
  [read_code_03](read_code_03_foothold_reference_generator.md)で扱った
  CoMオフセットをbase座標系で個別に調整する
- `esd.activated`：[read_code_14](read_code_14_early_stance_detector.md)
  で確認した`EarlyStanceDetector.activated`(既定`True`)を実行時に
  切り替えられる

### `"goUp"` / `"goDown"`(268〜336行目):起立・伏せ動作

```python
# goUp
initial_joint_positions = (現在の関節角度から構築)
reference_joint_positions = ("home"キーフレームから構築)
while(time.time() - start_time < time_motion):  # time_motion = 5.0秒
    alpha = time_diff / time_motion
    interpolated_positions = [(1-alpha)*initial + alpha*reference for ...]
    self.controller_node.stand_up_and_down_actions.FL = interpolated_positions[0]
    ...
    self.height_delta = initial_height + (cfg.simulation_params['ref_z'] * time_diff / time_motion)
    time.sleep(0.01)
self.height_delta = 0
self.isDown = False
```

- `time_motion`(秒)：`5.0`固定。起立・伏せにかける所要時間
- `goUp`：現在の関節角度から、MuJoCoモデルの`"home"`キーフレーム
  (通常の起立姿勢)まで、5秒かけて**線形補間**した関節角度を
  `self.controller_node.stand_up_and_down_actions`へ書き込み続ける
- `goDown`：`height_delta`を`0`から`-ref_z`まで5秒かけて線形に下げる
  (関節角度側の補間は無く、高さオフセットだけを動かす非対称な実装)
- **実装上の問題点**:この`while`ループは`Console`のスレッド内で
  `time.sleep(0.01)`しながら回り続けるだけで、実際に
  `stand_up_and_down_actions`をロボットの動作へ反映するのは
  `read_code_19`の`compute_control_callback`側(`self.console.isDown`が
  `True`の間、`pd_target_joints_pos`をこの変数で上書きする)です。
  ただし`goUp`実行中は`self.isDown`がまだ`True`のままなので
  (`goUp`ループの最後で初めて`False`にする)、`compute_control_callback`
  はこの5秒間ずっと`stand_up_and_down_actions`(補間中の値)をPD目標として
  使い続けます。`goDown`側は関節角度の補間を行わないため、伏せ動作中の
  実際の関節軌道はWBC側(`nmpc_joints_pos`等)に委ねられ、高さオフセットの
  変化だけがCoM高さの目標を通じて間接的に伏せる動きを作る、という
  `goUp`とは異なる実現方法になっている(**設計上の解釈**)

### `"help"`(338〜340行目)

```python
elif(input_string == "help"):
    self.print_all_commands()
```

### `"ictp"`(342〜393行目):キーボードによる速度指令の対話操作

```python
command = readchar.readkey()
if(command == "w"): self.controller_node.env._ref_base_lin_vel_H[0] += 0.1
elif(command == "s"): self.controller_node.env._ref_base_lin_vel_H[0] -= 0.1
elif(command == "a"): self.controller_node.env._ref_base_lin_vel_H[1] += 0.1
elif(command == "d"): self.controller_node.env._ref_base_lin_vel_H[1] -= 0.1
elif(command == "q"): self.controller_node.env._ref_base_ang_yaw_dot += 0.1
elif(command == "e"): self.controller_node.env._ref_base_ang_yaw_dot -= 0.1
elif(command == "0"): (すべて0にリセット)
elif(command == "1"): self.pitch_delta -= 0.1
elif(command == "2"): self.pitch_delta = 0
elif(command == "3"): self.pitch_delta += 0.1
else: (すべて0にリセットしてループを抜ける)
```

- キー1回につき`±0.1`(m/sまたはrad/s)ずつ`env._ref_base_lin_vel_H`
  (前後・左右)、`_ref_base_ang_yaw_dot`(ヨー)を増減する。これは
  `get_joy_callback`(read_code_19)が`/joy`経由で行うのと**同じ変数**を、
  こちらはキーボードから直接書き換える、もう1つの速度指令入力経路
- 上限・下限のクランプは無い(`get_joy_callback`側は`axes`値が
  `-1〜1`のジョイスティックハードウェア自体に物理的な制約があるが、
  こちらはキー入力回数に応じて際限なく増減できる、**実装上の問題点**)
- `w/a/s/d`はゲーム的な操作感(前後左右)、`q/e`はヨー回転、`1/2/3`は
  ピッチの微調整に割り当てられている

---

## 400〜415行:`print_all_commands`

この関数の役割:利用可能な全コマンドの一覧をターミナルへ表示する。

```python
def print_all_commands(self):
    print("\nAvailable Commands")
    print("help: Display all available messages")
    ...
```

- 単純な`print`の羅列。`self.commands`リスト(タブ補完用、`__init__`)とは
  別に、この関数内で個別にハードコードされている。**実装上の問題点**:
  `self.commands`に無い項目は無いが、逆に両方のリストを手動で同期させる
  必要があり、片方だけ更新すると表示とタブ補完がずれる可能性がある

---

## この章のまとめ

- 見つかった実装上の問題点:
  1. `"setGaitTimer"`のクロール歩容選択(`3`〜`6`)は、メニュー表示上は
     4種類の異なる歩容に見えるが、`config.py`側に`'crawl'`エントリが
     1つしか無いため、実際にはすべて同じ`BACKDIAGONALCRAWL`になる
  2. `"narrowStance"`/`"wideStance"`、`"ictp"`のキー操作、いずれも
     上限・下限のクランプが無く、繰り返し操作で際限なく値が変化しうる
     (`"setupGaitTimer"`等の数値入力コマンドはクランプがあるのと対照的)
  3. `"stw"`の「既に歩行中」チェックが`print`のみで`continue`しておらず、
     実質的に無意味
  4. `self.commands`(タブ補完用)と`print_all_commands`(表示用)が
     別々にハードコードされており、二重管理になっている
- 確認できた重要な事実:
  - `Console`はROS2通信を一切使わない、`Quadruped_PyMPC_Node`の内部状態を
    直接書き換えるだけの対話スレッドである
  - `simulation.py`経路には無い、いくつかの機能がROS2経路にだけ存在する:
    歩容発進/停止の遷移制御(`start_and_stop_activated`の切り替えUI)、
    実行時のフィードバック線形化/摩擦補償/積分器のON-OFF切り替え、
    キーボードによる速度指令・ピッチ操作(`ictp`)
  - `Console`は、read_code_06(`WBInterface.__init__`)以降で読んだほぼ全
    クラスの内部変数(`pgg`、`frg`、`stc`、`esd`、
    `srbd_controller_interface.controller`)に直接触れる、いわば
    「これまで読んだ制御ロジック全体のデバッグリモコン」である

これで、ROS2経路(read_code_16〜20)の5ファイルすべてを読み終えました。
`simulation.py`経路(read_code_01〜15)と合わせて、`WBInterface`初期化時の
主要コンポーネント(`pgg`/`frg`/`stc`/`terrain_computation`/`vm`/`esd`/`ik`)、
MPC本体(`SRBDControllerInterface`/`Centroidal_Model_Nominal`/
`Acados_NMPC_Nominal`)、そして通信層(ROS2)の3層が一通り揃いました。

残っている未読範囲(次に読む候補、優先度低・既定OFF):
Visual Foothold Adaptation(`helpers/visual_foothold_adaptation.py`)、
サンプリング/JAXベースのMPC経路、`SRBDBatchedControllerInterface`、
スイング軌道生成器の実体(`swing_generators/scipy_swing_trajectory_generator.py`、
`explicit_swing_trajectory_generator.py`)。
