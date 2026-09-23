# 姿勢復帰モードの試行記録

## 第1章 概要

- この文書は、姿勢復帰モードの試行を実行順に一件ずつ記録する
- 各試行は、背景、目的、結論、詳細、GIF、次への施策の順に記録する
- 成功試行と失敗試行と起動異常は、結果を省略せず同じ書式で記録する
- 試行 R01 から試行 R32 までを、実行順に記録した
- 試行 R16 は、録画ファイルが残らなかったため GIF が無い
- 平地の仰向けは試行 R12 で直立へ戻り、約 5.66 m 歩いた
- 平地の右横倒しは試行 R14 で直立へ戻り、約 3.10 m 歩いた
- 平地の左横倒しは、試行 R19 で一度直立へ戻ったが歩行で沈み、試行 R22 と試行 R23 は仰向けの押しが止まって失敗した
- 試行 R29 は左横倒しを指定したが、記録開始時には仰向けへ落ち着いており、仰向けの復帰で約 6.26 m 歩いた
- 試行 R30 は左横倒しから仰向けへつながったが、両股関節が 2.60 rad へ戻って押しが `u` 約 -0.73 で止まり `timeout` になった
- 試行 R31 は左横倒しから右脚の押しを続けて直立へ戻り、`MONITOR` の歩行まで進んだ。移動距離の CSV は残らなかった
- 試行 R32 は同じ左横倒しから直立へ戻り、約 6.55 m 歩いた
- 階段の試行 R24 は、約 5.05 m 進んで高さ約 0.92 m まで上がったあと転倒し、復帰は `attempt_limit` で終わった
- 階段の試行 R25 は、上りと下りを直立のまま歩き、終了位置は x 約 6.71 m、高さ約 0.31 m だった
- 階段の試行 R26 は、下りで仰向けへ落ち、静止してからの押しが `u` 約 -0.73 で止まり `timeout` になった
- 階段の試行 R27 は、回転中に仰向けの押しへつながったが、段の上の横向きで `timeout` になった
- 階段の試行 R28 は、上ってから落ちたあと 1 回で直立へ戻り、`RESUME` から歩行へ戻った

## 試行 R01 仰向け復帰の初回起動

### 背景

- 開発者は、新しい復帰ノードが既存の最終関節指令を排他的に所有できるかを最初に確認する必要があった

### 目的

- 開発者は、仰向けの初期姿勢を判定し、復帰キーフレームを最終関節指令へ出せることを確認した

### 結論

- 試行は、置換した `robot_driver_node` がロボットモデルを読み込めず終了したため、起動異常として失敗した
- 新しい復帰ノードは仰向けを判定して `RECOVERY` を開始したが、歩行指令を生成する置換ノードが無いため走行再開を確認できなかった

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 復帰ノードは、仰向けを判定し、二回の復帰試行を開始した
- 置換した `robot_driver_node` は、元の MuJoCo 起動が生成した `robot_description` を受け取らず、`Error loading model` で終了した
- 試行シェルは、購読者が無い `control/mode` の一回送信を待ち続けたため、試行を手動終了した
- 実行コマンドは `RECOVERY_DURATION_S=30 bash scripts/trial/run_recovery_trial.sh flat supine r01_supine_initial` である
- CSV は `artifacts/logs/recovery_modes/r01_supine_initial/recovery.csv` と `artifacts/logs/recovery_modes/r01_supine_initial/state_log.csv` に保存した

### GIF

![仰向けの初回起動で復帰ノードが動き始めた試行](../../artifacts/logs/recovery_modes/r01_supine_initial/trial.gif)

### 次への施策

- 次の試行では、元の `robot_driver_node` の全パラメータを保存し、同じ `robot_description` を使う置換ノードを起動する

## 試行 R02 ドライバパラメータの保存

### 背景

- 試行 R01 では、置換した `robot_driver_node` が `robot_description` を持たずに終了した
- 開発者は、起動中のドライバから全パラメータを保存し、同じモデル文字列で置換ノードを起動する必要があった

### 目的

- 開発者は、`/robot_1/robot_driver` のパラメータを保存して復帰ノードを起動できることを確認した

### 結論

- 試行は、元の `robot_driver_node` がすでに終了していてパラメータを保存できず、起動異常として失敗した
- 復帰ノードは起動前に試行シェルが終了したため、仰向けからの復帰を確認できなかった

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- `ros2 param dump /robot_1/robot_driver` は、対象ノードが無いため保存に失敗した
- 試行シェルは、保存失敗のあとシミュレータを終了した
- 実行コマンドは `bash scripts/trial/run_recovery_trial.sh flat supine r02_supine_driver_params` である
- コンソールは `artifacts/logs/recovery_modes/r02_supine_driver_params/console.log` に保存した

### GIF

![パラメータ保存に失敗して短時間で終了した試行](../../artifacts/logs/recovery_modes/r02_supine_driver_params/trial.gif)

### 次への施策

- 次の試行では、元のドライバが居ないときは置換せず、復帰ノードだけが最終関節指令を出す

## 試行 R03 元ドライバ不在での起動

### 背景

- 試行 R02 では、仰向けの起動直後に元の `robot_driver_node` が居なかった
- 開発者は、その終了が復帰ノードの置換によるものか、元の起動自体の失敗かを分ける必要があった

### 目的

- 開発者は、元の MuJoCo 起動が仰向けのまま `robot_driver_node` を維持できるかを確認した

### 結論

- 試行は、元の `robot_driver_node` が `Error loading model` で終了し、復帰ノードを起動する前に試行シェルが止まったため失敗した
- 胴体は仰向けのまま残り、復帰動作は始まらなかった

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 元の `robot_driver_node` は、起動から約 30 ms で `Error loading model` を出して終了した
- 試行シェルは、パラメータ保存を待ったあと、ドライバが居ないため終了した
- 実行コマンドは `bash scripts/trial/run_recovery_trial.sh flat supine r03_supine_side_push` である
- 状態 CSV は `artifacts/logs/recovery_modes/r03_supine_side_push/state_log.csv` に保存した

### GIF

![元のドライバが終了し仰向けのまま残った試行](../../artifacts/logs/recovery_modes/r03_supine_side_push/trial.gif)

### 次への施策

- 次の試行では、ドライバが居ないとき復帰ノードだけを起動し、仰向けの脚指令を最終関節指令へ出す

## 試行 R04 片側の脚を空へ向ける押し

### 背景

- 試行 R03 では、元の `robot_driver_node` がモデル読込で終了し、最終関節指令の所有者が居なかった
- 開発者は、復帰ノードだけが `control/joint_command` を出し、仰向けから胴体を回せるかを確認する必要があった

### 目的

- 開発者は、片側の股関節を 0.20 rad にした押しで、仰向けの胴体が横向きへ回ることを確認した

### 結論

- 試行は、二回の試行回数上限で `SAFETY` に入り、最終姿勢は仰向けのまま失敗した
- 股関節指令の最大は 1.70 rad で、仰向けでは足先が地面へ届かず、胴体の上向き成分 `u` は約 -1.0 のままだった

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 外転関節は動いたが、高さは 0.058 m から 0.067 m の範囲に留まった
- 試行回数は 2、移動距離は約 0 m、直立到達は無い
- 実行コマンドは `RECOVERY_DURATION_S=35 bash scripts/trial/run_recovery_trial.sh flat supine r04_supine_side_push` である
- CSV は `artifacts/logs/recovery_modes/r04_supine_side_push/recovery.csv` に保存した

### GIF

![足先が地面へ届かず仰向けのまま終わった試行](../../artifacts/logs/recovery_modes/r04_supine_side_push/trial.gif)

### 次への施策

- 次の試行では、仰向けで足先が地面へ届くよう、押す側の股関節を 3.10 rad 付近まで伸ばす

## 試行 R05 地面へ届く股関節の押し

### 背景

- 試行 R04 では、股関節 0.20 rad の指令が足先を空へ向け、胴体が回らなかった
- 開発者は、Go2 の股関節範囲に合わせ、押す側を 3.10 rad、膝を -0.90 rad へ変える必要があった

### 目的

- 開発者は、地面へ届く股関節指令で仰向けの胴体が横向きへ回ることを確認した

### 結論

- 試行は、二回の試行回数上限で `SAFETY` に入り、最終姿勢は仰向けのまま失敗した
- 押す側の股関節は約 2.95 rad まで届いたが、反対側の脚を股関節 1.20 rad へ畳んだため、開いた隙間が閉じて背中へ戻った

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 隙間を開いた瞬間の高さは約 0.14 m、`u` は約 -0.85 まで動いた
- 押しのあと高さは約 0.07 m、`u` は約 -1.0 へ戻った
- その後も仰向けのまま低い支持と起立のキーフレームへ進んだ
- 実行コマンドは `RECOVERY_DURATION_S=40 bash scripts/trial/run_recovery_trial.sh flat supine r05_supine_ground_push` である
- CSV は `artifacts/logs/recovery_modes/r05_supine_ground_push/recovery.csv` に保存した

### GIF

![地面へ届く押しのあと背中へ戻った試行](../../artifacts/logs/recovery_modes/r05_supine_ground_push/trial.gif)

### 次への施策

- 次の試行では、押さない側の脚も地面へ届く姿勢のまま残し、仰向けの間は起立キーフレームへ進まない

## 試行 R06 隙間を保った片側の押し

### 背景

- 試行 R05 では、押さない側の脚を畳んだため、開いた隙間が閉じて背中へ戻った
- 開発者は、左脚を外転の正方向、右脚を外転の負方向へ開き、片側だけ股関節をさらに伸ばす必要があった

### 目的

- 開発者は、隙間を保った押しで仰向けから直立まで戻ることを確認した

### 結論

- 試行は、2 回目の復帰で仰向けから直立まで戻り、`RESUME` へ入った
- 走行再開は、新しい歩行関節指令が来ないため未達で、移動距離は約 0.35 m だった

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 1 回目は横向きを短時間通過したあと仰向けへ戻り、起立キーフレームが背中の上で実行された
- 2 回目は右横向きからうつぶせを経て、高さ 0.316 m、`u` が 1.0 の直立へ到達した
- 直立後のモードは `RESUME` の `wait_new_plan` のままで、歩行関節指令は来なかった
- 実行コマンドは `RECOVERY_DURATION_S=40 bash scripts/trial/run_recovery_trial.sh flat supine r06_supine_hold_gap` である
- CSV は `artifacts/logs/recovery_modes/r06_supine_hold_gap/recovery.csv` に保存した

### GIF

![隙間を保った押しで直立まで戻った試行](../../artifacts/logs/recovery_modes/r06_supine_hold_gap/trial.gif)

### 次への施策

- 次の試行では、仰向けのまま起立キーフレームへ進まず、横向きの姿勢列へ切り替えて寝返りを続ける

## 試行 R07 横向きの姿勢列への切替

### 背景

- 試行 R06 の 1 回目は、仰向けのまま起立キーフレームへ進んで背中へ戻った
- 開発者は、横向きと判定した時点で左横倒しの姿勢列へ切り替え、仰向けの起立を止める必要があった

### 目的

- 開発者は、横向き判定のあと左横倒しの姿勢列が寝返りを続けることを確認した

### 結論

- 試行は、時間切れで `SAFETY` に入り、最終姿勢は仰向けのまま失敗した
- 左横倒しの姿勢列が脚を畳み、横向きまで回った胴体を背中へ戻した

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 復帰中の `u` は一時約 -0.15 まで上がり、左横向きを通過した
- 姿勢列の切替のあと `u` は約 -1.0 へ戻り、同じ往復を時間切れまで繰り返した
- 試行回数は 1、失敗理由は `timeout`、直立到達は無い
- 実行コマンドは `RECOVERY_DURATION_S=40 bash scripts/trial/run_recovery_trial.sh flat supine r07_supine_roll_gate` である
- CSV は `artifacts/logs/recovery_modes/r07_supine_roll_gate/recovery.csv` に保存した

### GIF

![横向きの姿勢列へ切り替えて背中へ戻った試行](../../artifacts/logs/recovery_modes/r07_supine_roll_gate/trial.gif)

### 次への施策

- 次の試行では、仰向けの復帰中は姿勢列を切り替えず、仰向けのキーフレームだけで寝返りを続ける

## 試行 R08 仰向けのキーフレームを維持

### 背景

- 試行 R07 では、左横倒しの姿勢列が脚を畳み、回り始めた胴体を背中へ戻した
- 開発者は、仰向けの復帰中は姿勢列を固定し、横向きのあいだは起立へ進まないようにする必要があった

### 目的

- 開発者は、仰向けの押しを維持したまま胴体がうつぶせまで回ることを確認した

### 結論

- 試行は、二回の試行回数上限で `SAFETY` に入り、最終姿勢は仰向けのまま失敗した
- 押しの終了時に `u` は約 -0.23 の左横向きだったが、次の低い支持が脚を胴体下へ集め、背中へ戻った

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 1 回目の押しは、終了時に左横向きで、その直後の低い支持で `u` が約 -0.55 より小さい仰向けへ戻った
- 2 回目も同じ位置で仰向けへ戻り、試行回数上限に達した
- 実行コマンドは `RECOVERY_DURATION_S=55 bash scripts/trial/run_recovery_trial.sh flat supine r08_supine_keep_roll` である
- CSV は `artifacts/logs/recovery_modes/r08_supine_keep_roll/recovery.csv` に保存した

### GIF

![横向きで低い支持へ移って背中へ戻った試行](../../artifacts/logs/recovery_modes/r08_supine_keep_roll/trial.gif)

### 次への施策

- 次の試行では、横向きのあいだは同じ押しを維持し、低い支持へ移らない

## 試行 R09 横向きの押しを維持

### 背景

- 試行 R08 では、横向きに達した直後の低い支持が胴体を背中へ戻した
- 開発者は、横向きのあいだは押しの関節目標を維持し、うつぶせまで回り切るかを確認する必要があった

### 目的

- 開発者は、横向きの押しを維持すると胴体がうつぶせまで回ることを確認した

### 結論

- 試行は、時間切れで `SAFETY` に入り、最終姿勢は仰向けのまま失敗した
- 押しの維持で左横向きは約 20 s 保たれたが、`u` は約 -0.21 で止まり、うつぶせまで回らなかった

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 反転した押しのあと、左横向きを時間切れまで維持した
- 時間切れ後の減衰で `u` は約 -0.55 を下回り、仰向けへ戻った
- 試行回数は 1、失敗理由は `timeout`、直立到達は無い
- 実行コマンドは `RECOVERY_DURATION_S=55 bash scripts/trial/run_recovery_trial.sh flat supine r09_supine_hold_side` である
- CSV は `artifacts/logs/recovery_modes/r09_supine_hold_side/recovery.csv` に保存した

### GIF

![横向きの押しを維持して止まった試行](../../artifacts/logs/recovery_modes/r09_supine_hold_side/trial.gif)

### 次への施策

- 次の試行では、横向きになった瞬間に地面側の脚を伸ばし、腹側まで倒す

## 試行 R10 地面側の脚を伸ばす

### 背景

- 試行 R09 では、横向きの押しを維持しても `u` が約 -0.21 で止まり、うつぶせまで回らなかった
- 開発者は、地面に付いている側の脚を股関節 3.20 rad まで伸ばし、反対側を畳む必要があった

### 目的

- 開発者は、地面側の脚を伸ばすと横向きの胴体がうつぶせまで回ることを確認した

### 結論

- 試行は、二回の試行回数上限で `SAFETY` に入り、最終姿勢は仰向けのまま失敗した
- 横向き判定が短時間で仰向けへ戻るたびに目標を切り替えたため、`u` は約 -0.39 までしか上がらなかった

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 左横向きの区間は 0.3 s 未満で、地面側の股関節指令が 3.20 rad に届く前に仰向けへ戻った
- 試行回数は 2、直立到達は無い、移動距離は約 -0.03 m だった
- 実行コマンドは `RECOVERY_DURATION_S=55 bash scripts/trial/run_recovery_trial.sh flat supine r10_supine_side_lever` である
- CSV は `artifacts/logs/recovery_modes/r10_supine_side_lever/recovery.csv` に保存した

### GIF

![地面側の脚を伸ばす前に仰向けへ戻った試行](../../artifacts/logs/recovery_modes/r10_supine_side_lever/trial.gif)

### 次への施策

- 次の試行では、隙間作りから押しまでの継ぎ目を短くし、初動の勢いが残るうちに次の姿勢へ移る

## 試行 R11 仰向けの継ぎ目を速くする

### 背景

- 開発者は、隙間を開いてから片側を押すまでの待ちが長く、胴体が背中へ戻ってから次の脚姿勢が来ていた
- 開発者は、補間が各キーフレームの端で速度を零に戻すため、初動の勢いが次の押しへ残らないと判断した

### 目的

- 開発者は、隙間作りから片側の押しまでを短い時間でつなぎ、仰向けから直立まで戻ることを確認した

### 結論

- 試行は、1 回の復帰で仰向けから横向き、うつぶせ、直立まで戻った
- 走行再開は、`robot_driver_node` がモデル読込で終了し、新しい歩行関節指令が来ないため未達である

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- `open_gap` の時間を 1.2 s から 0.35 s に、`side_push` の時間を 2.5 s から 0.90 s に短くした
- 関節補間は、経過 25% の時点で目標の半分を超える曲線へ変えた
- 復帰開始から直立判定まで、シミュレータ時刻で約 1.7 s だった
- 胴体の上向き成分 `u` は -1.0 から 1.0 まで変わり、高さは 0.058 m から 0.316 m まで上がった
- 最終モードは `RESUME` で、移動距離は約 0.02 m だった
- 実行コマンドは `RECOVERY_DURATION_S=40 bash scripts/trial/run_recovery_trial.sh flat supine r11_supine_fast_seam` である
- CSV は `artifacts/logs/recovery_modes/r11_supine_fast_seam/recovery.csv` に保存した

### GIF

![仰向けから速い継ぎ目で直立まで戻った試行](../../artifacts/logs/recovery_modes/r11_supine_fast_seam/trial.gif)

### 次への施策

- 次の試行では、直立後に `robot_driver_node` が歩行用の関節指令を出し続け、2 m 進むことを確認する

## 試行 R12 仰向けから平地を歩く

### 背景

- 試行 R11 では、仰向けから直立まで戻ったあと、`robot_driver_node` がモデルを読めず歩行関節指令が来なかった
- 開発者は、`quad_utils` と `robot_driver` を同じ Pinocchio 4.1 で作り直し、直立後に既存の歩行指令へ戻す必要があった

### 目的

- 開発者は、仰向けから復帰したあと平地を 2 m 以上歩くことを確認した

### 結論

- 試行は成功し、1 回の復帰で直立へ戻り、そのあと平地を約 5.66 m 歩いた
- 最終モードは `MONITOR` で、最終姿勢は直立だった

### 詳細

- 初期条件は、平地で roll を 3.14159 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 作り直した `robot_driver_node` は、Pinocchio のモデルを 14 関節で読み、終了しなかった
- 試行回数は 1、直立到達はあり、`RESUME` を経て歩行指令の通過まで進んだ
- 移動距離は 5.66 m で、シナリオ1の 2 m を超えた
- 実行コマンドは `RECOVERY_DURATION_S=70 bash scripts/trial/run_recovery_trial.sh flat supine r12_supine_walk` である
- CSV は `artifacts/logs/recovery_modes/r12_supine_walk/recovery.csv` に保存した

### GIF

![仰向けから復帰して平地を歩いた試行](../../artifacts/logs/recovery_modes/r12_supine_walk/trial.gif)

### 次への施策

- 次の試行では、左横倒しを初期姿勢にして、同じ速さの押しから直立と 2 m の歩行まで進む

## 試行 R13 左横倒しから歩く

### 背景

- 試行 R12 では、仰向けから復帰して平地を 2 m 以上歩いた
- 開発者は、シナリオ1の残りの初期姿勢として、左横倒しから同じ復帰で歩き始める必要があった

### 目的

- 開発者は、左横倒しから直立へ戻り、平地を 2 m 以上歩くことを確認した

### 結論

- 試行は、二回の試行回数上限で `SAFETY` に入り、失敗した
- 地面側の左脚を伸ばす押しが胴体を仰向け側へ回し、起立キーフレームのあいだも横向きのまま高さが 0.21 m を超えなかった

### 詳細

- 初期条件は、平地で roll を -1.5708 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 最初のキーフレームは、左脚の股関節を 3.20 rad、右脚の股関節を 1.50 rad にした
- 復帰開始の `u` は約 -0.06 で、押しの直後に約 -0.73 まで下がり、仰向け側へ回った
- 最終姿勢は減衰後にうつぶせと判定されたが、高さの最大は約 0.21 m で直立ではない
- 移動距離は約 -0.10 m だった
- 実行コマンドは `RECOVERY_DURATION_S=55 bash scripts/trial/run_recovery_trial.sh flat left_side r13_left_walk` である
- CSV は `artifacts/logs/recovery_modes/r13_left_walk/recovery.csv` に保存した

### GIF

![左横倒しの押しが仰向け側へ回った試行](../../artifacts/logs/recovery_modes/r13_left_walk/trial.gif)

### 次への施策

- 次の試行では、右横倒しを初期姿勢にして、右脚を伸ばす押しで直立と歩行まで進む

## 試行 R14 右横倒しから歩く

### 背景

- 試行 R13 では、左脚を伸ばす押しが左横倒しを仰向け側へ回した
- 開発者は、右横倒しでは右脚を伸ばす押しが腹側へ回るかを分けて確認する必要があった

### 目的

- 開発者は、右横倒しから直立へ戻り、平地を 2 m 以上歩くことを確認した

### 結論

- 試行は成功し、直立へ戻ったあと平地を約 3.10 m 歩いた
- 最終モードは `MONITOR` で、最終姿勢は直立だった

### 詳細

- 初期条件は、平地で roll を 1.5708 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 最初のキーフレームは、右脚の股関節を 3.20 rad、左脚の股関節を 1.50 rad にし、時間を 0.40 s にした
- 試行回数は 2 だが、直立到達と `RESUME` と歩行指令の通過まで進んだ
- 移動距離は 3.10 m で、シナリオ1の 2 m を超えた
- 実行コマンドは `RECOVERY_DURATION_S=55 bash scripts/trial/run_recovery_trial.sh flat right_side r14_right_walk` である
- CSV は `artifacts/logs/recovery_modes/r14_right_walk/recovery.csv` に保存した

### GIF

![右横倒しから復帰して平地を歩いた試行](../../artifacts/logs/recovery_modes/r14_right_walk/trial.gif)

### 次への施策

- 次の試行では、左横倒しの押しを右脚伸展へ入れ替え、腹側へ回してから立つ

## 試行 R15 左横倒しを右脚で押す

### 背景

- 試行 R13 では、左脚の伸展が左横倒しを仰向け側へ回した
- 試行 R14 では、右脚の伸展が右横倒しから直立と歩行まで届いた
- 開発者は、左横倒しでも右脚を伸ばすと同じ回転で腹側へ倒れるかを確認する必要があった

### 目的

- 開発者は、左横倒しから右脚の伸展で直立へ戻ることを確認した

### 結論

- 試行は、二回の試行回数上限で `SAFETY` に入り、最終姿勢は仰向けのまま失敗した
- 右脚の伸展は、左横倒しを腹側ではなく背中側へ回した

### 詳細

- 初期条件は、平地で roll を -1.5708 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 最初のキーフレームは、試行 R14 と同じく右脚の股関節を 3.20 rad、左脚の股関節を 1.50 rad にした
- 直立到達は無く、移動距離は約 -0.45 m だった
- 実行コマンドは `RECOVERY_DURATION_S=55 bash scripts/trial/run_recovery_trial.sh flat left_side r15_left_roll_over` である
- CSV は `artifacts/logs/recovery_modes/r15_left_roll_over/recovery.csv` に保存した

### GIF

![左横倒しを右脚で押して仰向けへ戻った試行](../../artifacts/logs/recovery_modes/r15_left_roll_over/trial.gif)

### 次への施策

- 次の試行では、高さ 0.15 m の既知の失敗階段を歩き、自然転倒のあと復帰して再び歩き始める

## 試行 R16 階段世界の起動

### 背景

- 試行 R15 までで、平地の仰向けと右横倒しは直立と走行まで届いていた
- 開発者は、シナリオ2として高さ 0.15 m、踏面 0.30 m、上り 4 段と下り 4 段の階段を歩かせ、自然転倒のあと復帰させる必要があった

### 目的

- 開発者は、`jp_stair_matrix_h15_d30_n04` の上で歩行中に転倒し、復帰して再び歩き始めることを確認した

### 結論

- 試行は、MuJoCo が世界ファイルを開けず、コントローラが `robot_description` を待ち続けたため、起動異常として失敗した
- 歩行も転倒も復帰も始まらなかった

### 詳細

- 初期条件は、階段の接近平地で x を 1.0 m、roll を零、胴体高さを 0.5 m にした
- `ros2_ws/install` の世界ディレクトリに `jp_stair_matrix_h15_d30_n04.xml` が無く、xacro も無かった
- `MujocoSystemInterface` は、その xml が存在しないと記録してハードウェア初期化に失敗した
- 録画プロセスは mp4 を残さなかったため、GIF は作らなかった
- 実行コマンドは `bash scripts/trial/run_recovery_trial.sh stair auto r16_stair_fall` である
- コンソールは `artifacts/logs/recovery_modes/r16_stair_fall/console.log` に保存した

### GIF

- この試行の GIF は無い
- 録画ファイルが生成される前に、世界ファイルの読込が失敗した

### 次への施策

- 次の試行では、階段の xacro を install へリンクして、同じ階段を再実行する

## 試行 R17 階段での初期着座

### 背景

- 試行 R16 では、階段の xacro が install に無く、シミュレーションが世界を開けなかった
- 開発者は、xacro をリンクしたあと、同じ階段で歩行中の転倒と復帰を確認する必要があった

### 目的

- 開発者は、階段を歩いて自然転倒したあと、直立へ戻り再び前へ進むことを確認した

### 結論

- 試行は、歩行前の着座を転倒と判定して復帰し、直立と `RESUME` まで届いたが、移動距離は約 0.02 m で失敗した
- 階段上の自然転倒は起きていない

### 詳細

- 初期条件は、階段の接近平地で x を 1.0 m、roll を零、胴体高さを 0.5 m にした
- 記録開始時の胴体高さは約 0.080 m、`u` は約 0.995 で、分類はうつぶせだった
- 復帰は 1 回で、約 14.7 s に `RECOVERY` が始まり、約 17.5 s に直立と `RESUME` へ入った
- `local_plan` の時刻は -1 s のままで、歩行指令の通過まで進まなかった
- 地形メッシュ `jp_stair_matrix_h15_d30_n04.ply` は install に無く、`mjcf_to_grid_map_node` は空のメッシュを記録した
- 実行コマンドは `bash scripts/trial/run_recovery_trial.sh stair auto r17_stair_fall` である
- CSV は `artifacts/logs/recovery_modes/r17_stair_fall/recovery.csv` に保存した

### GIF

![階段の前で着座から直立まで戻った試行](../../artifacts/logs/recovery_modes/r17_stair_fall/trial.gif)

### 次への施策

- 次の階段試行では、起立が終わるまで自動復帰を止め、地形メッシュを読める状態で歩かせてから転倒判定を有効にする

## 試行 R18 左横倒しを仰向けへ畳む

### 背景

- 試行 R15 では、左横倒しの押しが背中側へ回り、直後の仰向け押しが腹側で止まりきらなかった
- 開発者は、背中が地面に着いたあと脚を畳んで静止させ、仰向けの復帰を同じ初期姿勢から始める必要があった

### 目的

- 開発者は、左横倒しから仰向けへ畳んだあと、仰向けの復帰で直立と 2 m の歩行へ届くことを確認した

### 結論

- 試行は、仰向けへの畳み込みまでは届いたが、横向きで押しが止まり `timeout` で失敗した
- 直立到達は無く、最終姿勢は仰向けだった

### 詳細

- 初期条件は、平地で roll を -1.5708 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 左横倒しの動作は、0.40 s の右脚伸展のあと、0.70 s で四脚の股関節を 1.40 rad、膝を -2.60 rad へ畳む
- 約 12.8 s に `u` が -1.000、高さが 0.058 m、角速度が 0.16 rad/s になり、背中が地面へ着いた
- 2 回目の復帰は仰向けの押しへ入り、一度左右を入れ替えたあと左横倒しの `u` 約 -0.25 で止まった
- 横向きの保持は、脚の実位置から目標へ補間をやり直したため、股関節 3.20 rad の指令が維持されなかった
- 約 42.8 s に `timeout` で `SAFETY` へ入った
- `state_log.csv` は残らなかったため、移動距離は未確認である
- 実行コマンドは `RECOVERY_DURATION_S=70 bash scripts/trial/run_recovery_trial.sh flat left_side r18_left_tuck_then_supine` である
- CSV は `artifacts/logs/recovery_modes/r18_left_tuck_then_supine/recovery.csv` に保存した

### GIF

![左横倒しを仰向けへ畳んだあと横向きで止まった試行](../../artifacts/logs/recovery_modes/r18_left_tuck_then_supine/trial.gif)

### 次への施策

- 次の試行では、横向きの保持中も押し切った関節目標を維持し、3 回まで復帰をやり直す

## 試行 R19 左横倒しの押しを維持する

### 背景

- 試行 R18 では、横向きの保持が脚の実位置から補間をやり直したため、股関節の押しが維持されなかった
- 開発者は、押し切った関節目標を保持し、止まっても 3 回までやり直す必要があった

### 目的

- 開発者は、左横倒しから直立へ戻り、平地を 2 m 以上歩くことを確認した

### 結論

- 試行は、直立と `RESUME` には届いたが、歩行へ渡した直後に胴体が沈み、3 回の上限で失敗した
- 最終姿勢は仰向けで、移動距離は約 -0.89 m だった

### 詳細

- 初期条件は、平地で roll を -1.5708 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 1 回目は背中へ畳み、2 回目の仰向け押しで約 15.9 s に直立へ戻り、約 17.6 s に `RESUME` へ入った
- `RESUME` 中の関節指令列は空で、約 18.7 s に高さが 0.315 m から 0.186 m へ落ちた
- 3 回目はうつぶせから再び直立へ戻ったが、`MONITOR` の約 1.5 s 後に横へ倒れて `attempt_limit` になった
- 実行コマンドは `RECOVERY_DURATION_S=80 bash scripts/trial/run_recovery_trial.sh flat left_side r19_left_hold_target` である
- CSV は `artifacts/logs/recovery_modes/r19_left_hold_target/recovery.csv` に保存した

### GIF

![左横倒しから直立へ戻ったあと歩行指令で沈んだ試行](../../artifacts/logs/recovery_modes/r19_left_hold_target/trial.gif)

### 次への施策

- 次の試行では、12 関節が揃った歩行指令が来るまで起立姿勢を維持し、`RESUME` 中の再転倒でも復帰をやり直す

## 試行 R20 横向き保持の打ち切り

### 背景

- 試行 R19 では、左横倒しから直立まで戻ったが、空の歩行指令で胴体が沈んだ
- 開発者は、歩行指令が揃うまで起立を維持する変更と同時に、横向き保持を 3 回で打ち切る変更も入れていた

### 目的

- 開発者は、左横倒しから直立へ戻り、平地を 2 m 以上歩くことを確認した

### 結論

- 試行は、仰向けの押しが左横倒しで止まり、3 回目を左横倒しの動作へ戻したため `attempt_limit` で失敗した
- 直立到達は無かった

### 詳細

- 初期条件は、平地で roll を -1.5708 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 2 回目の仰向け押しは、左横倒しの `u` 約 -0.55 で約 2.7 s 保持したあと、3 回目として左横倒しの動作をやり直した
- 3 回目は `u` 約 0.919 のうつぶせまで回ったが、左横倒しの動作には起立フレームが無く、高さは約 0.08 m のまま終わった
- 約 26.3 s に `attempt_limit` で `SAFETY` へ入った
- 実行コマンドは `RECOVERY_DURATION_S=80 bash scripts/trial/run_recovery_trial.sh flat left_side r20_left_wait_walk_cmd` である
- CSV は `artifacts/logs/recovery_modes/r20_left_wait_walk_cmd/recovery.csv` に保存した

### GIF

![仰向けの押しを途中で打ち切って左横倒しへ戻った試行](../../artifacts/logs/recovery_modes/r20_left_wait_walk_cmd/trial.gif)

### 次への施策

- 次の試行では、横向きの保持を打ち切らず、押し切った関節目標を維持したまま仰向けの起立まで進める

## 試行 R21 押しを維持した左横倒し

### 背景

- 試行 R19 では、左脚の股関節が 3.20 rad まで届いたときだけ仰向けから腹側へ回った
- 試行 R20 では、横向き保持の打ち切りが起立前に左横倒しの動作へ戻した
- 開発者は、押し切った目標を維持したまま、空の歩行指令では起立を手放さないようにする必要があった

### 目的

- 開発者は、左横倒しから直立へ戻り、平地を 2 m 以上歩くことを確認した

### 結論

- 試行は、左右を入れ替えた押しが右股関節 2.63 rad で止まり、`timeout` で失敗した
- 直立到達は無く、移動距離は約 -0.06 m だった

### 詳細

- 初期条件は、平地で roll を -1.5708 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 1 回目は背中へ畳み、2 回目は仰向けの押しの終わりでまだ仰向けだったため左右を入れ替えた
- 入れ替え後の指令は左股関節 2.40 rad、右股関節 3.20 rad だったが、右股関節の実角度は約 2.63 rad で止まった
- `u` は約 -0.54 から約 -0.32 までしか動かず、約 28 s の `recovery_timeout` で `SAFETY` へ入った
- 実行コマンドは `RECOVERY_DURATION_S=80 bash scripts/trial/run_recovery_trial.sh flat left_side r21_left_sustain_push` である
- CSV は `artifacts/logs/recovery_modes/r21_left_sustain_push/recovery.csv` に保存した

### GIF

![左右を入れ替えた押しが横向きで止まった試行](../../artifacts/logs/recovery_modes/r21_left_sustain_push/trial.gif)

### 次への施策

- 次の試行では、`u` が -0.75 を超えて背中が浮き始めた押しは左右を入れ替えず、同じ脚で押し続ける

## 試行 R22 浮いた押しを同じ脚で維持する

### 背景

- 試行 R21 では、左右を入れ替えた押しの右股関節が 2.63 rad で止まり、体が横向きから動かなかった
- 試行 R19 では、左股関節が 3.20 rad へ届いた瞬間に腹側へ回って直立した
- 開発者は、背中が浮き始めた押しを入れ替えずに維持する必要があった

### 目的

- 開発者は、左横倒しから直立へ戻り、平地を 2 m 以上歩くことを確認した

### 結論

- 試行は、左股関節 3.20 rad を維持しても `u` が約 -0.726 で止まり、`timeout` で失敗した
- 直立到達は無かった

### 詳細

- 初期条件は、平地で roll を -1.5708 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 2 回目の押しは左右を入れ替えず、左股関節の指令と実角度がともに 3.20 rad になった
- `u` は -1.000 から約 -0.726 まで上がり、その後約 24 s 動かなかった
- 約 42.3 s に `timeout` で `SAFETY` へ入り、最終姿勢は仰向けだった
- 実行コマンドは `RECOVERY_DURATION_S=80 bash scripts/trial/run_recovery_trial.sh flat left_side r22_left_keep_push` である
- CSV は `artifacts/logs/recovery_modes/r22_left_keep_push/recovery.csv` に保存した

### GIF

![同じ脚の押しが背中を少し浮かせたまま止まった試行](../../artifacts/logs/recovery_modes/r22_left_keep_push/trial.gif)

### 次への施策

- 次の試行では、止まった押しを一度股関節 2.2 rad まで戻してから、もう一度 3.20 rad へ打ち込む

## 試行 R23 止まった押しの再打ち込み

### 背景

- 試行 R22 では、左股関節 3.20 rad を維持しても `u` が約 -0.726 で止まった
- 開発者は、止まった押しを一度戻してから、もう一度打ち込んで回転を作り直す必要があった

### 目的

- 開発者は、左横倒しから直立へ戻り、平地を 2 m 以上歩くことを確認した

### 結論

- 試行は、再打ち込みのあとも仰向けのまま `timeout` で失敗した
- 直立到達は無く、移動距離は約 -0.05 m だった

### 詳細

- 初期条件は、平地で roll を -1.5708 rad、pitch と yaw を零、胴体高さを 0.35 m にした
- 2 回目の押しは左右を入れ替えず、左股関節は 3.20 rad に届いた
- `u` は約 -0.726 で止まり、再打ち込みのあと約 -0.852 まで戻った
- 試行回数は 2 で、`timeout` により最終モードは `SAFETY`、最終姿勢は仰向けだった
- 実行コマンドは `RECOVERY_DURATION_S=80 bash scripts/trial/run_recovery_trial.sh flat left_side r23_left_repunch` である
- CSV は `artifacts/logs/recovery_modes/r23_left_repunch/recovery.csv` に保存した

### GIF

![再打ち込みのあと仰向けのまま止まった試行](../../artifacts/logs/recovery_modes/r23_left_repunch/trial.gif)

### 次への施策

- 次の試行では、階段の接近平地で起立させてから自動復帰を有効にし、歩行中の自然転倒を待つ

## 試行 R24 階段を歩いてから転倒する

### 背景

- 試行 R17 では、歩行前の着座を転倒と判定し、階段上の自然転倒が起きなかった
- 試行 R16 では、階段の世界ファイルが install に無かった
- 開発者は、地形メッシュを読める状態で、起立したあと自動復帰を有効にして階段を歩かせる必要があった

### 目的

- 開発者は、高さ 0.15 m の階段を歩いて自然転倒したあと、直立へ戻り再び前へ進むことを確認した

### 結論

- 試行は、約 5.05 m 進んで階段を上ったが、転倒後の復帰が 3 回で尽きたため失敗した
- 転倒後の `RESUME` は無く、最終姿勢はうつぶせだった

### 詳細

- 初期条件は、階段の接近平地で x を 1.0 m、roll を零、胴体高さを 0.5 m にした
- 記録開始時は胴体高さ約 0.079 m のうつぶせで、約 60.3 s に直立へ戻ってから歩き始めた
- 最大の胴体高さは約 0.916 m で、時刻は約 100.7 s だった
- 約 109.4 s に姿勢が崩れ、約 110.3 s に仰向けへ落ち、約 112.1 s に 1 回目の復帰が始まった
- 1 回目は腹側まで回ったが、高さは約 0.13 m から上がらなかった
- 2 回目は左横倒し、3 回目はうつぶせの動作で、約 124.2 s に `attempt_limit` で `SAFETY` へ入った
- 地形メッシュは `jp_stair_matrix_h15_d30_n04.ply` を install へリンクして読ませた
- 実行コマンドは `bash scripts/trial/run_recovery_trial.sh stair auto r24_stair_walk_fall` である
- CSV は `artifacts/logs/recovery_modes/r24_stair_walk_fall/recovery.csv` と `artifacts/logs/recovery_modes/r24_stair_walk_fall/state_log.csv` に保存した

### GIF

![階段を上ったあと転倒して復帰が止まった試行](../../artifacts/logs/recovery_modes/r24_stair_walk_fall/trial.gif)

### 次への施策

- 次の試行では、階段の段の上で止まった腹ばいから、脚を段の外へ出してから起立させる

## 試行 R25 起立を維持して階段を歩く

### 背景

- 試行 R24 では、接近平地で約 60 s 腹ばいのあと歩き、転倒後の起立が高さ約 0.13 m で止まった
- 開発者は、歩行指令が来るまで起立姿勢を維持し、高さが上がらない腹ばいでは脚を伸ばし、腹側の横向きでは左横倒しの畳み込みを使わないようにする必要があった

### 目的

- 開発者は、高さ 0.15 m の階段を歩いて自然転倒したあと、直立へ戻り再び前へ進むことを確認した

### 結論

- 試行は、上りと下りを直立のまま歩き切ったため、転倒後の復帰は起きなかった
- スクリプトの成功判定は、`RESUME` が無いので失敗になっている

### 詳細

- 初期条件は、階段の接近平地で x を 1.0 m、roll を零、胴体高さを 0.5 m にした
- 記録開始の高さは約 0.079 m で、約 6.4 s に約 0.322 m へ立ち上がった
- 起動待ちの表示は起立前のままだったが、胴体は起立したあと `MONITOR` のまま歩き始めた
- 最大高さは約 0.933 m、終了位置は x 約 6.71 m、高さ約 0.309 m、roll 約 0.008 rad だった
- 復帰の開始回数は 0 で、`reach_joints` と腹ばいへのやり直しは使われなかった
- 移動距離は約 5.74 m だった
- 実行コマンドは `bash scripts/trial/run_recovery_trial.sh stair auto r25_stair_reach` である
- CSV は `artifacts/logs/recovery_modes/r25_stair_reach/recovery.csv` と `artifacts/logs/recovery_modes/r25_stair_reach/state_log.csv` に保存した

### GIF

![階段を上って下り、直立のまま歩き切った試行](../../artifacts/logs/recovery_modes/r25_stair_reach/trial.gif)

### 次への施策

- 次の試行では、同じ階段をもう一度歩き、自然転倒が起きたときに脚の伸ばしと腹ばいのやり直しが働くかを確認する

## 試行 R26 階段の下りで転倒する

### 背景

- 試行 R25 では、階段の上りと下りを直立のまま歩き、転倒後の復帰は起きなかった
- 開発者は、同じ階段をもう一度歩き、自然転倒のときに脚の伸ばしと腹ばいのやり直しが働くかを確認する必要があった

### 目的

- 開発者は、高さ 0.15 m の階段で自然転倒したあと、直立へ戻り再び前へ進むことを確認した

### 結論

- 試行は、下りで右横倒しから仰向けへ落ち、仰向けの押しが `u` 約 -0.73 で止まり `timeout` で失敗した
- 転倒後の `RESUME` は無く、最終姿勢は仰向けだった

### 詳細

- 初期条件は、試行 R25 と同じ接近平地である
- 最大高さは約 0.989 m で、終了位置は x 約 6.27 m、高さ約 0.155 m、roll 約 1.52 rad だった
- 約 125.3 s に右横倒しへ崩れ、約 128.0 s に右横倒しの復帰が始まった
- 約 128.2 s の角速度は約 5.7 rad/s で背中が地面へ向いたが、右横倒しの動作を続けて静止した
- 2 回目は静止した仰向けから押し、左股関節 3.20 rad で `u` 約 -0.727 のまま `recovery_timeout` に入った
- 段差用の `reach_joints` は、起立フレームまで進まなかったため使われなかった
- 移動距離は約 5.30 m だった
- 実行コマンドは `bash scripts/trial/run_recovery_trial.sh stair auto r26_stair_repeat` である
- CSV は `artifacts/logs/recovery_modes/r26_stair_repeat/recovery.csv` と `artifacts/logs/recovery_modes/r26_stair_repeat/state_log.csv` に保存した

### GIF

![階段の下りで仰向けへ落ち、押しが止まった試行](../../artifacts/logs/recovery_modes/r26_stair_repeat/trial.gif)

### 次への施策

- 次の試行では、左右の横倒しから背中が地面へ向いた瞬間に、回転が残っているうちへ仰向けの押しをつなぐ

## 試行 R27 回転中に仰向けの押しへつなぐ

### 背景

- 試行 R26 では、下りで背中が地面へ向いたときの角速度が約 5.7 rad/s だったが、右横倒しの動作を続けて静止してから押し始めた
- 開発者は、横倒しから仰向けになった瞬間に、残っている回転のまま仰向けの押しへつなぐ必要があった

### 目的

- 開発者は、階段で自然転倒したあと、直立へ戻り再び前へ進むことを確認した

### 結論

- 試行は、仰向けの押しへ角速度約 11 rad/s でつながったが、段の上の左横倒しで押しを保持したまま `timeout` になった
- 転倒後の `RESUME` は無く、最終姿勢は左横倒しだった

### 詳細

- 約 96 s に高さ約 0.57 m で姿勢が崩れ、約 99.0 s に右横倒しの復帰が始まった
- 約 99.1 s に動作が仰向けへ切り替わり、角速度は約 11.0 rad/s だった
- 約 99.9 s に左横倒し、高さ約 0.43 m で `side_push` の保持に入り、約 127 s まで高さが約 0.37 m のままだった
- 段の上では高さが `upright_height_min` の 0.22 m を超えているため、平地の回転保持が足場を探し続けた
- 移動距離は約 2.33 m、試行回数は 1、失敗理由は `timeout` だった
- 実行コマンドは `bash scripts/trial/run_recovery_trial.sh stair auto r27_stair_chain_push` である
- CSV は `artifacts/logs/recovery_modes/r27_stair_chain_push/recovery.csv` に保存した

### GIF

![段の上で仰向けの押しへつながったあと横向きで止まった試行](../../artifacts/logs/recovery_modes/r27_stair_chain_push/trial.gif)

### 次への施策

- 次の試行では、高さが 0.22 m 以上の横向きでは回転の保持をやめ、起立フレームへ進む

## 試行 R28 段の低い位置で仰向けから起きる

### 背景

- 試行 R27 では、段の上の左横倒しで高さが約 0.43 m のまま `side_push` を保持し、`timeout` になった
- 開発者は、高さが 0.22 m 以上の横向きでは回転の保持をやめ、`low_support` と `stand` へ進める必要があった

### 目的

- 開発者は、高さ 0.15 m の階段で自然転倒したあと、直立へ戻り再び前へ進むことを確認した

### 結論

- 試行は、階段を上ったあと落ち、仰向けの一連の動作で 1 回直立へ戻り、`RESUME` から `MONITOR` の歩行へ戻った
- 落ちた位置の高さは約 0.08 m で、0.22 m 以上の横向き保持を外す分岐は使われなかった
- 復帰後の yaw は約 1.80 rad で、歩行は階段の前方ではなく横へ進んだ

### 詳細

- 初期条件は、階段の接近平地で x を 1.0 m、roll を零、胴体高さを 0.5 m にした
- 起動待ちの表示は起立前のままだったが、約 7 s に高さ約 0.32 m へ立ち上がってから歩き始めた
- 最大の前方位置は x 約 4.06 m、最大高さは約 0.999 m で、時刻は約 98.4 s だった
- そのあと後方へ落ち、約 103.3 s に x 約 2.66 m、高さ約 0.077 m の腹ばいまで下がった
- 復帰は 1 回で、仰向けの `open_gap` と `side_push` が右横倒しを経由して腹ばいへ回り、`stand` の股関節指令は 0.80 rad だった
- `side_push` が終わるときの姿勢は腹ばいで高さ約 0.08 m だったため、高さ 0.22 m 以上で保持をやめる分岐は通らなかった
- `reach_joints` の股関節 0.35 rad は、`stand` の終了時に高さが 0.22 m を超えていたため使われなかった
- 約 105 s に高さ約 0.308 m、yaw 約 1.80 rad で直立し、`RESUME` のあと最終モードは `MONITOR`、最終姿勢は直立だった
- 終了位置は x 約 2.46 m、y 約 0.70 m、高さ約 0.308 m、roll 約 0.01 rad だった
- 移動距離は約 1.49 m で、スクリプトの成功判定は成功だった
- 実行コマンドは `bash scripts/trial/run_recovery_trial.sh stair auto r28_stair_step_stand` である
- CSV は `artifacts/logs/recovery_modes/r28_stair_step_stand/recovery.csv` と `artifacts/logs/recovery_modes/r28_stair_step_stand/state_log.csv` に保存した

### GIF

![階段を上ったあと落ち、仰向けから起きて歩き直した試行](../../artifacts/logs/recovery_modes/r28_stair_step_stand/trial.gif)

### 次への施策

- 次の試行では、平地の左横倒しから、背中が地面へ向いた瞬間に仰向けの押しへつなぎ、直立のあと 2 m 以上歩くかを確認する

## 試行 R29 左横倒しの指定が記録前に仰向けへ落ちる

### 背景

- 試行 R22 と試行 R23 では、静止した左横倒しから仰向けへ回したあとの押しが `u` 約 -0.73 で止まった
- 開発者は、背中が地面へ向いた瞬間に、残っている回転のまま仰向けの押しへつなぐ必要があった

### 目的

- 開発者は、平地の左横倒しから直立へ戻り、2 m 以上歩くことを確認した

### 結論

- 試行は、記録開始の時点で roll が -3.14 rad、`u` が -1 の仰向けだったため、左横倒しの動作は始まらなかった
- 仰向けの一連の動作で 1 回直立へ戻り、約 6.26 m 歩いて最終モードは `MONITOR` だった

### 詳細

- 起動時の home キーフレームは roll -1.57 rad、高さ 0.35 m だった
- 状態記録の先頭は、x 約 0.00 m、y 約 0.15 m、高さ約 0.058 m、roll 約 -3.14 rad で、角速度はほぼ零だった
- 復帰開始の姿勢は仰向けで、`left_side` から `supine` への切替ログは無かった
- `open_gap` と `side_push` が右横倒しを経由して腹ばいへ回り、約 2.7 s で直立になった
- `RESUME` のあと最終姿勢は直立、試行回数は 1、移動距離は約 6.26 m、横方向は約 -0.70 m だった
- スクリプトの成功判定は成功だが、左横倒しの初期条件は満たしていない
- 実行コマンドは `RECOVERY_DURATION_S=80 bash scripts/trial/run_recovery_trial.sh flat left_side r29_left_chain` である
- CSV は `artifacts/logs/recovery_modes/r29_left_chain/recovery.csv` と `artifacts/logs/recovery_modes/r29_left_chain/state_log.csv` に保存した

### GIF

![指定は左横倒しだったが、記録開始時には仰向けから起きて歩いた試行](../../artifacts/logs/recovery_modes/r29_left_chain/trial.gif)

### 次への施策

- 次の試行では、同じ左横倒しの初期姿勢をもう一度使い、記録開始の roll が -1.57 rad 付近であることを確認してから押しのつなぎを見る

## 試行 R30 左横倒しから仰向けへ切り替える

### 背景

- 試行 R29 では、左横倒しを指定したのに記録開始時には仰向けへ落ち着いており、左横倒しの動作は始まらなかった
- 開発者は、記録開始の roll が -1.57 rad 付近の左横倒しから、背中が地面へ向いた瞬間に仰向けの押しへつなぐ必要があった

### 目的

- 開発者は、平地の左横倒しから直立へ戻り、2 m 以上歩くことを確認した

### 結論

- 試行は、左横倒しから約 0.2 s で仰向けへ切り替わったが、そのあと両股関節が 2.60 rad へ戻り、押しが `u` 約 -0.726 で止まって `timeout` になった
- 直立到達は無く、移動距離は約 0.13 m だった

### 詳細

- 記録開始は roll 約 -1.62 rad、高さ約 0.156 m、`u` 約 -0.051 の左横倒しだった
- 約 9.91 s に左横倒しの復帰が始まり、約 10.11 s に仰向けへ切り替わり、角速度は約 2.80 rad/s だった
- 切替の直前は右股関節の指令が約 2.85 rad で、左横倒しの `roll_to_back` が右脚を伸ばしていた
- 切替後の `open_gap` は両股関節を 2.60 rad へ戻し、そのあと左股関節 3.20 rad の保持で `u` が約 -0.726 のまま止まった
- 約 12.3 s に股関節指令が一度約 2.33 rad まで下がり、すぐ 3.20 rad へ戻った
- 試行回数は 1、約 37.9 s に `timeout` で `SAFETY` へ入り、最終姿勢は仰向けだった
- 実行コマンドは `RECOVERY_DURATION_S=80 bash scripts/trial/run_recovery_trial.sh flat left_side r30_left_chain` である
- CSV は `artifacts/logs/recovery_modes/r30_left_chain/recovery.csv` と `artifacts/logs/recovery_modes/r30_left_chain/state_log.csv` に保存した

### GIF

![左横倒しから仰向けへつながったあと押しが止まった試行](../../artifacts/logs/recovery_modes/r30_left_chain/trial.gif)

### 次への施策

- 次の試行では、左横倒しから仰向けへ切り替えるとき `open_gap` を飛ばし、すでに伸ばしている右脚の `side_push` を続ける

## 試行 R31 左横倒しから右脚の押しを続ける

### 背景

- 試行 R30 では、左横倒しから仰向けへ切り替えた直後の `open_gap` が両股関節を 2.60 rad へ戻し、押しが `u` 約 -0.726 で止まった
- 開発者は、`open_gap` を飛ばし、`roll_to_back` で伸ばしていた右脚の `side_push` を続ける必要があった

### 目的

- 開発者は、平地の左横倒しから直立へ戻り、2 m 以上歩くことを確認した

### 結論

- 試行は、左横倒しから右股関節 3.20 rad の押しで腹ばいへ回り、1 回で直立へ戻って `MONITOR` の歩行まで進んだ
- `state_log.csv` が残らなかったため、移動距離の数値は未確認である
- 固定カメラの映像では、約 43 s に直立で歩き、約 88 s には画角の外へ出ていた

### 詳細

- 記録開始は roll 約 -1.62 rad、高さ約 0.156 m、`u` 約 -0.047 の左横倒しだった
- 約 11.05 s に左横倒しの復帰が始まり、約 11.25 s に `side_push` へ切り替わり、角速度は約 2.86 rad/s だった
- 切替時のキーフレームは 1 で、`open_gap` は使われなかった
- 右股関節の指令は約 2.98 rad から 3.20 rad へ進み、左股関節は約 2.38 rad だった
- 約 11.91 s に腹ばい、約 12.62 s に直立、約 14.21 s に `RESUME`、約 16.21 s に `MONITOR` へ入った
- 終了時刻は約 100.5 s で、最終モードは `MONITOR`、最終姿勢は直立、高さは約 0.309 m、`u` は約 1.000 だった
- 試行回数は 1 で、失敗理由は空だった
- スクリプトの成功判定は、移動距離が数値でないため失敗になっている
- 実行コマンドは `RECOVERY_DURATION_S=80 bash scripts/trial/run_recovery_trial.sh flat left_side r31_left_keep_right_push` である
- CSV は `artifacts/logs/recovery_modes/r31_left_keep_right_push/recovery.csv` に保存した

### GIF

![左横倒しから右脚で回して起き、歩き出した試行](../../artifacts/logs/recovery_modes/r31_left_keep_right_push/trial.gif)

### 次への施策

- 次の試行では、同じ左横倒しの復帰で `state_log.csv` を残し、移動距離が 2 m 以上かを数値で確認する

## 試行 R32 左横倒しの歩行距離を記録する

### 背景

- 試行 R31 では、左横倒しから右脚の押しで直立へ戻り `MONITOR` まで進んだが、`state_log.csv` が残らず移動距離が数値で確認できなかった
- 開発者は、状態記録が歩行の終了前に書き出されるようにし、同じ左横倒しでもう一度距離を測る必要があった

### 目的

- 開発者は、平地の左横倒しから直立へ戻り、2 m 以上歩くことを確認した

### 結論

- 試行は、左横倒しから右脚の押しで 1 回直立へ戻り、約 6.55 m 歩いて最終モードは `MONITOR` だった
- スクリプトの成功判定は成功だった

### 詳細

- 記録開始は roll 約 -1.62 rad、高さ約 0.156 m、`u` 約 -0.046 の左横倒しだった
- 約 8.31 s に左横倒しの復帰が始まり、約 8.51 s に `side_push` へ切り替わり、角速度は約 2.98 rad/s だった
- 約 9.16 s に腹ばい、約 9.86 s に直立、約 11.47 s に `RESUME`、約 13.48 s に `MONITOR` へ入った
- 終了位置は x 約 6.55 m、y 約 -1.76 m、高さ約 0.308 m、roll 約 0.002 rad、yaw 約 -0.24 rad だった
- 試行回数は 1、最終姿勢は直立、失敗理由は空だった
- 状態記録は、速度指令の終了後に記録プロセスの終了を待ってからシミュレーションを止めるようにして残した
- 実行コマンドは `RECOVERY_DURATION_S=80 bash scripts/trial/run_recovery_trial.sh flat left_side r32_left_distance` である
- CSV は `artifacts/logs/recovery_modes/r32_left_distance/recovery.csv` と `artifacts/logs/recovery_modes/r32_left_distance/state_log.csv` に保存した

### GIF

![左横倒しから起きて約 6.55 m 歩いた試行](../../artifacts/logs/recovery_modes/r32_left_distance/trial.gif)

### 次への施策

- 次の試行では、階段で落ちたあとの yaw が前方からずれたまま横へ歩く点を直し、段の上の横向きでも起立フレームへ進むかを確認する



