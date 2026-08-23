#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
学習ノート docs/legged_control の章立てを、そのままスライドにする。
新しい物語や層分けは作らない。速習は各章の結論と必須の表/式、
じっくりは同じ見出しを節まで残す。
"""

from pathlib import Path

from _kit import GOLD, MUTED, NAVY, TEAL, TERR, Deck

OUT = Path(__file__).resolve().parent
META = "正本: docs/legged_control  /  コード: external/legged_control@a7f381c  /  既定ロボット: a1"


def fig_main(d, s):
    """00 §3 / ノートの概念図。箱は①–⑦とGaitを省略しない。"""
    d.node(s, 0.35, 1.15, 5.85, 1.15, "① 指令", "cmd_vel / goal", "速さ Twist 4  または  位置 Pose 6", TEAL)
    d.node(s, 0.35, 2.45, 5.85, 1.15, "② 目標軌道", "TargetTrajectories", "状態 24 × 2点（今と 1.0 s 先）。入力列はゼロ", TEAL)
    d.node(s, 0.35, 3.75, 5.85, 1.20, "③ 状態推定  500 Hz", "Kalman → rbd 36 → x 24", "IMU姿勢と関節はそのまま。並進位置・速さだけ推定", GOLD)
    d.node(s, 6.90, 1.15, 6.05, 0.95, "Gait 指令（並列）", "ModeSchedule", "接地の時刻表。胴体目標とは別蛇口", TERR)
    d.node(s, 6.90, 2.25, 6.05, 1.25, "④ NMPC  100 Hz", "初期状態 x 24 で未来 1 s", "出るのは今の 1点: x* 24, u* 24, mode", NAVY)
    d.node(s, 6.90, 3.65, 6.05, 1.15, "⑤ WBC  500 Hz", "tau = 解の末尾 12", "③ の rbd 36 も読む。内部は 42 個", TERR)
    d.node(s, 6.90, 4.90, 2.90, 1.10, "⑥ ハイブリッド", "q*, dq*, Kp=0", "Kd=3, ff=tau", MUTED)
    d.node(s, 10.00, 4.90, 2.95, 1.10, "⑦ モータ / Gazebo", "12 関節を回す", "センサは ③ へ戻る", MUTED)
    d.arr(s, 2.90, 2.25, "↓")
    d.arr(s, 2.90, 3.55, "↓")
    d.arr(s, 6.15, 2.70, "→")
    d.cap(s, 5.95, 3.00, 1.0, "24×2")
    d.arr(s, 6.15, 4.05, "→")
    d.cap(s, 5.90, 4.35, 1.1, "x 24")
    d.arr(s, 9.50, 2.05, "↓")
    d.cap(s, 9.70, 2.00, 2.2, "ModeSchedule")
    d.arr(s, 9.50, 3.45, "↓")
    d.cap(s, 9.55, 3.42, 3.2, "x* 24  u* 24  mode")
    d.arr(s, 9.50, 4.72, "↓")


def fig_cmd(d, s):
    """02 指令経路。ノートのイベント図と同じ箱。"""
    d.node(s, 0.35, 1.15, 6.00, 1.05, "人", "ゲームパッド / キーボード / RViz", "速さ 4  または  ゴール 6", TEAL)
    d.node(s, 0.35, 2.40, 2.85, 1.20, "① /cmd_vel", "Twist  4", "vx, vy, vz, 旋回速さ", TEAL)
    d.node(s, 3.40, 2.40, 2.95, 1.20, "①' goal", "Pose → 内部 6", "xyzw 位置 + ZYX 向き", TEAL)
    d.node(s, 0.35, 3.80, 6.00, 1.20, "② TargetTrajectoriesPublisher", "今と 1 s 先の 2 点", "t:2   x:(24,)×2   u:(24,)×2", TEAL)
    d.node(s, 7.00, 1.15, 5.95, 1.05, "人（別入力）", "端末に gait 名", "例 trot: 2 mode / 0.6 s", TERR)
    d.node(s, 7.00, 2.40, 5.95, 1.20, "GaitReceiver / GaitSchedule", "ModeSequenceTemplate", "接地の順番 + switchingTimes", TERR)
    d.node(s, 7.00, 3.80, 5.95, 1.20, "ModeSchedule", "eventTimes + mode 列", "④ の接地制約がこれを読む", TERR)
    d.node(s, 3.20, 5.15, 6.90, 0.95, "④ SqpMpc + MPC_MRT", "100 Hz スレッド。② の 2 点と ModeSchedule を両方受ける", "", NAVY)
    d.arr(s, 2.90, 2.18, "↓")
    d.arr(s, 4.40, 3.58, "↓")
    d.arr(s, 9.60, 2.18, "↓")
    d.arr(s, 9.60, 3.58, "↓")
    d.arr(s, 6.20, 4.95, "↓")
    d.arr(s, 9.60, 4.95, "↓")


def fig_loop(d, s):
    """02 500 Hz 閉ループ。ノートの箱を横一列＋戻り。"""
    d.node(s, 0.30, 1.15, 2.40, 1.70, "⑦ センサ", "関節 q,dq  各12", "IMU quat4 ω3 a3\n接地 bool 4", MUTED)
    d.node(s, 2.95, 1.15, 2.50, 1.70, "③ Kalman", "rbdState (36,)", "→ x (24,) と mode\n姿勢・関節は生", GOLD)
    d.node(s, 5.70, 1.15, 2.55, 1.70, "④ 今の 1 点", "evaluatePolicy", "x* 24  u* 24\nplannedMode", NAVY)
    d.node(s, 8.50, 1.15, 2.30, 1.70, "⑤ WBC", "内部 42", "出すのは τ 12\n③ の 36 も使う", TERR)
    d.node(s, 11.05, 1.15, 1.95, 1.70, "⑥⑦ 指令", "setCommand", "q* dq* Kp=0\nKd=3 ff=τ", MUTED)
    d.arr(s, 2.62, 1.80)
    d.arr(s, 5.38, 1.80)
    d.arr(s, 8.18, 1.80)
    d.arr(s, 10.72, 1.80)
    d.hline(s, 0.30, 3.05, 12.70, GOLD)
    d.cap(s, 0.35, 2.95, 12.5, "戻り: ⑦ が次周期の q, dq, IMU, 接地を読む。④ は observation (24+24) を ② へも出す")
    d.node(s, 0.30, 3.40, 4.10, 1.35, "③ → ⑤ のもう一本", "rbd 36 を WBC 計測へ", "NMPC用 24 とは別形", GOLD)
    d.node(s, 4.60, 3.40, 4.20, 1.35, "⑥ の 5 つ × 12 関節", "q*  dq*  Kp  Kd  ff", "全体 60 個。Kp は 0", TEAL)
    d.node(s, 9.00, 3.40, 3.95, 1.35, "植物", "浮動ベース 6 + 関節 12", "制御が書くのは関節 12 だけ", TERR)
    d.arr(s, 4.30, 3.90)
    d.arr(s, 8.70, 3.90)


def fig_clocks(d, s):
    """01/02 の 2 時計。ノート §4 と §5。"""
    d.node(s, 0.35, 1.15, 6.20, 4.90, "速い時計  500 Hz（2 ms）", "⑦ read → ③ → ④の1点 → ⑤ → ⑥ → ⑦ write", "", TERR)
    d.node(s, 6.80, 1.15, 6.15, 4.90, "遅い時計  100 Hz（10 ms）", "advanceMpc() が未来 1 s の束を更新", "", NAVY)
    d.cap(s, 0.55, 2.15, 5.8, "1  関節・IMU・接地を読む")
    d.cap(s, 0.55, 2.50, 5.8, "2  Kalman で rbd 36 と x 24")
    d.cap(s, 0.55, 2.85, 5.8, "3  最新 policy を t, x で補間")
    d.cap(s, 0.55, 3.20, 5.8, "4  WBC。Jacobian は毎回実測で作り直す")
    d.cap(s, 0.55, 3.55, 5.8, "5  τ12 + q*12 + dq*12 を書く")
    d.cap(s, 0.55, 3.90, 5.8, "NMPC が止まっても前回の束を使う")
    d.cap(s, 7.00, 2.15, 5.7, "観測 x 24 を初期値にする")
    d.cap(s, 7.00, 2.50, 5.7, "参照は ② の 24×2")
    d.cap(s, 7.00, 2.85, 5.7, "ModeSchedule で接地を切る")
    d.cap(s, 7.00, 3.20, 5.7, "SQP 1 回。約 67 点")
    d.cap(s, 7.00, 3.55, 5.7, "できた policy を共有する")
    d.cap(s, 7.00, 3.90, 5.7, "500 Hz 側が読む。4 回は同じ束")
    d.cap(s, 0.55, 4.50, 12.0, "切れ目: 層を増やさない。遅い束の「今」だけが速い時計へ渡る。")


def fig_vecs(d, s):
    """02 の 4 ベクトルを帯で見せる。"""
    d.vec_bar(s, 0.40, 1.20, 12.5, 0.85, "x  24", [
        ("勢い v_com  3", 3, TEAL),
        ("勢い L/m  3", 3, TEAL),
        ("位置 p_b  3", 3, NAVY),
        ("向き ψθφ  3", 3, NAVY),
        ("関節 q_j  12", 12, TERR),
    ])
    d.vec_bar(s, 0.40, 2.20, 12.5, 0.85, "u  24", [
        ("地面反力 f_c  12（トルクは無い）", 12, GOLD),
        ("関節速さ v_j  12", 12, TERR),
    ])
    d.vec_bar(s, 0.40, 3.20, 12.5, 0.85, "rbd  36", [
        ("ZYX 3", 3, NAVY),
        ("位置 3", 3, NAVY),
        ("関節角 12", 12, TERR),
        ("ω 3", 3, TEAL),
        ("並進速さ 3", 3, TEAL),
        ("関節速さ 12", 12, GOLD),
    ])
    d.vec_bar(s, 0.40, 4.20, 12.5, 0.85, "wbc  42", [
        ("加速度 q̈  18", 18, NAVY),
        ("地面反力  12", 12, GOLD),
        ("トルク τ  12  ← 外へ出すのはここだけ", 12, TERR),
    ])
    d.cap(s, 0.40, 5.20, 12.5, "x は NMPC 用。rbd は推定の生。wbc は⑤の内部。同じ「今」でも形が違う。")


def fig_gait(d, s):
    """05 trot の時間割。"""
    d.node(s, 0.35, 1.15, 6.20, 1.40, "0.0 – 0.3 s    mode LF_RH", "左前 + 右後 が接地", "右前と左後は空中（力 0）", TEAL)
    d.node(s, 6.80, 1.15, 6.15, 1.40, "0.3 – 0.6 s    mode RF_LH", "右前 + 左後 が接地", "左前と右後は空中", NAVY)
    d.hline(s, 0.35, 2.75, 12.60, TEAL)
    d.cap(s, 0.35, 2.70, 3.0, "0.0 s")
    d.cap(s, 6.50, 2.70, 3.0, "0.3 s  切替")
    d.cap(s, 11.4, 2.70, 1.5, "0.6 s")
    d.node(s, 0.35, 3.20, 12.60, 1.50, "NMPC は mode を選ばない", "GaitReceiver が ModeSchedule を渡す。初期は STANCE（四脚接地）だけ", "plannedMode（計画）が WBC の接地旗。推定の observation.mode とは別", TERR)
    d.node(s, 0.35, 4.85, 12.60, 1.20, "README", "THE GAIT AND THE GOAL ARE COMPLETELY DIFFERENT AND SEPARATED", "①② の胴体目標と、この時間割は別入力", GOLD)


def fig_twopoints(d, s):
    """03 の 2 点軌道。ノート §3.6 の例。"""
    d.node(s, 0.35, 1.15, 5.70, 2.35, "点 0    t = 今", "x0（24 個）", "速さ (0.5, 0, 0)\n位置 (0, 0, 0.3)\n向き 0、関節は立位", TEAL)
    d.node(s, 7.30, 1.15, 5.65, 2.35, "点 1    t = 今 + 1 s", "x1（24 個）", "速さ (0.5, 0, 0) 同じ\n位置 (0.5, 0, 0.3)\n向き・関節は同じ", NAVY)
    d.arr(s, 6.15, 2.10)
    d.cap(s, 5.85, 2.40, 1.5, "直線")
    d.node(s, 0.35, 3.70, 12.60, 2.30, "NMPC が見る x_ref(t)", "あいだは線形補間。速さ欄が同じなので参照速さは一定。位置だけが進む", "密な足軌道は無い。入力 24×2 はゼロ（コスト側が体重分担に置き換える）。流れる量は 98 個", GOLD)


def _00_intro(d: Deck, deep: bool):
    d.section_slide("00", "legged_control 理論・コード学習ガイド", "ノート 00_README.md と同じ見出し。略語は各ページ下部の※で説明する。")

    s = d.content("00  §1 目的", "このノートが対象にする閉ループ")
    d.bullets(
        s,
        [
            "qiayuanliao/legged_control を、理論とコードの両面から読むための学習ノートである。",
            "対象は、ユーザー速度指令と Gait 指令から、目標軌道、状態推定、Centroidal NMPC、WBC、関節ハイブリッド指令、Gazebo / 実機モータまでの閉ループ全体である。",
            "Gait: 足の接地時間割（stance / trot など）。NMPC: 非線形モデル予測制御。WBC: 全身制御（今の瞬間の二次計画）。",
            "Centroidal: 重心まわりに縮約した運動量。ハイブリッド指令: 目標角・目標速さ・位置ゲイン・速度ゲイン・トルクの5つ。",
        ],
        top=1.2,
        size=16,
    )
    d.fn(s, "※ 閉ループ: 指令→計算→モータ→センサ→また計算、の一周。  ※ Gazebo: 物理シミュレータ。実機の代わりに同じ指令インタフェースで動かす。  ※ stance: 四脚接地して立つ歩容。trot: 対角2脚ずつ接地して歩く歩容。")

    s = d.content("00  §2 対象コード", "照合したコミットと、この workspace に無いもの")
    d.table(
        s,
        [
            ["項目", "値"],
            ["legged_control", "https://github.com/qiayuanliao/legged_control"],
            ["照合コミット", "a7f381c0367e98e31c01336e678eef47e304d40d（2025-02-13, master）"],
            ["既定ロボット", "ROBOT_TYPE=a1"],
            ["この repo 外", "OCS2、pinocchio、ros-control、Unitree SDK"],
            ["未照合（OCS2側）", "LeggedRobotDynamicsAD、GaitReceiver、GaitSchedule 内部"],
            ["作者注記", "新規開発停止。後継は legged_perceptive"],
        ],
        0.35,
        1.25,
        12.6,
        5.05,
        col_w=[3.2, 9.4],
        font=14,
    )
    d.fn(s, "※ OCS2: ETHの最適制御ライブラリ。NMPCの本体。このworkspaceにソースは無い。  ※ pinocchio: ロボットの質量・運動学を計算するライブラリ。  ※ ros-control: ROSの制御器プラグイン枠。  ※ SDK: メーカ提供の実機通信一式。")

    s = d.content("00  §3 最初に理解する一本の経路", "概念図。型・単位・frame の正本は 02")
    fig_main(d, s)
    d.fn(s, "※ 左列①②③は指令と推定。右列④⑤⑥⑦は計画と実行。Gaitは右上から④へ入る別経路。⑦のセンサは③へ戻る。")

    s = d.content("00  §3 続き", "Gait は胴体目標軌道と独立である")
    fig_gait(d, s)
    d.fn(s, "※ ModeSchedule: いつ、どの足が地面についてよいかの時刻表。  ※ 前ページの右上「Gait指令」がこの時間割を④へ渡す。①②は触らない。")

    s = d.content("00  §4 推奨学習順序", "スライドもこの順。飛ばさない")
    d.table(
        s,
        [
            ["章", "ファイル", "中身"],
            ["01", "01_Packages_and_Control_Loop.md", "パッケージ、起動プロセス、500 Hz / 100 Hz、update() 順"],
            ["02", "02_System_Architecture_and_Dataflow.md", "Q1。境界の型・単位・配列サイズ"],
            ["03", "03_User_Command_and_Reference.md", "Q2 ①② 指令と2点軌道"],
            ["04", "04_State_Estimation.md", "Q2 ③ 線形Kalman"],
            ["05", "05_NMPC.md", "Q2 ④ と Gait 結合"],
            ["06", "06_WBC.md", "Q2 ⑤ WeightedWbc"],
            ["07", "07_Joint_Control_and_Hardware.md", "Q2 ⑥⑦ ハイブリッド指令とHW"],
        ],
        0.3,
        1.25,
        12.7,
        5.05,
        col_w=[1.0, 5.2, 6.5],
        font=13,
    )
    d.fn(s, "※ Q1/Q2: 会話で聞いた整理の番号。Q1=データの流れとサイズ、Q2=各箱の背景・式。  ※ Kalman: センサのノイズをならして位置・速さを推定するフィルタ。  ※ WeightedWbc: 既定の全身制御。一つの二次計画で重み付きに解く。")

    s = d.content("00  §5 記述上の区別", "各章で混同しない4区分")
    d.cards(
        s,
        [
            ("実装事実", ["現行コードに存在する処理。"], TEAL),
            ("理論", ["実装を説明する数式。OCS2本体は「OCS2側」と書く。"], NAVY),
            ("推奨改善", ["現行コードには無い、より明示的にする案。"], TERR),
            ("未実装 / 未確認", ["標準経路に無い機能、または OCS2 ソース未照合。"], MUTED),
        ],
        top=1.25,
        height=4.95,
    )
    d.fn(s, "※ OCS2側: 式や関数がOCS2の中にあり、このリポジトリのファイルでは中身を追えない、という意味。推測で断定しない。")

    s = d.content("00  §6 座標系と記号", "以降の式はこの定義を使う")
    d.table(
        s,
        [
            ["記号", "意味（このノートでの使い方）"],
            ["(W) / (B)", "世界座標（地図/odom）と、胴体に固定した座標"],
            ["ZYX Euler", "向きの3角。ψ=ヨー（旋回）、θ=ピッチ（前後傾）、φ=ロール（左右傾）"],
            ["関節順（制御）", "LF左前, LH左後, RF右前, RH右後。各脚 HAA股開閉, HFE股前後, KFE膝"],
            ["接触添字 i", "4本の足の番号。OCS2コメントは LF, RF, LH, RH の順のことがある"],
            ["x ∈ R^24", "NMPCが見る「今の形」。勢い6 + 位置向き6 + 関節12"],
            ["u ∈ R^24", "NMPCが決める入力。地面反力12 + 関節速度12。トルクは入らない"],
            ["x_rbd ∈ R^36", "推定が出す剛体状態。向き・位置・関節と、それぞれの速さ"],
            ["x_wbc ∈ R^42", "WBC内部の未知数。加速度18 + 地面反力12 + トルク12"],
        ],
        0.35,
        1.20,
        12.6,
        5.10,
        col_w=[3.0, 9.6],
        font=13,
    )
    d.fn(s, "※ odom: ロボットが積算した地図座標。  ※ ∈ R^n: 実数がn個並んだベクトル。  ※ 地面反力: 足が地面を押す力（英語GRF）。")

    s = d.content("00  §7 重要な結論", "ノート本文の箇条書きをそのまま置く")
    d.bullets(
        s,
        [
            "標準入力は目的地ではなく /cmd_vel の胴体速度である。/move_base_simple/goal は別経路。",
            "目標軌道は未来全体の密な軌道ではなく、現在と timeHorizon=1.0 s 先の 2点 である。",
            "NMPC状態は「胴体12 + 関節12」ではなく、正規化centroidal運動量6 + 胴体姿勢6 + 関節12である。",
            "NMPCは100 Hzスレッド、WBCと関節指令は500 Hz。WBCは最新policyを現在時刻で評価した1点だけを使う。",
            "既定WBCは階層QPではなく WeightedWbc（単一QP）。HierarchicalWbc は実装されているが未配線。",
            "モータ指令は τ_WBC を feedforward、位置ゲイン Kp=0、速度ゲイン Kd=3 である。",
            "Gait位相はNMPCが選ばない。legged_robot_gait_command が ModeSchedule として与える。",
        ],
        top=1.2,
        size=15,
    )
    d.fn(s, "※ /cmd_vel: 前後・横・上下・旋回の速さ4つを運ぶROSトピック。  ※ timeHorizon: 予測する未来の長さ（1秒）。  ※ policy: NMPCが今持っている未来の解の束。  ※ QP: 二次計画。等式・不等式の下で二次の罰を最小化。  ※ feedforward: 測ったズレではなく、計算済みの力をそのまま足すこと。")
    if deep:
        s = d.content("00  §8 Cursor運用", "ノートの運用ルール")
        d.bullets(
            s,
            [
                "最初に 00、02、01 を読み、その後に調査対象の章だけを足す。",
                "コードとノートが食い違ったら、コミットを記録してノートを更新する。",
                "変数の完全一覧は appendices/A_Variable_Dictionary.md。",
            ],
            top=1.2,
            size=17,
        )
        d.fn(s, "※ 変数辞書: 記号・単位・配列位置を一覧した付録。式のたびにここへ戻ってよい。")


def _01(d: Deck, deep: bool):
    d.section_slide("01", "パッケージと制御ループ", "01_Packages_and_Control_Loop.md。どの箱が何のソフトで、何Hzで動くか。")

    s = d.content("01  §1 結論", "中核と2つの周期")
    d.bullets(
        s,
        [
            "legged_control は ros-control の LeggedController を中核に、指令生成ノード、OCS2 NMPC、線形Kalman推定、WBC、ハイブリッド関節指令を組み合わせる。",
            "実機は 500 Hz のハードウェアループ、NMPCは別スレッド 100 Hz である。",
            "パッケージの役割と周期は本章が正本。境界データの型は 02。",
        ],
        top=1.2,
        size=17,
    )
    d.fn(s, "※ ノード: ROSで動く1プロセス。  ※ 線形Kalman: 加速度で予測し、接地足の位置で直す推定。姿勢はIMUを信じる。  ※ 別スレッド: 同じプログラム内の、もう一つの時計。NMPCだけ遅く回す。")

    s = d.content("01  §2 パッケージ対応", "このリポジトリでの主要ファイル")
    d.table(
        s,
        [
            ["パッケージ", "役割", "主要ファイル"],
            ["legged_controllers", "制御器プラグイン、目標軌道ノード", "LeggedController.cpp, TargetTrajectoriesPublisher.cpp"],
            ["legged_interface", "OCS2の最適制御問題を組み立てる", "LeggedInterface.cpp、制約・コスト"],
            ["legged_estimation", "状態推定", "LinearKalmanFilter.cpp, StateEstimateBase.cpp"],
            ["legged_wbc", "全身の二次計画（今のトルク）", "WeightedWbc.cpp, WbcBase.cpp, HierarchicalWbc.cpp"],
            ["legged_hw / unitree_hw", "実機ループ / 実機への通信", "LeggedHWLoop.cpp, UnitreeHW.cpp"],
            ["legged_gazebo", "Gazebo上の同一HWインタフェース", "LeggedHWSim.cpp"],
            ["legged_common", "HybridJointHandle、接地センサ", "HybridJointInterface.h"],
            ["OCS2（外部）", "SQP-NMPC、Gait、重心まわりの運動", "本 repo にソース無し"],
        ],
        0.25,
        1.2,
        12.8,
        5.05,
        col_w=[3.1, 4.0, 5.7],
        font=12,
    )
    d.fn(s, "※ OCP: 最適制御問題。未来の入力列を探す問題の組み立て。  ※ SQP: 非線形問題を、短い二次計画の列として解く。  ※ UDP: 実機との短い通信。  ※ プラグイン: ros-control が読み込む制御器本体。")

    s = d.content("01  §3 起動時に立つプロセス", "load_controller.launch が同時起動")
    d.table(
        s,
        [
            ["プロセス", "パッケージ", "役割"],
            ["controller_manager load .../legged_controller", "controller_manager", "プラグインをロード。startは別サービス"],
            ["legged_robot_gait_command", "ocs2_legged_robot_ros", "端末から gait 名を送り ModeSchedule を更新"],
            ["legged_robot_target", "legged_controllers", "/cmd_vel と /move_base_simple/goal を TargetTrajectories に変換"],
        ],
        0.3,
        1.25,
        12.7,
        3.2,
        font=13,
    )
    d.bullets(s, ["ゲームパッドは任意。joy_teleop.launch が /cmd_vel を出す。軸は joy.yaml。"], top=4.7, size=16)
    d.fn(s, "※ launch: ROSの起動ファイル。複数プロセスを一度に立ち上げる。  ※ TargetTrajectories: 時刻と目標状態の列。ここでは2点だけ。  ※ gait 名: 端末に打つ stance / trot などの文字列。")

    s = d.content("01  §4 2つの周期", "ハードウェア / WBC と NMPC スレッド")
    d.table(
        s,
        [
            ["項目", "値", "根拠"],
            ["loop_frequency（実機）", "500 Hz", "legged_unitree_hw/config/a1.yaml"],
            ["LeggedController::update", "同じ 500 Hz", "controller_manager_->update"],
            ["状態推定 / WBC / 関節指令", "500 Hz", "update() の先頭と末尾"],
            ["Gazebo 指令遅延", "delay 0.009 s", "gazebo 設定"],
            ["mpc.timeHorizon", "1.0 s", "task.info"],
            ["mpc.mpcDesiredFrequency", "100 Hz", "task.info。advanceMpc()"],
            ["sqp.dt / sqpIteration", "0.015 s / 1", "時間の刻み。1周期にSQPを1回だけ"],
            ["予測段数", "1.0/0.015 ≈ 67", "1秒を15 msで切った点数"],
        ],
        0.3,
        1.2,
        12.7,
        5.05,
        col_w=[4.2, 3.2, 5.3],
        font=13,
    )
    d.fn(s, "※ timeHorizon: 何秒先まで考えるか。  ※ advanceMpc(): NMPCスレッドが未来の解を更新する関数。  ※ 射撃間隔: 予測を何秒おきに区切るか（sqp.dt）。  ※ RTI: リアルタイム反復。毎回1回だけ改善して次へ進むやり方。")

    s = d.content("01  §4 図", "速い時計と遅い時計。同じ閉ループの二つの速さ")
    fig_clocks(d, s)
    d.fn(s, "※ 500 Hz: 推定・WBC・モータ。  ※ 100 Hz: 未来1秒の最適化。  ※ 切れ目: 速い側は束の全部ではなく「今」の1点だけを使う。")

    if deep:
        s = d.content("01  §4.2 続き", "WBCは最新policyを現在時刻で補間する")
        d.bullets(
            s,
            [
                "setupMrt() が別 std::thread を立て、mpcDesiredFrequency_ で mpcMrtInterface_->advanceMpc() を呼ぶ。",
                "WBC側は毎周期 updatePolicy() のあと evaluatePolicy(t, x)。NMPCが遅れても前回policyを使い続ける。",
                "mpc.mrtDesiredFrequency は task.info 上 1000 Hz だが、コメントが Useless。通常ループの周期には使わない。初期policy待ちの sleep にだけ使う。",
                "対応: LeggedController.cpp の setupMrt(), update()。設定は task.info の mpc と sqp。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ policy: NMPCが共有メモリに置いた、今使える未来の解。  ※ evaluatePolicy(t,x): その束から、今の時刻tと今の状態xに合う1点を切り出す。  ※ 補間: 隣り合う予測点のあいだを直線で読む。  ※ task.info: a1の重み・周期・ノイズを書いた設定ファイル。")

    s = d.content("01  §5 LeggedController::update の呼出順", "実装事実としての1周期")
    d.table(
        s,
        [
            ["順", "処理", "結果"],
            ["1", "updateStateEstimation", "measuredRbdState_ (36)、currentObservation_.state (24)"],
            ["2", "mpcMrtInterface_->setCurrentObservation", "NMPCへ今の観測"],
            ["3–4", "updatePolicy / evaluatePolicy", "optimizedState (24)、optimizedInput (24)、plannedMode"],
            ["5", "wbc_->update", "x (42)。torque = x.tail(12)"],
            ["6", "SafetyChecker", "失敗なら stopRequest"],
            ["7", "12関節へ setCommand", "q*, dq*, Kp=0, Kd=3, τ"],
            ["8", "visualization と publish", "今の観測を②へも送る（legged_robot_mpc_observation）"],
        ],
        0.3,
        1.2,
        12.7,
        5.05,
        col_w=[1.0, 4.4, 7.3],
        font=13,
    )
    d.fn(s, "※ (36)/(24): 剛体状態36個と、NMPC用に組み直した状態24個。  ※ plannedMode: 今この瞬間、計画上どの足が接地か。  ※ tail(12): 42個の解の末尾12=トルクだけを取る。  ※ q*/dq*: 目標の関節角と関節速さ。  ※ publish: ROSトピックへ出す。")

    s = d.content("01  §6–7 コントローラとロボット差分", "両方とも WBC は WeightedWbc")
    d.table(
        s,
        [
            ["プラグイン", "推定", "用途"],
            ["legged/LeggedController", "KalmanFilterEstimate", "実機と通常sim"],
            ["legged/LeggedCheaterController", "FromTopicStateEstimate（/ground_truth/state）", "sim専用。READMEは実機禁止"],
        ],
        0.3,
        1.25,
        12.7,
        2.3,
        font=14,
    )
    d.bullets(
        s,
        [
            "HierarchicalWbc はソースにあるが init() から呼ばれない。",
            "ROBOT_TYPE で a1 / go1 / aliengo。NMPC次元は同じ24/24。変わるのはURDF、comHeight、初期関節、トルク上限。",
            "comHeight: a1, go1 は 0.3 m、aliengo は 0.4 m。以降の数値例は a1。",
        ],
        top=3.8,
        size=15,
    )
    d.fn(s, "※ Cheater: シミュレータの正解位置をそのまま読む推定。実機には正解が無いので禁止。  ※ HierarchicalWbc: 優先度つき全身制御。コードはあるが起動しない。  ※ URDF: ロボットのリンク・関節・質量の記述。  ※ comHeight: 目標の重心高さ。")


def _02(d: Deck, deep: bool):
    d.section_slide("02", "全体データフロー", "02_System_Architecture_and_Dataflow.md。箱のあいだを流れる配列の長さと単位。")

    s = d.content("02  §1 結論", "標準経路を一文で")
    d.bullets(
        s,
        [
            "ユーザーが胴体速度（または2Dゴール）を指令し、2点のcentroidal目標軌道を作る。",
            "線形Kalmanが現在剛体状態を推定し、OCS2 SQP-NMPCが未来の運動量・姿勢・関節・地面反力を最適化する。",
            "WeightedWbc が現在瞬間の q̈, Fc, τ を解き、12関節へハイブリッド指令を出す閉ループである。",
            "型・単位・frame・配列サイズ付きの境界契約は本章が正本。各ブロックの式は 03 以降。",
        ],
        top=1.2,
        size=16,
    )
    d.fn(s, "※ 2Dゴール: RVizで地図上の1点を指定する経路。速さ指令とは別。  ※ SQP-NMPC: 逐次二次計画で解く非線形予測制御。  ※ q̈: 加速度。Fc: 地面反力。τ: トルク。  ※ 境界契約: 箱のあいだで渡す配列の長さ・単位・座標系。")

    s = d.content("02  §2 指令経路（イベント）", "Gait は胴体指令と並列。箱はノートと同じ")
    fig_cmd(d, s)
    d.fn(s, "※ 左が胴体（速さまたはゴール）。右が歩容。④で初めて合流する。  ※ TargetTrajectories: 時刻2 + 状態24×2 + 入力24×2。  ※ SqpMpc: OCS2の逐次二次計画ソルバ。")

    s = d.content("02  §2 制御閉ループ（500 Hz）", "NMPCは100 Hzのpolicyを読む。箱はノートと同じ")
    fig_loop(d, s)
    d.fn(s, "※ evaluatePolicy: 遅い束から「今」を1点切り出す。  ※ quat/ω/a: 向き4数・角速度・加速度。  ※ 植物: 制御されるロボット側。")

    s = d.content("02  §3 境界ごとのデータ契約（指令側）", "shape / 単位 / 周期")
    d.table(
        s,
        [
            ["上流 → 下流", "出力", "shape", "周期"],
            ["人 → ①", "/cmd_vel Twist", "linear 3 + angular.z = 4", "イベント"],
            ["RViz → ①'", "/move_base_simple/goal", "Pose → 内部 (6,)", "イベント"],
            ["gait_command → ④", "gait template", "mode列 + switchingTimes", "イベント"],
            ["② → ④", "TargetTrajectories", "t:2, x (24,)×2, u (24,)×2", "/cmd_vel ごと"],
            ["⑦ → ③", "関節・IMU・接地", "q12, dq12, quat4, ω3, a3, contact4", "500 Hz"],
            ["③ → ④⑤", "rbdState_ / x", "(36,) と (24,)", "500 Hz"],
            ["④ → ⑤", "evaluatePolicy", "x*24, u*24, plannedMode", "500 Hzで読む"],
            ["⑤ → ⑥", "qpSol の末尾", "tau (12,) N·m", "500 Hz"],
            ["⑥ → ⑦", "setCommand", "関節あたり5、全体60", "500 Hz"],
        ],
        0.2,
        1.2,
        12.9,
        5.05,
        col_w=[3.2, 3.3, 4.2, 2.2],
        font=12,
    )
    d.fn(s, "※ イベント: 人が打ったときだけ動く（周期ループではない）。  ※ N·m: トルクの単位。  ※ switchingTimes: 歩容が次の接地へ移る時刻。")

    s = d.content("02  §4 4本のベクトル", "同じ「今」でも、箱ごとに形が違う")
    fig_vecs(d, s)
    d.fn(s, "※ 帯の幅は個数に比例。  ※ x は④用、rbd は③の生、wbc は⑤の内部。外へ出る矢印は τ12 だけ。")

    s = d.content("02  §4.1 NMPC状態 x (24,)", "task.info の initialState コメントが正本")
    d.table(
        s,
        [
            ["index", "記号", "意味（覚える用）", "単位"],
            ["0:3", "v_com", "胴体がどの速さで並進したいか（質量で割った運動量）", "m/s"],
            ["3:6", "L/m", "胴体がどの速さで回転したいか（角運動量/質量）", "m²/s 相当"],
            ["6:9", "p_b", "地図上の胴体位置 x,y,z", "m（世界）"],
            ["9:12", "(ψ,θ,φ)", "旋回・前後傾・左右傾", "rad"],
            ["12:24", "q_j", "12関節の今の角度", "rad"],
        ],
        0.3,
        1.20,
        12.7,
        5.05,
        font=14,
    )
    d.fn(s, "※ index 0:3 は「0,1,2の3個」。Pythonと同じ半開区間。  ※ CoM: 質量中心。  ※ 正規化: 質量で割って、速度と同じ単位にしたもの。  ※ initialState: 起動時の目標姿勢が書いてある設定。")

    s = d.content("02  §4.2–4.4 入力・剛体・WBC", "u 24、x_rbd 36、x_wbc 42")
    d.table(
        s,
        [
            ["ベクトル", "index", "中身"],
            ["u (24,)", "0:12", "4脚が地面を押す力 f_c（各足3方向）。単位 N、世界座標"],
            ["u (24,)", "12:24", "12関節を動かす速さ v_j。rad/s。トルクはここには無い"],
            ["x_rbd (36,)", "0:3 / 3:6 / 6:18", "向きZYX、胴体位置、関節角12"],
            ["x_rbd (36,)", "18:21 / 21:24 / 24:36", "世界の角速度、世界の並進速さ、関節速さ12"],
            ["x_wbc (42,)", "0:18 / 18:30 / 30:42", "加速度18、地面反力12、トルク12。外へ出すのはトルクだけ"],
        ],
        0.25,
        1.20,
        12.8,
        5.05,
        col_w=[2.4, 3.4, 7.0],
        font=13,
    )
    d.fn(s, "※ ω: 角速度。  ※ N: 力の単位ニュートン。  ※ 観測の input: 推定は作らず、evaluatePolicy のあと NMPC入力で上書きする。")

    s = d.content("02  §5 実行周期の関係", "500 Hz が今を切り、100 Hz が束を更新する")
    fig_clocks(d, s)
    d.fn(s, "※ policy: NMPCが置いた未来の解の束。  ※ 100 Hzのあいだ、500 Hz側は同じ束を新しい t と x で4回読む。Jacobianは毎回実測。")

    if deep:
        s = d.content("02  §6–7 元スケッチから直した点 / 標準に無いもの", "実装事実")
        d.bullets(
            s,
            [
                "指令は3速度ではなく Twist の4成分（linear.z も含む）。NMPCは胴体12ではなく24。WBC出力の内部は42。",
                "Gaitを並列パスとして持つ。目標軌道は2点の線形補間。cmd_vel 経路だけ head(3) に速度指令を入れる。",
                "推定はNMPCの前段というより、500 Hzループの先頭である。低レベルは Kp=0, Kd=3 固定。",
                "標準に無い: 障害物回避ローカルプランナ、地形マップ、NMPCによるGait自動選択、HierarchicalWbc の配線、setSurfaceNormalInWorld。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ Twistの4成分: 前後・横・上下・旋回。上下はゲームパッドからは出ないが、コードは読む。  ※ setSurfaceNormalInWorld: 斜面の法線を摩擦錐に入れる関数。未実装で例外。")


def _03(d: Deck, deep: bool):
    d.section_slide("03", "ユーザー指令と目標軌道", "03_User_Command_and_Reference.md。速さ4つを、1秒先までの2点目標にする。")

    s = d.content("03  §1 結論", "人は速さを与える。軌道は2点。Gaitは触らない")
    d.bullets(
        s,
        [
            "ユーザーは「今どこにいるべきか」ではなく、「どの速さで胴体を動かしたいか」、または「odom上の1点へ行きたいか」を与える。",
            "TargetTrajectoriesPublisher は最新のNMPC観測を基準に、現在と最大1秒先の 2点 のcentroidal状態を作り、OCS2 RosReferenceManager へ送る。",
            "Gaitはこのノードでは触らない。",
        ],
        top=1.2,
        size=17,
    )
    d.fn(s, "※ odom: ロボットが自分で積算した地図。  ※ RosReferenceManager: OCS2が目標軌道を受け取る窓口。  ※ centroidal状態: 勢い6+位置向き6+関節12の24個。")

    s = d.content("03  §3.6 例", "前進 0.5 m/s。ノートの2点を図にする")
    fig_twopoints(d, s)
    d.fn(s, "※ これが②の出口。④はこれを直線で読む。足の振りは⑤と④の制約側。")

    if deep:
        s = d.content("03  §2.1–2.2 背景と目的", "ブロック①")
        d.bullets(
            s,
            [
                "四足のモデルベース歩行では、MPCへ生のジョイスティックを渡さず、まず胴体の参照軌道にする。MIT Cheetah系も ANYmal/OCS2系もこの分離を使う。",
                "本実装は OCS2 の TargetTrajectories に合わせ、速度指令とゴール指令を同じ2点軌道へ正規化する。",
                "目的: 前進・横歩き・旋回を x_ref(t) の種にする。指令ノードはモータを直接駆動しない。ゴールと速度は別callback、同じ publisher。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ x_ref(t): 時刻tで「こうなっていてほしい」状態。  ※ callback: メッセージが来たときに走る関数。  ※ publisher: その軌道をNMPC側へ出す側。")

    s = d.content("03  §2.3 入力経路", "実装事実として3経路")
    d.table(
        s,
        [
            ["経路", "トピック", "中間ベクトル", "shape"],
            ["速度", "/cmd_vel", "cmdVel", "(4,) = [vx, vy, vz, ψ̇]"],
            ["ゴール", "/move_base_simple/goal", "cmdGoal", "(6,) = [px, py, pz, ψ, θ, φ]"],
            ["観測", "legged_robot_mpc_observation", "latestObservation_", "state (24,), input (24,), time, mode"],
        ],
        0.25,
        1.25,
        12.8,
        3.0,
        font=14,
    )
    d.bullets(
        s,
        [
            "latestObservation_.time == 0 のあいだは両方の指令を捨てる。最初のobservationまで軌道は更新されない。",
            "前進 0.5 m/s だけ欲しいとき cmdVel = [0.5, 0, 0, 0]^T。元スケッチの3成分では linear.z が落ちる。",
        ],
        top=4.5,
        size=15,
    )
    d.fn(s, "※ ψ̇: 旋回の速さ（ヨーレート）。  ※ observation: NMPCが見ている今の状態・入力・時刻・接地番号。  ※ time==0: まだ一回も観測が来ていない印。")

    if deep:
        s = d.content("03  §2.3 続き", "joy.yaml と対応コード")
        d.bullets(
            s,
            [
                "軸1 → linear.x scale 1.0。軸2 → linear.y scale 0.8。軸0 → angular.z scale π。deadman ボタン 4。",
                "linear.z はゲームパッドからは出ないが、cmdVelCallback は msg->linear.z を読む。",
                "対応: TargetTrajectoriesPublisher.h コンストラクタ、TargetTrajectoriesPublisher.cpp の main()、config/joy.yaml。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ deadman: 押しているあいだだけ指令を出す安全ボタン。  ※ linear.z: 上下速度。パッドからは0のまま。")

    s = d.content("03  §3.1–3.2 ブロック② 背景と目的", "密な足軌道はここには無い")
    d.bullets(
        s,
        [
            "NMPCのコストは ||x − x_ref||_Q^2。参照が無いと、その場の姿勢維持と入力正則化しか持たない。",
            "OCS2は TargetTrajectories（時刻列 + 状態列 + 入力列）を線形補間する。",
            "本ノードは未来の密な軌道計画をしない。現在姿勢から指令速度を一定時間積分した終端ポーズを1点足すだけ。",
            "高さ・roll・pitchは設定値へ固定。関節参照は defaultJointState。遊脚軌道はここには無い。",
        ],
        top=1.2,
        size=16,
    )
    d.fn(s, "※ ||x−x_ref||_Q^2: 目標からのズレを、重みQで罰する。大きいQほど強く寄せる。  ※ 正則化: 入力を無駄に大きくしないための罰。  ※ 遊脚: 地面から離れている足。")

    d.eq_lesson(
        "03  §3.3 速度経路",
        "指令並進を現在ZYXでWorldへ回す",
        r"v_W=R_{\mathrm{ZYX}}(\psi,\theta,\phi)\,[v_x,v_y,v_z]^\top",
        "vW",
        "人の並進3成分は指令frame。NMPCの位置はWorld。yaw速度 ψ̇_cmd は回転しない。",
        "現在ポーズ p_cur = x[6:12] を観測から取り、R_ZYX で回す。",
        [
            ["記号", "読み方", "何の数か"],
            ["vx,vy,vz", "人が言った前後・横・上下の速さ", "① /cmd_vel の並進3つ"],
            ["ψ,θ,φ", "今の旋回・前後傾・左右傾", "観測 x の向き（index 9:12）"],
            ["R_ZYX", "その向きで座標を回す行列", "鼻先の「前」を地図のxyへ"],
            ["v_W", "地図から見た目標の速さ", "あとで x の先頭3に書く"],
        ],
        fontsize=18,
        notes="※ World / (W): 地図座標。指令frameの「前」とは違う。  ※ yaw速度は回さない: 旋回指令はそのまま積分する。",
    )
    d.eq_lesson(
        "03  §3.3 速度経路",
        "積分時間は mpc.timeHorizon = 1.0 s。高さは設定値",
        r"p_x^+=p_x+v_{W,x}T,\ p_y^+=p_y+v_{W,y}T,\ p_z^+=h_{\mathrm{com}},\ \psi^+=\psi+\dot\psi T",
        "pint",
        "v_{W,z} は位置 p_z には使わない。θ^+=0、φ^+=0。a1 では h_com=0.3 m。",
        "始点ポーズは現在xy/yaw、高さは h_com。関節は両方とも DEFAULT_JOINT_STATE。運動量6はいったんゼロ。入力列はゼロ（コスト側が重力補償に置き換える）。",
        [
            ["記号", "読み方", "値 / 意味"],
            ["T", "何秒先まで積分するか", "1.0 s（NMPCの予測時間と同じ）"],
            ["p^+, ψ^+", "1秒後の水平位置と旋回角", "今の位置 + 速さ×T"],
            ["h_com", "目標の胴体高さ", "a1 は 0.3 m。上下速度では動かさない"],
            ["x0, x1", "今と1秒後の目標状態（各24個）", "速度経路では先頭3を両方 v_W にする"],
        ],
        fontsize=14,
        notes="※ DEFAULT_JOINT_STATE: 立っているときの12関節角。足の振り軌道はここには無い。  ※ 入力列ゼロ: コスト側が体重を支える力に置き換える。",
    )
    d.eq_lesson(
        "03  §3.5 NMPCが見る軌道",
        "区間内は線形補間。速度指令時、参照CoM速度は一定",
        r"x^{\mathrm{ref}}(t)=\frac{t_1-t}{t_1-t_0}x_0+\frac{t-t_0}{t_1-t_0}x_1",
        "q_lerp",
        "x0, x1 の先頭3は同じ v_W なので、参照CoM速度は区間内で一定。位置参照だけが直線に進む。",
        "流れるデータは時刻2 + 状態 24×2 + 入力 24×2 = 98 scalar（時刻を除けば96）。",
        [
            ["記号", "読み方", "例（原点、前へ0.5 m/s）"],
            ["x_ref(t)", "時刻tの目標状態", "2点を直線で結んだもの"],
            ["x0 / t0", "今の目標 / 今の時刻", "速さ(0.5,0,0)、位置(0,0,0.3)"],
            ["x1 / t1", "1秒後の目標 / 今+1秒", "速さ同じ、位置(0.5,0,0.3)"],
            ["流れる量", "時刻2 + 状態24×2 + 入力24×2", "98個（時刻除くと96）"],
        ],
        fontsize=16,
        notes="※ 線形補間: あいだの時刻は、2点を割合で混ぜるだけ。細かい足軌道は作らない。  ※ CoM速度一定: 両端の速さ欄が同じなので、参照の速さは区間内で変わらない。",
    )

    if deep:
        s = d.content("03  §3.4 ゴール経路", "位置追従。運動量はゼロのまま")
        d.bullets(
            s,
            [
                "goalCallback は Pose を odom へ TF し、quaternion を eulerAngles(0,1,2) で ZYX に分解する。",
                "目標xyとyawをゴールから使う。zは COM_HEIGHT、pitch/rollは0。",
                "到達時刻は並進距離 / targetDisplacementVelocity と |Δψ| / targetRotationVelocity の大きい方。a1は 0.5 m/s と 1.57 rad/s。",
                "状態先頭6（運動量）はゼロのまま。対応: goalToTargetTrajectories(), estimateTimeToTarget()。上限は reference.info。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ TF: 座標系の変換（ゴールをodomへ）。  ※ quaternion: 向きの4数。ZYXの3角へ分解する。  ※ COM_HEIGHT: 設定の目標高さ。ゴールのzは使わない。")

    s = d.content("03  §3.7 / §4", "Gaitはこのブロックに含まれない。種類は別蛇口")
    d.table(
        s,
        [
            ["人が選ぶもの", "例", "④ NMPC へ", "⑤ WBC へ"],
            ["速さ / ゴール", "/cmd_vel または goal", "x_ref（犬 2 匹）", "直接は入らない"],
            ["stance", "四脚接地。初期", "4脚に力を許す", "4脚で方程式"],
            ["trot", "対角、0.6 s", "LF_RH / RF_LH で制約切替", "plannedMode の対角だけ押す"],
            ["flying_trot", "空中区間がある名前", "gait.info の列（未照合）", "その瞬間の c に従う"],
        ],
        0.35,
        1.20,
        12.6,
        3.15,
        font=13,
    )
    d.bullets(
        s,
        [
            "load_controller.launch は別に legged_robot_gait_command を起動する。速さ指令は種類を切り替えない。",
            "NMPC は mode を最適化しない。WBC は歩容を切り替えない。同じ接地旗 c を、計画はホライズン全体、実行は今の1点で読む。",
            "実装事実: 2点軌道、高さ固定。推奨改善: 指令とGaitを1ノードにまとめる案はREADME自身が書いている。現行は分離。",
        ],
        top=4.50,
        size=14,
    )
    d.fn(s, "※ modeSequence: 接地の順番（例 LF_RH → RF_LH）。  ※ plannedMode: 今の計画 mode。WBCの接地旗。推定の observation.mode とは別。")


def _04(d: Deck, deep: bool):
    d.section_slide("04", "状態推定", "04_State_Estimation.md。測れない胴体位置を、接地足とIMUから復元する。")

    s = d.content("04  §1 結論", "並進だけKalman。姿勢と関節はセンサをそのまま")
    d.bullets(
        s,
        [
            "実機経路の推定は、IMU姿勢と関節角をそのまま使い、胴体並進位置・速度だけを線形Kalmanで復元する。",
            "出力は剛体状態 (36,) であり、コントローラがcentroidal状態 (24,) へ変換してNMPCへ渡す。",
            "Cheater経路は /ground_truth/state を読むだけで、実機禁止である。",
        ],
        top=1.2,
        size=17,
    )
    d.fn(s, "※ IMU: 傾き（姿勢）と加速度のセンサ。位置は測れない。  ※ 線形Kalman: 予測と観測を直線の式でつなぐ推定。  ※ Cheater / ground_truth: シミュレータが知っている正解。実機には無い。")

    if deep:
        s = d.content("04  §2–3 背景と目的", "Flayols et al., Humanoids 2017 系")
        d.bullets(
            s,
            [
                "浮動ベースには胴体xyzの絶対エンコーダが無い。IMUは姿勢と加速度、モータは関節角、足力センサは接地の有無。",
                "接地足が世界に対して滑らないという仮定で、相対足位置から胴体位置を観測する。クラスは KalmanFilterEstimate。",
                "視覚オドメトリ /tracking_camera/odom/sample が来たときだけ位置を上書きする任意経路がある。",
                "目的: NMPCの x(t0)=x0、WBCがPinocchioに渡す q,v、yawの連続化、接地フラグを observation.mode にする。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ 浮動ベース: 胴体が空中に浮いており、位置を直接駆動するモータが無い。  ※ 視覚オドメトリ: カメラで地図位置を測る任意入力。  ※ Pinocchio: 質量・足位置を計算するライブラリ。")

    s = d.content("04  §4 センサ入力とサイズ", "updateStateEstimation() がHWハンドルから読む")
    d.table(
        s,
        [
            ["信号", "shape", "単位", "出典"],
            ["関節位置 / 速度", "(12,) / (12,)", "rad / rad/s", "HybridJointHandle"],
            ["接地", "(4,) bool", "無次元", "ContactSensorHandle。実機は footForce>40"],
            ["IMU姿勢", "quat (4,)", "係数順 x,y,z,w", "ImuSensorHandle"],
            ["IMU角速度", "(3,)", "rad/s base", "同上"],
            ["IMU並進加速度", "(3,)", "m/s² 胴体座標。重力を含む", "同上"],
        ],
        0.25,
        1.20,
        12.8,
        5.05,
        font=14,
    )
    d.fn(s, "※ Handle: ros-control が公開する読み書き口。  ※ quat x,y,z,w: 向きの4数。係数の順番に注意。  ※ footForce>40: 足力センサが40を超えたら「接地」とみなす（a1）。")

    s = d.content("04  §5 剛体状態の組み立て", "姿勢と関節はフィルタしない")
    d.bullets(
        s,
        [
            "StateEstimateBase が rbdState_ (36,) を埋める。",
            "updateJointStates: index 6:18 に q_j、24:36 に q̇_j。",
            "updateImu: quat → ZYX（zyxOffset_ を引く）、局所ωをglobalへ。index 0:3 にZYX、18:21 にglobal ω。",
            "KalmanFilterEstimate::update が位置と並進速度を決め、updateLinear が 3:6 と 21:24 に書く。",
            "フィルタ対象は胴体並進と足位置だけ。対応: StateEstimateBase.cpp、LinearKalmanFilter.cpp。",
        ],
        top=1.2,
        size=16,
    )
    d.fn(s, "※ q_j / q̇_j: 関節角と関節速さ。  ※ zyxOffset_: 起動時のヨーずれを引く補正。  ※ global ω: 胴体座標の角速度を世界座標へ回したもの。")

    d.eq_lesson(
        "04  §6.1 線形Kalman",
        "状態18、観測28。接触数 nc=4",
        r"\hat x=[p_b^\top,v_b^\top,p_{f1}^\top\ldots p_{f4}^\top]^\top\in\mathbb{R}^{18}",
        "kfx",
        "y ∈ R^28 は 4足の相対位置・相対速度と高さ4。p_s ≈ −p_ee（zに footRadius=0.02 m）。",
        "出力として剛体状態へ戻すのは p_b と v_b だけである。",
        [
            ["記号", "読み方", "何の数か"],
            ["x̂ (18)", "推定の内部状態", "胴体位置3 + 速さ3 + 足位置12"],
            ["p_b, v_b", "胴体の地図位置と速さ", "外へ出すのはこの6個"],
            ["p_f", "各足の地図位置", "接地中は世界に固定、と仮定"],
            ["y (28)", "観測", "4足の相対位置12 + 相対速さ12 + 高さ4"],
        ],
        fontsize=16,
        notes="※ p_s ≈ −p_ee: 胴体原点から見た足位置の符号反転。足半径 0.02 m をzに足す。  ※ 遊脚ノイズ100倍: 「この足は世界に止まっていない」とフィルタに伝える。",
    )
    d.eq_lesson(
        "04  §6.2–6.3 予測と更新",
        "加速度で予測し、接地足の相対位置で直す",
        r"a_W=R(q)\,a_B+[0,0,-9.81]^\top,\quad p_b-p_{f,i}\approx p_{s,i}",
        "kf2",
        "A は位置←位置+速度Δt、速度はそのまま、足位置は固定。B は (1/2 Δt², Δt, 0)。",
        "観測は p_b−p_f≈p_s、v_b≈v_s、p_{f,z}≈h。標準Kalman更新を密行列のLUで解く。",
        [
            ["記号", "読み方", "何の数か"],
            ["a_B / a_W", "胴体座標 / 地図座標の加速度", "IMUを向きで回し、重力を足す"],
            ["Δt", "1周期の長さ", "0.002 s（500 Hz）"],
            ["p_s", "足から見た胴体の相対位置", "関節角から計算できる観測"],
            ["LU", "連立方程式の解き方", "標準Kalman更新の中で使う"],
        ],
        fontsize=15,
        notes="※ 予測: 加速度を積分して位置・速さを先に進める。  ※ 更新: 接地足が滑らないなら、相対位置が位置の観測になる。空中の足は信用しない。",
    )

    s = d.content("04  §7–8 centroidal変換とCheater", "24への変換はOCS2側")
    d.bullets(
        s,
        [
            "x = computeCentroidalStateFromRbdModel(x_rbd) ∈ R^24。CentroidalModelRbdConversions（本repo外）。",
            "そのあと state(9) = yawLast + shortest_angular_distance(...) でyawを連続化。mode は stanceLeg2ModeNumber(contactFlag)。",
            "FromTopicStateEstimate は /ground_truth/state の pose/twist を rbdState_ へ直書き。関節は直前の updateJointStates が残る。",
            "実装事実: 並進だけKF。姿勢はIMU。接地は閾値bool。出力36→24。",
            "未確認: computeCentroidalStateFromRbdModel の運動量定義の係数はOCS2ソース未照合。",
        ],
        top=1.2,
        size=15,
    )
    d.fn(s, "※ yaw連続化: ±π をまたいでも差が跳ねないよう、短い角距離でつなぐ。  ※ mode: 今どの足が接地かの番号。  ※ KF: Kalmanフィルタの略。")
    if deep:
        s = d.content("04  §6.4 / §9", "視覚オドメトリは任意。境界")
        d.bullets(
            s,
            [
                "/tracking_camera/odom/sample が来ると topicUpdated_ が立ち、updateFromTopic() が xHat_ の胴体位置と足位置をTF経由で上書きする。トピックが無ければこの枝は動かない。",
                "理論: 接地足ゼロ速度・既知半径の線形観測。推奨改善: 遊脚ノイズ倍率100はマジックナンバー。接触力連続値を使っていない。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ マジックナンバー: 根拠が設定に出てこない決め打ちの数。  ※ 接触力連続値: 足力の大きさそのもの。今は閾値で真偽だけ使う。")


def _05(d: Deck, deep: bool):
    d.section_slide("05", "NMPC（非線形モデル予測制御）", "05_NMPC.md  /  ④ と Gait 結合。未来1秒を最適化し、トルクは出さない。")

    s = d.content("05  §1 結論", "状態24・入力24。WBCへ渡すのは1点")
    d.bullets(
        s,
        [
            "NMPCはOCS2 SqpMpc で、centroidalダイナミクスの最適制御を多重射撃SQPとして解く。",
            "状態24・入力24、ホライズン1.0 s、離散刻み15 ms、SQP反復1回、別スレッド100 Hz。",
            "WBCへ渡すのは evaluatePolicy が現在時刻で取り出した 1点 の x*, u* と mode。",
            "ダイナミクス本体 LeggedRobotDynamicsAD はOCS2側。本repoが組み立てるのはコスト、制約、参照、ソルバ設定。",
        ],
        top=1.2,
        size=16,
    )
    d.fn(s, "※ 多重射撃: 未来を何点かに区切り、各点で方程式をつなぐ解き方。  ※ ホライズン: 予測する未来の長さ。  ※ x*/u*: 最適だとして出た状態と入力。  ※ AD: 自動微分。偏微分を人手で書かない。")

    if deep:
        s = d.content("05  §2–3 背景と目的", "FullCentroidal が既定")
        d.bullets(
            s,
            [
                "Sleiman et al., RAL 2021 と Grandia et al. の枠。ocs2_legged_robot 経由で使い、URDF・重み・摩擦・自己衝突を LeggedInterface に置く。",
                "既定 centroidalModelType=0 の FullCentroidalDynamics。関節運動がcentroidal運動量へ与える影響を残す。SRBD（type=1）は設定にあるが a1/go1/aliengo の既定は0。",
                "ホライズン上で同時に決める: 目標2点に沿う運動量とポーズ、立脚GRF / 遊脚ゼロ力、関節速度、gaitの接地スケジュール、摩擦錐と自己衝突。",
                "12関節トルクはNMPCの決定変数ではない。トルクはWBCが現在瞬間に解く。",
            ],
            top=1.2,
            size=15,
        )
        d.fn(s, "※ FullCentroidal: 関節の動きが重心の勢いに与える影響を残したモデル。  ※ SRBD: 単一剛体近似。脚の慣性を落とす。既定ではない。  ※ GRF: 地面反力。  ※ URDF: ロボットの形状と質量の記述。")

    d.eq_lesson(
        "05  §4 最適制御問題",
        "READMEの式を、本repoが渡す中身で具体化する",
        r"\min_{u(\cdot)}\int\ell(x,u)\,dt\quad\mathrm{s.t.}\ \dot x=f(x,u),\ x(t_0)=x_0",
        "ocp",
        "転写は multiple shooting SQP + HPIPM。sqp.dt=0.015、反復1。x0 は currentObservation_.state。",
        "対応: LeggedInterface::setupOptimalControlProblem()、LeggedController::setupMpc() の SqpMpc。",
        [
            ["記号", "読み方", "何の数か"],
            ["x", "未来の状態", "勢い6+位置向き6+関節12 = 24"],
            ["u", "未来の入力", "地面反力12+関節速さ12 = 24。トルクは無い"],
            ["ℓ", "途中の罰（コスト）", "目標から外れるほど大きい"],
            ["f", "状態がどう進むかのモデル", "OCS2側。未照合"],
            ["x0", "今の状態（初期値）", "③が出した24個"],
        ],
        fontsize=16,
        notes="※ min: 罰が一番小さくなる入力列を探す。  ※ s.t.: subject to（次の式を満たせ）。  ※ HPIPM: SQPの内側で二次計画を解くライブラリ。  ※ dt=0.015: 15 msごとの区切り。",
    )
    d.eq_lesson(
        "05  §4.1–4.2 状態・入力とコスト",
        "入力参照は軌道のゼロではなく、接地脚への重力補償",
        r"x=[h_{\mathrm{com}}^\top,q_b^\top,q_j^\top]^\top,\ u=[f_c^\top,v_j^\top]^\top,\ \ell=\|x-x^{\mathrm{ref}}\|_Q^2+\|u-u_{\mathrm{wc}}\|_R^2",
        "ell2",
        "uNominal = weightCompensatingInput(info, contactFlags)。立脚数で mg を分け、遊脚力は0（OCS2側、未確認の分配）。",
        "a1で大きい Q: 位置 xx,yy=1000、高さ1500、yaw 100、roll/pitch 300、水平速度15。関節 2.5–5。R の足速度ブロックは J_b^T R_task J_b で関節速度へ写す。GRFは 10^{-3}。",
        [
            ["記号", "読み方", "何の数か"],
            ["h_com", "重心まわりの勢い", "並進3+回転3。質量で割ってある"],
            ["q_b / q_j", "胴体の位置向き / 関節角", "6 + 12"],
            ["f_c / v_j", "地面反力 / 関節速さ", "12 + 12"],
            ["Q, R", "状態ズレと入力ズレの重み", "大きいほど強く寄せる対角行列"],
            ["u_wc", "体重を接地足で分けた入力", "空中の足は0。OCS2側"],
        ],
        fontsize=13,
        notes="※ 重力補償: 立っているだけで「押すな」と罰しないよう、mgを接地足へ分配した目標入力。  ※ J_b: 足速度と関節速度をつなぐ行列。Rの足速度ブロックを関節側へ写す。",
    )

    s = d.content("05  §4.3 制約", "脚ごとに contactFlags(t)[i] で active が切り替わる")
    d.table(
        s,
        [
            ["名前", "種類", "active", "式", "次元"],
            ["ZeroForce", "等式", "遊脚", "f_c,i = 0", "3"],
            ["ZeroVelocity", "等式", "立脚", "足並進速度≈0。positionErrorGain=0", "3"],
            ["NormalVelocity", "等式", "遊脚", "ż = ż_sw(t)。gain非ゼロなら z も", "1"],
            ["FrictionCone", "軟制約（既定）", "立脚", "μ(Fz+Fg)−√(Fx²+Fy²+ε)≥0", "1"],
            ["selfCollision", "状態軟制約", "常時", "リンク対距離 ≥ 0.05 m", "ペア数"],
        ],
        0.2,
        1.2,
        12.9,
        4.2,
        font=12,
    )
    d.bullets(s, ["μ=0.3。useHardFrictionConeConstraint は既定 false。setSurfaceNormalInWorld は未実装で例外。世界鉛直錐。地形高さは 0。"], top=5.55, size=13)
    d.fn(s, "※ 立脚: 地面についている足。遊脚: 空中の足。  ※ 軟制約: 破ると罰は増えるが、解は続ける。硬制約は破ると失敗。  ※ μ: 摩擦係数。小さいと滑りやすいとみなす。  ※ ε: ゼロ割を避ける小さな数。")

    if deep:
        s = d.content("05  §4.3 続き", "遊脚zは SwingTrajectoryPlanner の spline CPG")
        d.table(
            s,
            [
                ["a1設定", "値"],
                ["liftOffVelocity", "0.05 m/s"],
                ["touchDownVelocity", "−0.1 m/s"],
                ["swingHeight", "0.08 m"],
                ["swingTimeScale", "0.15 s（短いswingは高さ縮小）"],
                ["地形高さ", "modifyReferences は 0 を渡す。不整地の真の標高は使わない"],
            ],
            0.35,
            1.20,
            12.6,
            5.05,
            col_w=[3.4, 9.2],
            font=14,
        )
        d.fn(s, "※ CPG / spline: 足を上げて下ろす高さを、短い曲線で作る。  ※ liftOff / touchDown: 離地・接地のときの上下速さ。  ※ swingHeight: 空中での上げ幅 8 cm。")

    s = d.content("05  §5 Receding horizon", "WBCが見るのはこの瞬間の24+24+modeだけ")
    d.bullets(
        s,
        [
            "NMPCスレッドは advanceMpc() でホライズン全体を解く。",
            "制御スレッド: setCurrentObservation → updatePolicy → evaluatePolicy(t, x, ...) → currentObservation.input = optimizedInput。",
            "useFeedbackPolicy=false。評価はフィードバックゲインではなく軌道の補間。",
            "未来のGRF列はvisualizationには出るが、トルク計算には使わない。",
        ],
        top=1.2,
        size=16,
    )
    d.fn(s, "※ Receding horizon: 毎回「今から1秒」を解き直し、使った分を捨てて窓をずらす。  ※ useFeedbackPolicy=false: 状態フィードバックゲインは使わず、軌道を読むだけ。  ※ visualization: 画面表示用。制御の矢印には乗らない。")

    s = d.content("05  §6 Gait結合", "NMPCはmodeを最適化しない。trot の時間割")
    fig_gait(d, s)
    d.fn(s, "※ GaitReceiver が ModeSchedule を更新（OCS2側）。  ※ plannedMode が WBC の接地旗。推定の observation.mode とは別。  ※ 初期は STANCE（四脚）。")

    s = d.content("05  §6 続き", "種類の選択が、NMPC の制約と WBC の旗になる")
    d.table(
        s,
        [
            ["人が打つ名前", "犬の足", "④ NMPC", "⑤ WBC"],
            ["stance（初期）", "四脚接地", "4脚に力・摩擦", "4脚で運動方程式"],
            ["trot", "対角。0.3 s 交代", "LF_RH / RF_LH で ZeroForce 切替", "今の対角だけ押す"],
            ["flying_trot", "空中区間がある", "gait.info の列（本repo未照合）", "その瞬間の c に従う"],
        ],
        0.35,
        1.20,
        12.6,
        3.20,
        font=13,
    )
    d.bullets(
        s,
        [
            "速さ指令は種類を自動変更しない。NMPC は trot の逆相を回答できない。WBC は歩容を切り替えない。",
            "同じ c を、計画はホライズン全体、実行は evaluatePolicy の今の 1 点（plannedMode）で読む。",
        ],
        top=4.55,
        size=15,
    )
    d.fn(s, "※ ZeroForce: 遊脚の力を 0 にする等式。  ※ contactFlag_: WBC が「この足は押してよい」と見る旗。計画 mode が駆動する。")

    if deep:
        s = d.content("05  §7–8 初期化と境界", "starting() と未確認")
        d.bullets(
            s,
            [
                "starting() は推定1回のあと、現在状態の1点軌道を参照に入れ、initialPolicyReceived() まで advanceMpc() を回す。その後 mpcRunning_=true で100 Hzスレッドが動き出す。",
                "実装事実: 状態24入力24、SQP 1回、100 Hz、摩擦は軟制約、地形高さ0、既定はFullCentroidal。",
                "未確認: f(x,u) の成分式、HPIPMのQPサイズ、weightCompensatingInput の正確な分配。",
                "未実装: 接触時刻の同時最適化、知覚地形、硬摩擦の既定利用。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ starting(): 制御器が走り始めるときの初期化。  ※ 知覚地形: 地図の高さを制約に入れること。後継 legged_perceptive 側。")


def _06(d: Deck, deep: bool):
    d.section_slide("06", "WBC（全身制御）", "06_WBC.md  /  ⑤。今の瞬間の二次計画。出力はトルク12。")

    s = d.content("06  §1 結論", "既定は WeightedWbc。出力は末尾12")
    d.bullets(
        s,
        [
            "既定WBCは WeightedWbc。NMPCが出した目標状態・入力と、推定した剛体状態を入力に、現在瞬間のQPをqpOASESで解く。",
            "接地旗は推定ではなく NMPC の plannedMode（人が選んだ gait の今の1点）。WBC は stance / trot を切り替えない。",
            "決定変数は42次元で、コントローラが使うのは末尾12の関節トルクだけである。",
            "HierarchicalWbc は実装済みだが LeggedController::init は呼ばない。",
        ],
        top=1.2,
        size=17,
    )
    d.fn(s, "※ qpOASES: 二次計画を解くライブラリ。  ※ 決定変数: ソルバが探す未知数。ここでは加速度・地面反力・トルク。  ※ init: 起動時の配線。ここで WeightedWbc だけが作られる。")

    s = d.content("06  §2–3 背景と目的", "READMEの「strict hierarchy」は未配線側の説明")
    d.table(
        s,
        [
            ["クラス", "解法", "既定?"],
            ["WeightedWbc", "ハード制約 + 重み付き最小二乗。単一QP", "はい"],
            ["HierarchicalWbc", "HoQp で null-space 階層", "いいえ"],
        ],
        0.3,
        1.25,
        12.7,
        2.2,
        font=14,
    )
    d.bullets(
        s,
        [
            "Centroidal NMPCは慣性の縮約モデル。実機は関節トルク、摩擦、遊脚加速度、トルク飽和を同時に満たす必要がある。",
            "今だけ両立する: 運動方程式、トルク上限と摩擦、立脚加速度ゼロ、NMPCの胴体加速度とGRFへの近さ、遊脚PD。",
            "NMPCのGRFは目標。WBCの Fc は必要ならずらし、その結果の τ を出す。",
        ],
        top=3.7,
        size=15,
    )
    d.fn(s, "※ ハード制約: 必ず満たす式。破ったらその解は捨てる。  ※ null-space 階層: 上の優先度を守ったまま、残った自由度で下を解く。  ※ トルク飽和: モータが出せる上限。  ※ PD: 位置ズレに比例、速さズレに比例のフィードバック。")

    s = d.content("06  §4 図", "入るもの3つ。中で42。出るのはトルク12")
    d.node(s, 0.35, 1.15, 3.90, 2.40, "入  ④ の今", "x* 24  と  u* 24", "desired 状態と押し方", NAVY)
    d.node(s, 0.35, 3.70, 3.90, 2.20, "入  ③ の今", "rbd 36  と  mode", "今の姿勢で方程式を書く", GOLD)
    d.node(s, 4.55, 1.15, 4.20, 4.75, "⑤ WeightedWbc", "未知数 42 = q̈18 + Fc12 + τ12", "硬制約: 運動方程式・摩擦・トルク上限・接地足は滑らない\nコスト: 遊脚PD ≫ 胴体加速度 ≫ 地面反力", TERR)
    d.node(s, 9.05, 1.15, 3.90, 4.75, "出  ⑥ へ", "τ = 末尾 12", "q̈ と Fc は内部解。指令には乗らない", TEAL)
    d.arr(s, 4.18, 2.20)
    d.arr(s, 4.18, 4.50)
    d.arr(s, 8.68, 3.20)
    d.fn(s, "※ 重み a1: 遊脚100、胴体1、地面反力0.01。NMPCの力は目標で、ずらしてよい。")

    s = d.content("06  §4 決定変数と入出力", "18+12+12=42。下流は τ だけ")
    d.table(
        s,
        [
            ["向き", "名前", "shape", "由来 / 使用"],
            ["入", "stateDesired / inputDesired", "(24,) / (24,)", "NMPC optimizedState / Input"],
            ["入", "rbdStateMeasured", "(36,)", "推定"],
            ["入", "mode / period", "scalar / s", "plannedMode。ベース加速度タスクの差分"],
            ["出", "qpSol", "(42,)", "全体"],
            ["出", "torque", "(12,)", "末尾12だけを指令のトルク（ff）にする"],
        ],
        0.25,
        1.20,
        12.8,
        5.05,
        font=14,
    )
    d.fn(s, "※ period: 制御周期。ベース加速度タスクで、前回との差分から加速度を作る。  ※ ff: feedforward。測ったズレではなく、計算済みトルクをそのまま足す欄。")

    if deep:
        s = d.content("06  §5 計測側と参照側の運動学", "Pinocchioの配置順が rbdState と違う")
        d.bullets(
            s,
            [
                "updateMeasured は推定 q,v で質量行列 M、非線形項 nle、足Jacobian J と J̇ を取る。",
                "ピンocchioの配置は [p(3), zyx(3), q_j(12)]。rbdState の [zyx, p, q_j] とは順序が違う。コードが入れ替える。",
                "updateDesired はNMPC状態から望ましい q,v を出し、centroidal運動量行列を更新する。遊脚タスクの望む足位置は、このFKから取る。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ M: 今の質量行列（動かしにくさ）。  ※ nle: 重力など、今わかっている力。  ※ J / J̇: 足位置の変わり方とその時間変化。  ※ FK: 関節角から足の位置・速さを計算すること。")

    d.eq_lesson(
        "06  §6.1 浮動ベース運動方程式",
        "硬制約。18等式",
        r"[M,\ -J^\top,\ -S^\top]\,[\ddot q,\ F_c,\ \tau]=-nle",
        "eom2",
        "S は関節選択（浮動6列が0、関節12が単位）。",
        "トルク上限は a1 で各関節 33.5 N·m。±τ_lim の不等式24。",
        [
            ["記号 / タスク", "読み方", "何を強制するか"],
            ["M, q̈", "質量行列と加速度", "胴体6+関節12の加速度"],
            ["F_c, τ", "地面反力とトルク", "未知数。外へ出すのはτ"],
            ["S", "関節だけを選ぶ行列", "浮動6列は0、関節12は1"],
            ["立脚ゼロ運動", "接地足は滑らない", "足の並進加速度=0"],
            ["摩擦ピラミッド", "円錐を直線5本で内側から近似", "Fz≥0 かつ |Fx|,|Fy| ≤ μ Fz"],
        ],
        fontsize=18,
        notes="※ nle: 重力・コリオリなど、今計算できる力。右辺へ移す。  ※ n_st: 今接地している足の本数。  ※ μ=0.3: NMPCの軟制約と同じ値。形は円錐ではなく線形ピラミッド。",
    )
    d.eq_lesson(
        "06  §6.5 遊脚PD",
        "コスト。a1 は kp=350、kd=37",
        r"J_i\ddot q=k_p(p_i^*-p_i)+k_d(v_i^*-v_i)-\dot J_i v",
        "sw",
        "p*,v* はNMPC望ましいFK、p,v は実測FK。モータの位置ばねとは別。",
        "ベース加速度追従（コスト6）は、NMPC関節速度の差分から q̈_b* を復元する。接触力追従は F_c = u*[0:12]。",
        [
            ["記号", "読み方", "何の数か"],
            ["p, v / p*, v*", "今の足 / 欲しい足", "実測FK と NMPC姿勢のFK"],
            ["kp, kd", "位置ばねと速さダンパ", "350 と 37。モータのKp=0とは別"],
            ["swingLeg 重み", "100", "一番強い望み。空中の足を運ぶ"],
            ["baseAccel 重み", "1", "胴体加速度をNMPCに寄せる"],
            ["contactForce 重み", "0.01", "地面反力は弱く追う。ずらしてよい"],
        ],
        fontsize=16,
        notes="※ このPDはWBCの中のコスト。モータへ出す位置ばね（Kp）とは別物。  ※ 重みが小さい接触力: NMPCの力は目標であり、方程式が厳しければずらす。",
    )

    s = d.content("06  §7 WeightedWbc のQP", "失敗時のフォールバックは無い")
    d.bullets(
        s,
        [
            "硬制約（EoM・トルク・摩擦・立脚ゼロ運動・遊脚ゼロ力）を qpOASES の lbA ≤ A x ≤ ubA に積む。",
            "コストは遊脚・ベース加速度・接触力の加重和。nWsr=20、options.setToMPC()。getPrimalSolution をそのまま返す。",
            "trot（立脚2・遊脚2）の目安: EoM 18 + 遊脚ゼロ力6 + 立脚ゼロ運動6 + トルク箱24 + 摩擦10 = 64行、変数42。",
        ],
        top=1.2,
        size=16,
    )
    d.fn(s, "※ EoM: 運動方程式（Equations of Motion）。  ※ lbA≤Ax≤ubA: 等式も不等式も、上下限つきの一本の行列に積む書き方。  ※ nWsr: 内部反復の上限。  ※ フォールバック無し: 解けなくても前回トルクへ戻さない。")

    s = d.content("06  §8–10 HierarchicalWbc・安全・境界", "優先度は未配線側")
    d.bullets(
        s,
        [
            "HierarchicalWbc の優先度: 0=EoM+トルク+摩擦+立脚ゼロ運動、1=ベース加速度+遊脚、2=接触力追従。HoQp が上位の null space で下位を解く。",
            "LeggedController は std::make_shared<WeightedWbc>(...) だけ。",
            "SafetyChecker は観測ポーズの roll（getBasePose の index 5）が ±π/2 を超えたら失敗し、コントローラを止める。ピッチもトルクも見ない。",
            "実装事実: 加重QP、42変数、出力τ12、μ=0.3ピラミッド。理論（README図）は階層WBC。推奨改善: 失敗時の前回トルク保持、README表記の揃え。",
        ],
        top=1.2,
        size=15,
    )
    d.fn(s, "※ HoQp: 階層二次計画の実装クラス。  ※ roll: 左右傾。±90°を超えたら倒れたとみなして止める。  ※ 未配線: ファイルはあるが、起動コードから呼ばれない。")


def _07(d: Deck, deep: bool):
    d.section_slide("07", "関節制御とハードウェア", "07_Joint_Control_and_Hardware.md。トルクをモータへ出し、センサを③へ返す。")

    s = d.content("07  §1 結論", "Kp=0、Kd=3、ff=τ_WBC。12関節同じゲイン")
    d.bullets(
        s,
        [
            "低レベルは関節空間のハイブリッド指令である。WBCトルクを feedforward にし、位置ゲインは0、速度ゲインは3である。",
            "Gazeboでは同じ式をプラグイン内で計算し、実機ではUnitree LowLevel コマンドとしてモータ側PDへ渡す。",
            "12関節すべてが同じゲインである。",
        ],
        top=1.2,
        size=17,
    )
    d.fn(s, "※ ハイブリッド指令: 目標角・目標速さ・位置ゲイン・速度ゲイン・トルクの5つ。  ※ feedforward: 計算済みトルクを、測ったズレと独立に足す。  ※ LowLevel: Unitreeモータが受け取る低レベルコマンド。")

    if deep:
        s = d.content("07  §2.1–2.2 背景と目的", "高い関節PDはWBCの力制御と喧嘩する")
        d.bullets(
            s,
            [
                "全身QPのトルクだけを開ループで流すと、接触衝撃とモデル誤差で関節が暴れる。",
                "本スタックは「低ゲインPD + トルクFF」で衝撃を抑えつつ、力はWBCに任せる。READMEもその意図を書いている。",
                "τ_WBC をモータの主トルクにする。NMPCの q_j*, q̇_j* を弱い速度フィードバックの目標にする。位置フィードバックは既定で切る。",
            ],
            top=1.2,
            size=16,
        )
        d.fn(s, "※ 開ループ: 測った角度で戻さないこと。  ※ PD: 位置ズレ×Kp + 速さズレ×Kd。  ※ FF: feedforward（計算済みトルク）。")

    d.eq_lesson(
        "07  §2.3 指令の組み立て",
        "Kp は0なので第2項は消える。ユーザー原稿の「主にKd」は実装事実",
        r"\tau_{\mathrm{cmd}}=\tau_{\mathrm{WBC}}+K_p(q^*-q)+K_d(\dot q^*-\dot q)=\tau_{\mathrm{WBC}}+3(\dot q^*-\dot q)",
        "tau2",
        "posDes = getJointAngles(optimizedState)、velDes = getJointVelocities(optimizedInput)、torque = x_wbc.tail(12)。",
        "setCommand の引数順は (q*, dq*, Kp, Kd, ff)。流れるデータは関節あたり5スカラー、全体60。",
        [
            ["記号", "読み方", "何の数か"],
            ["τ_WBC / τ_cmd", "⑤が出したトルク / モータへ出すトルク", "12個。単位 N·m"],
            ["q*, q̇* / q, q̇", "目標の角と速さ / 測った角と速さ", "目標はNMPCの「今」から抜く"],
            ["Kp, Kd", "位置ばねと速さダンパ", "0 と 3。12関節同じ"],
            ["未接続時 実機", "指令が来なければダンピングだけ", "ff=0, q̇*=0, Kd=3 に戻す"],
        ],
        fontsize=14,
        notes="※ setCommand の引数順は (q*, dq*, Kp, Kd, ff)。全体 5×12=60 個。  ※ 位置ばね0: ⑤の力を打ち消さない。速さダンパ3: 接地衝撃だけ抑える。",
    )

    s = d.content("07  §3.2 実機 UnitreeHW", "500 Hz で read → update → write")
    d.table(
        s,
        [
            ["LowCmd フィールド", "中身"],
            ["q / dq", "posDes_ / velDes_"],
            ["Kp / Kd", "kp_（既定0） / kd_（既定3）"],
            ["tau", "ff_（WBCトルク）"],
            ["その後", "Safety::PositionLimit と PowerProtect（a1 は power_limit: 4）。UDP送信"],
            ["read()", "モータの角・速さ・推定トルク、IMU、足力。脚順は関節名で対応"],
        ],
        0.3,
        1.20,
        12.7,
        5.05,
        col_w=[3.4, 9.3],
        font=14,
    )
    d.fn(s, "※ LowCmd: 実機へ送る1周期分のモータ指令。  ※ PowerProtect: 電力上限。a1は power_limit: 4。  ※ FR/FL/RR/RL: Unitree側の脚名。制御側の LF/LH/RF/RH とは並べ方が違う。")

    s = d.content("07  §3.3–3.4 Gazebo と閉ループ", "同じ式。delay 既定 9 ms")
    d.bullets(
        s,
        [
            "writeSim は指令を delay FIFO に入れ、取り出した指令で τ = Kp(q*−q)+Kd(q̇*−q̇)+τ_ff を計算する。既定では τ=τ_WBC+3(q̇*−q̇)。",
            "IMUはリンクのWorld姿勢と相対加速度から作り、接触は ContactManager。名前は LF_FOOT, LH_FOOT, RF_FOOT, RH_FOOT。",
            "モータ（またはGazebo）が次周期の q, q̇、IMU、接地を返し、ブロック③へ戻る。",
            "植物側の次元は浮動ベース6 + 関節12。制御が直接書くのは関節12だけ。",
            "実装事実: Kp=0, Kd=3 固定。ロボット別チューニングは無い。未実装: 電流ループやモータ慣性補償の明示モデル。",
        ],
        top=1.2,
        size=15,
    )
    d.fn(s, "※ FIFO: 先に入れた指令を、9 ms 遅らせて出す待ち行列。  ※ 植物: 制御される側（ロボット本体）。  ※ 浮動ベース6: 胴体の位置3+向き3。モータは関節12しか書けない。")


def _08(d: Deck):
    d.section_slide("08", "会話論点カバレッジ", "08_Conversation_Coverage_Map.md。本文の正本は各章。")
    s = d.content("08", "質問がどの章へ入ったか")
    d.table(
        s,
        [
            ["ID", "質問", "統合先"],
            ["Q1", "データ流れを、サイズ付きで整理する", "02。周期は01。変数表はA"],
            ["Q2", "各ブロックの背景・目的・数式・ロジック", "①② 03、③ 04、④ 05、⑤ 06、⑥⑦ 07"],
            ["Q3", "速習PPTとじっくりPPT", "本スライド。構成はノート 00–07"],
        ],
        0.3,
        1.25,
        12.7,
        3.4,
        font=14,
    )
    d.bullets(s, ["ユーザー原文は analysis-logs/00_user_chat_prompts.md に変更せず置いた。"], top=5.0, size=16)
    d.fn(s, "※ 監査表: 会話の質問が、どの章に書いたかの対応表。本文の正本は各章。  ※ 付録A: 変数の完全一覧。")


def build_quick() -> Path:
    d = Deck("速習  ·  docs/legged_control 00–07")
    d.title_slide(
        "legged_control 学習ノート  速習",
        "章立てはノートと同じ。パワポの本体は図（箱・矢印・ベクトル帯）。\n表と式の正本は md。ここでは空間に置いて見る。",
        META + "\n対象: ノートを画面で追うのが重いとき。",
    )
    _00_intro(d, deep=False)
    _01(d, deep=False)
    _02(d, deep=False)
    _03(d, deep=False)
    _04(d, deep=False)
    _05(d, deep=False)
    _06(d, deep=False)
    _07(d, deep=False)
    _08(d)
    return d.save(OUT / "01_quickstart_legged_control.pptx")


def build_deep() -> Path:
    d = Deck("じっくり  ·  docs/legged_control 00–07")
    d.title_slide(
        "legged_control 学習ノート  じっくり",
        "同じ章立て。図を先に置き、表・式・ファイルはノートと同じ中身で残す。",
        META,
    )
    _00_intro(d, deep=True)
    _01(d, deep=True)
    _02(d, deep=True)
    _03(d, deep=True)
    _04(d, deep=True)
    _05(d, deep=True)
    _06(d, deep=True)
    _07(d, deep=True)
    _08(d)
    return d.save(OUT / "02_deep_dive_legged_control.pptx")


def main():
    print(build_quick())
    print(build_deep())


if __name__ == "__main__":
    main()
