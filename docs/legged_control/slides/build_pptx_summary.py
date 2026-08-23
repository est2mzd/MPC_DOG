#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
docs/legged_control の要約スライド。
図は犬モデル・式との対応・アーキテクチャ。箱の並びではない。
"""

from pathlib import Path

from _figures import (
    fig_architecture,
    fig_clocks,
    fig_closed_loop,
    fig_cmd_frames,
    fig_dog_model,
    fig_gait_binds,
    fig_gait_trot,
    fig_gait_types,
    fig_input_u,
    fig_joints_tau,
    fig_kalman,
    fig_nmpc_horizon,
    fig_state_x,
    fig_two_points,
    fig_wbc_eom,
)
from _kit import TEAL, NAVY, TERR, Deck

OUT = Path(__file__).resolve().parent
META = "要約  /  正本: docs/legged_control  /  コード: external/legged_control@a7f381c  /  a1"


def _full(d, slide, path, notes):
    d.pic(slide, path, 0.45, 1.10, 12.40, 5.10)
    d.fn(slide, notes)


def build_quick() -> Path:
    d = Deck("速習  ·  犬・式・アーキテクチャ")

    d.title_slide(
        "legged_control  速習",
        "ノート 00–07 の要約。犬の絵、式との対応、アーキテクチャで追う。\n細部・ファイル名・未確認は md を正本とする。",
        META,
    )

    s = d.content("要約", "このソフトは何か")
    d.cards(
        s,
        [
            ("対象", ["Unitree A1 の閉ループ。", "速さ指令と歩容から 12 トルク。"], TEAL),
            ("分け方", ["計画は未来 1 s（100 Hz）。", "実行は今の瞬間（500 Hz）。"], NAVY),
            ("このデッキ", ["犬に量を載せる。", "箱の番号合わせではない。"], TERR),
        ],
        top=1.20,
        height=4.70,
    )
    d.fn(s, "※ NMPC: 非線形モデル予測制御。未来を解いて今の入力を決める。  ※ WBC: 全身制御。今の瞬間の二次計画。  ※ 歩容: どの足が地面についてよいかの時間割。")

    s = d.content("ロジック", "指令からトルクまで、人が選ぶのは2つ", "速さは胴体の行き先。歩容名は足の時間割。別蛇口。")
    d.cards(
        s,
        [
            ("速さ / ゴール", ["① /cmd_vel または goal。", "② が犬 2 匹（今と 1 s 先）。", "NMPC の x_ref になる。", "歩容は切り替えない。"], TEAL),
            ("歩容の名前", ["端末に stance / trot /", "flying_trot などを打つ。", "ModeSchedule になる。", "NMPC も WBC も種類を選ばない。"], TERR),
            ("あとの機械", ["NMPC: 与えられた c の上で", "力と運動を解く。トルクは無い。", "WBC: 同じ c で押す足を切り、", "今の犬で τ 12 を出す。"], NAVY),
        ],
        top=1.85,
        height=4.35,
    )
    d.fn(s, "※ README: THE GAIT AND THE GOAL ARE COMPLETELY DIFFERENT AND SEPARATED。  ※ 初期は STANCE（四脚）。寝たまま stance と打つ必要は無い。")

    s = d.content("モデル", "まず犬を見る（a1）")
    _full(d, s, fig_dog_model(), "※ 長さは a1/const.xacro。立位関節は reference.info。接触添字の順は LF, RF, LH, RH（コメント）。")

    s = d.content("00", "閉ループの本体は犬")
    _full(d, s, fig_closed_loop(), "※ 人が速さと歩容を言い、計画が 1 s を持ち、実行が τ 12 を書く。センサが戻る。")

    s = d.content("01", "アーキテクチャ：プロセスとロボット")
    _full(d, s, fig_architecture(), "※ 起動は 3 プロセス。計画スレッドと実行ループは周期が違う。推定は第4層ではない。")

    s = d.content("01", "同じ 20 ms を 2 つの時計で見る")
    _full(d, s, fig_clocks(), "※ SQP: 非線形を短い二次計画の列として解く。反復は 1 回。  ※ policy: NMPC が共有する未来の解の束。")

    d.eq_with_fig(
        "02  状態",
        "x は犬のどこか",
        fig_state_x(),
        r"x=[h_{\mathrm{com}}^\top,\,q_b^\top,\,q_j^\top]^\top\in\mathbb{R}^{24}",
        "sum_x",
        "勢い 6 は胴体の並進・回転の勢い。位置向き 6 は胴体の置き方。関節 12 は 4 脚 × HAA/HFE/KFE。\n\nよくある「胴体12+関節12」ではない。速度の代わりに正規化運動量が入る。",
        notes="※ h_com = [v_com, L/m]。単位は m/s と m²/s 相当。",
        fontsize=15,
    )

    d.eq_with_fig(
        "02  入力",
        "u は足の力と関節の速さ。トルクは無い",
        fig_input_u(),
        r"u=[f_c^\top,\,v_j^\top]^\top\in\mathbb{R}^{24}",
        "sum_u",
        "地面反力 12 は 4 足 × 3。立脚だけ非ゼロ。関節速さ 12 は足の相対運動の正則化。\n\n12 トルクはここには無い。トルクは WBC が今の姿勢で後から解く。",
        notes="※ 脚順は contactNames3DoF。task.info の R コメントは足速度だが、実装は関節速さへ写す。",
        fontsize=15,
    )

    d.eq_with_fig(
        "03  指令",
        "人の「前」を、今の向きで地図へ回す",
        fig_cmd_frames(),
        r"v_W=R_{\mathrm{ZYX}}(\psi,\theta,\phi)\,[v_x,v_y,v_z]^\top",
        "sum_vw",
        "鼻先の 0.5 を地図の x に足すと、旋回後にずれる。今の yaw/pitch/roll で並進 3 つだけ回す。\n\n旋回速さは回さない。高さ目標は 0.3 m。θ,φ の目標は 0。",
        notes="※ /cmd_vel は 4 つ: vx, vy, vz, yaw_rate。goal は別経路で位置 6。",
        fontsize=16,
    )

    d.eq_with_fig(
        "03  指令",
        "参照は密な軌道ではなく、犬が 2 匹",
        fig_two_points(),
        r"p^+=p+v_W T,\quad T=1\,\mathrm{s},\quad x^{\mathrm{ref}}=\mathrm{lerp}(x_0,x_1)",
        "sum_lerp",
        "点0 は今。点1 は 1 秒後。速さ指令では両端の速さ欄を同じ 0.5 にする。あいだは直線。\n\n関節参照は立位のまま。足の振り軌道は②には無い。Gait は触らない。",
        notes="※ 入力参照 24×2 はゼロ。コスト側が体重分担に置き換える。  ※ Gait は②に含まれない。",
        fontsize=13,
    )

    s = d.content("歩容", "種類は名前で選ぶ。犬の足が変わる")
    _full(d, s, fig_gait_types(), "※ stance: 四脚。trot: 対角、0.3 s 交代、周期 0.6 s。flying_trot: 空中区間がある名前。切替時刻の数値列は gait.info（OCS2側。本repo未照合）。")

    s = d.content("歩容 → NMPC → WBC", "同じ接地旗が、計画と実行に入る")
    _full(d, s, fig_gait_binds(), "※ NMPC: contactFlags(t) で ZeroForce / Friction が切替。WBC: plannedMode → contactFlag_。推定の observation.mode とは別。")

    d.eq_lesson(
        "歩容  旗",
        "c は人が選んだ時間割。NMPC も WBC もこれを読む",
        r"c_i(t)=\mathrm{contactFlags}(t)[i],\quad \mathrm{WBC}:\ \mathrm{plannedMode}",
        "sum_c",
        "速さだけでは、いつどの足が押してよいか決まらない。4脚同時では速く歩けない。",
        "端末の gait 名を ModeSchedule にし、ホライズンへ伸ばす。NMPCは mode を最適化しない。WBCは今の1点の mode だけ見る。",
        [
            ["記号", "読み方", "誰が使うか"],
            ["c_i(t)", "脚 i は今接地してよいか", "NMPC の制約切替"],
            ["ModeSchedule", "mode 列 + 切替時刻", "GaitReceiver → ④"],
            ["plannedMode", "今の計画 mode", "WBC の接地旗"],
        ],
        fontsize=14,
        notes="※ 速さ指令は gait 種類を自動変更しない。  ※ NMPC は trot の逆相を回答できない。",
    )

    d.eq_with_fig(
        "04  推定",
        "胴体 xyz のエンコーダは無い。足が位置を教える",
        fig_kalman(),
        r"p\leftarrow p+v\Delta t+\frac{1}{2}a_W\Delta t^2,\quad p_b-p_{f,i}\approx p_{s,i}",
        "sum_kf",
        "向きと関節はセンサを信じる。Kalman が直すのは地図の位置・速さと足位置。\n\n接地足が滑らないなら、関節から見た相対位置が胴体位置の観測になる。空中の足は信用しない（ノイズ 100 倍）。",
        notes="※ 出力は rbd 36。コントローラが x 24 に組み直す。Cheater は sim 専用。Δt=0.002 s。",
        fontsize=13,
    )

    d.eq_with_fig(
        "05  NMPC",
        "未来の犬を並べ、今の 1 匹だけ渡す",
        fig_nmpc_horizon(),
        r"\min_{u(\cdot)}\int\ell(x,u)\,dt\quad\mathrm{s.t.}\ \dot x=f(x,u),\ x(0)=x_{\mathrm{now}}",
        "sum_ocp",
        "今の力だけ見て足を出すと、次の接地に間に合わない。③の今から、②の 2 点へ近づく u を探す。\n\nホライズン 1 s、刻み 15 ms、約 67 点。WBC へは evaluatePolicy の今だけ。u にトルクは無い。",
        notes="※ SQP 1 回、100 Hz。既定 FullCentroidal。f は OCS2側で未照合。",
        fontsize=13,
    )

    d.eq_lesson(
        "05  NMPC",
        "罰: 目標の犬へ寄せ、体重は接地足へ",
        r"\ell=\|x-x^{\mathrm{ref}}\|_Q^2+\|u-u_{\mathrm{wc}}\|_R^2",
        "sum_ell",
        "u の目標を 0 にすると、立っているだけで「押すな」と罰する。",
        "状態は②の 2 点へ。入力は接地している足への体重分担へ。空中は 0。",
        [
            ["記号", "読み方", "何の数か"],
            ["x_ref", "②の直線（犬 2 匹）", "2 点"],
            ["u_wc", "体重を接地足で分けた入力", "OCS2側"],
            ["Q が大きい項", "水平位置 1000、高さ 1500", "a1"],
        ],
        fontsize=18,
        notes="※ 摩擦 μ=0.3 は軟制約。地形高さは 0。",
    )

    s = d.content("05", "trot の中身。対角が 0.3 s で代わる")
    _full(d, s, fig_gait_trot(), "※ 0.0–0.3 s は LF+RH、0.3–0.6 s は RF+LH。NMPC はこの組を選べない。WBC は今どちらの組かを plannedMode で受け取る。")

    d.eq_with_fig(
        "06  WBC",
        "今の犬の質量と足位置で、力を釣り合わせる",
        fig_wbc_eom(),
        r"[M,\ -J^\top,\ -S^\top]\,[\ddot q,\ F_c,\ \tau]=-nle",
        "sum_eom",
        "④の力は粗いモデル上の数字。今の足位置でそのままは成り立たない。加速度・地面反力・トルクを未知にして、この瞬間の運動方程式を必ず満たす。\n\n内部は 42 次元。出口は τ 12 だけ。遊脚は押さず、目標足へ PD（kp=350, kd=37）。",
        notes="※ WeightedWbc（単一QP）。階層版はコードにあるが起動しない。重み: 遊脚100 / 胴体1 / 力0.01。",
        fontsize=14,
    )

    d.eq_with_fig(
        "07  関節",
        "12 個の丸がモータ。主役は WBC の力",
        fig_joints_tau(),
        r"\tau_{\mathrm{cmd}}=\tau_{\mathrm{WBC}}+3(\dot q^*-\dot q)",
        "sum_tau",
        "位置ばねを強くすると⑤の力が消える。ゼロだけだと接地衝撃で速さが跳ねる。トルクは⑤のまま。速さズレに 3 だけかける。\n\n角度ズレは掛けない（Kp=0）。12 関節同じ。実機も Gazebo も同じ 5 数。",
        notes="※ sim は 9 ms 遅らせて同じ式。センサは③へ戻る。上限 33.5 N·m。",
        fontsize=16,
    )

    s = d.content("閉じる", "速さ + 歩容名 → NMPC → WBC → τ 12")
    _full(d, s, fig_architecture(), "※ 左上が速さ、その右が歩容名。計画は c の上で x,u。実行は同じ c で τ。次は md 01–07。")

    return d.save(OUT / "03_summary_quickstart_legged_control.pptx")


def build_deep() -> Path:
    d = Deck("じっくり  ·  犬・式・アーキテクチャ")

    d.title_slide(
        "legged_control  要約  式を足す",
        "速習と同じ犬とアーキ。各章で式を 1 枚足す。\nファイル名・未確認の全文は md。",
        META,
    )

    s = d.content("モデル", "a1")
    _full(d, s, fig_dog_model(), "※ 浮動6 + 関節12。NMPC 次元は go1 / aliengo も同じ 24/24。変わるのは URDF と高さ。")

    s = d.content("地図", "アーキテクチャ")
    _full(d, s, fig_architecture(), "※ 以降は 01→07。新しい分類は増やさない。")

    s = d.content("01", "2 時計")
    _full(d, s, fig_clocks(), "※ loop_frequency 500。mpcDesiredFrequency 100。timeHorizon 1.0。sqp.dt 0.015。")

    d.eq_with_fig(
        "02",
        "x と犬",
        fig_state_x(),
        r"x=[v_{\mathrm{com}}^\top,\,(L/m)^\top,\,p_b^\top,\,(\psi,\theta,\phi),\,q_j^\top]^\top",
        "d_x",
        "v_com 3, L/m 3, p_b 3, ZYX 3, q_j 12。関節順は LF/LH/RF/RH × HAA/HFE/KFE。",
        notes="※ rbd 36 は [ZYX, p, q_j, ω, v, dq]。ピンocchioは [p, zyx, q_j]。コードが入れ替える。",
        fontsize=13,
    )
    d.eq_with_fig(
        "02",
        "u と犬",
        fig_input_u(),
        r"u=[f_{c,1}^\top\ldots f_{c,4}^\top,\,v_j^\top]^\top",
        "d_u",
        "GRF は World。②→④ は t:2, x(24,)×2, u(24,)×2。⑦→③ は q12 dq12 IMU 接地4。",
        notes="※ currentObservation_.input は推定ではなく evaluatePolicy のあと上書き。",
        fontsize=14,
    )

    d.eq_with_fig(
        "03",
        "World へ回して 1 秒積分",
        fig_cmd_frames(),
        r"v_W=R(\psi,\theta,\phi)v_{\mathrm{cmd}},\quad p_x^+=p_x+v_{W,x}T,\ \psi^+=\psi+\dot\psi T",
        "d_pint",
        "高さは h_com=0.3。pitch/roll 目標は 0。速度経路だけ x0[0:3]=x1[0:3]=v_W。ゴール経路は運動量 0 のまま。",
        notes="※ 対応: cmdVelToTargetTrajectories()。T = mpc.timeHorizon。",
        fontsize=13,
    )
    d.eq_with_fig(
        "03",
        "2 点",
        fig_two_points(),
        r"x^{\mathrm{ref}}(t)=\mathrm{lerp}(x_0,x_1;t)",
        "d_two",
        "流れる量は時刻 2 + 状態 48 + 入力 48。足軌道は無い。",
        notes="※ 関節は defaultJointState。Gait は②に含まれない。",
        fontsize=16,
    )

    s = d.content("歩容", "種類")
    _full(d, s, fig_gait_types(), "※ 名前は gait.info。stance / trot はノートが列を持つ。flying_trot の数値列は本repo未照合。")

    s = d.content("歩容", "NMPC と WBC への入り方")
    _full(d, s, fig_gait_binds(), "※ ④: contactFlags(t)。⑤: plannedMode。推定 mode は WBC を駆動しない。")

    d.eq_lesson(
        "歩容",
        "旗の式",
        r"c_i(t)=\mathrm{contactFlags}(t)[i]",
        "d_c",
        "速さだけでは押してよい足が決まらない。",
        "gait 名 → ModeSchedule → ホライズンへ伸ばす。NMPCは選ばない。WBCは今の1点。",
        [
            ["名前", "NMPC がすること", "WBC がすること"],
            ["stance", "4脚に力を許す", "4脚で方程式"],
            ["trot", "対角2脚。0.6 s", "今の対角だけ押す"],
            ["flying_trot", "空中区間（列は gait.info）", "その瞬間の c に従う"],
        ],
        fontsize=18,
        notes="※ 速さ指令は種類を自動変更しない。",
    )

    d.eq_lesson(
        "04",
        "内部状態 18、観測 28",
        r"\hat x=[p_b^\top,v_b^\top,p_{f1}^\top\ldots p_{f4}^\top]^\top",
        "d_kfx",
        "傾きはセンサがある。知りたいのは地図位置と足の地図位置。",
        "外へは p_b, v_b だけを rbd 36 に戻し、24 に組み直す。",
        [
            ["記号", "読み方", "個数"],
            ["x̂", "推定の内部", "18"],
            ["y", "相対位置・速さ・高さ", "28"],
        ],
        fontsize=16,
        notes="※ 遊脚のプロセスノイズは 100 倍。視覚 odom は任意。",
    )
    d.eq_with_fig(
        "04",
        "予測と更新を犬に載せる",
        fig_kalman(),
        r"a_W=R a_B+[0,0,-9.81]^\top,\quad p_b-p_f\approx p_s",
        "d_kf2",
        "積分だけだと漂う。接地足が止まっていれば相対位置が観測。標準 Kalman。空中の足は信用しない。",
        notes="※ computeCentroidalStateFromRbdModel は OCS2側。未照合。",
        fontsize=14,
    )

    d.eq_with_fig(
        "05",
        "OCP と未来の犬",
        fig_nmpc_horizon(),
        r"\min_u\int\ell\,dt,\ \dot x=f(x,u),\ x(t_0)=x_0",
        "d_ocp",
        "多重射撃 SQP + HPIPM。dt=0.015、反復 1。トルクは決定変数にしない。",
        notes="※ 既定 FullCentroidal（type=0）。SRBD は設定にあるが既定ではない。",
        fontsize=16,
    )
    d.eq_lesson(
        "05",
        "コストと制約の要約",
        r"\ell=\|x-x^{\mathrm{ref}}\|_Q^2+\|u-u_{\mathrm{wc}}\|_R^2",
        "d_ell",
        "遊脚: 力 0、上下は短い曲線。立脚: 速度≈0、摩擦軟制約 μ=0.3。",
        "地形高さは 0。自己衝突 0.05 m。",
        [
            ["名前", "いつ", "何をするか"],
            ["ZeroForce", "遊脚", "f_c=0"],
            ["ZeroVelocity", "立脚", "足速さ≈0"],
            ["FrictionCone", "立脚", "滑らない側へ罰"],
        ],
        fontsize=18,
        notes="※ R の足速度ブロックは J^T R J で関節速さへ写す。",
    )

    s = d.content("05", "trot の時間割")
    _full(d, s, fig_gait_trot(), "※ LF_RH → RF_LH。0.3 s。NMPC は逆相を回答できない。WBC は plannedMode だけ見る。")

    d.eq_with_fig(
        "06",
        "今の犬で運動方程式",
        fig_wbc_eom(),
        r"M\ddot q-J^\top F_c-S^\top\tau+nle=0,\quad J_i\ddot q=k_p(p^*-p)+k_d(v^*-v)-\dot J_i v",
        "d_eom",
        "等式は壊さない。遊脚は押さず、目標足へ加速する。トルク上限 33.5 N·m。摩擦ピラミッド μ=0.3。",
        notes="※ 重み swing 100 / base 1 / force 0.01。失敗時のフォールバックは無い。S は関節だけ選ぶ。",
        fontsize=12,
    )

    d.eq_with_fig(
        "07",
        "ハイブリッド指令",
        fig_joints_tau(),
        r"\tau_{\mathrm{cmd}}=\tau_{\mathrm{WBC}}+3(\dot q^*-\dot q)",
        "d_tau",
        "5 数: q*, dq*, Kp=0, Kd=3, ff=τ。全体 60。実機は LowCmd。sim は同じ式を 9 ms 遅らせる。",
        notes="※ 未接続時、実機は Kd=3 だけが残る。SafetyChecker は roll が ±π/2 で停止。",
        fontsize=16,
    )

    s = d.content("閉じる", "速習と同じアーキ")
    _full(d, s, fig_architecture(), "※ 正本は md。このデッキは要約。")

    return d.save(OUT / "04_summary_deep_dive_legged_control.pptx")


def main():
    print(build_quick())
    print(build_deep())


if __name__ == "__main__":
    main()
