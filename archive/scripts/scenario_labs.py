#!/usr/bin/env python3
"""Advanced workshop scenarios — theory · equations · params · implementation."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "docs" / "pympc_2day" / "assets"
RESULTS_PATH = ASSETS / "scenario_lab_results.json"


@dataclass(frozen=True)
class ScenarioLab:
    num: int
    id: str
    title: str
    category: str  # flat | rough | slope | speed | transition
    difficulty: str  # basic | intermediate | advanced | expert
    terrain: str
    speed_kph: float | None
    slope: str  # flat | uphill +0.08 | downhill -0.08 | mixed
    narrative: str
    theory: str  # LaTeX-friendly markdown
    equations: str
    params_focus: str  # markdown table rows
    knowhow: str
    preset: str
    run_fn: str  # flat | speed_no_fall | speed_resilient | compare_presets | theory_only
    fail_kwargs: dict[str, Any] | None
    success_kwargs: dict[str, Any] | None
    kwargs: dict[str, Any] | None  # single-run scenarios
    impl_note: str
    qa: str  # discussion questions for customer


SCENARIO_LABS: list[ScenarioLab] = [
    ScenarioLab(
        num=1,
        id="sc01_flat_mu_ice",
        title="低μ平坦 — 摩擦円錐が支配する",
        category="flat",
        difficulty="basic",
        terrain="flat（チェッカー平坦）",
        speed_kph=None,
        slope="flat",
        narrative="地面摩擦が低い（氷・濡れ床相当）と、同じ垂直力でも水平 GRF を取れない。",
        theory="Layer 2 MPC の接触制約。stance 足 $i$ で $\\sqrt{F_{ix}^2+F_{iy}^2}\\le\\mu F_{iz}$。",
        equations=r"$$|F_{t,i}| \le \mu F_{z,i}, \quad m a_x \approx \frac{1}{N_{st}}\sum_{i\in stance} F_{ix}$$",
        params_focus="| `mu` | 0.55 → 0.28 | MPC 摩擦円錐の傾き |",
        knowhow="μ↓は加速↓だが姿勢は安定。sim 地面摩擦と MPC μ は別物。",
        preset="session02_flat_tune",
        run_fn="flat",
        fail_kwargs={"seconds": 4.0, "scene": "flat", "mu": 0.55, "step_freq": 1.4},
        success_kwargs={"seconds": 4.0, "scene": "flat", "mu": 0.28, "step_freq": 1.2, "duty_factor": 0.72},
        kwargs=None,
        impl_note="`pympc_lab.run_flat_sim` → `mpc_params.mu`",
        qa="Q: μを上げると vx は上がるはずなのに転倒するのは？\nA: 円錐は開くが実際の地面摩擦を超える GRF 計画→スリップ・姿勢崩れ。",
    ),
    ScenarioLab(
        num=2,
        id="sc02_flat_aggressive_gait",
        title="平坦・攻め gait — horizon 内の支持周期",
        category="flat",
        difficulty="basic",
        terrain="flat",
        speed_kph=None,
        slope="flat",
        narrative="Session 2 の速い trot (1.6 Hz) を平坦で試すと、MPC ホライゾン 0.24 s 内で支持→遊脚の切替が追いつかない。",
        theory="Layer 1 が $s_i(k)$ を生成。step_freq↑ → 支持時間 $T_{stance}=duty/freq$ 短縮。",
        equations=r"$$T_{stance} = \frac{duty}{f_{step}}, \quad N \Delta t \ge T_{cycle} \text{ が設計目安}$$",
        params_focus="| `step_freq` | 1.6 → 1.2 | 支持時間確保 |",
        knowhow="平坦でも freq 上限あり。不整地前に「MPC が解ける gait」を確認。",
        preset="session02_flat_tune",
        run_fn="flat",
        fail_kwargs={"seconds": 4.0, "scene": "flat", "mu": 0.5, "step_freq": 1.6},
        success_kwargs={"seconds": 4.0, "scene": "flat", "mu": 0.5, "step_freq": 1.2, "duty_factor": 0.70},
        kwargs=None,
        impl_note="`simulation_params.gait_params.trot.step_freq`",
        qa="Q: 実機で freq を上げれば速くなる？\nA: 解ける QP + WBC 追従 + 着地精度の3条件が必要。",
    ),
    ScenarioLab(
        num=3,
        id="sc03_flat_ref_z_low",
        title="ref_z 不足 — SRB 高さ参照の下限",
        category="flat",
        difficulty="basic",
        terrain="flat",
        speed_kph=None,
        slope="flat",
        narrative="CoM 高さ参照が低いと、WBC が脚を縮めすぎて即転倒（MPC 以前の問題）。",
        theory="状態参照 $\\mathbf{x}^{ref}$ の $z$ 成分 = `ref_z`。SRB 並進は $\\sum F_{iz}\\approx mg$ で支持。",
        equations=r"$$ref_z = k \cdot h_{hip}, \quad k \gtrsim 1.05 \text{（Go2 目安）}$$",
        params_focus="| `ref_z_scale` | 0.95 → 1.08 | CoM 目標高 |",
        knowhow="転倒の第一チェック: ref_z → duty → μ の順。",
        preset="session01_flat_smoke",
        run_fn="flat",
        fail_kwargs={"seconds": 3.0, "scene": "flat", "ref_z_scale": 0.95},
        success_kwargs={"seconds": 4.0, "scene": "flat", "ref_z_scale": 1.08},
        kwargs=None,
        impl_note="`simulation_params.ref_z = hip_height * ref_z_scale`",
        qa="Q: ref_z を上げると不安定になることは？\nA: 限界を超えると脚伸展限界・オーバーシュート。1.05–1.10 で探索。",
    ),
    ScenarioLab(
        num=4,
        id="sc04_flat_duty_low",
        title="duty 不足 — 支持脚が足りない",
        category="flat",
        difficulty="intermediate",
        terrain="flat",
        speed_kph=None,
        slope="flat",
        narrative="duty_factor=0.55 では遊脚比率が高く、単脚支持で姿勢を支えきれない。",
        theory="支持脚数 $N_{stance}(k)=\\sum s_i(k)$。duty↓ → 同時支持が減り $F_{iz}$ 配分が難化。",
        equations=r"$$N_{stance} \ge 2 \text{（trot 対角）}, \quad \sum_{i\in stance} F_{iz} \approx mg$$",
        params_focus="| `duty_factor` | 0.55 → 0.75 | 支持比率 |",
        knowhow="不整地・下り坂ほど duty↑（0.78–0.82）。",
        preset="session02_flat_tune",
        run_fn="flat",
        fail_kwargs={"seconds": 4.0, "scene": "flat", "mu": 0.45, "step_freq": 1.3, "duty_factor": 0.55},
        success_kwargs={"seconds": 4.0, "scene": "flat", "mu": 0.45, "step_freq": 1.2, "duty_factor": 0.75},
        kwargs=None,
        impl_note="`gait_params.trot.duty_factor`",
        qa="Q: duty=1 に近づけるとベスト？\nA: 遊脚がなく段差を越えられない。地形で 0.70–0.82 を調整。",
    ),
    ScenarioLab(
        num=5,
        id="sc05_flat_grf_max",
        title="grf_max 過大 — 垂直力ソフト制限",
        category="flat",
        difficulty="intermediate",
        terrain="flat",
        speed_kph=None,
        slope="flat",
        narrative="垂直 GRF 上限が高すぎると跳ね・関節飽和。低すぎると支持不足。",
        theory="各足 $F_{iz}^{min}\\le F_{iz}\\le F_{iz}^{max}$。`grf_max` が $F_{iz}^{max}$ に相当。",
        equations=r"$$F_{iz}^{max} \approx \frac{mg}{N_{stance}} \times \text{safety}, \quad \text{Go2: } mg/4 \times 1.2$$",
        params_focus="| `grf_max` | 500N → 120N | 垂直 GRF 上限 |",
        knowhow="跳ね→grf_max↓。加速不足→grf_max↑（μ とセットで）。",
        preset="session02_flat_tune",
        run_fn="flat",
        fail_kwargs={"seconds": 4.0, "scene": "flat", "mu": 0.5, "step_freq": 1.3, "grf_max": 500.0},
        success_kwargs={"seconds": 4.0, "scene": "flat", "mu": 0.5, "step_freq": 1.3, "grf_max": 120.0},
        kwargs=None,
        impl_note="`mpc_params.grf_max`",
        qa="Q: grf_max と mu のどちらを先に触る？\nA: 跳ね/飽和→grf_max、横滑り/転倒→μ。",
    ),
    ScenarioLab(
        num=6,
        id="sc06_boxes_s2_gait_fail",
        title="箱地形 × Session2 gait — 地形持ち込み失敗",
        category="rough",
        difficulty="intermediate",
        terrain="random_boxes（離散段差）",
        speed_kph=None,
        slope="flat + 箱",
        narrative="平坦で調整した step_freq=1.6 を箱地形に持ち込むと数秒で転倒。",
        theory="足場 opt ON でも gait が攻めすぎると $s_i(k)$ 周期内に MPC が安全な $F_i$ を確保できない。",
        equations=r"$$\min \sum \|x-x^{ref}\|_Q + \|u\|_R \quad \text{s.t. SRB}, \mu, s_i(k)$$",
        params_focus="| `step_freq` | 1.6 → 1.1 | 箱向け保守化 |",
        knowhow="地形変更 → freq↓ duty↑ が定石。S2 の勝ち gait を S3 に流用しない。",
        preset="session03_rough_boxes",
        run_fn="flat",
        fail_kwargs={"seconds": 4.0, "scene": "random_boxes", "step_freq": 1.6, "use_foothold_optimization": True},
        success_kwargs={"seconds": 4.0, "scene": "random_boxes", "step_freq": 1.1, "duty_factor": 0.75, "use_foothold_optimization": True},
        kwargs=None,
        impl_note="`simulation_params.scene=random_boxes`, foothold opt ON",
        qa="Q: 足場最適化 ON なら gait は攻めて良い？\nA: 着地点と GRF は別。gait が速いと支持時間不足。",
    ),
    ScenarioLab(
        num=7,
        id="sc07_perlin_mu_high",
        title="Perlin × 高μ — 連続起伏での過信",
        category="rough",
        difficulty="intermediate",
        terrain="perlin（連続起伏）",
        speed_kph=None,
        slope="flat + 起伏",
        narrative="boxes より滑らかだが、pitch/roll 変動が連続。μ=0.55 は危険。",
        theory="連続地形では $\\mathbf{r}_i$ が常に変化 → $\\boldsymbol{\\tau}=\\mathbf{r}_i\\times\\mathbf{F}_i$ の摂動大。",
        equations=r"$$\mathbf{I}\dot{\boldsymbol{\omega}} = \sum \mathbf{r}_i \times \mathbf{F}_i, \quad |F_{t,i}| \le \mu F_{z,i}$$",
        params_focus="| `mu` | 0.55 → 0.42 | perlin 向け |",
        knowhow="perlin は boxes より μ を下げる（0.42 前後）。",
        preset="session03_rough_perlin",
        run_fn="flat",
        fail_kwargs={"seconds": 4.0, "scene": "perlin", "mu": 0.55, "use_foothold_optimization": True},
        success_kwargs={"seconds": 4.0, "scene": "perlin", "mu": 0.42, "step_freq": 1.1, "duty_factor": 0.74, "use_foothold_optimization": True},
        kwargs=None,
        impl_note="scene=perlin, `use_foothold_optimization=true`",
        qa="Q: boxes と perlin で同じ preset で良い？\nA: 同系統だが perlin は μ さらに保守。",
    ),
    ScenarioLab(
        num=8,
        id="sc08_perlin_foothold_off",
        title="足場 opt OFF — 地形モデル不一致の切り分け",
        category="rough",
        difficulty="intermediate",
        terrain="perlin",
        speed_kph=None,
        slope="flat + 起伏",
        narrative="不整地で変な足位置 → まず foothold opt OFF で baseline 比較。",
        theory="Layer 2 の foothold 変数が地形推定 $\\hat{h}(x,y)$ に依存。不一致で変な $F_i$。",
        equations=r"$$u_k^* = \arg\min J(x,u) \quad \text{（foothold ON: } u \text{ に着地も含む）}$$",
        params_focus="| `use_foothold_optimization` | ON vs OFF | 切り分け |",
        knowhow="OFF で安定→地形/推定問題。OFF でも転倒→gait/μ 問題。",
        preset="session03_rough_perlin",
        run_fn="flat",
        fail_kwargs={"seconds": 4.0, "scene": "perlin", "mu": 0.45, "use_foothold_optimization": False, "step_freq": 1.2},
        success_kwargs={"seconds": 4.0, "scene": "perlin", "mu": 0.42, "use_foothold_optimization": True, "step_freq": 1.1, "duty_factor": 0.74},
        kwargs=None,
        impl_note="`mpc_params.use_foothold_optimization`",
        qa="Q: 本番は常に ON？\nA: 推定が信頼できるなら ON。デバッグ・平坦は OFF。",
    ),
    ScenarioLab(
        num=9,
        id="sc09_bumpy_3kph",
        title="凸凹平坦 3 kph — 速度を下げた no-fall",
        category="speed",
        difficulty="intermediate",
        terrain="bumpy_flat（Perlin heightfield）",
        speed_kph=3.0,
        slope="flat + 凸凹",
        narrative="5 kph が厳しいとき、3 kph + 長 ramp で no-fall 成功域を確認。",
        theory="指令 $v^{ref}(t)$ ランプ: $v^{ref}(t)=v_{target}\\min(t/T_{ramp},1)$。低速度は $F_{ix}$ 要求↓。",
        equations=r"$$m a_x \approx \sum F_{ix}, \quad v^{ref} \downarrow \Rightarrow |F_{ix}| \downarrow$$",
        params_focus="| `target_speed_kph` | 3.0 | `speed_ramp_s` | 15 |",
        knowhow="速度限界の探索: 3→4→5 kph と段階的に。",
        preset="session04_speed_bumpy_base",
        run_fn="speed_no_fall",
        fail_kwargs=None,
        success_kwargs=None,
        kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 3.0,
            "min_distance_m": 10.0,
            "max_seconds": 30.0,
            "mu": 0.45,
            "step_freq": 1.15,
            "duty_factor": 0.74,
            "ref_z_scale": 1.07,
            "speed_ramp_s": 15.0,
        },
        impl_note="`run_speed_terrain_sim` — min_distance 10 m で短時間検証",
        qa="Q: 3 kph 成功が 5 kph 成功の必要条件？\nA: 十分条件ではないが、失敗なら 5 kph は早すぎ。",
    ),
    ScenarioLab(
        num=10,
        id="sc10_bumpy_5kph_no_fall_fail",
        title="凸凹 5 kph no-fall — 距離 ~4 m で失敗",
        category="speed",
        difficulty="advanced",
        terrain="bumpy_flat",
        speed_kph=5.0,
        slope="flat + 凸凹",
        narrative="Session 4 の最初の壁。5 kph + 短 ramp → 数 m で転倒。",
        theory="凸凹で $F_{iz}^{min}$ 違反・姿勢摂動。$Q$ で roll/pitch 追従と $R$ で $\\|u\\|$ のトレードオフ。",
        equations=r"$$\min \sum \|x_k-x_k^{ref}\|_Q + \|u_k\|_R \quad \text{s.t. } f_{SRB}, \mu, F_z$$",
        params_focus="| `speed_ramp_s` | 12 → 18 | `step_freq` | 1.35 → 1.20 |",
        knowhow="no-fall 不可でも resilient で学習→パラメータ探索に使う。",
        preset="session04_speed_bumpy_base",
        run_fn="speed_no_fall",
        fail_kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 5.0,
            "min_distance_m": 20.0,
            "max_seconds": 25.0,
            "mu": 0.42,
            "step_freq": 1.35,
            "duty_factor": 0.74,
            "speed_ramp_s": 12.0,
        },
        success_kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 5.0,
            "min_distance_m": 8.0,
            "max_seconds": 25.0,
            "mu": 0.42,
            "step_freq": 1.20,
            "duty_factor": 0.76,
            "speed_ramp_s": 18.0,
        },
        kwargs=None,
        impl_note="`run_speed_terrain_sim` — success は 8 m 到達の緩和版",
        qa="Q: no-fall 20 m は必須？\nA: 本ワークショップでは resilient 20 m を実用目標に。",
    ),
    ScenarioLab(
        num=11,
        id="sc11_bumpy_uphill_gravity",
        title="凸凹上り坂 — 重力成分と pitch",
        category="slope",
        difficulty="advanced",
        terrain="bumpy_uphill（pitch +0.08 rad）",
        speed_kph=5.0,
        slope="uphill +0.08 rad",
        narrative="上りでは $mg\\sin\\theta$ 分の追加 $F_x$ が必要。μ も freq も平坦より保守。",
        theory="世界系 x 方向: $m a_x = \\sum F_{ix} - mg\\sin\\theta$（近似）。",
        equations=r"$$F_{ix}^{req} \approx m a_x + mg\sin\theta, \quad \theta \approx +0.08 \text{ rad}$$",
        params_focus="| `mu` | 0.42→0.38 | `step_freq` | 1.20→1.10 | `speed_ramp_s` | 18→20 |",
        knowhow="上り: μ↓ freq↓ ramp↑ ref_z やや↑。",
        preset="session04_bumpy_uphill",
        run_fn="speed_resilient",
        fail_kwargs={
            "scene": "bumpy_uphill",
            "target_speed_kph": 5.0,
            "min_distance_m": 15.0,
            "max_seconds": 60.0,
            "max_falls": 15,
            "mu": 0.42,
            "step_freq": 1.25,
            "duty_factor": 0.74,
            "speed_ramp_s": 14.0,
        },
        success_kwargs={
            "scene": "bumpy_uphill",
            "target_speed_kph": 5.0,
            "min_distance_m": 20.0,
            "max_seconds": 90.0,
            "max_falls": 21,
            "mu": 0.38,
            "step_freq": 1.10,
            "duty_factor": 0.78,
            "ref_z_scale": 1.08,
            "speed_ramp_s": 20.0,
        },
        kwargs=None,
        impl_note="`workshop_terrain.BUMPY_SCENES['bumpy_uphill']` pitch=+0.08",
        qa="Q: pitch を MPC に入れている？\nA: SRB+地形推定経由。明示 slope 項は sim 側重力投影。",
    ),
    ScenarioLab(
        num=12,
        id="sc12_bumpy_downhill_brake",
        title="凸凹下り坂 — 最難（制動・支持）",
        category="slope",
        difficulty="expert",
        terrain="bumpy_downhill（pitch -0.08 rad）",
        speed_kph=5.0,
        slope="downhill -0.08 rad",
        narrative="下りは加速しやすく、duty=0.82, μ=0.35, ramp=22 s まで保守化が必要。",
        theory="下り: $mg\\sin|\\theta|$ が加速方向。制動 $F_{ix}<0$ も摩擦円錐内で。",
        equations=r"$$m a_x = \sum F_{ix} + mg\sin\theta, \quad \theta < 0 \text{（下り）}$$",
        params_focus="| `duty_factor` | 0.76→0.82 | `mu` | 0.42→0.35 | `speed_ramp_s` | 22 |",
        knowhow="下り最難。duty 最大級・μ 最小級・ramp 最長。",
        preset="session04_bumpy_downhill",
        run_fn="speed_resilient",
        fail_kwargs={
            "scene": "bumpy_downhill",
            "target_speed_kph": 5.0,
            "min_distance_m": 15.0,
            "max_seconds": 60.0,
            "max_falls": 20,
            "mu": 0.42,
            "step_freq": 1.20,
            "duty_factor": 0.76,
            "speed_ramp_s": 16.0,
        },
        success_kwargs={
            "scene": "bumpy_downhill",
            "target_speed_kph": 5.0,
            "min_distance_m": 20.0,
            "max_seconds": 120.0,
            "max_falls": 30,
            "mu": 0.35,
            "step_freq": 1.05,
            "duty_factor": 0.82,
            "ref_z_scale": 1.10,
            "speed_ramp_s": 22.0,
        },
        kwargs=None,
        impl_note="preset: `session04_bumpy_downhill.yaml`",
        qa="Q: 下りで freq を上げて速く降りたい？\nA: 支持不足で転倒増。duty↑ freq↓ が先。",
    ),
    ScenarioLab(
        num=13,
        id="sc13_uphill_wrong_preset",
        title="上りで下り preset — パラメータ持ち込み失敗（反対）",
        category="transition",
        difficulty="advanced",
        terrain="bumpy_uphill",
        speed_kph=5.0,
        slope="uphill（下り用 μ,duty 適用）",
        narrative="下り坂用の μ=0.35, duty=0.82 を上りに適用 → 加速不足・停滞（反対方向の誤適用）。",
        theory="地形ごとに $\\theta$ 符号が変わり、必要な $F_{ix}$ 分布も変化。preset は分離保存。",
        equations=r"$$\text{uphill: } F_{ix}>0 \text{ 必要}, \quad \text{downhill preset: 過制動}$$",
        params_focus="| 誤 | session04_bumpy_downhill の μ,duty | 正 | session04_bumpy_uphill |",
        knowhow="YAML を地形別に。上り→下り切替時は全パラメータを見直し。",
        preset="session04_speed_bumpy_base",
        run_fn="speed_no_fall",
        fail_kwargs={
            "scene": "bumpy_uphill",
            "target_speed_kph": 5.0,
            "min_distance_m": 10.0,
            "max_seconds": 40.0,
            "mu": 0.35,
            "step_freq": 1.05,
            "duty_factor": 0.82,
            "speed_ramp_s": 22.0,
        },
        success_kwargs={
            "scene": "bumpy_uphill",
            "target_speed_kph": 5.0,
            "min_distance_m": 10.0,
            "max_seconds": 40.0,
            "mu": 0.38,
            "step_freq": 1.10,
            "duty_factor": 0.78,
            "speed_ramp_s": 20.0,
        },
        kwargs=None,
        impl_note="compare_presets: `session04_bumpy_downhill` vs `session04_bumpy_uphill`",
        qa="Q: 上りから下りへ連続走行は？\nA: 本 sim は scene 固定。実機は scene 検出+gain scheduling が必要。",
    ),
    ScenarioLab(
        num=14,
        id="sc14_downhill_duty_low",
        title="下り duty 不足 — 支持時間が命",
        category="slope",
        difficulty="expert",
        terrain="bumpy_downhill",
        speed_kph=5.0,
        slope="downhill",
        narrative="duty=0.70 では下り+凸凹で支持脚が足りず連続転倒。",
        theory="duty↑ → $T_{stance}$↑ → 1 足あたり $F_{iz}$ 配分時間↑。",
        equations=r"$$T_{stance} = duty / f_{step}, \quad F_{iz,i} \approx \frac{mg}{N_{stance}}$$",
        params_focus="| `duty_factor` | 0.70 → 0.82 | 下り必須 |",
        knowhow="下りで duty<0.78 は危険域。0.82 が S4 勝者値。",
        preset="session04_bumpy_downhill",
        run_fn="speed_resilient",
        fail_kwargs={
            "scene": "bumpy_downhill",
            "target_speed_kph": 5.0,
            "min_distance_m": 12.0,
            "max_seconds": 60.0,
            "max_falls": 15,
            "mu": 0.38,
            "step_freq": 1.10,
            "duty_factor": 0.70,
            "speed_ramp_s": 18.0,
        },
        success_kwargs={
            "scene": "bumpy_downhill",
            "target_speed_kph": 5.0,
            "min_distance_m": 20.0,
            "max_seconds": 90.0,
            "max_falls": 25,
            "mu": 0.35,
            "step_freq": 1.05,
            "duty_factor": 0.82,
            "speed_ramp_s": 22.0,
        },
        kwargs=None,
        impl_note="duty_factor を単独で A/B",
        qa="Q: duty と freq のどちらが効く？\nA: 下りは duty 優先。freq↓は MPC 解の時間的余裕。",
    ),
    ScenarioLab(
        num=15,
        id="sc15_ramp_short_vs_long",
        title="speed_ramp — 指令 $v^{ref}(t)$ の立ち上がり",
        category="speed",
        difficulty="advanced",
        terrain="bumpy_flat",
        speed_kph=5.0,
        slope="flat + 凸凹",
        narrative="ramp=8 s では GRF 要求が急峻。18 s で mean_kph は下がるが距離到達。",
        theory="Layer 1/指令: $v^{ref}(t)=v_t\\min(t/T_r,1)$。急峻 → $|F_{ix}|$ スパイク。",
        equations=r"$$\left|\frac{dv^{ref}}{dt}\right| \downarrow \Rightarrow \left|\sum F_{ix}\right| \text{ のピーク} \downarrow$$",
        params_focus="| `speed_ramp_s` | 8 → 18 | 指令ランプ |",
        knowhow="転倒→まず ramp↑。次に μ↓ freq↓。",
        preset="session04_speed_bumpy_base",
        run_fn="speed_no_fall",
        fail_kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 5.0,
            "min_distance_m": 12.0,
            "max_seconds": 35.0,
            "mu": 0.42,
            "step_freq": 1.20,
            "duty_factor": 0.76,
            "speed_ramp_s": 8.0,
        },
        success_kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 5.0,
            "min_distance_m": 12.0,
            "max_seconds": 45.0,
            "mu": 0.42,
            "step_freq": 1.20,
            "duty_factor": 0.76,
            "speed_ramp_s": 18.0,
        },
        kwargs=None,
        impl_note="`run_speed_terrain_sim` 内 `env._ref_base_lin_vel_H` を ramp",
        qa="Q: ramp を無限に長くすれば良い？\nA: 実用距離・時間制約あり。20 m / 5 kph なら 18–22 s が目安。",
    ),
    ScenarioLab(
        num=16,
        id="sc16_uphill_to_downhill_switch",
        title="上り→下り — preset 切替マトリクス",
        category="transition",
        difficulty="expert",
        terrain="bumpy_uphill → bumpy_downhill",
        speed_kph=5.0,
        slope="mixed",
        narrative="同一 5 kph でも上り→下りで μ, duty, ramp, ref_z がすべて変わる。",
        theory="3 層パイプラインは同じ。変わるのは $\\theta$ と必要 GRF 配分。",
        equations=r"$$\Delta F_{ix}^{req} \approx mg(\sin\theta_{up} - \sin\theta_{down})$$",
        params_focus="| パラメータ | uphill | downhill |\n| mu | 0.38 | 0.35 |\n| duty | 0.78 | 0.82 |\n| ramp | 20s | 22s |",
        knowhow="configs/pympc_presets/session04_bumpy_*.yaml を並べて diff。",
        preset="session04_speed_bumpy_base",
        run_fn="compare_presets",
        fail_kwargs=None,
        success_kwargs=None,
        kwargs={"presets": ["session04_bumpy_uphill", "session04_bumpy_flat", "session04_bumpy_downhill"]},
        impl_note="YAML diff + speed_terrain_results.json",
        qa="Q: 1 つの adaptive MPC で全部やれない？\nA: 可能だが gain scheduling / 地形推定が必要。まず固定 preset で理解。",
    ),
    ScenarioLab(
        num=17,
        id="sc17_bumpy_7kph",
        title="7 kph 挑戦 — 速度限界の探索",
        category="speed",
        difficulty="expert",
        terrain="bumpy_flat",
        speed_kph=7.0,
        slope="flat + 凸凹",
        narrative="5 kph 勝者 preset を 7 kph に上げると no-fall はほぼ不可能。",
        theory="$v^{ref}$↑ → 同 $\\mu$ で $|F_{ix}|$ 不足または円錐違反。",
        equations=r"$$a_x^{des} = \dot{v}^{ref} \text{ or steady } v^{ref}/T, \quad \sum F_{ix} \approx m a_x$$",
        params_focus="| `target_speed_kph` | 7.0 | 限界探索 |",
        knowhow="7 kph は本 workshop 範囲外。5 kph 安定後に μ↑ ramp↓ で段階探索。",
        preset="session04_speed_bumpy_base",
        run_fn="speed_no_fall",
        fail_kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 7.0,
            "min_distance_m": 10.0,
            "max_seconds": 30.0,
            "mu": 0.42,
            "step_freq": 1.20,
            "duty_factor": 0.76,
            "speed_ramp_s": 18.0,
        },
        success_kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 5.0,
            "min_distance_m": 10.0,
            "max_seconds": 30.0,
            "mu": 0.42,
            "step_freq": 1.20,
            "duty_factor": 0.76,
            "speed_ramp_s": 18.0,
        },
        kwargs=None,
        impl_note="7 kph fail vs 5 kph baseline",
        qa="Q: mean_kph 4.0 で success 判定の意味は？\nA: 指令 5 kph に対し 4 kph 以上で『実用追従』と定義。",
    ),
    ScenarioLab(
        num=18,
        id="sc18_uphill_ref_z_low",
        title="上り ref_z 不足 — 勾配+低 CoM",
        category="slope",
        difficulty="advanced",
        terrain="bumpy_uphill",
        speed_kph=5.0,
        slope="uphill",
        narrative="上りで ref_z_scale=1.02 では足が地面に引っかかる。1.08–1.10 が必要。",
        theory="$z^{ref}$↓ → WBC が脚縮小 → 勾配変化で toe collision。",
        equations=r"$$h_{CoM}^{ref} = ref_z, \quad h_{CoM}^{ref} \uparrow \Rightarrow \text{ground clearance} \uparrow$$",
        params_focus="| `ref_z_scale` | 1.02 → 1.08 | 上り向け |",
        knowhow="上り・下りとも ref_z やや↑。下りは 1.10 まで。",
        preset="session04_bumpy_uphill",
        run_fn="speed_no_fall",
        fail_kwargs={
            "scene": "bumpy_uphill",
            "target_speed_kph": 5.0,
            "min_distance_m": 8.0,
            "max_seconds": 35.0,
            "mu": 0.38,
            "step_freq": 1.10,
            "duty_factor": 0.78,
            "ref_z_scale": 1.02,
            "speed_ramp_s": 20.0,
        },
        success_kwargs={
            "scene": "bumpy_uphill",
            "target_speed_kph": 5.0,
            "min_distance_m": 8.0,
            "max_seconds": 35.0,
            "mu": 0.38,
            "step_freq": 1.10,
            "duty_factor": 0.78,
            "ref_z_scale": 1.08,
            "speed_ramp_s": 20.0,
        },
        kwargs=None,
        impl_note="ref_z_scale 単独 A/B on uphill",
        qa="Q: ref_z と duty の関係？\nA: 独立だが両方とも支持安定性に効く。上りは両方やや↑。",
    ),
    ScenarioLab(
        num=19,
        id="sc19_resilient_flat_win",
        title="resilient 20 m — no-fall 不可でも学習",
        category="speed",
        difficulty="advanced",
        terrain="bumpy_flat",
        speed_kph=5.0,
        slope="flat + 凸凹",
        narrative="転倒 reset しながら累積 20 m。17 falls で成功（Session 4 勝者）。",
        theory="評価関数を distance 累積に。制御則は同一 SRB-MPC。",
        equations=r"$$\max \sum distance \quad \text{s.t. falls} \le N_{max}$$",
        params_focus="| `max_falls` | 22 | resilient 許容 |",
        knowhow="デモ GIF は resilient 走行。議論は falls 数も必ず出す。",
        preset="session04_bumpy_flat",
        run_fn="speed_resilient",
        fail_kwargs=None,
        success_kwargs=None,
        kwargs={
            "scene": "bumpy_flat",
            "target_speed_kph": 5.0,
            "min_distance_m": 20.0,
            "max_seconds": 90.0,
            "max_falls": 22,
            "mu": 0.42,
            "step_freq": 1.20,
            "duty_factor": 0.76,
            "ref_z_scale": 1.07,
            "speed_ramp_s": 18.0,
        },
        impl_note="load speed_terrain_results.json でキャッシュ表示可",
        qa="Q: resilient はカンニング？\nA: 指令追従・gait 設計の評価には有効。本番は falls=0 が目標。",
    ),
    ScenarioLab(
        num=20,
        id="sc20_tradeoff_matrix",
        title="設計空間 — μ × duty × ramp × ref_z",
        category="transition",
        difficulty="expert",
        terrain="全地形（表形式）",
        speed_kph=5.0,
        slope="all",
        narrative="20 シナリオを貫くトレードオフ表。お客様 QA の索引。",
        theory="Convex 近似: $s_i(k)$ 固定下で GRF $u$ について QP。",
        equations=r"$$\min \|x-x^{ref}\|_Q + \|u\|_R \;\; \text{s.t.}\;\; |F_t|\le\mu F_z,\; F_z^{min}\le F_z\le F_z^{max}$$",
        params_focus="| 症状 | 第一 | 第二 | 第三 |\n| 即転倒 | ref_z↑ | freq↓ | μ↓ |\n| 加速不足 | μ↑ | ramp↓ | grf_max↑ |\n| 下り失敗 | duty↑ | μ↓ | ramp↑ |",
        knowhow="1 パラメータずつ A/B。YAML で勝ちパターンを資産化。",
        preset="session04_speed_bumpy_base",
        run_fn="theory_only",
        fail_kwargs=None,
        success_kwargs=None,
        kwargs={},
        impl_note="Notebook 11 QA マスターと連携",
        qa="Q: ADAS 操舵 MPC との対応は？\nA: SRB≈車両、GRF≈タイヤ力、μ≈路面摩擦、WBC≈下位アクチュエータ。",
    ),
]

SCENARIO_BY_ID: dict[str, ScenarioLab] = {s.id: s for s in SCENARIO_LABS}


def list_scenarios(*, category: str | None = None) -> list[ScenarioLab]:
    if category is None:
        return list(SCENARIO_LABS)
    return [s for s in SCENARIO_LABS if s.category == category]


def scenario_table() -> list[dict]:
    rows = []
    for s in SCENARIO_LABS:
        rows.append({
            "num": s.num,
            "id": s.id,
            "title": s.title,
            "category": s.category,
            "difficulty": s.difficulty,
            "terrain": s.terrain,
            "speed_kph": s.speed_kph,
            "slope": s.slope,
            "preset": s.preset,
            "run_fn": s.run_fn,
        })
    return rows


def _serialize_result(result: dict) -> dict:
    import numpy as np

    skip = {"t", "vx", "vz", "x"}
    out = {}
    for k, v in result.items():
        if k in skip:
            continue
        if hasattr(v, "tolist"):
            out[k] = v.tolist()
        elif isinstance(v, (np.bool_, np.integer, np.floating)):
            out[k] = v.item()
        else:
            out[k] = v
    return out


def _run_kwargs(run_fn: str, preset: str, kw: dict) -> dict:
    import pympc_lab as labmod

    labmod.apply_preset(preset)
    if run_fn == "flat":
        return labmod.run_flat_sim(**kw)
    if run_fn == "speed_no_fall":
        return labmod.run_speed_terrain_sim(**kw)
    if run_fn == "speed_resilient":
        return labmod.run_speed_terrain_sim_resilient(**kw)
    raise ValueError(run_fn)


def run_scenario(scenario_id: str, *, variant: str = "success") -> dict:
    if scenario_id not in SCENARIO_BY_ID:
        raise KeyError(f"Unknown scenario {scenario_id!r}")
    sc = SCENARIO_BY_ID[scenario_id]
    if sc.run_fn in ("compare_presets", "theory_only"):
        return {"scenario_id": sc.id, "title": sc.title, "mode": sc.run_fn, "kwargs": sc.kwargs}
    if sc.kwargs:
        kw = dict(sc.kwargs)
    elif variant == "fail" and sc.fail_kwargs:
        kw = dict(sc.fail_kwargs)
    elif sc.success_kwargs:
        kw = dict(sc.success_kwargs)
    elif sc.fail_kwargs:
        kw = dict(sc.fail_kwargs)
    else:
        raise ValueError(f"Scenario {scenario_id} has no runnable kwargs")
    result = _run_kwargs(sc.run_fn, sc.preset, kw)
    return {
        "scenario_id": sc.id,
        "title": sc.title,
        "preset": sc.preset,
        "variant": variant,
        "kwargs": kw,
        "result": _serialize_result(result),
        "metrics": result,
    }


def run_scenario_pair(scenario_id: str) -> list[tuple[str, dict]]:
    sc = SCENARIO_BY_ID[scenario_id]
    if not sc.fail_kwargs or not sc.success_kwargs:
        raise ValueError(f"Scenario {scenario_id} has no fail/success pair")
    out = []
    for label, kw in [("FAIL", sc.fail_kwargs), ("OK", sc.success_kwargs)]:
        result = _run_kwargs(sc.run_fn, sc.preset, dict(kw))
        out.append((f"{label}: {sc.title[:30]}", result))
    return out


def compare_preset_table(preset_names: list[str]) -> list[dict]:
    import pympc_lab as labmod

    rows = []
    for name in preset_names:
        y = labmod.load_preset_yaml(name)
        gp = y.get("gait_patches", {}).get("trot", {})
        rt = y.get("runtime", {})
        rows.append({
            "preset": name,
            "scene": y.get("patches", {}).get("simulation_params.scene", "?"),
            "mu": y.get("patches", {}).get("mpc_params.mu"),
            "step_freq": gp.get("step_freq"),
            "duty_factor": gp.get("duty_factor"),
            "ref_z_scale": y.get("ref_z_scale"),
            "speed_ramp_s": rt.get("speed_ramp_s"),
            "target_kph": rt.get("target_speed_kph"),
        })
    return rows


def load_cached_scenario_results() -> dict:
    if not RESULTS_PATH.is_file():
        return {}
    return json.loads(RESULTS_PATH.read_text(encoding="utf-8"))


def save_scenario_result(entry: dict) -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    cache = load_cached_scenario_results()
    cache[entry["scenario_id"]] = {
        k: entry[k] for k in ("scenario_id", "title", "preset", "kwargs", "result") if k in entry
    }
    RESULTS_PATH.write_text(json.dumps(cache, indent=2, ensure_ascii=False), encoding="utf-8")


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Advanced scenario labs")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--scenario", help="scenario id e.g. sc01_flat_mu_ice")
    parser.add_argument("--table", action="store_true", help="print scenario index")
    args = parser.parse_args()

    if args.list or args.table:
        for s in SCENARIO_LABS:
            print(f"sc{s.num:02d} {s.id:30} [{s.category:10}] {s.title}")
        return 0
    if args.scenario:
        entry = run_scenario(args.scenario)
        if "result" in entry:
            save_scenario_result(entry)
            print(json.dumps(entry["result"], indent=2))
        else:
            print(json.dumps(entry, indent=2))
        return 0
    parser.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
