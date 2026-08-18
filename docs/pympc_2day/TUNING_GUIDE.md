# パラメータ調整早見表 — MPC 設計者向け

四足 PyMPC で **「何を触ると何が起きるか」** を、成功・失敗パターンとセットでまとめた早見表です。

**体験ガイド（推奨）:** [MPC_TUNING_JOURNEY.md](./MPC_TUNING_JOURNEY.md) — Phase 1–4 の失敗・成功 narrative + `tuning_labs.py` 連携  
**Hands-on Notebook:** [06_mpc_tuning_journey.ipynb](./notebooks/06_mpc_tuning_journey.ipynb)

Notebook 00 Step 7、各デモ Notebook、講義中のトリアージで使用してください。

---

## 使い方

1. 転倒・不安定が出たら **症状** 列から該当行を探す
2. **failure_fix** を試す
3. 改善したら **success_sign** を確認して記録

シミュレーション API:

```python
from pympc_lab import apply_preset, run_flat_sim, run_speed_terrain_sim, run_speed_terrain_sim_resilient

apply_preset("session02_flat_tune")
m = run_flat_sim(seconds=4.0, mu=0.45, step_freq=1.4)
```

---

## 調整マトリクス

### mpc_params.mu — 摩擦円錐の傾き

| 項目 | 内容 |
|------|------|
| **what** | 摩擦円錐の傾き（地面との摩擦係数モデル） |
| **raise（上げる）** | 水平 GRF を取りやすい → 加速・旋回が積極的に |
| **lower（下げる）** | 水平力を抑える → 保守的・滑りにくい |
| **failure_symptom** | 転倒・横滑り・足が刺さる |
| **failure_fix** | μ を下げる / duty_factor↑ / step_freq↓ |
| **success_sign** | 狙った vx に追従、姿勢安定 |

**Session 別の目安**

| Session | 推奨 μ | 理由 |
|---------|--------|------|
| S1 flat | 0.5 | 標準 |
| S2 tune | 0.35–0.55 | 実験レンジ |
| S3 boxes | 0.48 | やや保守 |
| S3 perlin | 0.45 | 連続起伏は低め |
| S4 bumpy flat | 0.42 | 高速+凸凹 |
| S4 bumpy uphill | 0.38 | 上りはさらに低め |
| S4 bumpy downhill | 0.35 | 下りは最保守 |

---

### gait_params.trot.step_freq — 歩調

| 項目 | 内容 |
|------|------|
| **what** | 歩調（1 秒あたりの歩数） |
| **raise** | 足回しが速い → 速走向き、MPC 予測が追いにくい |
| **lower** | 歩幅・支持が長い → 安定、低速向き |
| **failure_symptom** | 足が地面に刺さる、MPC solve が間に合わない |
| **failure_fix** | step_freq↓、horizon↑、solver_mode='speed' |
| **success_sign** | 滑らかな trot、GRF 矢印が周期的 |

**目安:** 平坦 1.4–1.75 Hz / 不整地 1.1–1.2 Hz / Session 4 1.05–1.20 Hz

---

### gait_params.trot.duty_factor — 支持割合

| 項目 | 内容 |
|------|------|
| **what** | 1 周期のうち支持脚の割合 |
| **raise** | 支持が長い → 安定、敏捷性↓ |
| **lower** | 遊脚が長い → 敏捷、着地精度要求↑ |
| **failure_symptom** | 着地で転倒、ダブルサポート不足 |
| **failure_fix** | duty_factor↑（0.7–0.82） |
| **success_sign** | 不整地でも支持中に姿勢回復 |

**Session 4 下り坂:** duty=0.82 が勝ちパラメータ（最も高い）

---

### mpc_params.grf_max — 垂直 GRF 上限

| 項目 | 内容 |
|------|------|
| **what** | 1 足あたり垂直 GRF 上限 |
| **raise** | 大きな蹴り → 加速↑、跳ね・オーバーシュート |
| **lower** | ソフトな着地、加速↓ |
| **failure_symptom** | 跳ねる、関節飽和 |
| **failure_fix** | grf_max↓（≈ mg/4 × safety） |
| **success_sign** | 滑らかな垂直力、過度な跳ねなし |

---

### simulation_params.ref_z — 目標胴体高さ

| 項目 | 内容 |
|------|------|
| **what** | 目標胴体高さ |
| **raise** | 脚を伸ばす → 地面クリアランス↑ |
| **lower** | 低重心 → 安定だが地面接触リスク |
| **failure_symptom** | 即転倒、足が浮く/刺さる |
| **failure_fix** | ref_z = hip_height × 1.05–1.10 |
| **success_sign** | 一定の胴体高さを維持 |

**プリセット:** YAML の `ref_z_scale` で指定（例: 1.07 = hip_height × 1.07）

---

### mpc_params.use_foothold_optimization — 足場最適化

| 項目 | 内容 |
|------|------|
| **what** | MPC 内で着地点も最適化 |
| **raise（ON）** | 不整地向き |
| **lower（OFF）** | 平坦・デバッグ向き |
| **failure_symptom** | 変な位置に足を置く（地形モデル不一致） |
| **failure_fix** | OFF で比較 → 地形推定確認 |
| **success_sign** | 段差で足が安全な位置に着地 |

**ルール:** S1/S2 = OFF、S3/S4 = ON

---

### simulation_params.swing_position_gain_fb — スイング PD

| 項目 | 内容 |
|------|------|
| **what** | スイング脚の位置 PD ゲイン |
| **raise** | 足振りが硬い → オーバーシュート |
| **lower** | 柔らかい → 着地精度↓ |
| **failure_symptom** | スイング脚が振動 |
| **failure_fix** | gain↓ または step_height↓ |
| **success_sign** | 滑らかなスイング軌道 |

---

### mpc_params.horizon × dt — 予測ホライゾン

| 項目 | 内容 |
|------|------|
| **what** | 予測ホライゾン長 |
| **raise** | 先読み↑ → 計算重い |
| **lower** | 反応速い → 先読み不足で転倒 |
| **failure_symptom** | 急停止・方向転換で転倒 |
| **failure_fix** | horizon↑ または ref 速度を緩やかに |
| **success_sign** | 指令変更に滑らかに追従 |

---

## Session 4 追加：速度・坂道

### target_speed_kph + speed_ramp_s

| 項目 | 内容 |
|------|------|
| **what** | 指令前進速度と立ち上がり時間 |
| **5 kph** | ≈ 1.39 m/s。`vel_mult = target_mps / hip_height` |
| **speed_ramp_s** | 0 から target まで線形ランプ。短いと即転倒 |
| **failure_symptom** | 加速直後に前のめり転倒 |
| **failure_fix** | ramp を 18–25 s に延長 |
| **success_sign** | ランプ終了後も走行継続（no-fall の場合） |

### resilient モード（評価のみ）

| 項目 | 内容 |
|------|------|
| **what** | 転倒で env reset、累積距離を加算 |
| **when** | no-fall @ 5 kph が未達のときの到達確認 |
| **注意** | 実機向きではない。教育・ベンチマーク用 |
| **success** | 累積 ≥ 20 m かつ falls ≤ max_falls |

---

## 地形別クイックリファレンス

| 地形 | scene | 最初の一手 |
|------|-------|------------|
| 平坦 | `flat` | S1 プリセット、足場 opt OFF |
| 箱障害 | `random_boxes` | step_freq↓、足場 opt ON |
| 連続起伏 | `perlin` | μ↓ duty↑ |
| 凸凹平坦 | `bumpy_flat` | S4 プリセット + ramp 18 s |
| 凸凹上り | `bumpy_uphill` | μ=0.38 freq=1.10 duty=0.78 |
| 凸凹下り | `bumpy_downhill` | μ=0.35 freq=1.05 duty=0.82 |

カスタム地形は `scripts/workshop_terrain.py` で定義。

---

## トリアージフロー（簡易）

```
転倒？
 ├─ 即倒れ → ref_z ↑
 ├─ 加速時 → mu ↓ または speed_ramp_s ↑
 ├─ 不整地 → step_freq ↓, duty ↑, 足場 opt ON
 └─ 高速指令 → ramp ↑, resilient で距離評価
```

詳細な試行ログ：[SPEED_TERRAIN_TRIAL_LOG.md](./SPEED_TERRAIN_TRIAL_LOG.md)
