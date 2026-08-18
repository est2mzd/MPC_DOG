# 受講者ガイド — PyMPC 2日間ワークショップ

このガイドは **受講者（学習者）** 向けです。Notebook を開く前に読み、各セッション後にチェックリストで自己確認してください。

---

## 1. このワークショップで得られること

### 1.1 学習目標

ワークショップ修了時に、次ができることを目標とします。

- [ ] **GRF · MPC · WBC** の 3 層を 15 分以内に説明できる
- [ ] MuJoCo デモで **緑矢印（GRF）** の意味を指し示せる
- [ ] `mu` や `step_freq` を 1 つ変えたときの **速度・安定性の変化** を言語化できる
- [ ] 不整地で **足場最適化 ON** が必要な理由を 1 文で説明できる
- [ ] プリセット YAML → sim の手順を **自分で再現** できる

### 1.2 非目標（本 WS では扱わない）

- 実機 ROS2 へのデプロイ（参考リンクのみ）
- 強化学習ベース制御の実装
- acados / CasADi の内部モデル編集

---

## 2. 前提知識

| 項目 | レベル | 補足 |
|------|--------|------|
| Linux ターミナル | 基本操作 | `cd`, `source`, `python` |
| Python | 読める程度 | Notebook のセル実行 |
| 線形代数 | ベクトル・行列 | 合力 $F=ma$ 程度 |
| **MPC 経験** | **操舵 MPC 等** | ホライゾン・制約の概念があると最速 |

操舵 MPC 経験者向けの対応表は [WORKSHOP.md §1.3](./WORKSHOP.md#13-adas-操舵-mpc-との対応受講者の前提知識) を参照。

---

## 3. 環境準備（初回 30–60 分）

```bash
cd mpc_dog
./scripts/setup_references.sh    # Quadruped-PyMPC を clone
./scripts/setup_uv_workshop.sh     # uv .venv + acados
source .venv/bin/activate && . .env.workshop
jupyter lab docs/pympc_2day/notebooks/
```

**初回 sim 注意:** acados codegen で **約 5 分** かかります。2 回目以降は ~10 秒です。

---

## 4. 2 日間スケジュール（目安）

### 1 日目

| 時間 | 内容 | 教材 |
|------|------|------|
| 09:00–10:30 | 理論：3 層・摩擦円錐 | [00_theory](./notebooks/00_theory_grf_mpc_wbc.ipynb) |
| 10:45–12:00 | Session 1：平坦スモーク | [01_session01](./notebooks/01_demo_session01_flat_smoke.ipynb) |
| 13:00–15:00 | Session 2：パラメータチューニング | [02_session02](./notebooks/02_demo_session02_flat_tune.ipynb) |
| 15:15–17:00 | 復習 + [TUNING_GUIDE.md](./TUNING_GUIDE.md) 読み |

### 2 日目

| 時間 | 内容 | 教材 |
|------|------|------|
| 09:00–10:30 | Session 3a：箱障害 | [03_session03a](./notebooks/03_demo_session03a_rough_boxes.ipynb) |
| 10:45–12:00 | Session 3b：連続起伏 | [04_session03b](./notebooks/04_demo_session03b_rough_perlin.ipynb) |
| 13:00–15:00 | Session 4：5 kph × 凸凹坂 | [05_session04](./notebooks/05_demo_session04_speed_bumpy.ipynb) |
| 15:15–17:00 | 総合チェック + 質疑 |

---

## 5. セッション別：学ぶこと・やること・確認

### Session 0 — 理論（Notebook 00）

**学ぶこと**

- 四足制御の定番 3 層（ゲイト → MPC → WBC 相当）
- GRF = 地面反力 = MPC の主な最適化変数
- 摩擦円錐：$\sqrt{F_x^2 + F_y^2} \leq \mu F_z$

**やること**

1. Step 1–4 を読む（数式は直觉で OK）
2. Step 5：摩擦円錐プロット（μ を変える）
3. Step 7：調整マトリクスをざっと眺める

**確認**

- [ ] 「MPC は GRF を決め、WBC 相当層が関節トルクに変換する」と言える
- [ ] μ を上げると水平力が取りやすくなる（転倒リスクも上がる）と言える

---

### Session 1 — 平坦スモーク（Notebook 01）

**学ぶこと**

- 最小構成プリセットの意味（足場 opt OFF、flat、標準 trot）
- `ref_z` が低すぎると即転倒する

**やること**

1. プリセット YAML を読む
2. headless sim 4 秒
3. 意図的失敗（ref_z↓）→ 成功パターンを比較
4. [demo_s01_flat.gif](./assets/demo_s01_flat.gif) を見る

**確認**

- [ ] 足場 opt OFF の理由（初回デバッグを単純化）を説明できる
- [ ] GRF 矢印が stance 脚に出ていることを GIF で指せる

---

### Session 2 — 平坦チューニング（Notebook 02）

**学ぶこと**

- `mu`：摩擦円錐の傾き → 加速 vs 安定
- `step_freq`：歩調 → 速さ vs MPC 追従
- パラメータスタディ結果の読み方

**やること**

1. μ を 0.35 / 0.55 で比較実験
2. step_freq を 1.2 / 1.6 で比較実験
3. `param_study_results.json` のグラフを確認

**確認**

- [ ] 「μ↑ = 積極的、μ↓ = 保守的」を実験結果と結びつけられる
- [ ] 「step_freq↑ = 速いが不安定」を説明できる

---

### Session 3a — 箱障害（Notebook 03）

**学ぶこと**

- `scene=random_boxes`：離散的な段差・箱
- 足場 opt ON：MPC が着地点も計画
- 不整地では step_freq↓ / duty↑ が基本

**やること**

1. 足場 opt OFF vs ON を比較（可能なら）
2. step_freq 高すぎ → 低めで安定、を確認
3. [demo_s03_boxes.gif](./assets/demo_s03_boxes.gif) で箱地形を確認

**確認**

- [ ] S1/S2（平坦）と S3 の **地形・足場 opt** の違いを説明できる
- [ ] 不整地で step_freq を下げる理由を言える

---

### Session 3b — 連続起伏（Notebook 04）

**学ぶこと**

- `scene=perlin`：height field による連続うねり
- boxes（離散）vs perlin（連続）の難易度・見た目の違い
- μ を保守側（0.45 以下）に寄せる理由

**やること**

1. boxes vs perlin を同じ Notebook 内で比較
2. [demo_s03_perlin.gif](./assets/demo_s03_perlin.gif) を本番デモとして確認

**確認**

- [ ] お客様向け 1 文：「平坦は GRF-MPC の 3 層、不整地は **足場最適化** を足す」と言える

---

### Session 4 — 5 kph × 凸凹坂（Notebook 05）

**学ぶこと**

- 指令 5 kph = 1.39 m/s、`vel_mult = target_mps / hip_height`
- **speed_ramp_s**：指令を漸増しないと転倒しやすい
- **resilient モード**：転倒後 reset し累積 20 m を評価
- 上り / 下り / 平坦で tuning 方向が変わる

**やること**

1. `speed_terrain_trial_log.json` を読み、no-fail vs resilient の違いを把握
2. no-fall @ 5 kph が失敗することを再現
3. 勝ちパラメータで 3 地形の resilient run を実行
4. 3 つの GIF（flat / uphill / downhill）を確認

**確認**

- [ ] no-fall と resilient の **成功条件の違い** を説明できる
- [ ] 下り坂が最難で duty↑・μ↓ が必要な理由を言える
- [ ] [SPEED_TERRAIN_TRIAL_LOG.md](./SPEED_TERRAIN_TRIAL_LOG.md) の試行過程を追える

---

## 6. 転倒したときのトリアージ

| 症状 | 疑うパラメータ | 最初の一手 |
|------|----------------|------------|
| 即座に倒れる | `ref_z` | `ref_z = hip_height × 1.05` 以上 |
| 加速時に前のめり | `mu`, `step_freq` | μ↓ または step_freq↓ |
| 着地でバウンス | `grf_max`, swing gain | grf_max↓ |
| 不整地で足が変な位置 | 足場 opt, 地形モデル | opt OFF で比較 |
| 高速で転倒 | `speed_ramp_s` | ランプ時間を 2 倍に |

詳細は [TUNING_GUIDE.md](./TUNING_GUIDE.md) を参照。

---

## 7. ワークショップ後の復習

1. **[06_mpc_tuning_journey.ipynb](./notebooks/06_mpc_tuning_journey.ipynb)** で Phase 1–4 の fail/success を一気通貫
2. [MPC_TUNING_JOURNEY.md](./MPC_TUNING_JOURNEY.md) を読みながら `python scripts/tuning_labs.py --list` で lab を実行
3. 実行済み Notebook 01–05 を上から順に再実行
4. 必要なら `python scripts/run_workshop_pipeline.py` で GIF 再生成
5. コンサル向け学習パス：[learning_paths_for_consulting.md](../learning_paths_for_consulting.md)

---

## Session 6 — MPC 調整ジャーニー統合（Notebook 06）

**学ぶこと**

- Phase 1–4 の **失敗 lab → 成功 lab** を 1 本で体験
- `tuning_labs.py` の lab ID と Notebook / Markdown の対応
- 自分で 1 パラメータ変えて再試行

**やること**

1. [MPC_TUNING_JOURNEY.md](./MPC_TUNING_JOURNEY.md) を Phase 1 から読む
2. [06_mpc_tuning_journey.ipynb](./notebooks/06_mpc_tuning_journey.ipynb) を実行
3. Step 7 で `duty_factor` 等を 1 つ変更

**確認**

- [ ] `python scripts/tuning_labs.py --list` で lab 一覧を説明できる
- [ ] Phase 2 の fail/success を `run_lab_pair` で再現した
- [ ] S4 の no-fall 失敗と resilient 成功の違いを説明できる

---

## 8. 用語ミニ辞典

| 用語 | 意味 |
|------|------|
| **GRF** | Ground Reaction Force。足と地面の反力 |
| **SRB** | Single Rigid Body。ロボットを剛体 1 個に近似したモデル |
| **trot** | 対角線上の 2 脚が同時に接地する歩行パターン |
| **duty_factor** | 1 周期のうち支持脚の割合（高い = 安定） |
| **足場 opt** | MPC が着地点（foothold）も最適化する機能 |
| **resilient** | 転倒後 env reset し、累積走行距離で評価するモード |
