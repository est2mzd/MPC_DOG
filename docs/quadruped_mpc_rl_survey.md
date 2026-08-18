# 四足歩行ロボット制御：MPC・強化学習 技術動向サーベイ

**作成日:** 2026-08-18  
**目的:** 不整地を「犬の速度」で移動する四足ロボット制御のため、MPC系・学習系の主要論文と技術傾向を整理する  
**対象読者:** 制御工学の基礎はあるが、足式ロボットは初めて、という方（ADAS操舵MPC経験者向け）

---

## 目次

1. [はじめに：何を目指すか](#1-はじめに何を目指すか)
2. [初心者向け：MPCと足式ロボット制御の基礎](#2-初心者向けmpcと足式ロボット制御の基礎)
3. [なぜMPCは難しいのか（ADAS操舵との比較）](#3-なぜmpcは難しいのかadas操舵との比較)
4. [技術動向の全体像（2020–2026）](#4-技術動向の全体像20202026)
   - [4.5 表と論文の対応マップ](#45-表と論文の対応マップ)
5. [MPC系 主要論文10選](#5-mpc系-主要論文10選)
6. [学習系 主要論文10選](#6-学習系-主要論文10選)
7. [ハイブリッド（MPC＋RL）の潮流](#7-ハイブリッドmpcrlの潮流)
8. [方式選定の指針](#8-方式選定の指針)
9. [事実と推測の区別](#9-事実と推測の区別)
10. [参考文献一覧](#10-参考文献一覧)

---

## 1. はじめに：何を目指すか

### 1.1 ユーザーのゴール（解釈）

「不整地を犬の速度で移動する」とは、おおよそ次の要件を含むと解釈した。

| 要件 | 内容 |
|------|------|
| **地形** | 段差・斜面・砂利・草地・石畳など、フラット床以外 |
| **速度** | 犬並みの移動速度（後述） |
| **安定性** | 転倒せず、荷重変動や外乱にも耐える |
| **実用性** | 机上シミュレーションだけでなく、実機デモがあることが望ましい |

### 1.2 「犬の速度」とはどのくらいか

**【事実】** 一般論として、ペット犬の走行速度はおおよそ **15–20 mph（約 6.7–8.9 m/s）** とされる（犬種・個体差・距離による）。中型の運動犬（ボーダーコリー、ジャーマンシェパード等）は **約 30 mph（約 13.4 m/s）** 程度まで。グレーハウンドは **45 mph（約 20 m/s）** 級。

**【事実】** 四足ロボット論文で報告されている速度の例：

| 論文・システム | 報告速度 | 地形 |
|----------------|----------|------|
| MIT Cheetah 3 Convex MPC (2018) | 最大 **3.0 m/s**（前進） | 主に平坦〜中程度（論文は多様ゲイトだが不整地特化ではない） |
| ANYmal parkour (2024) | 最大 **2.0 m/s** | パルクール障害物連続 |
| RL-augmented MPC (2023) | ピーク **3.0 m/s** | 盲歩行＋階段 |
| Lee et al. blind RL (2020) | 明示的数値は少ない | 泥・雪・瓦礫など野外 |

**【推測】** 「犬の速度」を **日常の散歩〜小走り（1.5–3 m/s）** と解釈するのが現実的。競走犬クラス（10 m/s超）を四足ロボで不整地維持するのは、2026年時点では研究最前線を超える可能性が高い。

**【推測】** ユーザーのADAS操舵MPCで1年かかった経験から、**実機で安定して 2 m/s 前後の不整地歩行** を達成できれば、かなり良い成果と言える。

---

## 2. 初心者向け：MPCと足式ロボット制御の基礎

### 2.1 MPC（Model Predictive Control）とは

MPCは「**未来をちょっと先まで予測して、今の操作量を最適化する**」制御方式。

```
┌─────────────────────────────────────────────────────┐
│  現在の状態 x(t)                                       │
│       ↓                                              │
│  モデルで N ステップ先まで予測  x(t+1), x(t+2), ...   │
│       ↓                                              │
│  コスト（速度追従・エネルギー・制約違反など）を最小化   │
│       ↓                                              │
│  最適な u(t) だけ実行 → 次の周期で再計算（再帰的）     │
└─────────────────────────────────────────────────────┘
```

ADAS操舵MPCでも同じ発想だが、足式ロボットでは **接触（地面との衝突）** が支配的な違いになる。

### 2.2 足式ロボット制御の典型的な階層構造

```
┌──────────────────────────────────────────┐
│  高レベル：速度指令・経路計画・技能選択      │
├──────────────────────────────────────────┤
│  中レベル：MPC / RL  →  地面反力(GRF)計画  │
├──────────────────────────────────────────┤
│  低レベル：Whole-Body Control (WBC)       │
│           関節トルクへ変換 (QP)            │
├──────────────────────────────────────────┤
│  最下層：  モータPD / トルク制御            │
└──────────────────────────────────────────┘
```

**【事実】** 多くのMPC論文は **SRB（Single Rigid Body：剛体1個モデル）** で地面反力を最適化し、別のWBCで関節トルクに落とす。これがMIT Cheetah系の定番アーキテクチャ。

### 2.3 用語ミニ辞典

| 用語 | 意味 |
|------|------|
| **GRF (Ground Reaction Force)** | 足と地面の間の力。MPCが最適化する主対象 |
| **SRB / SRBD** | ロボット全体を1個の剛体として近似したモデル |
| **Friction cone** | 地面反力が滑らない範囲の制約（摩擦円錐） |
| **Contact schedule** | どの足がいつ地面につくか（接触スケジュール） |
| **WBC** | 地面反力→関節トルクへの変換（二次計画問題が多い） |
| **Sim-to-real** | シミュレーションで学習→実機へ転移 |
| **Privileged learning** | 学習時だけ地形の「正解情報」を使い、実行時は本体感覚のみ |
| **Domain randomization** | シミュレーションの物理パラメータをランダム化して頑健性向上 |

---

## 3. なぜMPCは難しいのか（ADAS操舵との比較）

ADAS操舵MPC経験者向けに、足式ロボットMPCの「追加で効いてくる難しさ」を整理する。

| 観点 | ADAS操舵MPC | 四足歩行MPC |
|------|-------------|-------------|
| **接触** | タイヤは常に接触（単純） | 足の離地・着地で接触が切り替わる（**ハイブリッド系**） |
| **モデル次元** | 車両モデル（低次元） | 12–18 DOF以上、またはSRB近似＋WBC |
| **制約** | 舵角・舵角速度・横G | 摩擦円錐、足位置、関節限界、自己衝突 |
| **非凸性** | 比較的扱いやすい | 接触切替で**非凸**。Convex化が研究の核心 |
| **知覚** | レーン認識（別モジュール） | 地形認識と制御が密結合（段差・足場） |
| **実装周期** | 10–50 Hz程度 | 20–500 Hz（方式による） |
| **チューニング** | 重み・モデルパラメータ | 上記＋接触スケジュール＋WBCゲイン |

**【事実】** Di Carlo et al. (2018) は、Convex MPCでも **1 ms 未満・20–30 Hz** で解けることを実証したが、これは **平坦〜中程度の地形** が主で、不整地特化ではない。

**【推測】** ADAS操舵MPCで1年かかった経験は、足式MPCでは **2–3年規模** になりうる。特に「不整地＋犬速度」を狙う場合、知覚パイプラインとロバスト化が追加コストになる。

**【推測】** ただし、2020年以降の **学習ベース＋MPCハイブリッド** や **SRB Convex MPCのオープンソース実装** により、ゼロからの開発時間は短縮可能。

---

## 4. 技術動向の全体像（2020–2026）

### 4.1 大きな流れ（3本柱）

```
                    ┌─────────────────┐
                    │  目標：不整地高速 │
                    └────────┬────────┘
           ┌─────────────────┼─────────────────┐
           ▼                 ▼                 ▼
    ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
    │  MPC系       │   │  RL系        │   │  ハイブリッド │
    │  モデル明示   │   │  経験から学習 │   │  MPC+RL      │
    └─────────────┘   └─────────────┘   └─────────────┘
```

### 4.2 MPC系の進化（時系列）

§5 の10論文は、この表の **各行＝1つの研究潮流** に対応している。  
「代表」列は §5 内の **§5.x 番号** を指す（詳細対応は [§4.5](#45-表と論文の対応マップ)）。

| 年代 | トレンド | 解決した課題 | 代表（§5） | 次の潮流への接続 |
|------|----------|--------------|------------|------------------|
| 2017–2019 | **Convex SRB-MPC** の確立 | 接触切替を固定化し、GRF最適化をConvex QP化 | **§5.1** Di Carlo | 足位置固定の限界 → §5.10 Kim |
| 2018–2019 | **足位置＋GRF同時最適化** | Convex MPCに足場計画を足す（知覚なし） | **§5.10** Kim (RPC) | 標高マップ統合 → §5.3 Jenelten |
| 2018–2020 | **Whole-body NMPC**、表現自由MPC | SRB近似の精度限界・姿勢特異点 | **§5.2** Neunert, **§5.4** Ding | 不整地は別問題 → §5.3 |
| 2020–2023 | **知覚統合NMPC**（足場最適化） | 標高マップ→踏可能領域→NMPC制約 | **§5.3** Jenelten → **§5.6** Grandia | モデル不確実性 → §5.7 Xu |
| 2022–2023 | **Whole-body BiConMP**、**Robust Convex MPC** | 全身NMPCの実時間化／Convex MPCのロバスト化 | **§5.5** Meduri, **§5.7** Xu | オンライン適応 → §5.8–§5.9 |
| 2023–2025 | **適応MPC**（L1, Chance-constrained） | 荷重・地形・摩擦のモデル不一致 | **§5.8** Sombolestan, **§5.9** CCMPC | RLとの統合 → [§7](#7-ハイブリッドmpcrlの潮流) |
| 2023– | **データ駆動補正**（GP, ARMAV） | 残差モデルでMPCをオンライン補正 | ※§5未収録（TR-MPC等） | §5.8/§5.9 と並行する研究線 |

**読み方:** 上から順に読むと、§5 の論文が **なぜ次の方式が必要になったか** の因果が追える。

```
§5.1 Convex SRB ──足位置固定の壁──▶ §5.10 RPC（足+GRF）
                                        │
§5.2/§5.4 全身・表現改善（並行線）        ▼
                              §5.3 Jenelten（標高マップ足場）
                                        │
                                        ▼
                              §5.6 Grandia（知覚NMPC完成形）
                                        │
                    ┌───────────────────┴───────────────────┐
                    ▼                                       ▼
           §5.5 BiConMP（全身NMPC）              §5.7 Xu（Robust Convex）
                    │                                       │
                    └───────────────────┬───────────────────┘
                                        ▼
                         §5.8 L1適応 / §5.9 Chance-constrained
                                        │
                                        ▼
                              §7 RL-augmented MPC 等
```

### 4.3 学習系の進化（時系列）

§6 の10論文は、この表の **各行＝1つの研究潮流** に対応している。  
「代表」列は §6 内の **§6.x 番号** を指す（詳細対応は [§4.5](#45-表と論文の対応マップ)）。

| 年代 | トレンド | 解決した課題 | 代表（§6） | 次の潮流への接続 |
|------|----------|--------------|------------|------------------|
| 2019 | **Sim-to-real RL** の実機成功 | シミュレーション止まり | **§6.1** Hwangbo | 平坦地限界 → §6.2 Lee |
| 2020 | **盲歩行RL**（本体感覚のみ） | カメラなし野外歩行 | **§6.2** Lee | 速度・先読み → §6.4 Miki |
| 2021 | **オンライン適応** | 荷重・摩耗・未知地形への追従 | **§6.3** RMA | MPC適応（§5.8）と対になるRL線 |
| 2022 | **知覚統合RL**（Attention） | 外覚失敗時のロバスト融合 | **§6.4** Miki | Grandia (§5.6) のRL版 |
| 2023 | **暗黙地形推定**、**MoB** | センサー簡素化／歩行戦略の切替 | **§6.5** DreamWaQ, **§6.6** Walk These Ways | 極限地形 → §6.7–§6.10 |
| 2024 | **パルクール**、**End-to-end視覚RL** | 連続障害物・跳躍・登攀 | **§6.7–§6.9** Extreme Parkour, ANYmal parkour, PIE | スパース足場 → §6.10 |

**読み方:** §6.1–§6.4 は「不整地歩行の基礎」、§6.5–§6.6 は「実装しやすい2023主流」、§6.7–§6.10 は「2024最前線」。

```
§6.1 Sim-to-real ──平坦限界──▶ §6.2 盲歩行（Lee）
                                    │
              ┌─────────────────────┼─────────────────────┐
              ▼                     ▼                     ▼
        §6.3 RMA（適応）      §6.5 DreamWaQ（暗黙推定）  §6.4 Miki（外覚+Attention）
              │                     │                     │
              │               §6.6 Walk These Ways（MoB） │
              │                     │                     │
              └─────────────────────┴──────────┬──────────┘
                                               ▼
                         §6.7–§6.9 パルクール系（2024）
                                               │
                                               ▼
                              §6.10 Terrain Reconstruction（スパース足場）
```

### 4.4 2024–2026の「最前線」キーワード

§4.2/§4.3 は2017–2024の **確立済み潮流** を時系列で整理した。  
ここでは、その延長線上にある **2024–2026時点で特に活発な研究テーマ** を、なぜ今重要か・何が解けたか・何が残っているかとともに説明する。

各キーワードと論文の対応は [§4.5](#45-表と論文の対応マップ) を参照。

---

#### Parkour / Extreme locomotion（パルクール・極限移動）

**何を指すか**  
連続する障害物（段差・隙間・登攀壁）を、歩行・跳躍・しゃがみ込みを切り替えながら **止まらず** 通過する歩行。通常の不整地歩行より、障害物の **形状・順序・タイミング** が制御のボトルネックになる。

**なぜ最前線か**  
§6.2 Lee の「盲歩行」や §5.6 Grandia の「粗地形登攀」が **単一スキル** だったのに対し、2024以降は **技能の連続切替** と **2 m/s級の速度維持** が同時に求められる段階に入った。

**主な論文:** §6.7 Extreme Parkour（End-to-end視覚RL）、§6.8 ANYmal parkour（階層RL、2.0 m/s）、§6.9 PIE（Implicit-Explicit）

**解決できたこと:** 事前地図・専門家デモなしで、未知コースへの sim-to-real 転移。低コスト四足＋Depthカメラでも実機パルクールが可能になった。

**残課題:** 「犬の日常速度（1.5–3 m/s）での長距離巡航」とは別問題（パルクールは瞬発力・技能切替が主）。転倒リスクが高く、安全保証は報酬設計依存。MPC側の明示的制約との統合は未成熟。

---

#### Implicit-explicit estimation（暗黙＋明示の地形推定）

**何を指すか**  
地形理解を **2段階** で行う発想。  
- **暗黙（Implicit）:** 本体感覚や潜在変数から「次に足を置くべき状態」を推定（地形の明示的地図を作らない）  
- **明示（Explicit）:** Depth等から heightmap や障害物形状を **中間表現として** 再構成する

**なぜ最前線か**  
Lee (§6.2) の完全盲歩行はセンサーが簡単だが速度に限界。Miki (§6.4) の外覚統合は高性能だがセンサー基盤が重い。**両者の中間** —— 安価なDepthでも、失敗時は本体感覚にフォールバック —— が2023–2024の実装トレンド。

**主な論文:** §6.5 DreamWaQ（暗黙のみ）、§6.9 PIE（暗黙+明示）、§6.10 Terrain Recon.（明示heightmap再構成）

**解決できたこと:** カメラなし（DreamWaQ）から、低コストDepth＋潜在推定（PIE）まで、 **センサーコストと性能のトレードオフ** を細かく設計できるようになった。

**残課題:** 雪・霧・反射面など外覚が壊れる条件での **定量評価** はMiki (§6.4) ほど体系的ではない。MPC側のSDF/凸制約への変換パイプラインは別途必要。

---

#### Chance-constrained / Tube-based robust MPC（確率・集合ロバストMPC）

**何を指すか**  
MPCの制約（特に摩擦円錐）に **不確実性** を組み込む方式。  
- **Chance-constrained:** 「制約違反確率 ≤ ε」として定式化（§5.9 CCMPC）  
- **Tube-based:** 状態が不確実性集合（チューブ）内に留まるよう制御（TR-MPC等、§5未収録）

**なぜ最前線か**  
§5.7 Xu の Min-max Robust MPC は保守的になりがちで、安全マージンの **手動チューニング** が必要だった。荷重変動・地形変動を **確率分布** として扱えば、チューニングなしで安全側に倒せる。

**主な論文:** §5.9 CCMPC（Chance-constrained）、§5.7 Xu（決定論的Robust、前段）

**解決できたこと:** 体重50%超の未知荷重を追加チューニングなしで対応（Go1実機）。ADAS MPC経験者にとって「確率制約」は比較的取り込みやすい拡張。

**残課題:** 速度面のベンチマークは荷重テスト中心。知覚統合（§5.6）との組み合わせは未整理。Tube-based系は§5未収録で、Convex QPとの求解速度比較が必要。

---

#### RL-augmented MPC（RLでMPCを拡張するハイブリッド）

**何を指すか**  
MPCの **構造（モデル・制約・GRF最適化）** は維持し、RLがMPC単体では扱いにくい部分 —— 足場反射（swing foot reflection）、stance制御、残差トルク —— を担う方式。

**なぜ最前線か**  
§5.8/§5.9 の適応MPCは **モデル構造が必要**。§6.3 RMA は適応できるが **安全制約が暗黙的**。2023年以降、「MPCで安全を担保しつつRLで表現力を足す」が ADAS MPC 経験者にも取り組みやすい折衷案として注目されている。

**主な論文:** [§7.2](#72-注目論文) RL-augmented MPC（3.0 m/s、盲階段）、IFM（MPC模倣→RL Finetune）

**解決できたこと:** Convex MPCの速度（3.0 m/s）を維持しつつ、盲階段登り・大荷重（83%）を実現。MPCの制約下でRLが動くため、純RLより安全側に倒しやすい。

**残課題:** 2系統（MPC+RL）の同期・チューニングコスト。ETH Grandia 級の知覚NMPCほどの **幾何学的足場保証** はRL側に依存。オープンソースはあるが、Walk These Ways ほど手軽ではない。

---

#### Proprioception-only with latent terrain（本体感覚のみ＋潜在地形）

**何を指すか**  
カメラ・LiDARなし（または実行時は使わない）で、 **関節トルク・IMU・足端力** などの本体感覚だけから、足元の地形（段差・軟弱地・障害物）を **潜在変数として推定** し歩行する方式。

**なぜ最前線か**  
産業四足（Unitree等）では **センサー追加コスト・外覚の故障・悪天候** を避けたい。Lee (§6.2) が実証したが、2023以降は DreamWaQ が **より軽量な実装** で同系統を発展させた。

**主な論文:** §6.2 Lee（Privileged learning の原点）、§6.5 DreamWaQ（暗黙地形想像）、§6.3 RMA（適応は別軸だが同じくproprio中心）

**解決できたこと:** 訓練未経験の泥・雪・瓦礫等への zero-shot 転移。カメラ配線・キャリブレーション不要で不整地デモが可能。

**残課題:** 明示的な速度報告が少ない（§5.1 の 3.0 m/s との直接比較は困難）。 **先読み** が効かないため、大段差・スパース足場では §6.4 Miki や §5.6 Grandia に劣る。潜在推定の解釈性・デバッグは難しい。

---

**【推測】** 産業応用（Unitree, ANYbotics等）では、**RLベース低レベル制御＋高レベルナビ** が主流になりつつあり、純MPCは研究・高信頼用途向けに残る。  
ユーザーのゴール（不整地＋犬速度）に直結しやすいのは **Proprioception-only 系（実装容易）** と **RL-augmented MPC（速度＋安全のバランス）** の2系統。

### 4.5 表と論文の対応マップ

§4.2/§4.3 の **潮流表（時系列）** と §5/§6/§7 の **個別論文** の1対1対応を一覧化する。

#### MPC系：§4.2 行 ↔ §5 論文

| §4.2 トレンド | §5 論文 | 解決課題 | 残課題 |
|---------------|---------|----------|--------|
| Convex SRB-MPC | **§5.1** Di Carlo | 接触切替を固定化しGRF最適化をConvex QP化。1 ms未満・20–30 Hz、3.0 m/s実証 | 足位置固定。不整地足場計画・モデル誤差・摩擦不確実性は未対応 |
| 足位置＋GRF同時最適化 | **§5.10** Kim (RPC) | §5.1の限界（足固定）を解消。足位置とGRFを予測ホライゾン上で同時最適化 | 知覚なし（標高マップ未統合）。非凸でConvex QPより重い |
| Whole-body NMPC | **§5.2** Neunert | 接触位置・タイミングも含む全身NMPCを190 Hz実時間実行 | 実装・ソフトウェア投資が大。不整地知覚は別モジュール必要 |
| 表現自由MPC | **§5.4** Ding | オイラー角/クォータニオンの特異点を回避。バク転等3D高ダイナミック動作 | 不整地足場計画は対象外。SRB Convex より用途が限定的 |
| 知覚統合NMPC（前半） | **§5.3** Jenelten | 標高マップからリアルタイム足場最適化。知覚→制御のオンライン統合の先駆け | 動力学は簡略化。NMPC全体統合は §5.6 まで未完成 |
| 知覚統合NMPC（完成） | **§5.6** Grandia | 標高→SDF→凸制約→NMPCの完全パイプライン。ギャップ・斜面・飛び石のdynamic climbing | 実装難易度が極めて高い。速度の定量報告は限定的。ETH級のエンジニアリング前提 |
| Whole-body BiConMP | **§5.5** Meduri | 双凸構造で全身12 DOF+を20 Hz実機最適化。外乱・地形ノイズにロバスト | 知覚統合なし。§5.1 Convex より計算コスト高 |
| Robust Convex MPC | **§5.7** Xu | 摩擦・モデル不確実性をMin-maxで明示。体重100%超荷重・滑り板でも安定 | 保守的になりがち。知覚統合は §5.6 と別系統 |
| 適応MPC（L1） | **§5.8** Sombolestan | L1 adaptiveで未知地形衝撃・荷重変動にオンライン適応。fast trot on uneven terrain | MPCのモデル構造は依然必要。Grandia級の足場幾何保証は別問題 |
| 適応MPC（Chance-constrained） | **§5.9** CCMPC | 荷重・地形変動を確率制約で扱い、追加チューニングなしで安全側に倒す | 検証は荷重テスト中心（0.25 m/s等）。高速不整地との組合せは未検証 |
| データ駆動補正 | ※未収録（TR-MPC等） | GP/残差モデルでMPC予測をオンライン補正する研究線 | §5本文未収録。§5.8/§5.9・§7ハイブリッドと並行開発中 |

#### 学習系：§4.3 行 ↔ §6 論文

| §4.3 トレンド | §6 論文 | 解決課題 | 残課題 |
|---------------|---------|----------|--------|
| Sim-to-real RL | **§6.1** Hwangbo | シミュレーションのみで学習→ANYmal実機転移。25 µs推論、転倒復帰 | 主に平坦〜中程度地形。不整地・速度の定量報告は限定的 |
| 盲歩行RL | **§6.2** Lee | 本体感覚のみで泥・雪・瓦礫等へzero-shot転移。カメラ不要 | 速度数値の明示報告が少ない。大段差・スパース足場は苦手 |
| オンライン適応 | **§6.3** RMA | 数秒で荷重・滑り・地形変化に適応。Reference trajectory不要 | 安全制約は報酬依存。MPC的な摩擦円錐保証はなし |
| 知覚統合RL | **§6.4** Miki | Attentionで外覚+本体感覚を融合。雪・霧で外覚をdiscount。アルプス遠足 | センサー基盤・学習パイプラインが重い。§5.6 Grandia と同等級の投資 |
| 暗黙地形推定 | **§6.5** DreamWaQ | カメラなしで潜在地形推定。Lee (§6.2) より実装が軽量 | 先読み不可。Miki/Grandia ほどの高速・極限地形は未達 |
| MoB | **§6.6** Walk These Ways | 1ポリシーに複数歩行戦略を埋込み、ジョイスティック的にリアルタイム調整 | 分布外環境では限界。安全保証は暗黙的。再学習なし汎化には限界 |
| End-to-end視覚RL | **§6.7** Extreme Parkour | Depth 1台→NN直接制御。3090で10–20時間学習、高跳び・長跳び | パルクール特化。長距離巡航・安全保証・解釈性は弱い |
| パルクール（階層RL） | **§6.8** ANYmal parkour | 知覚・技能・ナビの3モジュール階層RL。2.0 m/s連続障害物走破 | 事前地図不要だがANYmalエコシステム依存。一般四足への転用は非自明 |
| Implicit-Explicit | **§6.9** PIE | 暗黙+明示の二段推定。低コストDepthでパルクール、zero-shot転移 | §6.4 Miki ほどの悪天候評価は限定的。MPC制約との統合なし |
| スパース足場 | **§6.10** Terrain Recon. | Depth+proprioから局所heightmap再構成。飛び石等の危険地形 | End-to-endだが中間表現依存。Grandia級の幾何保証は学習結果に依存 |

#### MPC vs RL：同じ問題を別アプローチで攻めたペア

不整地・知覚・適応の **同じ課題** を、MPC系（§5）と学習系（§6）が別ルートで解いている。  
§4.2/§4.3 を横並びで読むと、この対応が見える。

| 課題 | MPC側（§5） | RL側（§6） | ハイブリッド（§7） |
|------|-------------|------------|-------------------|
| **足場計画** | **§5.3→§5.6:** 標高マップから踏可能領域・SDFを抽出し、NMPCの凸制約として足位置を **幾何学的に** 最適化。飛び石・ギャップで実証。 | **§6.10:** Depth+proprioから局所heightmapを **学習で** 再構成し、Locomotion policyへ。低コスト四足向け。幾何保証は学習結果依存。 | **RL-augmented MPC:** RLが **swing foot reflection**（足場反射）を担当。MPCがGRF・摩擦制約を担保。盲階段登りを実証。 |
| **知覚統合** | **§5.6 Grandia:** 知覚パイプライン（標高→平面分割→SDF）の出力を **NMPC制約に直接埋込**。外覚失敗時のフォールバック設計は別途必要。 | **§6.4 Miki:** 外覚+本体感覚を **Attention** で重み付け融合。雪・霧・反射面では外覚を自動discount。End-to-end学習。 | **—** （2026時点で知覚統合のMPC+RL一体型は §7未成熟。RL-augmented MPCは主にproprio+反射） |
| **モデル/環境適応** | **§5.8 L1, §5.9 CCMPC:** モデル不確実性を **L1 adaptive** または **確率制約** で明示的に扱う。荷重50–100%超に対応。制約の解釈性が高い。 | **§6.3 RMA:** 本体感覚履歴から潜在環境表現を推定し、 **数秒で** ポリシーを適応。モデル式不要だが安全は報酬設計依存。 | **RL-augmented MPC:** MPCがベースダイナミクスを保持し、RLが **stance制御・残差** で適応。83%荷重・盲階段を実証。 |
| **ロバスト性（摩擦・荷重）** | **§5.7 Xu, §5.9 CCMPC:** 摩擦円錐・荷重変動を **Min-max** または **Chance constraint** で硬く保証。滑り板・泥・草地で検証。保守的になりうる。 | **§6.6 Walk These Ways:** MoBで **複数歩行戦略** を1ポリシーに埋込み、リアルタイムでスタイル切替。明示的摩擦保証はなく、Domain randomizationで頑健化。 | **IFM:** DDP+MPCをExpertとして **模倣→RL Finetune**。MPCの制約構造を間接的に継承。7.5 cm障害物越え。 |
| **高速動的歩行** | **§5.1 Di Carlo:** Convex SRB-MPCで **3.0 m/s** 前進を平坦〜中程度地形で実証。1 ms未満求解。足位置固定が前提。 | **§6.1 Hwangbo:** 従来比高速走行・転倒復帰を実証。数値報告は限定的で主に平坦地。 **§6.8** が不整地2.0 m/s（パルクール特化）。 | **RL-augmented MPC:** Convex MPCの **3.0 m/s** を維持しつつ盲階段・大荷重に拡張。MPCの速度性能+RLの表現力。 |
| **実装コスト低** | **§5.1 OSS系:** SRB-MPC+WBCは **解釈性・チューニング** は分かりやすいが、WBC・接触スケジュールの実装で **数ヶ月〜年** 規模。不整地は §5.6 でさらに増大。 | **§6.6 Walk These Ways:** Go1/A1向け **オープンソース** 。平坦地学習→階段等へ再学習なし。個人プロジェクトの第一候補。 | **RL-augmented MPC:** Walk These Ways ほど手軽ではないが、MPC経験者には **既存Convex MPCへのRL追加** として取り組みやすい。OSSあり。 |

---

## 5. MPC系 主要論文10選

> **§4.2との関係:** 以下10論文は [§4.2 MPC系の進化](#42-mpc系の進化時系列) の各行に対応する。  
> 各論文冒頭の **「§4.2対応」** を見れば、§4.2 表上の位置づけが分かる。  
> 因果関係（なぜ次の論文が生まれたか）は §4.2 のフロー図を参照。

各論文について **背景・目的・課題設定・ブレイクスルー・結論** を整理。  
速度・実機結果は論文記載に基づく（**【事実】**）。解釈は **【推測】** と明記。

---

### 5.1 Di Carlo et al. (2018) — Convex MPC の原点

**§4.2対応:** 2017–2019「Convex SRB-MPC の確立」｜**次に読む:** §5.10（足位置拡張）→ §5.7/§5.8（ロバスト化）

| 項目 | 内容 |
|------|------|
| **タイトル** | Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control |
| **venue** | IROS 2018 |
| **DOI** | [10.1109/IROS.2018.8594448](https://doi.org/10.1109/IROS.2018.8594448) |

**背景**  
足式ロボットの動的歩行は、接触切替を含む非線形・非凸問題。従来はQPベースの瞬時バランス制御か、固定足位置のMPC。

**目的**  
3D空間での多様なゲイト（trot, bound, gallop等）を **同一フレームワーク・同一ゲイン** で安定化。

**課題設定**  
- モデル：SRB（Single Rigid Body）＋固定接触スケジュール  
- 最適化変数：各足の地面反力  
- 制約：摩擦円錐、GRF上下限  
- 問題を **Convex QP** に落としてリアルタイム求解

**ブレイクスルー**  
- 予測ホライゾン 0.5 s を **1 ms 未満・20–30 Hz** で求解  
- 実機で trot, bound, gallop 等を実証  
- 最大 **3.0 m/s** 前進、**1.0 m/s** 横移動、**180 deg/s** 旋回

**結論**  
Convex SRB-MPCは足式ロボ動的歩行の実用基盤になりうる。ただし **足位置は固定** で、不整地の足場計画は別問題。

**【推測】** この論文のアーキテクチャ（SRB-MPC + WBC）は、2026年時点でも教育・実装の第一歩として最適。

---

### 5.2 Neunert et al. (2018) — Whole-Body NMPC

**§4.2対応:** 2018–2020「Whole-body NMPC」｜**関連:** §5.5 Meduri（BiConMPで同系統を実用化）

| 項目 | 内容 |
|------|------|
| **タイトル** | Whole-Body Nonlinear Model Predictive Control Through Contacts for Quadrupeds |
| **venue** | IEEE RA-L 2018 |
| **DOI** | [10.1109/LRA.2018.2800124](https://doi.org/10.1109/LRA.2018.2800124) |

**背景**  
SRB近似は計算が速いが、全身動力学・接触タイミングを無視する。Whole-body NMPCは理論的には強力だが、計算時間が問題。

**目的**  
接触位置・順序・タイミングも含めて **全身NMPC** をリアルタイムで回す。

**課題設定**  
- 完全動力学モデル＋明示的接触モデル  
- 接触スケジュールは **最適化変数**  
- Auto-differentiation + コード生成で高速化

**ブレイクスルー**  
- **190 Hz** で 0.5 s ホライゾンを求解（当時のSOTA比で1桁以上高速）  
- HyQ, ANYmal の2機種で実機検証  
- 外乱付加時のリプランニングも実証

**結論**  
Whole-body NMPCの実時間実行は可能。ただし実装・ソフトウェア工学の投資が大きい。

**【推測】** 不整地より「跳躍・ダイナミックアクロバティック」向き。Convex MPCよりチューニング・計算コストは高い。

---

### 5.3 Jenelten et al. (2020) — オンライン足場最適化

**§4.2対応:** 2020–2023「知覚統合NMPC（前半）」｜**前段:** §5.10 Kim ｜**後続:** §5.6 Grandia ｜**RL対:** §6.10 Terrain Recon.

| 項目 | 内容 |
|------|------|
| **タイトル** | Perceptive Locomotion in Rough Terrain – Online Foothold Optimization |
| **venue** | IEEE RA-L 2020 |
| **DOI** | [10.1109/LRA.2020.3007427](https://doi.org/10.1109/LRA.2020.3007427) |

**背景**  
不整地では「どこに足を置くか」が速度・安定性を左右する。固定足位置のMPCでは限界。

**目的**  
標高マップから **リアルタイムで足場を最適化** し、粗地形歩行を実現。

**課題設定**  
- 知覚：Elevation map  
- 最適化：足位置＋（簡略化された）動力学  
- ANYmal プラットフォーム

**ブレイクスルー**  
- 知覚と制御を **オンライン統合** した先駆的研究  
- Grandia et al. (2023) NMPCの前身

**結論**  
不整地MPCには **知覚パイプラインが必須**。足場最適化なしにConvex MPCだけでは不十分。

---

### 5.4 Ding et al. (2021) — Representation-Free MPC

**§4.2対応:** 2018–2020「表現自由MPC」｜**関連:** §5.2 Neunert（並行するWhole-body系）

| 項目 | 内容 |
|------|------|
| **タイトル** | Representation-Free Model Predictive Control for Dynamic Motions in Quadrupeds |
| **venue** | IEEE TRO 2021 |
| **DOI** | [10.1109/TRO.2020.3046415](https://doi.org/10.1109/TRO.2020.3046415) |

**背景**  
オイラー角・クォータニオンによる姿勢表現は、3D動作（バク転等）で特異点問題を起こす。

**目的**  
回転行列を直接使い、**特異点のないMPC** で3D動的動作を制御。

**課題設定**  
- Variation-based linearization (VBL) で回転ダイナミクスを線形化  
- QP形式に落として **250 Hz** 実行

**ブレイクスルー**  
- 周期ゲイト＋**制御されたバク転** を実機で実証  
- 3D動作でのMPC安定性向上

**結論**  
表現の選び方がMPCの実用性を左右する。不整地歩行より **高ダイナミック動作** に強み。

---

### 5.5 Meduri et al. (2023) — BiConMP

**§4.2対応:** 2022–2023「Whole-body BiConMP」｜**前段:** §5.2 Neunert ｜**対:** §5.1 Convex（速度重視なら§5.1の方が軽い）

| 項目 | 内容 |
|------|------|
| **タイトル** | BiConMP: A Nonlinear Model Predictive Control Framework for Whole Body Motion Planning |
| **venue** | IEEE TRO 2023 |
| **DOI** | [10.1109/TRO.2022.3228390](https://doi.org/10.1109/TRO.2022.3228390) |
| **Code** | [machines-in-motion/biconvex_mpc](https://github.com/machines-in-motion/biconvex_mpc) |

**背景**  
Whole-body NMPCは非線形だが、ロボットダイナミクスには **双凸構造** が存在する。

**目的**  
双凸性を利用した **Whole-body NMPC** を20 Hzで実機実行。

**課題設定**  
- ADMM による双凸最適化  
- Solo12 四足で実機、trot/jump/bound  
- AnYmal, Talos でもシミュレーション検証

**ブレイクスルー**  
- Convex化せずに **全身12 DOF以上** をリアルタイム最適化  
- 外乱・地形ノイズへのロバスト性実証  
- 非周期動作（ハイタッチ等）も生成

**結論**  
SRB-MPCの次のステップとして、Whole-body NMPCの実用化に道筋。計算はConvex MPCより重い。

---

### 5.6 Grandia et al. (2023) — 知覚統合NMPC

**§4.2対応:** 2020–2023「知覚統合NMPC（完成）」｜**前段:** §5.3 Jenelten ｜**RL対:** §6.4 Miki ｜**§4.4:** Perceptive locomotion

| 項目 | 内容 |
|------|------|
| **タイトル** | Perceptive Locomotion Through Nonlinear Model-Predictive Control |
| **venue** | IEEE TRO 2023 (Vol.39, No.5) |
| **DOI** | [10.1109/TRO.2023.3275384](https://doi.org/10.1109/TRO.2023.3275384) |

**背景**  
粗地形では足場選定・衝突回避・ダイナミクスを **同時に** 最適化する必要がある。

**目的**  
知覚情報をNMPCに埋め込み、**全自由度をリアルタイム最適化**。

**課題設定**  
- 標高マップ → 踏可能領域分類、平面分割、SDF（符号付き距離場）  
- 凸不等式制約としてNMPCに埋込  
- Multiple-shooting + Real-time iteration + Filter line search

**ブレイクスルー**  
- ギャップ・斜面・飛び石での **dynamic climbing** を実機ANYmalで実証  
- 知覚→制約抽出→NMPC の完全パイプライン

**結論**  
**不整地＋MPC** の現時点での代表格。実装難易度は非常に高い。

**【推測】** 「犬速度の不整地歩行」をMPCで狙うなら、この系統が理論的には最も近い。ただしETHレベルのエンジニアリング投資が必要。

---

### 5.7 Xu et al. (2023) — Robust Convex MPC

**§4.2対応:** 2022–2023「Robust Convex MPC」｜**前段:** §5.1 Di Carlo ｜**後続:** §5.9 CCMPC（確率版ロバスト）

| 項目 | 内容 |
|------|------|
| **タイトル** | Robust Convex Model Predictive Control for Quadruped Locomotion Under Uncertainties |
| **venue** | IEEE TRO 2023 (Vol.39, No.6) |
| **DOI** | [10.1109/TRO.2023.3299527](https://doi.org/10.1109/TRO.2023.3299527) |

**背景**  
Convex MPCは摩擦係数・モデルパラメータの不確実性に弱い。荷重変動や滑りで転倒しやすい。

**目的**  
摩擦制約・モデルダイナミクスの不確実性を明示的に扱う **Robust Convex MPC**。

**課題設定**  
- Min-max 最適化 → **Convex QQCQP** に等価変換  
- 2段階最適化で求解速度向上（Gurobi比 **約11倍**）

**ブレイクスルー**  
- ロボット重量 **100%超の荷重** でも安定（実機・シミュレーション）  
- 滑り板の上でも加速可能  
- 泥・急斜面・草地等の野外地形で検証

**結論**  
Di Carlo (2018) のConvex MPCに **ロバスト性レイヤー** を追加する実用的方向性。

**【推測】** ADAS MPC経験者には、Min-max / Robust optimization の発想は馴染みやすい可能性。

---

### 5.8 Sombolestan & Nguyen (2024) — Adaptive Force-Based MPC

**§4.2対応:** 2023–2025「適応MPC（L1）」｜**前段:** §5.1/§5.7 ｜**RL対:** §6.3 RMA ｜**§7:** RL-augmented MPC

| 項目 | 内容 |
|------|------|
| **タイトル** | Adaptive Force-Based Control of Dynamic Legged Locomotion over Uneven Terrain |
| **venue** | IEEE TRO 2024 (Vol.40) |
| **DOI** | [10.1109/TRO.2024.3381554](https://doi.org/10.1109/TRO.2024.3381554) |
| **arXiv** | [2307.04030](https://arxiv.org/abs/2307.04030) |

**背景**  
MPCは「モデルが完璧」という前提。実際には地形・荷重・接触モデルの不一致がある。

**目的**  
**L1 adaptive control** を force-based MPC に統合し、モデル不確実性と未知地形衝撃に適応。

**課題設定**  
- ベース：Convex MPC + force control  
- 追加：L1 adaptive モジュール  
- Unitree A1 で実機検証

**ブレイクスルー**  
- 体重 **50%** の荷重を載せた状態で fast trot, bounding on uneven terrain  
- 準静的歩行ではなく **ダイナミックゲイト** を維持

**結論**  
MPCの弱点（モデル依存）を **適応制御** で補う方向。MPC経験者にとって理解しやすい拡張。

---

### 5.9 Chance-Constrained MPC (2024) — CCMPC

**§4.2対応:** 2023–2025「適応MPC（Chance-constrained）」｜**前段:** §5.7 Xu ｜**§4.4:** Chance-constrained robust MPC

| 項目 | 内容 |
|------|------|
| **タイトル** | Chance-Constrained Convex MPC for Robust Quadruped Locomotion Under Parametric and Additive Uncertainties |
| **venue** | arXiv 2024 (実機Go1) |
| **URL** | [https://cc-mpc.github.io/](https://cc-mpc.github.io/) |
| **arXiv** | [2411.03481](https://arxiv.org/abs/2411.03481) |

**背景**  
ロバストMPCの安全マージン調整は経験的で、荷重・地形変動に追従しにくい。

**目的**  
荷重・地形変動を **確率分布** としてモデル化し、Chance constraint で安全を保証。

**課題設定**  
- SRBD + パラメトリック/加法的扰動  
- 摩擦円錐を chance constraint として定式化  
- **Convex QP** として求解

**ブレイクスルー**  
- 追加チューニングなしで、体重 **50%超** の未知荷重に対応（Go1実機）  
- Linear MPC (LMPC) が失敗する荷重でもCCMPCは成功

**結論**  
不確実性を「確率」で扱うMPC。安全クリティカル用途に有望。

---

### 5.10 Kim et al. (2019) — 足位置＋GRF同時最適化 (RPC)

**§4.2対応:** 2018–2019「足位置＋GRF同時最適化」｜**前段:** §5.1 Di Carlo ｜**後続:** §5.3 Jenelten → §5.6 Grandia

| 項目 | 内容 |
|------|------|
| **タイトル** | Implementing Regularized Predictive Control for Simultaneous Real-Time Footstep and Ground Reaction Force Optimization |
| **venue** | IROS 2019 |
| **DOI** | [10.1109/IROS40897.2019.8968031](https://doi.org/10.1109/IROS40897.2019.8968031) |

**背景**  
Convex MPC (2018) は足位置固定。不整地では足位置も最適化したい。

**目的**  
**足位置と地面反力を同時に** 予測ホライゾン上で最適化する Regularized Predictive Control (RPC)。

**課題設定**  
- 非線形最適化（正則化付き）  
- MIT Cheetah 3 実機  
- ヒューリスティック正則化で高忠実度モデル不要

**ブレイクスルー**  
- Convex MPCから **足場計画込み** への拡張を実機で実証  
- Grandia (2023) 系の足位置最適化の前段

**結論**  
不整地MPCでは「足をどこに置くか」と「どれだけ力を出すか」は不可分。

---

### MPC系 まとめ表

| # | 論文 | §4.2トレンド | 年 | モデル | 知覚 | 実機 | 不整地 | 報告速度 |
|---|------|--------------|-----|--------|------|------|--------|----------|
| 1 | Di Carlo (Cheetah3) | Convex SRB-MPC | 2018 | Convex SRB | × | ○ | △ | 3.0 m/s |
| 2 | Neunert (Whole-body) | Whole-body NMPC | 2018 | Full NMPC | × | ○ | △ | — |
| 3 | Jenelten (Foothold) | 知覚統合（前半） | 2020 | Foothold opt | ○ | ○ | ○ | — |
| 4 | Ding (RF-MPC) | 表現自由MPC | 2021 | SRB/NMPC | × | ○ | △ | — |
| 5 | Meduri (BiConMP) | BiConMP | 2023 | Whole-body | × | ○ | △ | 20 Hz |
| 6 | Grandia (Perceptive) | 知覚統合（完成） | 2023 | NMPC | ○ | ○ | ◎ | dynamic climb |
| 7 | Xu (Robust) | Robust Convex | 2023 | Convex SRB | × | ○ | ○ | — |
| 8 | Sombolestan (Adaptive) | 適応MPC（L1） | 2024 | Convex+L1 | × | ○ | ○ | dynamic |
| 9 | CCMPC | 適応MPC（Chance） | 2024 | Convex SRB | × | ○ | ○ | 0.25 m/s (load test) |
| 10 | Kim (RPC) | 足+GRF同時最適化 | 2019 | Regularized | △ | ○ | ○ | — |

◎=強い、○=あり、△=限定的、×=なし

---

## 6. 学習系 主要論文10選

> **§4.3との関係:** 以下10論文は [§4.3 学習系の進化](#43-学習系の進化時系列) の各行に対応する。  
> 各論文冒頭の **「§4.3対応」** を見れば、§4.3 表上の位置づけが分かる。  
> MPC側の対応論文は [§4.5 MPC vs RL ペア表](#mpc-vs-rl同じ問題を別アプローチで攻めたペア) を参照。

---

### 6.1 Hwangbo et al. (2019) — Sim-to-real RL の突破口

**§4.3対応:** 2019「Sim-to-real RL」｜**次に読む:** §6.2 Lee（不整地へ）

| 項目 | 内容 |
|------|------|
| **タイトル** | Learning agile and dynamic motor skills for legged robots |
| **venue** | Science Robotics 2019 |
| **DOI** | [10.1126/scirobotics.aau5872](https://doi.org/10.1126/scirobotics.aau5872) |

**背景**  
足式ロボットのRLはシミュレーション止まりが多く、実機成功例が少なかった。

**目的**  
シミュレーションのみで学習したNN制御則を **ANYmal実機** に転移。

**課題設定**  
- 報酬：速度追従、エネルギー、安定性  
- Domain randomization  
- 比較的小さなNN（推論 **25 µs**/CPU thread）

**ブレイクスルー**  
- 高精度速度追従、従来より高速走行、**転倒からの復帰**  
- MPCと異なり **外部PC不要** でオンボード実行可能

**結論**  
RLでも動的・敏捷な足式歩行は可能。ただし主に **平坦〜中程度** の地形。

---

### 6.2 Lee et al. (2020) — 盲歩行RL（野外）

**§4.3対応:** 2020「盲歩行RL」｜**前段:** §6.1 ｜**§4.4:** Proprioception-only ｜**MPC対:** §5.3/§5.6（知覚あり版）

| 項目 | 内容 |
|------|------|
| **タイトル** | Learning quadrupedal locomotion over challenging terrain |
| **venue** | Science Robotics 2020 |
| **DOI** | [10.1126/scirobotics.abc5986](https://doi.org/10.1126/scirobotics.abc5986) |

**背景**  
状態マシン＋反射ベースの従来制御は複雑化するが汎用性に欠ける。

**目的**  
**本体感覚（proprioception）のみ** で、泥・雪・瓦礫・vegetation等の野外を歩行。

**課題設定**  
- Privileged learning：Teacher（地形正解あり）→ Student（本体感覚のみ）  
- Adaptive curriculum で地形難易度を自動調整  
- ANYmal C / D

**ブレイクスルー**  
- **Zero-shot sim-to-real** で訓練未経験の自然環境を歩行  
- 従来の足式ロボ研究を超える野外デモ

**結論**  
「シンプルなシミュレーションで学習→複雑な現実へ」は可能。**カメラ不要** で不整地対応。

**【推測】** 速度面ではMPC/Cheetah3ほどの数値報告はないが、**汎用性・開発速度** で優位。

---

### 6.3 Kumar et al. (2021) — RMA（オンライン適応）

**§4.3対応:** 2021「オンライン適応」｜**MPC対:** §5.8 Sombolestan, §5.9 CCMPC

| 項目 | 内容 |
|------|------|
| **タイトル** | RMA: Rapid Motor Adaptation for Legged Robots |
| **venue** | RSS 2021 |
| **arXiv** | [2107.04034](https://arxiv.org/abs/2107.04034) |

**背景**  
固定ポリシーは荷重変動・摩耗・未知地形に弱い。

**目的**  
**数秒以内** に環境変化（地形、荷重、滑り）に適応するRL。

**課題設定**  
- Base policy + Adaptation module  
- 本体感覚履歴から潜在環境表現を推定  
- Unitree A1、Fine-tuning なしで実機デプロイ

**ブレイクスルー**  
- 岩・滑り・草・階段・油汚れ板等で実機検証  
- Reference trajectory 不要

**結論**  
「学習＋適応」で、MPCのモデル更新問題を回避するアプローチ。

---

### 6.4 Miki et al. (2022) — 知覚統合RL

**§4.3対応:** 2022「知覚統合RL（Attention）」｜**MPC対:** §5.6 Grandia ｜**後続:** §6.9 PIE

| 項目 | 内容 |
|------|------|
| **タイトル** | Learning robust perceptive locomotion for quadrupedal robots in the wild |
| **venue** | Science Robotics 2022 |
| **DOI** | [10.1126/scirobotics.abk2822](https://doi.org/10.1126/scirobotics.abk2822) |

**背景**  
Lee (2020) は盲歩行で速度に限界。外覚（カメラ等）を使えば先読み歩行が可能だが、視覚は雪・霧等で信頼性が落ちる。

**目的**  
外覚＋本体感覚を **Attention機構** で統合し、信頼性に応じて重み付け。

**課題設定**  
- Recurrent encoder + Attention  
- End-to-end学習（ヒューリスティック切替なし）  
- ANYmal

**ブレイクスルー**  
- アルプス登山コースを **人間の推奨時間内** で完走  
- 雪・霧・反射面でも外覚を「discount」して本体感覚に切替

**結論**  
不整地 **高速歩行** には外覚が有効だが、**外覚の失敗を前提とした設計** が鍵。

**【推測】** Grandia (2023) MPC版のRL対応版と言える。

---

### 6.5 Nahrendra et al. (2023) — DreamWaQ

**§4.3対応:** 2023「暗黙地形推定」｜**前段:** §6.2 Lee ｜**§4.4:** Implicit-explicit（暗黙側）

| 項目 | 内容 |
|------|------|
| **タイトル** | DreamWaQ: Learning Robust Quadrupedal Locomotion With Implicit Terrain Imagination via Deep Reinforcement Learning |
| **venue** | ICRA 2023 |
| **DOI** | [10.1109/ICRA48891.2023.10161144](https://doi.org/10.1109/ICRA48891.2023.10161144) |

**背景**  
知覚統合RLはセンサー基盤が複雑。カメラなしでも不整地を歩きたい。

**目的**  
**本体感覚のみ** で地形（高さ・摩擦・障害物）を **暗黙的に推定** して歩行。

**課題設定**  
- Asymmetric actor-critic（Criticは特権情報、Actorは部分観測）  
- PPO  
- Unitree A1

**ブレイクスルー**  
- カメラなしで丘・階段等を長距離歩行  
- Lee (2020) より **センサーコスト低**、Miki (2022) より **実装簡単**

**結論**  
「地形を見えないけど、足の感覚から推測して歩く」という生物学的アプローチのRL版。

---

### 6.6 Margolis et al. (2023) — Walk These Ways

**§4.3対応:** 2023「MoB」｜**§7:** RL-augmented MPC のベースポリシー候補

| 項目 | 内容 |
|------|------|
| **タイトル** | Walk These Ways: Tuning Robot Control for Generalization with Multiplicity of Behavior |
| **venue** | CoRL 2022 / PMLR 2023 |
| **Code** | [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways) |

**背景**  
学習済みポリシーは分布外環境で失敗したとき、再学習が必要。

**目的**  
**Multiplicity of Behavior (MoB)** — 1つのポリシーに複数の歩行戦略を埋め込み、**リアルタイムで切替・調整**。

**課題設定**  
- 命令ベクトル：足振り、姿勢、接触スケジュール、速度等  
- 平坦地で学習→階段・障害物等に **再学習なし** で対応  
- Unitree Go1

**ブレイクスルー**  
- オープンソースで **再現性の高い** 四足RL基盤  
- オペレータがジョイスティック的に歩行スタイルを調整可能

**結論**  
「1ポリシー＋リアルタイムチューニング」で汎化。開発者にとって実用的。

**【推測】** 個人プロジェクトの出発点として **最もコスパが良い** 学習系選択肢の一つ。

---

### 6.7 Cheng et al. (2024) — Extreme Parkour

**§4.3対応:** 2024「End-to-end視覚RL」｜**§4.4:** Parkour / Extreme locomotion

| 項目 | 内容 |
|------|------|
| **タイトル** | Extreme Parkour with Legged Robots |
| **venue** | ICRA 2024 |
| **arXiv** | [2309.14341](https://arxiv.org/abs/2309.14341) |
| **Code** | [chengxuxin/extreme-parkour](https://github.com/chengxuxin/extreme-parkour) |

**背景**  
パルクールは正確な視覚・制御の協調が必要。従来は段階的パイプライン。

**目的**  
低コスト四足＋前方Depthカメラ1台で **End-to-end RL** パルクール。

**課題設定**  
- Base policy（盲）→ Distillation policy（カメラ付き）  
- 大規模シミュレーションRL（3090 GPUで10–20時間）  
- 高跳び（体高2倍）、長跳び（体長2倍）等

**ブレイクスルー**  
- 単一NNがカメラ画像から直接制御  
- 実機で未知コースに汎化

**結論**  
「犬速度」より **アクロバティック** だが、視覚→制御のEnd-to-endの可能性を示した。

---

### 6.8 Hoeller et al. (2024) — ANYmal Parkour

**§4.3対応:** 2024「パルクール（階層RL）」｜**§4.4:** Parkour ｜**比較:** §6.7（End-to-end vs 階層）

| 項目 | 内容 |
|------|------|
| **タイトル** | ANYmal parkour: Learning agile navigation for quadrupedal robots |
| **venue** | Science Robotics 2024 |
| **DOI** | [10.1126/scirobotics.adi7566](https://doi.org/10.1126/scirobotics.adi7566) |

**背景**  
単一スキルでは連続障害物を越えられない。知覚・技能・ナビの統合が必要。

**目的**  
**階層型RL** で歩行・跳躍・登攀・しゃがみ込みを切替え、パルクール連続走破。

**課題設定**  
- 3モジュール：Perception（点群→地形推定）、Locomotion skills catalog、Navigation policy  
- シミュレーションのみで学習→実機転移

**ブレイクスルー**  
- 最大 **2.0 m/s** で連続障害物走破  
- 専門家デモ・オフライン計算・事前環境地図 **不要**

**結論**  
2024年時点の **学習系SOTA** の一つ。ETHのANYmalエコシステムの集大成。

---

### 6.9 Li et al. (2024) — PIE

**§4.3対応:** 2024「Implicit-Explicit」｜**前段:** §6.4 Miki ｜**§4.4:** Implicit-explicit estimation

| 項目 | 内容 |
|------|------|
| **タイトル** | PIE: Parkour with Implicit-Explicit Learning Framework for Legged Robots |
| **venue** | IEEE RA-L 2024 |
| **DOI** | [10.1109/LRA.2024.3459797](https://doi.org/10.1109/LRA.2024.3459797) |

**背景**  
パルクールRLは複雑な地形再構成モジュールか、知覚精度を制限して安全側に倒すかの二択だった。

**目的**  
**Implicit-Explicit（暗黙＋明示）** 二段推定で、低コストDepthカメラでもパルクール。

**課題設定**  
- 本体感覚＋外覚を統合  
- Successor state 推定（暗黙）＋ 地形理解（明示）  
- End-to-end、比較的シンプルな報酬

**ブレイクスルー**  
- 安価な四足＋信頼性低いDepthでも高パフォーマンス  
- Zero-shot 実機転移

**結論**  
Miki (2022) のAttention思想をパルクールに拡張した最新例。

---

### 6.10 Terrain Reconstruction (2024) — リスク地形歩行

**§4.3対応:** §4.3表には未記載だが、2024「スパース足場」系 ｜**MPC対:** §5.3 Jenelten, §5.6 Grandia

| 項目 | 内容 |
|------|------|
| **タイトル** | Walking with Terrain Reconstruction: Learning to Traverse Risky Sparse Footholds |
| **venue** | arXiv 2024 |
| **arXiv** | [2409.15692](https://arxiv.org/abs/2409.15692) |

**背景**  
スパースな足場（飛び石等）では正確な足配置が必要。DepthカメラのFOVは限られる。

**目的**  
Depth＋本体感覚から **局所heightmapを再構成** し、危険地形を歩行。

**課題設定**  
- Reconstructor：proprioception + temporal depth → local heightmap  
- Heightmapを中間表現としてLocomotion policyへ  
- 低コスト四足

**ブレイクスルー**  
- End-to-end RLだが、heightmap再構成で **解釈可能な中間表現**  
- スパース・ランダム足場で敏捷歩行

**結論**  
「完全End-to-end」より **中間表現付きEnd-to-end** が不整地で有効な傾向。

---

### 学習系 まとめ表

| # | 論文 | §4.3トレンド | 年 | センサ | 実機 | 不整地 | 速度報告 |
|---|------|--------------|-----|--------|------|--------|----------|
| 1 | Hwangbo | Sim-to-real | 2019 | Proprio | ○ | △ | 高速（数値限定的） |
| 2 | Lee | 盲歩行RL | 2020 | Proprio | ○ | ◎ | — |
| 3 | RMA | オンライン適応 | 2021 | Proprio | ○ | ◎ | — |
| 4 | Miki | 知覚統合RL | 2022 | Proprio+Vision | ○ | ◎ | 高速（Alpス遠足） |
| 5 | DreamWaQ | 暗黙地形推定 | 2023 | Proprio | ○ | ○ | — |
| 6 | Walk These Ways | MoB | 2023 | Proprio | ○ | ○ | 高速度走行可 |
| 7 | Extreme Parkour | End-to-end視覚RL | 2024 | Depth | ○ | ◎ | アクロバティック |
| 8 | ANYmal parkour | パルクール（階層） | 2024 | LiDAR+Cam | ○ | ◎ | 2.0 m/s |
| 9 | PIE | Implicit-Explicit | 2024 | Depth | ○ | ◎ | パルクール |
| 10 | Terrain Recon. | スパース足場 | 2024 | Depth+Proprio | ○ | ◎ | — |

---

## 7. ハイブリッド（MPC＋RL）の潮流

MPCとRLは対立ではなく、**2023年以降に統合が加速** している。

> **§4.2/§4.3との関係:** §4.2末尾（適応MPC）と §4.3末尾（パルクール）の **接続点** がここ。  
> §5.8/§5.9（MPC適応）と §6.3/§6.6（RL適応）の限界を、相互補完で埋める潮流。  
> §4.4 キーワード「RL-augmented MPC」の具体例は §7.2。

### 7.1 統合パターン

| パターン | 説明 | 代表論文 |
|----------|------|----------|
| **RL → MPC** | RLが高レベル指令（速度、接触タイミング、足位置）を生成、MPCが低レベル実行 | IFM (2023), RL-Augmented MPC (2026) |
| **MPC → RL** | MPCをExpertとして模倣→RLでFinetune | IFM (2023) |
| **RL residual on MPC** | MPC出力にRLがトルク補正を加算 | RL-augmented MPC (2023) |
| **RL terminal cost for MPC** | RLがMPCのTerminal cost / Q-functionを学習 | MPC+Predictive RL (2023) |

### 7.2 注目論文

**RL-augmented MPC (2023)** — [arXiv:2310.09442](https://arxiv.org/abs/2310.09442)

- RLが **stance foot control + swing foot reflection** を統合  
- Unitree A1：ピーク **3.0 m/s**、旋回 **8.5 rad/s**、**10 kg (83%)** 荷重  
- **盲階段登り** を実現  
- Go1, AlienGo へ zero-shot transfer

**IFM (2023)** — Imitating and Finetuning MPC — [10.1109/LRA.2023.3320827](https://doi.org/10.1109/LRA.2023.3320827)

- DDP+MPCをExpert → 模倣学習 → RL Finetune  
- Mini-Cheetah で **7.5 cm** 障害物越え

**【推測】** ADAS MPC経験者にとって、**RL-augmented MPC** または **IFM** は「MPCの延長」として取り組みやすい。RL単体より安全制約をMPC側で担保できる。

---

## 8. 方式選定の指針

### 8.1 ゴール別おすすめ

| ゴール | 第一候補 | 理由 |
|--------|----------|------|
| **最短で不整地歩行デモ** | Walk These Ways / DreamWaQ | オープンソース、A1/Go1対応 |
| **2 m/s級の野外歩行** | Miki (2022) 系 or RL-augmented MPC | 知覚統合 or MPC+RL |
| **理論・安全性重視** | Xu Robust MPC / CCMPC | 制約・不確実性を明示 |
| **段差・飛び石が多い** | Grandia NMPC or Terrain Reconstruction RL | 足場計画が核心 |
| **荷重変動が大きい** | Sombolestan Adaptive / RMA / CCMPC | 適応機構 |
| **アクロバティック** | ANYmal parkour / Extreme Parkour | 研究最前線 |

### 8.2 MPC vs RL — 正直な比較

| 観点 | MPC | RL |
|------|-----|-----|
| **開発時間（ゼロから）** | 長い（1–3年） | 中（数週間–数ヶ月、基盤利用時） |
| **解釈性** | 高い | 低い |
| **安全制約** | 明示的 | 報酬設計に依存 |
| **未知地形** | モデル・知覚が必要 | 汎化性能に依存 |
| **速度上限** | モデル精度に依存 | シミュレーション品質に依存 |
| **計算資源（実行時）** | QP/NLP求解 | NN推論（軽量） |
| **再現性** | 中（チューニング敏感） | 中–高（オープンソース増加） |

### 8.3 個人的見解（【推測】）

ADAS操舵MPCで1年かけた経験を踏まえると：

1. **純MPCで不整地＋犬速度** を一から作るのは、操舵MPCより **明らかに難しい**（接触＋知覚＋WBC）
2. ただし **Convex SRB-MPC + 既存OSS** から始め、**RLで足場反射や適応を足す** ハイブリッドが、2026年時点で最も現実的
3. ハードウェアが Unitree A1/Go1 なら **Walk These Ways → RL-augmented MPC** の順が学習コスト対効果が高い
4. ハードウェアが ANYmal クラスなら **ETH系（Miki, Grandia）** のエコシステムが参考になる

---

## 9. 事実と推測の区別

### 9.1 本文中の【事実】（論文・公開情報に基づく）

- MIT Cheetah 3 Convex MPC：最大3.0 m/s、1ms未満求解（Di Carlo 2018）
- ANYmal parkour：最大2.0 m/s（Hoeller 2024）
- RL-augmented MPC：ピーク3.0 m/s、10kg荷重（2023）
- Lee (2020)：泥・雪・瓦礫等のzero-shot野外歩行
- Miki (2022)：アルプス遠足を人間推奨時間内
- Xu Robust MPC：体重100%超荷重、Gurobi比11倍高速
- CCMPC：体重50%超未知荷重（Go1）
- 犬の平均走行速度：約6.7–8.9 m/s（一般論、犬種による）

### 9.2 本文中の【推測】（筆者の解釈・推論）

- 「犬の速度」を日常の1.5–3 m/sと解釈するのが現実的
- 純MPCゼロからの開発はADAS MPCより2–3年規模になりうる
- 2026年時点の産業界主流はRL系低レベル制御に傾いている
- 個人プロジェクトの第一選択は Walk These Ways または Convex MPC OSS
- 競走犬クラスの速度（10 m/s+）を不整地で維持するのは現時点で研究限界を超える可能性

### 9.3 検証していないこと

- 各論文の再現実験は行っていない
- 2025–2026の最新論文（TR-MPC 2025, All-Terrain Quadrupeds 2025等）は補足的情報のみ
- 商用製品（Unitree公式コントローラ等）の内部実装は非公開のため言及なし

---

## 10. 参考文献一覧

### MPC系

1. Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., & Kim, S. (2018). Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control. *IROS*. https://doi.org/10.1109/IROS.2018.8594448
2. Neunert, M., et al. (2018). Whole-Body Nonlinear Model Predictive Control Through Contacts for Quadrupeds. *IEEE RA-L*. https://doi.org/10.1109/LRA.2018.2800124
3. Jenelten, F., Miki, T., Vijayan, A. E., Bjelonic, M., & Hutter, M. (2020). Perceptive Locomotion in Rough Terrain – Online Foothold Optimization. *IEEE RA-L*. https://doi.org/10.1109/LRA.2020.3007427
4. Ding, Y., Pandala, A., Li, C., Shin, Y.-H., & Park, H.-W. (2021). Representation-Free Model Predictive Control for Dynamic Motions in Quadrupeds. *IEEE TRO*. https://doi.org/10.1109/TRO.2020.3046415
5. Meduri, A., Shah, P., Viereck, J., Khadiv, M., Havoutis, I., & Righetti, L. (2023). BiConMP: A Nonlinear Model Predictive Control Framework for Whole Body Motion Planning. *IEEE TRO*. https://doi.org/10.1109/TRO.2022.3228390
6. Grandia, R., Jenelten, F., Yang, S., Farshidian, F., & Hutter, M. (2023). Perceptive Locomotion Through Nonlinear Model-Predictive Control. *IEEE TRO*. https://doi.org/10.1109/TRO.2023.3275384
7. Xu, S., Zhu, L., Zhang, H.-T., & Ho, C. P. (2023). Robust Convex Model Predictive Control for Quadruped Locomotion Under Uncertainties. *IEEE TRO*. https://doi.org/10.1109/TRO.2023.3299527
8. Sombolestan, M., & Nguyen, Q. (2024). Adaptive Force-Based Control of Dynamic Legged Locomotion over Uneven Terrain. *IEEE TRO*. https://doi.org/10.1109/TRO.2024.3381554
9. Chance-Constrained MPC authors (2024). Chance-Constrained Convex MPC for Robust Quadruped Locomotion Under Parametric and Additive Uncertainties. *arXiv:2411.03481*. https://cc-mpc.github.io/
10. Katz, B., Di Carlo, J., & Kim, S. (2019). Implementing Regularized Predictive Control for Simultaneous Real-Time Footstep and Ground Reaction Force Optimization. *IROS*. https://doi.org/10.1109/IROS40897.2019.8968031

### 学習系

1. Hwangbo, J., et al. (2019). Learning agile and dynamic motor skills for legged robots. *Science Robotics*. https://doi.org/10.1126/scirobotics.aau5872
2. Lee, J., Hwangbo, J., Wellhausen, L., Koltun, V., & Hutter, M. (2020). Learning quadrupedal locomotion over challenging terrain. *Science Robotics*. https://doi.org/10.1126/scirobotics.abc5986
3. Kumar, A., Fu, Z., Pathak, D., & Malik, J. (2021). RMA: Rapid Motor Adaptation for Legged Robots. *RSS*. https://arxiv.org/abs/2107.04034
4. Miki, T., Lee, J., Hwangbo, J., Wellhausen, L., Koltun, V., & Hutter, M. (2022). Learning robust perceptive locomotion for quadrupedal robots in the wild. *Science Robotics*. https://doi.org/10.1126/scirobotics.abk2822
5. Nahrendra, I. M. A., Yu, B., & Myung, H. (2023). DreamWaQ: Learning Robust Quadrupedal Locomotion With Implicit Terrain Imagination via Deep Reinforcement Learning. *ICRA*. https://doi.org/10.1109/ICRA48891.2023.10161144
6. Margolis, G., et al. (2023). Walk These Ways: Tuning Robot Control for Generalization with Multiplicity of Behavior. *CoRL/PMLR*. https://arxiv.org/abs/2212.03238
7. Cheng, X., Shi, K., Agarwal, A., & Pathak, D. (2024). Extreme Parkour with Legged Robots. *ICRA*. https://arxiv.org/abs/2309.14341
8. Hoeller, D., Rudin, N., Sako, D., & Hutter, M. (2024). ANYmal parkour: Learning agile navigation for quadrupedal robots. *Science Robotics*. https://doi.org/10.1126/scirobotics.adi7566
9. Li, et al. (2024). PIE: Parkour with Implicit-Explicit Learning Framework for Legged Robots. *IEEE RA-L*. https://doi.org/10.1109/LRA.2024.3459797
10. Walking with Terrain Reconstruction authors (2024). Learning to Traverse Risky Sparse Footholds. *arXiv:2409.15692*. https://arxiv.org/abs/2409.15692

### ハイブリッド

- RL-augmented MPC: https://arxiv.org/abs/2310.09442
- IFM: https://doi.org/10.1109/LRA.2023.3320827
- MPC + Predictive RL: https://arxiv.org/abs/2307.07752

---

## 付録A：読む順番（初心者向け）

§4.2/§4.3 の因果フローに沿った読み順。各ステップの §4.2/§4.3 対応も併記。

```
Step 1: 基礎理解（§4.2: Convex SRB-MPC）
  └─ §5.1 Di Carlo 2018 — SRB-MPCの基本

Step 2: なぜ不整地が難しいか（§4.2: 知覚統合NMPC）
  └─ §5.10 Kim → §5.3 Jenelten → §5.6 Grandia
  └─ RL側の対応: §6.2 Lee → §6.4 Miki

Step 3: RLの可能性（§4.3: Sim-to-real → 盲歩行 → 知覚統合）
  └─ §6.1 Hwangbo → §6.2 Lee → §6.4 Miki

Step 4: 最新動向（§4.3: パルクール / §4.4: RL-augmented MPC）
  └─ §6.8 Hoeller 2024 (Parkour) + §7 RL-augmented MPC

Step 5: 実装（§4.3: MoB / §4.2: Convex SRB）
  └─ §6.6 Walk These Ways (GitHub) または §5.1 MIT Cheetah OSS
```

## 付録B：オープンソース実装リンク

| 名称 | URL | 方式 |
|------|-----|------|
| Walk These Ways | https://github.com/Improbable-AI/walk-these-ways | RL |
| BiConMP | https://github.com/machines-in-motion/biconvex_mpc | NMPC |
| RF-MPC | https://github.com/ARCaD-Lab-UM/RF-MPC | MPC |
| Extreme Parkour | https://github.com/chengxuxin/extreme-parkour | RL |
| RL-augmented MPC | https://github.com/DRCL-USC/RL_augmented_MPC | Hybrid |
| CCMPC | https://cc-mpc.github.io/ | Robust MPC |

---

*本ドキュメントは2026年8月時点の公開論文・Web情報に基づく調査メモです。数値・速度は論文記載値を優先し、解釈部分は明示的に【推測】と区別しています。*
