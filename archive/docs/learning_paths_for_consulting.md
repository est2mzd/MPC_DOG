# 四足MPCコンサル到達：短期学習方法 4提案

**対象:** ADAS操舵MPC経験者（1年開発経験あり）  
**ゴール:** お客様に四足MPC制御について **技術コンサル** できるレベル  
**作成日:** 2026-08-18

---

## 「コンサルできる」とは何か（到達基準）

コードを全部書ける必要はない。**以下が説明・判断・提案できればコンサル可能** と定義する。

| できること | 具体例 |
|------------|--------|
| アーキテクチャ説明 | SRB-MPC + WBC の信号流をホワイトボードで描ける |
| 方式比較 | 「御社要件なら Convex MPC + ロバスト化 vs RL-augmented MPC」 |
| 工数見積 | 「不整地対応の知覚パイプライン追加で +6ヶ月」 |
| 論文マッピング | Di Carlo / Grandia / Xu Robust 等を用途別に引ける |
| トラブルシュート | 転倒原因を「接触スケジュール / 摩擦円錐 / WBC / 推定」に分類 |
| デモ提示 | MuJoCo上でConvex MPCが動く（説得材料） |

---

## 4つの学習方法（比較表）

| # | 名称 | 期間 | 費用目安 | 向いている人 | コンサル強み |
|---|------|------|----------|--------------|--------------|
| **A** | ADAS MPC 橋渡し型 | **6–8週** | 1–3万円 | MPC経験を活かしたい | 制約・最適化の深い説明 |
| **B** | 大学無料講義型 | **8–10週** | 0円 | 理論の裏付けが欲しい | 「なぜConvex化できるか」の数学 |
| **C** | 実装ドリブン型 | **6–8週** | 0–5万円 | 手を動かして覚える | デモ・PoC提案 |
| **D** | コンサルパッケージ型 | **4–6週** | 0–1万円 | 最短で商談に出たい | 提案書・見積・比較表 |

---

## 方法A：ADAS MPC 橋渡し型（6–8週）

**コンセプト:** すでに持っている操舵MPCの知識を「足式の追加要素」に拡張する。Udemyで制約付きMPCを復習し、MIT Cheetah系で足式特有部分だけ集中習得。

### なぜ有効か

操舵MPCで既に理解している要素：
- 状態空間モデル、予測ホライゾン、QP求解
- 制約（舵角限界 ≒ 摩擦円錐）
- リアルタイム性の問題

**足式で追加されるのは主に3点だけ：** 接触切替 / SRB近似 / WBC

### 週次プラン

| 週 | 内容 | リソース |
|----|------|----------|
| 1 | MPC制約・非線形系の復習 | Udemy [Applied Control Systems 2](https://www.udemy.com/course/applied-control-systems-2-autonomous-cars-360-tracking/)（LPV-MPC, 制約） |
| 2 | 3D剛体・クォータニオン（姿勢MPC） | Udemy [Applied Control Systems 4](https://www.udemy.com/course/applied-control-systems-4-uav-quaternions-mpc-pid-y/) |
| 3 | SRBモデル・摩擦円錐・Convex化 | 論文 Di Carlo 2018 + [PyQuad README](https://github.com/xh-P/PyQuad) |
| 4–5 | Convex MPC 実装・シミュレーション | [PyQuad](https://github.com/xh-P/PyQuad) または [ConvexMPC_MITCheetah3](https://github.com/hieutrongnguyen/ConvexMPC_MITCheetah3) |
| 6 | WBC（QPでGRF→トルク）の理解 | MIT Cheetah OSS / ブログ解説 |
| 7 | 不整地拡張（Robust / Perceptive） | 自社サーベイ doc + Grandia 2023 要約 |
| 8 | **コンサル成果物** 作成 | 下記「成果物」参照 |

### 主要リソース

| 種別 | 名称 | URL | 役割 |
|------|------|-----|------|
| Udemy | Applied Control Systems 1–4 | udemy.com（Dr. Bogdan Smarch） | MPC数学＋Python実装 |
| 論文 | Di Carlo 2018 | IROS | Convex MPC の原典 |
| GitHub | PyQuad | github.com/xh-P/PyQuad | Python/MuJoCo で動かす |
| ブログ | OSQP + MPC 系記事 | 各種 Medium/Qiita | QP設定の実務 Tips |

### 成果物（コンサル武器）

1. **「操舵MPC → 足式MPC」対応表**（1枚）
2. **PyQuad デモ動画**（30秒、trot歩行）
3. **方式選定フローチャート**（平坦/不整地/速度/荷重）

### 費用・時間

- Udemy: セール時 1,500–3,000円/コース × 2–3本
- 時間: 週10–15時間 × 6–8週

---

## 方法B：大学無料講義型（8–10週）

**コンセプト:** MIT・ETHの無料講義で「足式ロボット制御の理論的地基」を固め、コンサル時の説得力（なぜそう設計するか）を高める。

### なぜ有効か

お客様の技術者は「Udemy」より **MIT/ETH** の名前に反応しやすい。  
「Convex MPC は SRB 近似により接触力だけを最適化し、Centroidal dynamics の…」と説明できると信頼度が上がる。

### 週次プラン

| 週 | 内容 | リソース |
|----|------|----------|
| 1–2 | 非線形力学・最適制御基礎 | MIT [Underactuated Robotics](https://underactuated.mit.edu/) Ch.1–3 |
| 3–4 | 足式ロボットモデル（ZMP, Centroidal） | 同 Ch.4–5 |
| 5 | 軌道最適化・接触 | ETH [Optimal and Learning Control](https://arxiv.org/abs/1708.09342)（Buchli/Farshidian 講義ノート） |
| 6 | MPC理論（一般） | Rawlings "MPC: Theory, Computation, and Design" 第1–4章（書籍） |
| 7 | 足式MPC論文精読 3本 | Di Carlo, Grandia, Xu Robust |
| 8 | ETH RSL 動画 | [Gait and Trajectory Optimization](https://www.youtube.com/watch?v=KhWuLvb934g) |
| 9–10 | **コンサル成果物** | 技術ホワイトペーパー執筆 |

### 主要リソース

| 種別 | 名称 | URL | 無料 |
|------|------|-----|------|
| 講義 | MIT Underactuated Robotics | underactuated.mit.edu | ○ |
| 動画 | MIT 6.821 Lecture (YouTube) | Russ Tedrake チャンネル | ○ |
| 講義ノート | ETH Optimal and Learning Control | arXiv:1708.09342 | ○ |
| 動画 | ETH RSL Tutorial | YouTube KhWuLvb934g | ○ |
| Coursera | Robotics: Mobility (UPenn) | coursera.org | 監査無料 |
| 書籍 | Rawlings MPC | Nob Hill Pub. | 有料（推奨） |

### Courseraの位置づけ

[Coursera Robotics: Mobility](https://www.coursera.org/learn/robotics-mobility) は **MPC直接ではない** が、compass gait / SLIP 等の **足式の直感** を1–2週で補強できる。監査モードなら無料。

[Modern Robotics Course 3–4](https://www.coursera.org/specializations/modernrobotics) は逆動力学・軌道計画。WBC理解の前提として Week 1–2 だけでも可。

### 成果物

1. **10ページ技術ホワイトペーパー**「四足歩行MPCの設計原理」
2. **論文10本の1ページ要約**（サーベイ doc ベース）
3. **FAQ 20問**（例:「SRBモデルで足の質量は無視していいのか？」）

### 費用・時間

- 基本無料（Rawlings書籍 ~1万円推奨）
- 時間: 週8–12時間 × 8–10週

---

## 方法C：実装ドリブン型（6–8週）

**コンセプト:** 論文より先に **動くもの** を作る。コンサル商談で「我々環境ではこう動きました」と見せられることが最強の武器。

### なぜ有効か

四足MPCコンサルの現場で聞かれるのは：
- 「うちのUnitree Go1で動きますか？」
- 「不整地の段差○cmは？」
- 「開発期間は？」

→ **MuJoCoデモ + パラメータ変更実験** があれば回答の説得力が段違い。

### 週次プラン

| 週 | 内容 | 成果 |
|----|------|------|
| 1 | MuJoCo + PyQuad 環境構築 | trot が動く |
| 2 | SRB-MPC のパラメータ理解 | 重み変更→挙動変化を記録 |
| 3 | 接触スケジュール・ゲイト理解 | trot/bound/pace 切替 |
| 4 | 外乱実験（力を加える・段差） | 転倒条件の整理 |
| 5 | Robust MPC（Xu 2023）の概念をシミュで再現 | 荷重増加実験 |
| 6 | **簡易不整地**（MuJoCo terrain） | 段差・斜面テスト |
| 7 | RL-augmented MPC のデモ視聴＋比較 | Hybrid提案資料 |
| 8 | **デモパッケージ** 完成 | 動画 + パラメータ表 |

### 主要リソース

| 種別 | 名称 | URL |
|------|------|-----|
| GitHub | PyQuad | github.com/xh-P/PyQuad |
| GitHub | A1-QP-MPC-Controller | github.com/ShuoYangRobotics/A1-QP-MPC-Controller |
| GitHub | RL_augmented_MPC | github.com/DRCL-USC/RL_augmented_MPC |
| シミュ | MuJoCo | mujoco.org |
| ブログ | Unitree + MPC 実装記 | GitHub Issues / 各社Tech Blog |

### Udemy/Courseraの使い方（この方法では最小限）

- 詰まったら **Applied Control Systems 1** の MPC 導入（3時間）だけ視聴
- 本筋は **コードを読む・いじる・壊す**

### 成果物

1. **デモ動画 3本**（平坦trot / 外乱 / 段差）
2. **パラメータ感度レポート**（Q, R, 摩擦係数, horizon）
3. **「Go1移植チェックリスト」**（ハードあり案件用）

### 費用・時間

- 0–5万円（GPU不要、CPU + MuJoCo で可）
- 時間: 週12–18時間 × 6–8週

---

## 方法D：コンサルパッケージ型（4–6週）★最短

**コンセプト:** 実装時間を最小化し、**提案・見積・比較・FAQ** に全リソースを集中。ADAS MPC経験 + サーベイ知識を最大活用。

### なぜ有効か

「四足MPCコンサル」の商材は often：
- 技術選定支援（MPC vs RL vs Hybrid）
- 開発計画・工数見積
- 既存コードレビュー
- PoC支援の前段

→ **全部自前実装は不要**。お客様のエンジニアチームが実装し、コンサルタントは設計を導く形が多い。

### 週次プラン

| 週 | 内容 | 成果物 |
|----|------|--------|
| 1 | サーベイ doc 精読 + ADAS→足式対応表 | 対応表完成 |
| 2 | 論文10+10の「コンサル用1枚要約」 | スライド20枚 |
| 3 | **方式選定ツリー** 作成 | フローチャート |
| 4 | **工数見積テンプレート** | Excel/Notion |
| 5 | **FAQ 30問** + 模擬商談 | Q&A集 |
| 6 | PyQuad を **他人が動かした動画** で可 + 提案書テンプレ | 提案パッケージ |

### 主要リソース

| 種別 | 内容 |
|------|------|
| 自社doc | quadruped_mpc_rl_survey.md |
| Udemy | Applied Control Systems **1のみ**（復習用、4時間） |
| ブログ | [Legged Robotics ETH](https://leggedrobotics.github.io/) 各プロジェクトページ |
| 動画 | ANYmal parkour / MIT Cheetah デモ（YouTube） |
| 書籍 | 必要なら Rawlings MPC 第1章のみ |

### コンサルパッケージ構成（納品物セット）

```
📁 quadruped_mpc_consulting_kit/
├── 01_executive_summary.pdf      … 2枚、経営層向け
├── 02_technology_comparison.pdf  … MPC/RL/Hybrid比較
├── 03_architecture_patterns.pdf  … 5種のアーキテクチャ図
├── 04_effort_estimation.xlsx     … 工数見積（フェーズ別）
├── 05_paper_cheatsheet.pdf       … 論文20本1行要約
├── 06_faq_30.pdf                 … よくある質問
├── 07_demo_links.md              … 参考動画URL集
└── 08_poc_roadmap.pdf            … PoC 3ヶ月計画テンプレ
```

### 費用・時間

- 0–1万円（Udemy 1本 optional）
- 時間: 週10–12時間 × 4–6週

---

## おすすめの選び方

```
                    実装も自分でやりたい？
                         │
              ┌──────────┴──────────┐
              NO                    YES
              │                     │
         方法D（最短）          MPC経験を活かす？
         4–6週                     │
                          ┌───────┴───────┐
                         YES              NO
                          │                │
                     方法A              方法C
                     6–8週              6–8週
                          │
                    理論も固めたい？
                          │
                         YES → 方法Bを A or C に追加（+2週）
```

**【推測】ユーザー（ADAS MPC 1年）への推奨:**

1. **まず方法D（4週）** でコンサルパッケージを作る → 商談可能に
2. **並行して方法A or C（6週）** で PyQuad デモ → 説得力追加
3. 必要に応じ **方法Bの Ch.5のみ（2週）** で理論補強

---

## 方法別：コンサル商談シミュレーション

### 想定質問と、学習後の回答例

**Q1: 「不整地を犬の速度で走らせたい。MPCとRLどっち？」**

| 方法 | 回答の質 |
|------|----------|
| D | 「速度2m/s・不整地・開発6ヶ月なら RL-augmented MPC。Pure MPCなら知覚パイプラインで+3ヶ月」 |
| A | 上記 + 「Convex MPCの摩擦円錐制約は御社ADASの舵角制約と同型で…」 |
| C | 上記 + 「我々MuJoCoで段差5cmはSRB-MPC単体では厳しい、実験結果あり」 |
| B | 上記 + 「Centroidal dynamics の観点からSRB近似の限界は…」 |

**Q2: 「Unitree Go1でMPCは現実的？」**

→ 方法C: 「A1-QP-MPC-Controller, PyQuad が参考実装。Go1 SDK + Convex MPC + WBC で3–6ヶ月」

**Q3: 「開発チーム3人、1年で不整地対応できる？」**

→ 方法D: 工数テンプレで「Convex MPC基盤:4人月、知覚:6人月、ロバスト化:3人月…」

---

## 共通：避けるべき時間の使い方

| やらない方がよい | 理由 |
|------------------|------|
| ゼロからWhole-body NMPC実装 | コンサル以前に6ヶ月以上かかる |
| RLを scratch から学ぶ | 四足MPCコンサルには直接不要 |
| Courseraだけで完結 | 四足MPC特化コンテンツがない |
| 論文20本を順番通り精読 | 方法Dなら要約で十分 |

---

## 参考リンク集

### Udemy（MPC一般 → 足式橋渡し）

- [Applied Control Systems 1: MPC + PID](https://www.udemy.com/course/applied-systems-control-for-engineers-modelling-pid-mpc/)
- [Applied Control Systems 2: 制約付きMPC](https://www.udemy.com/course/applied-control-systems-2-autonomous-cars-360-tracking/)
- [Applied Control Systems 4: Quaternion MPC](https://www.udemy.com/course/applied-control-systems-4-uav-quaternions-mpc-pid-y/)
- [Autonomous Robots: MPC](https://www.udemy.com/course/model-predictive-control/)（入門、3時間）

### Coursera（足式の直感・力学）

- [Robotics: Mobility (UPenn)](https://www.coursera.org/learn/robotics-mobility) — 足式キinematics/compass gait
- [Modern Robotics Specialization](https://www.coursera.org/specializations/modernrobotics) — 逆動力学・計画

### 無料大学講義

- [MIT Underactuated Robotics](https://underactuated.mit.edu/)
- [ETH Optimal and Learning Control (lecture notes)](https://arxiv.org/abs/1708.09342)
- [ETH IDSC MPC Course Info](https://idsc.ethz.ch/education/lectures/model-predictive-control.html)

### 実装・ブログ

- [PyQuad (Python Convex MPC)](https://github.com/xh-P/PyQuad)
- [ETH Legged Robotics Projects](https://leggedrobotics.github.io/)
- [CCMPC Project Page](https://cc-mpc.github.io/)

### 動画（コンサルデモ引用用）

- [ETH: Gait and Trajectory Optimization](https://www.youtube.com/watch?v=KhWuLvb934g)
- [ETH: Imitation Learning from MPC](https://www.youtube.com/watch?v=AUNIhr5I6Dg)

---

*【推測】期間は週10–15時間投入を前提。フルタイムなら各方法2–3週短縮可能。*
