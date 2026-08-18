# 出発点上位2スタック比較

**対象:**
1. **Quadruped-PyMPC** — IIT Dynamic Legged Systems Lab  
2. **MuJoCo MPC (iLQR)** — CMU + Google DeepMind（mujoco_mpc + mujoco_mpc_deploy）

**【事実】** 本文の数値・機能は各論文・README・config.py に基づく。  
**【推測】** 「どちらを先に使うか」の推奨は末尾に分離して記載。

---

## 一言で言うと何が違うか

| | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|---|-----------------|-------------------|
| **MPCが最適化するもの** | 箱1個（SRB）の **地面反力** | **全身関節** の目標角度 |
| **動力学モデル** | 簡略 centroidal モデル | **MuJoCo そのもの**（全関節＋接触） |
| **求解** | acados（凸/NLP） or JAX（MPPI） | iLQR（DDP系、局所線形化） |
| **強み** | 実機パイプライン・足場opt・不整地sim | 全身・接触モード探索・Sim=Real同一モデル |
| **弱み** | モデル近似誤差 | 状態推定・計算コスト・実験室依存 |

```
PyMPC:   [SRB-MPC → 足の力] → スイング/スタンス制御 → 関節
MuJoCo:  [Whole-body iLQR → 関節目標] → Unitree内部PD
```

---

## 用語（動力学まわり）

| 用語 | 日本語での意味 |
|------|----------------|
| **動力学（どうがく）** | 力・トルクを入れたとき、ロボットが **どう動くか** を記述したモデル |
| **動力学モデル** | 上記を数式・シミュレータで表したもの（「次の1ステップで姿勢がこう変わる」） |
| **centroidal / SRB** | ロボット全体を **箱1個** とみなした簡略モデル（足の詳細は省略） |
| **whole-body（全身）** | 胴体＋全関節＋接触を **まとめて** モデル化 |
| **前進計算（forward）** | 今の状態＋指令 → **次の状態** をシミュレーションで求めること |
| **全関節（DOF）** | Degrees of Freedom。動かせる関節の数（四足ならおおよそ12） |

---

## 比較表（全項目）

### 1. 背景（Background）

| 項目 | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|------|-----------------|-------------------|
| **研究文脈** | Centroidal / SRB MPC は四足で標準だが、**閉ループ安定性の保証**や **GPU サンプリング MPC** の実装が分散していた | モデルベース制御は **独自の動力学ライブラリ** が多く再現性が低い。一方 RL+Sim は MuJoCo 等で加速 |
| **所属・体制** | IIT DLS Lab（HyQ, Aliengo 等の実機伝統）+ Honda R&D 協業（RAL'25） | CMU REx Lab + **Google DeepMind**（MuJoCo/MJPC 本家） |
| **OSS 位置づけ** | 「**Python で触れる最新 centroidal MPC** + 実機デプロイ部品」を1 repo に集約 | 「**MuJoCo をそのまま MPC モデルに**」する最小ベースラインを公開 |
| **前史** | MIT Cheetah Convex MPC, OCS2 centroidal, acados, MPPI 系研究 | MuJoCo MPC (DeepMind), sampling MPC on MuJoCo, OCS2/Pinocchio 系 whole-body |

---

### 2. 目的（Purpose）

| 項目 | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|------|-----------------|-------------------|
| **主目的** | 四足向け **実用的 MPC フレームワーク**（勾配 + サンプリング）を Python で提供し、**Unitree 実機**まで繋ぐ | **再現性の高い whole-body MPC ベースライン**を MuJoCo 上に置き、**実機 Go1/Go2/H1** で検証 |
| **副目的** | 足場最適化、RTI/AS-RTI、Lyapunov 制約、GPU 並列サンプリング | インタラクティブ GUI、Sim ツイン付き実機チューニング、接触リッチタスク |
| **ターゲットユーザー** | MPC パラメータをいじって **歩行・ロバスト性** を研究/開発したい人 | **全身の動力学** を明示的に MPC に入れたい人、Sim と Real で **同じ MuJoCo モデル** を使いたい人 |

---

### 3. 課題設定（Problem Setup）

| 項目 | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|------|-----------------|-------------------|
| **最適化問題** | 有限ホライゾン OCP：SRB 状態 + **各足 GRF**（12次元力）を決める | 有限ホライゾン：状態軌道 + **関節目標**（低レベル PD 込みのモデル内） |
| **接触の扱い** | **接触スケジュールはゲイトで与える**（trot/pace 等）。勾配 MPC では離地足の力=0 | **接触モードを cost（残差）で誘導**し、硬い制約にはしない。MuJoCo soft contact で滑らか化 |
| **制約** | 摩擦円錐、GRF 上下限、（任意）ZMP/静的安定、Lyapunov、足場可行域 | 関節・制御限界（iLQR 拡張）、cost による balance/gait/height 等 |
| **時間スケール** | horizon=12, dt=0.02 → **0.24 s**（config デフォルト）、mpc_frequency=100 Hz（sim） | horizon **0.35–0.5 s**、離散化 100 Hz、iLQR **~50 Hz** + TV-LQR **~300 Hz** |
| **求解器** | **acados** (SQP/RTI/DDP) または **JAX** (random/MPPI/CEM) | **iLQR**（1 iter/周期、warm start、収束判定なし） |

---

### 4. 結論（論文・プロジェクトが示したこと）

| 項目 | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|------|-----------------|-------------------|
| **勾配 MPC** | i7 CPU で **<5 ms**/step（README） | i9 desktop で iLQR **~10–20 ms**/iter（impratio 調整後） |
| **サンプリング MPC** | Laptop GPU で **10k rollout <2 ms**（IROS'24） | 本 stack の主眼ではない（MJPC 別途 Predictive Sampling あり） |
| **実機** | muse + unitree-ros2-dls 経由で **Unitree 複数機種**（README） | **Go1, Go2, H1** 実機成功（2025 論文） |
| **理論** | RAL'25：**適応 centroidal MPC + 安定性保証**（Aliengo, ergoCub） | 「**驚くほど単純な iLQR + MuJoCo でも実機が動く**」 |
| **限界の自認** | kinodynamic モードは experimental、Docker/CUDA WIP | 状態推定・SysID・iLQR の接触探索・長ホライズンが課題（論文 Sec.VI） |

---

### 5. MPC モデル化対象

| 項目 | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|------|-----------------|-------------------|
| **予測モデル** | **Single Rigid Body（SRB）/ Centroidal** | **MuJoCo 完全多体モデル**（浮動基座 + 全関節 + soft contact） |
| **状態（ざっくり）** | CoM 位置/速度、姿勢/角速度、（必要なら角運動量） | 基座 pos+quat、全 joint q/dq、速度 |
| **制御入力** | **地面反力 GRF**（足4本 × 3） | **関節角度リファレンス**（モデル内 PD がトルク生成） |
| **近似の中身** | 脚の質量・関節の動力学は **MPC 層では無視**。WBC/脚制御が担当 | 近似 **なし**（MuJoCo がそのまま **前進計算モデル**） |
| **接触モデル** | 離散的（stance/swing）。摩擦は **線形化された円錐制約** | MuJoCo **soft contact** + 有限差分でヤコビアン |
| **Sim と Real のモデル** | Sim=MuJoCo（全身）、MPC=SRB → **意図的に不一致** | Sim も Real も **同じ MuJoCo XML** を planner に使用 |

**覚え方:** PyMPC は「**力プランナ**」、MuJoCo iLQR は「**全身動作プランナ**」。

---

### 6. アーキテクチャ

#### Quadruped-PyMPC

```
┌─────────────────────────────────────────────────────────┐
│  Reference (速度指令, ゲイト trot/pace, 足位置参照)        │
└────────────────────────┬────────────────────────────────┘
                         ▼
┌─────────────────────────────────────────────────────────┐
│  MPC Layer (acados or JAX)                               │
│  ・入力: GRF 　・モデル: SRB 　・任意: 足場最適化          │
└────────────────────────┬────────────────────────────────┘
                         ▼
┌─────────────────────────────────────────────────────────┐
│  Swing / Stance Leg Control                              │
│  ・スイング: Bezier / scipy 軌道 + PD                    │
│  ・スタンス: GRF → 関節トルク/インピーダンス              │
└────────────────────────┬────────────────────────────────┘
                         ▼
┌─────────────────────────────────────────────────────────┐
│  MuJoCo Sim  or  Real Robot (Unitree SDK via ROS2)       │
│  状態推定: muse (実機)                                    │
└─────────────────────────────────────────────────────────┘
```

- **階層:** 典型 MIT Cheetah 型（MPC→脚→関節）
- **設定:** `quadruped_pympc/config.py` 一箇所
- **実機:** [muse](https://github.com/iit-DLSLab/muse) + [unitree-ros2-dls](https://github.com/iit-DLSLab/unitree-ros2-dls)

#### MuJoCo MPC (iLQR)

```
┌─────────────────────────────────────────────────────────┐
│  Cost residuals (目標位置, gait pattern, balance, etc.)  │
│  GUI でリアルタイム重み変更                               │
└────────────────────────┬────────────────────────────────┘
                         ▼
┌─────────────────────────────────────────────────────────┐
│  iLQR Planner (~50 Hz, C++)                            │
│  ・MuJoCo 前進計算 + 有限差分ヤコビアン                   │
│  ・出力: 関節目標軌道 + TV-LQR ゲイン K(t)               │
└────────────────────────┬────────────────────────────────┘
                         ▼
┌─────────────────────────────────────────────────────────┐
│  TV-LQR feedback (~300 Hz)                               │
│  u = u_nom + K(x - x_nom)                                │
└────────────────────────┬────────────────────────────────┘
                         ▼
┌─────────────────────────────────────────────────────────┐
│  Unitree 内部 joint PD  or  MuJoCo Sim（同一モデル）      │
│  状態: OptiTrack + 関節エンコーダ融合 (ROS)               │
└─────────────────────────────────────────────────────────┘
```

- **階層:** **フラット**（SRB/WBC なし。MPC が直接関節を決める）
- **GUI:** MuJoCo MPC GUI + 実機 Sim ツイン
- **Deploy:** [mujoco_mpc_deploy](https://github.com/johnzhang3/mujoco_mpc_deploy)（WIP、推定は開発中）

---

### 7. 対象シナリオ

| シナリオ | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|----------|-----------------|-------------------|
| **平坦走行** | ◎ trot/pace/crawl/bound | ◎ Go1/Go2 四足歩行 |
| **不整地（sim）** | ◎ `scene`: random_boxes, pyramids, **perlin** + 足場opt | △ 主に平坦実験室（論文） |
| **足場計画** | ◎ `use_foothold_optimization` | △ cost で foot lifting pattern（硬い制約ではない） |
| **外乱・荷重** | ◎ 外力補償、integrator、Lyapunov、適応 MPC（論文） | △ 実験は主に指令追従 |
| **高速** | ○ ゲイト freq 調整（optimize_step_freq 可） | ○ 四足歩行中心（速度数値は論文で明示少） |
| **アクロバティック** | △ bound 等（SRB 限界あり） | ◎ **二足歩行、ハンドスタンド**（Go1） |
| **人形** | △ quadruped 専用 | ◎ **H1 人形 trot** |
| **実機環境** | Unitree 屋外/屋内（muse 次第） | **OptiTrack 実験室**（論文） |
| **対応ロボ** | go1, go2, aliengo, b2, spot, mini_cheetah, hyq 等 | Go1, Go2, H1（Menagerie / go1 branch） |

---

### 8. 残課題

| 残課題 | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|--------|-----------------|-------------------|
| **状態推定** | muse に依存（別 repo）。VFA `visual_foothold_adaptation` は blind/height 程度 | **WIP**。現状 **OptiTrack 必須**（論文明言）。オンボードのみは未整備 |
| **Sim-to-Real** | SRB≠MuJoCo 全身 → **モデルギャップ**を integrator/サンプリングで吸収 | MuJoCo=Planner だが **actuator/contact SysID 不足**（論文） |
| **構築コスト** | **acados ビルド**が重い。初回 tera_renderer 等 | **C++ MJPC ビルド** + Unitree SDK + ROS。desktop CPU 推奨 |
| **理論ギャップ** | SRB 近似の限界（大段差・強い非凸接触） | iLQR：**接触モード探索が弱い**、長ホライズン不安定、シリアル計算 |
| **不整地・知覚** | 標高/VFA は限定的。**カメラ統合は別開発** | 知覚パイプライン **なし**（cost で目標位置を与えるのみ） |
| **実装成熟度** | 2026年も更新、実機パイプライン **一体** | deploy repo **WIP**、推定モジュール未完成 |
| **コンサルで語れる先端性** | IROS'24 sampling, RAL'25 安定性, AS-RTI | 2025 whole-body, DeepMind 公式 stack, ICRA accepted |

---

## 機能マトリクス（クイック参照）

| 機能 | PyMPC | MuJoCo iLQR |
|------|:-----:|:-----------:|
| SRB / Centroidal MPC | ◎ | ✗ |
| Whole-body MPC | ✗ | ◎ |
| 足場最適化 | ◎ | △ |
| 摩擦円錐（明示制約） | ◎ | △（soft contact） |
| GPU サンプリング (MPPI) | ◎ | △（別 MJPC 機能） |
| 実機 Unitree 手順 | ◎ | ○（WIP） |
| 不整地 sim | ◎ | △ |
| 二足/人形 | ✗ | ◎ |
| GUI 実機チューニング | △ | ◎ |
| Python 中心 | ◎ | △（C++ core） |

---

## ADAS 操舵 MPC 経験者向けの対応

| 操舵 MPC でやっていたこと | PyMPC | MuJoCo iLQR |
|---------------------------|-------|-------------|
| 車両モデル + QP | **SRB + acados QP/NLP** ← 近い | 全身 + iLQR ← **構造が違う** |
| 制約（舵角限界） | 摩擦円錐 + GRF 限界 | cost + control limits |
| リアルタイム性 | ◎ 5ms 級 | ○ 10–20ms + デスクトップ |
| チューニング | config.py の Q/R/μ | GUI residual weights |

---

## 【推測】どちらを主軸にするか

| ゴール | 主軸 | 副軸 |
|--------|------|------|
| **不整地 + 犬速度 + 実機/consult** | **Quadruped-PyMPC** | MuJoCo iLQR で「全身も可能」のデモ |
| **全身・接触モード・研究の新規性** | **MuJoCo iLQR** | PyMPC で実機歩行の安定性 |
| **両方理解して提案** | PyMPC で **力ベース**、MuJoCo で **関節ベース** と説明分け | — |

**現実的な組み合わせ:**

1. **Phase 1:** PyMPC（gradient + foothold + perlin terrain）→ コンサルデモ・実機路線  
2. **Phase 2:** MuJoCo iLQR で whole-body の挙動理解 → 「SRB の限界はここ」と説明材料  

---

## 参考文献・リンク

| | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|---|-----------------|-------------------|
| **Repo** | https://github.com/iit-DLSLab/Quadruped-PyMPC | https://github.com/google-deepmind/mujoco_mpc |
| **Deploy** | muse + unitree-ros2-dls | https://github.com/johnzhang3/mujoco_mpc_deploy |
| **論文1** | [IROS'24 Sampling MPC](https://arxiv.org/abs/2403.11383) | [Whole-Body MPC with MuJoCo](https://arxiv.org/abs/2503.04613) |
| **論文2** | [RAL'25 Adaptive Stable MPC](https://arxiv.org/abs/2409.01144) | — |
| **設定** | `quadruped_pympc/config.py` | MJPC task XML + GUI |

---

## お客様が「接地反力・MPC・WBC」をやりたい場合

お客様が言う **接地反力（GRF）・MPC・WBC** は、四足制御の教科書的な **3層構造** を指している可能性が高い。  
この文書の2スタックに当てはめると、**「やりたいこと」と「向く repo」がかなりはっきり分かれる。**

### 用語の整理（この比較での意味）

| 用語 | やっていること | ADAS 操舵 MPC との対応 |
|------|------------------|------------------------|
| **接地反力 GRF** | 各足が地面から受ける力（3次元×4足）。**MPC の最適化変数** になることが多い | タイヤ力に相当。ただし足は **離地する** ので時々ゼロ |
| **MPC** | 未来を予測して、今の指令（GRF や関節目標）を最適化 | 操舵 MPC と同じ発想。モデルと制約が違う |
| **WBC** | MPC が決めた GRF を **関節トルクに変換** する低レベル（QP が定番） | MPC の出力をアクチュエータ指令に落とす **下位層** |

```
【教科書型（MIT Cheetah 系）】

  速度指令
      ↓
  MPC ──→ 各足の GRF（＋摩擦円錐制約）
      ↓
  WBC ──→ 関節トルク
      ↓
  モータ PD
```

**【事実】** この3層のうち、**GRF を MPC の最適化変数として明示的に計画する** のが Quadruped-PyMPC。下位の Stance Leg Control が **WBC 相当** で、その GRF を関節トルクに変換する。  
**【事実】** MuJoCo MPC (iLQR) は **GRF 計画層も WBC 層も持たず**、MPC が直接関節目標を出す（第3のルート）。

**GRF と WBC は相容れないどころか、教科書型ではセット。** 対立するのは「GRF-MPC+WBC」路線と「全身 MPC 直結」路線の **アーキテクチャ全体** である。

---

### 2スタックでの対応関係

◎=そのまま触れる　○=相当機能あり（名称・実装は異なる）　△=限定的　✗=この層なし

| 観点 | Quadruped-PyMPC | MuJoCo MPC (iLQR) |
|------|-----------------|-------------------|
| **① GRF-MPC + WBC の3層パイプライン全体** | ◎ MPC→GRF→Stance制御→関節。**教科書型がそのまま repo 構造** | ✗ GRF 計画も WBC も省略。**別アーキテクチャ** |
| **② 中レベル：GRF を MPC で計画** | ◎ 最適化変数 = GRF 12次元。摩擦円錐・上下限は acados 制約。ログで追える | ✗ MPC は **関節目標** を直接最適化。GRF は MuJoCo 接触の副産物 |
| **③ 低レベル：GRF → 関節トルク（WBC 相当）** | ○ **Stance Leg Control** が担当（GRF→トルク/インピーダンス）。論文のフル QP WBC と名称・詳細は異なる | ✗ TV-LQR + Unitree 内部 PD。**GRF 指令を受け取る層がない** |
| **④ GRF と WBC の接続（上下整合）** | ◎ ②の出力が ③の入力。**意図的にパイプされている** | — ②③自体がないため **接続の概念なし** |
| **⑤ 摩擦円錐など「力」の硬い制約** | ◎ MPC 層で明示（操舵 MPC の制約設計に近い） | △ soft contact + cost。**力ベースの硬い制約ではない** |
| **⑥ Whole-body：接触込みで MPC が関節まで直接** | △ SRB 近似のため **全身・接触モード探索は弱い** | ◎ MuJoCo 全身 + iLQR。**GRF/WBC を飛ばす代わり** にこちらが強い |
| **⑦ コンサルで「力計画→トルク変換」と説明したい** | ◎ §6 アーキテクチャ図がそのまま使える | △ **対比例** として「階層省略型 whole-body」 |
| **⑧ MPC 本体の改造・読みやすさ** | ◎ Python + SRB モデル + acados/JAX | ○ C++ iLQR + MuJoCo XML 全体 |

**読み方:** お客様が **GRF と WBC の両方** に興味があるなら、行 **①〜④** を見る。PyMPC は **②+③+④ が一体**、MuJoCo は **⑥ の whole-body 特化** で ①〜④ は該当しない。

---

### 【推測】結論：お客様向けの選び方

**GRF・MPC・WBC を「ちゃんと触りたい／提案の芯にしたい」なら → Quadruped-PyMPC が第一候補。**

理由を3点に絞ると：

1. **GRF-MPC と WBC 相当層がパイプで繋がっている**（表 ①②③④）— 「力を計画してから関節に落とす」が repo 構造そのもの。  
2. **摩擦円錐・GRF 上下限** が acados 側の制約として触れる（表 ⑤）— ADAS 操舵 MPC 経験者には **Q/R より制約設計** の話がしやすい。  
3. **下位層（Swing/Stance）が WBC 相当** — GRF と WBC は **別々に選ぶものではなく**、PyMPC 内で上下に接続されている。

**MuJoCo MPC (iLQR) を第一候補にするのは、次のようなお客様向け：**

- 「GRF/WBC は飛ばして、**MuJoCo 全身をそのまま MPC に入れたい**」  
- **二足歩行・ハンドスタンド** など SRB では無理な接触リッチ動作  
- GUI で cost をいじりながら **全身挙動を探索** したい  

GRF・WBC にこだわるお客様には、MuJoCo 単体だと **「やりたい3層と違うルート」** になり、説明コストが上がりやすい。

---

### お客様への説明例（そのまま使えるトーク）

> 四足の定番は、操舵 MPC と同じく **MPC で未来を予測** しますが、最適化するのが舵角ではなく **足の地面反力（GRF）** です。  
> 摩擦円錐で「滑らない力」を決め、下の **WBC（Whole-Body Control）** で関節トルクに変換します。  
> 今回の2候補のうち、**Quadruped-PyMPC がこの教科書構成にそのまま対応** します。  
> MuJoCo MPC は **GRF を経由せず関節を直接最適化** する whole-body 路線で、研究デモとしては強いですが、  
> 「接地反力を設計変数として扱う」という意味では **別アーキテクチャ** です。

必要なら Phase 2 で MuJoCo を **「WBC を省略した全身 MPC の例」** として見せ、PyMPC との対比に使う（§【推測】どちらを主軸にするか の Phase 1/2 と同じ）。

---

### 最初に触るモジュール（PyMPC を選んだ場合）

| 順番 | 場所 | 何が分かるか |
|------|------|--------------|
| 1 | `quadruped_pympc/config.py` | MPC の Q/R、摩擦 μ、GRF 限界、ゲイト |
| 2 | MPC 層（acados 設定） | **最適化変数 = GRF**、SRB 状態方程式 |
| 3 | Swing / Stance Leg Control | **WBC 相当**：GRF → スタンス脚トルク、スイング軌道 |
| 4 | MuJoCo sim + perlin terrain | 不整地で GRF 計画がどう変わるか |

**【推測】** お客様が ADAS MPC 経験者なら、**config の制約（摩擦円錐・GRF 限界）→ MPC ログの GRF 波形 → Stance 制御** の順で見せると、操舵 MPC との対応が一発で伝わる。

**2日ワークショップ:** [docs/pympc_2day/WORKSHOP.md](./pympc_2day/WORKSHOP.md)

---

### 期待値のすり合わせ（事前に言っておくとよいこと）

| 項目 | 伝えておく内容 |
|------|----------------|
| **GRF と WBC の関係** | **対立しない。セット。** GRF=MPC が計画する力、WBC=それを関節トルクに変換する下位層 |
| **WBC の名称** | PyMPC では "Swing/Stance Leg Control" と呼ばれ、論文の **完全 QP WBC** と実装詳細が異なる場合がある |
| **MuJoCo との違い** | MuJoCo は GRF も WBC も **使わない第3ルート**。GRF/WBC 両方やりたいなら PyMPC が自然 |
| **不整地** | GRF-MPC+WBC だけでは足場計画不足。**足場 opt（PyMPC）** や **知覚統合（別開発）** が追加で必要 |
| **開発規模** | GRF-MPC+WBC も ADAS 操舵 MPC と同様 **数ヶ月〜年** 規模になりうる（[quadruped_mpc_rl_survey.md](./quadruped_mpc_rl_survey.md) §3 参照） |
| **RL との関係** | GRF/MPC/WBC に興味があるなら **モデルベース路線（PyMPC）** が自然。RL は「WBC を NN で置き換える」別系統 |

---

*作成: 2026-08-18 / mpc_dog プロジェクト*
