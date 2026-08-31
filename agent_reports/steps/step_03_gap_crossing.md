# Step 03:前進方向に並ぶ穴(轍)を、落ちずに越える(平面マップ)

対象commit: `external/Quadruped-PyMPC` = Step 01/02 と同じ
`cc145a2d353db4c39df4b49e6624959acc4b87b0`。`external/` 配下は**一切変更していない**。
制御は Step 02 と同じく `compute_actions()` の引数・順序を変えずに呼ぶ。

---

## 1. 目的とマップ仕様

**やりたいこと**: 前進方向(+x)に一定間隔で穴が並ぶ平面マップで、
穴に落ちずに前進できるか(0.3 / 0.5 / 0.7 m/s)を確認する。

**マップ**(`src/trial/assets/scene_gaps.xml`、生成 `gen_scene_gaps.py <depth>`):

- 通路は y ∈ [-2.5, 2.5] の **5 m 幅**。
- x 方向に **1.7 m の凸条**(上面が z=0 の板)を並べ、間に **0.30 m 長**の
  トレンチを **2.0 m 間隔**で作る。
- 穴(トレンチ)の中心は x = 1, 3, 5, …。凸条の中心は x = 0, 2, 4, …。
- 穴は通路の**全幅 5 m** を横切る(横中央 y=0 にいるロボットは、穴を
  横方向には避けられない — x 方向にまたぐしかない)。
- ロボットは y=0(穴の横中央)を +x 方向に歩く。

**穴の深さは調整対象にした**(下記 3 節)。

---

## 2. 結論(先に)

- **底なしの深い穴(深さ 0.5 m)は、素の Quadruped-PyMPC(`blind`)では
  最初の穴で転落した。** 理由は 5 節参照(地形を見ないので足場を穴の外へ
  ずらさない)。
- **深さ 5 cm の浅いトレンチ(轍)にすると、0.3 / 0.5 / 0.7 m/s すべてで
  穴を越えて前進できた。** 転倒なし、姿勢の乱れも小さい
  (|傾き| < 0.21 rad、胴体高さの落ち込み ≤ 2 cm)。
- 歩容周波数 `step_freq` は 2.0〜3.0 Hz の範囲でいずれも成立
  (Step 02 と同じく、速度に対して低すぎなければよい)。

---

## 3. 方法と反復の経緯

ハーネス: `src/trial/step_03_gap_crossing.py`(実行 `bash scripts/trial/run_step_03.sh`)。
`scene_gaps.xml` を実行時に gym_quadruped の `robot_model/` へコピーし、
`QuadrupedEnv(scene="gaps")` で読み込ませる(`external/` は不変)。
`visual_foothold_adaptation` は Step 02 と同じ `blind`(地形非考慮)。

反復(短時間スイープ、平面マップ、fall = 胴体 z<0.12 または |傾き|>0.8):

| 穴深さ | v [m/s] | step_freq [Hz] | 結果 | 越えた穴 | 転倒 |
|---|---|---|---|---|---|
| 0.50 m(底なし) | 0.3 | 1.6 | 転落 | 0 | 3.1 s(最初の穴) |
| 0.50 m(底なし) | 0.3 | 2.4 | 転落 | 0 | 3.4 s(最初の穴) |
| 0.10 m | 0.3 | 2.0 | 転倒 | 1 | 6.4 s(2 つ目付近、前のめり) |
| **0.05 m** | 0.3 | 2.0 | **OK** | 3 | なし |
| **0.05 m** | 0.5 | 2.0 | **OK** | 4 | なし |
| **0.05 m** | 0.5 | 2.6 | **OK** | 4 | なし |
| **0.05 m** | 0.7 | 2.4 | **OK** | 6 | なし |
| **0.05 m** | 0.7 | 3.0 | **OK** | 6 | なし |

深さ 5 cm を採用。理由: Quadruped-PyMPC の**遊脚の跳ね上げ高さ**は
`step_height = 0.2 × hip_height ≈ 6 cm`(config.py)しかなく、深い穴の
向こう側の縁(高さ差)に遊脚が引っかかって前のめりに転ぶ。5 cm なら
遊脚が縁を越えられ、踏み外しても 5 cm 落ちて戻るだけで済む。

---

## 4. 本記録(深さ 5 cm、各 20 s)

3 速度すべて **PASS**(転倒なし、穴を 3 つ以上通過、横ズレ < 2 m):

| id | v [m/s] | step_freq [Hz] | 前進距離 [m] | 越えた穴 | 横ズレ [m] | 転倒 | 判定 |
|---|---|---|---|---|---|---|---|
| 01 | 0.3 | 2.0 | 5.69 | 3 | +0.25 | なし | PASS |
| 02 | 0.5 | 2.0 | 9.29 | 5 | -0.46 | なし | PASS |
| 03 | 0.7 | 2.4 | 12.83 | 6 | -0.02 | なし | PASS |

いちばん厳しい 0.7 m/s の軌道の質(`state_log.csv` より):

- 胴体高さ z: min 0.290 / mean 0.300 / max 0.309 m(公称 ≈ 0.30。穴で
  ほとんど揺れていない)
- |roll| max 0.103 rad(≈ 6°)、|pitch| max 0.139 rad(≈ 8°)
- **足先の最低高さ −0.043 m**(足が轍に約 4.3 cm 入り、次の一歩で復帰。
  轍深さ 5 cm 内に収まっている)
- 整定後の実 vx ≈ 0.64 m/s(指令 0.7 の約 92%)
- `compute_actions()` mean 2.3 ms / max 32 ms

- 生成物(いずれも `.gitignore` 対象ではない):
  - `artifacts/logs/step_03/state_log.csv`(最後の 1 回分)
  - `artifacts/logs/step_03/trials_summary.csv`(3 回分:id, 速度, step_freq,
    sim 時間, 前進距離, 横ズレ, 越えた穴数, 転倒時刻, 判定)
  - `artifacts/logs/step_03/gif_meta.json`
  - `artifacts/gifs/step_03_{id}.gif`(横視点、時刻・速度・越えた穴数を焼き込み)

---

## 5. 大学院初心者向け解説:なぜ穴で転ぶのか / なぜ浅いと越えられるのか

### 5.1 四足の「安定」には 2 種類ある

- **静的安定(static stability)**: 接地している足で作る多角形(support
  polygon)の中に、重心(の鉛直投影)が入っていれば、止まっていても倒れない。
  4 脚接地なら大きな四角形、対角 2 脚接地(トロットの基本状態)なら
  「対角線」= ほぼ線分になり、静的安定の余裕はほとんど無い。
- **動的安定(dynamic stability)**: トロットのように常時 2 脚しか着いていない
  歩き方は、静的には倒れかけている状態を、**次の一歩を正しい場所に置くこと**で
  連続的に立て直している。倒立振子を手のひらで支え続けるのと同じ。
  → 「次の足を置く場所」が歩行の生命線。

### 5.2 穴があると何が壊れるか

トロットで前進中、遊脚(swing leg)は空中を前へ運ばれ、ある目標地点
(**foothold = 足場**)に**着地**する。着地した瞬間からその脚は支持脚になり、
体重を受け、次の一歩まで体を支える。

- 目標地点が**穴の中**だと:
  - 深い穴 → 足はどこにも当たらず落ち続ける。支持脚が 1 本足りない状態で
    体は前へ倒れ込み、復帰できずに**転落**。
  - 浅い穴 → 足は 5 cm 下の穴底に当たる。予定より低い位置で接地するが、
    支持自体は得られる。多少姿勢は乱れるが、他の 3 脚と次の一歩で立て直せる。
- 目標地点が穴の**手前/向こうの縁**だと:
  - 遊脚が前へ振り出される途中、穴の**向こう側の縁**(z の段差)に
    つま先が当たる。Quadruped-PyMPC の遊脚跳ね上げは約 6 cm しかないので、
    段差が 6 cm 以上あると引っかかり、脚が止まって体だけ前へ行き、
    **前のめりに転倒**(3 節の深さ 10 cm の例がこれ)。

### 5.3 「地形を見る」とはどういうことか

足場を決める方法には 2 段階ある:

1. **Raibert 則(地形を見ない)**: 「今の速度と目標速度の差」から、
   倒立振子を安定化させる着地点を幾何的に計算する
   (`足場 ≈ hip 直下 + √(h/g)·(v − v_desired) + 遠心力補正`)。
   go2 の平地歩行はこれで足りる。**穴の存在は一切考慮しない**ので、
   計算結果がたまたま穴の中でも、そこへ足を出す。
2. **地形適応(VFA, visual foothold adaptation)**: 高さマップ
   (heightmap、周囲の地面高さを格子で持ったもの)を見て、Raibert の
   足場が穴・急斜面なら**近くの安全な地点へ x/y をずらす**。
   これが本来「穴を避けて踏む」ための仕組み。

Quadruped-PyMPC には VFA の口はある(`visual_foothold_adaptation` = `blind` /
`height` / `vfa`)が:

- `blind` … 地形非考慮(Step 02/03 のデフォルト)。
- `height` … 足場の**高さ(z)だけ**地形に合わせる。x/y はずらさない
  → 穴回避には使えない。
- `vfa` … x/y を安全地点へずらす本命。ただし実装 (`virall`) は
  **非公開でインストールされていない**。

→ **素の Quadruped-PyMPC は、穴を「避けて踏む」ことができない。**
できるのは「Raibert が出した足場をそのまま踏んで、深い穴なら落ちる/
浅い穴なら耐える」だけ。だから Step 03 は**浅い轍**に設定して
「避けなくても越えられる」形にした。

### 5.4 歩幅と穴幅の関係(なぜ step_freq が効くか)

1 歩で足場が前進する距離(歩幅)は、おおよそ

```
歩幅 ≈ 前進速度 / step_freq
```

- `step_freq` が低い → 歩幅が大きい → 1 歩が長く、脚を大きく振る。
  平地では Step 02 で見たとおり不安定化しやすい。穴地形では、遊脚が
  長く空中を移動する間に穴の縁に引っかかるリスクも上がる。
- `step_freq` が高い → 歩幅が小さい → 一歩ずつ細かく置く。
  穴(0.30 m)に対して歩幅が十分小さければ、支持脚がずっと穴の外の
  凸条上にあり続けやすい。

Step 03 では 2.0〜3.0 Hz で 0.3〜0.7 m/s すべて成立した(歩幅 ≈ 0.10〜0.35 m)。
これは 0.30 m の穴に対して「連続する支持脚のどれかが必ず凸条上」に
なりやすい範囲。低すぎる `step_freq`(平地で転んだ 1.0 Hz など)は
未検証だが、穴地形ではより危険と考えられる。

### 5.5 まとめの一言

> 四足のトロットは「次の一歩を正しい場所に置いて倒れ続ける」動的歩行。
> 素の Quadruped-PyMPC は地形を見て足場を穴の外へずらす機能を持たない
> (`blind`、VFA は非公開)。したがって「深い穴を避けて越える」のは無理で、
> 「浅い轍を、避けずに踏み越える」ことだけが成立する。Step 03 はその範囲で
> 0.3 / 0.5 / 0.7 m/s の前進を確認した。

---

## 6. 事実 / 推測

### 事実(コード・実行で確認)

- `QuadrupedEnv` は `robot_model/scene_<name>.xml` が存在すればそれを
  そのまま読む(`utils/mujoco/terrain.py: generate_terrain`)。Step 03 は
  `scene_gaps.xml` を実行時にそこへコピーして `scene="gaps"` で使う。
- `visual_foothold_adaptation='vfa'` は `from virall.vfa.vfa import VFA` の
  ImportError で使えない("VFA not installed, not open source yet")。
- `'height'` 戦略は `reference_footholds[leg][2] = height_adjustment`(z のみ)。
- `step_height = 0.2 * hip_height`(config.py、go2 で約 6 cm)。
- 深さ 0.5 m で 0.3 m/s は最初の穴(x=1.0、凸条端 x=0.85)で転落。
- 深さ 0.05 m で 0.3 / 0.5 / 0.7 m/s すべて転倒せず 3〜6 個の穴を通過。

### 推測(未検証)

- **越えられる穴の最大深さ**は 5〜10 cm の間(10 cm で前のめり転倒、5 cm で
  安定)。二分探索はしていない。
- `step_height` を config で 2 倍程度に上げれば、より深い穴も越えられる
  可能性がある(未実施 — Step 03 はデフォルトのまま)。
- **深い穴を素の PyMPC で越える**には x/y 足場回避が要り、それには
  非公開の VFA か自前の足場修正が必要。
- 低い `step_freq`(< 1.4 Hz)での穴地形挙動は未検証。

---

## 7. その後

- 「深い穴の飛び越え(leap)」は Quad-SDK 側の別ドキュメント
  (`agent_reports/quadsdk_step01_simple_model_terrain_and_gaps.md`)で
  整理済み。PyMPC でやるなら (a) 自前の足場回避 + heightmap、
  (b) `step_height` を上げてより深い轍まで対応、が現実的な次段。
- Step 03 のログは「浅い轍を blind で越えられる」基準として保存する。

---

## 8. ソース早見表

- ハーネス / マップ
  - `src/trial/step_03_gap_crossing.py`(記録ハーネス、`external/` 不変)
  - `src/trial/assets/gen_scene_gaps.py`(マップ生成、引数 = 穴深さ[m])
  - `src/trial/assets/scene_gaps.xml`(生成物、深さ 5 cm)
  - `scripts/trial/run_step_03.sh`
- Quadruped-PyMPC 側(参照のみ、変更なし)
  - `quadruped_pympc/config.py`(`visual_foothold_adaptation`, `step_height`,
    `gait_params`)
  - `quadruped_pympc/helpers/visual_foothold_adaptation.py`(`height` / `vfa`)
  - `quadruped_pympc/helpers/foothold_reference_generator.py`(Raibert 則)
  - `<venv>/gym_quadruped/utils/mujoco/terrain.py`(`generate_terrain`)
- 関連ドキュメント
  - `agent_reports/steps/step_02_frequency.md`(平地・歩容周波数)
  - `agent_reports/quadsdk_step01_gait_and_mpc.md`(歩容と MPC の役割分担)
  - `agent_reports/quadsdk_step01_simple_model_terrain_and_gaps.md`(穴超えの整理)
