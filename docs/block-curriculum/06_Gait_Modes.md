# 足の運びモードはゲイトパラメータである

進める順番の正本は [00](00_README.md) である。本章はフェーズ 0 の S4 である。

## 1. 結論

トロットもペースも、制御器を増やさない。  
`PeriodicGait(freq, duty, offset)` の3つ（と脚の対応）だけ変える。  
S4 で触ってよいファイルは、そのパラメータ表とログ名である。

## 2. 理論 — 共通式

どのモードも同じである。

\[
\phi_i \leftarrow (\phi_i + \Delta t\, f)\bmod 1,\qquad
c_i = [\phi_i < d]
\]

違うのは初期位相 \(\phi_i(0)\)（offset）と \(f,d\) である。  
脚力も `MapJT` もこの式を知らない。`c` だけを見る。

## 3. 実装事実 — PyMPC の種類

`GaitType` と `config.py` の `gait_params`:

| 人が呼ぶ名前 | 何が同じ位相か（概念） | その場足踏みでの見え方 |
|---|---|---|
| `full_stance` | 4脚 | S1 に近い。足が上がらない確認 |
| `trot` | 対角（FL+RR と FR+RL） | 標準の足踏み |
| `pace` | 左右（同側前後） | 左右に揺れる |
| `bound` | 前後 | ピッチが揺れる |
| `crawl` | ほぼ1脚ずつ（duty 高め） | 遅い。常に3脚接地に近い |

offset の数値の正本は上流 `gait_params` と `PeriodicGaitGenerator` である。  
自分の表は、上流を見て数字を写し、単体テストで `c` のパターンを固定する（方針）。

LC も trot 等の名前を端末から送るが、本体は OCS2 の mode 列である。自分の S4 は **配列 \(c(t)\)** を正本にする。名前はラベルにすぎない。

## 4. 方針 — S4 のやり方

1. S3 の \(\tau\)（`notebook/05` の瞬間 wrench + `MapJT` + swing）は変えない
2. yaml または dataclass で gait 名を切る
3. 歩くモードは上流の freq / duty / 遊脚高さで **10 s かつ 10 m**。`full_stance` は 10 s 静止。duty を上げて直立に見せるのは完了ではない。数字の正本は `notebook/06` の背景
4. グラフは接地の4本線を必ず出す（どれが上がっているか）

制御の if 文に `if gait == "trot"` を書かない。offset が全部やる。

## 5. 方針 — 比較との関係

| 見たいもの | 固定 | 変える |
|---|---|---|
| 運びの違い | EqualShare | gait 名 |
| 予測がその運びに強いか | 脚力は一つに固定（フェーズ 1 なら LcNmpc、フェーズ 2 なら GrfMpc）、同じ重み | gait 名 |
| 予測の意味 | 同じ gait（trot） | EqualShare vs LcNmpc または EqualShare vs GrfMpc |

3つを1本の実験に混ぜない。ゲイト比較はフェーズ 0 で EqualShare のまま先にやる。予測との交差はフェーズ 1 以降である。

## 6. 推測 — どの順で足踏みするか

転倒しにくそうな順（未実験）:

1. `full_stance`（S1 回帰）
2. `crawl`（常に多くの脚）
3. `trot`（overlap あり、duty 0.74）
4. `pace`
5. `bound`（ピッチが厳しい）

bound で落ちても、trot ができていればゲイトの問題として切れる。  
最初から bound + NMPC は一段に現象が二つ入る。bound と坂・高速も混ぜない（[11](11_Difficulty_Map.md)）。

## 7. 次

次へ進む前の確認。[07](07_Understand_Then_Harder.md)。
