# 自分の `src/` へ技術ブロックを切り出す — 方針ノート

## 1. 目的

`external/` にある2リポジトリの技術を、**使い回しやすい関数・クラス**として切り、数式を理解したうえで **自分が管理する `src/`** に置く。各技術をブロックとして組み合わせられる状態を目指す。

このディレクトリは、既存の理論ノート（`docs/qpympc-study/`、`docs/legged_control/`）の再記述ではない。**分割可否・境界・抽出順・`src/` 案**の正本である。

## 2. 対象

| リポジトリ | 場所 | 照合コミット（実装事実） |
|---|---|---|
| [Quadruped-PyMPC](https://github.com/iit-DLSLab/Quadruped-PyMPC) | `external/Quadruped-PyMPC` | `3adfad9f814c499fb996cf046c8fb4ac3a574e55` |
| [legged_control](https://github.com/qiayuanl/legged_control) | `external/legged_control` | `a7f381c0367e98e31c01336e678eef47e304d40d`（2025-02-13） |

`src/` は現時点で空ディレクトリである（実装事実）。制御コードの正本は当面 `external/` のまま残す。

理論・変数・数式の詳細は次を正本とする。

- PyMPC: [docs/qpympc-study/00_README.md](../qpympc-study/00_README.md)
- legged_control: [docs/legged_control/00_README.md](../legged_control/00_README.md)

## 3. 結論（先に答える）

**ブロックに分けられる。ただし「リポジトリごと丸ごと移植」はできない。**

| 問い | 答え | 区分 |
|---|---|---|
| 概念として層は分かれているか | はい。両リポとも指令→参照→接地→MPC→下位トルク→プラントの層がある | 実装事実（呼出順がそうなっている） |
| 現行コードを関数単位でそのまま `import` できるか | ほぼできない。グローバル設定、dict契約、ROS/OCS2、acados codegen に縛られている | 実装事実 + 推測（§4） |
| 同じ役割の実装を差し替えられるか | 境界を自分で定義すれば可能。現状の型は互いに非互換 | 推測 |
| 両リポの「良い部品」を混成できるか | 一部は補完、一部は二者択一。混成には変換層が要る | 推測。根拠は[02](02_Can_We_Split.md)、[03](03_Unified_Block_Catalog.md) |

最初に切るべきは、ソルバではなく **純計算ブロック**（ゲイト、足場、地形、指令、安全、関節則）である。NMPC は最後である。根拠と順は[09](09_Extraction_Policy.md)。

## 4. 記述の区別

各章で次を混同しない。見出しまたは文頭で明示する。

| 区分 | 意味 |
|---|---|
| **実装事実** | 現行コード・設定ファイル・既存学習ノートで確認できること。パスと関数名を書く |
| **理論** | 実装を説明する数式。詳細は既存ノートへリンクする |
| **推測** | コードから合理的に言えるが、実行検証や完全照合をしていないこと |
| **方針** | このノートが決める今後のやり方。コードにはまだ無い |

「ブロックに分けられる」は **方針** であり、今日の `src/` の事実ではない。

## 5. ファイル一覧

| 順序 | ファイル | 目的 |
|---|---|---|
| 0 | [00_README.md](00_README.md) | 入口、結論、読み方 |
| 1 | [01_External_Inventory.md](01_External_Inventory.md) | 2リポに何があるか（在庫） |
| 2 | [02_Can_We_Split.md](02_Can_We_Split.md) | 分割可否と結合度 |
| 3 | [03_Unified_Block_Catalog.md](03_Unified_Block_Catalog.md) | 共通ブロック一覧（B00–B13） |
| 4 | [04_PyMPC_to_Blocks.md](04_PyMPC_to_Blocks.md) | PyMPC ファイル → ブロック |
| 5 | [05_LeggedControl_to_Blocks.md](05_LeggedControl_to_Blocks.md) | legged_control ファイル → ブロック |
| 6 | [06_Interfaces_and_Contracts.md](06_Interfaces_and_Contracts.md) | 境界の型・次元・frame |
| 7 | [07_Equation_to_Block.md](07_Equation_to_Block.md) | 数式 → ブロック |
| 8 | [08_Src_Layout.md](08_Src_Layout.md) | `src/mpc_dog/` の配置案 |
| 9 | [09_Extraction_Policy.md](09_Extraction_Policy.md) | 抽出順、やってよいこと、禁止 |

## 6. 推奨読み順

1. 本ファイルの §3
2. [02](02_Can_We_Split.md) — 「分けられるのか」
3. [03](03_Unified_Block_Catalog.md) — 何をブロックと呼ぶか
4. [08](08_Src_Layout.md) と [09](09_Extraction_Policy.md) — 今後の手

ファイル対応の詳細は 04 / 05、境界と式は 06 / 07 である。

## 7. 概念図（方針）

両リポをリポ名ではなく **役割** で並べる。矢印はデータ、箱は将来の `src/` モジュールである。

```text
B01 指令 ──► B02 参照 ──► B08 NMPC ──► B09 下位制御 ──► B11 関節 ──► B13 プラント
                ▲            ▲              ▲
 tre           B03 ゲイト    B07 予測モデル   B10 遊脚
                ▲            ▲
               B04 足場     B06 推定
                ▲
               B05 地形
```

実装事実として、PyMPC は B06 を持たない（sim の真値が状態）。legged_control の B07/B08 本体は OCS2 側にあり、この workspace にソースが無い。

## 8. 既存ノートとの関係

| 欲しいもの | 読む場所 |
|---|---|
| PyMPC の1ループと変数 | `docs/qpympc-study/02`, `16`, Appendix A |
| PyMPC の式 | `docs/qpympc-study/04`–`10`, Appendix B |
| legged_control の1ループと型 | `docs/legged_control/01`, `02`, Appendix A |
| legged_control の式 | `docs/legged_control/03`–`07` |
| 自分の `src/` をどう切るか | **このディレクトリ** |

ノートとコードが食い違ったら、コードを正とし、このディレクトリも直す。
