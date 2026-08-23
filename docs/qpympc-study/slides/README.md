# 教育用スライド（Quadruped-PyMPC）

口頭用である。定義・境界表・フラグの正本は親ディレクトリの Markdown とコードである。食い違いがあればコードを正し、Markdown を直し、このフォルダを再生成する。

## 対象

四足MPCが初めての大学院生。専門語は、使う直前のページで定義する。数式の記号は覚えなくてよい。そのページの変数表を見る。

## どれを開くか

| ファイル | 役割 | 長さの目安 |
|---|---|---|
| [05_md_visual_summary.pptx](05_md_visual_summary.pptx) | **推奨速習。** 本文を犬の絵・力・時間・式で要約。箱の列ではない | 口頭 45–70 分 |
| [06_md_visual_chapters.pptx](06_md_visual_chapters.pptx) | 各本文を、1枚の光景と1つの式に縮める | 自習 90–150 分 |
| [01_quickstart_qpympc.pptx](01_quickstart_qpympc.pptx) | 残置。`00` §6.1 の見出し順 | 口頭 60–90 分 |
| [02_deep_dive_qpympc.pptx](02_deep_dive_qpympc.pptx) | 残置。`00`–`19` の `##` 順 | 口頭 3–5 時間 |
| [03_md_summary_quick_qpympc.pptx](03_md_summary_quick_qpympc.pptx) | 残置。先の Markdown 要約 | — |
| [04_md_summary_chapters_qpympc.pptx](04_md_summary_chapters_qpympc.pptx) | 残置。先の章ごと要約 | — |

図は次を指す。パイプラインの角丸箱ではない。

- 犬の姿（物体）
- 力・座標が体のどこか
- 歩容の時間
- 人 → 今の犬 → 次の犬、という光景

数式は1式1ページとし、背景・意図・変数説明を欠かない（05）。06 は式を光景の下に置く。

## 再生成

```text
# 見出し順（01 / 02）。上書きするが、05 / 06 は触らない
/home/takuya/work/mpc_dog/.venv/bin/python docs/qpympc-study/slides/build_pptx.py

# 図と式の要約（05 / 06）。01〜04 は触らない
/home/takuya/work/mpc_dog/.venv/bin/python docs/qpympc-study/slides/build_visual_summary.py
```

依存: `python-pptx`、`matplotlib`、`Pillow`、フォント `Noto Sans CJK JP`。式画像は `_eqcache/`、作図は `_figcache/`（どちらも git 対象外）。犬の参照絵は `_img/`。

## 正本との対応

| スライド | Markdown |
|---|---|
| 05 の順 | [00](../00_README.md) §6.1 に、物体・粗い胴体・着地点・歩幅を足した要約 |
| 06 の順 | `00`–`19` の結論。見出し全節は写さない |
| 出口の確認 | [02](../02_System_Architecture_and_Dataflow.md) §6 |

制御コードは、スライド作成では変更しない。
