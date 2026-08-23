# スライド

正本はノート `docs/legged_control` とコードである。

図は箱の番号合わせではない。犬モデル、式が犬のどこに載るか、プロセスとロボットのアーキテクチャである。

指令からトルクまでのロジックは、速さ指令と歩容名（stance / trot / flying_trot）を別蛇口とし、同じ接地旗が NMPC の制約と WBC の `plannedMode` に入る、と書く。

| ファイル | 用途 |
|---|---|
| [01_quickstart_legged_control.pptx](01_quickstart_legged_control.pptx) | ノート 00–07 を章の順で置いた速習 |
| [02_deep_dive_legged_control.pptx](02_deep_dive_legged_control.pptx) | 同じ章立て。各節まで残す |
| [03_summary_quickstart_legged_control.pptx](03_summary_quickstart_legged_control.pptx) | 要約・速習。犬・式・アーキ |
| [04_summary_deep_dive_legged_control.pptx](04_summary_deep_dive_legged_control.pptx) | 同じ図に、章ごとの式を足す |

再生成（01 / 02）:

```bash
/home/takuya/work/mpc_dog/.venv/bin/python docs/legged_control/slides/build_pptx.py
```

再生成（03 / 04）:

```bash
/home/takuya/work/mpc_dog/.venv/bin/python docs/legged_control/slides/build_pptx_summary.py
```
