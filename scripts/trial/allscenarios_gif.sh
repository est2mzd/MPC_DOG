#!/usr/bin/env bash
# 全シナリオ動作確認GIF。制御パラメータは1セットに固定(stop-only:
# multistep_planner.enabled=true + apply_stop_request=true, apply_foothold=false,
# edge_clearance=0, クロール)し、world だけ変えて撮った mp4 を GIF へ変換する。
# 録画は scripts/trial での再録画(quadsdk_allrec_*)を使う。
# 出力は artifacts/gifs/quadsdk_allsc_*.gif。
set -euo pipefail
cd /home/takuya/work/mpc_dog
MG=scripts/trial/make_gif.sh
G=artifacts/gifs
L=artifacts/logs
mkdir -p "$G"
mp4() { ls "$L/quadsdk_quadsdk_allrec_$1/logs/"*.mp4 2>/dev/null | head -1; }

# out-name                     allrec-tag       ss  t   (ss/t は歩行区間の切り出し)
rows=(
  "01_flat|flat|8|14"
  "02_gaps2m|gaps2m|9|24"
  "03_repgap15_n5|repgap_n5|9|24"
  "04_trench15|trench15|9|18"
  "05_trench25|trench25|9|18"
  "06_trench30|trench30|9|18"
  "07_trench35|trench35|9|18"
  "08_trench50_stop|trench50|8|18"
  "09_trench100_stop|trench100|8|18"
  "10_composite_stop|composite|9|24"
  "11_trench30_v050_fall|trench30_v050|8|15"
)

for r in "${rows[@]}"; do
  IFS='|' read -r name tag ss t <<<"$r"
  src="$(mp4 "$tag" || true)"
  if [ -z "${src:-}" ] || ! ffprobe -v error -show_entries format=duration -of csv=p=0 "$src" >/dev/null 2>&1; then
    echo "SKIP $name : mp4 missing or unreadable ($L/quadsdk_quadsdk_allrec_$tag)"
    continue
  fi
  out="$G/quadsdk_allsc_${name}.gif"
  bash "$MG" "$src" "$out" 8 400 "$ss" "$t"
done
echo "ALLSC GIF DONE"
ls -la "$G"/quadsdk_allsc_*.gif
