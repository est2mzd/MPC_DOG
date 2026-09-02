#!/usr/bin/env bash
# 全シナリオ動作確認GIF。制御パラメータは1セットに固定(stop-only:
# multistep_planner.enabled=true + apply_stop_request=true, apply_foothold=false,
# edge_clearance=0, クロール)し、world だけ変えて撮った mp4 を GIF へ変換する。
# 録画は scripts/trial/allscenarios_record.sh の出力(quadsdk_quadsdk_allrec_*)。
#
# 切り出し区間は state_log から自動で決める:
#   ws  = base_pos_x が初期値から 0.12 m 動き出した sim_time(= 歩き始め)
#   off = mp4_dur - log_dur(録画は logger より先に始まるため、その頭出し分)
#   ss  = ws + off - 5   … 歩き始めの約2秒前から
#   t   = min(26, mp4_dur - ss - 2)   … 結末(通過 or 手前停止)まで入るように
# 出力は artifacts/gifs/quadsdk_allsc_*.gif。
set -euo pipefail
cd /home/takuya/work/mpc_dog
MG=scripts/trial/make_gif.sh
G=artifacts/gifs
L=artifacts/logs
mkdir -p "$G"

# out-name              allrec-tag
rows=(
  "01_flat|flat"
  "02_gaps2m|gaps2m"
  "03_repgap15_n5|repgap_n5"
  "04_trench15|trench15"
  "05_trench25|trench25"
  "06_trench30|trench30"
  "07_trench35|trench35"
  "08_trench50_stop|trench50"
  "09_trench100_stop|trench100"
  "10_composite_stop|composite"
  "11_trench30_v050_fall|trench30_v050"
)

for r in "${rows[@]}"; do
  IFS='|' read -r name tag <<<"$r"
  d="$L/quadsdk_quadsdk_allrec_$tag"
  src="$(ls "$d/logs/"*.mp4 2>/dev/null | head -1 || true)"
  csv="$d/state_log.csv"
  if [ -z "${src:-}" ] || ! ffprobe -v error -show_entries format=duration -of csv=p=0 "$src" >/dev/null 2>&1; then
    echo "SKIP $name : mp4 missing/unreadable ($d)"; continue
  fi
  read -r ss t < <(python3 - "$src" "$csv" <<'PY'
import sys, subprocess, csv
mp4, csvp = sys.argv[1], sys.argv[2]
mdur = float(subprocess.check_output(
    ["ffprobe","-v","error","-show_entries","format=duration","-of","csv=p=0",mp4]).strip())
rows = list(csv.reader(open(csvp)))
hdr, data = rows[0], rows[1:]
xi, ti = hdr.index("base_pos_x_m"), hdr.index("sim_time_s")
x0 = float(data[0][xi]); ws = None
for row in data:
    if abs(float(row[xi]) - x0) > 0.12:
        ws = float(row[ti]); break
ldur = float(data[-1][ti])
if ws is None: ws = max(0.0, ldur - 20.0)
ss = max(0.0, ws + (mdur - ldur) - 5.0)
t  = min(26.0, mdur - ss - 2.0)
print(f"{ss:.1f} {t:.1f}")
PY
)
  echo "$name : ss=$ss t=$t"
  bash "$MG" "$src" "$G/quadsdk_allsc_${name}.gif" 10 440 "$ss" "$t"
done
echo "ALLSC GIF DONE"
ls -la "$G"/quadsdk_allsc_*.gif
