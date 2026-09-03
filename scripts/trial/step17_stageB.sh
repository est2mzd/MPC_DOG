#!/usr/bin/env bash
# Step 17b Stage B: G1(gait=実質STAND) + G3(stand_pos_error 広げ) + W1(姿勢重み)
# だけ入れて、その場・垂直ジャンプを 5 回走らせ、再現性とホップ高さの分散を測る。
#
# 合格条件: 5/5 で 着地後 2 s の |roll|,|pitch| < 0.1 rad、転倒 0、NMPC 失敗 0。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPO=$(pwd)
N="${N:-5}"
OUT="$REPO/artifacts/step17/stageB"
mkdir -p "$OUT"
SUM="$OUT/summary.csv"
echo "iter,built,nmpc_fails,airborne_ms,hop_rise_m,max_roll,max_pitch,post2s_maxroll,post2s_maxpitch,fell,verdict" > "$SUM"

for i in $(seq 1 "$N"); do
  TAG="step17_stageB_$i"
  echo "=== Stage B run $i / $N ($TAG) ==="
  # clear any stale output so a startup-failed run cannot be scored on an old CSV
  rm -rf "$REPO/artifacts/step17/$TAG"
  JUMP_TAKEOFF_VX=0.0 JUMP_DZ_LO=1.1 JUMP_DZ_HI=1.5 JUMP_TS_LO=0.20 JUMP_TS_HI=0.28 \
    JUMP_PRELOAD_FRACTION=1.0 JUMP_FRONT_LAND_FRACTION=0.0 JUMP_ATT_WEIGHT=20 JUMP_CROUCH_VZ=0.4 \
    STEP_TAG="$TAG" bash scripts/trial/run_step17_jump.sh > "$OUT/${TAG}.log" 2>&1
  # restore configs defensively (harness trap should have, but be sure)
  git checkout -- external/quad-sdk/quad_utils/config/go2.yaml \
    external/quad-sdk/local_planner/config/local_planner.yaml \
    external/quad-sdk/global_body_planner/config/global_body_planner.yaml 2>/dev/null || true
  CSV="$REPO/artifacts/step17/$TAG/state_log.csv"
  BUILT=$(grep -c "forced jump\] built" "$OUT/${TAG}.log")
  NF=$(grep -c "NMPC solving fail" "$OUT/${TAG}.log")
  python3 - "$CSV" "$i" "$NF" "$BUILT" >> "$SUM" <<'PY'
import csv, sys
csvp, it, nf, built = sys.argv[1:5]
def f(x):
    try: return float(x)
    except: return None
try:
    rows=list(csv.DictReader(open(csvp)))
except FileNotFoundError:
    print(f"{it},{built},{nf},,,,,,,1,NO_CSV"); sys.exit()
t=[f(r['sim_time_s']) for r in rows]
bz=[f(r['base_pos_z_m']) for r in rows]
ro=[f(r['base_roll_rad']) for r in rows]
pi=[f(r['base_pitch_rad']) for r in rows]
ph=[r.get('jump_phase') or '' for r in rows]
legs=['FL','BL','FR','BR']
fz={l:[f(r[f'foot_{l}_pos_z_m']) for r in rows] for l in legs}
j0=next((k for k in range(len(rows)) if ph[k] in ('preload','rear_push','leap_stance')), None)
if j0 is None:
    print(f"{it},{built},{nf},,,,,,,1,NO_JUMP"); sys.exit()
# airborne window
best=0.0; cs=None
for k in range(j0,len(rows)):
    air=all((fz[l][k] is not None and fz[l][k]>0.06) for l in legs)
    if air and cs is None: cs=t[k]
    elif not air and cs is not None: best=max(best,t[k]-cs); cs=None
if cs is not None: best=max(best,t[-1]-cs)
zpre=bz[j0]; zap=max(v for v in bz[j0:] if v is not None); apx=bz.index(zap)
mr=max(abs(v) for v in ro[j0:] if v is not None)
mp=max(abs(v) for v in pi[j0:] if v is not None)
# 2s post landing: from apex+ ~0.4s window start, take [tland, tland+2]
tland=t[apx]+0.35
win=[k for k in range(len(rows)) if tland<=t[k]<=tland+2.0]
p2r=max((abs(ro[k]) for k in win if ro[k] is not None), default=9)
p2p=max((abs(pi[k]) for k in win if pi[k] is not None), default=9)
fell=1 if any(bz[k] is not None and bz[k]<0.15 for k in range(j0,len(rows))) else 0
ok = (fell==0 and int(nf)==0 and p2r<0.1 and p2p<0.1 and best>=0.03)
print(f"{it},{built},{nf},{best*1000:.0f},{zap-zpre:.3f},{mr:.2f},{mp:.2f},{p2r:.3f},{p2p:.3f},{fell},{'PASS' if ok else 'FAIL'}")
PY
done

echo
echo "=== Stage B summary ($SUM) ==="
column -t -s, "$SUM"
