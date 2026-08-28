# Session 4 — 5 kph × 凸凹地形 試行錯誤ログ

**目標:** 指令速度 **5 kph**（≈1.39 m/s）、累積走行 **20 m 以上**、3 シナリオ（凸凹平坦 / 上り坂 / 下り坂）。

**成功判定（本ワークショップ）:**

| モード | 条件 |
|--------|------|
| no-fall | 転倒なし & 20 m & 平均速度 ≥ 4 kph |
| resilient | 累積 20 m & 転倒回数 ≤ max_falls（転倒後 env reset） |

---

## 1. 平坦ベースライン（scene=flat）

| target kph | distance | mean kph | 結果 |
|------------|----------|----------|------|
| 2.0 | 20 m | ~2.0 | ✅ no-fall |
| 3.0 | 20 m | ~3.0 | ✅ no-fall |
| 5.0 | ~6 m | — | ❌ 転倒 |

**学び:** 5 kph は平坦でも現行 trot + nominal MPC では **単一セグメント no-fall 不可**。速度ランプ（10–20 s）必須だが、それだけでは不十分。

---

## 2. 凸凹平坦（bumpy_flat）

Perlin heightfield、全局傾斜なし（`workshop_terrain.py`）。

### no-fall スイープ

| kph | distance | falls | 結果 |
|-----|----------|-------|------|
| 5.0 | 3.7 m | — | ❌ |
| 4.5 | 4.3 m | — | ❌ |
| 4.0 | 4.8 m | — | ❌ |
| 3.5 | 5.6 m | — | ❌ |

### 調整の方向

- `step_freq` ↓（1.35 → 1.20）
- `duty_factor` ↑（0.74 → 0.76）
- `mu` ↓（0.42）
- `speed_ramp_s` ↑（18 s）
- `ref_z_scale` 1.07

### ✅ 勝ちパラメータ（resilient）

```
mu=0.42  step_freq=1.20  duty=0.76  ref_z=1.07  ramp=18s
→ 20.0 m, 17 falls, target 5 kph
```

---

## 3. 凸凹上り坂（bumpy_uphill）

Perlin + pitch **+0.08 rad**（+x 方向が上り）。

### no-fall

全 kph（3.5–5.0）で **6 m 未満** で転倒。

### ✅ 勝ちパラメータ（resilient）

```
mu=0.38  step_freq=1.10  duty=0.78  ref_z=1.08  ramp=20s
→ 20.0 m, 16 falls, target 5 kph
```

**学び:** 上りは mu をさらに下げ、duty を上げる（支持長め）。

---

## 4. 凸凹下り坂（bumpy_downhill）

Perlin + pitch **-0.08 rad**（+x 方向が下り）。**最難関**。

### 第1ラウンド（mu=0.38, duty=0.77）

| mode | distance | falls | 結果 |
|------|----------|-------|------|
| no-fall 5 kph | 3.1 m | — | ❌ |
| resilient 5 kph | 12.0 m | 21 | ❌ |

### 第2ラウンド（より保守的）

| 試行 | パラメータ | distance | falls | 結果 |
|------|-----------|----------|-------|------|
| d1 | mu=0.35, freq=1.05, duty=0.82, ref_z=1.10, ramp=22s | **20.0 m** | 24 | ✅ |

**学び:** 下り坂は **duty=0.82** と **低 mu** が必須。平均速度は 0.5 kph 程度と低いが、**指令は 5 kph ランプ**（実際の vx は地形・転倒で変動）。

---

## 5. 技術メモ

1. **制御ループ** は `simulation.py` と同一（`tau` soft clip 0.9、`joints_pos` from `legs_qvel_idx`、`return_rot_jac=False`）。
2. **resilient モード:** 転倒で `env.reset()` + `wrapper.reset()`、累積距離を加算。
3. **JSON ログ:** `assets/speed_terrain_trial_log.json`（全試行）、`assets/speed_terrain_results.json`（勝者のみ）。
4. **プリセット:** `configs/pympc_presets/session04_bumpy_{flat,uphill,downhill}.yaml`

---

## 6. デモ GIF

| 地形 | GIF |
|------|-----|
| 凸凹平坦 | `demo_s04_flat.gif` |
| 凸凹上り | `demo_s04_uphill.gif` |
| 凸凹下り | `demo_s04_downhill.gif` |

再生成:

```bash
source .venv/bin/activate && . .env.workshop
python scripts/capture_speed_terrain_demos.py
```
