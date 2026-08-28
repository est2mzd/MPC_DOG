# Log 11: `contact_sequence` End-to-End

対応プロンプト: PGG → acados p → 力学 → GRF参照/制約 → 足速度 → Foothold抽出 → 出力mask → Stance/Swing。本文未修正。

## 境界表

値の例は Trot のある段 `[FL,FR,RL,RR]=[1,0,0,1]`。

| 順序 | 変数名 | shape | 値の例 | 生成元 | 使用先 | 数式上の役割 |
|---|---|---|---|---|---|---|
| 1 | `_phase_signal` | (4,) | `[0.24,0.74,0.74,0.24]` | `PGG.run` | `compute_contact_sequence` | \(\phi_i\) |
| 2 | `contact_sequence` | (4,12) | 列jが `[1,0,0,1]` | `PGG.compute_contact_sequence` | `ref`組立後のMPC、`current_contact` | \(C_{i,k}\) |
| 3 | `current_contact` | (4,) | `[1,0,0,1]` | `C[:,0]` | FRG, TE, STC, SRBD mask | \(c_{i,0}\) |
| 4 | `FL_contact_sequence[j]` 等 | (12,) | `1,0,0,1` の時刻列 | `compute_control` 分割 | `param[0:4]`, yref Fz, foothold抽出 | \(c_{i,k}\) |
| 5 | `param[0:4]` | (4,) | `[1,0,0,1]` | 上 | `solver.set(j,"p")` | 段kの接触 |
| 6 | `stanceFL`… | SX | pの先頭4 | `Centroidal_Model_Nominal` | \(\sum c_i F_i\), \(\dot p_i\) | Gate |
| 7 | `yref[44,47,50,53]` | scalar×4 | `[mg/2, 0, 0, mg/2]` | `mg/n_s * c` | 入力参照 | \(F_{z,i}^{ref}\) |
| 8 | `constr_lh/uh` 摩擦 | (20,) | Fz∈[0,mg] 全脚 | `create_friction_cone_constraints` | `h` | 錐。c非依存 |
| 9 | `u[0:12]` 足速度 | (12,) | 遊脚のみ効く | OCP | \(\dot p_i=(1-c_i)v_i\) | 遊脚移動 |
| 10 | `optimal_GRF` | (12,) | u0[12:] | `solver.get(0,"u")` | mask | \(F^{MPC}\) |
| 11 | `nmpc_GRFs` | 脚ごと(3,) | FL,RR残、FR,RL=0 | `F * c_0` | `-J.T @ F` | 指令GRF |
| 12 | `nmpc_footholds` | 脚ごと(3,) | 立脚=現在足、遊脚=次TD x | `compute_control` 抽出 | STC `touch_down` | \(p_{td}\) |
| 13 | `tau` 立脚 | (3,)×4 | FL,RRは `-J.T F` | `compute_stance_and_swing_torque` | clip→`action` | 支持 |
| 14 | `tau` 遊脚 | (3,) | FR,RLは Cartesian swing | `current_contact==0` | 同上 | 足軌道 |

## `[1,0,0,1]` のとき

- **胴体に寄与するGRF**: FL と RR。`temp += F_FL@1 + F_FR@0 + F_RL@0 + F_RR@1`。
- **固定される足**: FL, RR。\(\dot p=(1-1)v=0\)。
- **移動可能な足**: FR, RL。`use_foothold_optimization=True` かつ `s=0`。
- **GRF referenceがゼロ**: FR, RL の Fz参照（および全脚Fx,Fy）。FL,RR は `mg/2`。
- **OCP内部で遊脚GRFが厳密ゼロか**: いいえ。力学寄与は0。摩擦は `Fz∈[0,mg]` のまま。コストが0へ寄せる。
- **OCP出力後のMask**: `nmpc_GRFs.FR *= 0`, `RL *= 0`。先頭接触のみ。将来段のmaskはない。
- **stance torque**: 全脚で `-J.T@F` を先に入れる。FR/RLは F=0 なのでこの項は0。その後 `current_contact==0` で **上書き** して swing torque。
- **swing torque**: FR と RL。`touch_down=nmpc_footholds`。

## 方式比較

| 方式 | Gait scheduleの扱い | 接触が決定変数か | 計算量 | 現行実装か |
|---|---|---|---|---|
| Fixed schedule | 外部の \(c_{i,k}\) を p に入れる | いいえ | 小（標準） | **はい**（nominal） |
| Mixed-integer | \(c_{i,k}\in\{0,1\}\) も最適化 | はい | 大 | いいえ |
| Contact-implicit | \(F_z\ge0,\phi\ge0,F_z\phi=0\) | 相補性 | 大・非線形 | いいえ |

`optimize_step_freq` は接触時刻を連続最適化せず、離散候補周波数で接触列を作り直してコスト比較する。標準オフ。samplingのgait adaptiveも標準オフ。

## `08_Gait_MPC_Coupling.md` 検証

| 記載 | 判定 | 差分 |
|---|---|---|
| 位相は決定変数でない | 正しい | なし |
| 並進/回転Gate | 正しい | なし |
| 足速度Gate | 正しい | `s≡0` を未記載 |
| `solver.set(j,"p",param)` 先頭4が接触 | 正しい | なし |
| 出力後 mask | 正しい | なし |
| OCP内ゼロは未再検証 | 当時正しい | **本ログで再検証済み: 厳密ゼロではない** |
| 推奨 \(F_z\le c F_{z,max}\) | 推奨改善と明記 | 正しい（未実装） |
| 3方式表 | 正しい | なし |
| 低レベルと同一 `current_contact` | 正しい | なし |

`08` 本文はまだ直していない。F の「OCP内ゼロ未確認」は、このログで解消できる。
