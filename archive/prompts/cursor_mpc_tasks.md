# Cursor タスク — Quadruped-PyMPC / MuJoCo MPC / OCS2

出発点は **Quadruped-PyMPC**。go2-convex-mpc は使わない。

---

## Phase 1: Quadruped-PyMPC セットアップ

```
external/Quadruped-PyMPC の README_install.md に従い acados をビルドし、
pip install -e . まで完了させて。Linux 前提。
エラーは1つずつ直す。数式説明不要。
```

---

## Smoke test

```
quadruped_pympc/config.py で mpc_type=gradient, foothold_optimization=false に設定。
MuJoCo sim を起動するコマンドを実行。成功/失敗とログ1行だけ報告。
```

---

## 足場最適化 ON

```
config.py で foothold_optimization=true のみ変更。
rough terrain シーンがあればそれで実行。転倒したら acados ステータスと
config のどのキーを次に触るか1つ提案。
```

---

## Sampling MPC (MPPI)

```
config.py で mpc_type=sampling, strategy=mppi。
JAX GPU 有無を確認してから実行。IROS 2024 設定に合わせる。
```

---

## Phase 2: MuJoCo iLQR

```
external/mujoco_mpc_go1 (go1 branch) をビルド。
Quadruped task を GUI で起動する手順だけ。Whole-body iLQR のログ項目を列挙。
```

---

## Phase 3: OCS2 perceptive

```
ocs2_ros2 の Perceptive Locomotion example のビルド依存だけリストアップ。
quadruped_ros2_control との関係を3行で。
```

---

## コンサル1枚

```
Quadruped-PyMPC vs MuJoCo iLQR vs OCS2 perceptive を
「先端性/実機/不整地/構築コスト」の4軸表に。数式なし。
```
