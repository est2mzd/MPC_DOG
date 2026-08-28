# legged_control 理論・コード学習ガイド

## 1. 目的

このディレクトリは、`qiayuanliao/legged_control` を理論とコードの両面から理解し、Cursorで継続的に分析・更新するための学習ノートである。

対象は、ユーザー速度指令とGait指令から、目標軌道、状態推定、Centroidal NMPC、WBC、関節ハイブリッド指令、Gazebo / 実機モータまでの閉ループ全体である。

## 2. 対象コード

- legged_control: <https://github.com/qiayuanliao/legged_control>
- 依存（このリポジトリ外）: [OCS2](https://github.com/leggedrobotics/ocs2)、pinocchio、ros-control、Unitree SDK

この更新で照合した対象は次である。

- `external/legged_control` のコミット `a7f381c0367e98e31c01336e678eef47e304d40d`（2025-02-13、`master`）
- 数値例の既定ロボットは README の `ROBOT_TYPE=a1`
- OCS2本体（`LeggedRobotDynamicsAD`、`GaitReceiver`、`GaitSchedule` 内部）はこの workspace に無い。OCS2側の式は README と公開APIの使われ方から復元し、未確認は明示する

作者注記: 本ソフトウェアは新規開発を止めており、後継は [legged_perceptive](https://github.com/qiayuanl/legged_perceptive) である。

## 3. 最初に理解する一本の経路

概念図である。型・単位・frame付きの境界契約は[02 全体データフロー](02_System_Architecture_and_Dataflow.md)を正本とする。

```
                    Gait 指令
                    ModeSchedule
                         |
                         v
┌──────────────┐   ┌──────────────┐   ┌──────────────────┐
│ ① cmd_vel    │   │ ③ 状態推定   │   │ ④ NMPC 100 Hz    │
│    / goal    │   │    500 Hz    │   │                  │
│ Twist 4 または│   │ rbd 36       │-->│ x 24 を初期状態  │
│ Pose 6       │   │ → x 24       │   │                  │
└──────┬───────┘   └──────┬───────┘   └────────┬─────────┘
       │                  │                    ^
       │ 状態 24 x 2点    │                    |
       v                  |                    |
┌──────────────┐          |                    |
│ ② Target     │----------+--------------------+
│ Trajectories │
└──────────────┘

       ④ の現在解 x* 24, u* 24, mode
                         |
                         v
                ┌──────────────────┐
                │ ⑤ WBC 500 Hz     │<-- ③ の rbd 36
                │ tau = tail 12    │
                └────────┬─────────┘
                         | 12関節トルク
                         v
                ┌──────────────────┐
                │ ⑥ ハイブリッド   │
                │ q*, dq*, Kp=0    │
                │ Kd=3, ff=tau     │
                └────────┬─────────┘
                         v
                ┌──────────────────┐
                │ ⑦ モータ/Gazebo  │
                └────────┬─────────┘
                         | IMU, 関節, 接地
                         v
                        ③ へ戻る
```

Gaitは胴体目標軌道と独立である。READMEも「THE GAIT AND THE GOAL ARE COMPLETELY DIFFERENT AND SEPARATED」と書いている。

## 4. 推奨学習順序

1. [パッケージと制御ループ](01_Packages_and_Control_Loop.md)
2. [全体データフロー](02_System_Architecture_and_Dataflow.md)（Q1）
3. [ユーザー指令と目標軌道](03_User_Command_and_Reference.md)（Q2 ①②）
4. [状態推定](04_State_Estimation.md)（Q2 ③）
5. [NMPC](05_NMPC.md)（Q2 ④）
6. [WBC](06_WBC.md)（Q2 ⑤）
7. [関節制御とハードウェア](07_Joint_Control_and_Hardware.md)（Q2 ⑥⑦）
8. [会話論点カバレッジ](08_Conversation_Coverage_Map.md)

口頭用スライドは [slides/](slides/README.md) にある。01/02 は本ノートの章立て、03/04 は犬モデル・式の対応・アーキテクチャの要約である。

変数の完全一覧は[Appendix A](appendices/A_Variable_Dictionary.md)である。

## 5. 記述上の区別

各章では、次を混同しない。

- **実装事実**: 現行コードに存在する処理。
- **理論**: 実装を説明する数式・原理。OCS2本体にあり本リポジトリに無いものは「OCS2側」と書く。
- **推奨改善**: 現行コードにはないが、より明示的・堅牢にする案。
- **未実装 / 未確認**: 標準経路に無い機能、または OCS2 ソース未照合。

## 6. 座標系と記号

- (W): world / odom
- (B): base
- ZYX Euler: \(\theta = (\psi, \theta, \phi)\) = yaw, pitch, roll。centroidal状態の姿勢はこの順
- 関節順（コントローラ）: LF, LH, RF, RH。各脚 HAA, HFE, KFE
- 接触名の添字 \(i\): `modelSettings().contactNames3DoF` の順。OCS2既定コメントは LF, RF, LH, RH
- \(\mathbf{x}\in\mathbb{R}^{24}\): NMPC状態
- \(\mathbf{u}\in\mathbb{R}^{24}\): NMPC入力
- \(\mathbf{x}_{\mathrm{rbd}}\in\mathbb{R}^{36}\): 剛体状態（推定出力）
- \(\mathbf{x}_{\mathrm{wbc}}\in\mathbb{R}^{42}\): WBC決定変数

## 7. 重要な結論

- 標準入力は目的地ではなく `/cmd_vel` の胴体速度である。`/move_base_simple/goal` は別経路。
- 目標軌道は未来全体の密な軌道ではなく、現在と `timeHorizon=1.0` s 先の **2点** である。
- NMPC状態は「胴体12 + 関節12」ではなく、正規化centroidal運動量6 + 胴体姿勢6 + 関節12である。
- NMPCは100 Hzスレッド、WBCと関節指令は500 Hz。WBCは最新policyを現在時刻で評価した1点だけを使う。
- 既定WBCは階層QPではなく `WeightedWbc`（単一QP）。`HierarchicalWbc` は実装されているが未配線。
- モータ指令は \(\tau_{\mathrm{WBC}}\) を feedforward、位置ゲイン \(K_p=0\)、速度ゲイン \(K_d=3\) である。
- Gait位相はNMPCが選ばない。`legged_robot_gait_command` が `ModeSchedule` として与える。

## 8. Cursor運用

Cursorには最初に本ファイル、`02_System_Architecture_and_Dataflow.md`、`01_Packages_and_Control_Loop.md`を読み込ませ、その後、調査対象の章だけを追加する。コードとノートが食い違った場合は、コードのコミットを記録し、ノートを更新する。
