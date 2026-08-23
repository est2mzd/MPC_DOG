# Corrections and Clarifications

本文から除いた旧説明の理由だけを残す。正しい説明の再掲はしない。正本は各本文章と[Variable Dictionary](A_Variable_Dictionary.md)である。

## 1. 「PDがなければ立てない」

訂正：関節角PDは必須ではないが、何らかの閉ループ駆動は必要である。標準構成は胴体誤差→MPC GRF→Jacobian転置Torqueで立つ。正本は[11](../11_Joint_Torque_and_MuJoCo_Closed_Loop.md)。

## 2. 「MPCがTrotの逆相を回答する」

訂正：固定Schedule MPCでは位相は決定変数でない。Gait Generatorの\(c_{i,k}\)を力学パラメータとして使う。正本は[04](../04_Gait_Generator_and_Contact_Schedule.md)、[08](../08_Gait_MPC_Coupling.md)。

## 3. 「接地Scheduleは数式上不要」

訂正：固定Gaitでは、将来利用可能なGRFと移動可能な足を指定するため必要である。不要にできるのはMixed-integer/Contact-implicit等で接触自体を最適化する場合。

## 4. 「遊脚GRFはOCPで厳密にゼロ」

訂正：出力Mask \(F^{cmd}=c_0 F^{MPC}\) で指令はゼロになる。OCP内部に等式 \(F_i=0\) は無い。力学は \(c_i F_i\)、摩擦は全脚常時、yref遊脚 \(F_z=0\)。初版では内部制約を未検証とした。正本は[09](../09_MPC_Output_and_Receding_Horizon.md) §6。設計意図だけ[F](F_Open_Questions.md)。

## 5. 「周波数が速度を決める」

訂正：速度、周波数、接地点間隔は\(v=fL\)で連成する。システムでは速度を上位指令とし、周波数に応じてFootholdを変えるが、実現可能範囲がある。正本は[12](../12_Speed_Frequency_Duty_and_Stride.md)。

## 6. 「5 m/s、2 Hzでは脚を2.5 m前へ置く」

訂正：2.5 mは同一脚の連続する地面上の接地点間隔で、胴体相対Touchdown伸展量ではない。それでもStance中の胴体相対移動が大きく、Go2には非現実的である。

## 7. 「定常高速なら小歩幅でよい」

訂正：定常時は平均水平GRFを小さくできるが、足が地面に固定されるため、低周波のまま小さい接地点間隔にはできない。小歩幅なら高Cadenceが必要である。

## 8. 「Footholdを安全位置へ動かせば地形対応完了」

訂正：地形安全性だけでなく、IK可到達性、残りSwing時間、接触Timing、胴体速度との整合が必要である。正本は[13](../13_Feasibility_on_Rough_Terrain.md)。

## 9. 「Quadruped-PyMPCは完全なWBC」

訂正：標準`WBInterface`は立脚`-J.T@F`とSwing制御を統合するが、一般的な全身QP-WBCと同一ではない。正本は[10](../10_Stance_and_Swing_Control.md)。

## 10. 「MPC GRFをMuJoCoへ直接入力」

訂正：GRFを関節Torqueへ変換し、MuJoCo接触Solverが実接触力を求める。正本は[11](../11_Joint_Torque_and_MuJoCo_Closed_Loop.md)。

## 11. 「キー入力が標準の指令生成」

訂正：全モードの指令初期化は`QuadrupedEnv._sample_ref_vel()`である。`_key_callback()`は`render()`でviewerを開いたときだけ登録される。正本は[03](../03_User_Command_and_Reference_Generation.md)。

## 12. 「`ref_state`のキーは`ref_foot_FL_constraints`」

訂正：現行コードのキーは`ref_foot_constraints_FL`（他脚も同様）。正本は[03](../03_User_Command_and_Reference_Generation.md)と[A](A_Variable_Dictionary.md)。

## 13. 「Call graphでTerrainEstimatorは末尾」

訂正：`WBInterface.update_state_and_reference()`では`TerrainEstimator.compute_terrain_estimation()`が最初である。正本は[16](../16_Code_Map_and_Call_Graph.md)。

## 14. 「`joints_pos`は関節角」

訂正：現行`simulation.py`は`joints_pos`に`legs_qvel_idx`（整数index）を入れている。nominal MPCはこのキーを読まない。正本は[16](../16_Code_Map_and_Call_Graph.md)と[A](A_Variable_Dictionary.md)。

## 15. 「確認コミットは`cc145a2` / PyMPCは`3adfad9`」

訂正：学習資料初版のzip記録は`cc145a2`である。`3adfad9f814c499fb996cf046c8fb4ac3a574e55` は wrapper リポジトリ `mpc_dog` の HEAD であり、`external/Quadruped-PyMPC` のgit commitではない（展開ディレクトリに`.git`がない）。正本は[00](../00_README.md)。

## 16. 「Go2 XMLにセンサがない」

訂正：`go2.xml`にはIMU系4個と12`jointpos`の計16センサがある。標準`run_simulation()`は`sensors=None`かつ`sensordata`未読である。正本は[01](../01_MuJoCo_Go2_Plant_Model.md)。

## 17. 「実行時モデルはMenagerie Go2」

訂正：実行時にロードするのはgym-quadruped同梱`robot_model/go2/go2.xml`である。Menagerieは上流参照リンクであり、HEADとの同一性は未確認。正本は[01](../01_MuJoCo_Go2_Plant_Model.md)、未確認は[F](F_Open_Questions.md)。

## 18. 「本スタックに通常版とMJX版の接触差がある」

訂正：照合したQuadruped-PyMPCとgym-quadruped 1.1.5にMJX切替はない。接触差の表は根拠がない。正本は[01](../01_MuJoCo_Go2_Plant_Model.md)。

## 19. 「XML足摩擦が実行時摩擦」

訂正：`QuadrupedEnv.reset()`の`_set_ground_friction()`が床と足geomを上書きする。`run_simulation`既定は`friction_coeff=(0.5, 1.0)`。正本は[01](../01_MuJoCo_Go2_Plant_Model.md)。

## 20. 「終端コストは別行列 \(Q_N\)」

訂正：`LINEAR_LS` の `W_e=Q` であり、別の \(Q_N\) はない。正本は[07](../07_MPC_Formulation.md)。

## 21. 「Solver失敗時は前回GRFまたは基準鉛直」

訂正：`status in {1,4}` では `previous_optimal_GRF` のあと solver `reset()`。`mg/n_s` 代入は直後に上書きされる死文。正本は[07](../07_MPC_Formulation.md) §9。

## 22. 「GRF rate weight は nominal の調整項目」

訂正：\(R_{\dot F}\) は `type='input_rates'` 専用。標準`nominal`には無い。正本は[14](../14_MPC_and_Controller_Tuning.md)、[C](C_Parameter_Index.md)。

## 23. 「Reflex既定は tracking」

訂正：ディスク上 `reflex_trigger_mode=False`。`tracking` は有効時のモード名。正本は[C](C_Parameter_Index.md)、[16](../16_Code_Map_and_Call_Graph.md)。

## 24. 「周波数候補評価は Foothold も含めて \(J_{MPC}\) 最小」

訂正：標準は`optimize_step_freq=False`。有効時も評価ループは接触列の作り直しが主で、候補ごとにFootholdを再計算しない。目的に周波数penaltyが加わる。正本は[12](../12_Speed_Frequency_Duty_and_Stride.md) §6。

## 25. 「`stance_proximity` が有効」

訂正：代入は `1*0` のため常に0。正本は[06](../06_Centroidal_SRBD_Model.md) §7。

## 26. 「external wrench に内部推定がある」

訂正：標準経路は Wrapper が渡さず `zeros(6,)`。推定器は無い。フラグがTrueの理由だけ[F](F_Open_Questions.md)。正本は[09](../09_MPC_Output_and_Receding_Horizon.md) §3.3。

## 27. 「`mpc_frequency` は `mpc_params`」

訂正：ディスク上は `simulation_params['mpc_frequency']`。Wrapper もそこを読む。正本は[C](C_Parameter_Index.md)、[02](../02_System_Architecture_and_Dataflow.md) §5。
