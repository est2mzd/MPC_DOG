# 07  制御ロジック（新規）

既存の 01〜06 は変更していない。このデッキだけが制御の順番を絵で追う。

| ファイル | 用途 |
|---|---|
| [07_control_logic_qpympc.pptx](07_control_logic_qpympc.pptx) | 指令 → 歩容 → 着地点 → 予測の点 → マスク → 立脚/遊脚 → 12トルク |

図の約束は 05/06 と同じである。

- 物体は犬。予測の体は質量点と作用点。形も棒も無い
- 力は接地点で世界向き。重心へは合力とモーメント
- 立脚の脚は本物のリンク。ただし力はリンク方向には伝わらない（\(J^\top\)）

再生成（01〜06 は触らない）:

```bash
/home/takuya/work/mpc_dog/.venv/bin/python docs/qpympc-study/slides/build_control_logic.py
```

正本は [03](../03_User_Command_and_Reference_Generation.md)–[11](../11_Joint_Torque_and_MuJoCo_Closed_Loop.md)。
