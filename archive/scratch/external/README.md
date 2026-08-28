# scratch/external/ — external/ のゼロからの検証置き場

`external/Quadruped-PyMPC`・`external/legged_control`(いずれもgit submodule)
の挙動を実際に動かして確かめる作業を、ここで進めます。

## 置き方

1トピック(検証したい疑問・仮説1つ)につき、1サブフォルダを作る。

```text
scratch/external/<yyyymmdd>_<短い説明>/
  README.md   ← 何を確かめたいか(仮説)、結果、次にやること
  (検証スクリプト・ログ・図など、自由な構成)
```

例:

```text
scratch/external/20260829_pympc_friction_cone_sim_vs_mpc/
scratch/external/20260830_legged_control_gazebo_launch_smoke_test/
```

## 昇格の目安

- コードを読んだだけの分析結果 → `agent_reports/quadruped_pympc_onboarding/`
  または `agent_reports/legged_control_onboarding/`(`read_code_*.md`の
  作法に従う、各ディレクトリの`agent_instruction_*.md`参照)
- 実際に動かして得た再現可能なベンチマーク・デモ →
  `notebook_pympc/`・`notebook_legged/`
- 恒久的に使うツール・ラッパースクリプト → `scripts/`・`src/`

## ブランチとの関係

大きめの検証([過去の合意事項](../../AGENTS.md)により)は`experiment/*`
ブランチ上で行う想定。ブランチを切った場合も、置き場所のルール自体は
このディレクトリ構成を踏襲する。
