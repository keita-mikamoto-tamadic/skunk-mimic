# docker_isaaclab — Isaac Lab 3.0 コンテナ

`~/ros_for_isaaclab/isaaclab_ws` からの移植 (2026-11-01)。**ROS 2 関連は外してある**
(ros2_ws / xr_ws コンテナは移植せず、compose の ROS ブリッジ環境変数も削除。
必要になったら ros_for_isaaclab 側の構成を参照して足す)。
dora (skunk-mimic 連携、`sim_base/dora_bridge.py` 用) はイメージに残している。

## ロボットファイルの置き場 (二重管理しない)

アセットの**正本はこのリポジトリの `robot_file/`**:

```
skunk-mimic/robot_file/
├── <robot>/            # CSV (Fusion 質量プロパティ) や USD など、ロボットごとのフォルダ
│   └── mesh/           # メッシュ (USD/USDZ)。USD が相対参照するので名前は変えない
├── scripts/            # 変換スクリプト (build_*_usd.py など)
├── mimic_v2_5/         # mimic (車輪倒立二足) — CSV + model.json + mesh/ + MJCF 群 (正本は mjmodel_converter で生成)
├── fly_wheel/          # リアクションホイール倒立振子
├── rsarm_v0/           # RSARM
├── crx5ia_2f85/        # CRX-5iA + Robotiq 2F-85 組み立て
└── robotiq/            # Robotiq 公式アセット (元は submodule + Git LFS — 下の注意参照)
```

コンテナには `/workspace/robot_file` としてマウントされる。変換スクリプトは
`/workspace/robot_file` のコンテナ内絶対パスを既定にしているので、ホスト側の置き場には依存しない。

> **注意 (robotiq)**: `robot_file/robotiq/isaacsim_assets` は元リポジトリでは
> git submodule (Git LFS) だった。ここへは実体コピーしてある。skunk-mimic に
> コミットする場合は、submodule に戻すか Git LFS を設定するかを先に決めること
> (USD バイナリ ~60 MB をそのまま git に入れると履歴が膨らむ)。

## ビルドと起動

```bash
# 1) ベースイメージ (isaac-lab-base) を IsaacLab 側でビルド (初回のみ)
cd ~/IsaacLab && ./docker/container.py build base

# 2) このイメージ (dora 入り) をビルド
cd skunk-mimic/docker_ws/docker_isaaclab && docker compose build

# 3) 起動 (ホストの ~/IsaacLab を ISAACLAB_DIR で差し替え可)
docker compose up -d && docker compose exec isaaclab bash
```

起動時に `/workspace/isaaclab_ws/src` 以下の `pyproject.toml` を持つパッケージが
自動で `uv pip install -e` される (コンテナを作り直してもセットアップ不要)。

## 学習 (例: flywheel)

コンテナ内で (`isaaclab` は bashrc の alias → isaaclab.sh)。自作タスクは
`--external_callback <pkg>.tasks.register` で登録する:

```bash
# 本学習 (デフォルトでヘッドレス。--headless フラグは廃止された)
isaaclab train --rl_library rsl_rl --task Flywheel-Balance \
    --external_callback flywheel_sim.tasks.register --num_envs 4096

# 動作確認だけなら --num_envs 256 --max_iterations 3

# 再生 (Newton viewer 付き)
isaaclab play --rl_library rsl_rl --task Flywheel-Balance \
    --external_callback flywheel_sim.tasks.register --num_envs 16 --viz newton_gl
```

ログ/チェックポイントは `/workspace/isaaclab/logs/rsl_rl/<task>/` (named volume)。
旧 `scripts/reinforcement_learning/rsl_rl/train.py` 直叩きは IsaacLab 更新で
`scripts/reinforcement_learning/train.py --rl_library rsl_rl` に統合済み —
`isaaclab train` / `isaaclab play` サブコマンドを使うのが確実。

アクチュエータ定数 (トルク上限 0.38 N·m、armature、摩擦) は実機 SysID の同定値
(`robot_file/fly_wheel/fit_result.json`、skunk-mimic 側 `sysid/flywheel/`) を
`flywheel_sim/robots.py` に反映してある。

## src/ (自作タスク)

`src/` には ros_for_isaaclab から `sim_base` / `fanuc_sim` / `rsarm_sim` / `flywheel_sim` を
コピーしてある。**変換スクリプト (build_*) は各パッケージの `scripts/` から
`robot_file/scripts/` に移動した** (ロボットファイルと一緒に正本管理するため)。
view_* / check_* / sysid_* はシミュレーション側の道具なので各パッケージに残している。

## dora ブリッジ

`../../src/data_format` (AxisRef / AxisAct の正本 JSON) が `/workspace/skunk_data_format` に
読み取り専用でマウントされる。dora は skunk-mimic の採用コミット (docs/dora-upgrade.md) と
同一のものをイメージ内でソースビルドしている — バージョンが揃わないと register できない。
