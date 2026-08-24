# skunk-mimic

車輪倒立 2 足ロボット **mimic_v2_5** の統合コントローラ。
[dora-rs](https://dora-rs.ai/) のデータフローで、C++ ノード (リアルタイム制御・CAN 通信) と
Python ノード (MuJoCo シミュレーション・SysID・ツール類) を接続する。

```
dualsense_input ──state_command──▶ robot_control_manager ──motor_commands──▶ device_control_manager ──CAN──▶ moteus ×6
     │                               │  ▲                                      │            ▲
     │                               └── state_status ─▶ stabilizer ◀──────────┤ motor_status│imu_data
     └────────drive_command (前後/旋回/姿勢) ──────────▶ stabilizer ── run_command ─▶ RCM    │
                                                                               ◀─ raw_imu ── imu_node (Spresense)
```

- **robot_control_manager (RCM)** — 状態機械 (OFF/STOP/READY/RUN)。
- **device_control_manager (DCM)** — 3 ms 周期の CAN I/O (SCHED_FIFO、PREEMPT_RT)。
- **stabilizer** — 倒立制御 (angle_pid) + 姿勢 IK (Pinocchio、重心高さ変更)。
- **シミュレーション** — DCM + imu_node を `mujoco_backend` (Python) に差し替えるだけで
  同じ制御側バイナリがそのまま動く (`dataflow_mimic_sim.yaml`)。

| ディレクトリ | 内容 |
|---|---|
| `src/cpp/` | C++ ノード (RCM / DCM / stabilizer / imu_node)・共有ライブラリ・モータドライバ |
| `src/python/` | Python ノード (mujoco_backend, dualsense_input, web GUI, SysID, …)。uv workspace |
| `src/data_format/` | C++/Python 間バイナリ構造体の単一正本 (JSON → コード生成) |
| `robot_config/` | ロボット定義 JSON (`ROBOT_CONFIG` 環境変数で選択) |
| `sim/` | MuJoCo モデル。`mimic_v2_5.xml` は生成物、`scene.xml` は手書き |
| `scripts/` | オフライン解析 (LQR / A-B 行列 / MJCF 生成・倒立点補正)。別 uv workspace |
| `docs/` | 実機起動手順・RT 化・PCAN ドライバ・dora 更新などの詳細ドキュメント |
| `tools/` | データフォーマット生成、ブラウザモニタ |

アーキテクチャの詳細は [README_ARCH.md](README_ARCH.md)、開発時の約束事は [CLAUDE.md](CLAUDE.md) を参照。

> [!NOTE]
> **環境**
> * OS : Ubuntu 22.04 / 24.04 (実機は Jetson + PREEMPT_RT カーネル → [docs/rt-enable.md](docs/rt-enable.md))
> * dora **1.0.0-rc1** (ソースビルド。全マシンで同一コミットに揃える → [docs/dora-upgrade.md](docs/dora-upgrade.md))
> * [Apache Arrow](https://arrow.apache.org/) (libarrow-dev)
> * Python 3.11+ / [uv](https://docs.astral.sh/uv/)
> * Pinocchio ≥ 3 (stabilizer の姿勢 IK。`pip install pin`)

---

# 1. ビルド用依存関係インストール

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config git \
  clang lld \
  curl ca-certificates \
  libssl-dev libeigen3-dev
```

# 2. Rust と dora のビルド

### Rust ツールチェーン

```bash
curl https://sh.rustup.rs -sSf | sh
source ~/.cargo/env
cargo --version
```

### dora のビルド

> [!IMPORTANT]
> dora 1.0.0-rc 系は **Python 3.11 以上が必須**（PyO3 が `abi3-py311` を要求し、CLI 自体も
> これに依存する）。Ubuntu 22.04 のシステム Python は 3.10 なので、そのままだと
> `cannot set a minimum Python version 3.11 higher than the interpreter version 3.10`
> でビルドが失敗する。uv 管理の 3.11 を `PYO3_PYTHON` で指定して回避する
> (システム Python 3.10 は触らない)。

~/dora をクローンしてビルド（CLI + C++ API の両方が生成される）：

```bash
cd ~
git clone https://github.com/dora-rs/dora.git
cd dora
git checkout fd1f050b   # 採用コミット (1.0.0-rc1)。全マシンで揃える → docs/dora-upgrade.md
```

PyO3 に 3.11 を使わせる（uv 管理の 3.11 を指定。無ければ `uv python install 3.11`）：
```bash
export PYO3_PYTHON="$(uv python find 3.11)"
```

本体 build：
```bash
cargo build --release -p dora-cli
```

C++ API build：
```bash
cargo build --release -p dora-node-api-cxx
```

これで以下が生成される：
- `~/dora/target/release/dora` — CLI 実行ファイル
- `~/dora/target/release/libdora_node_api_cxx.a` — C++ 静的ライブラリ
- `~/dora/target/cxxbridge/dora-node-api-cxx/install/` — C++ ヘッダーファイル

### 環境変数設定

```bash
echo 'alias dora="$HOME/dora/target/release/dora"' >> ~/.bashrc
source ~/.bashrc
dora --version    # dora-cli 1.0.0-rc1
```

### /opt/dora symlink (Python 側 dora-rs 用)

`src/python` の `uv sync` は `dora-rs` を `/opt/dora/apis/python/node` からソースビルドする
(rc コミットには PyPI wheel が無いため)。マシン非依存パスにするための symlink を初回に 1 度だけ張る：

```bash
sudo ln -s ~/dora /opt/dora
```

### バージョンアップ・分散構成の注意

手順と「バージョン不一致の症状」(TCP は繋がるのに register が通らない等) は
[docs/dora-upgrade.md](docs/dora-upgrade.md)。要点:
- **全マシン (robot / PC) を同一コミットに checkout** してから CLI / C++ API / C++ ノード /
  `uv sync --reinstall-package dora-rs` を再ビルド。
- `dora-message` のワイヤフォーマットが違うとノード↔daemon 通信が壊れる
  ("message format version mismatch")。

### dora メトリクス収集と RT 性について

dora daemon は 2 秒ごとに `sysinfo` で各プロセスの CPU/メモリを `/proc` 走査して収集する。
v0.4.1 ではこれが daemon のメインループ上で同期実行され、RT ノード (DCM, SCHED_FIFO) に周期的な
レイテンシスパイクを起こしていた。**1.0-rc では upstream が改善済み** (収集は `spawn_blocking` に
退避) で自前パッチは不要。もし再計測でスパイクが出る場合のみ `binaries/daemon/src/lib.rs` の
`Event::MetricsInterval` ハンドラの `self.spawn_metrics_collection();` をコメントアウトする。
実測方法と切り分け済みの項目は [docs/rt_meas.md](docs/rt_meas.md)。

# 3. Apache Arrow

```bash
sudo apt install -y ca-certificates lsb-release wget
# Ubuntu 22.04 (jammy) の場合。24.04 は jammy → noble に読み替える
wget https://packages.apache.org/artifactory/arrow/ubuntu/apache-arrow-apt-source-latest-jammy.deb
sudo apt install -y ./apache-arrow-apt-source-latest-jammy.deb
sudo apt update
sudo apt install -y libarrow-dev
pkg-config --modversion arrow   # 確認
```

本プロジェクトでは `arrow/api.h`, `arrow/c/bridge.h` を使用する。

# 4. Python 環境セットアップ (uv)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv --version
```

uv workspace は 2 つある：

```bash
cd src/python && uv sync   # ランタイムノード (dora-rs, pyarrow, mujoco, ...)
cd scripts && uv sync      # オフライン解析 (mjmodel_converter, LQR, A-B 行列)
```

- `dora-rs` は `/opt/dora/apis/python/node` からソースビルドされ CLI とバージョンが揃う
  (§2 の symlink が前提)。初回は Rust 拡張のコンパイルで数分〜十数分かかる。
- 確認: `cd src/python && uv run python -c "import dora; print(dora.__version__)"` → `1.0.0-rc1`

# 5. Pinocchio (stabilizer の姿勢 IK)

`stabilizer` は Pinocchio ≥ 3 の C++ ライブラリで MuJoCo モデル (MJCF) を直接読む。
pip の `pin` (cmeel ビルド) が C++ ライブラリ・ヘッダ・CMake config ごと
`~/.local/lib/python3.*/site-packages/cmeel.prefix` に入り、stabilizer の CMakeLists が
自動でそこを探す (Jetson も同じ)：

```bash
pip install pin
```

robotpkg / ソースビルド等で入れた場合は通常の `find_package(pinocchio)` で見つかる
(別の場所なら `-DCMAKE_PREFIX_PATH=<prefix>`)。詳細は
[src/cpp/node/stabilizer/README_PIN_IK.md](src/cpp/node/stabilizer/README_PIN_IK.md)。

# 6. C++ ノードのビルド

```bash
git submodule update --init --recursive   # 初回のみ: OpenKF (stabilizer の EKF)
cd src/cpp && mkdir -p build && cd build && cmake .. && make
```

- 各ノードのバイナリは `src/cpp/node/<name>/build/<name>` に出る (dataflow yaml が参照するパス)。
- データフォーマット (`src/data_format/*.json` → `*_format.hpp/py`) は CMake configure 時に自動生成される。

# 7. 実行

## シミュレーション (PC、実機不要)

実機と同じ制御側 (RCM + stabilizer) を MuJoCo で動かす。DualSense パッドがあれば実機と同じ操作。

```bash
dora up
dora start dataflow_mimic_sim.yaml
# 別ターミナル (dynamic ノード。両方 attach するまで dataflow は開始バリアで待つ):
cd src/python && MUJOCO_MODEL=scene.xml MUJOCO_VIEWER=1 uv run mujoco_backend/mujoco_backend.py
cd src/python && uv run dualsense_input/dualsense_input.py   # パッド未接続でも起動する
```

- 操作: ○ SERVO_ON → △ READY → □ RUN、× STOP、L1+L3 SERVO_OFF。
  左スティック = 前後/旋回、右スティック上下 = 重心高さ (倒立点を変えない姿勢 IK)。
  ボタン番号は HID ドライバ差 (hid-generic / hid_playstation) を自動吸収する。
- ビューアのカメラは base_link を追従 (`MUJOCO_TRACK=0` で解除)。
- パッドが無い場合は `dataflow_mimic_sim.yaml` のコメントに従い `robot_web_gui` 等に切り替える。

## 実機 (mimic_v2_5)

起動チェックリストは **[docs/robot_start.md](docs/robot_start.md)**。概要:

```bash
bash pcan_setup.bash can0 can1 keep    # CAN (PEAK PCAN-M.2) → docs/pcan_socketcan_driver.md
bash keep_serial_open.bash             # IMU シリアル保持 (別ターミナル)
bash dora_rt_damon.bash                # RT daemon (別ターミナル) → docs/rt-enable.md
dora start dataflow_mimic.yaml
cd src/python && uv run dualsense_input/dualsense_input.py
```

ハードウェア無しで試すには robot_config の `"transport"` を `"dummy"` にする
(`DummyComm` が moteus 互換のダミーデータを返す)。

## dataflow 一覧

| YAML | 用途 |
|---|---|
| `dataflow_mimic.yaml` | **実機 mimic_v2_5 (現行メインフロー)** |
| `dataflow_mimic_sim.yaml` | **上記の MuJoCo シミュレーション版** |
| `dataflow_sysid.yaml` / `dataflow_sim_sysid.yaml` | SysID データ収集 (実機 / シム) |
| `dataflow_foctive_control.yaml` / `dataflow_mimic_web_control.yaml` ほか | 単軸テスト・ゼロ点調整 (状態機械バイパス) |
| `dataflow.yaml` / `dataflow_sim.yaml` | v2 時代のもの (レガシー) |

## モニタリング

いずれも dora ノードではない pull 型 (dataflow の起動に影響せず、いつでも起動/終了できる):

```bash
cd src/python && ROBOT_CONFIG=robot_config/mimic_v2_5.json python3 data_viewer.py   # ターミナル (robot 上)
python3 tools/gui/web_monitor.py    # ブラウザ http://<host>:8765/ (PC からは DORA_COORDINATOR_ADDR=<robot-ip>)
```

## dora コマンド

```bash
dora list                                  # 実行中の dataflow 確認
dora logs <dataflow-id> --node <node-id>   # ノードログ (1.0.0-rc1 から --node が必須)
dora stop <dataflow-id>
dora destroy                               # coordinator & daemon 終了
```

---

# ドキュメント

| ファイル | 内容 |
|---|---|
| [README_ARCH.md](README_ARCH.md) | ノードグラフ・構造体レイアウト・バイナリフォーマット |
| [CLAUDE.md](CLAUDE.md) | ビルド/実行の詳細、設計上の約束事 (コード生成、操作系の排他、など) |
| [docs/robot_start.md](docs/robot_start.md) | 実機起動チェックリスト |
| [docs/rt-enable.md](docs/rt-enable.md) | PREEMPT_RT カーネル導入・RT 権限・CPU 隔離 |
| [docs/rt_meas.md](docs/rt_meas.md) | 制御ループのレイテンシ/ジッタの実測方法と結果 |
| [docs/pcan_socketcan_driver.md](docs/pcan_socketcan_driver.md) | PEAK PCAN-M.2 ドライバ (カーネル更新時の再ビルド含む) |
| [docs/dora-upgrade.md](docs/dora-upgrade.md) | dora の更新手順・バージョン不一致の症状 |
| [HowToSocketcan.md](HowToSocketcan.md) | SocketCAN / vcan の基本 |
| [scripts/mjmodel_converter/README.md](scripts/mjmodel_converter/README.md) | Fusion CSV → MuJoCo モデル生成、倒立点補正 (com_comp) |
| [src/cpp/node/stabilizer/README_PIN_IK.md](src/cpp/node/stabilizer/README_PIN_IK.md) | 姿勢 IK の定式化と Pinocchio の使い方 |
| [scripts/dualsense_monitor/README.md](scripts/dualsense_monitor/README.md) | DualSense の接続とボタン/軸の実測マップ |
| [tools/gui/README.md](tools/gui/README.md) | ブラウザモニタ |
