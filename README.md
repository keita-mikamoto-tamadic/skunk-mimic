# skunk-mimic
Unified robot controller by Dora-rs.

> [!NOTE]
> **環境**
> * OS : Ubuntu 22.04 (jammy)
> * dora CLI
> * [Apache Arrow](https://arrow.apache.org/)

# 1. ビルド用依存関係インストール
```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config git \
  clang lld \
  curl ca-certificates \
  libssl-dev
```

# 2. Rust と dora のビルド

### Rust ツールチェーン

```bash
curl https://sh.rustup.rs -sSf | sh
source ~/.cargo/env
```

確認：
```bash
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
git checkout v1.0.0-rc.1   # 検証時の実コミット: f342aeb0 (v1.0.0-rc.1 + 360)
```

PyO3 に 3.11 を使わせる（uv 管理の 3.11 を指定。無ければ `uv python install 3.11`）：
```bash
export PYO3_PYTHON="$(uv python find 3.11)"
# 例: /home/<user>/.local/share/uv/python/cpython-3.11-linux-aarch64-gnu/bin/python3.11
```

本体build
```bash
cargo build --release
```
これでビルドが失敗した場合は、

```bash
cargo build -p dora-cli --release
```


CAPI build
```bash
cargo build --package dora-node-api-cxx --release
```

これで以下が生成されます：
- `~/dora/target/release/dora` — CLI 実行ファイル
- `~/dora/target/release/libdora_node_api_cxx.a` — C++ 静的ライブラリ
- `~/dora/target/cxxbridge/dora-node-api-cxx/install/` — C++ ヘッダーファイル

### dora メトリクス収集と RT 性について

dora daemon は 2 秒ごとに `sysinfo` で各プロセスの CPU/メモリ/ディスクを `/proc` 走査して
収集する。**v0.4.1 ではこれが daemon のメインループ上で同期実行**され、発火のたびに RT ノード
(`device_control_manager`, SCHED_FIFO) のループに周期的なレイテンシスパイクを起こしていた
(当時は `binaries/daemon/src/lib.rs` の `Event::MetricsInterval` ハンドラで
`collect_and_send_metrics()` の呼び出しをコメントアウトして回避)。

**1.0-rc 以降は upstream が改善済み**: 収集は `tokio::task::spawn_blocking` で別の blocking
スレッドに退避され、`try_lock` で多重起動も skip するようになった。重い `/proc` 走査が
メイン reactor から外れたため、SCHED_FIFO で pin した RT ノードが preempt でき、
**自前パッチは基本不要**。

> もし 1.0-rc でも RT スパイクが再計測で出る場合のみ、`Event::MetricsInterval` ハンドラの
> `self.spawn_metrics_collection();` を 1 行コメントアウトすれば収集を完全に止められる
> (`binaries/daemon/src/lib.rs`。`git checkout` で上書きされるので都度再適用)。

### 環境変数設定

dora CLI を使えるように alias を設定：
```bash
echo 'alias dora="$HOME/dora/target/release/dora"' >> ~/.bashrc
source ~/.bashrc
```

確認：
```bash
dora --version
# 出力例:
# dora-cli 1.0.0-rc1
```

### dora をバージョンアップする場合

既存の dora を新しいバージョンに更新する手順：

```bash
cd ~/dora
git fetch --tags
git checkout v1.0.0-rc.1     # 目的のバージョン/コミットを指定
cargo clean                  # 古いビルドキャッシュを削除（重要）
export PYO3_PYTHON="$(uv python find 3.11)"   # 1.0-rc は Python 3.11 必須
cargo build --release
cargo build --release -p dora-node-api-cxx    # C++ API を明示的にビルド
```

C++ ノードを再ビルド：
```bash
cd ~/skunk-mimic/src/cpp
rm -rf node/*/build  # 各ノードのビルドディレクトリを削除
mkdir -p build && cd build
cmake .. && make
```

Python 側 dora-rs も更新：このリポジトリは `src/python/pyproject.toml` の
`[tool.uv.sources]` で `dora-rs` を `~/dora/apis/python/node` のローカルパスに向けている
(rc コミットには対応する PyPI wheel が無いため)。`uv sync` するとそのソースから maturin で
ビルドされ、CLI と同じバージョンに揃う。初回は Rust 拡張をフルコンパイルするため数分〜十数分
かかる (止まって見えるが進行中。`uv sync -v` で確認可)。
```bash
cd ~/skunk-mimic/src/python
uv sync
```

確認：
```bash
dora --version                                          # CLI のバージョン
uv run python -c "import dora; print(dora.__version__)"  # Python パッケージのバージョン
```

> **重要**: `cargo clean` を実行しないと、古いバージョンのバイナリが残り、バージョンが更新されない場合があります。C++ API も明示的にビルドしないと、ヘッダーファイルが生成されません。

# 3. Apache Arrow
前提ツール
```bash
sudo apt update
sudo apt install -y ca-certificates lsb-release wget
```

(Ubuntu) apt リポジトリインストール
```bash
# Ubuntu 22.04 (jammy) の場合。24.04 の場合は jammy → noble に読み替える
wget https://packages.apache.org/artifactory/arrow/ubuntu/apache-arrow-apt-source-latest-jammy.deb
sudo apt install -y ./apache-arrow-apt-source-latest-jammy.deb
```

(rasbianOS) apt リポジトリインストール
```bash
wget https://packages.apache.org/artifactory/arrow/debian/apache-arrow-apt-source-latest-bookworm.deb
sudo apt install -y ./apache-arrow-apt-source-latest-bookworm.deb
sudo apt update
```

本体インストール
```bash
sudo apt update
sudo apt install -y libarrow-dev
```

本プロジェクトでは `arrow/api.h`, `arrow/c/bridge.h` を使用する。

確認
```bash
pkg-config --modversion arrow
```

# 4. Python 環境セットアップ (uv)

Python ノード用のパッケージマネージャーとして `uv` を使用しています。

### uv インストール
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

確認
```bash
uv --version
```

### Python 依存関係のインストール
```bash
cd src/python
uv sync
```

これで以下がインストールされます：
- `dora-rs==0.4.1` — dora Python API（CLI とバージョンを揃える）
- `pyarrow>=23.0.1` — Apache Arrow Python バインディング
- `rich>=14.3.3` — data_viewer 用ターミナル UI ライブラリ

> **Note**: `dora-rs` のバージョンは dora CLI (0.4.1) と揃える必要があります。バージョン不一致があると "message format version mismatch" エラーが発生します。

### Python ノードの実行
```bash
# data_viewer の例
uv run src/python/data_viewer/data_viewer.py

# または workspace 内で
cd src/python
uv run data_viewer/data_viewer.py
```

# 5. CMakeLists.txt でのリンク設定

各ノードの CMakeLists.txt で以下のように参照している：

```cmake
# ---- Dora node api (built from ~/dora) ----
set(DORA_ROOT "$ENV{HOME}/dora")
set(DORA_NODE_API_INC "${DORA_ROOT}/target/cxxbridge/dora-node-api-cxx/install")
set(DORA_NODE_API_LIB "${DORA_ROOT}/target/release/libdora_node_api_cxx.a")
```

# 6. ビルド

```bash
cd src/cpp && mkdir -p build && cd build && cmake .. && make
```

# 7. 通信モードの切り替え

robot_config JSON ファイルの `transport` フィールドで通信方式を切り替えられます。

### Dummy モード（ハードウェア不要）

`robot_config/mimic_v2.json` を編集：
```json
{
  "robot_name": "mimic_v2",
  "transport": "dummy",
  ...
}
```

Dummy モードでは：
- CAN ハードウェア不要でテスト可能
- `DummyComm` が moteus 互換のダミーデータを生成
- position, velocity, torque が自動的に変化（デバッグ用）

### SocketCAN モード（実機接続）

`robot_config/mimic_v2.json` を編集：
```json
{
  "robot_name": "mimic_v2",
  "transport": "socketcan",
  ...
}
```

SocketCAN モードでは：
- 実際の CAN バスデバイス（can0）経由で通信
- moteus モーターコントローラーと通信
- 事前に CAN バスのセットアップが必要（`bash can_setup.bash`）

# 8. 実行

### Dummy モードでの実行（推奨・初回テスト用）

```bash
# 1. dora デーモン起動
dora up

# 2. dataflow 起動
dora start dataflow.yaml

# 3. 別ターミナルで dummy_input を起動（state コマンド送信用）
cd src/python
uv run dummy_input/dummy_input.py

# - on       : サーボ ON
# - ready    : READY 状態へ遷移
# - run      : RUN 状態へ遷移（制御開始）
# - stop     : STOP 状態（ブレーキ）
# - off      : サーボ OFF
# - reset    : 初期位置リセット
# - q        : 終了

# 4. さらに別ターミナルで data_viewer を起動（モーター状態表示用）
cd src/python
uv run data_viewer/data_viewer.py

```

### SocketCAN モードでの実行（実機接続時）

```bash
# 0. CAN バスセットアップ（初回のみ）
bash can_setup.bash

# 1-4. Dummy モードと同じ手順
dora up
dora start dataflow.yaml
cd src/python && uv run dummy_input/dummy_input.py  # (別ターミナル)
cd src/python && uv run data_viewer/data_viewer.py  # (別ターミナル)
```

### その他のコマンド

```bash
dora list                          # 実行中の Dataflow 確認
dora logs <dataflow-id> <node-id>  # ノードログ確認
dora stop <dataflow-id>            # Dataflow 停止
dora destroy                       # Coordinator & Daemon 終了
```
