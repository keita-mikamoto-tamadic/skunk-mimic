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

~/dora をクローンしてビルド（CLI + C++ API の両方が生成される）：

```bash
cd ~
git clone https://github.com/dora-rs/dora.git
cd dora
git checkout v0.4.1
cargo build --release
```

これで以下が生成されます：
- `~/dora/target/release/dora` — CLI 実行ファイル
- `~/dora/target/release/libdora_node_api_cxx.a` — C++ 静的ライブラリ
- `~/dora/target/cxxbridge/dora-node-api-cxx/install/` — C++ ヘッダーファイル

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
# dora-cli 0.4.1
# dora-message: 0.7.0
# dora-rs (Python): not found
```

### dora をバージョンアップする場合

既存の dora を新しいバージョンに更新する手順：

```bash
cd ~/dora
git fetch --tags
git checkout v0.4.2  # 新しいバージョンを指定
cargo clean          # 古いビルドキャッシュを削除（重要）
cargo build --release
cargo build --release -p dora-node-api-cxx  # C++ API を明示的にビルド
```

C++ ノードを再ビルド：
```bash
cd ~/ws/dora-test/src/cpp
rm -rf node/*/build  # 各ノードのビルドディレクトリを削除
mkdir -p build && cd build
cmake .. && make
```

Python パッケージも更新：
```bash
cd ~/ws/dora-test/src/python
uv add dora-rs==0.4.2  # dora CLI と同じバージョンに揃える
```

確認：
```bash
dora --version                    # CLI のバージョン確認
uv run python -c "import dora; print(dora.__version__)"  # Python パッケージのバージョン確認
```

> **重要**: `cargo clean` を実行しないと、古いバージョンのバイナリが残り、バージョンが更新されない場合があります。C++ API も明示的にビルドしないと、ヘッダーファイルが生成されません。

# 3. Apache Arrow
前提ツール
```bash
sudo apt update
sudo apt install -y ca-certificates lsb-release wget
```

apt リポジトリインストール
```bash
# Ubuntu 22.04 (jammy) の場合。24.04 の場合は jammy → noble に読み替える
wget https://packages.apache.org/artifactory/arrow/ubuntu/apache-arrow-apt-source-latest-jammy.deb
sudo apt install -y ./apache-arrow-apt-source-latest-jammy.deb
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
