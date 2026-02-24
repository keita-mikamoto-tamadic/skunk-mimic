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

# 2. dora CLI インストール
### cargo (Rust パッケージマネージャー)

```bash
curl https://sh.rustup.rs -sSf | sh
```

環境変数登録
```bash
source ~/.cargo/env
```

確認
```bash
which cargo
cargo --version
```

### dora-rs
```bash
cargo install dora-cli
```

確認
```bash
which dora
dora --version
```

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

# 4. C++ API の準備
API 用にソースをクローン（ホームディレクトリ直下を想定）

```bash
cd ~
git clone https://github.com/dora-rs/dora.git
```

C API をビルド
```bash
cd ~/dora
cargo build -p dora-node-api-cxx --release
```

確認 — .a ファイルの場所
```bash
cd ~/dora
ls -lah target/release | egrep 'dora.*api.*cxx|node_api|\.a$|\.so$' || true
```

確認 — ヘッダファイル (.h)
```bash
cd ~/dora
find . -maxdepth 6 -type f -name 'dora-node-api.h' -print
```

この 2 つを CMake でリンクする。

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

# 7. 実行

```bash
dora up                            # Coordinator & Daemon 起動
dora start src/dataflow.yaml       # Dataflow 実行
dora list                          # 実行中の Dataflow 確認
dora logs <dataflow-id> <node-id>  # ノードログ確認
dora stop <dataflow-id>            # Dataflow 停止
dora destroy                       # Coordinator & Daemon 終了
```
