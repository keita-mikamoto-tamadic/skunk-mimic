# dora アップグレード手順 (v0.4.1 → main / v1.0.0-rc1)

## 背景・目的
- **realtime tuning 機能**(`dora daemon --rt`、ノード単位の `cpu_affinity`)を使うための更新。
- これらは v0.5.0 にも**未搭載**で、**main(= v1.0.0-rc1、未リリース)にのみ**存在する。
- そのため PyPI / 正式リリース版では入手できず、**C++・Rust・Python すべてを同一の main ソースから揃える**必要がある。
- バージョンが揃っていないと `dora-message`(シリアライズ形式)の不一致でノード↔daemon 通信が壊れる。

---

## 手順

### 1. dora 本体を main に更新
```bash
cd /home/tama/dora
git checkout main && git pull        # v0.4.1 → v1.0.0-rc1
```

### 2. CLI / daemon をビルド
```bash
cargo build --release -p dora-cli    # パッケージは dora-cli、バイナリは dora
```
- 生成物: `target/release/dora`
- これで `dora daemon --rt` / `--worker-threads` / YAML の `cpu_affinity` が使用可能になる。
- ⚠️ `cargo build --release`(ワークスペース全体)ではない。CLIパッケージ指定が正しい。

### 3. C++ node API を再ビルド(忘れやすい)
```bash
cargo build --release -p dora-node-api-cxx
```
- 独立クレート(`crate-type = ["staticlib"]`)なので **本体ビルドには含まれず別途必要**。
- 生成物: `dora-node-api.h` / `libdora_node_api_cxx.a`(install ディレクトリと `target/release/`)。
- 注: ゼロコピー allocate API(`allocate_data_sample` / `DataSample`)は **Rust / Python のみ**で、
  C++ バインディング(cxx ブリッジ)には未公開(最新 main でも同様, 2026-06 時点で確認)。C++ の送信は
  `send_output` 系のみで、SHM への 1 コピーが入る。小さい制御構造体では無視できるため対応不要。

### 4. skunk-mimic(C++側)をビルド
- 更新された `dora-node-api.h` / `libdora_node_api_cxx.a` を使って通常どおりビルド。

### 5. Python 側 dora-rs を main ソースに切替(uv ワークスペース)
`src/python/` 配下の pyproject.toml を編集する。

**a. ピン解除**:`dora-rs==0.4.1` → `dora-rs`(PyPI に 1.0.0-rc1 が無いため)

**b. main ソースを指定**(ルート `src/python/pyproject.toml`):
```toml
[tool.uv.sources]
dora-rs = { path = "/home/tama/dora/apis/python/node" }
```

**c. 宣言をルートに1本化**:`sysid_controller` / `data_recorder` の個別 `dora-rs` を削除。
共有 venv 運用なので、ルートが `dora-rs` を持てば各 member は `uv run` でimport可能。

**d. 同期**:
```bash
cd /home/tama/skunk-mimic/src/python
uv sync       # maturin で main から dora-rs 1.0.0rc1 をビルド&インストール
```

---

## 完了後の状態
| コンポーネント | バージョン |
|---|---|
| dora CLI / daemon | 1.0.0-rc1 (main) |
| C++ node API | 1.0.0-rc1 (main) |
| Python dora-rs | 1.0.0-rc1 (main・ソースビルド) |

---

## 確認コマンド
```bash
target/release/dora --version
cd /home/tama/skunk-mimic/src/python && uv run python -c "import dora; print(dora.__version__)"
```

---

## 正式版 (v1.0.0) リリース後にやること
1. dora を `git checkout v1.0.0` 等でタグに切替、手順2・3を再実行。
2. Python: `src/python/pyproject.toml` の `[tool.uv.sources]` を削除し、依存を `dora-rs==1.0.0`(PyPI版)に戻す → `uv sync`。
3. C++ は正式版のソース or 配布アーカイブ(`find_package(dora-node-api-cxx)` 対応)に切替可能。

---

## 関連メモ
- realtime tuning の詳細は dora リポジトリの `docs/realtime-tuning.md` 参照(CPUガバナー、THP無効化、`isolcpus`/`nohz_full`、`sched_rt_runtime_us` の注意など)。
- `--rt` の SCHED_FIFO はメインスレッドのみ。全プロセスRT化には `chrt` / `isolcpus` 併用。
- 本環境のカーネルは PREEMPT_RT 済み(`6.8.1-realtime`)。