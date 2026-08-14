# dora アップグレード手順 (v0.4.1 → main / v1.0.0-rc1)

## 背景・目的
- **realtime tuning 機能**(`dora daemon --rt`、ノード単位の `cpu_affinity`)を使うための更新。
- これらは v0.5.0 にも**未搭載**で、**main(= v1.0.0-rc1、未リリース)にのみ**存在する。
- そのため PyPI / 正式リリース版では入手できず、**C++・Rust・Python すべてを同一の main ソースから揃える**必要がある。
- バージョンが揃っていないと `dora-message`(シリアライズ形式)の不一致でノード↔daemon 通信が壊れる。
- ⚠️ **分散構成 (robot + PC) では全マシンを同一コミットに揃えること**。rc1 と rc4 の
  混在ですら register が通らない (下の「バージョン不一致の症状」参照)。`main` を各マシンが
  別々のタイミングで pull すると必ずズレるので、**コミットハッシュを決めて全マシンで
  checkout する**運用にする。

**現在の採用コミット (全マシン共通): `fd1f050b`** (1.0.0-rc1)

---

## 手順

各マシン (robot / PC) で同じ手順を実行する。

### 1. dora 本体を採用コミットに更新
```bash
cd ~/dora        # 無ければ git clone https://github.com/dora-rs/dora.git ~/dora
git fetch
git checkout fd1f050b    # ← 採用コミット。main を pull しない (マシン間でズレる)
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

**b. main ソースを指定**(ルート `src/python/pyproject.toml`)。パスはユーザー名を
含まない `/opt/dora` (各マシンで `sudo ln -s ~/dora /opt/dora` した symlink) を使う:
```toml
[tool.uv.sources]
dora-rs = { path = "/opt/dora/apis/python/node" }
```

**c. 宣言をルートに1本化**:`sysid_controller` / `data_recorder` の個別 `dora-rs` を削除。
共有 venv 運用なので、ルートが `dora-rs` を持てば各 member は `uv run` でimport可能。

**d. 同期**:
```bash
cd ~/skunk-mimic/src/python
uv sync       # maturin でソースから dora-rs をビルド&インストール
# dora のコミットを変えた後は再ビルドを強制:
uv sync --reinstall-package dora-rs
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
~/dora/target/release/dora --version              # 全マシンで一致していること
cd ~/dora && git log -1 --format="%h"             # 採用コミットと一致していること
cd ~/skunk-mimic/src/python && uv run python -c "import dora; print(dora.__version__)"
```
⚠️ PATH に古い dora (cargo install 版など) が残っていると `dora` コマンドが別物を指す。
daemon/CLI は `~/dora/target/release/dora` のフルパスで起動するのが確実。

---

## バージョン不一致の症状 (2026-08-14 実例: robot=rc1, PC=rc4)

分散構成でマシン間のコミットがズレると、**TCP は繋がるのに登録が完了しない**:

- PC 側 daemon: `waiting for coordinator: ... timeout waiting for register reply from coordinator` を繰り返す
- robot 側 `dora start`: `no matching daemon for machine id 'pc'`
- `ss -tnp | grep 6013` では ESTAB の接続が見える (= ネットワークは正常)
- `dora cluster status` に相手マシンの daemon が出てこない

原因は `dora-message` のワイヤフォーマット不一致で register 要求が解釈されないこと。
**対処: 全マシンを採用コミットに checkout → 手順 2〜5 を再実行**。

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