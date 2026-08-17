# RT 有効化手順

dora daemon (`--rt`) と device_control_manager (DCM) がリアルタイム動作
(`SCHED_FIFO` + `mlockall`) するための設定。3 層あり、下の 2 つは独立している:

1. **PREEMPT_RT カーネル** (§0) — 入れなくても FIFO 自体は取れる。ジッタの質が変わる
2. **rtprio / memlock の権限** (§1) — これが無いと FIFO を取れない。**setcap は使わない**
3. **ノード側の実装** (§3) — dora は FIFO を配布しないのでノードが自分で付ける

計測方法と、それで何が分かったかは docs/rt_meas.md。

## 0. PREEMPT_RT カーネル (JetPack 7.2.1 / L4T r39.2)

素の JetPack は `CONFIG_PREEMPT` (低レイテンシ desktop) であって `CONFIG_PREEMPT_RT`
ではない。`CONFIG_EXPERT` が無効なため `.config` に選択肢としても現れない。
PREEMPT_RT のメインライン統合は 6.12 からだが、L4T r39.2 のベースは 6.8 系なので
そちらの経路では入らない。**NVIDIA が別リポジトリで RT ビルドを配っている。**

正本: [Installing Real-Time Kernel — Jetson Linux Developer Guide r39.2](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/Kernel/RealTimeKernel.html)
(§0 / §0.1 / §0.2 と §4 の rtla はすべてこのページが出典。L4T のバージョンを上げたら
URL の `r39.2` を差し替えて読み直すこと)

```bash
# jetson/{common,som,ffmpeg} とは別のリポジトリを足す (既定では入っていない)
sudo sh -c 'echo "deb https://repo.download.nvidia.com/jetson/rt-kernel r39.2 main" \
  >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list'
sudo apt update

# Orin 系。Jetson Thor は nvgpu の代わりに nvidia-l4t-rt-kernel-openrm
sudo apt install nvidia-l4t-rt-kernel nvidia-l4t-rt-kernel-headers \
  nvidia-l4t-rt-kernel-oot-modules nvidia-l4t-display-rt-kernel \
  nvidia-l4t-rt-kernel-nvgpu
sudo reboot
```

確認 (3 つ揃って初めて RT):

```bash
uname -srv                  # → 6.8.12-1021-rt-tegra ... SMP PREEMPT_RT
cat /sys/kernel/realtime    # → 1
zcat /proc/config.gz | grep PREEMPT_RT   # → CONFIG_PREEMPT_RT=y
```

- `/boot/Image.real-time` が追加され、`extlinux.conf` に `LABEL real-time` が生える。
  **`LABEL primary` はそのまま残るので `DEFAULT primary` に戻して再起動すれば元の
  カーネル**。バックアップを自分で作る必要はない
- **limits.conf (§1) はカーネル非依存なので再設定不要**
- **pcan.ko の再ビルドが必須** (docs/pcan_socketcan_driver.md)。ヘッダは
  `nvidia-l4t-rt-kernel-headers` が `/lib/modules/$(uname -r)/build` に入れてくれる。
  **`make install` + `depmod` + 再起動までやること**: ビルドディレクトリから
  `insmod` しただけの状態だと (a) 再起動でモジュールが消える、(b) `/etc/modprobe.d/pcan.conf`
  の `fdirqcl/fdirqtl` が適用されない、(c) 内蔵 mttcan が先に probe されて **can0 を奪い、
  PEAK が can1..can4 にずれる**。正しくインストールして再起動すれば
  従来どおり PEAK=can0..can3 / mttcan=can4 に戻る (実機で確認済み)
- NVIDIA は RT カーネルを **Developer-Preview 品質**としている

以降 §0.1 / §0.2 はブートパラメータの調整。どちらも `extlinux.conf` の
**`LABEL real-time` 側だけ**を書き換え、`LABEL primary` は退避経路として無傷で残す。

## 0.1 コア隔離 (isolcpus)

目的: DCM が載る **CPU1 から、DCM 以外を全部追い出す**。

現状 (隔離なし) の CPU1 の住人 — `ps -eLo tid,cls,rtprio,psr,comm | awk '$4==1'`:

| 種別 | 例 | 隔離で消えるか |
|---|---|---|
| NVMe の IRQ スレッド (FF 50) | `irq/186-s-nvme0q0` `irq/191-nvme0q5` | **消える** (`irqaffinity`) |
| kworker / kthread | `kworker/1:1H` `ksmd` `oom_reaper` | **消える** (`kthread_cpus`) |
| 一般タスク (TS) | RCM / imu_node など affinity 無しのノード | **消える** (`isolcpus`) |
| per-CPU kthread | `ksoftirqd/1` `ktimers/1` `rcuc/1` `migration/1` | **残る** (常駐、外せない) |
| DCM (FF 80) | `device_control_manager` | **残る** (これが目的) |

### 各オプションの意味

| パラメータ | 何をするか | このカーネルで効くか |
|---|---|---|
| `isolcpus=managed_irq,domain,1` | CPU1 をスケジューラのロードバランス対象から外す。**明示的に affinity を指定したタスクだけ**が載る | **効く** (`CONFIG_CPU_ISOLATION=y`) |
| ├ `domain` | スケジューリングドメインから除外 = 自動でタスクが振られなくなる本体 | 〃 |
| └ `managed_irq` | カーネル管理の IRQ (NVMe 等) の割り当て先からも除外 | 〃 |
| `irqaffinity=0,2-5` | **全 IRQ の既定の割り当て先**を CPU1 以外に限定 | **効く** |
| `kthread_cpus=0,2-5` | kworker などカーネルスレッドの配置先を CPU1 以外に限定 | **効く** |
| `nohz_full=1` | CPU1 のスケジューラ tick を止める (実行タスクが 1 個のとき) | **効かない** ✗ |
| `rcu_nocbs=1` / `rcu_nocb_poll` | RCU のコールバック処理を CPU1 から他コアへ追い出す | **効かない** ✗ |

「効かない」の根拠 — `zcat /proc/config.gz | grep -E "CPU_ISOLATION|NO_HZ_FULL|RCU_EXPERT"`:

```
CONFIG_CPU_ISOLATION=y          → isolcpus / irqaffinity / kthread_cpus は有効
# CONFIG_NO_HZ_FULL is not set  → nohz_full は無視される
# CONFIG_RCU_EXPERT is not set  → rcu_nocbs / rcu_nocb_poll も無視される
```

- [NVIDIA のドキュメント](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/Kernel/RealTimeKernel.html)
  の推奨例には `nohz_full` / `rcu_nocbs` も含まれるが、**同ページが前提としている
  `CONFIG_NO_HZ_FULL=y` / `CONFIG_RCU_NOCB_CPU=y` を配布 RT カーネルは満たしていない**
- **効かないパラメータを書いてもエラーは出ず、黙って無視される。** 効いたつもりに
  ならないよう `/proc/cmdline` と実挙動で確認すること
- 効かせたい場合はカーネル再ビルドが要る (§参照 の Kernel Customization)

### 適用 (6 コア機で CPU1 を隔離)

```bash
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak
sudo sed -i '/^LABEL real-time/,$ s|\(nvme.use_threaded_interrupts=1\)|\1 isolcpus=managed_irq,domain,1 irqaffinity=0,2-5 kthread_cpus=0,2-5|' \
  /boot/extlinux/extlinux.conf
grep -n APPEND /boot/extlinux/extlinux.conf
sudo reboot
```

sed の読み方:

- `/^LABEL real-time/,$` — **`LABEL real-time` から行末までの範囲だけ**に適用。
  `LABEL primary` には同じ文字列があるが手を付けない (退避経路を無傷で残すため)
- `s|\(既存の末尾パラメータ\)|\1 追加分|` — APPEND 行の末尾に追記。区切りが `|` なのは
  パラメータに `/` が含まれても壊れないようにするため
- `grep -n APPEND` — **primary 側が変わっていないこと**を目視確認する。ここを飛ばさない

### 確認 (再起動後)

```bash
cat /proc/cmdline | tr ' ' '\n' | grep -E "isolcpu|irqaffinity|kthread"
```
→ 3 つとも出れば適用済み。出ないパラメータは無視されたか、sed が当たっていない

```bash
ps -eLo tid,cls,rtprio,psr,comm | awk '$4==1'
```
→ `psr` (実行中の CPU) が 1 のスレッド一覧。**DCM と per-CPU kthread だけ**になっていれば成功

```bash
sudo rtla timerlat top -c 1 -p 3000 -d 60s -P o:0
```
→ 非 RT スレッドの max が隔離前 (2071us) からどこまで下がるか。効果の定量評価

### 注意

- **DCM は隔離コアでそのまま動く。** `isolcpus` が排除するのは「自動で振られるタスク」で、
  yaml の `cpu_affinity: [1]` による明示指定は対象外。逆に affinity 無しの
  RCM / imu_node は CPU1 から締め出される (望ましい方向)
- **per-CPU kthread は消せない。** `ksoftirqd/1` `ktimers/1` `rcuc/1` `migration/1` は
  各 CPU に必ず 1 本ずつ存在する

### CONFIG_HZ=250 という上限

```bash
zcat /proc/config.gz | grep -E "^CONFIG_HZ"
# CONFIG_HZ_250=y
# CONFIG_HZ=250          → スケジューラ tick = 1/250 = 4ms
```

- `CONFIG_HZ` はスケジューラ tick の周波数 [Hz]。tick 間隔 = `1/CONFIG_HZ`
- RT 用途では 1000 (1ms) が一般的だが、**配布 RT カーネルは 250 (4ms)**。
  変えるにはカーネル再ビルドが要る (§参照 の Kernel Customization)
- CFS の preempt 判定は tick 境界に依存する部分があるため、**SCHED_OTHER スレッドが
  待たされる粒度が 4ms オーダーになる**
- docs/rt_meas.md §2b の実測 (非 RT max 2.1〜2.3ms) はちょうど 1 tick 未満に収まる
- → **隔離は「同居タスクを減らして当たる確率を下げる」効果にとどまる。**
  非 RT である限りこの粒度は原理的に残るので、閉ループから非 RT スレッドを無くす方針
  (rt_meas.md §3) の代わりにはならない

### sysctl には手を出さない (現時点)

| sysctl | NVIDIA の推奨 | このリポジトリの判断 |
|---|---|---|
| `kernel.sched_rt_runtime_us` | `-1` (無効化) | **採用しない。** RT タスクが CPU の 95% までしか使えない安全弁で、外すと暴走時に機体が固まる |
| `kernel.timer_migration` | `0` | 保留。副作用が小さいので隔離の効果を測った後に試す候補 (既定 1) |

## 0.2 efi=runtime を外す

UEFI runtime services がレイテンシを増やすため、NVIDIA が RT では非推奨としている。

```bash
sudo sed -i '/^LABEL real-time/,$ s/ efi=runtime//' /boot/extlinux/extlinux.conf
cat /proc/cmdline | tr ' ' '\n' | grep efi    # 再起動後、消えていること
mount | grep efivar                            # efivarfs も消える
```

- 副作用として **efivarfs が消える**ので、EFI 変数を読む機能 (nvbootctrl / capsule 更新)
  を使うときは `DEFAULT primary` で起動し直す
- ブートメニューは `console=ttyTCU0,115200` = **シリアルコンソール**に出る (TIMEOUT 30s)。
  起動しなくなった場合にメニューから `primary` を選ぶにはシリアルが要る。
  繋がっていなければ、ストレージを別マシンでマウントして `DEFAULT` を戻すことになる

## 1. 権限: `/etc/security/limits.d` で rtprio / memlock を与える

```bash
sudo tee /etc/security/limits.d/50-skunk-rt.conf >/dev/null <<'EOF2'
jetmimic  -  rtprio   90
jetmimic  -  memlock  unlimited
EOF2
```

一度ログアウト → 再ログインで有効 (limits はログイン時に適用)。確認:

```bash
ulimit -r    # → 90
ulimit -l    # → unlimited
```

- これで一般ユーザーのまま `sched_setscheduler(SCHED_FIFO)` と `mlockall` が通る。
  **バイナリを何度リビルドしても再設定不要**。
- `dora_rt_damon.bash` は limits が効いた新しいログインシェルから起動すること
  (daemon → spawn されるノードへ継承される)。
- pipewire が同じ方式 (`/etc/security/limits.d/25-pw-rlimits.conf`) で rtprio を
  持っているのと同じ流儀。

## 2. setcap (§1 の代替。このリポジトリでは使わない)

capability をバイナリに直接付ける方式。**§1 の limits.conf があれば不要**。

### 手動で付ける

```bash
sudo setcap cap_sys_nice,cap_ipc_lock+ep src/cpp/node/device_control_manager/build/device_control_manager
getcap  src/cpp/node/device_control_manager/build/device_control_manager   # 確認
sudo setcap -r src/cpp/node/device_control_manager/build/device_control_manager   # 外す
```

- `cap_sys_nice` = `sched_setscheduler(SCHED_FIFO)` 用、`cap_ipc_lock` = `mlockall` 用
- `+ep` = effective + permitted
- **リビルドのたびに消える** (capability はファイルに紐づくため)

### ビルド時に自動で付ける

上を忘れないようにするなら、ノードの CMakeLists に POST_BUILD を足す:

```cmake
add_custom_command(TARGET device_control_manager POST_BUILD
  COMMAND sudo setcap cap_sys_nice,cap_ipc_lock+ep $<TARGET_FILE:device_control_manager>
  COMMENT "setcap for RT")
```

- ビルドのたびに `sudo` を要求する (パスワードなしで通すには sudoers 設定が要る)
- CI やクリーンビルドが sudo なしの環境で失敗する

### 使わない理由

| | |
|---|---|
| 消える | リビルドのたびに再実行が必要 (自動化しても sudo 依存が残る) |
| デバッグしづらい | capability 付き実行は **secure-execution mode** になり、`/proc/<pid>/fd` が同一ユーザーでも読めず、環境変数の扱いも変わる |
| 切り分けを濁した | DCM に付けた状態で zenoh の直接ピアリンクが張れず motor_status が daemon 中継に落ちた。真因は「init 前に全スレッドを FIFO にしていた」(§3) だが、setcap が原因調査を難しくした |

現状の正は `getcap` で**何も出ないこと**:

```bash
getcap src/cpp/node/*/build/*     # 出力が空なら OK
```

## 3. ノード側の RT の付け方 (DCM の実装)

dora は設計上 SCHED_FIFO をノードに配布しない (dora `docs/realtime-tuning.md`)。
ノードが自分で付ける。DCM (`src/cpp/node/device_control_manager/main.cpp`):

- **CPU 隔離は dataflow yaml の `cpu_affinity: [1]`** — dora が `pre_exec` で全スレッドに適用
- **SCHED_FIFO は `init_dora_node()` の後にメインスレッドだけ** (`SetRealtimePriority(80)`)。
  init 前に呼ぶと dora ランタイム (zenoh の rx/tx ワーカー) まで FIFO を継承し、
  同一 CPU でメインループと競合して起動時のピア接続が完了せず、motor_status が
  daemon 経由の中継 (+数 ms) に落ちる (実測)

## 4. rtla (レイテンシ計測ツール)

「タイマ満了 → スレッドが実際に走る」までの遅れを直接測る。これで OS 由来のジッタと
アプリ/ミドルウェア由来を分離できる (使い方と結果は docs/rt_meas.md §2b)。

RT カーネルは `CONFIG_OSNOISE_TRACER=y` / `CONFIG_TIMERLAT_TRACER=y` なので
`rtla timerlat` / `rtla osnoise` は使える。`CONFIG_HWLAT_TRACER` は無効なので
`rtla hwnoise` だけ使えない。

**罠: `/usr/bin/rtla` はラッパースクリプト** (1.6KB) で、中身は

```sh
full_version=`uname -r`
this="/usr/lib/linux-tools/$full_version/`basename $0`"
```

つまり `/usr/lib/linux-tools/6.8.12-1021-rt-tegra/rtla` を探すが、**NVIDIA カーネル用の
`linux-tools` パッケージは存在しない** (`apt-cache policy linux-tools-$(uname -r)` が空)。
実体は Ubuntu 汎用カーネル用のパッケージにしか無いので、symlink で繋ぐ:

```bash
sudo apt install linux-tools-common linux-tools-generic
sudo ln -s /usr/lib/linux-tools/6.8.0-137-generic /usr/lib/linux-tools/$(uname -r)
rtla --help
```

`6.8.0-137-generic` は `apt-cache depends linux-tools-generic` で確認する (更新で変わる)。
rtla は tracefs 経由でカーネルと話すだけなので、同じ 6.8 系ならマイナー差は問題ない。
実行には `sudo` が要る (`/sys/kernel/tracing` が root 専用)。

```bash
# 3ms 周期 = 制御ループと同条件。-c でコア指定、-P o:0 で SCHED_OTHER として測る
sudo rtla timerlat top -c 1 -p 3000 -d 60s          # RT スレッド
sudo rtla timerlat top -c 1 -p 3000 -d 60s -P o:0   # 非 RT スレッド
```

IRQ latency = タイマ割り込み → ハンドラ、Thread latency = タイマ満了 → ユーザ空間の
スレッドが走り出すまで。**両者を分けて見るのが肝**で、IRQ が小さいまま Thread だけ
跳ねていれば原因は割り込みや C-state ではなくスケジューラにある。

## 確認

```bash
# DCM: メインスレッドだけ FF 80、他は TS、全部 CPU1
ps -L -o tid,cls,rtprio,psr,comm -p $(pgrep -f device_control_manager/build | head -1)
# daemon --rt: RT: mlockall enabled / sched_setscheduler の失敗ログが出ないこと
```

失敗時のログ:
- daemon: `RT: sched_setscheduler failed: Operation not permitted` → rtprio 未設定 or 古いシェル
- daemon: `memory allocation of N bytes failed` で abort → memlock 未設定 (mlockall 後の確保失敗)
- DCM: `Warning: Failed to set RT priority (check limits.conf rtprio)`

## sysctl (参考)

`kernel.sched_rt_runtime_us = 950000` (既定) のままでよい。RT タスクが CPU の 95% までしか
使えない安全弁で、DCM の 3ms tick は数 % なので影響しない。
NVIDIA は RT では `-1` (無効化) を勧めているが、暴走した RT タスクで機体が固まるため
採用していない (§0.1 末尾)。

## 参照

- [Installing Real-Time Kernel — Jetson Linux Developer Guide r39.2](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/Kernel/RealTimeKernel.html)
  — RT カーネルの apt リポジトリとパッケージ名、`extlinux.conf` でのカーネル切り替え、
  隔離用ブートパラメータ、`efi=runtime` の除去、`rtla timerlat` の推奨。**§0 系の出典**
- [Kernel Customization — Jetson Linux Developer Guide r39.2](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/Kernel/KernelCustomization.html)
  — `CONFIG_NO_HZ_FULL` / `CONFIG_RCU_NOCB_CPU` を有効にするなど、配布バイナリで
  足りない場合のカーネル再ビルド手順
- dora `docs/realtime-tuning.md` — dora が SCHED_FIFO をノードに配布しない設計理由 (§3)
- docs/rt_meas.md — 計測方法と、RT/非RT で何が変わるかの実測 (§2b)
- docs/pcan_socketcan_driver.md — カーネル入れ替え後の `pcan.ko` 再ビルド
