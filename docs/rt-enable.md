# RT 有効化手順 (limits.conf)

dora daemon (`--rt`) と device_control_manager (DCM) がリアルタイム動作
(`SCHED_FIFO` + `mlockall`) するための権限設定。**setcap は使わない**。

## 方式: `/etc/security/limits.d` で rtprio / memlock を与える

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

## なぜ setcap をやめたか

以前は `sudo setcap cap_sys_nice,cap_ipc_lock+ep <バイナリ>` で付与していたが:

1. **リビルドのたびに消える** (capability はファイルに紐づく)。今日 3 回忘れた
2. **capability 付き実行は secure-execution mode になる**: `/proc/<pid>/fd` が同一
   ユーザーでも読めない (デバッグしづらい)、環境変数の扱いが変わる、等
3. DCM に付けた状態では zenoh の直接ピアリンクが張れず motor_status が daemon 中継に
   落ちていた (直接の原因は init 前に全スレッドを FIFO にしていたことで、setcap
   自体ではないが、切り分けを難しくした)

## ノード側の RT の付け方 (DCM の実装)

dora は設計上 SCHED_FIFO をノードに配布しない (dora `docs/realtime-tuning.md`)。
ノードが自分で付ける。DCM (`src/cpp/node/device_control_manager/main.cpp`):

- **CPU 隔離は dataflow yaml の `cpu_affinity: [1]`** — dora が `pre_exec` で全スレッドに適用
- **SCHED_FIFO は `init_dora_node()` の後にメインスレッドだけ** (`SetRealtimePriority(80)`)。
  init 前に呼ぶと dora ランタイム (zenoh の rx/tx ワーカー) まで FIFO を継承し、
  同一 CPU でメインループと競合して起動時のピア接続が完了せず、motor_status が
  daemon 経由の中継 (+数 ms) に落ちる (実測)

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
