# RT 性の計測方法と、これまでに分かったこと

3ms 周期の制御ループ (DCM → RCM → DCM) の遅延・ジッタを測る手段と、
2026-08-15〜16 に一通り試した結果の記録。次に同じ穴を掘らないためのメモ。

## 1. 計測手段 (どれも実機を動かさず STOP 状態で採れる)

### 1-1. RCM の受信計測ログ (`motor_status rx: ...`)

robot_control_manager が 1 秒ごとに stdout に出す。`dora logs <df> --node robot_control_manager`。

```
motor_status rx: 334/s  gap max G  proc avg P max Pm  wait avg W max Wm  transit avg T max Tm (us)
```

| 項目 | 定義 | 見方 |
|---|---|---|
| rx | 1 秒に受け取った motor_status の件数 | 333 前後なら欠けなし。大きく下回れば配送落ち |
| gap | 到着間隔 (t_recv(今) − t_recv(前)) の最大 | 理想 3000。≒ proc + wait |
| proc | `next()` が返してから motor_commands 送信完了まで | RCM 自身の処理。avg ~0.4ms、max のばらつきは非 RT で割り込まれた分 |
| wait | 前の処理完了から次のイベントを `next()` で手にするまで | 「経路の遅れ + RCM の起床遅れ」。avg ≈ 3000 − proc なら周期どおり |
| **transit** | 送信側 (DCM) の HLC タイムスタンプ → RCM が手にした時刻 | **経路そのものの遅延** (RCM の都合を含まない)。同一ホストなので CLOCK_REALTIME 共通 |

`rx` が全件でも `wait max` が数 ms になるのは矛盾ではない: 遅れた 1 件は次の 1 件と
まとめて届く (欠けない・到着が揺れる)。

### 1-2. DCM の latency topic

`device_control_manager/latency` (LatencyData): `can_avg/max` = 1 tick の CAN 送受信
所要、`ctrl_avg/max` = motor_status 送信 → motor_commands 受信の往復。
data_viewer.py の latency 行、または `dora topic echo -d <df> device_control_manager/latency --format json`。

### 1-3. スレッド別の scheduling / CPU

```bash
ps -L -o tid,cls,rtprio,psr,pcpu,comm -p $(pgrep -f device_control_manager/build | head -1)
```

DCM は「メインスレッドだけ FF 80、zenoh ワーカー (rx/tx/net/acc) は TS、全部 CPU1」が正。

### 1-4. zenoh の直接リンク確認

ノード間データは daemon を通らず zenoh 直接 (rc1)。RCM が DCM の listen ポートに
繋がっているか:

```bash
DP=$(pgrep -f device_control_manager/build | head -1); RP=$(pgrep -f robot_control_manager/build | head -1)
ss -tlnp | grep "pid=$DP,"                       # DCM の zenoh listen ポート
ss -tanp | grep "pid=$RP," | grep ESTAB | awk '{print $5}'   # RCM の接続先にそのポートがあれば直接
```

注: setcap 付きプロセスは `ss -p` に名前が出ず `/proc/<pid>/fd` も読めない (secure-exec)。
setcap をやめて limits.conf 方式にした理由の一つ。

### 1-5. CAN バス上の往復遅延

```bash
timeout 3 candump -td can0    # 送信 (0x80xx) → 応答 (0xXX00) の相対時刻差
```

### 1-6. OS 側の状態

```bash
grep . /sys/devices/system/cpu/cpu*/cpuidle/state1/{name,latency,disable,usage}  # C-state
cat /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor                       # DVFS
awk '$3+0>0' /proc/interrupts | sort -k3 -nr | head                            # 割り込み分布
ps -eLo pid,tid,cls,rtprio,psr,comm | awk '$3=="FF" && $4>=50'                   # 高優先度 RT
```

## 2. 試したことと結果 (全部 STOP 状態、同条件で比較)

| 変えたもの | wait/transit への効果 |
|---|---|
| daemon `--rt` ⇄ 非 RT (`dora up`) | なし (daemon はデータ経路にいない) |
| ZENOH_CONFIG (zenoh_robot.json5) 撤去 | ctrl_avg 半減 (1090→595us)、wait は不変。ローカル運用では付けない |
| メッセージ数 (imu_data 333Hz を停止) | なし |
| RCM を SCHED_FIFO 化 | proc は安定するが wait 不変 → 戻した |
| DCM の zenoh 直接リンク (中継→直接) | なし |
| pcan `fdirqcl/fdirqtl` = 1 (割り込み合体オフ) | CAN 遅延 1.3→0.5ms だが IRQ ~4000/s で DCM tick が揺れ**悪化**。既定 16/10 のまま |
| cpuidle c7 禁止 (CPU1 のみ / 全 CPU) | わずかに丸くなるが決定的でない (exit latency 5ms は一因止まり) |

**現状の値** (robot ローカル構成、DCM メインのみ FIFO、直接リンク):
transit **avg ~0.45ms / max 1.5〜4ms** (毎秒数回)、wait avg ≈ 周期どおり / max 4〜8ms、
proc avg 0.4ms / max 1〜3ms。

## 3. 結論と残課題

- 平常時の配送は健全 (0.45ms)。スパイク (max 数 ms) は 1〜2 tick 分で、PID バランス制御の
  段階では許容と判断。**PID を先に進める**
- スパイクの主因は未特定。dora 側 (優先度・トポロジ・件数) では消えない。OS 側は
  cpuidle と RT 優先度では消えず、残る候補は IPI (Function call 割り込み、RT より上) /
  arch_timer / softirq。**本気で詰めるなら ftrace (`trace-cmd record -e sched_switch
  -e irq`) でスパイクの瞬間に CPU を奪っているものを直接見る**のが最短。推測で設定を
  いじるのはもうやらない
- **「zenoh の処理が原因」か「OS 由来で任意のプロセス間受け渡しに乗るジッタ」かは未分離**。
  transit は「送信スレッドが timestamp を押す → 受信スレッドが手にする」の全部を含む。
  分離するには、同じ 2 プロセス間で zenoh を使わない最小の送受信 (shm+futex や
  Unix domain socket 直) を並走させて transit を比較する。shm 化の効果見積もりにもなる
- 根本対策は方針として決定済み: 閉ループ (CAN/IMU/PID) を dora メッセージから外し
  shm 直結にする (dora は低レート配線のみ)。着手は「transit avg が ms 級になる /
  遅延に敏感な制御に進む」等、必要になったとき

## 4. 関連

- docs/rt-enable.md — RT 権限 (limits.conf)、DCM の FIFO/affinity の付け方
- docs/pcan_socketcan_driver.md §6b — pcan の割り込み合体
- docs/robot_start.md — 起動手順・トラブルシュート
