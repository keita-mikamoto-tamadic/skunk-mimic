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
所要、`ctrl_avg/max` = motor_status 送信 → motor_commands 受信の往復、
`send_avg/max` = motor_status の `send_output` 所要 (呼び出し→戻り = zenoh put の同期
部分、1 秒窓)。data_viewer.py の latency 行 (`SEND avg/max`)、または
`dora topic echo -d <df> device_control_manager/latency --format json`。

RCM の transit と同じ時間帯で並べれば、経路遅延のスパイクが**送信側で起きているか
受信側/経路で起きているか**を切り分けられる (transit max が大きい秒に send max も
大きければ送信、send max が小さいままなら受信側)。

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

**送信側と受信側の分解** (2026-08-16、send_avg/max を追加して同時計測):

| | avg | max |
|---|---|---|
| DCM `send_output` (zenoh put 同期部分) | 162〜171us | **215〜255us (スパイクなし)** |
| RCM transit (送信 timestamp → 受信) | 440〜565us | 1.5〜4.6ms |

→ 平常時の内訳は送信 ~165us + 受信側 ~285us。**スパイク (1.5〜4.6ms) は送信後、
すなわち zenoh の受信側 (RCM プロセス内の rx ワーカー → Arrow デコード → mpsc →
アプリスレッド起床) で発生している。送信側 (DCM) は無罪。**

wait max − transit max の差 (2〜4ms) は RCM 側で失う時間 = 非 RT のアプリスレッドの
起床遅れ + 直前 tick の proc が伸びた分。RCM を RT にすると proc は安定するが
transit (zenoh rx 経路) は RT の外なので縮まない (実測どおり)。

## 2b. RT カーネル + rtla timerlat による決着 (2026-08-17)

JetPack 7.2.1 / L4T r39.2 の RT カーネル (`nvidia-l4t-rt-kernel`、`6.8.12-1021-rt-tegra`、
`CONFIG_PREEMPT_RT=y`) に移行し、`rtla timerlat` で「タイマ満了 → スレッドが実際に走る」
までの遅れを直接測った。これで §3 に残っていた「zenoh の処理が重いのか、OS に
待たされているのか」が分離できた。

導入は docs/rt-enable.md を参照 (RT カーネルの apt リポジトリ、rtla のラッパー問題)。

```bash
# 3ms 周期 = 制御ループと同条件。-c でコア、-P o:0 で SCHED_OTHER
sudo rtla timerlat top -c 1 -p 3000 -d 60s          # RT スレッド
sudo rtla timerlat top -c 1 -p 3000 -d 60s -P o:0   # 非 RT スレッド (RCM と同条件)
```

daemon 起動のみ (dataflow なし)、CPU1 = DCM (FIFO 80) 同居、CPU4 = 空きコア:

| コア | クラス | IRQ max | Thread avg | **Thread max** |
|---|---|---|---|---|
| 1 | RT | 25us | 12us | **35us** |
| 1 | 非RT | 24us | 59us | **2071us** |
| 4 | RT | 22us | 12us | **40us** |
| 4 | 非RT | 15us | 22us | **2283us** |

読み取れること:

- **決定変数はスケジューリングクラスであって、コアではない。** RT はどのコアでも
  35〜40us、非 RT はどのコアでも 2.1〜2.3ms。約 55 倍差。**`cpu_affinity` を足すだけでは
  テールは縮まない** (CPU4 の方が avg は良いが max はむしろ悪い)
- 非 RT の max 2.1〜2.3ms は、実測の transit スパイク (1.5〜4.6ms) と同オーダー。
  **スパイクの正体は「zenoh の受信処理が重い」ではなく「非 RT スレッドの起床遅れ」**
- **IRQ latency は 4 条件すべて 15〜25us。** 深い C-state からの復帰が効いているなら
  タイマ割り込み自体が遅れて IRQ latency に出るはずで、出ていない。
  → **cpuidle は一因ですらない** (§2 の「exit latency 5ms は一因止まり」を撤回)。
  2ms は全部「IRQ ハンドラは時刻どおり走った後、スレッドがスケジュールされるまで」の区間
- 同じ理由で、§3 で候補に挙げていた IPI / arch_timer / softirq も否定される。
  それらが原因なら RT スレッドの Thread max にも出る (35us に収まっている)

**帰結: 3ms 周期の閉ループに非 RT スレッドが 1 つでも挟まれば、ms 級スパイクは
構造的に避けられない。** 60 倍差はチューニングで埋まる幅ではない。RCM のアプリ
スレッドも手前の zenoh rx ワーカーも SCHED_OTHER なので、一番遅いところが律速する
(§2 の「RCM を FIFO 化しても wait 不変」は、rx ワーカーが TS のまま残っていたためと
説明が付く)。

## 3. 結論と残課題

- 平常時の配送は健全 (0.45ms)。スパイク (max 数 ms) は 1〜2 tick 分で、PID バランス制御の
  段階では許容と判断。**PID を先に進める**
- スパイクの発生箇所は **zenoh の受信側 (RCM プロセス内)**、原因は **非 RT スレッドの
  起床遅れ** と確定 (§2b)。送信側・cpuidle・IPI/softirq・コア配置はすべて否定された。
  ftrace で追う必要はもう無い
- 「経路上の全スレッドを RT 化する」のは dora 内では分が悪い。zenoh の rx ワーカーは
  dora が内部で立てるため、ノード側から優先度を触る手段が素直に無い。加えて
  「init 前 FIFO で起動時ピア接続失敗」(DCM で踏んだ) が RCM でも再現しうる
- 根本対策は方針として決定済み: 閉ループ (CAN/IMU/PID) を dora メッセージから外し
  shm 直結にする (dora は低レート配線のみ)。§2b はこの判断を裏付けている
- **shm 化するときの必須条件**: §2b の 2ms は「非 RT スレッドの起床遅れ」であって
  zenoh 固有ではない。**shm にしても読み書きするスレッドが SCHED_OTHER なら同じ 2ms を
  食う。** 経路上の全スレッドを FIFO にすることが shm 化とセットで必要 (逆に言えば、
  dora の内部スレッドを避けて自前スレッドだけにするからこそ、それが可能になる)

## 4. 関連

- docs/rt-enable.md — RT 権限 (limits.conf)、DCM の FIFO/affinity の付け方
- docs/pcan_socketcan_driver.md §6b — pcan の割り込み合体
- docs/robot_start.md — 起動手順・トラブルシュート
