# mimic_v2_5 起動手順 (dataflow_mimic.yaml / 実機運転)

robot = Jetson (192.168.1.9, coordinator + RT daemon + C++ ノード群 + robot_web_gui)、
PC = ブラウザで操作・表示するだけ (PC 側に dora は不要)。
※ 以前は robot_web_gui を PC daemon に分散配置していたが、zenoh 経由の
  state_command が数秒詰まって SERVO_OFF が届かない等の問題が多く、robot ローカルに戻した。

単軸テスト・ゼロ点直接操作 (state machine バイパス) は
`dataflow_mimic_web_control.yaml` を使う — 手順はそのファイル冒頭コメント参照。

---

## 0. 前提 (初回 / リビルド後のみ)

- (dataflow_mimic.yaml は robot ローカル完結なので PC 側の dora は不要。分散配置の
  dataflow_*_web_control.yaml を使う場合のみ) dora のバージョン一致: robot と PC の
  `~/dora` は同一コミット (`fd1f050b`)。ズレると register されない — docs/dora-upgrade.md
- **RT 権限は limits.conf で与える** (setcap は使わない。初回のみ、再ログインで有効):

  ```bash
  sudo tee /etc/security/limits.d/50-skunk-rt.conf >/dev/null <<'EOF'
  jetmimic  -  rtprio   90
  jetmimic  -  memlock  unlimited
  EOF
  # 再ログイン後に確認: ulimit -r → 90、ulimit -l → unlimited
  ```

  これで device_control_manager (SCHED_FIFO 80、メインスレッドのみ) も dora daemon
  (`--rt`) も setcap なしで RT を取れる。**リビルドしても何もしなくてよい**。
  ノードに setcap すると secure-exec になり不利 (以前 DCM に付けていた
  `cap_sys_nice,cap_ipc_lock` は外した — `getcap` で何も出ないのが正)。
  `dora_rt_damon.bash` は limits が効いた新しいログインシェルから起動すること
  (spawn されるノードが継承する)。
- **DCM の CPU 隔離は dataflow yaml の `cpu_affinity: [1]`** (dora が起動時に全スレッドへ
  適用)。SCHED_FIFO は DCM が `init_dora_node()` の後にメインスレッドだけに付ける
  (init 前だと zenoh ワーカーまで FIFO になり、起動時のピア接続が失敗して
  motor_status が daemon 中継に落ちる)

- pcan.ko の `fdirqcl`/`fdirqtl` は**既定 (16/10) のまま**にする (1 にすると割り込み負荷で
  DCM の tick が揺れて補間がカクつく — docs/pcan_socketcan_driver.md §6b)。確認:
  `cat /sys/module/pcan/parameters/fdirqcl` が 16
- モータ電源 ON (モータが CAN に応答しないと ACK 不在でバスが ERROR-PASSIVE に落ちる)
- IMU (Spresense) が /dev/ttyUSB0 に接続されていること (imu_node が使用)
- **シリアルの権限**: imu_node を動かすユーザーが `dialout` グループに属していること
  (`sudo usermod -aG dialout <user>` → 再ログインで有効)。無いと imu_node が
  `Permission denied` で IMU なし続行になり **imu_data が全ゼロ**になる
- IMU の取り付け回転は robot_config の `imu_mount_rpy_deg` (v2_5 は [180,0,0] =
  X 軸周り 180°、実機確認済み)。取り付きを変えたら json のこの 1 行を更新
  (dataflow 再起動で反映、リビルド不要)

## 1. robot 側

```bash
cd ~/skunk-mimic

# 1) CAN UP (can0=右脚, can1=左脚)。keep = UP したまま
bash pcan_setup.bash can0 can1 keep

# 1b) IMU シリアルを掴みっぱなしにする (別ターミナル、セッション中ずっと)。
#     Spresense は一度 close されると次の open で沈黙し、USB 抜き差しでしか
#     復帰しない。imu_node の起動/停止をまたいで fd を保持するために必須。
#     dataflow より先に、電源投入 (USB 接続) 後の最初の open がこれになるように。
bash keep_serial_open.bash

# 2) coordinator + RT daemon (前セッションの孤児ノードも掃除される)。
#    起動時に UP 済み CAN の状態も表示 — "!!! CAN canX: ERROR-PASSIVE" が出たら
#    pcan_setup.bash canX down → 上げ直してから進む
bash dora_rt_damon.bash          # 引数なし = ローカル運用 (分散 yaml のときだけ `distributed` を付ける)

# 3) dataflow 起動
dora start dataflow_mimic.yaml
#    attach モード (既定) のまま使う: そのターミナルの Ctrl-C で dataflow が確実に
#    止まる。--detach はターミナルを閉じても走り続けるので使わない

# 4) 操作 GUI (別ターミナル)。dynamic node が attach するまで dataflow は開始バリアで待つ
cd src/python
ROBOT_CONFIG=robot_config/mimic_v2_5.json uv run robot_web_gui/robot_web_gui.py

# 5) 生データ表示 (別ターミナル、任意。dataflow 非依存の pull 型なので手順に影響しない)
cd src/python
ROBOT_CONFIG=robot_config/mimic_v2_5.json python3 data_viewer.py
```

## 2. PC 側

ブラウザを開くだけ:

- 操作: **http://192.168.1.9:8766/** (robot_web_gui)
- 表示 (任意): web_monitor を PC で動かすなら
  `DORA_COORDINATOR_ADDR=192.168.1.9 ROBOT_CONFIG=robot_config/mimic_v2_5.json python3 tools/gui/web_monitor.py`
  → **http://localhost:8765/**

## 3. ブラウザでの運転手順

1. **基準姿勢セット** (電源投入毎に必要 — エンコーダリセットは揮発性):
   脚をストッパーに押し当てた姿勢にする → 状態が OFF であることを確認 →
   「基準姿勢セット」→ web_monitor で position が reset_position
   (hip -0.1309 / knee -2.3736 / wheel 0.0 rad) になることを確認
2. **サーボON** (OFF → STOP: 現在位置保持でサーボが入る)
3. **READY** (STOP → READY: interpolation_time [3s] かけて initial_position へ補間移動。
   進捗バーが完了するまで RUN は無効)
4. **RUN** (補間完了後に有効化。受理されるまで自動リトライ。stabilizer の
   angle_pid が車輪速度を制御し始める — 転倒に備えること)
5. **STOP** で現在位置保持に戻る / **サーボOFF** は常時有効 (緊急停止)

状態表示は RCM の state_status 由来。fault・トルクリミット 100 tick 連続・
motor_status 途絶で自動的に OFF へ落ちることがある。

## 4. 終了

```bash
# robot 側
dora stop <dataflow-id>        # DCM が停止時に全軸 OFF を送る
bash pcan_setup.bash can0 can1 down
# dora_rt_damon.bash のターミナルを Ctrl-C (coordinator + daemon 終了)
```

PC 側はブラウザを閉じるだけ (web_monitor を動かしていればそのターミナルを Ctrl-C)。

## 5. トラブルシュート (実際に踏んだもの)

| 症状 | 原因と対処 |
|---|---|
| `no matching daemon for machine id 'pc'` | (分散配置していた頃の症状。dataflow_mimic.yaml は robot ローカルに戻したので出ないはず。dataflow_*_web_control.yaml で出たら) PC daemon 未接続。`dora_pc_daemon.bash` を再実行 |
| PC daemon が `timeout waiting for register reply` を繰り返す | robot 側 coordinator が落ちている、または dora のコミット不一致 (docs/dora-upgrade.md) |
| daemon が `memory allocation of N bytes failed` で abort | RT 権限なしで `--rt` の mlockall が失敗 (limits.conf の memlock、§0。または旧運用の dora バイナリ setcap 消失) |
| dataflow start 後に何も動かない (CPU 0%) | dynamic node (robot_web_gui) 未 attach。全 dynamic node が揃うまで開始バリアで待つ |
| `multiple dataflows contain dynamic node id ...` | 過去の dataflow が Running のまま残存。`dora list` → 全部 `dora stop` |
| **モータ電源を落とした / 強制駆動 OFF した後** | CAN が ERROR-PASSIVE に落ち、PEAK ドライバは電源復帰後も自力で送信を再開しない (仕様として受容)。**復旧手順: `pcan_setup.bash can0 can1 down` → `keep` で上げ直し → dora を立ち上げ直す**。電源 OFF 中は DCM が fault=255 (NoResponse) を立てて RCM/GUI が OFF に落ち、復帰時の初回指令は必ず OFF になる (暴走しない) |
| motor_status が全ゼロ / モータ無応答 | モータ電源、`candump can0/can1` で応答確認、`ip -details link show canX` で ERROR-PASSIVE なら `pcan_setup.bash canX down` → 上げ直し |
| imu_data が全ゼロ (imu_node は Opened 成功) | Spresense が沈黙している。確認: imu_node 稼働中は `PID=$(pgrep -x imu_node); A=$(awk '/rchar/{print $2}' /proc/$PID/io); sleep 1; B=$(awk '/rchar/{print $2}' /proc/$PID/io); echo $((B-A))` (数千 bytes/s なら流れている、0 なら沈黙。`pgrep -x` でプロセス名完全一致 — `-f` だと自分のシェルに誤マッチする。imu_node が全部読むので dd では見えない)。imu_node 停止中は `timeout 2 dd if=/dev/ttyUSB0 bs=64 count=1 \| xxd` で `aa55` を確認 (keep_serial_open.bash が open 時に 921600 raw を設定するので、それを立てた後なら見える。旧版 keep_serial や素の状態だとポートが既定 9600 のままで、流れていても何も見えない — 実機で確認済み)。本当に沈黙なら USB 抜き差しで復帰 → 以後 keep_serial_open.bash を先に立てて close させない。(dialout 追加後まだ再ログインしていない場合のみ、一時しのぎに `sudo chmod 666 /dev/ttyUSB0`) |
| 指令がロボットに届かない (登録は成功) | PC daemon の `--zenoh-peer` 抜け (`dora_pc_daemon.bash` 経由なら自動で付く) |
