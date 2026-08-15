# mimic_v2_5 起動手順 (dataflow_mimic.yaml / 実機運転)

robot = Jetson (192.168.1.9, coordinator + RT daemon + C++ ノード群)、
PC = 操作用 (robot_web_gui を deploy、ブラウザ操作)。

単軸テスト・ゼロ点直接操作 (state machine バイパス) は
`dataflow_mimic_web_control.yaml` を使う — 手順はそのファイル冒頭コメント参照。

---

## 0. 前提 (初回 / リビルド後のみ)

- **dora のバージョン一致**: robot と PC の `~/dora` は同一コミット (`fd1f050b`)。
  ズレると「TCP は繋がるのに register されない」— docs/dora-upgrade.md 参照。
- **setcap** (バイナリを作り直すたびに消える):

  ```bash
  # robot 側
  sudo setcap cap_sys_nice,cap_ipc_lock+ep ~/dora/target/release/dora
  sudo setcap cap_sys_nice,cap_ipc_lock+ep ~/skunk-mimic/src/cpp/node/device_control_manager/build/device_control_manager
  getcap で確認。dora 側が無いと --rt daemon が dataflow spawn 時に
  "memory allocation of N bytes failed" で abort する
  ```

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
bash dora_rt_damon.bash

# 3) dataflow 起動 (PC daemon の接続後でも先でも可。ただし start は
#    PC daemon 接続後でないと "no matching daemon for machine id `pc`" になる)
dora start dataflow_mimic.yaml
#    attach モード (既定) のまま使う: そのターミナルの Ctrl-C で dataflow が確実に
#    止まる。--detach はターミナルを閉じても走り続けるので使わない
```

## 2. PC 側

```bash
cd ~/ws/skunk_mimic

# 1) PC daemon を robot の coordinator に接続 (繋ぎっぱなしにする)
bash dora_pc_daemon.bash 192.168.1.9

# 2) (robot 側で dora start 後) 操作 GUI — dynamic node が attach するまで
#    dataflow は開始バリアで待つ
cd src/python
ROBOT_CONFIG=robot_config/mimic_v2_5.json uv run robot_web_gui/robot_web_gui.py

# 3) 表示 (別ターミナル、任意)
cd ~/ws/skunk_mimic
DORA_COORDINATOR_ADDR=192.168.1.9 ROBOT_CONFIG=robot_config/mimic_v2_5.json \
  python3 tools/gui/web_monitor.py
```

ブラウザ:

- 操作: **http://localhost:8766/** (robot_web_gui)
- 表示: **http://localhost:8765/** (web_monitor、6軸 position/velocity/torque)

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

PC 側は daemon / GUI のターミナルを Ctrl-C。

## 5. トラブルシュート (実際に踏んだもの)

| 症状 | 原因と対処 |
|---|---|
| `no matching daemon for machine id 'pc'` | PC daemon 未接続 (coordinator 再起動後は PC 側も再接続が必要)。`dora_pc_daemon.bash` を再実行 |
| PC daemon が `timeout waiting for register reply` を繰り返す | robot 側 coordinator が落ちている、または dora のコミット不一致 (docs/dora-upgrade.md) |
| daemon が `memory allocation of N bytes failed` で abort | dora バイナリの setcap 消失 (§0) |
| dataflow start 後に何も動かない (CPU 0%) | dynamic node (robot_web_gui) 未 attach。全 dynamic node が揃うまで開始バリアで待つ |
| `multiple dataflows contain dynamic node id ...` | 過去の dataflow が Running のまま残存。`dora list` → 全部 `dora stop` |
| **モータ電源を落とした / 強制駆動 OFF した後** | CAN が ERROR-PASSIVE に落ち、PEAK ドライバは電源復帰後も自力で送信を再開しない (仕様として受容)。**復旧手順: `pcan_setup.bash can0 can1 down` → `keep` で上げ直し → dora を立ち上げ直す**。電源 OFF 中は DCM が fault=255 (NoResponse) を立てて RCM/GUI が OFF に落ち、復帰時の初回指令は必ず OFF になる (暴走しない) |
| motor_status が全ゼロ / モータ無応答 | モータ電源、`candump can0/can1` で応答確認、`ip -details link show canX` で ERROR-PASSIVE なら `pcan_setup.bash canX down` → 上げ直し |
| imu_data が全ゼロ (imu_node は Opened 成功) | Spresense が沈黙している。確認: imu_node 稼働中は `PID=$(pgrep -x imu_node); A=$(awk '/rchar/{print $2}' /proc/$PID/io); sleep 1; B=$(awk '/rchar/{print $2}' /proc/$PID/io); echo $((B-A))` (数千 bytes/s なら流れている、0 なら沈黙。`pgrep -x` でプロセス名完全一致 — `-f` だと自分のシェルに誤マッチする。imu_node が全部読むので dd では見えない)。`dd` 直読み (`timeout 2 dd if=/dev/ttyUSB0 bs=64 count=1 \| xxd`) は **見えたら流れている、見えなくても沈黙とは限らない** (termios 状態やタイミングで空になる) — 確実なのは imu_node の rchar レートか GUI/topic の値。本当に沈黙なら USB 抜き差しで復帰 → 以後 keep_serial_open.bash を先に立てて close させない。(dialout 追加後まだ再ログインしていない場合のみ、一時しのぎに `sudo chmod 666 /dev/ttyUSB0`) |
| 指令がロボットに届かない (登録は成功) | PC daemon の `--zenoh-peer` 抜け (`dora_pc_daemon.bash` 経由なら自動で付く) |
