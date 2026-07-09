# web_controller — FOCTIVE ブラウザ操作ノード

`foctive_controller`(端末版) のブラウザ版。PC のブラウザから FOCTIVE 単軸へ
指令 (電圧/電流/速度/位置/SERVO OFF) を送る。

## なぜノードなのか / なぜ PC で動かせるのか

- バイナリ指令 (`AxisRef` 72B) を `motor_commands` に流せるのは**本物の dora ノード**
  だけ。`dora topic pub` は JSON テキストを UInt8 にするだけで使えない。
- そのノードを dataflow で **PC 側 daemon に deploy** する (start 時の `_unstable_deploy`
  は効く)。device_control_manager(robot, can0) へは Zenoh 越しに届く。これは
  `dora node add` の制約 (deploy 無視) に当たらない正攻法。

```
ブラウザ(PC) ──POST /cmd──▶ web_controller (PC daemon) ──motor_commands(Zenoh)──▶ device_control_manager (robot, can0)
```

descriptor: [`dataflow_foctive_web_control.yaml`](../../../dataflow_foctive_web_control.yaml)
(web_controller を `machine: pc` に deploy、device_control_manager は既定マシン=robot)。

## 起動手順 (分散)

robot-ip は例として `10.42.0.235`、PC daemon の machine-id は `pc`
(descriptor の `_unstable_deploy.machine` と一致させること)。

```bash
# 1) robot: coordinator + 既定 daemon、can0 UP は従来どおり (dora up / can_setup)

# 2) PC: machine-id 付き daemon を robot coordinator に参加させる (常駐)
dora daemon --coordinator-addr 10.42.0.235 --machine-id pc

# 3) dataflow を起動 (robot のリポジトリルートで叩くのが確実: 既定マシンの
#    相対パス device_control_manager をそのマシンが解決するため)
export DORA_COORDINATOR_ADDR=10.42.0.235
dora start dataflow_foctive_web_control.yaml

# 4) PC: ノード本体を起動 (path: dynamic → localhost の pc daemon にアタッチ)
cd src/python && uv run web_controller/web_controller.py

# 5) PC ブラウザ: http://localhost:8770/
```

止めるとき: `dora stop <id>`、web_controller.py は Ctrl-C。

## 操作 (ブラウザ)

- **■ SERVO OFF** (赤) … いつでも停止。最上部に常駐。
- 電圧 VOLTAGE (volt_d, volt_q, vir_ang_freq) … キャリブ不要
- 電流 CURRENT (cur_d, cur_q)
- 速度 VELOCITY / impedance (vel, kp, kd, accel_limit)
- 位置 POSITION / impedance (pos, kp, kd, accel_limit)
- インピーダンス POSITION_PD (pos, vel, torq(FF), kp_scale, kd_scale)
- カスケードPID 位置 cpos (pos, accel_limit) … FOCTIVE 専用
- カスケードPID 速度 cvel (vel, accel_limit) … FOCTIVE 専用

各行の Send で送信 (自動送信なし)。送った内容は "last sent" に出る。
マッピングは更新後の `foctive_controller` と同一 (`v`/`c`/`vel`/`p`/`pd`/`cpos`/`cvel`/`f`、
MotorState 2/3/6/7/8/9/10)。kp/kd/accel_limit の既定は 1/1/0。

## 注意

- **実モータが動く。** まず SERVO OFF を手元に。電圧は小さい値から。
- **閉ループ (電流/速度/位置) は事前に電気角キャリブが必要。** これは設定系
  (`settings_request`) なので web_controller には無い。端末版 `foctive_controller`
  の `setting calib <volt_d>` で済ませてから使うこと。電圧制御 (開ループ) は
  キャリブ不要で単体で動く。
- monitor (`tools/gui/web_monitor.py`) と併用すると、PC で操作しながら同じく PC の
  ブラウザで motor_status を見られる (web_monitor は dataflow 非依存なのでこの
  dataflow でもそのまま使える)。

## 環境変数

| var | 既定 | 用途 |
|-----|------|------|
| `WEB_CONTROLLER_PORT` | `8770` | HTTP ポート |
