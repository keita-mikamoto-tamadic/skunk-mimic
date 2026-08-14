# robot_web_gui

dummy_input (端末) のブラウザ版。`state_command` (1 byte) を
robot_control_manager (RCM) に送り、`state_status` で現在状態を表示する。
motor_commands を直接送る `web_controller` と違い、必ず RCM の状態機械を経由する
(通常運転はこちら、単軸テスト/ゼロ点直接操作は web_controller)。

## 起動 (dataflow_mimic.yaml、全部 robot 上)

```bash
bash pcan_setup.bash can0 can1 keep
bash dora_rt_damon.bash
dora start dataflow_mimic.yaml
cd src/python && \
ROBOT_CONFIG=robot_config/mimic_v2_5.json uv run robot_web_gui/robot_web_gui.py   # dynamic ノード attach
# PC のブラウザで http://<robot-ip>:8766/
```

表示 (motor_status/imu_data) は web_monitor:
`ROBOT_CONFIG=robot_config/mimic_v2_5.json python3 tools/gui/web_monitor.py` → :8765

## ボタンと RCM 遷移ガード

| ボタン | StateCommand | 有効条件 (RCM ガードのミラー) |
|---|---|---|
| サーボON | SERVO_ON (3) | OFF のみ |
| 基準姿勢セット | INIT_POSITION_RESET (4) | OFF のみ (ストッパー押し当て後に!) |
| READY | READY (5) | STOP のみ。initial_position へ interpolation_time かけ補間 |
| RUN | RUN (1) | READY + 補間完了のみ |
| STOP | STOP (0) | OFF 以外 |
| サーボOFF | SERVO_OFF (2) | 常時 (緊急停止) |

- 表示は必ず受信した state_status 駆動 (fault / torque limit / watchdog で
  非同期に OFF へ落ちるため)。
- RUN ゲート: RCM は補間進捗を publish しないため、STOP→READY 遷移からの経過時間
  (interpolation_time + 0.3s) で GUI 側が判定。さらに RCM が黙って拒否するケースに
  備え、受理 (state==RUN) まで毎 tick RUN を再送する (sysid_controller と同方式、
  3s でタイムアウト表示)。

## 環境変数

- `ROBOT_WEB_GUI_PORT` HTTP ポート (既定 8766)
- `ROBOT_CONFIG` robot_config JSON (robot 名と interpolation_time の取得。
  相対パスはリポジトリルート基準)
