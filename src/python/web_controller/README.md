# web_controller — 多軸ブラウザ操作ノード

ブラウザから `motor_commands` (`AxisRef`) を直接送る操作ノード。state machine
(RCM) をバイパスするので、単軸テスト・ゼロ点直接操作用。多軸 (mimic 6軸 /
2 CAN チャンネル) と moteus に対応 (per-axis limits / SET_POSITION 注入)。

FOCTIVE 単体テスト (パラメータ読み書き・キャリブ・モニタ込み) は
[`foctive_web_gui`](../foctive_web_gui/) を使う。

## なぜノードなのか

バイナリ指令 (`AxisRef` 72B) を `motor_commands` に流せるのは**本物の dora ノード**
だけ。`dora topic pub` は JSON テキストを UInt8 にするだけで使えない。

```
ブラウザ(PC) ──POST /cmd──▶ web_controller (robot) ──motor_commands──▶ device_control_manager (can0+can1)
```

## 起動

descriptor: [`dataflow_mimic_web_control.yaml`](../../../dataflow_mimic_web_control.yaml)。
web_controller は dora が `src/python/run_node.bash` 経由で robot 上に起動する。

```bash
# robot
bash pcan_setup.bash can0 can1 keep
bash dora_rt_damon.bash
dora start dataflow_mimic_web_control.yaml
# PC ブラウザ: http://<robot-ip>:8770/
```

## 操作 (ブラウザ)

- **■ SERVO OFF** (赤) … いつでも全軸停止。最上部に常駐。
- 対象軸セレクタ (`[i] 軸名` / 全軸)。送信は常に全軸分の dense array で、選択軸のスロットだけ更新。
- 速度 VELOCITY / 位置 POSITION (moteus / FOCTIVE 共通)
- 基準姿勢セット (moteus のみ。config の reset_position をセット)
- 電圧 / 電流 / インピーダンス / カスケードPID (FOCTIVE のみ。moteus では非表示)

各行の Send で送信 (自動送信なし)。

## 注意

- **実モータが動く。** まず SERVO OFF を手元に。
- WiFi 断中はブラウザから止める手段がない (moteus は DCM が最新指令を再送し続ける)。
- 表示は `tools/gui/web_monitor.py` と併用。

## 環境変数

| var | 既定 | 用途 |
|-----|------|------|
| `WEB_CONTROLLER_PORT` | `8770` | HTTP ポート |
| `ROBOT_CONFIG` | (なし) | 軸名・per-axis limits。相対パスはリポジトリルート基準 |
