# web motor monitor (tools/gui)

ブラウザでモータ状態 (position / velocity / torque / i_d / i_q / fault) をライブ表示する
GUI。`data_viewer.py` のブラウザ版。**特定の dataflow に依存しない** ―― foctive でも
mimic でも、`device_control_manager/motor_status` を出している相手なら何でも映る。

## 仕組み

```
[実行中 dataflow を自動発見]  dora list --format json
            │
            ▼
dora topic echo -d <df> device_control_manager/motor_status --format json
            │   {"data":[<AxisAct 48B>...], ...}   ← dora が envelope を解いて生バイトを渡す
            ▼
web_monitor.py ── AxisAct デコード(lib.data_format) ──HTTP/SSE(:8765)──> web_monitor.html
```

- **dataflow 非依存**: `dora topic echo` でトピックを購読するだけ。`dora node add` も
  `_unstable_deploy` も daemon 配置も使わない。dataflow が別の相手(mimic 等)に変わっても、
  `dora list` で再発見して自動追従する。
- **どこでも動く**: `dora topic echo` は coordinator に WebSocket で繋ぐので、PC からでも
  robot からでも実行できる。`DORA_COORDINATOR_ADDR` で robot を指すだけ。
- **デコードは dora 任せ**: 受け取る `data` は AxisAct の生バイト。こちらは
  `lib.data_format` (正本) で unpack するだけ。bincode/Arrow には触らない。
- **唯一の前提**: 監視したい dataflow に `_unstable_debug.enable_debug_inspection: true`。
  これはトピック inspection 一般に必要なフラグで、monitor 固有の依存ではない。
  (`dataflow_foctive_control.yaml` には既に設定済み。)

## 依存

第三者パッケージなし。Python stdlib だけ。`dora` CLI が PATH か
`~/dora/target/release/dora` にあること。**uv 不要** ―― `python3 web_monitor.py` で動く。

ファイル:

| file | 役割 |
|------|------|
| `web_monitor.py` | dora topic echo を回し、デコードして HTTP/SSE 配信 |
| `web_monitor.html` | ブラウザ GUI (数値テーブル＋リアルタイムグラフ) |
| `uPlot.iife.min.js` / `uPlot.min.css` | 同梱グラフライブラリ (uPlot v1.6.31, MIT)。同一オリジン配信 |

## グラフ (uPlot)

数値テーブルの下に時系列グラフを表示。x 軸は時間 (dora の unix 秒)。

- **信号の選択表示**: 上部のフィールドチェックボックス (position/velocity/torque/
  cur_d/cur_q) で軸まとめて on/off。凡例クリックで 1 系列ずつ on/off も可。
  初期は桁の大きい `position` を非表示にしてある。
- **時間窓スケール**: `window` で 5/10/30/60 秒を切替 (ライブ追従)。
- **Pause**: 追従を止めて凍結。グラフをドラッグで範囲ズーム / ダブルクリックで
  リセット。`▶ Live` で追従再開。
- **Clear**: バッファを消去。
- Y 軸は表示中の系列でオートスケール (桁が違う信号を一緒に出すと小さい方が潰れる
  ので、見たい信号だけ選ぶと良い)。多軸ロボット (mimic 等) では系列数 = 軸数 × 5。

## 起動

```bash
# coordinator を指す (PC から robot を見るなら robot の IP)
export DORA_COORDINATOR_ADDR=10.42.0.235

# 監視したい dataflow を起動済みにしておく (enable_debug_inspection: true 必須)
#   例: robot 側で dora start dataflow_foctive_control.yaml

# モニタ起動 (PC でも robot でも可)
cd tools/gui
ROBOT_CONFIG=robot_config/foctive_motor_test.json python3 web_monitor.py
#   ↑ ROBOT_CONFIG は軸名/CAN ID 表示用 (任意。未指定なら #0,#1,...)

# ブラウザ
#   PC で動かしたなら http://localhost:8765/
#   robot で動かして PC から見るなら http://<robot-ip>:8765/
```

止めるのは Ctrl-C のみ。dataflow 側には何も足していないので後始末不要。

## 環境変数

| var | 既定 | 用途 |
|-----|------|------|
| `DORA_COORDINATOR_ADDR` | `127.0.0.1` | coordinator の IP (dora CLI が読む) |
| `WEB_MONITOR_PORT` | `8765` | HTTP ポート |
| `DORA_TOPIC` | `device_control_manager/motor_status` | 購読トピック |
| `DORA_DATAFLOW` | (自動発見) | 固定したい時の dataflow 名/UUID |
| `DORA_BIN` | PATH / `~/dora/target/release/dora` | dora 実行ファイル |
| `ROBOT_CONFIG` | (なし) | 軸名/CAN ID 表示用 |
