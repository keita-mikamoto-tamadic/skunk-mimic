#!/usr/bin/env bash
# PC 側 daemon 起動 (分散構成: robot_web_gui / web_controller を PC に deploy する dataflow 用)。
#
# 使い方:
#   bash dora_pc_daemon.bash <robot-ip>     # 例: bash dora_pc_daemon.bash 192.168.1.9
#
# - --machine-id pc は dataflow yaml の _unstable_deploy.machine: pc と一致させる
# - --zenoh-peer は必須。robot 側 daemon の zenoh は listen 専用
#   (robot_config/zenoh_robot.json5) なので、PC 側から明示接続しないと
#   「daemon 登録は通るのにノードのデータが流れない」状態になる
# - RUST_LOG は "Received Hello with no locators" WARN 洪水の抑制
#   (実害なしのログ公害。詳細は dora_rt_damon.bash のコメント参照)
#
# 終了時 (Ctrl-C) の後始末:
#   daemon を先に落とすと、この PC で手動起動した dynamic ノード
#   (robot_web_gui / web_controller) は「二度と来ないイベント」を dora の
#   recv() で待ち続け、Python に制御が戻らないため Ctrl-C が効かなくなる。
#   daemon 終了時にそれらのノードも一緒に落とす。
ROBOT_IP="${1:?usage: bash dora_pc_daemon.bash <robot-ip>}"
DORA="$HOME/dora/target/release/dora"
export RUST_LOG="${RUST_LOG:-warn,zenoh::net::runtime::orchestrator=error}"

cleanup() {
  echo "dora_pc_daemon: stopping local dynamic nodes..."
  pkill -f "robot_web_gui/robot_web_gui.py" 2>/dev/null
  pkill -f "web_controller/web_controller.py" 2>/dev/null
}
trap cleanup EXIT

"$DORA" daemon --coordinator-addr "$ROBOT_IP" --machine-id pc \
     --zenoh-peer "tcp/$ROBOT_IP:5456"
