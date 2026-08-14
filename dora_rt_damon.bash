DORA="$HOME/dora/target/release/dora"
# 分散構成 (web_controller 等) 用: daemon の Zenoh を listen-only 設定にする
# (robot_config/zenoh_robot.json5 で tcp/0.0.0.0:5456 を listen)。
# リモート daemon (PC) は --zenoh-peer tcp/<robot-ip>:5456 で明示接続する。
#
# 注意: robot 側に --zenoh-peer を使ってはいけない。listen と connect の両方に
# 追加されるため自分自身への接続試行 (CONNECTION_TO_SELF) が 4 秒ごとに走り、
# linkstate 経路が壊れてリモートノードのデータが最初の 1 件しか届かなくなる。
# 詳細は robot_config/zenoh_robot.json5 のコメント参照。
export ZENOH_CONFIG="$HOME/skunk-mimic/robot_config/zenoh_robot.json5"
# "Received Hello with no locators" WARN の洪水を抑制する。
# 原因: daemon が spawn したノードは ZENOH_CONFIG 継承で 5456 bind に失敗し
# listen なし peer になる (意図的、zenoh_robot.json5 参照) → locator 空の Hello を
# multicast し続け、全セッションが毎回 WARN を出す。実害なしのログ公害なので
# orchestrator だけ error に落とす (他の WARN は残す)。spawn ノードにも継承される。
export RUST_LOG="${RUST_LOG:-warn,zenoh::net::runtime::orchestrator=error}"
pkill -f "dora (coordinator|daemon)"; sleep 1
$DORA coordinator --interface 0.0.0.0 &
sleep 2
$DORA daemon --rt
