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
# --- CAN 健全性チェック (情報表示のみ、起動は止めない) ---
# UP 済み can インターフェースが ERROR-PASSIVE / BUS-OFF に落ちていないかを
# 起動時に確認する。落ちていると motor_status が間欠/全ゼロになるが、
# 症状からは電源断・配線・ID 不一致と区別がつかないため、ここで先に炙り出す。
# 復旧は `bash pcan_setup.bash <canX> down` → 上げ直し (エラーカウンタもリセット)。
for dev in $(ip -br link show type can 2>/dev/null | awk '$2=="UP"{print $1}'); do
  st=$(ip -details link show "$dev" | grep -E "^\s*can " | grep -oE "state [A-Z-]+" | awk '{print $2}')
  berr=$(ip -details link show "$dev" | grep -oE "berr-counter tx [0-9]+ rx [0-9]+" | head -1)
  case "$st" in
    ERROR-ACTIVE) echo "CAN $dev: OK ($st, $berr)";;
    *)            echo "!!! CAN $dev: $st ($berr) -> pcan_setup.bash $dev down して上げ直しを推奨";;
  esac
done
pkill -f "dora (coordinator|daemon)"; sleep 1
# daemon を kill するとその daemon が spawn した C++ ノードは孤児化して残る
# (イベント待ちでブロックしたまま生存し続け、次セッションの動作を乱す)。
# 前回セッションの残骸をここで掃除してから起動する。
pkill -f "skunk-mimic/src/cpp/node/.*/build/" 2>/dev/null
sleep 1
$DORA coordinator --interface 0.0.0.0 &
sleep 2
$DORA daemon --rt
