DORA="$HOME/dora/target/release/dora"
# daemon 間 Zenoh ランデブー endpoint (分散構成: web_controller 等)。
# 既定では daemon の Zenoh リスナーは loopback にしか bind されず、multicast
# scouting も 127.0.0.1 の locator しか広告しないため、リモート daemon から
# データが届かない。ここを bind してハブになり、リモート側は同じポートへ
# --zenoh-peer tcp/<robot-ip>:5456 で connect する。
ZENOH_PEER="tcp/0.0.0.0:5456"
pkill -f "dora (coordinator|daemon)"; sleep 1
$DORA coordinator --interface 0.0.0.0 &
sleep 2
$DORA daemon --rt --zenoh-peer "$ZENOH_PEER"