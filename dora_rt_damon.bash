DORA="$HOME/dora/target/release/dora"
pkill -f "dora (coordinator|daemon)"; sleep 1
$DORA coordinator --interface 0.0.0.0 &
sleep 2
$DORA daemon --rt