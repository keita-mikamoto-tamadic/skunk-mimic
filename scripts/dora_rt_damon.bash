DORA="$HOME/dora/target/release/dora"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 全ノード robot ローカルで動かす (分散構成は廃止)。dora の既定 zenoh 設定で足りるので
# ZENOH_CONFIG は付けない。シェルに残っていると ctrl_avg (RCM 往復) が約 2 倍
# (595→1090us)、GUI の受信 CPU も約 2 倍になる (実測) ので明示的に外す。
unset ZENOH_CONFIG
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
pkill -f "$REPO_DIR/src/cpp/node/.*/build/" 2>/dev/null
# Python ノード (run_node.bash 経由の spawn / 手動起動の dynamic) も同様に残る。
pkill -f "data_viewer/data_viewer.py|robot_web_gui/robot_web_gui.py|web_controller/web_controller.py|foctive_web_gui/foctive_web_gui.py" 2>/dev/null
sleep 1
$DORA coordinator --interface 0.0.0.0 &
sleep 2
$DORA daemon --rt
