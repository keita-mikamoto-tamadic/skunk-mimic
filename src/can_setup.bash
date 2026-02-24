# 0. カーネルモジュールをリロード（エラーカウンターをリセット）
sudo modprobe -r mttcan && sudo modprobe mttcan

# 1. インターフェースをダウン
sudo ip link set can0 down

# 2. 再設定（CAN FD対応）
# mttcan 50MHzクロックでは0.666のサンプルポイントは達成できない（0.7または0.8のみ）
# 参考: https://mjbots.github.io/moteus/platforms/socketcan/
sudo ip link set can0 type can \
  bitrate 1000000 dbitrate 5000000 \
  sample-point 0.65 dsample-point 0.7 \
  restart-ms 100 fd on

# 3. txqueuelenを設定
sudo ip link set can0 txqueuelen 1000

# 4. インターフェースをアップ
sudo ip link set can0 up

# 5. 状態確認
ip -details link show can0
