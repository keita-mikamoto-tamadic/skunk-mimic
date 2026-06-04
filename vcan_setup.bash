# 仮想 CAN (vcan0) を立ち上げるスクリプト
# 実機 CAN が無い状態で FDCAN プロトコル / GUI サーバー の動作確認に使う
#
# vcan は CAN FD 対応 (MTU 72): can_frame / canfd_frame どちらも流せる
# 物理ハードは不要、ローカルでループバックのみ

# 0. vcan カーネルモジュールをロード
sudo modprobe vcan

# 1. 既存の vcan0 があれば一度落とす（再実行用）
if ip link show vcan0 > /dev/null 2>&1; then
  sudo ip link set vcan0 down
  sudo ip link delete vcan0 type vcan
fi

# 2. vcan0 を作成
sudo ip link add dev vcan0 type vcan

# 3. txqueuelen を can0 と揃える
sudo ip link set vcan0 txqueuelen 1000

# 4. インターフェースをアップ
sudo ip link set vcan0 up

# 5. 状態確認
ip -details link show vcan0
