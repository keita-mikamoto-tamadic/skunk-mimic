#!/usr/bin/env bash
# PEAK-System PCAN-USB FD (0c72:0012) を SocketCAN (CAN-FD 1M/5M) として起動する。
#
# カーネル標準の peak_usb ドライバで canX として見えるので、追加ドライバは不要。
# インターフェース名は引数で指定、省略時は peak_usb に紐づく netdev を自動検出する。
#
# 使い方:
#   bash usbcan_setup.bash          # PCAN-USB FD を自動検出して UP
#   bash usbcan_setup.bash can0     # インターフェース指定
#   bash usbcan_setup.bash can0 down
#
# ビットレートは 1Mbps 名目 / 5Mbps データ。
# クロック 80MHz のため名目は 0.8 を取れるが、データ 5Mbps は 16tq しかなく
# dsample-point はドライバが 0.750 を自動選択する (PCAN-M.2 と同じ挙動で実績あり)。

set -euo pipefail

# peak_usb ドライバに紐づく CAN netdev を列挙
peak_usb_ifaces() {
  for d in /sys/class/net/*; do
    drv=$(readlink "$d/device/driver" 2>/dev/null || true)
    [[ "$drv" == */peak_usb ]] && basename "$d"
  done
  return 0
}

IFACE=${1:-}
MODE=${2:-up}

if [ -z "$IFACE" ]; then
  mapfile -t FOUND < <(peak_usb_ifaces)
  if [ ${#FOUND[@]} -eq 0 ]; then
    echo "エラー: PCAN-USB FD が見つかりません (lsusb / dmesg | grep peak_usb を確認)" >&2
    exit 1
  fi
  if [ ${#FOUND[@]} -gt 1 ]; then
    echo "エラー: PCAN-USB が複数あります (${FOUND[*]})。インターフェース名を指定してください" >&2
    exit 1
  fi
  IFACE=${FOUND[0]}
fi

if ! ip link show "$IFACE" &>/dev/null; then
  echo "エラー: $IFACE が存在しません" >&2
  exit 1
fi

# 1. インターフェースをダウン
sudo ip link set "$IFACE" down

if [ "$MODE" = down ]; then
  echo "$IFACE を DOWN しました"
  exit 0
fi

# 2. CAN-FD 設定 (moteus: 1M/5M, sample-point 0.8)
sudo ip link set "$IFACE" type can \
  bitrate 1000000 dbitrate 5000000 \
  sample-point 0.8 dsample-point 0.8 \
  restart-ms 100 fd on

# 3. txqueuelen
sudo ip link set "$IFACE" txqueuelen 1000

# 4. アップ
sudo ip link set "$IFACE" up

# 5. 状態確認
ip -details link show "$IFACE"
echo ""
echo "$IFACE セットアップ完了 (PCAN-USB FD, CAN-FD 1M/5M)"
