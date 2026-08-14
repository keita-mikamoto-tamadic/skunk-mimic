#!/usr/bin/env bash
# PEAK-System PCAN-M.2 (CAN-FD) を SocketCAN として起動する。
#
# 前提: PEAK 用 SocketCAN ドライバ (pcan.ko, NETDEV_SUPPORT でビルド) が導入済み。
#   未導入なら peak-linux-driver を `make -C driver NET=NETDEV_SUPPORT` でビルドし
#   `sudo make -C driver install` 済みであること。
#
# チャンネル対応 (このマシン、/proc/pcan で確認):
#   can0..can3  = PEAK PCAN-M.2 4ch (pcifd hw 0..3)
#   can4        = Tegra 内蔵 CAN (c310000.mttcan)
# 注: D-Sub コネクタのラベルと hw ch の対応は SUR ケーブルの配線次第なので、
#     不明なら candump で実際に流れるチャンネルを確認すること。
#
# 使い方 (チャンネルは複数指定可。最後の引数でモード指定):
#   bash pcan_setup.bash can2             # UP して待機。Ctrl-C / 終了で自動 DOWN (デフォルト)
#   bash pcan_setup.bash can0 can1        # 複数チャンネルを UP、Ctrl-C で全部 DOWN
#   bash pcan_setup.bash can0 can1 keep   # UP したまま終了 (従来挙動。ロボット常用時)
#   bash pcan_setup.bash can0 can1 down   # DOWN するだけ
#   bash pcan_setup.bash all down         # PEAK 全ch を DOWN (テスト後の一括掃除)
#
# ビットレートは moteus 実績値 (1Mbps 名目 / 5Mbps データ) に合わせている。
# PEAK PCIe-FD のクロックは 80MHz で sample-point 0.8 を正確に取れる
# (mttcan の 50MHz と違い 0.666 も可能だが、moteus 検証済みの 0.8 に揃える)。

set -euo pipefail

usage() {
  grep -E '^#   bash pcan_setup' "$0" | sed 's/^# *//'
  exit 1
}

# 引数解析: 末尾が hold/keep/down ならモード、残りはチャンネル名
MODE=hold
IFACES=()
ARGS=("$@")
if [ ${#ARGS[@]} -gt 0 ]; then
  case "${ARGS[-1]}" in
    hold|keep|down) MODE="${ARGS[-1]}"; unset 'ARGS[-1]' ;;
  esac
fi
for a in ${ARGS[@]+"${ARGS[@]}"}; do
  case "$a" in
    can[0-9]*|all) IFACES+=("$a") ;;
    *) echo "エラー: 不明な引数 '$a'" >&2; usage ;;
  esac
done
[ ${#IFACES[@]} -eq 0 ] && IFACES=(can1)

# PEAK が持つ全チャンネル名 (/proc/pcan の ndev 列)
peak_ifaces() {
  awk '/^ *[0-9]/ {print $3}' /proc/pcan 2>/dev/null
}

# 'all' の展開 (down 専用)
if [[ " ${IFACES[*]} " == *" all "* ]]; then
  if [ "$MODE" != down ] || [ ${#IFACES[@]} -ne 1 ]; then
    echo "エラー: 'all' は単独かつ down 専用です" >&2
    usage
  fi
  mapfile -t IFACES < <(peak_ifaces)
  [ ${#IFACES[@]} -gt 0 ] || { echo "エラー: /proc/pcan が読めません (pcan 未ロード?)" >&2; exit 1; }
fi

if [ "$MODE" = down ]; then
  for i in "${IFACES[@]}"; do
    echo "$i を DOWN します"
    sudo ip link set "$i" down 2>/dev/null || true
  done
  ip -br link | grep -E "^can" || true
  exit 0
fi

# 1. ドライバ確認 (未ロードなら modprobe、失敗ならビルド済み .ko を直接 insmod)
# 注: `lsmod | grep -q` は pipefail + SIGPIPE で常に偽陽性になるため /sys を直接見る
if [ ! -d /sys/module/pcan ]; then
  echo "pcan モジュール未ロード。ロードします..."
  if ! sudo modprobe pcan 2>/dev/null; then
    KO=$(ls -t "$HOME"/peak-linux-driver-*/driver/pcan.ko 2>/dev/null | head -1)
    if [ -z "$KO" ]; then
      echo "エラー: pcan.ko が見つかりません。peak-linux-driver をビルド/インストールしてください。" >&2
      exit 1
    fi
    echo "modprobe 不可 (未インストール)。$KO を直接ロードします..."
    sudo insmod "$KO"
  fi
fi

for IFACE in "${IFACES[@]}"; do
  # 2. インターフェースをダウン (未設定でも無視)
  sudo ip link set "$IFACE" down 2>/dev/null || true

  # 3. CAN-FD 再設定 (moteus: 1M/5M, sample-point 0.8)
  sudo ip link set "$IFACE" type can \
    bitrate 1000000 dbitrate 5000000 \
    sample-point 0.65 dsample-point 0.8 \
    restart-ms 100 fd on

  # 4. txqueuelen
  sudo ip link set "$IFACE" txqueuelen 1000

  # 5. アップ
  sudo ip link set "$IFACE" up

  # 6. 状態確認
  ip -details link show "$IFACE"
done

if [ "$MODE" = keep ]; then
  echo "${IFACES[*]} は UP のままです (落とすときは: bash $0 ${IFACES[*]} down)"
  exit 0
fi

# hold モード: Ctrl-C / 終了時に全チャンネル自動 DOWN
SLEEP_PID=
cleanup() {
  [ -n "$SLEEP_PID" ] && kill "$SLEEP_PID" 2>/dev/null || true
  echo ""
  for i in "${IFACES[@]}"; do
    echo "終了: $i を DOWN します"
    # sudo のタイムスタンプが切れていたら通常 sudo (パスワード入力) にフォールバック
    sudo -n ip link set "$i" down 2>/dev/null || sudo ip link set "$i" down
  done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

echo ""
echo "${IFACES[*]} 使用中... (Ctrl-C で DOWN して終了。DOWN には sudo を使います)"
sleep infinity &
SLEEP_PID=$!
wait "$SLEEP_PID" || true
