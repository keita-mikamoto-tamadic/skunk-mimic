#!/bin/bash

SERIAL_PORT="/dev/ttyUSB0"

echo "Keeping serial port $SERIAL_PORT open..."
echo "Press Ctrl+C to stop and release port"

# 終了シグナルをキャッチしてクリーンアップ
trap 'echo -e "\nStopping serial port keeper..."; exit 0' INT TERM

# 開く前にボーレート等を IMU に合わせて設定しておく (921600 8N1 raw)。
# termios はカーネル側に残るので、以後 imu_node の起動有無に関わらず
# `timeout 2 dd if=$SERIAL_PORT bs=64 count=1 | xxd` で aa55 が確認できる
# (未設定だと既定 9600 で読むことになり、流れていても何も見えない)。
stty -F "$SERIAL_PORT" 921600 raw -echo -hupcl 2>/dev/null

# バックグラウンドでポートを開いたまま
exec 3<$SERIAL_PORT
sleep infinity