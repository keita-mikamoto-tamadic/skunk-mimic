#!/bin/bash

SERIAL_PORT="/dev/ttyUSB0"

echo "Keeping serial port $SERIAL_PORT open..."
echo "Press Ctrl+C to stop and release port"

# 終了シグナルをキャッチしてクリーンアップ
trap 'echo -e "\nStopping serial port keeper..."; exit 0' INT TERM

# バックグラウンドでポートを開いたまま
exec 3<$SERIAL_PORT
sleep infinity