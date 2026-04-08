#!/bin/bash
# リアルタイム優先度付きでdora dataflowを起動
# SCHED_FIFO を使うために必要な権限を設定
#
# 使い方:
#   ./start_rt.sh                    # dataflow.yaml (実機)
#   ./start_rt.sh dataflow_sim.yaml  # シミュレーション

DATAFLOW="${1:-dataflow.yaml}"
DORA="$HOME/dora/target/release/dora"

# dora daemon/coordinator に RT 権限を付与
# (既に起動済みなら不要)
if ! pgrep -f "dora-daemon" > /dev/null; then
    echo "Starting dora daemon..."
    sudo setcap cap_sys_nice+ep "$DORA"
    $DORA up
    sleep 1
fi

# RCM と DCM のバイナリに RT ケーパビリティ付与
RCM_BIN="src/cpp/node/robot_control_manager/build/robot_control_manager"
DCM_BIN="src/cpp/node/device_control_manager/build/device_control_manager"

for bin in "$RCM_BIN" "$DCM_BIN"; do
    if [ -f "$bin" ]; then
        sudo setcap cap_sys_nice+ep "$bin"
        echo "Set RT capability: $bin"
    fi
done

echo "Starting dataflow: $DATAFLOW"
$DORA start "$DATAFLOW"
