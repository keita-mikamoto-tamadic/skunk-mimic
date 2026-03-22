"""Auto input: SERVO_ON → READY → RUN を自動送信するテスト用ノード。

Usage:
  uv run auto_input/auto_input.py [--ready-wait 4] [--run-duration 10]
"""
import argparse
import time

import pyarrow as pa
from dora import Node

SERVO_ON = 3
READY = 5
RUN = 1
STOP = 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ready-wait", type=float, default=4.0,
                        help="READY 後の待機時間 [s]")
    parser.add_argument("--run-duration", type=float, default=10.0,
                        help="RUN 継続時間 [s]")
    args = parser.parse_args()

    node = Node("dummy_input")

    def send(cmd, name):
        node.send_output("state_command", pa.array([cmd], type=pa.uint8()))
        print(f"[auto_input] sent: {name}")

    send(SERVO_ON, "SERVO_ON")
    time.sleep(1.0)

    send(READY, "READY")
    time.sleep(args.ready_wait)

    send(RUN, "RUN")
    time.sleep(args.run_duration)

    send(STOP, "STOP")
    time.sleep(0.5)

    print("[auto_input] done")


if __name__ == "__main__":
    main()
