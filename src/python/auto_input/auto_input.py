"""Auto dummy_input for headless simulation testing."""

import time
import pyarrow as pa
from dora import Node

SERVO_OFF = 2
SERVO_ON = 3
READY = 5
RUN = 1

node = Node("dummy_input")

def send_cmd(val, name):
    node.send_output("state_command", pa.array([val], type=pa.uint8()))
    print(f"[auto_input] sent: {name}")

time.sleep(1.0)
send_cmd(SERVO_ON, "SERVO_ON")
time.sleep(1.0)
send_cmd(READY, "READY")
time.sleep(3.0)
send_cmd(RUN, "RUN")
print("[auto_input] Running for 10s...")
time.sleep(10.0)
send_cmd(SERVO_OFF, "SERVO_OFF")
print("[auto_input] Done")
