from dora import Node
import pyarrow as pa

COMMANDS = {
    "s": 0,       # StateCommand::STOP
    "l": 1,        # StateCommand::RUN
    "f": 2,        # StateCommand::SERVO_OFF
    "o": 3,         # StateCommand::SERVO_ON
    "z": 4,      # StateCommand::INIT_POSITION_RESET
    "r": 5,      # StateCommand::READY
}

node = Node("dummy_input")

print("[dummy_input] Commands:", ", ".join(COMMANDS.keys()), ", q (quit)")

while True:
    line = input("> ").strip().lower()
    if line == "q":
        print("[dummy_input] quitting")
        break
    if line in COMMANDS:
        val = COMMANDS[line]
        node.send_output("state_command", pa.array([val], type=pa.uint8()))
        print(f"[dummy_input] sent: {line} ({val})")
    else:
        print(f"[dummy_input] unknown: '{line}'")
