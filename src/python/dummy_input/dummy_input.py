from dora import Node
import pyarrow as pa
import sys
import tty
import termios

COMMANDS = {
    "s": (0, "STOP"),
    "l": (1, "RUN"),
    "f": (2, "SERVO_OFF"),
    "o": (3, "SERVO_ON"),
    "z": (4, "POSITION_RESET"),
    "r": (5, "READY"),
}

node = Node("dummy_input")

print("[dummy_input] Commands:")
for key, (val, name) in COMMANDS.items():
    print(f"  {key} : {name}")
print("  q : QUIT")

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
try:
    tty.setraw(fd)
    while True:
        ch = sys.stdin.read(1).lower()
        if ch == "q":
            print("\r[dummy_input] quitting\r\n", end="")
            break
        if ch in COMMANDS:
            val, name = COMMANDS[ch]
            node.send_output("state_command", pa.array([val], type=pa.uint8()))
            print(f"\r[dummy_input] sent: {name}\r\n", end="")
        elif ch == "\x03":  # Ctrl+C
            print("\r[dummy_input] interrupted\r\n", end="")
            break
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
