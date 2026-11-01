"""dora bridge: a dynamic dora node speaking skunk-mimic's axis structs, as its mujoco_backend does.

    in   motor_commands  AxisRef[]  one per axis
    out  motor_status    AxisAct[]  one per axis, every physics step

So the sim slots into a skunk-mimic dataflow in place of device_control_manager (or mujoco_backend):

    - id: isaac_sim            # --dora_node_id
      path: dynamic
      inputs:
        motor_commands: robot_control_manager/motor_commands
      outputs:
        - motor_status

Start the dataflow (dora daemon of the same dora commit, see the Dockerfile), then the sim with --bridge dora.
Axes are joints by name, in axis order (--dora_axes; default: every joint in Isaac order).

The articulation takes position targets only: POSITION / POSITION_PD / CASCADE_POS_PID set the target to ref_val,
OFF and STOP hold the position they were entered at, the other modes (velocity, torque, current, voltage) are
ignored and hold too. Gain scales and limits are not applied.
"""

import argparse
import math
import os

from .bridge import Bridge
from .skunk_format import load_structs

# MotorState: mirror of skunk-mimic src/cpp/lib/enum_def.hpp (the source; src/python/lib/enum_def.py mirrors it too)
MOTOR_OFF, MOTOR_STOP, MOTOR_POSITION, MOTOR_POSITION_PD, MOTOR_CASCADE_POS_PID = 0, 1, 2, 8, 9
POSITION_MODES = {MOTOR_POSITION, MOTOR_POSITION_PD, MOTOR_CASCADE_POS_PID}


class DoraBridge(Bridge):
    name = "dora"

    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser) -> None:
        group = parser.add_argument_group("dora bridge (--bridge dora)")
        group.add_argument("--dora_node_id", default="isaac_sim", help="Node id in the dataflow (path: dynamic).")
        group.add_argument("--dora_axes", default="", help="Comma separated joint names in axis order (default: all).")
        group.add_argument(
            "--dora_data_format",
            default=os.environ.get("SKUNK_DATA_FORMAT", "/workspace/skunk_data_format") + "/axis_data.json",
            help="skunk-mimic's axis_data.json (AxisRef / AxisAct layout).",
        )

    def __init__(self, args, joint_names, name_map=None, prefix=""):
        super().__init__(args, joint_names, name_map, prefix)
        import numpy as np
        import pyarrow as pa
        from dora import Node

        self._np, self._pa = np, pa
        structs = load_structs(args.dora_data_format)
        self._ref, self._act = structs["AxisRef"], structs["AxisAct"]
        axes = [a.strip() for a in args.dora_axes.split(",") if a.strip()] or self.names
        unknown = [a for a in axes if a not in self._index]
        if unknown:
            raise ValueError(f"--dora_axes: unknown joints {unknown} (joints: {self.names})")
        self.axes = [self._index[a] for a in axes]
        self._mode = [MOTOR_STOP] * len(self.axes)
        self._pos: list[float] = []  # latest measured position (Isaac order), for the hold of OFF / STOP
        self._warned: set[int] = set()
        self._stopped = False  # the dataflow stopped: nothing to send to any more
        self.node = Node(args.dora_node_id)
        print(f"[INFO] dora bridge: node {args.dora_node_id}, in motor_commands, out motor_status, axes {axes}")

    def _receive(self) -> None:
        while (event := self.node.try_recv()) is not None:
            if event["type"] == "INPUT" and event["id"] == "motor_commands":
                self._on_commands(bytes(event["value"].to_numpy(zero_copy_only=False)))
            elif event["type"] == "STOP":
                self._stop("the dataflow stopped")

    def _on_commands(self, raw: bytes) -> None:
        for axis, i in enumerate(self.axes[: len(raw) // self._ref.size]):
            cmd = self._ref.unpack_from(raw, axis * self._ref.size)
            entered = cmd.motor_state != self._mode[axis]
            self._mode[axis] = cmd.motor_state
            if cmd.motor_state in POSITION_MODES:
                if math.isfinite(cmd.ref_val):
                    self.target[i] = cmd.ref_val
            elif entered and self._pos:
                # OFF / STOP, or a mode the position drive cannot do: hold where it was entered
                self.target[i] = self._pos[i]
                if cmd.motor_state not in (MOTOR_OFF, MOTOR_STOP) and cmd.motor_state not in self._warned:
                    self._warned.add(cmd.motor_state)
                    print(f"[WARN] dora bridge: motor_state {cmd.motor_state} not supported, holding position")

    def _stop(self, reason: str) -> None:
        self._stopped = True
        print(f"[INFO] dora bridge: {reason}; holding the last targets")

    def publish(self, sim_time, pos, vel, effort=None):
        if self._stopped:
            return
        self._pos = pos.tolist()
        self._receive()
        if self._stopped:
            return
        vel, torque = vel.tolist(), effort.tolist() if effort is not None else None
        status = b"".join(
            self._act.pack(position=self._pos[i], velocity=vel[i], torque=torque[i] if torque else 0.0)
            for i in self.axes
        )
        try:
            self.node.send_output("motor_status", self._pa.array(self._np.frombuffer(status, dtype=self._np.uint8)))
        except RuntimeError as e:  # the daemon went away (e.g. dora destroy)
            self._stop(f"send failed ({str(e).splitlines()[0]})")
