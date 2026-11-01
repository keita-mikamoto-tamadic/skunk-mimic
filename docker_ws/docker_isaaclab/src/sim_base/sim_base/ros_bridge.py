"""ROS 2 bridge (rclpy, no OmniGraph).

Topics (names configurable):
    pub  /clock                 rosgraph_msgs/Clock      sim time, every physics step
    pub  /isaac_joint_states    sensor_msgs/JointState   position / velocity
    sub  /isaac_joint_commands  sensor_msgs/JointState   position targets (matched by name), optional efforts
"""

import argparse
import os
import sys
import threading

from .bridge import Bridge

# Isaac Sim strips ROS paths from sys.path at startup; re-add the bundled Jazzy rclpy.
_RCLPY_DIR = "/isaac-sim/exts/isaacsim.ros2.core/jazzy/rclpy"


class RosBridge(Bridge):
    name = "ros"

    @staticmethod
    def add_arguments(parser: argparse.ArgumentParser) -> None:
        group = parser.add_argument_group("ROS bridge (--bridge ros)")
        group.add_argument("--joint_states_topic", default="/isaac_joint_states")
        group.add_argument("--joint_commands_topic", default="/isaac_joint_commands")
        group.add_argument("--joint_prefix", default="", help="Prefix added to joint names on the ROS side.")

    def __init__(self, args, joint_names, name_map=None, prefix=None):
        super().__init__(args, joint_names, name_map, args.joint_prefix if prefix is None else prefix)
        for p in os.environ.get("PYTHONPATH", _RCLPY_DIR).split(os.pathsep):
            if p and p not in sys.path:
                sys.path.append(p)
        import rclpy
        import rclpy.executors
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState

        self._rclpy = rclpy
        self._Clock = Clock
        self._JointState = JointState
        rclpy.init()
        self.node = rclpy.create_node("isaac_sim_bridge")
        self.pub_clock = self.node.create_publisher(Clock, "/clock", 10)
        self.pub_js = self.node.create_publisher(JointState, args.joint_states_topic, 10)
        # Several ros2_control systems may publish on the commands topic (e.g. arm and gripper, 500 Hz each). Take
        # them on an executor thread that only stores the latest value per joint; spinning per message in the
        # physics loop costs ~1 ms each and made the commands lag by seconds.
        self.node.create_subscription(JointState, args.joint_commands_topic, self._on_command, 10)
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        print(f"[INFO] ROS bridge: pub {args.joint_states_topic}, /clock  sub {args.joint_commands_topic}")

    def _on_command(self, msg):
        if self.target and msg.position:
            self.set_command(msg.name, msg.position, msg.effort)

    def start(self):
        self._thread.start()

    def publish(self, sim_time, pos, vel, effort=None):
        # no effort state: ros2_control's JointStateTopicSystem compares a command with the state of the same name
        sec = int(sim_time)
        nanosec = int((sim_time - sec) * 1e9)
        clock = self._Clock()
        clock.clock.sec, clock.clock.nanosec = sec, nanosec
        self.pub_clock.publish(clock)
        js = self._JointState()
        js.header.stamp.sec, js.header.stamp.nanosec = sec, nanosec
        js.name = self.names
        js.position = pos.tolist()
        js.velocity = vel.tolist()
        self.pub_js.publish(js)

    def shutdown(self):
        self._executor.shutdown()
        self.node.destroy_node()
        self._rclpy.shutdown()
