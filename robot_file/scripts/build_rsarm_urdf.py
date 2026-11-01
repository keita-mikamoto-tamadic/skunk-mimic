"""Build the RSARM kinematic URDF (robot_file/rsarm_v0/rsarm.urdf) for Pinocchio / Pink IK, from the same CSVs as
build_rsarm_usd.py (so the two cannot disagree). Run once after parts change.

Links A000..A400 with their mass properties and no geometry, the four revolute joints (the USD has no limits; the
URDF gets +-2 pi, as a revolute joint must have one and Pinocchio keeps it one coordinate), and a frame ``tcp`` fixed
on A400 at the tip sphere's center (tcp.csv).

    python build_rsarm_urdf.py [--dir /workspace/robot_file/rsarm_v0]
"""

import argparse
import math
import re
from pathlib import Path

import numpy as np

from build_rsarm_usd import JOINTS, PARTS
from sim_base.usd_build import MM, read_csv

AXES = {"X": "1 0 0", "Y": "0 1 0", "Z": "0 0 1"}


def read_position(path: Path) -> np.ndarray:
    """ "World X,Y,Z" of a Fusion 360 export without mass properties (tcp.csv) [m]."""
    m = re.search(r"World X,Y,Z\t([-\d.E+]+) mm, ([-\d.E+]+) mm, ([-\d.E+]+) mm", path.read_text(errors="replace"))
    return np.array([float(v) for v in m.groups()]) * MM


def xyz(v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--dir", default="/workspace/robot_file/rsarm_v0", type=Path)
    d = parser.parse_args().dir

    csv = {link: read_csv(d / csv_file) for link, csv_file, _ in PARTS}
    lines = ['<?xml version="1.0"?>', '<robot name="rsarm">']
    for link, c in csv.items():
        com = c["com_world"] - c["origin"]  # link frames are the parts' frames, unrotated in the world
        i = c["inertia"]
        lines += [
            f'  <link name="{link}">',
            "    <inertial>",
            f'      <origin xyz="{xyz(com)}" rpy="0 0 0"/>',
            f'      <mass value="{c["mass"]:.6g}"/>',
            f'      <inertia ixx="{i[0, 0]:.6g}" ixy="{i[0, 1]:.6g}" ixz="{i[0, 2]:.6g}" iyy="{i[1, 1]:.6g}" '
            f'iyz="{i[1, 2]:.6g}" izz="{i[2, 2]:.6g}"/>',
            "    </inertial>",
            "  </link>",
        ]
    for name, parent, child, axis in JOINTS:
        lines += [
            f'  <joint name="{name}" type="revolute">',
            f'    <parent link="{parent}"/>',
            f'    <child link="{child}"/>',
            f'    <origin xyz="{xyz(csv[child]["origin"] - csv[parent]["origin"])}" rpy="0 0 0"/>',
            f'    <axis xyz="{AXES[axis]}"/>',
            f'    <limit lower="{-2 * math.pi:.6f}" upper="{2 * math.pi:.6f}" effort="5" velocity="3"/>',
            "  </joint>",
        ]
    tcp = read_position(d / "tcp.csv") - csv["A400"]["origin"]
    lines += [
        '  <link name="tcp"/>',
        '  <joint name="tcp_fixed" type="fixed">',
        '    <parent link="A400"/>',
        '    <child link="tcp"/>',
        f'    <origin xyz="{xyz(tcp)}" rpy="0 0 0"/>',
        "  </joint>",
        "</robot>",
    ]
    out = d / "rsarm.urdf"
    out.write_text("\n".join(lines) + "\n")
    print(f"wrote {out} (tcp in A400: {xyz(tcp)})")


if __name__ == "__main__":
    main()
