"""Build the RSARM assembly USD (robot_file/rsarm_v0/rsarm.usd) and the table wrapper (table.usd) from the part USDs.
Run once after parts change.

Parts (mesh/, mm, Z up, part-local frame) are referenced as they are, collision included, except that a drawn
mesh that is also a collider is split into the visual mesh and a guide-purpose collision copy (see
sim_base.usd_build.split_visual_colliders). The split is written into one layer per part (parts/<link>.usd), and
rsarm.usd references those as instanceable (sim_base.usd_build.write_part_layer). The CSVs (Fusion 360 properties)
give only each part's position ("World X,Y,Z" = joint position) and its mass properties.

    python build_rsarm_usd.py [--dir /workspace/robot_file/rsarm_v0]
"""

import argparse
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from sim_base.usd_build import add_part_body, read_csv, split_visual_colliders, write_part_layer

# (link, CSV, part file in mesh/)
PARTS = [
    ("A000", "waist.csv", "RSARM000-A000 v35.usdz"),
    ("A100", "shoulder_pitch_R.csv", "RSARM000-A100.usd"),
    ("A200", "shoulder_roll_R.csv", "RSARM000-A200.usd"),
    ("A300", "shoulder_yaw_R.csv", "RSARM000-A300.usd"),
    ("A400", "elbow_R.csv", "RSARM000-A400.usd"),
]
# (joint, parent, child, axis): roll = X, pitch = Y, yaw = Z, about the child part origin
JOINTS = [
    ("shoulder_pitch", "A000", "A100", "Y"),
    ("shoulder_roll", "A100", "A200", "X"),
    ("shoulder_yaw", "A200", "A300", "Z"),
    ("elbow_pitch", "A300", "A400", "Y"),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--dir", default="/workspace/robot_file/rsarm_v0", type=Path)
    d = parser.parse_args().dir

    out = d / "rsarm.usd"
    out.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/rsarm").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)

    # one layer per part with its colliders split, referenced as instanceable below
    (d / "parts").mkdir(exist_ok=True)
    for link, _, part in PARTS:
        part_stage, _ = write_part_layer(d / "parts" / f"{link}.usd", f"../mesh/{part}")
        part_stage.GetRootLayer().Save()

    csv = {}
    for link, csv_file, _ in PARTS:
        csv[link] = read_csv(d / csv_file)
        add_part_body(stage, f"/rsarm/{link}", csv[link], f"./parts/{link}.usd")

    fixed = UsdPhysics.FixedJoint.Define(stage, "/rsarm/joints/root_joint")  # A000 fixed to the world
    fixed.CreateBody1Rel().SetTargets(["/rsarm/A000"])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(*csv["A000"]["origin"]))
    for name, parent, child, axis in JOINTS:
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/rsarm/joints/{name}")
        joint.CreateBody0Rel().SetTargets([f"/rsarm/{parent}"])
        joint.CreateBody1Rel().SetTargets([f"/rsarm/{child}"])
        joint.CreateAxisAttr(axis)
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(csv[child]["origin"] - csv[parent]["origin"])))
        # PhysX applies the Isaac Lab actuator gains only to joints that have a drive (Newton does without)
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular").CreateTypeAttr("force")

    stage.GetRootLayer().Save()
    print(f"wrote {out}")

    # the table as it is (mm, scaled in TABLE_CFG), with its colliders split the same way
    out = d / "table.usd"
    out.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    table = stage.DefinePrim("/table", "Xform")
    table.GetReferences().AddReference("./mesh/table_assy.usd")
    stage.SetDefaultPrim(table)
    split_visual_colliders(stage)
    stage.GetRootLayer().Save()
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
