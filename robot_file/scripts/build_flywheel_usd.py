"""Build the flywheel robot USD (robot_file/fly_wheel/flywheel.usd) from the part USDs. Run once after parts change.

Also writes flywheel_fixed.usd: the same robot with the base fixed to the world, for the wheel's identification.

Two bodies whose part origins coincide: base (free, it stands on the tip of its foot) and wheel, joined at that
origin by the revolute joint ``wheel`` about X. Parts (mesh/, mm, Z up) are referenced as they are, collision
included, through one layer each (parts/<part>.usd) with the colliders split into visual + guide copies and
instanced (sim_base.usd_build). The base collider, a triangle mesh in the export, is made its convex hull: a moving
body needs a convex collider, and against the floor the hull is exact (with enough hull vertices, see below). The
CSVs give the mass properties.

    python build_flywheel_usd.py [--dir /workspace/robot_file/fly_wheel]
"""

import argparse
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from sim_base.usd_build import add_part_body, read_csv, write_part_layer

# (body, CSV, part file in mesh/)
PARTS = [
    ("base", "body.csv", "base.usd"),
    ("wheel", "wheel.csv", "wheel.usd"),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--dir", default="/workspace/robot_file/fly_wheel", type=Path)
    d = parser.parse_args().dir

    out = d / "flywheel.usd"
    out.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/flywheel").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)

    (d / "parts").mkdir(exist_ok=True)
    for body, csv_file, part in PARTS:
        part_stage, colliders = write_part_layer(d / "parts" / f"{body}.usd", f"../mesh/{part}")
        for col in colliders:
            UsdPhysics.MeshCollisionAPI(col).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
            # the default 64-vertex hulls cut the foot tip off at one end (the base then stands on a slanted
            # edge): Newton's exact hull, PhysX's largest
            col.AddAppliedSchema("NewtonMeshCollisionAPI")
            col.CreateAttribute("newton:maxHullVertices", Sdf.ValueTypeNames.Int).Set(-1)
            col.AddAppliedSchema("PhysxConvexHullCollisionAPI")
            col.CreateAttribute("physxConvexHullCollision:hullVertexLimit", Sdf.ValueTypeNames.Int).Set(255)
        part_stage.GetRootLayer().Save()
        add_part_body(stage, f"/flywheel/{body}", read_csv(d / csv_file), f"./parts/{body}.usd")

    # no root joint: the base is free
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/flywheel/joints/wheel")
    joint.CreateBody0Rel().SetTargets(["/flywheel/base"])
    joint.CreateBody1Rel().SetTargets(["/flywheel/wheel"])
    joint.CreateAxisAttr("X")
    UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular").CreateTypeAttr("force")

    stage.GetRootLayer().Save()
    print(f"wrote {out}")

    # the same with the base fixed to the world (wheel system identification): a layer over flywheel.usd adding the
    # root joint (the articulation root is the /flywheel Xform, so Isaac Lab's fix_root_link cannot add one)
    fixed = d / "flywheel_fixed.usd"
    fixed.unlink(missing_ok=True)
    fixed_stage = Usd.Stage.CreateNew(str(fixed))
    fixed_stage.GetRootLayer().subLayerPaths.append("./flywheel.usd")
    fixed_stage.SetDefaultPrim(fixed_stage.GetPrimAtPath("/flywheel"))
    root_joint = UsdPhysics.FixedJoint.Define(fixed_stage, "/flywheel/joints/root")
    root_joint.CreateBody1Rel().SetTargets(["/flywheel/base"])
    fixed_stage.GetRootLayer().Save()
    print(f"wrote {fixed}")


if __name__ == "__main__":
    main()
