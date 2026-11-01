"""Build the CRX-5iA + Robotiq 2F-85 assembly USD (robot_file/crx5ia_2f85/crx5ia_2f85.usd). Run once.

The CRX (NVIDIA asset server) and the 2F-85 (robot_file/robotiq/isaacsim_assets submodule) are referenced as they
are. Mounting follows the Robotiq guide: the gripper is a child of the flange body (J6_link), a fixed joint ties
it to the flange, and its own articulation root and world root_joint are switched off so that it joins the CRX
articulation. The root variant set "Gripper" selects the gripper's Physics variant: PhysX or Newton.

    isaaclab -p build_crx5ia_2f85_usd.py [--out /workspace/robot_file/crx5ia_2f85/crx5ia_2f85.usd]
"""

import argparse
import math
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--out", default="/workspace/robot_file/crx5ia_2f85/crx5ia_2f85.usd", type=Path)
args = parser.parse_args()
app = AppLauncher(headless=True).app

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # noqa: E402

CRX_USD = f"{ISAAC_NUCLEUS_DIR}/Robots/Fanuc/crx5ia/crx5ia.usd"
GRIPPER_USD = "../robotiq/isaacsim_assets/grippers/Robotiq_2F_85/Robotiq_2F_85.usda"  # relative to --out
GRIPPER_VARIANTS = {"PhysX": "Physx_parallel_grip", "Newton": "Newton_parallel_grip"}

FLANGE = "/crx5ia/J6_link"
GRIPPER = f"{FLANGE}/Robotiq_2F_85"
# Flange face: J6_link/flange is at x = 0.145 m, axes as J6_link; the tool axis is +x.
# The gripper approaches along its +z, so turn it +90 deg about y (z -> x). Fingers open along y.
MOUNT_POS = Gf.Vec3d(0.145, 0.0, 0.0)
MOUNT_ROT = Gf.Quatd(math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0)


def main():
    out = args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = stage.DefinePrim("/crx5ia", "Xform")
    root.GetReferences().AddReference(CRX_USD)
    stage.SetDefaultPrim(root)

    gripper = UsdGeom.Xform(stage.DefinePrim(GRIPPER, "Xform"))
    gripper.GetPrim().GetReferences().AddReference(GRIPPER_USD)
    gripper.AddTranslateOp().Set(MOUNT_POS)
    gripper.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(MOUNT_ROT)
    # one articulation (the CRX's): drop the gripper's root API and its world-pinned root joint
    body = stage.OverridePrim(f"{GRIPPER}/Robotiq_2F_85")
    body.SetMetadata("apiSchemas", Sdf.TokenListOp.Create(deletedItems=["PhysicsArticulationRootAPI"]))
    stage.OverridePrim(f"{GRIPPER}/Robotiq_2F_85/root_joint").SetActive(False)

    mount = UsdPhysics.FixedJoint.Define(stage, f"{FLANGE}/gripper_mount_joint")
    mount.CreateBody0Rel().SetTargets([FLANGE])
    mount.CreateBody1Rel().SetTargets([f"{GRIPPER}/Robotiq_2F_85/base_link"])
    mount.CreateLocalPos0Attr(Gf.Vec3f(MOUNT_POS))
    mount.CreateLocalRot0Attr(Gf.Quatf(MOUNT_ROT))

    vset = root.GetVariantSets().AddVariantSet("Gripper")
    for name, physics in GRIPPER_VARIANTS.items():
        vset.AddVariant(name)
        vset.SetVariantSelection(name)
        with vset.GetVariantEditContext():
            stage.OverridePrim(GRIPPER).GetVariantSets().SetSelection("Physics", physics)
    vset.SetVariantSelection("PhysX")

    stage.GetRootLayer().Save()
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
    app.close()
