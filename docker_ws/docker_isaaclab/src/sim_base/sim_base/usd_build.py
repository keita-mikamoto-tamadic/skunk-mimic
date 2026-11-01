"""Helpers for the scripts that build robot USDs from Fusion 360 exports (part USDs in mm + property CSVs)."""

import re
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

MM = 1e-3


def read_csv(path: Path) -> dict:
    """Part position [m] and mass properties [kg, m, kg m^2] from a Fusion 360 property export.

    Keys and values are separated by tabs or by spaces (both export variants), one entry per line.
    """
    txt = path.read_text(encoding="utf-8", errors="replace")

    def vec(key):
        m = re.search(rf"^\s*{re.escape(key)}\s+([-\d.E+]+) mm, ([-\d.E+]+) mm, ([-\d.E+]+) mm", txt, re.M)
        return np.array([float(v) for v in m.groups()]) * MM

    sec = txt.split("Moment of Inertia at Center of Mass")[1].split("Moment of Inertia at Origin")[0]
    i = {k: float(v) for k, v in re.findall(r"^\s*(I[xyz]{2})\s+([-\d.E+]+)", sec, re.M)}
    inertia = np.array([[i[f"I{a}{b}"] for b in "xyz"] for a in "xyz"]) * 1e-9  # g mm^2 -> kg m^2
    mass = float(re.search(r"^\s*Mass\s+([-\d.E+]+) g", txt, re.M).group(1)) * 1e-3
    return {"origin": vec("World X,Y,Z"), "com_world": vec("Center of Mass"), "mass": mass, "inertia": inertia}


def apply_mass(prim: Usd.Prim, csv: dict) -> None:
    """MassAPI from read_csv() on a body whose frame is the part frame (at csv["origin"], unrotated)."""
    # USD wants the inertia as principal moments + principal axes
    moments, axes = np.linalg.eigh(csv["inertia"])
    if np.linalg.det(axes) < 0:
        axes[:, 2] *= -1
    x, y, z, w = Rotation.from_matrix(axes).as_quat()
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr(csv["mass"])
    mass.CreateCenterOfMassAttr(Gf.Vec3f(*(csv["com_world"] - csv["origin"])))
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*moments))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(w, x, y, z))


def split_visual_colliders(stage: Usd.Stage) -> list[Usd.Prim]:
    """Split each drawn collider mesh into the visual mesh and a guide-purpose collision copy (sibling <name>_collision).

    Isaac Lab's Newton cloner imports with hide_collision_shapes=True, which hides every collider of a body (or of
    the static scene) that also has visual meshes, so a drawn mesh that is also a collider vanishes in the Newton
    viewer. Guide purpose keeps the copy hidden in Kit too, as the viewports draw only default and proxy.
    Returns the collision copies.
    """
    copies = []
    for prim in [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)]:
        img = UsdGeom.Imageable(prim)
        if img.ComputeVisibility() == UsdGeom.Tokens.invisible or img.ComputePurpose() == UsdGeom.Tokens.guide:
            continue
        col = stage.DefinePrim(prim.GetParent().GetPath().AppendChild(f"{prim.GetName()}_collision"), "Mesh")
        for attr in prim.GetAttributes():
            if attr.HasAuthoredValue():
                col.CreateAttribute(attr.GetName(), attr.GetTypeName(), custom=attr.IsCustom()).Set(attr.Get())
        for api in (UsdPhysics.CollisionAPI, UsdPhysics.MeshCollisionAPI):
            if prim.HasAPI(api):
                api.Apply(col)
                prim.RemoveAPI(api)
        UsdGeom.Imageable(col).CreatePurposeAttr(UsdGeom.Tokens.guide)
        copies.append(col)
    return copies


def write_part_layer(out: Path, part_ref: str) -> tuple[Usd.Stage, list[Usd.Prim]]:
    """New layer ``out`` whose default prim /part references ``part_ref`` (relative to ``out``) with its colliders
    split (split_visual_colliders). Returns the stage, unsaved, and the collision copies.

    Robot USDs reference these layers as instanceable: cloned envs then share each part's meshes (the Fusion exports
    are heavy; uninstanced, 4096 PhysX envs took 60 GB of host memory). An instance cannot be overridden from
    outside, hence the split in a layer of its own.
    """
    out.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = stage.DefinePrim("/part", "Xform")
    root.GetReferences().AddReference(part_ref)
    stage.SetDefaultPrim(root)
    return stage, split_visual_colliders(stage)


def add_part_body(stage: Usd.Stage, path: str, csv: dict, part_layer: str) -> Usd.Prim:
    """Rigid body ``path`` at the part position with the CSV mass properties, drawing the instanced ``part_layer``
    (in mm) as its child ``part``."""
    body = UsdGeom.Xform.Define(stage, path)
    body.AddTranslateOp().Set(Gf.Vec3d(*csv["origin"]))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    apply_mass(body.GetPrim(), csv)
    geom = UsdGeom.Xform.Define(stage, f"{path}/part")
    geom.AddScaleOp().Set(Gf.Vec3f(MM, MM, MM))
    geom.GetPrim().GetReferences().AddReference(part_layer)
    geom.GetPrim().SetInstanceable(True)
    return body.GetPrim()
