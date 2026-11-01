"""Physics backend configs."""

import math

ENGINES = ("physx", "newton_mjwarp")


def physics_cfg(engine: str, dt: float, impratio: float = 1.0, max_solver_dt: float | None = None):
    """Physics cfg for ``engine`` ("physx" or "newton_mjwarp"); None = Isaac Lab's PhysxCfg default.

    Newton: ``max_solver_dt`` splits each ``dt`` step into substeps of at most that length (for assets tuned at a
    finer step, e.g. MuJoCo equalities that soften below 2 solver steps), so the sim loop still runs once per ``dt``.
    """
    if engine != "newton_mjwarp":
        return None
    from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

    substeps = math.ceil(dt / max_solver_dt - 1e-9) if max_solver_dt else 1
    # implicitfast: stiff USD drive gains are unstable with the default explicit euler
    return NewtonCfg(solver_cfg=MJWarpSolverCfg(integrator="implicitfast", impratio=impratio), num_substeps=max(1, substeps))
