"""フルMuJoCoモデルから3CoM等価パラメータを合成する。

3CoM = ボディ（非ホイール全リンク合成）+ wheel_L + wheel_R
出力: mb, mw, L, Ib, Iw, It, r, D
"""

import mujoco
import numpy as np
from dataclasses import dataclass


@dataclass
class ThreeCoMParams:
    """3CoMモデルの物理パラメータ"""
    mb: float   # ボディ合成質量 [kg]（ホイール以外）
    mw: float   # ホイール質量 [kg]（片輪）
    L: float    # 車輪接地点からボディ合成CoMまでの高さ [m]
    Ib: float   # ボディ合成慣性 Y軸（ピッチ） [kg·m²]
    Iw: float   # ホイール回転慣性 [kg·m²]
    It: float   # ボディ合成慣性 Z軸（ヨー） [kg·m²]
    r: float    # ホイール半径 [m]
    D: float    # トレッド幅（左右ホイール中心間距離） [m]


WHEEL_BODIES = {"wheel_L", "wheel_R"}


def create_three_com_params(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> ThreeCoMParams:
    """FK済みdata から3CoM等価パラメータを計算する。

    手順:
    1. 各ボディのワールド座標CoM・質量・慣性をMuJoCo APIから取得
    2. 非ホイールボディの合成質量・合成CoM・合成慣性（平行軸の定理）を計算
    3. ホイール半径はcollision geom (cylinder) のsizeから取得
    4. トレッド幅は左右ホイールCoMのY距離
    5. Lは合成CoMのZ座標 - ホイール接地点Z座標
    """
    # --- ホイール半径をcylinder geomから取得 ---
    r = _get_wheel_radius(model)

    # --- ボディごとの質量・ワールドCoM・ワールド慣性を収集 ---
    body_masses = []   # 非ホイール
    body_coms = []     # 非ホイール ワールドCoM
    body_inertias = [] # 非ホイール ワールド慣性行列

    wheel_mass = None
    wheel_coms = {}    # {"wheel_L": array, "wheel_R": array}
    wheel_Iw = None    # ホイール回転軸慣性

    for body_id in range(1, model.nbody):  # 0=world, skip
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        mass = model.body_mass[body_id]
        com_world = data.xipos[body_id].copy()       # ワールドCoM
        rot_world = data.ximat[body_id].reshape(3, 3) # ワールド姿勢

        # ローカル慣性（対角）→ ワールドフレームに変換
        I_local = np.diag(model.body_inertia[body_id])  # (Ixx, Iyy, Izz)
        I_world = rot_world @ I_local @ rot_world.T

        if name in WHEEL_BODIES:
            wheel_coms[name] = com_world
            wheel_mass = mass
            # ホイール回転慣性: ローカルX軸が回転軸（cylinder axis）
            # MuJoCo body_inertia は主軸 (Ixx, Iyy, Izz)
            wheel_Iw = model.body_inertia[body_id][0]  # Ixx = spin axis
        else:
            body_masses.append(mass)
            body_coms.append(com_world)
            body_inertias.append(I_world)

    # --- 非ホイールボディの合成 ---
    mb = sum(body_masses)
    # 合成CoM
    com_composite = np.zeros(3)
    for m, c in zip(body_masses, body_coms):
        com_composite += m * c
    com_composite /= mb

    # 合成慣性（平行軸の定理）: I_total = Σ(I_i + m_i * d_i^2)
    I_composite = np.zeros((3, 3))
    for m, c, I in zip(body_masses, body_coms, body_inertias):
        d = c - com_composite
        # 平行軸: I_parallel = m * (|d|² E - d⊗d)
        I_composite += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    Ib = I_composite[1, 1]  # Y軸（ピッチ）
    It = I_composite[2, 2]  # Z軸（ヨー）

    # --- トレッド幅 ---
    D = abs(wheel_coms["wheel_L"][1] - wheel_coms["wheel_R"][1])

    # --- L: ホイール接地点からボディ合成CoMまでの高さ ---
    # ホイール接地点Z = ホイールCoMのZ - r
    wheel_ground_z = (wheel_coms["wheel_L"][2] + wheel_coms["wheel_R"][2]) / 2 - r
    L = com_composite[2] - wheel_ground_z

    return ThreeCoMParams(
        mb=mb, mw=wheel_mass, L=L, Ib=Ib, Iw=wheel_Iw, It=It, r=r, D=D
    )


def _get_wheel_radius(model: mujoco.MjModel) -> float:
    """wheel_R or wheel_L のcollision cylinder geomからホイール半径を取得"""
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if name is not None and "wheel" in name:
            if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                return float(model.geom_size[geom_id, 0])  # size[0] = radius
    # fallback: collision geomは無名なのでbody所属で探す
    for body_name in WHEEL_BODIES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            continue
        for geom_id in range(model.ngeom):
            if model.geom_bodyid[geom_id] == body_id:
                if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                    return float(model.geom_size[geom_id, 0])
    raise RuntimeError("wheel cylinder geom not found")
