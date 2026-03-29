"""MuJoCoモデルローダー: XMLからmjModel/mjDataを生成し、keyframeで初期姿勢を適用"""

import mujoco
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = PROJECT_ROOT / "sim" / "mimic_v2.xml"


def load_model(
    model_path: str | Path = DEFAULT_MODEL,
    keyframe: str = "standing",
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """MuJoCoモデルをロードし、keyframeを適用してFK計算済みの状態で返す。

    Returns:
        (model, data) — data は mj_forward() 済み
    """
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    # keyframe適用
    if keyframe is not None:
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
        if key_id < 0:
            raise ValueError(f"keyframe '{keyframe}' not found in model")
        mujoco.mj_resetDataKeyframe(model, data, key_id)

    mujoco.mj_forward(model, data)
    return model, data
