#!/usr/bin/env python3
"""
mjmodel_converter — Fusion 360 の物理プロパティ CSV + 骨格定義 JSON から MuJoCo MJCF を生成する。

入力 (1 ディレクトリ = 1 ロボット, 例: sim/fusion_param/mimic_v2_5/):
  model.json          骨格: 親子関係・関節位置/軸・メッシュ・衝突形状・センサ・アクチュエータ利得
  <part>.csv          Fusion 360「物理」タブの書き出し (質量・重心・重心まわり慣性テンソル)。
                      model.json の各 body の "part" が <part>.csv を指す
  robot_config/*.json model.json の "robot_config" が指す。初期姿勢 (keyframe "standing")・
                      トルク制限 (forcerange)・軸名と順序 (actuator) の正本

出力:
  sim/<model>.xml     (--output で変更可)。ロボットだけ (床・光源・カメラは持たない —
                      それらは手書きの sim/scene.xml が <include> で足す):
                      free joint の base_link、visual/collision class、IMU site + センサ、
                      general アクチュエータ (gainprm/biasprm を mujoco_backend が実行時に切替)、
                      keyframe "standing" (robot_config の initial_position、base 高さは順運動学で算出)

単位変換: Fusion CSV は g / mm / g·mm² → kg / m / kg·m²。
慣性テンソルの非対角成分は Fusion の値をそのまま使う (テンソル成分そのもの。平行軸の定理で
"at Origin" ブロックと突き合わせて検証するので、規約が違えば警告が出る)。

使い方:
  cd scripts && uv sync
  uv run mjmodel_converter/mjmodel_converter.py                      # sim/fusion_param/mimic_v2_5 → sim/mimic_v2_5.xml
  uv run mjmodel_converter/mjmodel_converter.py sim/fusion_param/<robot> [--output sim/<name>.xml]
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from xml.dom import minidom

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PARAM_DIR = REPO_ROOT / "sim" / "fusion_param" / "mimic_v2_5"
SPEC_FILENAME = "model.json"

# ---------------------------------------------------------------------------
# Fusion 360 物理プロパティ CSV
# ---------------------------------------------------------------------------

MASS_UNIT = {"g": 1e-3, "kg": 1.0, "lbmass": 0.45359237}
LENGTH_UNIT = {"mm": 1e-3, "cm": 1e-2, "m": 1.0, "in": 0.0254}
INERTIA_UNIT = {
    "g mm^2": 1e-9,
    "kg mm^2": 1e-6,
    "g cm^2": 1e-7,
    "kg cm^2": 1e-4,
    "kg m^2": 1.0,
}
INERTIA_KEYS = ("Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz")


@dataclass
class MassProps:
    part: str
    mass: float                 # [kg]
    com: np.ndarray             # [m]  部品原点基準
    inertia_com: np.ndarray     # [kg m^2]  (xx, yy, zz, xy, xz, yz) 重心まわり
    inertia_origin: np.ndarray | None  # 同上、原点まわり (検証用。無ければ None)


def _num(s: str) -> float:
    s = s.strip().replace(",", "")
    return float(s)


def _value_with_unit(text: str, table: dict[str, float], what: str) -> float:
    """'2863.556 g' → 2.863556 (kg)"""
    m = re.match(r"^\s*([-+0-9.eE]+)\s*(.*?)\s*$", text)
    if not m:
        raise ValueError(f"{what}: cannot parse '{text}'")
    unit = m.group(2)
    if unit not in table:
        raise ValueError(f"{what}: unknown unit '{unit}' in '{text}' (known: {list(table)})")
    return _num(m.group(1)) * table[unit]


def parse_fusion_csv(path: Path) -> MassProps:
    mass = None
    com = None
    blocks: dict[str, dict[str, float]] = {}
    block_unit: dict[str, float] = {}
    current = None

    for raw in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw.strip("\r\n")
        if not line.strip():
            current = None if current is None else current  # 空行は無視 (ブロックは継続)
            continue
        cells = [c.strip() for c in line.split("\t") if c.strip() != ""]
        if not cells:
            continue
        key = cells[0]
        val = cells[1] if len(cells) > 1 else ""

        m = re.match(r"^Moment of Inertia at (Center of Mass|Origin)\s*\((.+?)\)\s*$", key)
        if m:
            current = "com" if m.group(1) == "Center of Mass" else "origin"
            unit = re.sub(r"\s+", " ", m.group(2)).strip()
            if unit not in INERTIA_UNIT:
                raise ValueError(f"{path.name}: unknown inertia unit '{unit}'")
            block_unit[current] = INERTIA_UNIT[unit]
            blocks[current] = {}
            continue

        if key in INERTIA_KEYS or key in ("Iyx", "Izx", "Izy"):
            if current is None:
                raise ValueError(f"{path.name}: inertia entry '{key}' outside a block")
            blocks[current][key] = _num(val) * block_unit[current]
            continue

        # ブロック外のキーに出会ったらブロック終了
        if key == "Mass":
            current = None
            mass = _value_with_unit(val, MASS_UNIT, f"{path.name} Mass")
        elif key == "Center of Mass":
            current = None
            parts = [p.strip() for p in val.split(",")]
            if len(parts) != 3:
                raise ValueError(f"{path.name}: Center of Mass '{val}' is not 3 components")
            com = np.array([_value_with_unit(p, LENGTH_UNIT, f"{path.name} CoM") for p in parts])
        elif key in ("Physical", "General", "Manage", "Bounding Box", "Volume", "Density", "Area",
                     "World X,Y,Z", "Length", "Width", "Height", "Part Number", "Part Name",
                     "Description", "Material Name", "Item Number", "Lifecycle", "Revision",
                     "State", "Change Order"):
            current = None

    if mass is None:
        raise ValueError(f"{path.name}: 'Mass' not found")
    if com is None:
        raise ValueError(f"{path.name}: 'Center of Mass' not found")
    if "com" not in blocks:
        raise ValueError(f"{path.name}: 'Moment of Inertia at Center of Mass' block not found")

    def vec(block: dict[str, float]) -> np.ndarray:
        missing = [k for k in INERTIA_KEYS if k not in block]
        if missing:
            raise ValueError(f"{path.name}: inertia block missing {missing}")
        return np.array([block[k] for k in INERTIA_KEYS])

    return MassProps(
        part=path.stem,
        mass=mass,
        com=com,
        inertia_com=vec(blocks["com"]),
        inertia_origin=vec(blocks["origin"]) if "origin" in blocks else None,
    )


def check_parallel_axis(mp: MassProps, tol: float = 0.02) -> list[str]:
    """'at Origin' が 'at CoM' + 平行軸の定理 と一致するか。

    一致すれば (a) 読み取りが正しい、(b) 非対角成分が「テンソル成分」規約 (I_xy = -∫xy dm)
    であることが同時に確認できる (積の規約なら符号が逆になって一致しない)。
    """
    if mp.inertia_origin is None:
        return [f"{mp.part}: no 'at Origin' block, parallel-axis check skipped"]
    m = mp.mass
    x, y, z = mp.com
    d2 = x * x + y * y + z * z
    shift = np.array([
        m * (d2 - x * x), m * (d2 - y * y), m * (d2 - z * z),
        -m * x * y, -m * x * z, -m * y * z,
    ])
    predicted = mp.inertia_com + shift
    warnings = []
    scale = max(np.max(np.abs(mp.inertia_origin)), 1e-12)
    for k, p, a in zip(INERTIA_KEYS, predicted, mp.inertia_origin):
        # Fusion は 4 桁程度に丸めるので、最大成分に対する相対誤差で見る
        if abs(p - a) > tol * scale:
            warnings.append(
                f"{mp.part}: parallel-axis mismatch on {k}: "
                f"CoM+shift={p:.4e} vs Origin={a:.4e} (inertia sign convention?)"
            )
    return warnings


# ---------------------------------------------------------------------------
# 骨格定義 JSON
# ---------------------------------------------------------------------------

def load_spec(param_dir: Path) -> dict:
    spec_path = param_dir / SPEC_FILENAME
    if not spec_path.exists():
        sys.exit(f"error: {spec_path} not found")
    with spec_path.open(encoding="utf-8") as f:
        return json.load(f)


def load_robot_config(spec: dict) -> dict | None:
    rc = spec.get("robot_config")
    if not rc:
        return None
    path = (REPO_ROOT / rc) if not Path(rc).is_absolute() else Path(rc)
    if not path.exists():
        sys.exit(f"error: robot_config {path} not found")
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def fmt(x: float) -> str:
    """MJCF 用の数値文字列 (有効 7 桁、末尾ゼロなし、-0 を避ける)"""
    if abs(x) < 1e-15:
        return "0"
    s = f"{x:.7g}"
    if "e" in s:
        # 1e-05 → 1e-05 (MuJoCo は指数表記を読める) ただし見やすさのため小さな値は固定小数に
        if abs(x) >= 1e-6:
            s = f"{x:.9f}".rstrip("0").rstrip(".")
    return s


def fmt_vec(v) -> str:
    return " ".join(fmt(float(c)) for c in v)


# ---------------------------------------------------------------------------
# MJCF 生成
# ---------------------------------------------------------------------------

class MjcfBuilder:
    def __init__(self, spec: dict, param_dir: Path, robot_config: dict | None):
        self.spec = spec
        self.param_dir = param_dir
        self.rc = robot_config
        self.model_name = spec["model"]
        self.bodies = [b for b in spec["bodies"]]
        self.mass_props: dict[str, MassProps] = {}
        self.warnings: list[str] = []
        self.root = ET.Element("mujoco", model=self.model_name)
        self.body_elems: dict[str, ET.Element] = {}
        self.meshdir_abs: Path | None = None  # 検証用 (main が出力先から解決して入れる)

    def load_mjmodel(self):
        """検証用に MuJoCo へ読み込む。meshdir は出力先に依らず絶対パスへ差し替える"""
        import mujoco  # 重いので遅延 import

        xml = self.to_pretty_xml()
        if self.meshdir_abs is not None:
            xml = xml.replace(f'meshdir="{self.spec.get("meshdir", "meshes/")}"',
                              f'meshdir="{self.meshdir_abs.as_posix()}/"', 1)
        return mujoco.MjModel.from_xml_string(xml)

    # -- 入力の読み込み / 検証 --------------------------------------------------
    def load_mass_props(self):
        for b in self.bodies:
            part = b["part"]
            csv_path = self.param_dir / f"{part}.csv"
            if not csv_path.exists():
                sys.exit(f"error: {csv_path} not found (body '{b['name']}')")
            mp = parse_fusion_csv(csv_path)
            self.warnings.extend(check_parallel_axis(mp))
            self.mass_props[b["name"]] = mp

    def validate_tree(self):
        names = [b["name"] for b in self.bodies]
        if len(set(names)) != len(names):
            sys.exit("error: duplicate body names in spec")
        seen = set()
        roots = 0
        for b in self.bodies:
            parent = b.get("parent")
            if parent is None:
                roots += 1
            elif parent not in seen:
                sys.exit(f"error: body '{b['name']}' parent '{parent}' must be defined earlier")
            seen.add(b["name"])
        if roots != 1:
            sys.exit(f"error: exactly one root body (parent=null) expected, got {roots}")

    # -- 各セクション ------------------------------------------------------------
    def build(self):
        self.validate_tree()
        self.load_mass_props()
        self._header_comment()
        ET.SubElement(self.root, "compiler", angle="radian", meshdir=self.spec.get("meshdir", "meshes/"))
        ET.SubElement(self.root, "option", timestep=fmt(self.spec.get("timestep", 0.001)))
        self._assets()
        self._defaults()
        self._contact()
        self._worldbody()
        self._sensors()
        # keyframe は順運動学が要るので後で差し込む (add_keyframe)
        self._actuators()

    def _header_comment(self):
        parts = ", ".join(f"{b['name']}={b['part']}" for b in self.bodies)
        self.root.append(ET.Comment(
            f" Generated by scripts/mjmodel_converter/mjmodel_converter.py — DO NOT EDIT BY HAND. "
            f"Inputs: {self.param_dir.relative_to(REPO_ROOT)}/{SPEC_FILENAME} + Fusion CSVs ({parts}), "
            f"robot_config={self.spec.get('robot_config')}. "
            f"Mass/CoM/inertia from Fusion (g,mm,g·mm² → kg,m,kg·m²); joint offsets from {SPEC_FILENAME}"
            + ("; com_offset (com_comp.py, measured balance pitch) applied to "
               + ", ".join(b["name"] for b in self.bodies if b.get("com_offset")) if any(b.get("com_offset") for b in self.bodies) else "")
            + ". "
        ))

    def _assets(self):
        asset = ET.SubElement(self.root, "asset")
        seen = set()
        for b in self.bodies:
            mesh = b.get("mesh")
            if mesh and mesh not in seen:
                seen.add(mesh)
                ET.SubElement(asset, "mesh", name=mesh, file=b.get("mesh_file", f"{mesh}.stl"))

    def _defaults(self):
        default = ET.SubElement(self.root, "default")
        vis = ET.SubElement(default, "default", {"class": "visual"})
        ET.SubElement(vis, "geom", contype="0", conaffinity="0", group="2")
        col = ET.SubElement(default, "default", {"class": "collision"})
        ET.SubElement(col, "geom", contype="1", conaffinity="1", group="3")

    def _contact(self):
        pairs = self.spec.get("contact_exclude", [])
        if not pairs:
            return
        contact = ET.SubElement(self.root, "contact")
        for b1, b2 in pairs:
            ET.SubElement(contact, "exclude", body1=b1, body2=b2)

    def _worldbody(self):
        wb = ET.SubElement(self.root, "worldbody")
        # 床・光源・カメラはロボットモデルの一部ではないので出さない。sim/scene.xml (手書き) が
        # このファイルを <include> して足す。単体で読むと床が無いので MUJOCO_MODEL=scene.xml で読むこと。
        wb.append(ET.Comment(" robot only — no floor/light/camera here; load via sim/scene.xml "))
        for b in self.bodies:
            parent_elem = wb if b.get("parent") is None else self.body_elems[b["parent"]]
            self.body_elems[b["name"]] = self._body(parent_elem, b)

    def _body(self, parent_elem: ET.Element, b: dict) -> ET.Element:
        body = ET.SubElement(parent_elem, "body", name=b["name"], pos=fmt_vec(b.get("pos", [0, 0, 0])))
        if "quat" in b:
            body.set("quat", fmt_vec(b["quat"]))
        elif "euler" in b:
            body.set("euler", fmt_vec(b["euler"]))

        j = b.get("joint")
        if j:
            jt = j.get("type", "hinge")
            if jt == "free":
                ET.SubElement(body, "joint", type="free")
            else:
                je = ET.SubElement(body, "joint", name=j["name"], type=jt, axis=fmt_vec(j.get("axis", [0, 1, 0])))
                if "range" in j:
                    je.set("range", fmt_vec(j["range"]))
                for k in ("armature", "damping", "frictionloss", "stiffness"):
                    if k in j:
                        je.set(k, fmt(j[k]))

        mesh = b.get("mesh")
        if mesh:
            ET.SubElement(body, "geom", {"class": "visual", "type": "mesh", "mesh": mesh})

        col = b.get("collision", {"type": "mesh"} if mesh else None)
        if col:
            ct = col["type"]
            if ct == "mesh":
                ET.SubElement(body, "geom", {"class": "collision", "type": "mesh", "mesh": col.get("mesh", mesh)})
            elif ct == "cylinder":
                g = ET.SubElement(body, "geom", {
                    "class": "collision", "type": "cylinder",
                    "size": fmt_vec([col["radius"], col["half_length"]]),
                })
                if "euler" in col:
                    g.set("euler", fmt_vec(col["euler"]))
                if "pos" in col:
                    g.set("pos", fmt_vec(col["pos"]))
            elif ct == "sphere":
                g = ET.SubElement(body, "geom", {"class": "collision", "type": "sphere", "size": fmt(col["radius"])})
                if "pos" in col:
                    g.set("pos", fmt_vec(col["pos"]))
            else:
                sys.exit(f"error: body '{b['name']}': unsupported collision type '{ct}'")

        mp = self.mass_props[b["name"]]
        # com_offset: CAD の重心に足す補正 [m] (body 座標系)。com_comp.py が実測の倒立 pitch から
        # 決めて model.json に書く。慣性テンソルは重心まわりのまま (小さな未モデル化質量が
        # 重心を少し動かした、という扱い)。
        com = mp.com + np.asarray(b.get("com_offset", [0.0, 0.0, 0.0]), dtype=float)
        ET.SubElement(body, "inertial", pos=fmt_vec(com), mass=fmt(mp.mass),
                      fullinertia=fmt_vec(mp.inertia_com))

        for s in b.get("sites", []):
            attrs = {k: (fmt_vec(v) if isinstance(v, list) else str(v)) for k, v in s.items() if not k.startswith("_")}
            ET.SubElement(body, "site", attrs)
        return body

    def _sensors(self):
        sensors = self.spec.get("sensors", [])
        if not sensors:
            return
        se = ET.SubElement(self.root, "sensor")
        for s in sensors:
            attrs = {k: str(v) for k, v in s.items() if k != "tag" and not k.startswith("_")}
            ET.SubElement(se, s["tag"], attrs)

    # -- アクチュエータ ----------------------------------------------------------
    def _axis_joint_map(self) -> dict[str, str]:
        return {b["axis"]: b["joint"]["name"] for b in self.bodies if b.get("axis") and b.get("joint")}

    def _actuators(self):
        act_spec = self.spec.get("actuators")
        if not act_spec:
            return
        if self.rc is None:
            sys.exit("error: 'actuators' requires 'robot_config' in the spec (axis names/order, torque_limit)")
        gains = act_spec["moteus_gains_per_rev"]
        modes = act_spec.get("initial_mode", {})
        axis_joint = self._axis_joint_map()

        act = ET.SubElement(self.root, "actuator")
        act.append(ET.Comment(
            " general actuators: gainprm/biasprm are switched at runtime by mujoco_backend "
            "for POSITION/VELOCITY/TORQUE/OFF (it reads base_kp=-biasprm[1], base_kv=-biasprm[2] from here). "
            "Gains are moteus kp [Nm/rev], kd [Nm/(rev/s)] converted to per-rad (/2π). "
            "Force = gainprm[0]*ctrl + biasprm[0] + biasprm[1]*qpos + biasprm[2]*qvel: "
            "POSITION gain=kp bias=[0,-kp,-kv]; VELOCITY gain=kv bias=[0,0,-kv]; TORQUE gain=1; OFF gain=0. "
            "forcerange = robot_config torque_limit. "
        ))
        for ax in self.rc["axes"]:
            name = ax["name"]
            if name not in axis_joint:
                sys.exit(f"error: robot_config axis '{name}' has no body with \"axis\": \"{name}\" in the spec")
            if name not in gains:
                sys.exit(f"error: no moteus gain for axis '{name}' in spec actuators.moteus_gains_per_rev")
            kp = gains[name]["kp"] / (2 * math.pi)
            kv = gains[name]["kd"] / (2 * math.pi)
            mode = modes.get(name, "position")
            if mode == "position":
                gainprm, biasprm = [kp, 0, 0], [0, -kp, -kv]
            elif mode == "velocity":
                gainprm, biasprm = [kv, 0, 0], [0, 0, -kv]
            elif mode == "torque":
                gainprm, biasprm = [1, 0, 0], [0, 0, 0]
            else:
                sys.exit(f"error: axis '{name}': unknown initial_mode '{mode}'")
            tl = float(ax["torque_limit"])
            ET.SubElement(act, "general", {
                "joint": axis_joint[name], "name": name,
                "gaintype": "fixed", "biastype": "affine",
                "gainprm": fmt_vec(gainprm), "biasprm": fmt_vec(biasprm),
                "forcelimited": "true", "forcerange": fmt_vec([-tl, tl]),
            })

    # -- keyframe ----------------------------------------------------------------
    def joint_order(self) -> list[dict]:
        """qpos の並び = body 定義順の hinge joint (free joint は先頭 7 要素)"""
        return [b for b in self.bodies if b.get("joint") and b["joint"].get("type", "hinge") != "free"]

    def initial_qpos_hinges(self) -> list[float]:
        if self.rc is None:
            return [0.0] * len(self.joint_order())
        init = {ax["name"]: float(ax["initial_position"]) for ax in self.rc["axes"]}
        out = []
        for b in self.joint_order():
            ax = b.get("axis")
            out.append(init.get(ax, 0.0) if ax else 0.0)
        return out

    def add_keyframe(self, base_z: float | None):
        has_free = any(b.get("joint", {}).get("type") == "free" for b in self.bodies)
        hinges = self.initial_qpos_hinges()
        if has_free:
            if base_z is None:
                base_z = float(self.bodies[0].get("pos", [0, 0, 0.5])[2])
            qpos = [0, 0, base_z, 1, 0, 0, 0] + hinges
        else:
            qpos = hinges
        kf = ET.SubElement(self.root, "keyframe")
        desc = ", ".join(f"{b['axis']}={fmt(q)}" for b, q in zip(self.joint_order(), hinges) if b.get("axis"))
        kf.append(ET.Comment(f" Standing pose from robot_config initial_position ({desc}); "
                             f"base z = wheel bottom on the ground at that pose (forward kinematics) "))
        ET.SubElement(kf, "key", name="standing", qpos=fmt_vec(qpos))
        # actuator の前に置きたいので並べ替え (見た目だけ)
        act = self.root.find("actuator")
        if act is not None:
            self.root.remove(kf)
            self.root.insert(list(self.root).index(act), kf)

    # -- 出力 ----------------------------------------------------------------------
    def to_pretty_xml(self) -> str:
        raw = ET.tostring(self.root, encoding="unicode")
        pretty = minidom.parseString(raw).toprettyxml(indent="    ")
        lines = [ln for ln in pretty.split("\n") if ln.strip()]
        return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# 順運動学で立位の base 高さを決める
# ---------------------------------------------------------------------------

def compute_base_height(builder: MjcfBuilder) -> float | None:
    """keyframe の関節角で、接地体 (cylinder 衝突を持つ body) の最下点が z=0 に来る base 高さ"""
    import mujoco  # 重いので遅延 import

    model = builder.load_mjmodel()
    data = mujoco.MjData(model)
    wheels = [(b["name"], b["collision"]["radius"]) for b in builder.bodies
              if b.get("collision", {}).get("type") == "cylinder"]
    if not wheels:
        return None
    root = builder.bodies[0]
    if root.get("joint", {}).get("type") != "free":
        return None

    hinges = builder.initial_qpos_hinges()
    data.qpos[:] = 0
    data.qpos[3] = 1.0  # quat w
    for b, q in zip(builder.joint_order(), hinges):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, b["joint"]["name"])
        data.qpos[model.jnt_qposadr[jid]] = q
    mujoco.mj_kinematics(model, data)

    bottoms = []
    for name, radius in wheels:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        bottoms.append(float(data.xpos[bid][2]) - radius)
    margin = float(builder.spec.get("base_height_margin", 0.0))
    return -min(bottoms) + margin


def set_standing_pose(model, data, builder: MjcfBuilder, pitch: float):
    """keyframe の関節角 + base を +y 軸まわりに pitch 回した姿勢にして順運動学を回す。
    pitch の符号は mujoco_backend.quat_to_euler (= stabilizer が見る ImuData.pitch) と同じ。"""
    import mujoco

    data.qpos[:] = 0
    data.qpos[3:7] = [math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0]
    for b, q in zip(builder.joint_order(), builder.initial_qpos_hinges()):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, b["joint"]["name"])
        data.qpos[model.jnt_qposadr[jid]] = q
    mujoco.mj_forward(model, data)


def com_over_axle(model, data, builder: MjcfBuilder, pitch: float) -> tuple[float, float]:
    """standing 姿勢 + pitch での (全体重心 x − 車軸 x, 全体重心 z − 車軸 z) [m, world]。
    ホイールが接地していれば接地点は車軸の真下なので、x 差 = 0 が倒立点。"""
    import mujoco

    set_standing_pose(model, data, builder, pitch)
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, builder.bodies[0]["name"])
    wheels = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b["name"]) for b in builder.bodies
              if b.get("collision", {}).get("type") == "cylinder"]
    if not wheels:
        raise RuntimeError("no wheel (cylinder collision) body — cannot define the balance point")
    axle = np.mean([data.xpos[w] for w in wheels], axis=0)
    com = data.subtree_com[root]
    return float(com[0] - axle[0]), float(com[2] - axle[2])


def balance_pitch(model, builder: MjcfBuilder, lo: float = -1.0, hi: float = 1.0) -> float:
    """standing 姿勢で全体重心が車軸の真上に来る base pitch [rad] (二分法)"""
    import mujoco

    data = mujoco.MjData(model)
    f_lo = com_over_axle(model, data, builder, lo)[0]
    f_hi = com_over_axle(model, data, builder, hi)[0]
    if f_lo * f_hi > 0:
        raise RuntimeError(f"balance pitch not bracketed in [{lo}, {hi}] rad")
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        f_mid = com_over_axle(model, data, builder, mid)[0]
        if f_lo * f_mid <= 0:
            hi, f_hi = mid, f_mid
        else:
            lo, f_lo = mid, f_mid
    return 0.5 * (lo + hi)


def summarize(xml_path: Path, builder: MjcfBuilder):
    import mujoco

    model = builder.load_mjmodel()
    shown = xml_path.relative_to(REPO_ROOT) if xml_path.is_relative_to(REPO_ROOT) else xml_path
    print(f"\n== {shown} ==")
    print(f"  nbody={model.nbody} njnt={model.njnt} nu={model.nu} nsensor={model.nsensor} nkey={model.nkey}")
    total = 0.0
    for b in builder.bodies:
        mp = builder.mass_props[b["name"]]
        total += mp.mass
        print(f"  {b['name']:<14} {b['part']:<14} m={mp.mass:7.4f} kg  "
              f"CoM=({mp.com[0]:+.4f}, {mp.com[1]:+.4f}, {mp.com[2]:+.4f}) m  "
              f"Ixx/Iyy/Izz=({mp.inertia_com[0]:.3e}, {mp.inertia_com[1]:.3e}, {mp.inertia_com[2]:.3e})")
    print(f"  total mass = {total:.4f} kg (MuJoCo body_subtreemass[root] = {float(model.body_subtreemass[1]):.4f})")
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "standing")
    if key >= 0:
        print(f"  keyframe standing qpos = {fmt_vec(model.key_qpos[key])}")
    offsets = {b["name"]: b["com_offset"] for b in builder.bodies if b.get("com_offset")}
    if offsets:
        print("  com_offset applied: " + ", ".join(f"{n}={fmt_vec(v)} m" for n, v in offsets.items()))
    try:
        bp = balance_pitch(model, builder)
        _, h = com_over_axle(model, mujoco.MjData(model), builder, bp)
        print(f"  balance pitch at standing pose = {bp:+.4f} rad ({math.degrees(bp):+.2f} deg), "
              f"CoM {h:.4f} m above the axle  (com_comp.py で実測値に合わせられる)")
    except RuntimeError as e:
        print(f"  balance pitch: n/a ({e})")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("param_dir", nargs="?", default=str(DEFAULT_PARAM_DIR),
                    help=f"Fusion CSV + {SPEC_FILENAME} のディレクトリ (default: {DEFAULT_PARAM_DIR.relative_to(REPO_ROOT)})")
    ap.add_argument("--output", "-o", help="出力 MJCF パス (default: sim/<model>.xml)")
    ap.add_argument("--no-validate", action="store_true", help="MuJoCo での読み込み検証とサマリ表示を省く")
    args = ap.parse_args(argv)

    param_dir = Path(args.param_dir)
    if not param_dir.is_absolute():
        param_dir = (Path.cwd() / param_dir).resolve()
        if not param_dir.exists():
            param_dir = (REPO_ROOT / args.param_dir).resolve()
    if not param_dir.is_dir():
        sys.exit(f"error: {param_dir} is not a directory")

    spec = load_spec(param_dir)
    rc = load_robot_config(spec)
    builder = MjcfBuilder(spec, param_dir, rc)
    builder.build()

    out = Path(args.output) if args.output else REPO_ROOT / "sim" / f"{spec['model']}.xml"
    if not out.is_absolute():
        out = (REPO_ROOT / out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    # meshdir は出力 XML からの相対パス (sim/ に置く前提)。検証用には絶対パスに解決する:
    # 出力先の隣に無ければ sim/ 下で探す (一時的に別の場所へ出すときのため)。
    meshdir = spec.get("meshdir", "meshes/")
    for cand in (out.parent / meshdir, REPO_ROOT / "sim" / meshdir):
        if cand.is_dir():
            builder.meshdir_abs = cand.resolve()
            break
    if builder.meshdir_abs is None:
        builder.warnings.append(f"meshdir '{meshdir}' not found next to {out} nor under sim/ — "
                                "forward kinematics / validation will fail")
    elif builder.meshdir_abs != (out.parent / meshdir).resolve():
        builder.warnings.append(f"{out} references meshdir '{meshdir}' relative to itself, "
                                f"but meshes were found at {builder.meshdir_abs} — the file will not load from there")

    # keyframe の base 高さは順運動学で求める (メッシュが要るので MuJoCo に読ませる)
    base_z = None
    try:
        base_z = compute_base_height(builder)
    except Exception as e:  # noqa: BLE001 — 失敗しても spec の pos で続行
        builder.warnings.append(f"base height via forward kinematics failed ({e}); using spec pos z")
    builder.add_keyframe(base_z)
    out.write_text(builder.to_pretty_xml(), encoding="utf-8")
    print(f"wrote {out.relative_to(REPO_ROOT) if out.is_relative_to(REPO_ROOT) else out}")
    if base_z is not None:
        print(f"  standing base height = {base_z:.4f} m (from forward kinematics)")

    for w in builder.warnings:
        print(f"warning: {w}", file=sys.stderr)

    if not args.no_validate:
        summarize(out, builder)
    return 0


if __name__ == "__main__":
    sys.exit(main())
