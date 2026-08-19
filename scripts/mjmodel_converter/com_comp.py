#!/usr/bin/env python3
"""
com_comp — 実測の倒立 pitch に合わせて model.json の重心補正 (com_offset) を決める。

CAD (Fusion CSV) の質量特性で作った MuJoCo モデルは、配線・バッテリー・搭載物など CAD に無い
質量のぶん倒立点 (全体重心が車軸の真上に来る base pitch) が実機とずれる。実機の倒立点は
その姿勢で平衡する角度を実測したもので確かなので、それを正とし、モデル側のいくつかの body の
重心を body 座標 x 方向にだけ動かして倒立点を一致させる。

同定できる未知数の数 = 実測した姿勢の数。
  * 1 姿勢 (立位のみ)           → base_link の x だけ
  * 2 姿勢 (立位 + 脚を伸ばした姿勢) → base_link の x + upper_link (左右同じ値) の x   ← 既定
  * 姿勢を増やせば lower_link 等も加えられる (--bodies)
base だけの補正は姿勢を変えると倒立点がずれる (立位 0.165 → 最高点 0.181 rad、2026-08-20 実機) ので、
脚リンクの重心誤差として脚にも振る。

やること:
  1. model.json を読み、対象 body の既存 com_offset を 0 にした (= CAD そのままの) モデルを組む
  2. 各実測姿勢 (hip, knee) での倒立 pitch を計算し、実測との残差を最小二乗 (Gauss-Newton、
     数値微分) で 0 にする各 body 群の dx を求める
  3. model.json の対象 body に "com_offset": [dx, 0, 0] と根拠コメント "_com_offset" を書く
     (他の行・整形は触らない)
  4. converter を呼んで sim/<model>.xml を再生成 (--no-regen で省略)

使い方:
  cd scripts
  # 立位 (initial_position) の倒立 pitch 0.165 と、重心を +0.08 m 上げた姿勢での 0.181:
  uv run mjmodel_converter/com_comp.py 0.165 --at 0.08:0.181
  # 姿勢を関節角で直接指定 (stabilizer の "posture:" ログの hip/knee):
  uv run mjmodel_converter/com_comp.py 0.165 --pose 0.6056,-1.1979:0.181
  # 補正する body 群を変える (prefix は左右に展開される):
  uv run mjmodel_converter/com_comp.py 0.165 --at 0.08:0.181 --bodies base_link,lower_link
  uv run mjmodel_converter/com_comp.py 0.165 --dry-run       # 計算だけ
  uv run mjmodel_converter/com_comp.py --reset               # 全 com_offset を外して CAD の値に戻す

--at の「上げ量」は、実測時にロボットが使っていたモデル (= いまの model.json、補正込み) の IK で
関節角に直してから使う (実測は関節角に紐づくので)。結果の根拠コメントには関節角で残す。

pitch の符号は mujoco_backend.quat_to_euler (= stabilizer の ImuData.pitch) と同じ: base を +y 軸
まわりに回した角。angle_pid の kTargetPitch と同じ量なのでそのまま渡せる。
"""

from __future__ import annotations

import argparse
import datetime as _dt
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mjmodel_converter as conv  # noqa: E402

REPO_ROOT = conv.REPO_ROOT


# ---------------------------------------------------------------------------
# モデルと倒立点
# ---------------------------------------------------------------------------

class BalanceModel:
    """spec から組んだ MuJoCo モデル上で、任意の (hip, knee) 姿勢の倒立 pitch を求める。
    body_ipos (body 座標の重心) を直接いじって補正を試せる (XML を作り直さない)。"""

    def __init__(self, spec: dict, param_dir: Path, rc: dict | None, meshdir_abs: Path):
        import mujoco

        self.mujoco = mujoco
        self.builder = conv.MjcfBuilder(spec, param_dir, rc)
        self.builder.build()
        self.builder.meshdir_abs = meshdir_abs
        self.model = self.builder.load_mjmodel()
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, self.builder.bodies[0]["name"])
        self.wheels = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b["name"]) for b in self.builder.bodies
                       if b.get("collision", {}).get("type") == "cylinder"]
        if not self.wheels:
            sys.exit("error: no wheel (cylinder collision) body — cannot define the balance point")
        # hip/knee の関節 (robot_config の軸名 = joint 名)
        if rc is None:
            sys.exit("error: robot_config required (hip_pitch_*/knee_* axes and initial_position)")
        names = {a["name"] for a in rc["axes"]}
        need = {"hip_pitch_r", "knee_r", "hip_pitch_l", "knee_l"}
        if not need <= names:
            sys.exit(f"error: robot_config must have axes {sorted(need)}")
        self.qadr = {}
        for n in need:
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            if j < 0:
                sys.exit(f"error: joint '{n}' not in the model (converter names joints after robot_config axes)")
            self.qadr[n] = m.jnt_qposadr[j]
        init = {a["name"]: float(a["initial_position"]) for a in rc["axes"]}
        self.hip0, self.knee0 = init["hip_pitch_r"], init["knee_r"]
        self.ipos0 = m.body_ipos.copy()

    def body_id(self, name: str) -> int:
        bid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            sys.exit(f"error: body '{name}' not found in the built model")
        return bid

    def com_rel(self, hip: float, knee: float, pitch: float) -> tuple[float, float]:
        d, m = self.data, self.model
        d.qpos[:] = 0
        d.qpos[3:7] = [math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0]
        for n, a in self.qadr.items():
            d.qpos[a] = hip if n.startswith("hip") else knee
        self.mujoco.mj_forward(m, d)
        axle = np.mean([d.xpos[w] for w in self.wheels], axis=0)
        com = d.subtree_com[self.root]
        return float(com[0] - axle[0]), float(com[2] - axle[2])

    def balance_pitch(self, hip: float, knee: float, lo: float = -1.0, hi: float = 1.0) -> float:
        f_lo = self.com_rel(hip, knee, lo)[0]
        f_hi = self.com_rel(hip, knee, hi)[0]
        if f_lo * f_hi > 0:
            sys.exit(f"error: balance pitch not bracketed in [{lo}, {hi}] rad for hip={hip}, knee={knee}")
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            f_mid = self.com_rel(hip, knee, mid)[0]
            if f_lo * f_mid <= 0:
                hi, f_hi = mid, f_mid
            else:
                lo, f_lo = mid, f_mid
        return 0.5 * (lo + hi)

    def com_height(self, hip: float, knee: float, pitch: float) -> float:
        return self.com_rel(hip, knee, pitch)[1]

    def ik_for_rise(self, rise: float, pitch: float) -> tuple[float, float]:
        """倒立 pitch を保ったまま重心を rise [m] 上げる (hip, knee) — stabilizer の posture IK と同じ拘束"""
        h_target = self.com_height(self.hip0, self.knee0, pitch) + rise
        hip, knee = self.hip0, self.knee0
        for _ in range(50):
            x, h = self.com_rel(hip, knee, pitch)
            r = np.array([x, h - h_target])
            if np.linalg.norm(r) < 1e-9:
                break
            eps = 1e-6
            J = np.column_stack([
                (np.array(self.com_rel(hip + eps, knee, pitch)) - np.array([x, h])) / eps,
                (np.array(self.com_rel(hip, knee + eps, pitch)) - np.array([x, h])) / eps,
            ])
            step = np.linalg.solve(J, -r)
            hip += step[0]
            knee += step[1]
        return hip, knee

    def set_offsets(self, groups: list[list[int]], dx: np.ndarray, axis: int = 0):
        self.model.body_ipos[:] = self.ipos0
        for bids, d in zip(groups, dx):
            for bid in bids:
                self.model.body_ipos[bid][axis] += d

    def restore(self):
        self.model.body_ipos[:] = self.ipos0


# ---------------------------------------------------------------------------
# 同定
# ---------------------------------------------------------------------------

def expand_groups(spec: dict, group_names: list[str]) -> list[list[str]]:
    """'upper_link' → ['upper_link_R', 'upper_link_L'] のように prefix を展開。完全一致があればそれ 1 つ"""
    names = [b["name"] for b in spec["bodies"]]
    out = []
    for g in group_names:
        if g in names:
            out.append([g])
            continue
        hits = [n for n in names if n.startswith(g)]
        if not hits:
            sys.exit(f"error: --bodies '{g}' matches no body in {conv.SPEC_FILENAME} (bodies: {names})")
        out.append(hits)
    return out


def fit(bm: BalanceModel, groups_bids: list[list[int]], poses: list[tuple[float, float, float]]):
    """poses = [(hip, knee, measured_pitch), ...]。各 group の dx を Gauss-Newton で求める"""
    n_u, n_m = len(groups_bids), len(poses)
    if n_u > n_m:
        sys.exit(f"error: {n_u} unknown offsets but only {n_m} measurements — add --at/--pose or reduce --bodies")
    dx = np.zeros(n_u)

    def resid(dx_):
        bm.set_offsets(groups_bids, dx_)
        return np.array([bm.balance_pitch(h, k) - p for h, k, p in poses])

    for it in range(30):
        r = resid(dx)
        if np.linalg.norm(r) < 1e-7:
            break
        eps = 1e-4  # [m]
        J = np.zeros((n_m, n_u))
        for j in range(n_u):
            e = np.zeros(n_u); e[j] = eps
            J[:, j] = (resid(dx + e) - r) / eps
        step, *_ = np.linalg.lstsq(J, -r, rcond=None)
        dx = dx + step
    r = resid(dx)
    bm.restore()
    return dx, r


# ---------------------------------------------------------------------------
# model.json の局所編集 (整形を保ったまま対象 body に com_offset を書く / 消す)
# ---------------------------------------------------------------------------

def _find_body_block(text: str, body_name: str) -> tuple[int, int]:
    """'"name": "<body>"' を含む最も内側の {...} の範囲 (start, end) を返す (end は '}' の次)"""
    m = re.search(r'"name"\s*:\s*"%s"' % re.escape(body_name), text)
    if not m:
        sys.exit(f"error: body '{body_name}' not found in model.json text")
    depth = 0
    i = m.start()
    while i >= 0:
        c = text[i]
        if c == "}":
            depth += 1
        elif c == "{":
            if depth == 0:
                break
            depth -= 1
        i -= 1
    start = i
    depth = 0
    j = start
    while j < len(text):
        c = text[j]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return start, j + 1
        j += 1
    sys.exit("error: unbalanced braces in model.json")


def write_offset(spec_path: Path, body_name: str, dx: float | None, note: str | None):
    text = spec_path.read_text(encoding="utf-8")
    start, end = _find_body_block(text, body_name)
    block = text[start:end]
    block = re.sub(r'\n[ \t]*"_com_offset"\s*:\s*"[^"\n]*",?', "", block)
    block = re.sub(r'\n[ \t]*"com_offset"\s*:\s*\[[^\]\n]*\],?', "", block)
    if dx is not None:
        anchor = re.search(r'\n([ \t]*)"part"\s*:[^\n]*', block) or re.search(r'\n([ \t]*)"name"\s*:[^\n]*', block)
        indent = anchor.group(1)
        insert = (f'\n{indent}"_com_offset": "{note}",'
                  f'\n{indent}"com_offset": [{dx:.6f}, 0.0, 0.0],')
        block = block[:anchor.end()] + insert + block[anchor.end():]
    block = re.sub(r",(\s*\})$", r"\1", block)
    text = text[:start] + block + text[end:]
    spec_path.write_text(text, encoding="utf-8")


# ---------------------------------------------------------------------------

def _parse_pose(s: str) -> tuple[float, float, float]:
    m = re.fullmatch(r"\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*:\s*([-+0-9.eE]+)\s*", s)
    if not m:
        raise argparse.ArgumentTypeError(f"--pose expects HIP,KNEE:PITCH, got {s!r}")
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


def _parse_at(s: str) -> tuple[float, float]:
    m = re.fullmatch(r"\s*([-+0-9.eE]+)\s*:\s*([-+0-9.eE]+)\s*", s)
    if not m:
        raise argparse.ArgumentTypeError(f"--at expects RISE:PITCH, got {s!r}")
    return float(m.group(1)), float(m.group(2))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("standing_pitch", nargs="?", type=float,
                    help="立位 (robot_config initial_position) の実測倒立 pitch [rad] (= angle_pid の kTargetPitch)")
    ap.add_argument("--at", action="append", type=_parse_at, default=[], metavar="RISE:PITCH",
                    help="重心を RISE [m] 上げた姿勢 (stabilizer の posture IK、いまのモデルで換算) での実測 pitch。複数可")
    ap.add_argument("--pose", action="append", type=_parse_pose, default=[], metavar="HIP,KNEE:PITCH",
                    help="関節角 [rad] を直接指定した姿勢での実測 pitch。複数可")
    ap.add_argument("--bodies", default=None,
                    help="x を動かす body 群 (カンマ区切り。prefix は左右に展開: upper_link → upper_link_R/L)。"
                         "既定: 実測 1 つなら base_link、2 つ以上なら base_link,upper_link")
    ap.add_argument("--param-dir", default=str(conv.DEFAULT_PARAM_DIR),
                    help=f"model.json のディレクトリ (default: {conv.DEFAULT_PARAM_DIR.relative_to(REPO_ROOT)})")
    ap.add_argument("--dry-run", action="store_true", help="計算だけして model.json を書かない")
    ap.add_argument("--reset", action="store_true", help="全 body の com_offset を外して CAD の値に戻す")
    ap.add_argument("--no-regen", action="store_true", help="model.json 更新後の XML 再生成をしない")
    args = ap.parse_args(argv)

    param_dir = Path(args.param_dir)
    if not param_dir.is_absolute():
        param_dir = (Path.cwd() / param_dir).resolve() if (Path.cwd() / param_dir).exists() else (REPO_ROOT / args.param_dir).resolve()
    spec_path = param_dir / conv.SPEC_FILENAME
    spec = conv.load_spec(param_dir)
    rc = conv.load_robot_config(spec)
    meshdir_abs = (REPO_ROOT / "sim" / spec.get("meshdir", "meshes/")).resolve()

    if args.reset:
        for b in spec["bodies"]:
            if "com_offset" in b:
                write_offset(spec_path, b["name"], None, None)
                print(f"removed com_offset from {b['name']}")
        if not args.no_regen:
            print("\nregenerating MJCF ...")
            return conv.main([str(param_dir)])
        return 0

    if args.standing_pitch is None:
        ap.error("standing_pitch is required (or use --reset)")

    # --- 実測姿勢の組み立て。--at は「実測時にロボットが使っていたモデル」(= いまの model.json、
    #     補正込み) の IK で関節角に直す
    current = BalanceModel(spec, param_dir, rc, meshdir_abs)
    poses: list[tuple[float, float, float]] = [(current.hip0, current.knee0, args.standing_pitch)]
    labels = ["standing (initial_position)"]
    for rise, pitch in args.at:
        hip, knee = current.ik_for_rise(rise, args.standing_pitch)
        poses.append((hip, knee, pitch))
        labels.append(f"rise {rise:+.3f} m (current model IK)")
    for hip, knee, pitch in args.pose:
        poses.append((hip, knee, pitch))
        labels.append("explicit pose")

    # --- 補正対象 body 群
    group_names = (args.bodies.split(",") if args.bodies
                   else (["base_link"] if len(poses) == 1 else ["base_link", "upper_link"]))
    groups = expand_groups(spec, [g.strip() for g in group_names])

    # --- CAD そのもの (全 com_offset を外す) のモデルで同定
    cad_spec = conv.load_spec(param_dir)
    for b in cad_spec["bodies"]:
        b.pop("com_offset", None)
    bm = BalanceModel(cad_spec, param_dir, rc, meshdir_abs)
    groups_bids = [[bm.body_id(n) for n in g] for g in groups]
    dx, resid = fit(bm, groups_bids, poses)

    print(f"bodies: {', '.join('+'.join(g) for g in groups)}   measurements: {len(poses)}")
    for (hip, knee, pitch), lab in zip(poses, labels):
        bm.restore()
        cad = bm.balance_pitch(hip, knee)
        bm.set_offsets(groups_bids, dx)
        after = bm.balance_pitch(hip, knee)
        h = bm.com_height(hip, knee, pitch)
        print(f"  {lab:<30} hip={hip:+.4f} knee={knee:+.4f}  CoM {h:.4f} m above axle"
              f"  | CAD {cad:+.4f}  measured {pitch:+.4f}  after {after:+.4f} rad (resid {math.degrees(after - pitch):+.3f} deg)")
    bm.restore()
    for g, d in zip(groups, dx):
        print(f"  => com_offset x = {d * 1000:+.2f} mm on {'+'.join(g)}")
    if args.dry_run:
        print("dry-run: model.json not modified")
        return 0

    meas = "; ".join(f"hip {h:+.4f} knee {k:+.4f} → {p:.4f} rad" for h, k, p in poses)
    cmd = f"uv run mjmodel_converter/com_comp.py {args.standing_pitch}" + "".join(
        f" --pose {h:.4f},{k:.4f}:{p}" for h, k, p in poses[1:]) + (f" --bodies {args.bodies}" if args.bodies else "")
    # 先に全 body の古い補正を消してから書く (対象 body 群が変わっても残骸を残さない)
    for b in spec["bodies"]:
        if "com_offset" in b and b["name"] not in {n for g in groups for n in g}:
            write_offset(spec_path, b["name"], None, None)
    for g, d in zip(groups, dx):
        for n in g:
            note = (f"com_comp.py {_dt.date.today().isoformat()}: 実測倒立 pitch ({meas}) に合わせる補正 "
                    f"(x のみ、{'+'.join(g)} に同値)。再計算は `{cmd}`")
            write_offset(spec_path, n, float(d), note)
    print(f"wrote com_offset to {spec_path.relative_to(REPO_ROOT)}")
    if args.at:
        print("note: --at は「実測時のモデル」で関節角に換算した。補正後のモデルで --at を再実行すると別の姿勢に"
              f"なるので、再実行は関節角固定のこちらで: {cmd}")

    if args.no_regen:
        return 0
    print("\nregenerating MJCF ...")
    return conv.main([str(param_dir)])


if __name__ == "__main__":
    sys.exit(main())
