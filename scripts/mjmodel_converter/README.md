# mjmodel_converter

Fusion 360 の物理プロパティ CSV + 骨格定義 JSON から MuJoCo モデル (`sim/<model>.xml`) を生成する。
`sim/mimic_v2.xml` (手作業で URDF から変換したもの) と同じ構造の MJCF を、CAD の値から再生成できるようにしたもの。

```bash
cd scripts && uv sync
uv run mjmodel_converter/mjmodel_converter.py                       # sim/fusion_param/mimic_v2_5 → sim/mimic_v2_5.xml
uv run mjmodel_converter/mjmodel_converter.py sim/fusion_param/<robot> [-o sim/<name>.xml] [--no-validate]
```

生成後に MuJoCo で読み込んで body 数・質量・keyframe を表示する (`--no-validate` で省略)。
シミュレーションで使うには `dataflow_mimic_sim.yaml` (実機 `dataflow_mimic.yaml` のシム版) を起動し、`mujoco_backend` を `MUJOCO_MODEL=scene.xml` 付きで立ち上げる (手順は yaml 冒頭コメント)。`sim/scene.xml` は**手書き**で、床 (無限グリッド)・光源・カメラを持ち `mimic_v2_5.xml` を `<include>` する。生成側 (ロボット XML) は床・光源・カメラを一切持たない (単体で読むと床が無い)。

## 入力

1 ロボット = 1 ディレクトリ (`sim/fusion_param/<robot>/`)。

| ファイル | 役割 |
|---|---|
| `<part>.csv` | Fusion 360 の「物理」タブ書き出し (タブ区切り)。質量・重心・**重心まわり慣性テンソル**を使う。`Mass`, `Center of Mass`, `Moment of Inertia at Center of Mass (g mm^2)` を読む。`at Origin` ブロックは検証にだけ使う |
| `model.json` | CSV に無い情報: body の親子関係・関節位置 (`pos`) / 軸 / 可動域 / armature / damping・メッシュ名・衝突形状・IMU site・センサ・moteus 利得。各 body の `"part"` が `<part>.csv` を指す |
| `robot_config/*.json` | `model.json` の `"robot_config"` が指す。**軸名と順序** (actuator 名・順番)、`initial_position` (keyframe `standing`)、`torque_limit` (forcerange) の正本。二重管理しない |

### 単位・座標系の前提

- CSV は g / mm / g·mm² → kg / m / kg·m² に変換する (他の単位表記も `MASS_UNIT` 等に追加すれば読める)。
- `Center of Mass` は Fusion の**部品原点**基準。MuJoCo の body 原点 = 関節の回転中心なので、**Fusion 側で部品原点を関節中心に置いておく**こと (v2 以来の前提。`wheel` の CoM が車軸上に乗っているかで確認できる)。
- 慣性テンソルの非対角成分 (Ixy, Ixz, Iyz) は Fusion の値を**そのまま** `fullinertia` に渡す。Fusion の出力はテンソル成分そのもの (−∫xy dm) で、スクリプトが `at Origin` ブロックと平行軸の定理で突き合わせて検証する。規約が違えば `parallel-axis mismatch` 警告が出る (mimic_v2_5 の CSV では全部一致)。
- 関節位置 (`pos`) は CSV に含まれない。`model.json` に書く。mimic_v2_5 は関節オフセット・メッシュ・衝突形状が v2 と同一で、変わったのは質量・重心・慣性 (CSV 側) のみ。

### `model.json` の主なキー

```jsonc
{
  "model": "mimic_v2_5",                        // 出力 sim/<model>.xml と <mujoco model=...>
  "robot_config": "robot_config/mimic_v2_5.json",
  "meshdir": "meshes/", "timestep": 0.001,
  "bodies": [                                   // 親が先に来る順で並べる (qpos の並びもこの順)
    {"name": "base_link", "part": "MIM2_5-A000", "mesh": "base_link", "parent": null,
     "pos": [0,0,0.5], "joint": {"type": "free"}, "collision": {"type": "mesh"},
     "sites": [{"name": "imu", "size": "0.01", "rgba": "1 0 0 1", "type": "box"}]},
    {"name": "wheel_R", "part": "MIM2_5-A300", "mesh": "wheel_R", "parent": "lower_link_R",
     "pos": [0,-0.04985,-0.18],
     "joint": {"name": "wheel_R_joint", "axis": [0,1,0], "armature": 0.008361, "damping": 0.047421},
     "axis": "wheel_r",                         // robot_config の軸名 → actuator / keyframe の対応付け
     "collision": {"type": "cylinder", "radius": 0.07795, "half_length": 0.024361, "euler": [1.5708,0,0]}}
  ],
  "contact_exclude": [["upper_link_L","wheel_L"], ["upper_link_R","wheel_R"]],
  "sensors": [{"tag": "gyro", "name": "gyro", "site": "imu"}, ...],   // tag = MJCF 要素名、残りは属性
  "actuators": {
    "moteus_gains_per_rev": {"hip_pitch_r": {"kp": 600, "kd": 6.0}, "wheel_r": {"kp": 140, "kd": 0.5}, ...},
    "initial_mode": {"hip_pitch_r": "position", "wheel_r": "velocity", ...}
  }
}
```

`_` で始まるキーはコメント扱い。`collision` は `mesh` / `cylinder` / `sphere` / `null` (無し)。
`cylinder` 衝突を持つ body は接地体 (ホイール) とみなし、keyframe `standing` の base 高さを
「その姿勢で円筒の最下点が z=0」になるよう順運動学で決める (`base_height_margin` で上乗せ可)。

## 倒立点の補正 (`com_comp.py`)

CAD の質量特性だけだと、配線・バッテリー・搭載物など CAD に無い質量のぶん倒立点 (standing 姿勢で
全体重心が車軸の真上に来る base pitch) が実機とずれる。実機の倒立点 (初期姿勢で平衡する pitch の実測
= angle_pid の `kTargetPitch`) を正とし、`base_link` の重心を body 座標 **x 方向にだけ**動かして合わせる:

```bash
cd scripts
uv run mjmodel_converter/com_comp.py 0.165            # 実測倒立 pitch [rad] → model.json に com_offset を書き、XML を再生成
uv run mjmodel_converter/com_comp.py 0.165 --dry-run  # 計算だけ
uv run mjmodel_converter/com_comp.py --reset          # 補正を外して CAD の値に戻す
```

- `model.json` の該当 body に `"com_offset": [dx, 0, 0]` と根拠コメント `"_com_offset"` が書かれる
  (他の行・整形は触らない。再実行は冪等、`--reset` で元のファイルに戻る)。converter はこれを CSV の重心に足すだけ。
- converter は毎回 `balance pitch at standing pose` を表示するので、CAD 更新後に補正量が不自然に膨らんでいないか分かる。
- 1 姿勢の pitch 1 個からはスカラー 1 個しか同定できないので、補正は 1 body・1 方向に限る。IMU 取り付け角や
  関節原点のズレもこの補正に吸収される (standing 姿勢では一致するが、他の姿勢での一致は保証しない)。
- pitch の符号は `mujoco_backend.quat_to_euler` (= stabilizer が見る `ImuData.pitch`) と同じで、`kTargetPitch` をそのまま渡せる。
- mimic_v2_5 (2026-08-20): CAD 倒立点 +0.1416 rad (8.11°) → 実測 +0.165 rad (9.45°)、差 1.34° ≒ 全体重心 2.3 mm、
  `base_link` x **−7.12 mm** (CAD +9.7 mm → +2.6 mm)。

## 出力の互換性

`mujoco_backend` が参照する名前をそのまま出す: joint 名 (`upper_link_R_joint` …)、actuator 名 (= robot_config 軸名)、
keyframe `standing`、センサ `gyro` / `accel` / `linvel` / `imu_pos` / `imu_quat`。
actuator は `general` で、`mujoco_backend` が `biasprm` から `base_kp = -biasprm[1]`, `base_kv = -biasprm[2]` を読んで
実行時に POSITION / VELOCITY / TORQUE / OFF を切り替える。`initial_mode: velocity` (ホイール) は v2 と同様 `base_kp = 0` になる。

## sysid 版 (`mimic_v2_sysid.xml` 相当) について

未対応。free joint 無し + jointpos/jointvel センサの変種が要るときは、生成した XML をもとに作るか、
`model.json` に `"joint": {"type": "free"}` を外した variant を書く (センサは `sensors` に `jointpos` / `jointvel` を列挙)。
