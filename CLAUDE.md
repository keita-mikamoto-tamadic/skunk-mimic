# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

skunk-mimic is a unified robot controller for a wheeled biped (mimic_v2) built on the [dora-rs](https://dora-rs.ai/) dataflow framework. C++ nodes handle real-time control and CAN communication; Python nodes handle simulation, system identification, and tooling. Nodes exchange raw byte structs over Apache Arrow. Documentation and code comments are primarily in Japanese.

## Build Commands

### C++ nodes

```bash
git submodule update --init --recursive   # first checkout: fetches OpenKF (header-only EKF)
cd src/cpp && mkdir -p build && cd build && cmake .. && make
```

- `src/cpp/lib/vendor/OpenKF` is a git submodule (header-only Kalman-filter library) used by `stabilizer`'s `body_state_ekf`. A fresh clone without `submodule update` fails to configure.
- Requires dora built from source at `~/dora` (v1.0.0-rc1): `libdora_node_api_cxx.a` (`~/dora/target/release/`) and the cxxbridge headers (`~/dora/target/cxxbridge/dora-node-api-cxx/install`; see README.md for the full setup). Also requires `libarrow-dev`.
- `stabilizer` additionally needs **Pinocchio ≥ 3 (C++)** for `posture_ik` (it parses the MuJoCo MJCF directly). The expected install is the pip wheel `pin` (cmeel build): `pip install pin` drops the C++ libs/headers/CMake config under `~/.local/lib/python3.*/site-packages/cmeel.prefix`, which the stabilizer CMakeLists adds to `CMAKE_PREFIX_PATH` automatically (the binary gets a RUNPATH into it). Same on the Jetson. Any other install (robotpkg, source) works via the normal `find_package(pinocchio)`; pass `-DCMAKE_PREFIX_PATH=<prefix>` if it is somewhere else.
- Each node's binary is output to its own directory (`src/cpp/node/<name>/build/<name>`) via `CMAKE_RUNTIME_OUTPUT_DIRECTORY` — the dataflow YAMLs reference those paths.
- C++20, plain CMake; there is no automated test target. `src/cpp/lib/robot_config_test.cpp` is a standalone assert-based test not wired into the build.

### Python

Python uses `uv` workspaces. Two separate workspaces exist:

```bash
cd src/python && uv sync     # runtime nodes (dora-rs, pyarrow, mujoco, ...)
cd scripts && uv sync        # offline analysis (LQR gain calc, A/B matrix calc)
```

**MuJoCo model generation:** `sim/mimic_v2_5.xml` is generated, not hand-edited — `cd scripts && uv run mjmodel_converter/mjmodel_converter.py` rebuilds it from `sim/fusion_param/mimic_v2_5/` (Fusion 360 physical-property CSVs for mass/CoM/inertia + `model.json` for the skeleton: parent/child, joint offsets/axes, meshes, collision shapes, moteus gains) and `robot_config/mimic_v2_5.json` (axis names/order, `initial_position` → keyframe `standing`, `torque_limit` → forcerange). Joint offsets are **not** in the CSVs; they live in `model.json` (v2_5 shares v2's kinematics, meshes and collision shapes — only mass/CoM/inertia changed). `sim/mimic_v2.xml` predates the converter and is still hand-maintained. See `scripts/mjmodel_converter/README.md`. Simulate with `dataflow_mimic_sim.yaml` + `MUJOCO_MODEL=scene.xml`, not the v2-era `dataflow_sim.yaml`; `sim/scene.xml` is hand-maintained (not generated) — scene-level things (floor, lights, cameras) go there, not into the converter. `scripts/mjmodel_converter/com_comp.py <measured_pitch_rad>` matches the model's balance point to the real robot (writes `com_offset` on `base_link` into `model.json`, then regenerates) — re-run it after any CAD/CSV update.

The `dora-rs` Python package version must exactly match the dora CLI version (1.0.0-rc1), otherwise you get "message format version mismatch" errors. It is installed from source — `pyproject.toml` points `dora-rs` at `/opt/dora/apis/python/node`, a machine-independent symlink each machine creates once with `sudo ln -s ~/dora /opt/dora` (dora itself stays at `~/dora`). Without the symlink, `uv sync` fails.

### Data-format code generation

`src/data_format/*.json` are the single source of truth for every binary struct exchanged between C++ and Python — one spec file per domain. `tools/gen_data_format.py` picks up each spec and emits a matching pair:

| spec | C++ | Python | structs |
|------|-----|--------|---------|
| `axis_data.json` | `src/cpp/lib/axis_data_format.hpp` | `src/python/lib/axis_data_format.py` | `AxisRef`, `AxisAct`, `SettingsRequest`, `SettingsResult`, `ParamScalars` |
| `sensor_data.json` | `src/cpp/lib/sensor_data_format.hpp` | `src/python/lib/sensor_data_format.py` | `ImuData`, `LatencyData`, `EstimatedState` |
| `command_data.json` | `src/cpp/lib/command_data_format.hpp` | `src/python/lib/command_data_format.py` | `DriveCommand` (operator input → stabilizer) |

Adding a domain = drop a new `.json` in `src/data_format/` — output names are always `<spec>_format.hpp` / `<spec>_format.py`, no special cases. Omit `"types"` to use the generator's `DEFAULT_TYPES`. `src/cpp/lib/shm_data_format.hpp` is just an aggregator that includes the generated headers — it defines no structs itself.

It runs automatically at CMake configure time; run manually with `python3 tools/gen_data_format.py` from the repo root. Never hand-edit the generated files — edit the JSON and regenerate. Never hand-write a struct format in a node either (that is exactly how `mujoco_backend` silently drifted to a 32 B `AxisAct` / 56 B `AxisRef`); import from the generated module and use named fields, not positional unpacking.

## Running

```bash
dora up                              # start daemon
dora start dataflow.yaml             # launch a dataflow
dora list                            # list running dataflows
dora logs <dataflow-id> --node <node-id>   # node logs (1.0.0-rc1: positional node id no longer accepted)
dora stop <dataflow-id>
dora destroy                         # stop coordinator & daemon
```

Nodes with `path: dynamic` in the YAML (dummy_input, robot_web_gui, foctive_controller, ...) are launched manually in separate terminals. **A dataflow blocks at the start barrier until every dynamic node has attached**, so only put a node in the YAML if it will always be started — pull-type viewers are deliberately left out.

```bash
cd src/python && uv run dummy_input/dummy_input.py         # state commands: on/ready/run/stop/off/reset/q
cd src/python && uv run auto_input/auto_input.py           # scripted SERVO_ON → READY → RUN → STOP
cd src/python && uv run dualsense_input/dualsense_input.py # DualSense pad: state_command + drive_command (current dataflow_mimic input)
cd src/python && uv run data_viewer/data_viewer.py         # node-type viewer (wired in the YAML)
```

**Operator input is exclusive.** `dataflow_mimic.yaml` takes `state_command` from exactly one source (dora allows one source per input name): currently `dualsense_input`, with the `robot_web_gui` block commented out. Switching back means flipping *both* node blocks **and** the RCM `state_command:` line **and** removing the `drive_command:` input on `stabilizer` (a dangling source is a start-time error). `dualsense_input` reads the pad via the plain joystick API (`/dev/input/js0`, `hid-generic` + `joydev`; no `hid-playstation` on this kernel), starts without a pad and survives disconnects so it never blocks the start barrier; on disconnect it keeps sending `drive_command = 0` but does *not* send `state_command`. `scripts/dualsense_monitor/dualsense_monitor.py` (stdlib, not a dora node) shows raw pad input and its README records the axis/button mapping.

**Two kinds of viewer.** `data_viewer/data_viewer.py` is a real dora node wired into the YAML (costs CPU just receiving, and holds the start barrier). The preferred day-to-day tools are the **pull-type** ones — not dora nodes, they shell out to `dora topic echo` and decode the raw struct bytes with `src/python/lib/*_format.py`, so they can be started and Ctrl-C'd at any time without touching the dataflow. Both need `_unstable_debug.enable_debug_inspection: true` on the dataflow (already set in `dataflow_mimic.yaml` / `dataflow_foctive_control.yaml`) and the `dora` CLI:

- `src/python/data_viewer.py` — terminal viewer on the robot, stdlib only (no `uv` needed).
- `tools/gui/web_monitor.py` — browser version (`http://<host>:8765/`), stdlib only. Runs on the PC or robot (`dora topic echo` reaches the coordinator over WebSocket via `DORA_COORDINATOR_ADDR`); auto-rediscovers the running dataflow via `dora list`, so it works for any robot config. See `tools/gui/README.md`.

**Two browser input GUIs** (sending binary commands requires being a real dora node — `dora topic pub` only carries JSON-as-UInt8, not struct bytes, so unlike the pull-type viewers these are wired into the YAML):

- `src/python/robot_web_gui/` (`:8766`) — browser version of `dummy_input`: sends `state_command` and goes **through** the RCM state machine. This is the normal-operation GUI (`dataflow_mimic.yaml`). It runs **on the robot** — it was previously deployed to a PC daemon via `_unstable_deploy`, and that was reverted because `state_command` (including SERVO_OFF) could stall for seconds over zenoh; the PC now just opens `http://<robot-ip>:8766/`. See its README for the button/RCM-guard table.
- `src/python/web_controller/` (`:8765`-side counterpart to the monitor) — browser version of `foctive_controller`: sends `motor_commands` (`AxisRef`) **directly**, bypassing the state machine. For single-axis tests and zero-point work. This one *is* PC-deployed via `_unstable_deploy: machine: <pc>` (start-time deploy is honored; `dora node add` is not): start a PC daemon with `bash dora_pc_daemon.bash <robot-ip>`, `dora start dataflow_*_web_control.yaml` on the robot, then `uv run web_controller/web_controller.py` on the PC. Motor-command scope only; settings round-trips stay in the terminal `foctive_controller`.

Dataflows (repo root):

| YAML | Purpose | Device side | Control side |
|------|---------|-------------|--------------|
| `dataflow_mimic.yaml` | **mimic_v2_5 real robot (current main flow)** — robot-local, `mimic_v2_5.json`, 2 CAN channels; startup sequence in `docs/robot_start.md` | `device_control_manager` + `imu_node` (C++) | `stabilizer` (C++), state + drive from `dualsense_input` (or `robot_web_gui`, see above) |
| `dataflow_mimic_sim.yaml` | **MuJoCo sim of `dataflow_mimic.yaml`** — same control side and `mimic_v2_5.json`, DCM + imu_node swapped for `mujoco_backend`; run the backend with `MUJOCO_MODEL=scene.xml` (hand-written `sim/scene.xml` = infinite grid floor + lights + cameras, `<include>`s the generated `mimic_v2_5.xml`, which has **no floor** of its own; the backend defaults to the v2 model) | `mujoco_backend` (Python, dynamic) | `stabilizer` (C++), `dualsense_input` |
| `dataflow.yaml` | Real robot (v2, legacy) | `device_control_manager` (C++) | `stabilizer` (C++) |
| `dataflow_sim.yaml` | MuJoCo simulation (v2-era: `mimic_v2.json`, `dummy_input`, 30 ms watchdog — not the current flow) | `mujoco_backend` (Python) | `stabilizer` (C++) |
| `dataflow_sysid.yaml` | Real-robot SysID | `device_control_manager` | `sysid_controller` (Python) |
| `dataflow_sim_sysid.yaml` | Sim SysID | `mujoco_backend` | `sysid_controller` (Python) |
| `dataflow_foctive_control.yaml` | Single-motor test (FOCTIVE by default; point `ROBOT_CONFIG` at `moteus_motor_test.json` for a moteus motor) | `device_control_manager` | `foctive_controller` (Python, bypasses the state machine) |
| `dataflow_foctive_web_control.yaml` | FOCTIVE test driven from a browser | `device_control_manager` (robot) | `web_controller` (Python, deployed to a PC daemon via `_unstable_deploy`) |
| `dataflow_mimic_web_control.yaml` | mimic 6-axis / 2-CAN-channel test from a browser (`mimic_v2_5.json`: can0 = right leg, can1 = left leg) | `device_control_manager` (robot) | `web_controller` (multi-axis: per-axis selector + all-axes OFF) |

Hardware modes: `bash can_setup.bash` brings up the Tegra built-in CAN; `bash vcan_setup.bash` for virtual CAN (see HowToSocketcan.md). Set `"transport": "dummy"` in the robot config to run with no hardware at all.

**PEAK PCAN-M.2 (4ch CAN-FD):** exposed as extra SocketCAN netdevs via an out-of-tree `pcan.ko` built in NETDEV mode (setup + rebuild-after-kernel-update caveats in `docs/pcan_socketcan_driver.md`). Bring channels up with `bash pcan_setup.bash <canX> ...` — by default it holds the interfaces UP and DOWNs them on Ctrl-C (pass `keep` to leave them up, `down`/`all down` to tear down). Which `canX` is PEAK vs Tegra built-in is machine-dependent — check `/proc/pcan`, `dmesg | grep -i pcan`, or `candump`; the header comment in `pcan_setup.bash` records the current machine's mapping. The robot config's `comm_ch` list selects which netdev(s) `device_control_manager` opens — one driver instance per channel, with each axis assigned to a channel by its per-axis `comm_ch` index (e.g. `mimic_v2_5.json`: can0 = right leg, can1 = left leg).

**IMU (Spresense over serial):** `imu_node` opens `/dev/ttyUSB0` at 921600 and publishes `raw_imu`; `device_control_manager` re-sends it as `imu_data`. The user must be in the `dialout` group (re-login required) — no chmod/udev rule needed. Run `bash keep_serial_open.bash` in its own terminal before starting the dataflow: it sets `921600 raw -hupcl` and holds the port open, because the Spresense goes silent once the port is fully closed (and `-hupcl` keeps closing from resetting the board). The mount orientation comes from the config's `imu_mount_rpy_deg` (`[180,0,0]` on v2_5), applied in `imu_node`, not downstream.

**Real-time scheduling:** the robot runs NVIDIA's **PREEMPT_RT kernel** (`6.8.12-1021-rt-tegra`, installed from the separate `jetson/rt-kernel` apt repo — the stock JetPack kernel is only `CONFIG_PREEMPT`). `docs/rt-enable.md` §0 has the install, the `extlinux.conf` rollback path, and the trap that **`pcan.ko` must be rebuilt + `make install`ed + rebooted** after any kernel change (a bare `insmod` leaves the CAN interfaces renumbered and `modprobe.d` options unapplied). Measured with `rtla timerlat`: an RT thread wakes within ~35 µs on any core, a `SCHED_OTHER` thread within ~2.3 ms — so **any non-RT thread in the 3 ms closed loop costs milliseconds**, which is why the loop is moving to shm outside dora (`docs/rt_meas.md` §2b). The daemon (`--rt`) and `device_control_manager` use `SCHED_FIFO` + `mlockall`. Grant the permission via `/etc/security/limits.d` (`rtprio 90` / `memlock unlimited` for the user; kernel-independent, so no re-setup after a kernel switch; see `docs/rt-enable.md`) — **do not use `setcap`** (lost on every rebuild, and capability-bearing execution is secure-exec, which hurts debugging and confused the zenoh peer-link diagnosis). DCM isolates its CPU via the dataflow yaml `cpu_affinity: [1]` and sets SCHED_FIFO on its main thread only, *after* `init_dora_node()` (setting it before makes the zenoh workers FIFO too and the node falls back to daemon relay). `dora_rt_damon.bash` launches a coordinator + `--rt` daemon (and warns if an UP CAN interface is already ERROR-PASSIVE); pass `distributed` only for the PC-deployed dataflows (sets `ZENOH_CONFIG`). `docs/rt_meas.md` documents how loop latency/jitter is actually measured (the RCM `motor_status rx:` log line and the DCM `latency` topic) and what has already been ruled out — read it before re-investigating jitter. Further docs live in `docs/`: `robot_start.md` (full v2_5 startup checklist), `rt-enable.md`, `pcan_socketcan_driver.md`, `dora-upgrade.md`, `web_gui_260710.md`.

## Architecture

See README_ARCH.md for the full node graph, struct layouts, and Python binary formats.

**Node graph (real robot):** `dummy_input` / `robot_web_gui` / `dualsense_input` → `robot_control_manager` (state machine: OFF/STOP/READY/RUN, emits motor_commands) → `device_control_manager` (3 ms tick, CAN I/O) → motor_status/imu_data back to `robot_control_manager` and `stabilizer`. `imu_node` (3 ms tick, serial) → raw_imu → DCM, which re-sends it as `imu_data`. `stabilizer` computes run_command from motor_status + imu_data and feeds it to `robot_control_manager`. `dualsense_input` additionally sends `drive_command` (`DriveCommand`: normalized forward/yaw/height in -1..1 at ~50 Hz) straight to `stabilizer`, which zeroes it after 300 ms of silence and feeds forward/yaw into the *outer* loop's target velocity (`angle_pid`'s `kMaxDriveWheelVel` / `kMaxYawWheelDiff` own the physical scaling — adding to the PID output instead is cancelled by the outer loop).

**Posture (CoM height) control** — `height` (right stick) is a *rate*; `angle_pid` integrates it into a target CoM height and `stabilizer/posture_ik` (Pinocchio) solves hip/knee (left/right symmetric) each 3 ms tick so that, at `kTargetPitch`, the whole-body CoM stays directly above the wheel axle (balance point unchanged) while its height tracks the command. Moving the *base* straight up is not enough — roughly half the mass is in the legs and shifts fore/aft with the knee — so the IK constrains the CoM, not the base. It needs `model_mjcf` (repo-relative path to the generated `sim/<model>.xml`, so sim and controller share one mass model incl. the `com_comp` correction) in the robot config; without it hip/knee stay at `initial_position` as before. **MJCF joint names = robot_config axis names** (the converter names actuated joints after `body.axis`), so neither the stabilizer nor `mujoco_backend` carries a joint-name table (`mujoco_backend` walks actuator → joint, which also works with the old `mimic_v2.xml`). Range: `[h0, h0 + kMaxComRise]` (0.08 m on v2_5 = hip hits its 0.6 rad limit) plus the MJCF joint limits; `kMaxComHeightVel` = 0.03 m/s. Verified in sim against a MuJoCo finite-difference reference (identical solutions, 45 µs/step).

Two deliberate omissions in `dataflow_mimic.yaml` versus the older `dataflow.yaml`, both undone on purpose — don't "fix" them without reading the YAML comments: there is **no 30 ms RCM watchdog** (moteus's own 100 ms watchdog already stops the motors, and delivery jitter made the RCM one fire spuriously into SERVO_OFF), and `robot_web_gui` is **not** PC-deployed.

**Configuration:** `robot_config/*.json` selected via the `ROBOT_CONFIG` env var (defaults exist per node; dataflow YAMLs set it under `env:` for spawned nodes — dynamic nodes need it exported in their shell). Key fields: `transport` (`socketcan` | `dummy`), `protocol` (`moteus` default | `foctive`), `controller` (e.g. `angle_pid`, `lqr`), `comm_ch` (ordered list of SocketCAN netdevs, default `["can0"]`; each axis picks one via its `comm_ch` index, default 0 — `device_control_manager` opens one driver instance per channel; device_ids must be globally unique across channels), and per-axis limits/IDs.

**Two extension seams:**

1. **Motor drivers** — `device_control_manager` talks to hardware through the `MotorDriver` interface (`src/cpp/interface/motor_driver.hpp`). Implementations live in `src/cpp/driver/`: `MoteusCanDriver` (CAN-FD + moteus), `FoctiveCanDriver`, `DummyDriver`. Selection happens in `CreateDriver()` in `src/cpp/node/device_control_manager/main.cpp` based on the config's `transport`/`protocol`. To add a new bus or protocol, implement `MotorDriver` and add a branch there.
2. **Controllers** — `stabilizer` swaps control algorithms via the `Controller` interface (`src/cpp/node/stabilizer/controller.hpp`), selected by the config's `controller` field.

To replace either side with Python (as `mujoco_backend` and `sysid_controller` do), swap the whole dora node — match the input/output names and byte formats from the generated data format (`AxisRef` 72 B commands, `AxisAct` 48 B status, `ImuData` 112 B). The struct sizes are enforced by `static_assert`s in `axis_data_format.hpp` / `sensor_data_format.hpp`; treat the generated files (not README_ARCH.md) as the source of truth for current layouts.

**Shared C++ helpers** live in `src/cpp/lib/` (robot_config parsing, dora helpers, PID, IMU mount rotation, moteus fault decoding, FOCTIVE protocol/param definitions, enums shared with Python). Shared Python helpers live in `src/python/lib/` — the generated format modules plus enums; every Python node imports the structs from there.

**SysID data** is captured by `src/python/data_recorder/` (records motor_commands / motor_status / imu_data to `.npz` under `sysid/data/`, flushed on SIGINT or dora shutdown) and processed offline by the `scripts/` workspace (`calc_ab_matrix`, `calc_lqr_gain`), with results in `sysid/results/`.
