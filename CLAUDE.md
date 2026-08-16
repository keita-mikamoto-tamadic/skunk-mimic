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
- Each node's binary is output to its own directory (`src/cpp/node/<name>/build/<name>`) via `CMAKE_RUNTIME_OUTPUT_DIRECTORY` — the dataflow YAMLs reference those paths.
- C++20, plain CMake; there is no automated test target. `src/cpp/lib/robot_config_test.cpp` is a standalone assert-based test not wired into the build.

### Python

Python uses `uv` workspaces. Two separate workspaces exist:

```bash
cd src/python && uv sync     # runtime nodes (dora-rs, pyarrow, mujoco, ...)
cd scripts && uv sync        # offline analysis (LQR gain calc, A/B matrix calc)
```

The `dora-rs` Python package version must exactly match the dora CLI version (1.0.0-rc1), otherwise you get "message format version mismatch" errors. It is installed from source — `pyproject.toml` points `dora-rs` at `/opt/dora/apis/python/node`, a machine-independent symlink each machine creates once with `sudo ln -s ~/dora /opt/dora` (dora itself stays at `~/dora`). Without the symlink, `uv sync` fails.

### Data-format code generation

`src/data_format/*.json` are the single source of truth for every binary struct exchanged between C++ and Python — one spec file per domain. `tools/gen_data_format.py` picks up each spec and emits a matching pair:

| spec | C++ | Python | structs |
|------|-----|--------|---------|
| `axis_data.json` | `src/cpp/lib/axis_data_format.hpp` | `src/python/lib/axis_data_format.py` | `AxisRef`, `AxisAct`, `SettingsRequest`, `SettingsResult`, `ParamScalars` |
| `sensor_data.json` | `src/cpp/lib/sensor_data_format.hpp` | `src/python/lib/sensor_data_format.py` | `ImuData`, `LatencyData`, `EstimatedState` |

Adding a domain = drop a new `.json` in `src/data_format/` — output names are always `<spec>_format.hpp` / `<spec>_format.py`, no special cases. Omit `"types"` to use the generator's `DEFAULT_TYPES`. `src/cpp/lib/shm_data_format.hpp` is just an aggregator that includes both generated headers — it defines no structs itself.

It runs automatically at CMake configure time; run manually with `python3 tools/gen_data_format.py` from the repo root. Never hand-edit the generated files — edit the JSON and regenerate. Never hand-write a struct format in a node either (that is exactly how `mujoco_backend` silently drifted to a 32 B `AxisAct` / 56 B `AxisRef`); import from the generated module and use named fields, not positional unpacking.

## Running

```bash
dora up                              # start daemon
dora start dataflow.yaml             # launch a dataflow
dora list                            # list running dataflows
dora logs <dataflow-id> <node-id>    # node logs
dora stop <dataflow-id>
dora destroy                         # stop coordinator & daemon
```

Nodes with `path: dynamic` in the YAML (dummy_input, data_viewer, foctive_controller, ...) are launched manually in separate terminals:

```bash
cd src/python && uv run dummy_input/dummy_input.py   # state commands: on/ready/run/stop/off/reset/q
cd src/python && uv run data_viewer/data_viewer.py   # live motor/IMU/state display (incl. cur_d/cur_q)
```

**Web motor monitor** (`tools/gui/web_monitor.py`, stdlib-only): a browser version of `data_viewer` that is **dataflow-independent**. It is *not* a dora node — it shells out to `dora topic echo -d <auto-discovered dataflow> device_control_manager/motor_status --format json`, decodes the raw `AxisAct` bytes with `lib.data_format`, and serves JSON over HTTP/SSE to `web_monitor.html` (`http://<host>:8765/`). Works for any robot (foctive/mimic/...) since it just taps the topic; auto-rediscovers the running dataflow via `dora list`. Requires `_unstable_debug.enable_debug_inspection: true` on the dataflow (already set in `dataflow_foctive_control.yaml`). Runs on the PC or robot — `dora topic echo` reaches the coordinator over WebSocket (`DORA_COORDINATOR_ADDR`). See `tools/gui/README.md`.

**Web controller** (`src/python/web_controller/`): the input counterpart — a browser version of `foctive_controller` that sends `motor_commands` (`AxisRef`) from the PC. Unlike the monitor, *sending* binary commands requires being a real dora node (`dora topic pub` only carries JSON-as-UInt8, not struct bytes), so `web_controller` is wired into a dataflow and deployed to a PC-side daemon via `_unstable_deploy: machine: <pc>` (start-time deploy is honored; `dora node add` is not). Started distributed: a PC daemon joins the robot's coordinator, `dora start dataflow_foctive_web_control.yaml`, then `uv run web_controller/web_controller.py` on the PC. Motor-command scope only (voltage/current/velocity/position/off); settings round-trips stay in the terminal `foctive_controller`. See `src/python/web_controller/README.md`.

Dataflows (repo root):

| YAML | Purpose | Device side | Control side |
|------|---------|-------------|--------------|
| `dataflow.yaml` | Real robot | `device_control_manager` (C++) | `stabilizer` (C++) |
| `dataflow_sim.yaml` | MuJoCo simulation | `mujoco_backend` (Python) | `stabilizer` (C++) |
| `dataflow_sysid.yaml` | Real-robot SysID | `device_control_manager` | `sysid_controller` (Python) |
| `dataflow_sim_sysid.yaml` | Sim SysID | `mujoco_backend` | `sysid_controller` (Python) |
| `dataflow_foctive_control.yaml` | Single-motor test (FOCTIVE by default; point `ROBOT_CONFIG` at `moteus_motor_test.json` for a moteus motor) | `device_control_manager` | `foctive_controller` (Python, bypasses the state machine) |
| `dataflow_foctive_web_control.yaml` | FOCTIVE test driven from a browser | `device_control_manager` (robot) | `web_controller` (Python, deployed to a PC daemon via `_unstable_deploy`) |
| `dataflow_mimic_web_control.yaml` | mimic 6-axis / 2-CAN-channel test from a browser (`mimic_v2_5.json`: can0 = right leg, can1 = left leg) | `device_control_manager` (robot) | `web_controller` (multi-axis: per-axis selector + all-axes OFF) |

Hardware modes: `bash can_setup.bash` brings up the Tegra built-in CAN; `bash vcan_setup.bash` for virtual CAN (see HowToSocketcan.md). Set `"transport": "dummy"` in the robot config to run with no hardware at all.

**PEAK PCAN-M.2 (4ch CAN-FD):** exposed as extra SocketCAN netdevs via an out-of-tree `pcan.ko` built in NETDEV mode (setup + rebuild-after-kernel-update caveats in `docs/pcan_socketcan_driver.md`). Bring channels up with `bash pcan_setup.bash <canX> ...` — by default it holds the interfaces UP and DOWNs them on Ctrl-C (pass `keep` to leave them up, `down`/`all down` to tear down). Which `canX` is PEAK vs Tegra built-in is machine-dependent — check `/proc/pcan`, `dmesg | grep -i pcan`, or `candump`; the header comment in `pcan_setup.bash` records the current machine's mapping. The robot config's `comm_ch` list selects which netdev(s) `device_control_manager` opens — one driver instance per channel, with each axis assigned to a channel by its per-axis `comm_ch` index (e.g. `mimic_v2_5.json`: can0 = right leg, can1 = left leg).

**Real-time scheduling:** the daemon (`--rt`) and `device_control_manager` use `SCHED_FIFO` + `mlockall`. Grant the permission via `/etc/security/limits.d` (`rtprio 90` / `memlock unlimited` for the user; see `docs/rt-enable.md`) — **do not use `setcap`** (lost on every rebuild, and capability-bearing execution is secure-exec, which hurts debugging and confused the zenoh peer-link diagnosis). DCM isolates its CPU via the dataflow yaml `cpu_affinity: [1]` and sets SCHED_FIFO on its main thread only, *after* `init_dora_node()` (setting it before makes the zenoh workers FIFO too and the node falls back to daemon relay). `dora_rt_damon.bash` launches a coordinator + `--rt` daemon; pass `distributed` only for the PC-deployed dataflows (sets `ZENOH_CONFIG`). Further docs live in `docs/` (`rt-enable.md`, `dora-upgrade.md`, `robot_start.md`).

## Architecture

See README_ARCH.md for the full node graph, struct layouts, and Python binary formats.

**Node graph (real robot):** `dummy_input` → `robot_control_manager` (state machine: OFF/STOP/READY/RUN, emits motor_commands) → `device_control_manager` (3 ms tick, CAN I/O, IMU fusion) → motor_status/imu_data back to `robot_control_manager`, `stabilizer`, and `data_viewer`. `stabilizer` computes run_command from motor_status + imu_data and feeds it to `robot_control_manager`.

**Configuration:** `robot_config/*.json` selected via the `ROBOT_CONFIG` env var (defaults exist per node; dataflow YAMLs set it under `env:` for spawned nodes — dynamic nodes need it exported in their shell). Key fields: `transport` (`socketcan` | `dummy`), `protocol` (`moteus` default | `foctive`), `controller` (e.g. `angle_pid`, `lqr`), `comm_ch` (ordered list of SocketCAN netdevs, default `["can0"]`; each axis picks one via its `comm_ch` index, default 0 — `device_control_manager` opens one driver instance per channel; device_ids must be globally unique across channels), and per-axis limits/IDs.

**Two extension seams:**

1. **Motor drivers** — `device_control_manager` talks to hardware through the `MotorDriver` interface (`src/cpp/interface/motor_driver.hpp`). Implementations live in `src/cpp/driver/`: `MoteusCanDriver` (CAN-FD + moteus), `FoctiveCanDriver`, `DummyDriver`. Selection happens in `CreateDriver()` in `src/cpp/node/device_control_manager/main.cpp` based on the config's `transport`/`protocol`. To add a new bus or protocol, implement `MotorDriver` and add a branch there.
2. **Controllers** — `stabilizer` swaps control algorithms via the `Controller` interface (`src/cpp/node/stabilizer/controller.hpp`), selected by the config's `controller` field.

To replace either side with Python (as `mujoco_backend` and `sysid_controller` do), swap the whole dora node — match the input/output names and byte formats from the generated data format (`AxisRef` 72 B commands, `AxisAct` 48 B status, `ImuData` 112 B). The struct sizes are enforced by `static_assert`s in `axis_data_format.hpp` / `sensor_data_format.hpp`; treat the generated files (not README_ARCH.md) as the source of truth for current layouts.

**Shared C++ helpers** live in `src/cpp/lib/` (robot_config parsing, dora helpers, PID, FOCTIVE protocol/param definitions, enums shared with Python).
