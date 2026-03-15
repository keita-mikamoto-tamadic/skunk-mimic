# robot_control_manager

6軸ロボットのステートマシンと安全管理を担う Dora ノード。
外部ノードから RUN モードの制御コマンドを注入（パススルー）できる設計が特徴。

## ステートマシン

```
OFF ──SERVO_ON──▶ STOP ──READY──▶ READY ──RUN──▶ RUN
 ▲                 ▲                               │
 │                 └──────────STOP──────────────────┘
 └──────────────────SERVO_OFF───────────────────────┘
```

| State | 動作 |
|-------|------|
| OFF   | 全軸 MotorState::OFF。`INIT_POSITION_RESET` でエンコーダ位置リセット可能 |
| STOP  | 全軸 MotorState::STOP（サーボON、現在位置保持） |
| READY | 現在位置 → `initial_position` へ線形補間で移動。hip/knee は POSITION、wheel は VELOCITY(0) |
| RUN   | **外部ノードからの `run_command` をパススルー**（後述） |

遷移は `state_command` 入力で駆動。`READY → RUN` は補間完了（`IsReadyComplete()`）が前提条件で、未完了なら RUN コマンドを拒否する。

## RUN モード — 外部からの制御注入

RUN モードの核心は、**RCM 自身は制御ロジックを持たず、外部ノードが送る `run_command`（AxisRef[]）をそのままモーターコマンドとして出力する**点にある。

```
[外部コントローラ]                [RCM]                    [DCM / mujoco_node]
      │                           │                              │
      │── run_command (AxisRef[]) ─▶│                              │
      │── state_command ──────────▶│                              │
      │◀── state_status ──────────│                              │
      │                           │── motor_commands (AxisRef[]) ─▶│
      │                           │◀── motor_status (AxisAct[]) ──│
      │                           │   (position, velocity,        │
      │                           │    torque, fault)             │
```

### motor_status 駆動アーキテクチャ

システムの周期クロックは **DCM（または mujoco_node）の 3ms tick** が唯一の源。
RCM と外部コントローラは tick を持たず、`motor_status` の到着を制御ループのトリガーにする。

| ノード | tick | 駆動 |
|--------|------|------|
| **DCM / mujoco_node** | 3ms | **唯一のクロック源** — CAN送受信（実機）またはシム物理ステップ |
| **RCM** | 30ms (watchdog) | motor_status 受信で制御ループ実行 |
| **外部コントローラ** | なし | motor_status 受信で制御計算 |

```
  tick (3ms)
     │
     ▼
[DCM / mujoco_node]          [RCM]                    [外部コントローラ]
  毎tick:                     motor_status受信時:        motor_status受信時:
  CAN送受信 or シムステップ    UpdateMotorStatus()       PID/SysID計算
  → motor_status出力 ────────▶ RobotController()        → run_command出力
                               → motor_commands出力     ◀── state_status
                               → state_status出力 ─────▶ state_command発行
                                                          run_command発行 ──▶ RCM
```

**ポイント:**
- DCM が自律的に 3ms tick で CAN 通信タイミングを決定。コマンドがなくてもクエリのみで motor_status を生成
- RCM は motor_status 受信のたびに制御計算→コマンド出力。DCM の CAN 応答に自然に同期
- 外部コントローラも motor_status 受信をトリガーに即計算。独自 tick 不要で同期ずれなし
- RCM の watchdog（30ms）は motor_status 途絶検出専用。通常の制御ループには使わない

### 起動シーケンス

```
時間軸 ─────────────────────────────────────────────────────────────▶

                                                    DCM tick
                                                       │
[外部コントローラ]     [RCM]           [DCM]           ▼
                                        │←── 起動直後から motor_status を生成
                                        │
  SERVO_ON ──────────▶ OFF→STOP         │
                       初回 motor_commands 出力 ─────▶ コマンド反映
                       ◀── motor_status ─────────────┘
                       (以降 motor_status 駆動ループ)
  ◀── state_status=STOP

  READY ─────────────▶ STOP→READY ── 補間中 cmd ──▶ 初期姿勢へ移動
                       ◀── state_status=READY
  (IsReadyComplete を待つ)

  RUN ───────────────▶ READY→RUN
  run_command ────────▶ パススルー ── run cmd ─────▶ 制御実行
                       ◀── motor_status ────────────┘
                       (フォルト監視・limit クランプ)

  STOP / SERVO_OFF ──▶ → STOP or OFF ─ OFF cmd ──▶ サーボOFF
```

- **DCM**: 起動直後から自律 tick で motor_status を生成し続ける。RCM やコントローラの状態に依存しない
- **RCM**: SERVO_ON 受信時に初回 motor_commands を出力（DCM → motor_status → 駆動ループ開始のキック）
- **外部コントローラ**: motor_status 受信で state_status を確認し、適切なタイミングで state_command を発行

### パススルーの仕組み (`RobotController()` の RUN ケース)

```cpp
case State::RUN:
    for (size_t i = 0; i < axes.size(); i++) {
        const auto& ref = run_command_[i];
        // SET_POSITION / OFF は RUN 中に許可しない
        if (ref.motor_state == MotorState::SET_POSITION ||
            ref.motor_state == MotorState::OFF) continue;

        commands_[i].motor_state = ref.motor_state;
        commands_[i].ref_val    = ref.ref_val;
        commands_[i].kp_scale   = ref.kp_scale;
        commands_[i].kv_scale   = ref.kv_scale;

        // limits をコンフィグ上限でクランプ（NaN なら config 値を使用）
        commands_[i].velocity_limit = clamp_or_config(...);
        commands_[i].accel_limit    = clamp_or_config(...);
        commands_[i].torque_limit   = clamp_or_config(...);
    }
    run_command_received_ = false;
```

**ポイント:**
- `motor_state`, `ref_val`, ゲイン (`kp_scale`/`kv_scale`) は外部ノードが自由に指定できる
- `velocity_limit`, `accel_limit`, `torque_limit` は **config の上限値でクランプ** → 安全制約を RCM が保証
- `SET_POSITION` / `OFF` は RUN 中に発行不可（安全バルブ）
- `run_command` が届かない tick は前回値を維持

### 外部コントローラの例

| ノード | 用途 | dataflow |
|--------|------|----------|
| `stabilizer` | 通常歩行制御 | `dataflow.yaml`, `dataflow_sim.yaml` |
| `sysid_controller` | SysID 励起信号生成 | `dataflow_sysid.yaml`, `dataflow_sim_sysid.yaml` |

どちらも同じインターフェース（`state_command` + `run_command`）で RCM と接続し、RUN モードの振る舞いだけが異なる。

## 安全機構

RCM は RUN パススルーに加えて以下の安全監視を常時実行する:

1. **フォルト検出** — `FaultEvaluator`（moteus フォルトコード評価）が注入される。フォルト検出時は全軸 OFF に遷移
2. **トルクリミット連続ヒット** — 各軸で `torque >= config.torque_limit` が 100 tick 連続したらその軸をサーボ OFF
3. **Limit クランプ** — RUN モードで外部から指定された limits が config 上限を超えないよう強制
4. **ウォッチドッグ** — 30ms ごとに motor_status 受信を確認。途絶時は全軸 SERVO_OFF に遷移

## Dora I/O

| 方向 | ID | 型 | 説明 |
|------|----|----|------|
| 入力 | `watchdog` | timer/30ms | motor_status 途絶検出用ウォッチドッグ |
| 入力 | `state_command` | uint8 (StateCommand) | ステート遷移指示 |
| 入力 | `motor_status` | AxisAct[] | 各軸の実測値 — **制御ループのトリガー** |
| 入力 | `run_command` | AxisRef[] | RUN モード時の制御コマンド（外部注入） |
| 出力 | `motor_commands` | AxisRef[] | DCM へ送る最終コマンド |
| 出力 | `state_status` | uint8 (State) | 現在のステート |

## ビルド

```bash
cd src/cpp && mkdir -p build && cd build && cmake .. && make
```

バイナリ: `src/cpp/node/robot_control_manager/build/robot_control_manager`
