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
[外部コントローラ]                [RCM]                    [device_control_manager]
      │                           │                              │
      │── run_command (AxisRef[]) ─▶│                              │
      │── state_command ──────────▶│                              │
      │◀── state_status ──────────│                              │
      │                           │── motor_commands (AxisRef[]) ─▶│
      │                           │◀── motor_status (AxisAct[]) ──│
      │                           │   (position, velocity,        │
      │                           │    torque, fault)             │
```

### 起動シーケンスと制御の主導権

3ノードは `dora start` で同時に起動するが、**制御の主導権は外部コントローラが握る**。
RCM と device_control_manager は受動的に動作し、自発的に状態遷移を起こさない。

#### tick の所在と制御ループの駆動

システムの周期実行は **tick（`dora/timer/millis/3`）を誰が受け取るか** で決まる。

| ノード | tick | 役割 |
|--------|------|------|
| **外部コントローラ** | あり (3ms) | tick ごとに `state_status` を見て `state_command` / `run_command` を発行 |
| **RCM** | あり (3ms) | tick ごとに `RobotController()` → `motor_commands` を出力 |
| **device_control_manager** | なし | `motor_commands` 受信駆動。受け取ったら CAN 送信 → `motor_status` を返す |

つまり **3ms tick で自律的にループを回すのは外部コントローラと RCM の2ノード**。
device_control_manager は tick を持たず、`motor_commands` が届いたときだけ動く受動ノード。

```
  tick (3ms)                tick (3ms)
     │                        │
     ▼                        ▼
[外部コントローラ]          [RCM]                    [device_control_manager]
  毎tick:                   毎tick:                   motor_commands受信時:
  state_status確認           RobotController()          CAN送受信
  state_command発行          → motor_commands出力 ────▶  → motor_status返却
  run_command発行 ─────────▶ state_status出力
  ◀── state_status ────────                           ◀── motor_status
```

#### 起動シーケンス

```
時間軸 ─────────────────────────────────────────────────────────────▶

[外部コントローラ]  state_command    [RCM]          motor_commands  [device_control_manager]

  ┌─ SERVO_ON ────────▶ OFF→STOP ─── STOP cmd ────▶ サーボON・保持
  │                      ◀── state_status=STOP
  │
  ├─ READY ───────────▶ STOP→READY ─ 補間中 cmd ──▶ 初期姿勢へ移動
  │                      ◀── state_status=READY
  │  (IsReadyComplete を待つ)
  │
  ├─ RUN ─────────────▶ READY→RUN
  │  run_command ──────▶ パススルー ─ run cmd ─────▶ 制御実行
  │                      ◀── motor_status ──────────┘
  │                      (フォルト監視・limit クランプ)
  │
  └─ STOP / SERVO_OFF ▶ → STOP or OFF ── OFF cmd ─▶ サーボOFF
```

- **外部コントローラ**: tick で自律ループ。`state_status` を監視しながら適切なタイミングで `state_command` を発行。RUN 中は毎 tick `run_command` を送り続ける
- **RCM**: tick で自律ループ。`state_command` に応じて遷移するだけ。自発的に RUN へ入ることはない。RUN 中は `run_command` を安全チェック付きで中継
- **device_control_manager**: tick なし。`motor_commands` を受け取り CAN 通信でモーターを駆動、結果を `motor_status` で返すだけ。ステートマシンの存在を知らない

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
    run_command_received_ = false;  // 毎tick リセット
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

## Dora I/O

| 方向 | ID | 型 | 説明 |
|------|----|----|------|
| 入力 | `tick` | timer/3ms | 制御周期トリガー |
| 入力 | `state_command` | uint8 (StateCommand) | ステート遷移指示 |
| 入力 | `motor_status` | AxisAct[] | 各軸の実測値（position, velocity, torque, fault） |
| 入力 | `run_command` | AxisRef[] | RUN モード時の制御コマンド（外部注入） |
| 出力 | `motor_commands` | AxisRef[] | device_control_manager へ送る最終コマンド |
| 出力 | `state_status` | uint8 (State) | 現在のステート |

## ビルド

```bash
cd src/cpp && mkdir -p build && cd build && cmake .. && make
```

バイナリ: `src/cpp/node/robot_control_manager/build/robot_control_manager`
