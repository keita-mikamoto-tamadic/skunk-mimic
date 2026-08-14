"""enum_def.hpp の Python ミラー (手動同期)。

C++ 側 src/cpp/lib/enum_def.hpp が正本。値を変えるときは両方更新すること。
既存ノード (dummy_input / auto_input / sysid_controller / data_viewer /
web_controller) には手書きの重複定義が残っている — 新規コードはこのモジュールを
import する。
"""

from enum import IntEnum


class MotorState(IntEnum):
    """AxisRef.motor_state (enum_def.hpp:7-21)。"""
    OFF = 0
    STOP = 1
    POSITION = 2          # 位置(インピーダンス)
    VELOCITY = 3          # 速度(インピーダンス)
    TORQUE = 4
    SET_POSITION = 5      # エンコーダ位置リセット (OFF時のみ)
    CURRENT = 6
    VOLTAGE = 7
    POSITION_PD = 8       # 位置/速度PDゲイン+FFトルク (インピーダンス制御)
    CASCADE_POS_PID = 9   # 位置カスケードPID (FOCTIVE 専用)
    CASCADE_VEL_PID = 10  # 速度カスケードPID (FOCTIVE 専用)


class State(IntEnum):
    """robot_control_manager の状態 (enum_def.hpp:24-29)。state_status の 1 byte。"""
    OFF = 0
    STOP = 1
    READY = 2
    RUN = 3


class StateCommand(IntEnum):
    """state_command の 1 byte (enum_def.hpp:32-39)。"""
    STOP = 0
    RUN = 1
    SERVO_OFF = 2
    SERVO_ON = 3
    INIT_POSITION_RESET = 4
    READY = 5
