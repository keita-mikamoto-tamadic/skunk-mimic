"""AUTO-GENERATED from src/data_format/axis_data.json by tools/gen_data_format.py.

DO NOT EDIT. 再生成: python3 tools/gen_data_format.py
"""
import struct
from collections import namedtuple

AXIS_REF_FMT = "<B7xdddddddd"
AXIS_REF_SIZE = struct.calcsize(AXIS_REF_FMT)  # 72
AXIS_REF_FIELDS = ['motor_state', 'ref_val', 'ref_val_1', 'ref_val_2', 'kp_scale', 'kv_scale', 'velocity_limit', 'accel_limit', 'torque_limit']
AxisRef = namedtuple("AxisRef", AXIS_REF_FIELDS)
AxisRef.__new__.__defaults__ = (0, 0, 0, 0, 0, 0, 0, 0, 0,)

def pack_axis_ref(rec):
    return struct.pack(AXIS_REF_FMT, *rec)

def unpack_axis_ref(buf, offset=0):
    return AxisRef(*struct.unpack_from(AXIS_REF_FMT, buf, offset))

AXIS_ACT_FMT = "<dddB7x"
AXIS_ACT_SIZE = struct.calcsize(AXIS_ACT_FMT)  # 32
AXIS_ACT_FIELDS = ['position', 'velocity', 'torque', 'fault']
AxisAct = namedtuple("AxisAct", AXIS_ACT_FIELDS)
AxisAct.__new__.__defaults__ = (0, 0, 0, 0,)

def pack_axis_act(rec):
    return struct.pack(AXIS_ACT_FMT, *rec)

def unpack_axis_act(buf, offset=0):
    return AxisAct(*struct.unpack_from(AXIS_ACT_FMT, buf, offset))

SETTINGS_REQUEST_FMT = "<BBB1xI"
SETTINGS_REQUEST_SIZE = struct.calcsize(SETTINGS_REQUEST_FMT)  # 8
SETTINGS_REQUEST_FIELDS = ['device_id', 'cmd', 'param_index', 'value']
SettingsRequest = namedtuple("SettingsRequest", SETTINGS_REQUEST_FIELDS)
SettingsRequest.__new__.__defaults__ = (0, 0, 0, 0,)

def pack_settings_request(rec):
    return struct.pack(SETTINGS_REQUEST_FMT, *rec)

def unpack_settings_request(buf, offset=0):
    return SettingsRequest(*struct.unpack_from(SETTINGS_REQUEST_FMT, buf, offset))

SETTINGS_RESULT_FMT = "<BBB1xI"
SETTINGS_RESULT_SIZE = struct.calcsize(SETTINGS_RESULT_FMT)  # 8
SETTINGS_RESULT_FIELDS = ['cmd', 'param_index', 'ok', 'value']
SettingsResult = namedtuple("SettingsResult", SETTINGS_RESULT_FIELDS)
SettingsResult.__new__.__defaults__ = (0, 0, 0, 0,)

def pack_settings_result(rec):
    return struct.pack(SETTINGS_RESULT_FMT, *rec)

def unpack_settings_result(buf, offset=0):
    return SettingsResult(*struct.unpack_from(SETTINGS_RESULT_FMT, buf, offset))
