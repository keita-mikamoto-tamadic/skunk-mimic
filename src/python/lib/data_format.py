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

SETTINGS_RESULT_FMT = "<BBB1xII"
SETTINGS_RESULT_SIZE = struct.calcsize(SETTINGS_RESULT_FMT)  # 12
SETTINGS_RESULT_FIELDS = ['cmd', 'param_index', 'ok', 'value', 'old_value']
SettingsResult = namedtuple("SettingsResult", SETTINGS_RESULT_FIELDS)
SettingsResult.__new__.__defaults__ = (0, 0, 0, 0, 0,)

def pack_settings_result(rec):
    return struct.pack(SETTINGS_RESULT_FMT, *rec)

def unpack_settings_result(buf, offset=0):
    return SettingsResult(*struct.unpack_from(SETTINGS_RESULT_FMT, buf, offset))

PARAM_SCALARS_FMT = "<IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII"
PARAM_SCALARS_SIZE = struct.calcsize(PARAM_SCALARS_FMT)  # 360
PARAM_SCALARS_FIELDS = ['motor_id', 'device_id', 'rot_dir', 'mech_angle_dir', 'elec_angle_dir', 'mot_pole_pairs', 'gear_enable', 'gear_ratio', 'zero_pos_ofs', 'p_gain_cur', 'i_gain_cur', 'd_gain_cur', 'p_gain_vel', 'i_gain_vel', 'd_gain_vel', 'p_gain_pos', 'i_gain_pos', 'd_gain_pos', 'cur_q_mx', 'cur_q_mn', 'trq_out_mx', 'trq_out_mn', 'vel_out_mx', 'vel_out_mn', 'pos_out_mx', 'pos_out_mn', 'elec_angle_ofs']
ParamScalars = namedtuple("ParamScalars", PARAM_SCALARS_FIELDS)
ParamScalars.__new__.__defaults__ = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (),)

PARAM_SCALARS_COUNTS = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 64]
def pack_param_scalars(rec):
    flat = []
    for v, c in zip(rec, PARAM_SCALARS_COUNTS):
        flat.extend(v) if c > 1 else flat.append(v)
    return struct.pack(PARAM_SCALARS_FMT, *flat)

def unpack_param_scalars(buf, offset=0):
    flat = struct.unpack_from(PARAM_SCALARS_FMT, buf, offset)
    vals = []; pos = 0
    for c in PARAM_SCALARS_COUNTS:
        if c > 1: vals.append(flat[pos:pos + c]); pos += c
        else: vals.append(flat[pos]); pos += 1
    return ParamScalars(*vals)
