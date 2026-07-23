"""AUTO-GENERATED from src/data_format/sensor_data.json by tools/gen_data_format.py.

DO NOT EDIT. 再生成: python3 tools/gen_data_format.py
"""
import struct
from collections import namedtuple

IMU_DATA_FMT = "<dddddddddddddd"
IMU_DATA_SIZE = struct.calcsize(IMU_DATA_FMT)  # 112
IMU_DATA_FIELDS = ['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'q0', 'q1', 'q2', 'q3', 'roll', 'pitch', 'yaw']
ImuData = namedtuple("ImuData", IMU_DATA_FIELDS)
ImuData.__new__.__defaults__ = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,)

def pack_imu_data(rec):
    return struct.pack(IMU_DATA_FMT, *rec)

def unpack_imu_data(buf, offset=0):
    return ImuData(*struct.unpack_from(IMU_DATA_FMT, buf, offset))

LATENCY_DATA_FMT = "<dddd"
LATENCY_DATA_SIZE = struct.calcsize(LATENCY_DATA_FMT)  # 32
LATENCY_DATA_FIELDS = ['can_avg_us', 'can_max_us', 'ctrl_avg_us', 'ctrl_max_us']
LatencyData = namedtuple("LatencyData", LATENCY_DATA_FIELDS)
LatencyData.__new__.__defaults__ = (0, 0, 0, 0,)

def pack_latency_data(rec):
    return struct.pack(LATENCY_DATA_FMT, *rec)

def unpack_latency_data(buf, offset=0):
    return LatencyData(*struct.unpack_from(LATENCY_DATA_FMT, buf, offset))

ESTIMATED_STATE_FMT = "<ddd"
ESTIMATED_STATE_SIZE = struct.calcsize(ESTIMATED_STATE_FMT)  # 24
ESTIMATED_STATE_FIELDS = ['velocity', 'yaw', 'yaw_rate']
EstimatedState = namedtuple("EstimatedState", ESTIMATED_STATE_FIELDS)
EstimatedState.__new__.__defaults__ = (0, 0, 0,)

def pack_estimated_state(rec):
    return struct.pack(ESTIMATED_STATE_FMT, *rec)

def unpack_estimated_state(buf, offset=0):
    return EstimatedState(*struct.unpack_from(ESTIMATED_STATE_FMT, buf, offset))
