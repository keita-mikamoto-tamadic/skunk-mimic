"""AUTO-GENERATED from src/data_format/command_data.json by tools/gen_data_format.py.

DO NOT EDIT. 再生成: python3 tools/gen_data_format.py
"""
import struct
from collections import namedtuple

DRIVE_COMMAND_FMT = "<dddd"
DRIVE_COMMAND_SIZE = struct.calcsize(DRIVE_COMMAND_FMT)  # 32
DRIVE_COMMAND_FIELDS = ['timestamp', 'forward', 'yaw', 'height']
DriveCommand = namedtuple("DriveCommand", DRIVE_COMMAND_FIELDS)
DriveCommand.__new__.__defaults__ = (0, 0, 0, 0,)

def pack_drive_command(rec):
    return struct.pack(DRIVE_COMMAND_FMT, *rec)

def unpack_drive_command(buf, offset=0):
    return DriveCommand(*struct.unpack_from(DRIVE_COMMAND_FMT, buf, offset))
