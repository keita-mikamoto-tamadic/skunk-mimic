"""skunk-mimic's dora structs (AxisRef, AxisAct, ...) read from its single source, src/data_format/axis_data.json.

The layout is derived from the JSON at run time, as skunk-mimic's tools/gen_data_format.py does at build time (C
struct, natural alignment, little endian), so it cannot drift from the C++ side: mujoco_backend once kept a hand
copy of 56 / 32 bytes while the structs had grown to 72 / 48.
"""

import json
import struct
from collections import namedtuple
from pathlib import Path


class StructFormat:
    """One struct of the JSON: ``size``, ``pack(**fields)``, ``unpack_from(buf, offset)``."""

    def __init__(self, spec: dict, types: dict):
        fmt, offset, max_align = "<", 0, 1
        for field in spec["fields"]:
            if field.get("count", 1) != 1:
                raise ValueError(f"{spec['name']}.{field['name']}: array fields are not supported")
            t = types[field["type"]]
            pad = -offset % t["align"]
            fmt += f"{pad}x" if pad else ""
            fmt += t["py"]
            offset += pad + t["size"]
            max_align = max(max_align, t["align"])
        pad = -offset % max_align
        self.format = fmt + (f"{pad}x" if pad else "")
        self.size = struct.calcsize(self.format)
        self.fields = [f["name"] for f in spec["fields"]]
        self.tuple = namedtuple(spec["name"], self.fields, defaults=(0,) * len(self.fields))

    def pack(self, **values) -> bytes:
        return struct.pack(self.format, *self.tuple(**values))

    def unpack_from(self, buf, offset: int = 0):
        return self.tuple(*struct.unpack_from(self.format, buf, offset))


def load_structs(path: str | Path) -> dict[str, StructFormat]:
    """All structs of a skunk-mimic data_format JSON without array fields, by name."""
    data = json.loads(Path(path).read_text())
    return {
        s["name"]: StructFormat(s, data["types"])
        for s in data["structs"]
        if all(f.get("count", 1) == 1 for f in s["fields"])
    }
