#!/usr/bin/env python3
"""src/data_format/axis_data.json を正本に、C++ と Python のデータフォーマット定義を生成する。

dora で C++↔Python 間を生バイト列で交換する構造体の単一正本。手書き重複(と
バイナリのサイレントなドリフト)を防ぐためにここから両言語の定義を生成する。

  入力 : src/data_format/axis_data.json
  出力 : src/cpp/lib/data_format_generated.hpp   (型付き struct + static_assert)
         src/python/lib/data_format.py           (struct format / namedtuple / pack)

CMake の configure 時に自動実行される。手動でも実行可:
    python3 tools/gen_data_format.py
"""
import json
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JSON_PATH = os.path.join(ROOT, "src", "data_format", "axis_data.json")
CPP_OUT = os.path.join(ROOT, "src", "cpp", "lib", "data_format_generated.hpp")
PY_OUT = os.path.join(ROOT, "src", "python", "lib", "data_format.py")

GEN_NOTE = "AUTO-GENERATED from src/data_format/axis_data.json by tools/gen_data_format.py"


def camel_to_snake(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def layout(fields, types):
    """C 構造体のレイアウトを計算。
    returns (entries, total_size) where entries = list of dict(field, cpp, py, off, pad)
    """
    offset = 0
    struct_align = 1
    entries = []
    for f in fields:
        t = types[f["type"]]
        size, align = t["size"], t["align"]
        pad = (-offset) % align          # このフィールド前の整列パディング
        offset += pad
        entries.append({
            "name": f["name"],
            "cpp": f.get("cpp_type", t["cpp"]),  # cpp_type 指定があれば優先
            "py": t["py"],
            "off": offset,
            "pad": pad,
            "doc": f.get("doc", ""),
        })
        offset += size
        struct_align = max(struct_align, align)
    tail = (-offset) % struct_align       # 構造体末尾パディング
    return entries, offset + tail, tail


def gen_cpp(structs, types):
    out = [
        "#pragma once",
        f"// {GEN_NOTE}",
        "// DO NOT EDIT. 再生成: python3 tools/gen_data_format.py",
        "#include <cstdint>",
        "#include <cstddef>",
        '#include "enum_def.hpp"',
        "",
    ]
    for s in structs:
        entries, total, _ = layout(s["fields"], types)
        if s.get("doc"):
            out.append(f"// {s['doc']}")
        out.append(f"struct {s['name']} {{")
        for e in entries:
            doc = f"  // {e['doc']}" if e["doc"] else ""
            out.append(f"    {e['cpp']} {e['name']};{doc}")
        out.append("};")
        out.append(f'static_assert(sizeof({s["name"]}) == {total}, '
                   f'"{s["name"]} size mismatch vs axis_data.json");')
        for e in entries:
            out.append(f'static_assert(offsetof({s["name"]}, {e["name"]}) == {e["off"]}, '
                       f'"{s["name"]}.{e["name"]} offset mismatch vs axis_data.json");')
        out.append("")
    return "\n".join(out) + "\n"


def gen_py(structs, types):
    out = [
        '"""' + GEN_NOTE + ".",
        "",
        "DO NOT EDIT. 再生成: python3 tools/gen_data_format.py",
        '"""',
        "import struct",
        "from collections import namedtuple",
        "",
    ]
    for s in structs:
        entries, total, tail = layout(s["fields"], types)
        fmt = "<"
        for e in entries:
            if e["pad"]:
                fmt += f"{e['pad']}x"
            fmt += e["py"]
        if tail:
            fmt += f"{tail}x"

        upper = camel_to_snake(s["name"]).upper()
        snake = camel_to_snake(s["name"])
        field_names = [e["name"] for e in entries]

        out.append(f'{upper}_FMT = "{fmt}"')
        out.append(f"{upper}_SIZE = struct.calcsize({upper}_FMT)  # {total}")
        out.append(f"{upper}_FIELDS = {field_names!r}")
        out.append(f'{s["name"]} = namedtuple("{s["name"]}", {upper}_FIELDS)')
        # 全フィールド 0 デフォルト(必要なものだけ kwargs で渡せる)
        out.append(f"{s['name']}.__new__.__defaults__ = "
                   f"({', '.join(['0'] * len(field_names))},)")
        out.append("")
        out.append(f"def pack_{snake}(rec):")
        out.append(f"    return struct.pack({upper}_FMT, *rec)")
        out.append("")
        out.append(f"def unpack_{snake}(buf, offset=0):")
        out.append(f"    return {s['name']}(*struct.unpack_from({upper}_FMT, buf, offset))")
        out.append("")
    return "\n".join(out)


def main():
    with open(JSON_PATH, encoding="utf-8") as f:
        spec = json.load(f)
    types = spec["types"]
    structs = spec["structs"]

    with open(CPP_OUT, "w", encoding="utf-8") as f:
        f.write(gen_cpp(structs, types))
    with open(PY_OUT, "w", encoding="utf-8") as f:
        f.write(gen_py(structs, types))

    print(f"generated:\n  {CPP_OUT}\n  {PY_OUT}")


if __name__ == "__main__":
    main()
