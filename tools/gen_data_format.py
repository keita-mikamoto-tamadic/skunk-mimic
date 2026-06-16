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
        count = f.get("count", 1)        # 配列要素数 (省略時 1 = スカラ)
        size, align = t["size"] * count, t["align"]
        pad = (-offset) % align          # このフィールド前の整列パディング
        offset += pad
        entries.append({
            "name": f["name"],
            "cpp": f.get("cpp_type", t["cpp"]),  # cpp_type 指定があれば優先
            "py": t["py"],
            "off": offset,
            "pad": pad,
            "count": count,
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
            arr = f"[{e['count']}]" if e["count"] > 1 else ""
            out.append(f"    {e['cpp']} {e['name']}{arr};{doc}")
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
            fmt += e["py"] * e["count"]  # 配列はコードを count 回繰り返す
        if tail:
            fmt += f"{tail}x"

        upper = camel_to_snake(s["name"]).upper()
        snake = camel_to_snake(s["name"])
        field_names = [e["name"] for e in entries]
        counts = [e["count"] for e in entries]
        has_array = any(c > 1 for c in counts)

        out.append(f'{upper}_FMT = "{fmt}"')
        out.append(f"{upper}_SIZE = struct.calcsize({upper}_FMT)  # {total}")
        out.append(f"{upper}_FIELDS = {field_names!r}")
        # フィールドが wire index を持つ場合、name -> index マップを出力
        # (構造体の並び順 != wire param_index のためコード側で数えさせない)
        wire_index = {f["name"]: f["index"] for f in s["fields"] if "index" in f}
        if wire_index:
            out.append(f"{upper}_WIRE_INDEX = {wire_index!r}")
        out.append(f'{s["name"]} = namedtuple("{s["name"]}", {upper}_FIELDS)')
        # デフォルト: スカラは 0、配列は空 tuple
        defaults = ["()" if c > 1 else "0" for c in counts]
        out.append(f"{s['name']}.__new__.__defaults__ = ({', '.join(defaults)},)")
        out.append("")
        if has_array:
            # 配列フィールドあり: フラット化 / 再グループ化
            out.append(f"{upper}_COUNTS = {counts!r}")
            out.append(f"def pack_{snake}(rec):")
            out.append(f"    flat = []")
            out.append(f"    for v, c in zip(rec, {upper}_COUNTS):")
            out.append(f"        flat.extend(v) if c > 1 else flat.append(v)")
            out.append(f"    return struct.pack({upper}_FMT, *flat)")
            out.append("")
            out.append(f"def unpack_{snake}(buf, offset=0):")
            out.append(f"    flat = struct.unpack_from({upper}_FMT, buf, offset)")
            out.append(f"    vals = []; pos = 0")
            out.append(f"    for c in {upper}_COUNTS:")
            out.append(f"        if c > 1: vals.append(flat[pos:pos + c]); pos += c")
            out.append(f"        else: vals.append(flat[pos]); pos += 1")
            out.append(f"    return {s['name']}(*vals)")
            out.append("")
        else:
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
