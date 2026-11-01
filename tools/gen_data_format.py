#!/usr/bin/env python3
"""src/data_format/*.json を正本に、C++ と Python のデータフォーマット定義を生成する。

dora で C++↔Python 間を生バイト列で交換する構造体の単一正本。手書き重複(と
バイナリのサイレントなドリフト)を防ぐためにここから両言語の定義を生成する。

スペックは 1 ドメイン = 1 ファイル。src/data_format/ に置いた .json は自動で拾われ、
対になる生成物が出る (命名は <スペック名>_format.hpp / <スペック名>_format.py で統一):

  src/data_format/axis_data.json    → src/cpp/lib/axis_data_format.hpp
                                      src/python/lib/axis_data_format.py
  src/data_format/sensor_data.json  → src/cpp/lib/sensor_data_format.hpp
                                      src/python/lib/sensor_data_format.py

新しいドメインを足すときは .json を置くだけ。"types" を省略すると下の
DEFAULT_TYPES が使われる。

wire index 付き構造体 (フィールドに "index" を持つもの。例: ParamScalars):
  外部プロトコル (FOCTIVE の param_index) と番号で対応する構造体。構造体の並びは
  dora で受け渡す都合の順 (配列は末尾) だが、相手側は index 順のメモリ並びで扱うので、
  index 順のレイアウトと番号表も一緒に生成する。値の解釈は "repr" で指定
  ("float" = 4byte を float として読む。省略時は型どおり = 整数)。
    C++    : namespace <snake名> { struct Wire; enum Index; kCount; struct Desc; kDesc[] }
    Python : <UPPER>_WIRE_INDEX (name→index), <UPPER>_WIRE_REPR (name→"float"|型名)

CMake の configure 時に自動実行される。手動でも実行可:
    python3 tools/gen_data_format.py
"""
import glob
import json
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SPEC_DIR = os.path.join(ROOT, "src", "data_format")
CPP_DIR = os.path.join(ROOT, "src", "cpp", "lib")
PY_DIR = os.path.join(ROOT, "src", "python", "lib")

# スペックが "types" を持たない場合に使う型テーブル (言語マッピングであって
# ドメインデータではないので、スペック側での重複を避けここに置く)
DEFAULT_TYPES = {
    "double": {"cpp": "double", "size": 8, "align": 8, "py": "d"},
    "uint32": {"cpp": "uint32_t", "size": 4, "align": 4, "py": "I"},
    "uint8": {"cpp": "uint8_t", "size": 1, "align": 1, "py": "B"},
}


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


def snake_to_camel(name):
    return "".join(w.capitalize() for w in name.split("_"))


def wire_fields(s):
    """wire index を持つ構造体なら index 順のフィールド list、無ければ None。"""
    if not any("index" in f for f in s["fields"]):
        return None
    if not all("index" in f for f in s["fields"]):
        raise SystemExit(f'{s["name"]}: "index" は全フィールドに付けること')
    fields = sorted(s["fields"], key=lambda f: f["index"])
    if [f["index"] for f in fields] != list(range(len(fields))):
        raise SystemExit(f'{s["name"]}: index は 0 から連番であること')
    return fields


def gen_cpp_wire(s, types, src_name):
    """index 順レイアウト (相手側のメモリ並び) と番号表。"""
    fields = wire_fields(s)
    ns = camel_to_snake(s["name"])
    wire = [dict(f, cpp_type="float") if f.get("repr") == "float" else f for f in fields]
    entries, total, _ = layout(wire, types)
    out = [f"// {s['name']} の wire index 順レイアウトと番号表 (相手側のメモリ並び)",
           f"namespace {ns} {{"]
    out.append("struct Wire {")
    for e in entries:
        arr = f"[{e['count']}]" if e["count"] > 1 else ""
        out.append(f"    {e['cpp']} {e['name']}{arr};")
    out.append("};")
    out.append(f'static_assert(sizeof(Wire) == {total}, "{s["name"]} wire size mismatch vs {src_name}");')
    out.append(f"constexpr uint8_t kCount = {len(fields)};")
    out.append("enum Index : uint8_t {")
    for f in fields:
        out.append(f"    k{snake_to_camel(f['name'])} = {f['index']},")
    out.append("    kInvalid = 0xFF,")
    out.append("};")
    out.append("struct Desc {")
    out.append("    uint16_t offset;    // Wire 先頭からのバイト位置")
    out.append("    uint16_t size;")
    out.append('    bool     is_float;  // repr == "float"')
    out.append("};")
    out.append("inline constexpr Desc kDesc[kCount] = {")
    for f, e in zip(fields, entries):
        size = types[f["type"]]["size"] * e["count"]
        isf = "true" if f.get("repr") == "float" else "false"
        out.append(f"    {{{e['off']}, {size}, {isf}}},  // {f['index']} {f['name']}")
    out.append("};")
    out.append(f"}}  // namespace {ns}")
    out.append("")
    return out


def gen_cpp(structs, types, src_name):
    out = [
        "#pragma once",
        "// Do not edit!",
        f"// AUTO-GENERATED from src/data_format/{src_name} by tools/gen_data_format.py",
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
                   f'"{s["name"]} size mismatch vs {src_name}");')
        for e in entries:
            out.append(f'static_assert(offsetof({s["name"]}, {e["name"]}) == {e["off"]}, '
                       f'"{s["name"]}.{e["name"]} offset mismatch vs {src_name}");')
        out.append("")
        if wire_fields(s):
            out += gen_cpp_wire(s, types, src_name)
    return "\n".join(out) + "\n"


def gen_py(structs, types, src_name):
    out = [
        "# Do not edit!",
        f'"""AUTO-GENERATED from src/data_format/{src_name} by tools/gen_data_format.py.',
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
            wire_repr = {f["name"]: f.get("repr", f["type"]) for f in s["fields"]}
            out.append(f"{upper}_WIRE_REPR = {wire_repr!r}")
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
    spec_paths = sorted(glob.glob(os.path.join(SPEC_DIR, "*.json")))
    if not spec_paths:
        raise SystemExit(f"no spec files found in {SPEC_DIR}")

    for path in spec_paths:
        src_name = os.path.basename(path)
        stem = os.path.splitext(src_name)[0]
        with open(path, encoding="utf-8") as f:
            spec = json.load(f)
        types = spec.get("types", DEFAULT_TYPES)
        structs = spec["structs"]

        cpp_out = os.path.join(CPP_DIR, f"{stem}_format.hpp")
        py_out = os.path.join(PY_DIR, f"{stem}_format.py")

        with open(cpp_out, "w", encoding="utf-8") as f:
            f.write(gen_cpp(structs, types, src_name))
        with open(py_out, "w", encoding="utf-8") as f:
            f.write(gen_py(structs, types, src_name))

        print(f"generated from {src_name}:\n  {cpp_out}\n  {py_out}")


if __name__ == "__main__":
    main()
