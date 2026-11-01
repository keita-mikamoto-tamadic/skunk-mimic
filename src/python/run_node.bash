#!/usr/bin/env bash
# dora から Python ノードを spawn するための共通ラッパ。
#   path: src/python/run_node.bash
#   args: <node>/<node>.py          (src/python からの相対)
# dora は path が .py だとシステムの python3 で起動し、uv 環境の dora パッケージが
# 見えないため、src/python/.venv の python で実行する (要: cd src/python && uv sync)。
cd "$(dirname "$0")" || exit 1
exec .venv/bin/python -u "$@"
