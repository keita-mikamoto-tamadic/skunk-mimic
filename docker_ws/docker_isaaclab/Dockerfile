# Isaac Lab runtime for isaaclab_ws: the Isaac Lab image plus dora for the dora bridge (sim_base/dora_bridge.py).
#
# The base is built on the IsaacLab side:  cd ~/IsaacLab && ./docker/container.py build base
# then here:                                 docker compose build
#
# dora is skunk-mimic's pinned commit (its docs/dora-upgrade.md): every node and daemon of a dataflow must run the
# same commit, or the dora-message serialization does not match and the node cannot register. The commit is not on
# PyPI, so the CLI and the Python node API are built from source.
ARG DORA_COMMIT=fd1f050b48d8dd943c1764e8e73631e8c310866b

# glibc 2.36 here <= 2.39 of the Ubuntu 24.04 base, so the binary and the wheel run there
FROM rust:1.88-bookworm AS dora
ARG DORA_COMMIT
RUN apt-get update && apt-get install -y --no-install-recommends python3-venv && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv /opt/maturin && /opt/maturin/bin/pip install --no-cache-dir maturin
RUN git init /dora && git -C /dora fetch --depth 1 https://github.com/dora-rs/dora.git ${DORA_COMMIT} \
    && git -C /dora checkout FETCH_HEAD
WORKDIR /dora
RUN cargo build --release -p dora-cli
# abi3 (Python >= 3.11) wheel
RUN /opt/maturin/bin/maturin build --release -m apis/python/node/Cargo.toml -o /wheels

FROM isaac-lab-base
COPY --from=dora /dora/target/release/dora /usr/local/bin/dora
# --no-deps: pyarrow, numpy and pyyaml are in the Isaac Lab venv already; leave their versions alone.
# Mounted rather than copied: the base runs as a non-root user, who could not remove a copied wheel.
RUN --mount=type=bind,from=dora,source=/wheels,target=/tmp/dora-wheels \
    uv pip install --python /opt/isaaclab-venv/bin/python --no-deps /tmp/dora-wheels/*.whl
