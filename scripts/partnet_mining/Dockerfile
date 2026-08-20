# Environment for running mining code against an already-downloaded
# PartNet-Mobility corpus. The repository itself is not baked in: the entrypoint
# checks it out at container start, so a code change needs no rebuild.
#
# Build from the repository root:
#   docker build -f scripts/partnet_mining/Dockerfile -t partnet-mining:latest .
FROM python:3.12-slim

# pygraphviz builds against graphviz's headers; python-fcl and trimesh's
# accelerators need a compiler.
RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        build-essential \
        git \
        graphviz \
        libgraphviz-dev \
    && rm -rf /var/lib/apt/lists/*

COPY krrood/requirements.txt /requirements/krrood.txt
COPY semantic_digital_twin/requirements.txt /requirements/semantic_digital_twin.txt
COPY giskardpy/requirements.txt /requirements/giskardpy.txt

# semantic_digital_twin's collision checking imports giskardpy, so that package's
# requirements are needed too. Two more are imported directly but declared by
# neither file: mujoco, which only physics_simulators declares, and objgraph,
# which only the root project's dev extra declares.
RUN pip install --no-cache-dir \
        -r /requirements/krrood.txt \
        -r /requirements/semantic_digital_twin.txt \
        -r /requirements/giskardpy.txt \
        mujoco==3.9.0 \
        objgraph

COPY scripts/partnet_mining/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR /work
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
