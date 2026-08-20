# Running mining jobs against the PartNet-Mobility corpus on neem-4

The corpus lives on `neem-4.informatik.uni-bremen.de` at
`/raid/users/tom_sch/datasets/partnet-mobility-dataset` (2218 models, 8.1GB) and
is never copied locally or committed to git. Runs happen on that host, inside a
container that mounts the corpus **read-only**.

The image carries the environment only. The repository is cloned when the
container starts, so changing code needs a push, not a rebuild.

## Prerequisites

- The `Uni Bremen` VPN must be active: `nmcli con up "Uni Bremen"`. Without it,
  SSH to the host times out.
- SSH access as a user in the host's `docker` group.

## Build the image (once, on the host)

The build needs the two requirements files, so it runs from a checkout:

```bash
ssh tom_sch@neem-4.informatik.uni-bremen.de
git clone --depth 1 https://github.com/tomsch420/cognitive_robot_abstract_machine.git /raid/users/tom_sch/work/partnet-mining-build
cd /raid/users/tom_sch/work/partnet-mining-build
docker build -f scripts/partnet_mining/Dockerfile -t partnet-mining:latest .
```

Docker's images live on `/`, not `/raid`. If a container fails to start, check
`df -h /` before anything else — a full `/` stops `docker run` and even
`docker exec`.

## Run a module

From Python, which assembles the invocation and reports how it finished:

```python
from pathlib import Path

from semantic_digital_twin.adapters.partnet_mobility_dataset.remote_execution import (
    RemoteExecutionConfiguration,
    RemoteMiningJob,
)

configuration = RemoteExecutionConfiguration(
    host="neem-4.informatik.uni-bremen.de",
    user="tom_sch",
    dataset_directory=Path("/raid/users/tom_sch/datasets/partnet-mobility-dataset"),
    image_name="partnet-mining:latest",
    repository_url="https://github.com/tomsch420/cognitive_robot_abstract_machine.git",
    branch="partnet-remote-access",
)
result = RemoteMiningJob(configuration=configuration, module="your.module").run()
print(result.standard_output)
```

The container receives the corpus path as `PARTNET_MOBILITY_DATASET_DIRECTORY`,
which is also the variable the corpus-backed tests are gated on, so
`pytest test/semantic_digital_twin_test/test_adapters/test_partnet_mobility.py`
runs the real-corpus test inside the container and skips it everywhere else.

## What is deliberately not installed

`sapien` is absent from the image, and no `SAPIEN_ACCESS_TOKEN` is passed. Both
are only needed to *download* models; the corpus is already on disk and is read
by `PartNetMobilityDatasetLoader.load_from_directory`, which touches neither.

There is no ROS and no `physics_simulators`, so the parts of the test suite that
resolve `package://` URIs or drive a simulator do not run here — they fail the
same way on any branch. The image covers the corpus and mining path, not the
whole workspace.
