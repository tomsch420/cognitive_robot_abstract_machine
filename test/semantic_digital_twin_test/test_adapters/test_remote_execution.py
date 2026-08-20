from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import pytest
from typing_extensions import List, Optional

from semantic_digital_twin.adapters.partnet_mobility_dataset.remote_execution import (
    BRANCH_VARIABLE_NAME,
    CompletedCommand,
    DATASET_DIRECTORY_VARIABLE_NAME,
    REPOSITORY_URL_VARIABLE_NAME,
    RemoteExecutionConfiguration,
    RemoteMiningJob,
)

# %% test doubles


@dataclass
class RecordingCommandRunner:
    """
    Records the command it is given instead of executing it.
    """

    result: CompletedCommand = field(
        default_factory=lambda: CompletedCommand(
            exit_code=0, standard_output="", standard_error=""
        )
    )
    """
    The result handed back to the caller.
    """

    recorded_command: Optional[List[str]] = None
    """
    The command of the most recent call.
    """

    def run(self, command: List[str]) -> CompletedCommand:
        self.recorded_command = list(command)
        return self.result


# %% fixtures


@pytest.fixture
def configuration() -> RemoteExecutionConfiguration:
    return RemoteExecutionConfiguration(
        host="neem-4.informatik.uni-bremen.de",
        user="tom_sch",
        dataset_directory=Path("/raid/users/tom_sch/datasets/partnet-mobility-dataset"),
        image_name="partnet-mining:latest",
        repository_url="https://github.com/tomsch420/cognitive_robot_abstract_machine.git",
        branch="partnet-remote-access",
    )


# %% building the invocation


def test_command_is_the_full_remote_docker_invocation(
    configuration: RemoteExecutionConfiguration,
):
    job = RemoteMiningJob(
        configuration=configuration,
        module="semantic_digital_twin.adapters.partnet_mobility_dataset.loader",
    )

    assert job.command == [
        "ssh",
        "tom_sch@neem-4.informatik.uni-bremen.de",
        "docker",
        "run",
        "--rm",
        "--volume",
        "/raid/users/tom_sch/datasets/partnet-mobility-dataset:"
        "/data/partnet-mobility-dataset:ro",
        "--env",
        f"{REPOSITORY_URL_VARIABLE_NAME}="
        "https://github.com/tomsch420/cognitive_robot_abstract_machine.git",
        "--env",
        f"{BRANCH_VARIABLE_NAME}=partnet-remote-access",
        "--env",
        f"{DATASET_DIRECTORY_VARIABLE_NAME}=/data/partnet-mobility-dataset",
        "partnet-mining:latest",
        "semantic_digital_twin.adapters.partnet_mobility_dataset.loader",
    ]


def test_dataset_is_mounted_read_only(configuration: RemoteExecutionConfiguration):
    job = RemoteMiningJob(configuration=configuration, module="a_module")

    mount = job.command[job.command.index("--volume") + 1]

    assert mount.endswith(":ro")
    assert mount.startswith(f"{configuration.dataset_directory}:")


def test_module_arguments_are_appended_after_the_module(
    configuration: RemoteExecutionConfiguration,
):
    job = RemoteMiningJob(
        configuration=configuration,
        module="a_module",
        arguments=["--model-id", "35059"],
    )

    assert job.command[-3:] == ["a_module", "--model-id", "35059"]


def test_container_dataset_directory_is_configurable(
    configuration: RemoteExecutionConfiguration,
):
    configuration.container_dataset_directory = Path("/elsewhere")

    job = RemoteMiningJob(configuration=configuration, module="a_module")

    assert job.command[job.command.index("--volume") + 1] == (
        f"{configuration.dataset_directory}:/elsewhere:ro"
    )
    assert f"{DATASET_DIRECTORY_VARIABLE_NAME}=/elsewhere" in job.command


# %% running the invocation


def test_run_passes_the_built_command_to_the_runner_and_returns_its_result(
    configuration: RemoteExecutionConfiguration,
):
    expected_result = CompletedCommand(
        exit_code=0, standard_output="bodies: 3", standard_error=""
    )
    command_runner = RecordingCommandRunner(result=expected_result)
    job = RemoteMiningJob(
        configuration=configuration,
        module="a_module",
        command_runner=command_runner,
    )

    result = job.run()

    assert command_runner.recorded_command == job.command
    assert result == expected_result


def test_completed_command_reports_failure_by_exit_code():
    assert CompletedCommand(
        exit_code=0, standard_output="", standard_error=""
    ).succeeded
    assert not CompletedCommand(
        exit_code=1, standard_output="", standard_error=""
    ).succeeded
