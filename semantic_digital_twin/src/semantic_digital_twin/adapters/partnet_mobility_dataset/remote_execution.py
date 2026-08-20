from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from pathlib import Path

from typing_extensions import List, Protocol, runtime_checkable

# %% the container's interface

REPOSITORY_URL_VARIABLE_NAME = "PARTNET_MINING_REPOSITORY_URL"
"""
Environment variable naming the repository the container checks out.
"""

BRANCH_VARIABLE_NAME = "PARTNET_MINING_BRANCH"
"""
Environment variable naming the branch the container checks out.
"""

DATASET_DIRECTORY_VARIABLE_NAME = "PARTNET_MINING_DATASET_DIRECTORY"
"""
Environment variable naming the corpus location inside the container.
"""

# %% running a command


@dataclass
class CompletedCommand:
    """
    The outcome of a command that has finished.
    """

    exit_code: int
    """
    The exit code the command returned.
    """

    standard_output: str
    """
    What the command wrote to standard output.
    """

    standard_error: str
    """
    What the command wrote to standard error.
    """

    @property
    def succeeded(self) -> bool:
        """
        :return: Whether the command reported success.
        """
        return self.exit_code == 0


@runtime_checkable
class CommandRunner(Protocol):
    """
    Executes a command and reports how it finished.
    """

    def run(self, command: List[str]) -> CompletedCommand: ...


@dataclass
class SubprocessCommandRunner:
    """
    Runs a command as a subprocess of this process.
    """

    def run(self, command: List[str]) -> CompletedCommand:
        completed_process = subprocess.run(command, capture_output=True, text=True)
        return CompletedCommand(
            exit_code=completed_process.returncode,
            standard_output=completed_process.stdout,
            standard_error=completed_process.stderr,
        )


# %% describing and invoking a remote run


@dataclass
class RemoteExecutionConfiguration:
    """
    Where a mining run happens and what it runs against.
    """

    host: str
    """
    The machine holding the corpus.
    """

    user: str
    """
    The account to connect as.
    """

    dataset_directory: Path
    """
    The corpus location on the host.
    """

    image_name: str
    """
    The container image providing the environment.
    """

    repository_url: str
    """
    The repository the container checks out.
    """

    branch: str
    """
    The branch the container checks out.
    """

    container_dataset_directory: Path = Path("/data/partnet-mobility-dataset")
    """
    Where the corpus appears inside the container.
    """

    @property
    def destination(self) -> str:
        """
        :return: The account and machine to connect to.
        """
        return f"{self.user}@{self.host}"

    @property
    def read_only_dataset_mount(self) -> str:
        """
        :return: The bind mount exposing the corpus to the container without write
            access.
        """
        return f"{self.dataset_directory}:{self.container_dataset_directory}:ro"


@dataclass
class RemoteMiningJob:
    """
    A Python module run against the corpus on the host.
    """

    configuration: RemoteExecutionConfiguration
    """
    Where the run happens and what it runs against.
    """

    module: str
    """
    The module to execute inside the container.
    """

    arguments: List[str] = field(default_factory=list)
    """
    The arguments handed to the module.
    """

    command_runner: CommandRunner = field(default_factory=SubprocessCommandRunner)
    """
    Executes the assembled command.
    """

    @property
    def command(self) -> List[str]:
        """
        :return: The full invocation, from this machine to the module in the container.
        """
        return [
            "ssh",
            self.configuration.destination,
            "docker",
            "run",
            "--rm",
            "--volume",
            self.configuration.read_only_dataset_mount,
            "--env",
            f"{REPOSITORY_URL_VARIABLE_NAME}={self.configuration.repository_url}",
            "--env",
            f"{BRANCH_VARIABLE_NAME}={self.configuration.branch}",
            "--env",
            f"{DATASET_DIRECTORY_VARIABLE_NAME}="
            f"{self.configuration.container_dataset_directory}",
            self.configuration.image_name,
            self.module,
            *self.arguments,
        ]

    def run(self) -> CompletedCommand:
        """
        Execute the module against the corpus on the host.

        :return: How the run finished.
        """
        return self.command_runner.run(self.command)
