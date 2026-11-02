"""
The names the scripts that run the tasks share: their command-line options, the files
they write and how a run of a task ends.
"""

from __future__ import annotations

from enum import IntEnum, StrEnum

# %% running one task


class TaskRunOption(StrEnum):
    """
    The command-line options of the script that runs one task.
    """

    LAYOUT = "--layout"
    """
    The kitchen layout.
    """

    STYLE = "--style"
    """
    The visual style of the kitchen.
    """

    SEED = "--seed"
    """
    The seed RoboCasa samples the task's scene with.
    """

    PHYSICS = "--physics"
    """
    Simulate the robot and the kitchen physically in MuJoCo.
    """

    VIDEO = "--video"
    """
    Where to write a video of the episode seen from the robot.
    """

    COLLISION_AVOIDANCE = "--collision-avoidance"
    """
    Whether the robot avoids colliding with the kitchen and with itself.
    """

    ROBOT = "--robot"
    """
    The robot that performs the task.
    """

    TIME_LIMIT = "--time-limit"
    """
    How many seconds the run may take before it is killed.
    """


class TaskExitStatus(IntEnum):
    """
    The exit status of the script that runs one task, by how the task ended.
    """

    ACCOMPLISHED = 0
    """
    RoboCasa's own check found the task accomplished.
    """

    FAILED_WITH_AN_ERROR = 1
    """
    The run raised an error, as Python exits on one.
    """

    NOT_ACCOMPLISHED = 2
    """
    The robot finished, but RoboCasa's own check did not find the task accomplished.
    """


# %% what a run of every task writes


class ResultFile(StrEnum):
    """
    The files a run of every task writes into its results directory.
    """

    LOG = "{task}.log"
    """
    The log of one task.
    """

    VIDEO = "{task}.mp4"
    """
    The video of one task.
    """

    OUTCOMES = "outcomes.json"
    """
    How every task ended.
    """

    def for_task(self, task: str) -> str:
        """
        :param task: The name of a task.
        :return: The name of this file of that task.
        """
        return self.format(task=task)
