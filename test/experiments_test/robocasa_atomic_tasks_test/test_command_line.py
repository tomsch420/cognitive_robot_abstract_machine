"""
The names the scripts that run the tasks share.
"""

from __future__ import annotations

from experiments.robocasa_atomic_tasks.command_line import ResultFile


def test_a_task_writes_its_log_under_its_own_name():
    assert ResultFile.LOG.for_task("OpenDrawer") == "OpenDrawer.log"


def test_the_outcomes_of_every_task_share_one_file():
    assert ResultFile.OUTCOMES.for_task("OpenDrawer") == ResultFile.OUTCOMES
