"""
RoboCasa's verdict on an episode with the state of its simulation it judged.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pytest

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.judge import JudgedJointPosition, RoboCasaJudge
from robosuite.utils.binding_utils import MjSim

# %% a kitchen whose simulation holds a joint of RoboCasa's robot


@dataclass
class KitchenWithARobotJoint:
    """
    Mimics a RoboCasa environment whose simulation holds a lever, a drawer and a mug of
    the kitchen, and one joint of RoboCasa's robot.
    """

    sim: MjSim
    """
    The simulation of the kitchen.
    """


@pytest.fixture
def kitchen_with_a_robot_joint() -> KitchenWithARobotJoint:
    model = Path(__file__).parent / "kitchen_with_a_robot_joint.xml"
    return KitchenWithARobotJoint(sim=MjSim.from_xml_string(model.read_text()))


# %% the joints judged


def test_the_judged_joints_are_the_kitchens_hinges_and_slides_without_the_robots(
    kitchen_with_a_robot_joint, panda_omron
):
    data = kitchen_with_a_robot_joint.sim.data
    data.set_joint_qpos("sink_main_group_handle_joint", 0.38)
    data.set_joint_qpos("drawer_slide", 0.1)
    data.set_joint_qpos("robot0_joint1", 1.0)
    [arm] = panda_omron.all_arms
    judge = RoboCasaJudge(
        environment=kitchen_with_a_robot_joint,
        world=panda_omron._world,
        robot=panda_omron,
        arm=arm,
    )

    assert judge.judged_joint_positions() == [
        JudgedJointPosition(joint_name="sink_main_group_handle_joint", position=0.38),
        JudgedJointPosition(joint_name="drawer_slide", position=0.1),
    ]
