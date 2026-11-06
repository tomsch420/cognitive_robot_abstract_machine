"""
Tests of joint goals observed against the positions the physics measures.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from ...pytest_environment import runs_in_continuous_integration

from cramph.context import StatechartContext
from cramph.data_types import ObservationStateValues
from cramph.executor import StatechartExecutor, SteppedSimulationPacer
from cramph.statechart import Statechart
from experiments.robocasa_atomic_tasks.skills import MeasuredJointPosition
from giskardpy.motion_control import MotionControl
from giskardpy.motion_statechart.tasks.joint_tasks import JointState
from giskardpy.qp.qp_controller_config import QPControllerConfig
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    RevoluteConnection,
)
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedom,
    DegreeOfFreedomLimits,
)
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

pytestmark = pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)

# %% a door on a hinge


@dataclass
class DoorOnAHinge:
    """
    A world with a door turning about a vertical hinge, which nothing but the controller
    or the physics moves.
    """

    world: World
    """
    The world itself.
    """

    hinge: RevoluteConnection
    """
    The hinge the door turns about.
    """


@pytest.fixture
def door_on_a_hinge() -> DoorOnAHinge:
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("root"))
        world.add_body(root)
        door = Body(name=PrefixedName("door"))
        door.collision = ShapeCollection(
            [
                Box(
                    origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                        x=0.1, reference_frame=door
                    ),
                    scale=Scale(0.2, 0.02, 0.02),
                )
            ],
            reference_frame=door,
        )
        degree_of_freedom = DegreeOfFreedom(
            name=PrefixedName("hinge"),
            limits=DegreeOfFreedomLimits.from_position_range_and_speed(
                lower_position=-1.0, upper_position=1.0, maximum_speed=1.0
            ),
        )
        world.add_degree_of_freedom(degree_of_freedom)
        hinge = RevoluteConnection(
            name=PrefixedName("hinge"),
            parent=root,
            child=door,
            axis=Vector3.Z(reference_frame=door),
            raw_dof=degree_of_freedom,
        )
        world.add_connection(hinge)
    return DoorOnAHinge(world=world, hinge=hinge)


# %% observing the physics


def observation_after_driving_the_hinge(
    door_on_a_hinge: DoorOnAHinge,
    goal_position: float,
    left_to_the_physics: bool,
    ticks: int = 100,
) -> ObservationStateValues:
    """
    :param door_on_a_hinge: The world with the hinge.
    :param goal_position: The position the hinge is driven to.
    :param left_to_the_physics: Whether the world's positions of the hinge stay out of
        the simulation while it is driven.
    :param ticks: How many control cycles the hinge is driven for.
    :return: What the goal observes after driving the hinge.
    """
    world = door_on_a_hinge.world
    goal = MeasuredJointPosition(
        name="hinge goal",
        goal_state=JointState.from_mapping({door_on_a_hinge.hinge: goal_position}),
    )
    simulation = MujocoSim(world=world, headless=True)
    simulation.start_stepped_simulation()
    try:
        executor = StatechartExecutor(
            context=StatechartContext(world=world),
            pacer=SteppedSimulationPacer(simulation),
            extensions=[
                MotionControl(
                    qp_controller_config=QPControllerConfig.create_with_physical_simulation_defaults()
                )
            ],
        )
        statechart = Statechart(context=executor.context)
        statechart.add_node(goal)
        executor.compile(statechart=statechart)
        connections = [door_on_a_hinge.hinge] if left_to_the_physics else []
        with simulation.synchronizer.leave_to_the_physics(connections):
            for _ in range(ticks):
                executor.tick()
            observation = goal.observation_state
            assert world.state[door_on_a_hinge.hinge.raw_dof.id].position == (
                pytest.approx(goal_position, abs=goal.threshold)
            )
    finally:
        simulation.stop_simulation()
    return observation


def test_a_joint_goal_is_not_reached_while_the_physics_holds_the_joint_back(
    door_on_a_hinge,
):
    observation = observation_after_driving_the_hinge(
        door_on_a_hinge, goal_position=0.5, left_to_the_physics=True
    )

    assert observation == ObservationStateValues.FALSE


def test_a_joint_goal_is_reached_once_the_physics_moved_the_joint_there(
    door_on_a_hinge,
):
    observation = observation_after_driving_the_hinge(
        door_on_a_hinge, goal_position=0.5, left_to_the_physics=False
    )

    assert observation == ObservationStateValues.TRUE
