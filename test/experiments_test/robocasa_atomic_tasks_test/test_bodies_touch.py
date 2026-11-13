"""
Tests of observing whether bodies touch, in the world and in the physics.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from cramph.context import StatechartContext
from cramph.data_types import ObservationStateValues
from cramph.executor import StatechartExecutor, SteppedSimulationPacer
from cramph.statechart import Statechart
from experiments.robocasa_atomic_tasks.skills import BodiesTouch
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from ...pytest_environment import runs_in_continuous_integration
from .conftest import box_body

# %% a block above a base


@dataclass
class BlockAboveABase:
    """
    A world with a block, free to move, above a base fixed to the world.
    """

    world: World
    """
    The world itself.
    """

    base: Body
    """
    The base, 10 cm thick, its top 5 cm above the world's origin.
    """

    block: Body
    """
    The block, 4 cm thick.
    """


def block_above_a_base(gap: float) -> BlockAboveABase:
    """
    :param gap: How far above the base's top the block's underside lies, in metres.
    :return: The world with the block and the base.
    """
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("root"))
        world.add_kinematic_structure_entity(root)
        base = box_body(world, root, "base", Scale(0.3, 0.3, 0.1), np.zeros(3))
        block = Body(
            name=PrefixedName("block"),
            collision=ShapeCollection([Box(scale=Scale(0.04, 0.04, 0.04))]),
        )
        drive = Connection6DoF.create_with_dofs(world=world, parent=root, child=block)
        world.add_connection(drive)
    drive.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        z=0.05 + gap + 0.02, reference_frame=root
    )
    return BlockAboveABase(world=world, base=base, block=block)


def observation_of_the_block_on_the_base(
    block_above_a_base: BlockAboveABase,
    simulation: MujocoSim | None = None,
    ticks: int = 1,
) -> ObservationStateValues:
    """
    :param block_above_a_base: The world with the block and the base.
    :param simulation: The physics the world is synchronized with, if any.
    :param ticks: How many cycles the node observes for.
    :return: What a node observing whether the block touches the base observes last.
    """
    touching = BodiesTouch(
        name="block on the base",
        bodies=[block_above_a_base.block],
        others=[block_above_a_base.base],
    )
    executor = StatechartExecutor(
        context=StatechartContext(world=block_above_a_base.world),
        **(
            {}
            if simulation is None
            else {"pacer": SteppedSimulationPacer(simulation=simulation)}
        ),
    )
    statechart = Statechart(context=executor.context)
    statechart.add_node(node=touching)
    executor.compile(statechart=statechart)
    for _ in range(ticks):
        executor.tick()
    return touching.observation_state


# %% touching in the world


def test_a_block_resting_on_a_base_touches_it():
    observation = observation_of_the_block_on_the_base(block_above_a_base(gap=0.0))

    assert observation == ObservationStateValues.TRUE


def test_a_block_above_a_base_does_not_touch_it():
    observation = observation_of_the_block_on_the_base(block_above_a_base(gap=0.05))

    assert observation == ObservationStateValues.FALSE


# %% touching in the physics


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
def test_a_block_the_physics_dropped_onto_a_base_touches_it():
    world_with_the_block = block_above_a_base(gap=0.02)
    simulation = MujocoSim(world=world_with_the_block.world, headless=True)
    simulation.start_stepped_simulation()
    try:
        observation = observation_of_the_block_on_the_base(
            world_with_the_block, simulation=simulation, ticks=200
        )
    finally:
        simulation.stop_simulation()

    assert observation == ObservationStateValues.TRUE
