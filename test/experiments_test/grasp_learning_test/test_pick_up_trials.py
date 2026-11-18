"""
Tests of trying whole pick-ups of the object of a pick and place task from a starting
world stored in the database.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from sqlalchemy import select

pytest.importorskip("robosuite", reason="robosuite is not installed")

from .test_kitchen_grasp_trials import episode, kitchen, simulates_physics
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.exceptions import (
    NoPickUpTrial,
    NoStoredWorld,
    StartingWorldDoesNotMatch,
)
from experiments.grasp_learning.pick_up_trials import (
    PickUpParameterSpace,
    PickUpPerformer,
    StoredStartingWorld,
)
from experiments.grasp_learning.records import (
    PickUpParameters,
    PickUpStage,
    PickUpTrial,
    StandOffset,
)
from experiments.robocasa_atomic_tasks.stored_runs import (
    SimulatedMoment,
    StoredTaskRun,
)
from experiments.orm.ormatic_interface import PickUpTrialDAO
from krrood.ormatic.data_access_objects.helper import to_dao
from krrood.parametrization.model_registries import UniformPriorRegistry
from krrood_spatial_math.spatial_types import Twist, Vector3
from semantic_digital_twin.grasping.object_relative_grasp import GraspResult
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp
from semantic_digital_twin.world import World

# %% the database


@pytest.fixture
def database() -> GraspDatabase:
    """
    An empty database in memory.
    """
    return GraspDatabase.connect("sqlite:///:memory:")


@pytest.fixture
def stored_world_id(episode, database) -> int:
    """
    The database id the episode's world is stored under.
    """
    dao = to_dao(episode.world)
    database.session.add(dao)
    database.session.commit()
    return dao.database_id


# %% the kitchen cut down


def test_the_holding_fixture_of_the_cut_down_kitchen_is_the_sink(kitchen, episode):
    """
    The fixture the broccoli lies in is still found once the rest of the kitchen is cut
    away.
    """
    assert kitchen.holding_fixture(episode) is episode.environment.sink


# %% drawing pick-ups


@pytest.fixture(scope="module")
def drawn(kitchen, episode) -> list[PickUpParameters]:
    """
    Pick-ups drawn uniformly from the default space.
    """
    np.random.seed(0)
    return PickUpParameterSpace().drawn_parameters(
        graspable=episode.graspable(kitchen.task.picked_object),
        number_of_pick_ups=20,
        model_registry=UniformPriorRegistry(),
    )


def test_pick_ups_are_drawn(drawn):
    """
    The statement of the default space is answered at all.
    """
    assert len(drawn) > 0


def test_every_drawn_pick_up_lies_in_the_space(drawn):
    """
    Every parameter, nested ones included, lies between the bounds of the space.
    """
    space = PickUpParameterSpace()
    for parameters in drawn:
        offset = parameters.stand_offset
        for value, bounds in [
            (offset.forward, space.forward),
            (offset.sideways, space.sideways),
            (offset.turn, space.turn),
            (parameters.torso_height, space.torso_height),
            (parameters.opening, space.opening),
            (parameters.closing_force, space.closing_force),
            (parameters.approach_clearance, space.approach_clearance),
            (parameters.approach_velocity, space.approach_velocity),
            (parameters.settling_duration, space.settling_duration),
            (parameters.lift_velocity, space.lift_velocity),
            (parameters.grasp.pitch, space.pitch),
            (parameters.grasp.height, space.height),
        ]:
            assert bounds.lower <= value < bounds.upper


def test_every_drawn_grasp_comes_from_the_side_of_the_space(kitchen, episode, drawn):
    """
    The side a grasp comes from is bounded in the world's frame, whatever way the object
    is turned.
    """
    space = PickUpParameterSpace()
    world_T_object = episode.graspable(kitchen.task.picked_object).root.global_pose
    world_T_object = world_T_object.to_np()
    object_yaw = math.atan2(world_T_object[1, 0], world_T_object[0, 0])
    for parameters in drawn:
        side = parameters.grasp.yaw + object_yaw
        assert space.side.lower <= side < space.side.upper


# %% the starting world


def test_a_stored_world_puts_its_state_into_the_episode(
    episode, database, stored_world_id
):
    """
    Putting a stored world into the episode undoes a change made after storing it.
    """
    world = episode.world
    [spout] = [
        degree_of_freedom
        for degree_of_freedom in world.degrees_of_freedom
        if degree_of_freedom.name.name == "sink_main_group_spout_joint"
    ]
    stored_position = world.state[spout.id].position
    world.state[spout.id].position = stored_position + 0.5
    world.notify_state_change()

    StoredStartingWorld(database=database, world_id=stored_world_id).put_into(
        world=world
    )

    assert world.state[spout.id].position == stored_position


def test_a_world_with_other_degrees_of_freedom_does_not_match(
    database, stored_world_id
):
    """
    A world without the stored world's degrees of freedom is refused.
    """
    with pytest.raises(StartingWorldDoesNotMatch):
        StoredStartingWorld(database=database, world_id=stored_world_id).put_into(
            world=World()
        )


def test_reading_a_world_no_world_is_stored_under_fails(database):
    """
    Reading a world by a number nothing is stored under raises.
    """
    with pytest.raises(NoStoredWorld):
        database.starting_world(database_id=1)


# %% storing a trial


def pick_up_trial(kitchen) -> PickUpTrial:
    """
    :return: A trial that lifted the object, with a run of two moments of one degree of
        freedom.
    """
    return PickUpTrial(
        starting_world_id=1,
        parameters=PickUpParameters(
            stand_offset=StandOffset(forward=0.1, sideways=-0.05, turn=0.1),
            torso_height=0.2,
            grasp=SurfaceGrasp(yaw=4.0, height=0.6, depth=0.02, pitch=0.3, roll=0.1),
            opening=0.9,
            closing_force=40.0,
            approach_clearance=0.05,
            approach_velocity=0.05,
            settling_duration=0.5,
            lift_velocity=0.1,
        ),
        result=GraspResult(
            object_raised=True,
            motion_completed=True,
            object_displacement=Twist(translation=Vector3(0.0, 0.0, 0.2)),
            slip=Twist(translation=Vector3(0.0, 0.0, 0.0)),
        ),
        failed_stage=PickUpStage.LEAVING,
        run=StoredTaskRun(
            task=kitchen.task.robocasa_task_name,
            layout=kitchen.task.layout,
            style=kitchen.task.style,
            seed=kitchen.task.seed,
            robot=kitchen.task.robot,
            degree_of_freedom_names=["lever"],
            moments=[
                SimulatedMoment(simulation_time=0.0, positions=[0.0]),
                SimulatedMoment(simulation_time=0.5, positions=[0.1]),
            ],
            judgement=None,
        ),
    )


def test_a_stored_trial_reads_back_as_it_was_stored(kitchen, database):
    """
    A trial reads back with the parameters, stage and moments it was stored with.
    """
    trial = pick_up_trial(kitchen)
    database.add_trial(trial=trial)
    [stored_id] = database.session.scalars(select(PickUpTrialDAO.database_id)).all()

    stored = database.pick_up_trial(database_id=stored_id)

    assert stored.parameters == trial.parameters
    assert stored.failed_stage == trial.failed_stage
    assert stored.run.moments == trial.run.moments


def test_reading_a_trial_no_trial_is_stored_under_fails(database):
    """
    Reading a trial by a number nothing is stored under raises.
    """
    with pytest.raises(NoPickUpTrial):
        database.pick_up_trial(database_id=1)


# %% performing a pick-up


@simulates_physics
def test_a_performed_pick_up_completed_its_motion_exactly_when_no_stage_failed(
    kitchen, drawn
):
    """
    A pick-up's motion counts as completed exactly when none of its stages failed.
    """
    episode = kitchen.compose()
    graspable = episode.graspable(kitchen.task.picked_object)
    with episode.executed(physics=True):
        performer = PickUpPerformer(
            episode=episode,
            task=kitchen.task,
            graspable=graspable,
            holding_fixture_bodies=episode.fixtures.bodies_of(
                kitchen.holding_fixture(episode)
            ),
            start=episode.simulation.take_snapshot(),
        )
        trial = performer.perform(parameters=drawn[0])

    assert trial.result.motion_completed == (trial.failed_stage is None)
