"""
Copying the state of the world a robot acts in into RoboCasa's own simulation.
"""

from __future__ import annotations

import pytest

from experiments.robocasa_goals.exceptions import NotARoboCasaFixture
from experiments.robocasa_task_execution.counter_to_sink import RoboCasaObjectName
from semantic_digital_twin.world_description.connections import RevoluteConnection

# %% objects


def test_an_object_moved_in_the_world_stands_there_in_robocasa(episode, mirror):
    mirror.synchronize()
    body = episode.target_object.root
    body.parent_connection.origin = episode.placement_pose().homogeneous_matrix

    mirror.synchronize()

    environment = episode.environment
    position = environment.sim.data.body_xpos[
        environment.obj_body_id[RoboCasaObjectName.TARGET]
    ]
    assert position.tolist() == pytest.approx(body.global_pose.to_np()[:3, 3].tolist())


def test_an_object_is_named_as_robocasa_names_it(episode, mirror):
    assert (
        mirror.object_name_of(episode.target_object.root) == RoboCasaObjectName.TARGET
    )


# %% fixtures


def test_a_fixture_joint_moved_in_the_world_is_moved_in_robocasa(episode, mirror):
    simulation = episode.environment.sim
    model = simulation.model
    hinge, position = next(
        (connection, 0.2)
        for connection in episode.task.world.connections
        if isinstance(connection, RevoluteConnection)
        and connection.name.name in model.joint_names
        and model.jnt_range[model.joint_name2id(connection.name.name)][0]
        < 0.2
        < model.jnt_range[model.joint_name2id(connection.name.name)][1]
    )
    hinge.position = position

    mirror.synchronize()

    assert simulation.data.get_joint_qpos(hinge.name.name) == pytest.approx(position)


def test_a_fixture_is_found_by_its_body(episode, mirror):
    sink = episode.environment.sink

    assert (
        mirror.fixture_of(episode.task.world.get_body_by_name(sink.root_body)) is sink
    )


def test_a_body_of_no_fixture_has_no_fixture(episode, mirror):
    with pytest.raises(NotARoboCasaFixture):
        mirror.fixture_of(episode.target_object.root)
