"""
Robots picking objects up in MuJoCo, held by contact alone.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import numpy as np
import pytest
import trimesh

from ...pytest_environment import runs_in_continuous_integration

from experiments.physical_pick_up.physical_simulation_preparation import (
    PR2PhysicalSimulationPreparation,
)
from experiments.physical_pick_up.pick_up import PhysicalPickUp
from experiments.physical_pick_up.objects import (
    HandleTowardsRobot,
    ObjectCannotHaveAHandleError,
    ObjectChoice,
    ObjectGeometry,
    PickUpObject,
    RoboCasaObjectDescription,
)
from experiments.physical_pick_up.pick_up_experiment import PickUpExperiment
from experiments.physical_pick_up.robots import ObjectPlacement, PR2Setup
from experiments.physical_pick_up.scene import PickUpScene
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
)
from semantic_digital_twin.api import RobotSpecification
from semantic_digital_twin.grasping.handle_finding import ProtrudingHandleFinder
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.pr2 import PR2, PR2Joint
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import (
    Body,
    GravityCompensation,
)

simulates_physics = pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)

requires_robocasa_assets = pytest.mark.skipif(
    not (RoboCasaDatasetLoader().directory / "objects").is_dir(),
    reason="the RoboCasa object assets are not downloaded",
)

# %% fixtures


@pytest.fixture
def prepared_pr2() -> PR2:
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(Body(name=PrefixedName("floor")))
    robot = RobotSpecification(PR2).spawn(world)
    PR2PhysicalSimulationPreparation(robot=robot).apply()
    return robot


@pytest.fixture(scope="module")
def bowl_scene() -> PickUpScene:
    return PickUpScene(object_description=PickUpObject.BOWL.value)


@pytest.fixture(scope="module")
def milk_experiment() -> PickUpExperiment:
    """
    An experiment with an object that collides as its own mesh, which is quick to build.
    """
    return PickUpExperiment(
        scene=PickUpScene(object_description=PickUpObject.MILK.value)
    )


# %% preparing the PR2 for physical simulation


def test_every_commanded_degree_of_freedom_is_driven_by_one_servo(prepared_pr2):
    """
    The coupled finger joints of a gripper share one degree of freedom and therefore one
    servo.
    """
    world = prepared_pr2._world
    expected = {
        connection.raw_dof
        for arm in prepared_pr2.all_arms
        for connection in arm.active_connections
    }
    expected |= {
        world.get_connection_by_name(joint).raw_dof
        for joint in (
            PR2Joint.TORSO_LIFT,
            PR2Joint.LEFT_GRIPPER_LEFT_FINGER,
            PR2Joint.RIGHT_GRIPPER_LEFT_FINGER,
        )
    }

    driven = [
        degree_of_freedom
        for actuator in world.actuators
        for degree_of_freedom in actuator.dofs
    ]

    assert set(driven) == expected
    assert len(driven) == len(expected)


def test_a_servoed_joint_without_position_limits_is_given_some(prepared_pr2):
    preparation = PR2PhysicalSimulationPreparation(robot=prepared_pr2)
    forearm_roll = prepared_pr2._world.get_connection_by_name(
        PR2Joint.LEFT_FOREARM_ROLL
    )

    limits = forearm_roll.raw_dof.limits

    assert limits.lower.position == -preparation.continuous_joint_position_limit
    assert limits.upper.position == preparation.continuous_joint_position_limit


def test_every_body_has_inertia_a_rigid_body_can_have(prepared_pr2):
    """
    No principal moment of inertia of a rigid body exceeds the sum of the other two.
    """
    for body in prepared_pr2.bodies:
        if body.inertial is None:
            continue
        moments, _ = body.inertial.inertia.to_principal_moments_and_axes()
        smallest, middle, largest = np.sort(moments.data.flatten())
        assert smallest + middle >= largest


def test_the_weight_of_every_body_is_carried(prepared_pr2):
    for body in prepared_pr2.bodies:
        assert body.get_simulator_property_of_type(GravityCompensation) == (
            GravityCompensation(fraction=1.0)
        )


# %% the objects


def test_a_mesh_in_millimeters_is_loaded_in_meters():
    description = PickUpObject.YCB_CRACKER_BOX.value

    extents = description.load_geometry().visual.extents

    assert extents == pytest.approx(
        trimesh.load_mesh(description.mesh_file).extents
        * description.meters_per_mesh_unit
    )


def test_a_tool_lies_flat_with_its_narrower_end_towards_the_robot():
    """
    A tool modeled standing upright, its wide head up, is laid along the x-axis on its
    thinnest side, its narrow handle towards the robot at negative x.
    """
    handle = trimesh.creation.box(extents=[0.02, 0.01, 0.15])
    head = trimesh.creation.box(extents=[0.06, 0.01, 0.05])
    head.apply_translation([0.0, 0.0, 0.1])
    geometry = ObjectGeometry(
        visual=trimesh.util.concatenate([handle, head]),
        collision_parts=[handle, head],
    )

    resting = geometry.transformed(HandleTowardsRobot().rotation(geometry))

    resting_handle, resting_head = resting.collision_parts
    assert resting.visual.extents == pytest.approx([0.2, 0.06, 0.01])
    assert resting_head.centroid[0] > resting_handle.centroid[0]


@requires_robocasa_assets
def test_a_robocasa_object_shows_its_meshes_without_region_boxes():
    """
    RoboCasa marks regions with invisible boxes; left in, they would be the first
    surface a grasp's line of sight meets.
    """
    geometry = PickUpObject.ROBOCASA_SPOON.value.load_geometry()

    collision = trimesh.util.concatenate(geometry.collision_parts)

    assert geometry.visual.bounds == pytest.approx(collision.bounds, abs=0.005)


def test_another_robocasa_model_keeps_the_grasping_of_its_category():
    description = PickUpObject.ROBOCASA_SPOON.value
    other_model_index = description.instance_index + 1

    other_model = description.instance(other_model_index)

    assert other_model.instance_index == other_model_index
    assert other_model.body_name != description.body_name
    assert other_model.default_grasp == description.default_grasp
    assert other_model.handle_finder == description.handle_finder


def test_the_command_line_takes_another_robocasa_model():
    parser = argparse.ArgumentParser()
    object_choice = ObjectChoice(parser)
    object_choice.add_arguments()
    other_model_index = 3

    description = object_choice.description(
        parser.parse_args(
            [
                "--object",
                PickUpObject.ROBOCASA_SPOON.name.lower(),
                "--instance",
                str(other_model_index),
            ]
        )
    )

    assert description == PickUpObject.ROBOCASA_SPOON.value.instance(other_model_index)


def test_the_command_line_refuses_a_model_of_an_object_from_a_mesh_file():
    parser = argparse.ArgumentParser()
    object_choice = ObjectChoice(parser)
    object_choice.add_arguments()
    arguments = parser.parse_args(
        ["--object", PickUpObject.MILK.name.lower(), "--instance", "1"]
    )

    with pytest.raises(SystemExit):
        object_choice.description(arguments)


# %% the scene


def test_the_bowl_stands_loose_on_the_table(bowl_scene):
    bowl = bowl_scene.graspable
    bowl_connection = bowl.root.parent_connection
    lowest_point = bowl.root.combined_mesh.bounds[0][2]
    bowl_height = bowl_scene.world.compute_forward_kinematics_np(
        bowl_scene.world.root, bowl.root
    )[2, 3]

    assert isinstance(bowl_connection, Connection6DoF)
    assert bowl_height + lowest_point == pytest.approx(
        bowl_scene.placement_area.height + bowl_scene.drop_height, abs=1e-3
    )


def test_the_bowl_collides_as_convex_parts_that_leave_it_hollow(bowl_scene):
    """
    A single convex hull would fill the bowl, leaving no wall for fingers to straddle.
    """
    parts = bowl_scene.graspable.root.collision.shapes
    filled_bowl = bowl_scene.graspable.root.visual.combined_mesh.convex_hull

    assert len(parts) > 1
    assert sum(part.mesh.volume for part in parts) < filled_bowl.volume / 2


def test_the_floor_is_a_slab_whose_top_is_at_height_zero(milk_experiment):
    """
    An object dropped off the table comes to rest on the floor instead of falling
    forever.
    """
    floor = milk_experiment.scene.world.root

    lowest, highest = floor.collision.combined_mesh.bounds

    assert highest[2] == pytest.approx(0.0)
    assert lowest[2] == pytest.approx(-milk_experiment.scene.floor_scale.z)


def test_a_handle_cannot_be_found_for_an_object_that_cannot_have_one():
    milk_with_a_handle = replace(
        PickUpObject.MILK.value, handle_finder=ProtrudingHandleFinder()
    )

    with pytest.raises(ObjectCannotHaveAHandleError):
        PickUpScene(object_description=milk_with_a_handle)


def test_the_object_is_what_its_description_says(milk_experiment):
    scene = milk_experiment.scene

    assert type(scene.graspable) is scene.object_description.semantic_annotation_type
    assert scene.graspable.root.name.name == scene.object_description.body_name


def object_footprint_middle(scene: PickUpScene) -> np.ndarray:
    """
    :return: Where the middle of the object's footprint is in the world, as x and y.
    """
    world_T_object = scene.world.compute_forward_kinematics_np(
        scene.world.root, scene.graspable.root
    )
    middle = scene.graspable.root.visual.combined_mesh.bounds.mean(axis=0)
    return (world_T_object[:3, :3] @ middle + world_T_object[:3, 3])[:2]


def test_the_object_starts_in_the_middle_of_its_placement_area(milk_experiment):
    scene = milk_experiment.scene
    middle = scene.placement_area.middle()

    assert object_footprint_middle(scene) == pytest.approx([middle.x, middle.y])


def test_an_object_placed_elsewhere_stands_there_turned(milk_experiment):
    scene = milk_experiment.scene
    placement = ObjectPlacement(
        x=scene.placement_area.x.lower, y=scene.placement_area.y.upper, yaw=0.4
    )

    scene.place_object(placement)

    world_T_object = scene.world.compute_forward_kinematics_np(
        scene.world.root, scene.graspable.root
    )
    assert object_footprint_middle(scene) == pytest.approx([placement.x, placement.y])
    assert np.arctan2(world_T_object[1, 0], world_T_object[0, 0]) == pytest.approx(
        placement.yaw
    )
    scene.place_object(scene.placement_area.middle())


def test_a_random_placement_stays_in_the_area_and_the_turn(milk_experiment):
    area = milk_experiment.scene.placement_area
    generator = np.random.default_rng(0)
    maximum_yaw = 0.5

    placements = [area.random_placement(generator, maximum_yaw) for _ in range(50)]

    for placement in placements:
        assert area.x.lower <= placement.x <= area.x.upper
        assert area.y.lower <= placement.y <= area.y.upper
        assert abs(placement.yaw) <= maximum_yaw


def test_an_object_without_decomposition_collides_as_its_mesh(milk_experiment):
    graspable = milk_experiment.scene.graspable

    (collision_shape,) = graspable.root.collision.shapes

    assert collision_shape.mesh.vertices == pytest.approx(
        graspable.root.visual.combined_mesh.vertices
    )


def test_the_object_weighs_its_described_mass(milk_experiment):
    scene = milk_experiment.scene

    assert scene.graspable.root.inertial.mass == scene.object_description.mass


# %% picking objects up


@simulates_physics
@pytest.mark.parametrize(
    "pick_up_object",
    [
        pytest.param(
            pick_up_object,
            marks=(
                [requires_robocasa_assets]
                if isinstance(pick_up_object.value, RoboCasaObjectDescription)
                else []
            ),
        )
        for pick_up_object in PickUpObject
    ],
    ids=lambda pick_up_object: pick_up_object.name,
)
def test_the_pr2_lifts_every_object_by_contact_alone(pick_up_object):
    """
    The object rises with the gripper although the world never attaches it to the
    gripper: it stays a free body below the world's root.
    """
    experiment = PickUpExperiment(
        scene=PickUpScene(object_description=pick_up_object.value)
    )

    result = experiment.run()

    connection = experiment.scene.graspable.root.parent_connection
    assert result.motion_completed
    assert result.lifted
    assert isinstance(connection, Connection6DoF)
    assert connection.parent is experiment.scene.world.root


@simulates_physics
def test_a_grip_too_weak_leaves_the_bowl_on_the_table():
    """
    Guards the premise of the test above: the bowl is held by the fingers' pressure, so
    without enough of it the same motion lifts nothing.
    """
    experiment = PickUpExperiment(
        scene=PickUpScene(robot_setup=PR2Setup(grip_torque=0.05))
    )

    result = experiment.run()

    assert result.motion_completed
    assert not result.lifted


@simulates_physics
def test_a_bowl_left_behind_slips_out_of_the_gripper_by_the_whole_lift():
    experiment = PickUpExperiment(
        scene=PickUpScene(robot_setup=PR2Setup(grip_torque=0.05))
    )

    result = experiment.run()

    assert result.translational_slip == pytest.approx(
        PhysicalPickUp.lift_height, abs=0.03
    )


@simulates_physics
def test_a_held_bowl_slips_by_much_less_than_the_lift():
    result = PickUpExperiment().run()

    assert result.translational_slip < PhysicalPickUp.lift_height / 4
