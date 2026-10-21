"""
The Toyota HSR picks a milk carton up from a table of the kitchen, drives along the
table carrying it over the cooktop, and sets it down on the cooktop's far side,
simulated in MuJoCo.

The kitchen is semantic_digital_twin's predetermined
:class:`~semantic_digital_twin.predefined_maps.kitchen_environment.KitchenEnvironment`.
The HSR's base, arm, head and hand are driven by servos, and the carton is held by
nothing but the friction between it and the fingers: it is neither moved under the hand
in the world model nor fastened to it in MuJoCo.

Run it with::

    python -m experiments.hsr_kitchen_mujoco.demo
"""

from __future__ import annotations

import math
from datetime import timedelta
from enum import StrEnum

import mujoco
from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import PhysicallySimulatedRobot
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from semantic_digital_twin.adapters.multi_sim import (
    ContactDimensionality,
    MujocoGeom,
    MujocoSim,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.robots.hsrb import HSRB, HSRBJoint
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.contact import ContactParameters
from semantic_digital_twin.world_description.geometry import (
    Box,
    Color,
    Scale,
    VolumetricBoundingBox,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% the scene


class SceneBody(StrEnum):
    """
    The bodies of the scene the demo refers to, by their names.
    """

    TABLE = "cooking_table"
    """
    The kitchen's cooking table against the back wall, where the milk stands.
    """

    MILK = "milk"
    """
    The milk carton the HSR picks up and sets down.
    """


MILK_SCALE = Scale(0.06, 0.06, 0.2)
"""
The milk carton's extents: along the gripper's approach, across the gripper, and its
height.
"""

MILK_COLOR = Color(0.95, 0.95, 0.95)
"""
The milk carton's colour.
"""

MILK_START_X = 0.8
"""
Where along the table the milk stands, in the kitchen's x, clear of the cooktop in the
table's middle.
"""

EDGE_INSET = 0.1
"""
How far in from the table's edge facing the HSR the milk stands, in metres: close
enough to reach without leaning over the table.
"""

STANDING_DISTANCE = 0.45
"""
How far from the table's edge the HSR's base stands, in metres: clear of the edge, with
the milk within the arm's reach.
"""

SETTLING_CLEARANCE = 0.002
"""
How far above the table top the milk is put, so it settles onto it rather than starting
inside it.
"""

CARRYING_DISTANCE = 0.95
"""
How far along the table the HSR carries the milk, in the kitchen's x: from the left of
the cooktop, over it, to its right, where the base still stands clear of the middle
wall.
"""

RELEASE_CLEARANCE = 0.01
"""
How far above the table the HSR lets the milk go, in metres: pressed onto the table by
a hand that tracks its target only so closely, the carton tips in the hand and falls
over once it is let go; dropped from a centimetre it lands upright.
"""


def table_extent(world: World) -> VolumetricBoundingBox:
    """
    :return: The axis-aligned box the table occupies in the kitchen's frame.
    """
    body = world.get_body_by_name(SceneBody.TABLE)
    return body.collision.as_bounding_box_collection_in_frame(world.root).bounding_box()


def start_pose(world: World) -> HomogeneousTransformationMatrix:
    """
    :return: Where the milk stands on the table, near its edge facing the room. Its x
        axis points away from the HSR, the direction the grasp at its origin
        approaches it in.
    """
    extent = table_extent(world)
    return HomogeneousTransformationMatrix.from_xyz_rpy(
        x=MILK_START_X,
        y=extent.min_y + EDGE_INSET,
        z=extent.max_z + MILK_SCALE.z / 2 + SETTLING_CLEARANCE,
        yaw=math.pi / 2,
        reference_frame=world.root,
    )


def goal_pose(world: World) -> Pose:
    """
    :return: Where the milk is set down: :data:`CARRYING_DISTANCE` further along the
        table than where it stood, turned as it was.
    """
    extent = table_extent(world)
    return Pose.from_xyz_rpy(
        x=MILK_START_X + CARRYING_DISTANCE,
        y=extent.min_y + EDGE_INSET,
        z=extent.max_z + MILK_SCALE.z / 2 + RELEASE_CLEARANCE,
        yaw=math.pi / 2,
        reference_frame=world.root,
    )


def standing_pose(
    world: World, milk_pose: Pose | HomogeneousTransformationMatrix
) -> Pose:
    """
    :param milk_pose: Where on the table the milk stands or is to be set down.
    :return: Where the HSR stands to reach the milk there: in front of the table,
        facing it, with its hand rather than its base in line with the milk.
    """
    extent = table_extent(world)
    return Pose.from_xyz_rpy(
        x=float(milk_pose.position.to_np()[0]) + hand_offset(world),
        y=extent.min_y - STANDING_DISTANCE,
        yaw=math.pi / 2,
        reference_frame=world.root,
    )


def hand_offset(world: World) -> float:
    """
    :return: How far to its left of its base's centre the HSR carries its hand, in
        metres: the arm swings about a flex joint mounted beside the torso, and the
        base has to make up for it.
    """
    robot = world.get_semantic_annotations_by_type(HSRB)[0]
    flexed_link = world.get_connection_by_name(HSRBJoint.ARM_FLEX).child
    return float(world.compute_forward_kinematics_np(robot.root, flexed_link)[1, 3])


# %% building the world and the plan


def build_world() -> World:
    """
    :return: The kitchen with the HSR standing in front of the table, and the milk on
        the table.
    """
    world = KitchenEnvironment().get_world()
    robot = RobotSpecification(HSRB).spawn(world)
    robot.root.parent_connection.origin = standing_pose(
        world, start_pose(world)
    ).homogeneous_matrix
    with world.modify_world():
        _stand_the_milk_on_the_table(world)
    return world


def _stand_the_milk_on_the_table(world: World) -> None:
    """
    Stand the milk on the table, free to be picked up and held by friction.

    :param world: The world being built, already open for modification.
    """
    body = Body(name=PrefixedName(SceneBody.MILK))
    shapes = ShapeCollection(
        [
            Box(
                origin=HomogeneousTransformationMatrix(reference_frame=body),
                scale=MILK_SCALE,
                color=MILK_COLOR,
            )
        ],
        reference_frame=body,
    )
    body.collision = shapes
    body.visual = shapes
    world.add_kinematic_structure_entity(body)
    world.add_connection(
        Connection6DoF.create_with_dofs(
            world=world,
            parent=world.root,
            child=body,
            name=PrefixedName(f"{SceneBody.MILK}_connection"),
            parent_T_connection_expression=start_pose(world),
        )
    )
    ContactParameters.create_for_grasped_object().apply_to([body])
    world.add_semantic_annotations([Milk(root=body)])


def build_plan(context: Context) -> Plan:
    """
    Pick the milk up, drive along the table holding it, and set it down there.

    :param context: The context the actions are built in.
    :return: The plan.
    """
    robot = context.robot
    milk = context.world.get_semantic_annotations_by_type(Milk)[0]
    goal = goal_pose(context.world)
    return sequential(
        [
            ParkArmsAction(robot.all_arms),
            PickUpAction(GraspCandidate.from_body_origin(milk), robot.all_arms[0]),
            NavigateAction(standing_pose(context.world, goal)),
            PlaceAction(milk, goal),
            ParkArmsAction(robot.all_arms),
        ],
        context=context,
    ).plan


# %% running it in MuJoCo

STEP_SIZE = 1e-3
"""
How much simulated time one physics step covers, in seconds.
"""

SETTLING_DURATION = timedelta(seconds=1)
"""
How long the scene is left to settle before the HSR starts and after it is done, so the
milk rests on the table.
"""

HEADLESS = False
"""
Whether the simulation runs without MuJoCo's own viewer.
"""

FRICTION_CONE = mujoco.mjtCone.mjCONE_ELLIPTIC
"""
How MuJoCo bounds a contact's friction: the elliptic cone is the exact one; the default
pyramid only approximates it and lets a squeezed object creep out of the hand.
"""

IMPEDANCE_RATIO = 10.0
"""
How much stiffer MuJoCo makes friction than the push along the contact normal: at the
default of 1 a held object slides out under its own weight despite a firm squeeze.
"""

NO_SLIP_ITERATIONS = 10
"""
How many passes MuJoCo spends removing the slip its soft contacts leave, so a held
object stays where the fingers hold it.
"""


def run(world: World, headless: bool = HEADLESS) -> None:
    """
    Carry out the plan against MuJoCo physics, the milk held by friction alone.

    :param world: The world :func:`build_world` built.
    :param headless: Whether to run without MuJoCo's viewer.
    """
    robot = world.get_semantic_annotations_by_type(HSRB)[0]
    context = Context(world=world, robot=robot, evaluate_conditions=False)
    plan = build_plan(context)

    _resist_turning_in_the_hand(world.get_body_by_name(SceneBody.MILK))
    simulation = MujocoSim(
        world=world,
        headless=headless,
        step_size=STEP_SIZE,
        cone=FRICTION_CONE,
        impratio=IMPEDANCE_RATIO,
        noslip_iterations=NO_SLIP_ITERATIONS,
    )
    simulation.start_stepped_simulation()
    try:
        simulation.step_simulation(SETTLING_DURATION)
        with PhysicallySimulatedRobot(simulation=simulation):
            plan.perform()
        simulation.step_simulation(SETTLING_DURATION)
    finally:
        simulation.stop_simulation()


def _resist_turning_in_the_hand(held: Body) -> None:
    """
    Have MuJoCo resolve the twisting friction on a held body, so it does not swing about
    the line the fingers hold it along.

    :param held: The body the hand holds.
    """
    for shape in held.collision.shapes:
        shape.add_simulator_property(
            MujocoGeom(
                contact_dimensionality=ContactDimensionality.SLIDING_AND_TORSIONAL
            )
        )


if __name__ == "__main__":
    run(build_world())
