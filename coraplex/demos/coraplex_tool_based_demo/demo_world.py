"""
Shared world and tool setup for the tool-based action demos (cutting, pouring,
mixing, wiping).
"""

import math
import os

from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.datastructures.enums import Arms
from coraplex.view_manager import ViewManager

OBJECTS_DIRECTORY = os.path.join(
    os.path.dirname(__file__), "..", "..", "resources", "objects"
)

TARGET_POSITION_XYZ = (2.4, 2.2, 1.0)
"""
Position of the manipulated object on the apartment kitchen counter.
"""

BASE_POSITION_XYZ = (1.85, 2.2, 0.0)
"""
Base position in front of the kitchen counter, facing the target.
"""

CUT_MOUNT = {"z": 0.08, "pitch": -math.pi / 2}
"""
Knife mount transform on the right gripper's tool frame.
"""

MIX_MOUNT = {"z": -0.08, "pitch": math.pi / 2}
"""
Whisk mount transform on the right gripper's tool frame.
"""

POUR_MOUNT = {"z": -0.08}
"""
Cup mount transform on the right gripper's tool frame.
"""

WIPE_MOUNT = {"pitch": math.pi / 2}
"""
Sponge mount transform on the right gripper's tool frame.
"""


def parse_object(stl_file_name: str) -> World:
    """
    :return: A world containing the mesh from the demo resources.
    """
    return STLParser(os.path.join(OBJECTS_DIRECTORY, stl_file_name)).parse()


def attach_tool(
    world: World, robot: AbstractRobot, arm: Arms, tool_world: World, mount: dict
) -> Body:
    """
    Rigidly attach a tool mesh to the arm's tool frame.

    :param world: The world the robot lives in.
    :param robot: The robot holding the tool.
    :param arm: The arm the tool is mounted on.
    :param tool_world: The world containing the parsed tool mesh.
    :param mount: Keyword arguments describing the mount transform.
    :return: The tool's root body inside ``world``.
    """
    tool_frame = ViewManager.get_end_effector_view(arm, robot).tool_frame
    connection = FixedConnection(
        parent=tool_frame,
        child=tool_world.root,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            reference_frame=tool_frame, **mount
        ),
    )
    with world.modify_world():
        world.merge_world(tool_world, connection)
    return connection.child


def attach_sponge(world: World, robot: AbstractRobot, arm: Arms) -> Body:
    """
    Attach a primitive box sponge to the arm's tool frame.

    :return: The sponge body inside ``world``.
    """
    sponge_body = Body(
        name=PrefixedName("sponge"),
        collision=ShapeCollection([Box(scale=Scale(0.05, 0.05, 0.05))]),
        visual=ShapeCollection([Box(scale=Scale(0.05, 0.05, 0.05))]),
    )
    tool_frame = ViewManager.get_end_effector_view(arm, robot).tool_frame
    connection = FixedConnection(
        parent=tool_frame,
        child=sponge_body,
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            reference_frame=tool_frame, **WIPE_MOUNT
        ),
    )
    with world.modify_world():
        world.add_kinematic_structure_entity(sponge_body)
        world.add_connection(connection)
    return sponge_body
