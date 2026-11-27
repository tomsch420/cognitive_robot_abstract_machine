"""
Where the grasps that turn a knob by the bar on its face lie.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    KnobWithHandle,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import ScrewAxisConnection
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedomLimits,
)
from semantic_digital_twin.world_description.geometry import Box, Cylinder, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% a knob with a bar on its face


@dataclass(kw_only=True)
class KnobOnAPanel:
    """
    A knob turning about the vertical, whose face is its top, with a bar standing on it.
    """

    knob: KnobWithHandle
    """
    The knob.
    """

    face_height: float
    """
    The height of the knob's face, in metres.
    """

    bar_height: float
    """
    The height of the bar's middle, in metres.
    """

    bar_scale: Scale
    """
    The bar's extents.
    """


@pytest.fixture
def knob_on_a_panel() -> KnobOnAPanel:
    """
    A knob 2.2 cm high and 5.2 cm wide with a bar 1.3 cm high on its face, as an oven's.
    """
    knob_width, knob_height = 0.052, 0.022
    bar_scale = Scale(0.014, 0.037, 0.013)
    face_height = knob_height / 2
    bar_height = face_height + bar_scale.z / 2
    world = World()
    with world.modify_world():
        panel = Body(name=PrefixedName("panel"))
        world.add_kinematic_structure_entity(panel)
        body = Body(name=PrefixedName("knob"))
        body.collision = ShapeCollection(
            [
                Cylinder(
                    width=knob_width,
                    height=knob_height,
                    origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                        reference_frame=body
                    ),
                ),
                Box(
                    scale=bar_scale,
                    origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                        z=bar_height, reference_frame=body
                    ),
                ),
            ],
            reference_frame=body,
        )
        world.add_connection(
            ScrewAxisConnection.create_with_dofs(
                world=world,
                parent=panel,
                child=body,
                screw_axis=ScrewAxisConnection.create_revolute_screw_axis(
                    axis=Vector3.Z()
                ),
                dof_limits=DegreeOfFreedomLimits.from_position_range_and_speed(
                    lower_position=0.0, upper_position=5.0, maximum_speed=1.0
                ),
            )
        )
    knob = KnobWithHandle.split_from(body)
    with world.modify_world():
        world.add_semantic_annotation(knob)
    return KnobOnAPanel(
        knob=knob, face_height=face_height, bar_height=bar_height, bar_scale=bar_scale
    )


def grasp_height(knob: KnobWithHandle, **placement: float) -> float:
    """
    :param knob: A knob with a bar on its face.
    :param placement: How the grasps are placed, see
        :meth:`KnobWithHandle.turning_grasp_candidates`.
    :return: The height of the untilted grasp, which comes first.
    """
    grasp = knob.turning_grasp_candidates(**placement)[0]
    world = knob.root._world
    return float(world.transform(grasp.grasp_pose, world.root).to_np()[2, 3])


# %% keeping the fingers off the face


def test_a_grasp_leaves_fingers_reaching_past_the_bar_clear_of_the_face(
    knob_on_a_panel: KnobOnAPanel,
):
    fingers_reach = knob_on_a_panel.bar_scale.z
    face_clearance = 0.002

    height = grasp_height(
        knob_on_a_panel.knob,
        fingers_reach=fingers_reach,
        face_clearance=face_clearance,
    )

    assert height == pytest.approx(
        knob_on_a_panel.face_height + fingers_reach + face_clearance
    )


def test_a_grasp_lies_at_its_depth_when_the_fingers_stop_short_of_the_face(
    knob_on_a_panel: KnobOnAPanel,
):
    depth = 0.25

    height = grasp_height(knob_on_a_panel.knob, depth=depth, fingers_reach=0.0)

    assert height == pytest.approx(
        knob_on_a_panel.bar_height - depth * knob_on_a_panel.bar_scale.z
    )
