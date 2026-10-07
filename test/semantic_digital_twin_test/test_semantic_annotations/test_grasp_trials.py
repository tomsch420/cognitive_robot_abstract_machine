"""
Trying grasps drawn from the statement of where an object may be grasped.
"""

from dataclasses import dataclass, field

import pytest
from krrood.parametrization.exceptions import UnboundedParameterError
import trimesh
from typing_extensions import List

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.grasp_trials import GraspTrier, GraspTrials
from semantic_digital_twin.grasping.surface_grasp import GraspResult
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Handle,
    Milk,
    Mug,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Mesh
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% fixtures


@dataclass
class RecordingTrier(GraspTrier):
    """
    Lifts the object by every grasp, and remembers which grasps it was given.
    """

    tried: List[GraspCandidate] = field(default_factory=list)
    """
    The grasps tried so far.
    """

    def try_grasp(self, grasp: GraspCandidate) -> GraspResult:
        self.tried.append(grasp)
        return GraspResult(
            lifted=True,
            object_rise=0.2,
            translational_slip=0.0,
            rotational_slip=0.0,
            motion_completed=True,
        )


def in_world(annotation_type, shape: trimesh.Trimesh, name: str):
    """
    :return: An annotation of ``annotation_type`` on a body colliding as ``shape``, in
        a world of its own.
    """
    body = Body(name=PrefixedName(name, prefix="grasp_trials"))
    body.collision = ShapeCollection(
        [
            Mesh.from_trimesh(
                mesh=shape,
                origin=HomogeneousTransformationMatrix(reference_frame=body),
            )
        ],
        reference_frame=body,
    )
    annotation = annotation_type(root=body)
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_semantic_annotation(annotation)
    return annotation


@pytest.fixture
def carton() -> Milk:
    return in_world(Milk, trimesh.creation.box(extents=[0.06, 0.08, 0.2]), "carton")


@pytest.fixture
def mug_with_handle() -> Mug:
    handle_shape = trimesh.creation.box(extents=[0.03, 0.01, 0.04])
    handle_shape.apply_translation([0.055, 0.0, 0.0])
    mug = in_world(
        Mug,
        trimesh.util.concatenate(
            [
                trimesh.creation.annulus(r_min=0.035, r_max=0.04, height=0.1),
                handle_shape,
            ]
        ),
        "mug",
    )
    Handle.create_from_part_of_shape(mug, handle_shape)
    return mug


# %% trials


def test_drawn_grasps_lie_in_the_regions_the_annotation_states(carton):
    trials = GraspTrials(graspable=carton, trier=RecordingTrier(), number_of_trials=30)
    [region] = carton.surface_grasp_regions()

    grasps = trials.drawn_grasps()

    assert len(grasps) == trials.number_of_trials
    for grasp in grasps:
        for value, parameter_range in (
            (grasp.azimuth, region.azimuth),
            (grasp.height, region.height),
            (grasp.depth, region.depth),
            (grasp.pitch, region.pitch),
            (grasp.roll, region.roll),
        ):
            assert parameter_range.lower <= value < parameter_range.upper


def test_every_tried_grasp_is_recorded_with_its_result(carton):
    trier = RecordingTrier()
    trials = GraspTrials(graspable=carton, trier=trier, number_of_trials=5)

    records = list(trials.run())

    assert len(records) == len(trier.tried)
    for record in records:
        assert record.grasp.result.lifted
        assert record.grasped_part == type(carton).__name__


def test_an_object_with_a_handle_is_tried_at_its_handle(mug_with_handle):
    trier = RecordingTrier()
    trials = GraspTrials(graspable=mug_with_handle, trier=trier, number_of_trials=5)

    records = list(trials.run())

    assert records
    for grasp in trier.tried:
        assert grasp.graspable is mug_with_handle.handle
    for record in records:
        assert record.grasped_part == Handle.__name__


def test_trials_can_ask_only_for_lifting_grasps(carton):
    trials = GraspTrials(
        graspable=carton,
        trier=RecordingTrier(),
        number_of_trials=1,
        require_lifting=True,
    )

    with pytest.raises(UnboundedParameterError):
        trials.drawn_grasps()
