import numpy as np
from numpy.typing import NDArray
import pytest
import trimesh

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import (
    MissingReferenceFrameError,
    NoGraspGeometry,
    ReferenceFrameMismatchError,
)
from semantic_digital_twin.semantic_annotations.mixins import HasRootBody
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.semantic_annotations.natural_language import (
    NaturalLanguageWithTypeDescription,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bowl,
    Cabinet,
    Dishwasher,
    Floor,
    Handle,
    Milk,
    Mug,
    Rim,
    Spoon,
    Table,
)
from semantic_digital_twin.grasping.rim_finding import RimFinder
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Mesh, Scale
from semantic_digital_twin.world_description.inertial_properties import Inertial
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from ._statements import allowed_intervals

# %% fixtures

BOWL_INNER_RADIUS = 0.09
"""
Radius of the synthetic bowl's inner wall.
"""

BOWL_OUTER_RADIUS = 0.10
"""
Radius of the synthetic bowl's outer wall.
"""

BOWL_HEIGHT = 0.06
"""
Height of the synthetic bowl's wall.
"""

BOX_SCALE = Scale(0.1, 0.2, 0.3)
"""
Extents of the box body used by the default grasp pose tests.
"""


@pytest.fixture
def bowl(tmp_path) -> Bowl:
    """
    A bowl whose wall is an exact tube, so its rim radius is known by construction.
    """
    mesh = trimesh.creation.annulus(
        r_min=BOWL_INNER_RADIUS, r_max=BOWL_OUTER_RADIUS, height=BOWL_HEIGHT
    )
    mesh_path = tmp_path / "bowl.stl"
    mesh.export(mesh_path)
    shape = Mesh(origin=HomogeneousTransformationMatrix(), filename=str(mesh_path))
    body = Body(
        name=PrefixedName("bowl", prefix="grasp_candidates"),
        collision=ShapeCollection([shape]),
    )
    annotation = Bowl(root=body)
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_semantic_annotation(annotation)
    Rim.create_on(annotation)
    return annotation


@pytest.fixture
def milk() -> Milk:
    """
    A box-shaped body, which the default implementation grasps at its origin.
    """
    body = Body(
        name=PrefixedName("milk", prefix="grasp_candidates"),
        collision=ShapeCollection(
            [Box(origin=HomogeneousTransformationMatrix(), scale=BOX_SCALE)]
        ),
    )
    annotation = Milk(root=body)
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_semantic_annotation(annotation)
    return annotation


def axes_of(pose: Pose) -> NDArray[np.float64]:
    """
    :param pose: The pose to read the frame axes of.
    :return: The pose's x, y and z axis as the columns of a 3x3 array.
    """
    return pose.to_np()[:3, :3]


# %% default grasp poses


def test_default_grasp_candidates_are_in_the_root_frame(milk):
    for grasp in milk.grasp_candidates():
        assert grasp.grasp_pose.reference_frame is milk.root


def test_default_grasp_candidates_belong_to_the_annotation_that_offers_them(milk):
    for grasp in milk.grasp_candidates():
        assert grasp.graspable is milk


def test_default_grasp_candidates_are_at_the_root_origin(milk):
    for grasp in milk.grasp_candidates():
        np.testing.assert_allclose(
            grasp.grasp_pose.to_np()[:3, 3], np.zeros(3), atol=1e-9
        )


def test_default_grasp_candidate_count_follows_the_field(milk):
    milk.grasp_candidate_count = 7
    assert len(milk.grasp_candidates()) == 7


def test_default_grasp_candidates_differ_only_in_yaw(milk):
    for grasp in milk.grasp_candidates():
        # A pure yaw keeps the frame's z-axis on the body's z-axis.
        np.testing.assert_allclose(
            axes_of(grasp.grasp_pose)[:, 2], [0, 0, 1], atol=1e-9
        )


def test_default_grasp_candidates_approach_along_evenly_spaced_yaws(milk):
    approach_yaws = sorted(
        np.arctan2(axes_of(grasp.grasp_pose)[1, 0], axes_of(grasp.grasp_pose)[0, 0])
        for grasp in milk.grasp_candidates()
    )
    expected = np.linspace(0, 2 * np.pi, milk.grasp_candidate_count, endpoint=False)
    np.testing.assert_allclose(
        approach_yaws, np.sort(np.arctan2(np.sin(expected), np.cos(expected)))
    )


# %% rim grasp poses


def test_bowl_grasps_sit_in_the_rim_wall(bowl):
    for grasp in bowl.grasp_candidates():
        position = grasp.grasp_pose.to_np()[:3, 3]
        assert BOWL_INNER_RADIUS < np.linalg.norm(position[:2]) < BOWL_OUTER_RADIUS


def test_bowl_grasps_sit_in_the_middle_of_the_rims_height(bowl):
    lowest, highest = bowl.rim.grasp_surface().bounds
    rim_height = highest[2] - lowest[2]

    assert rim_height == pytest.approx(RimFinder().depth, abs=0.002)
    for grasp in bowl.grasp_candidates():
        height_on_rim = grasp.grasp_pose.to_np()[2, 3] - lowest[2]
        assert 0.25 * rim_height - 1e-9 <= height_on_rim <= 0.75 * rim_height + 1e-9


def test_bowl_grasps_approach_from_above(bowl):
    allowed = allowed_intervals(bowl.rim.surface_grasp_statement())
    for grasp in bowl.grasp_candidates():
        approach = axes_of(grasp.grasp_pose)[:, 0]
        assert -approach[2] >= np.cos(allowed["pitch"].simple_sets[-1].upper)


def test_bowl_grasp_fingers_close_across_the_rim_wall(bowl):
    """
    The finger axis must be close to radial, so the fingers straddle the wall rather
    than pinching along it.
    """
    allowed = allowed_intervals(bowl.rim.surface_grasp_statement())
    for grasp in bowl.grasp_candidates():
        position = grasp.grasp_pose.to_np()[:3, 3]
        radial = position / np.linalg.norm(position[:2])
        radial[2] = 0
        finger_axis = axes_of(grasp.grasp_pose)[:, 1]
        assert abs(float(np.dot(finger_axis, radial))) >= np.cos(
            allowed["roll"].simple_sets[-1].upper
        ) * np.cos(allowed["pitch"].simple_sets[-1].upper)


def test_bowl_grasps_close_on_the_middle_of_the_wall(bowl):
    allowed = allowed_intervals(bowl.rim.surface_grasp_statement())
    wall_thickness = BOWL_OUTER_RADIUS - BOWL_INNER_RADIUS

    assert bowl.rim.wall_thickness() == pytest.approx(wall_thickness, rel=0.02)
    assert allowed["depth"].simple_sets[0].lower == pytest.approx(
        0.25 * wall_thickness, rel=0.02
    )
    assert allowed["depth"].simple_sets[-1].upper == pytest.approx(
        0.75 * wall_thickness, rel=0.02
    )


# %% cutlery grasp poses


def _spoon_lying_along(length_axis: int) -> Spoon:
    """
    :param length_axis: The axis of its own frame the spoon lies along, 0 for x and 1
        for y.
    :return: A spoon whose collision box is long along that axis.
    """
    extents = [0.02, 0.02, 0.01]
    extents[length_axis] = 0.15
    body = Body(
        name=PrefixedName(f"spoon_along_{length_axis}", prefix="grasp_candidates"),
        collision=ShapeCollection(
            [Box(origin=HomogeneousTransformationMatrix(), scale=Scale(*extents))]
        ),
    )
    annotation = Spoon(root=body)
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_semantic_annotation(annotation)
    return annotation


@pytest.mark.parametrize("length_axis", [0, 1], ids=["along-x", "along-y"])
def test_cutlery_is_grasped_from_above_across_its_length(length_axis):
    """
    A piece of cutlery lies flat, so the fingers come down onto it and close across it,
    never along it.
    """
    spoon = _spoon_lying_along(length_axis)
    allowed = allowed_intervals(spoon.surface_grasp_statement())
    first_end = allowed["azimuth"].simple_sets[0]
    azimuth_spread = (first_end.upper - first_end.lower) / 2
    length_direction = np.eye(3)[length_axis]

    grasps = spoon.grasp_candidates()

    assert grasps
    for grasp in grasps:
        approach, closing = (
            axes_of(grasp.grasp_pose)[:, 0],
            axes_of(grasp.grasp_pose)[:, 1],
        )
        assert -approach[2] >= np.cos(allowed["pitch"].simple_sets[-1].upper)
        assert abs(float(np.dot(closing, length_direction))) <= np.sin(
            allowed["roll"].simple_sets[-1].upper - np.pi / 2 + azimuth_spread
        )


def test_cutlery_without_a_shape_offers_no_grasp():
    body = Body(name=PrefixedName("shapeless_spoon", prefix="grasp_candidates"))
    spoon = Spoon(root=body)
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_semantic_annotation(spoon)

    with pytest.raises(NoGraspGeometry):
        spoon.grasp_candidates()


# %% grasping at a handle


def _mug(with_handle: bool) -> Mug:
    """
    :param with_handle: Whether the mug is given its handle.
    :return: A mug whose round body is a tube and whose handle is a box sticking out
        towards positive x.
    """
    body_shape = trimesh.creation.annulus(
        r_min=BOWL_INNER_RADIUS, r_max=BOWL_OUTER_RADIUS, height=BOWL_HEIGHT
    )
    handle_shape = trimesh.creation.box(extents=[0.03, 0.01, BOWL_HEIGHT / 2])
    handle_shape.apply_translation([BOWL_OUTER_RADIUS + 0.015, 0.0, 0.0])
    body = Body(name=PrefixedName(f"mug_{with_handle}", prefix="grasp_candidates"))
    body.collision = ShapeCollection(
        [
            Mesh.from_trimesh(
                mesh=trimesh.util.concatenate([body_shape, handle_shape]),
                origin=HomogeneousTransformationMatrix(reference_frame=body),
            )
        ],
        reference_frame=body,
    )
    mug = Mug(root=body)
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_semantic_annotation(mug)
    if with_handle:
        Handle.create_from_part_of_shape(mug, handle_shape)
    return mug


def test_an_object_with_a_handle_is_grasped_at_its_handle():
    mug = _mug(with_handle=True)

    grasps = mug.grasp_candidates()

    assert grasps
    for grasp in grasps:
        assert grasp.graspable is mug.handle
        assert grasp.grasp_pose.reference_frame is mug.handle.root


def test_the_grasped_part_of_an_object_with_a_handle_is_its_handle():
    mug = _mug(with_handle=True)

    assert mug.grasped_part() is mug.handle


def test_an_object_without_its_handle_is_grasped_as_it_offers_otherwise():
    mug = _mug(with_handle=False)

    grasps = mug.grasp_candidates()

    assert grasps
    for grasp in grasps:
        assert grasp.graspable is mug
    assert mug.grasped_part() is mug


def test_a_handle_made_from_part_of_a_shape_is_fixed_to_its_whole():
    mug = _mug(with_handle=True)

    connection = mug.handle.root.parent_connection

    assert connection.parent is mug.root
    assert not mug.handle.root.collision


def test_a_handle_made_from_part_of_a_shape_weighs_next_to_nothing():
    """
    Its material belongs to the whole. A body left with the default inertial would add a
    kilogram to the object, and one without any would be weighed by its shape.
    """
    mug = _mug(with_handle=True)

    inertial = mug.handle.root.inertial
    negligible = Inertial.negligible()

    assert inertial.mass == negligible.mass
    np.testing.assert_array_equal(inertial.inertia.data, negligible.inertia.data)


# %% the frame a grasp is expressed in


def test_a_grasp_in_a_foreign_frame_is_refused(milk, bowl):
    """
    A grasp written in another frame moves with the wrong body, so the approach would
    clear the wrong geometry. Nothing downstream can tell, so it is refused here.
    """
    with pytest.raises(ReferenceFrameMismatchError):
        GraspCandidate(milk, Pose(reference_frame=bowl.root))


def test_a_grasp_without_a_frame_is_refused(milk):
    """
    A frameless pose names no body at all, so it cannot be a grasp on one.
    """
    with pytest.raises(MissingReferenceFrameError):
        GraspCandidate(milk, Pose())


def test_a_grasp_from_the_body_origin_takes_the_object_at_its_own_origin(milk):
    grasp = GraspCandidate.from_body_origin(milk)

    assert grasp.graspable is milk
    assert grasp.grasp_pose.reference_frame is milk.root
    np.testing.assert_allclose(grasp.grasp_pose.to_np(), np.eye(4), atol=1e-9)


def test_a_grasp_in_the_world_frame_follows_where_the_object_stands():
    world = World()
    world_root = Body(name=PrefixedName("map", prefix="grasp_candidates"))
    body = Body(name=PrefixedName("milk", prefix="grasp_candidates"))
    milk = Milk(root=body)
    world_T_milk = HomogeneousTransformationMatrix.from_xyz_rpy(1.0, 2.0, 0.5, yaw=0.3)
    with world.modify_world():
        world.add_connection(
            FixedConnection(
                parent=world_root,
                child=body,
                parent_T_connection_expression=world_T_milk,
            )
        )
        world.add_semantic_annotation(milk)
    grasp = milk.grasp_candidates()[1]

    world_T_grasp = grasp.world_T_grasp

    assert world_T_grasp.reference_frame is world_root
    np.testing.assert_allclose(
        world_T_grasp.to_np(),
        world_T_milk.to_np() @ grasp.grasp_pose.to_np(),
        atol=1e-9,
    )


# %% the contract itself


def test_only_annotations_that_can_be_held_offer_grasps():
    """
    A root body is not enough to be graspable: furniture has one and is not picked up.

    The mixin sits below :class:`HasRootBody` rather than above it precisely so that a
    dishwasher cannot be asked where to grasp it.
    """
    for graspable in (Bowl, Milk, Spoon, Handle, Mug):
        assert issubclass(graspable, HasGraspCandidates)
    for fixed in (Dishwasher, Cabinet, Table, Floor):
        assert issubclass(fixed, HasRootBody)
        assert not issubclass(fixed, HasGraspCandidates)


def test_an_object_described_with_its_type_can_be_grasped(milk):
    """
    A typed description stands for an object a robot is asked to pick up, so it offers
    the default grasps of any graspable object.
    """
    described = NaturalLanguageWithTypeDescription(
        root=milk.root, description="a carton of milk", type_description="milk"
    )

    assert [
        grasp.grasp_pose.to_np().tolist() for grasp in described.grasp_candidates()
    ] == [grasp.grasp_pose.to_np().tolist() for grasp in milk.grasp_candidates()]


def test_a_bowl_is_grasped_at_its_rim(bowl):
    assert bowl.grasped_part() is bowl.rim
    assert isinstance(bowl.rim, Rim)


def test_a_bowl_without_a_rim_is_grasped_as_a_whole(bowl):
    bowl.rim = None

    assert bowl.grasped_part() is bowl
