"""
CRAM action that picks up a loose Montessori shape and inserts it through the shape-
sorting board's hole matching its category.
"""

from __future__ import annotations

from dataclasses import dataclass

import rustworkx
from typing_extensions import Optional

from coraplex.datastructures.enums import ApproachDirection, Arms, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.plans.factories import sequential
from coraplex.plans.plan_node import PlanNode
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.misc import MoveToReach
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.view_manager import ViewManager
from experiments.montessori.semantics import MontessoriShape, ShapeSortingBoard
from krrood.entity_query_language.factories import underspecified
from krrood.entity_query_language.query.match import Match
from semantic_digital_twin.exceptions import PointOccupiedError
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types.spatial_types import Point3, Pose, Pose2D
from semantic_digital_twin.world_description.graph_of_convex_sets import (
    navigation_map_at_target,
    translate_free_space_to_where_condition,
)
from semantic_digital_twin.world_description.world_entity import Body

STANDOFF_CLEARANCE = 0.1
"""
Horizontal clearance kept between a standoff point (see
:meth:`InsertMontessoriShapeAction._standoff_point_near_surface`) and the edge of the
surface it stands off from.
"""


@dataclass
class InsertMontessoriShapeAction(ActionDescription):
    """
    Picks up a loose Montessori shape and inserts it through the shape-sorting board's
    hole matching its category.

    The robot moves to a reachable stance before picking the shape up, and again before
    placing it, so this works regardless of where the robot happens to be standing
    relative to the shape and the board.
    """

    montessori_shape: MontessoriShape
    """
    The loose shape to pick up and insert.
    """

    board: ShapeSortingBoard
    """
    The shape-sorting board whose matching hole the shape is inserted through.
    """

    arm: Arms
    """
    Arm used to pick up and insert the shape.
    """

    grasp_description: Optional[GraspDescription] = None
    """
    Grasp used to pick up the shape; a default top-down grasp is used if not given,
    since the shape rests flat on a table rather than standing on an edge.
    """

    insertion_hover_height: float = 0.03
    """
    Height above the target hole at which the shape is released, so the gripper clears
    the board's surface on approach.
    """

    target_horizontal_offset: Optional[Point3] = None
    """
    Offset added to the target hole's own (x, y) position before releasing the shape
    above it. ``None`` (the default) targets the hole's center exactly; a small nonzero
    offset lets a caller retry a failed insertion (see
    :func:`~experiments.montessori.montessori_demo._insert_all_shapes`) with an actually
    different drop point, rather than repeating the exact same teleport-then-settle that
    just failed.
    """

    def _standoff_point_near_surface(
        self, surface: Body, target_position: Point3
    ) -> Pose:
        """
        A point just outside ``surface``'s bounding box, offset from whichever edge is
        nearest to ``target_position``, at ``target_position``'s height.

        :func:`~semantic_digital_twin.world_description.graph_of_convex_sets.navigation_map_at_target`
        projects obstacles to a 2D floor footprint (any point above or on a wide
        surface like a table or the sorting board reads as occupied, regardless of
        height), so a reach target actually on ``surface`` is never in free space.
        Stepping just past the nearest edge, mirroring
        :meth:`~experiments.montessori.world.MontessoriWorld.spawn_robot`'s own
        ``table_bounding_box.min_x - standoff_distance`` pattern, gives
        :meth:`_move_to_reach` a genuinely free point near the target to reach for.

        :param surface: The body ``target_position`` rests on (or near).
        :param target_position: The position to stand off from.
        """
        bounding_box = surface.collision.as_bounding_box_collection_in_frame(
            self.world.root
        ).bounding_box()
        x, y = float(target_position.x), float(target_position.y)
        distance_to_edge = {
            "min_x": x - bounding_box.min_x,
            "max_x": bounding_box.max_x - x,
            "min_y": y - bounding_box.min_y,
            "max_y": bounding_box.max_y - y,
        }
        nearest_edge = min(distance_to_edge, key=distance_to_edge.get)
        if nearest_edge == "min_x":
            x = bounding_box.min_x - STANDOFF_CLEARANCE
        elif nearest_edge == "max_x":
            x = bounding_box.max_x + STANDOFF_CLEARANCE
        elif nearest_edge == "min_y":
            y = bounding_box.min_y - STANDOFF_CLEARANCE
        else:
            y = bounding_box.max_y + STANDOFF_CLEARANCE
        return Pose.from_xyz_rpy(
            x, y, float(target_position.z), reference_frame=self.world.root
        )

    def _grasp_description_query(self) -> Match[GraspDescription]:
        """
        :attr:`grasp_description`, rebuilt through the query DSL.

        The :class:`~krrood.entity_query_language.backends.ProbabilisticBackend`
        (required by :meth:`_move_to_reach`'s underspecified standing offset; see
        ``query_backend`` on :class:`~coraplex.datastructures.dataclasses.Context`)
        needs every field of every ``underspecified(...)``-wrapped action's arguments expressed
        through the query DSL to build its probabilistic circuit, even fields that are
        already fully concrete: passing :attr:`grasp_description` directly raises
        ``ValueError: ... not in domain of variable ...`` once that backend is active.
        """
        return underspecified(GraspDescription)(
            approach_direction=self.grasp_description.approach_direction,
            vertical_alignment=self.grasp_description.vertical_alignment,
            end_effector=self.grasp_description.end_effector,
            rotate_gripper=self.grasp_description.rotate_gripper,
        )

    def _move_to_reach(self, target: Body, target_pose_end_effector: Pose) -> PlanNode:
        """
        Move the robot to a stance, within the free space of a Graph of Convex Sets
        navigation map built around ``target``, from which ``target_pose_end_effector``
        is reachable by the end effector.

        Mirrors the reach-planning pattern of
        :class:`~experiments.sage_10k.sage10k_actions.Sage10kOpenDoor`: rather than a
        hand-picked standing offset or a costmap search, the robot's standing offset is
        left underspecified and constrained to the navigation map's free space, letting
        it be resolved generatively.

        :param target: The body to build the navigation map around.
        :param target_pose_end_effector: The pose the end effector should reach.
        :raises PointOccupiedError: If ``target_pose_end_effector`` is not in the
            navigation map's free space.
        """
        gcs = navigation_map_at_target(target=target)
        target_node = gcs.node_of_point(target_pose_end_effector.position)
        if target_node is None:
            raise PointOccupiedError(
                self.world.transform(target_pose_end_effector, self.world.root).position
            )
        gcs = gcs.create_subgraph(
            list(
                rustworkx.node_connected_component(
                    gcs.graph, gcs.box_to_index_map[target_node]
                )
            )
        )

        reach_query = underspecified(MoveToReach)(
            target_pose_offset_robot=underspecified(Pose2D)(
                x=..., y=..., yaw=..., reference_frame=None
            ),
            hip_rotation=0.0,
            target_pose_end_effector=target_pose_end_effector,
            grasp_description=self._grasp_description_query(),
        )
        where_condition = translate_free_space_to_where_condition(
            gcs.free_space_event,
            reach_query.expression,
            x_variable_name="MoveToReach.target_pose_offset_robot.x",
            y_variable_name="MoveToReach.target_pose_offset_robot.y",
        )
        return reach_query.where(where_condition)

    @property
    def _action_plan(self) -> PlanNode:
        hole = self.board.hole_for(self.montessori_shape)
        hole_position = hole.root.global_transform.to_position()
        offset = self.target_horizontal_offset or Point3(0.0, 0.0, 0.0)
        target_location = Pose.from_xyz_rpy(
            hole_position.x + offset.x,
            hole_position.y + offset.y,
            hole_position.z + self.insertion_hover_height,
            reference_frame=self.world.root,
        )
        shape_position = self.montessori_shape.root.global_pose.to_position()
        [table] = self.world.get_semantic_annotations_by_type(Table)
        pickup_standoff_pose = self._standoff_point_near_surface(
            table.root, shape_position
        )
        placement_standoff_pose = self._standoff_point_near_surface(
            table.root, target_location.to_position()
        )
        self.grasp_description = self.grasp_description or GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.TOP,
            ViewManager.get_end_effector_view(self.arm, self.robot),
        )

        return sequential(
            [
                ParkArmsAction(Arms.BOTH),
                self._move_to_reach(self.montessori_shape.root, pickup_standoff_pose),
                underspecified(PickUpAction)(
                    object_designator=self.montessori_shape.root,
                    arm=self.arm,
                    grasp_description=self._grasp_description_query(),
                ),
                ParkArmsAction(Arms.BOTH),
                self._move_to_reach(self.board.root, placement_standoff_pose),
                underspecified(PlaceAction)(
                    object_designator=self.montessori_shape.root,
                    target_location=target_location,
                    arm=self.arm,
                ),
                ParkArmsAction(Arms.BOTH),
            ]
        )

    def has_fallen_through_hole(self) -> bool:
        """
        Whether :attr:`montessori_shape` currently rests below the board's top
        surface, directly beneath its matching hole, i.e. has actually fallen
        through that hole rather than still resting on top of the board or having
        never been moved there at all.

        :attr:`_action_plan` only ever kinematically teleports the shape to its
        target pose via
        :class:`~coraplex.robot_plans.actions.core.placing.PlaceAction`, which does
        not check whether the shape actually fits through a tight-clearance hole;
        call this only after the shape has had a chance to physically settle (e.g.
        by simulating it in MuJoCo), not right after the action plan finishes.

        :return: ``True`` if the shape's own center is horizontally within the
            hole's footprint and its highest point is below the board's top
            surface.
        """
        hole = self.board.hole_for(self.montessori_shape)
        hole_position = hole.root.global_transform.to_position()
        hole_bounds = hole.root.area.combined_mesh.bounds
        hole_min_x = float(hole_position.x) + hole_bounds[0][0]
        hole_max_x = float(hole_position.x) + hole_bounds[1][0]
        hole_min_y = float(hole_position.y) + hole_bounds[0][1]
        hole_max_y = float(hole_position.y) + hole_bounds[1][1]

        shape_position = self.montessori_shape.root.global_transform.to_position()
        shape_x, shape_y = float(shape_position.x), float(shape_position.y)
        is_below_the_hole = (
            hole_min_x <= shape_x <= hole_max_x and hole_min_y <= shape_y <= hole_max_y
        )

        board_top_z = (
            self.board.root.collision.as_bounding_box_collection_in_frame(
                self.world.root
            )
            .bounding_box()
            .max_z
        )
        shape_top_z = (
            self.montessori_shape.root.collision.as_bounding_box_collection_in_frame(
                self.world.root
            )
            .bounding_box()
            .max_z
        )
        return bool(is_below_the_hole and shape_top_z < board_top_z)
