"""
CRAM action that picks up a loose Montessori shape and inserts it through the
shape-sorting board's hole matching its category.
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Optional

from coraplex.datastructures.enums import ApproachDirection, Arms, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.locations.base import DeferredLocation
from coraplex.locations.factories import reachability_location
from coraplex.plans.factories import sequential
from coraplex.plans.plan_node import PlanNode
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.view_manager import ViewManager
from experiments.montessori.semantics import MontessoriShape, ShapeSortingBoard
from krrood.entity_query_language.factories import a, variable
from semantic_digital_twin.spatial_types.spatial_types import Pose

INSERTION_HOVER_HEIGHT = 0.03
"""
Height above the target hole at which the shape is released, so the gripper clears
the board's surface on approach.
"""


@dataclass
class InsertMontessoriShapeAction(ActionDescription):
    """
    Picks up a loose Montessori shape and inserts it through the shape-sorting
    board's hole matching its category.

    The robot navigates to a reachable stance before picking the shape up, and again
    before placing it, so this works regardless of where the robot happens to be
    standing relative to the shape and the board.
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

    @property
    def _action_plan(self) -> PlanNode:
        hole = self.board.hole_for(self.montessori_shape)
        hole_position = hole.root.global_transform.to_position()
        target_location = Pose.from_xyz_rpy(
            hole_position.x,
            hole_position.y,
            hole_position.z + INSERTION_HOVER_HEIGHT,
            reference_frame=self.world.root,
        )
        self.grasp_description = self.grasp_description or GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.TOP,
            ViewManager.get_end_effector_view(self.arm, self.robot),
        )

        return sequential(
            [
                ParkArmsAction(Arms.BOTH),
                a(NavigateAction)(
                    target_location=variable(
                        Pose,
                        domain=DeferredLocation(
                            lambda: reachability_location(
                                self.montessori_shape.root,
                                self.context,
                                self.arm,
                                self.grasp_description,
                            )
                        ),
                    ),
                    keep_joint_states=True,
                ),
                a(PickUpAction)(
                    object_designator=self.montessori_shape.root,
                    arm=self.arm,
                    grasp_description=self.grasp_description,
                ),
                ParkArmsAction(Arms.BOTH),
                a(NavigateAction)(
                    target_location=variable(
                        Pose,
                        domain=reachability_location(
                            target_location,
                            self.context,
                            self.arm,
                            self.grasp_description,
                        ),
                    ),
                    keep_joint_states=True,
                ),
                a(PlaceAction)(
                    object_designator=self.montessori_shape.root,
                    target_location=target_location,
                    arm=self.arm,
                ),
                ParkArmsAction(Arms.BOTH),
            ]
        )
