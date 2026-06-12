from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from datetime import timedelta

from typing_extensions import Optional, Type, Any

from krrood.entity_query_language.factories import underspecified, variable
from pycram.datastructures.enums import DetectionTechnique
from pycram.locations.factories import visibility_location
from pycram.plans.factories import sequential, try_in_order
from pycram.plans.plan_node import PlanNode
from pycram.robot_plans.actions.base import ActionDescription
from pycram.robot_plans.actions.core.misc import DetectAction
from pycram.robot_plans.actions.core.navigation import NavigateAction, LookAtAction
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation


@dataclass
class SearchAction(ActionDescription):
    """
    Searches for a target object around the given location.
    """

    target_location: Pose
    """
    Location around which to look for a target object.
    """

    object_sem_annotation: Type[SemanticAnnotation]
    """
    Type of the object which is searched for.
    """

    @property
    def _action_plan(self) -> PlanNode:

        # define searching cone
        target_base = self.world.transform(self.target_location, self.world.root)

        target_base_left = deepcopy(target_base)
        target_base_left.y -= 0.5

        target_base_right = deepcopy(target_base)
        target_base_right.y += 0.5

        return sequential(
            [
                # go to a location where the target location is visible
                underspecified(NavigateAction)(
                    target_location=variable(
                        Pose,
                        domain=visibility_location(self.target_location, self.context),
                    ),
                ),
                try_in_order(
                    [
                        sequential(
                            [
                                LookAtAction(target),
                                DetectAction(
                                    DetectionTechnique.TYPES,
                                    object_sem_annotation=self.object_sem_annotation,
                                ),
                            ]
                        )
                        for target in [target_base, target_base_left, target_base_right]
                    ]
                ),
            ]
        )

    def validate(
        self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None
    ):
        pass
