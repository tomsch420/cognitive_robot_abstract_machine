import time

from pycram.action_executor import ActionGraphParser
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription
from pycram.plans.factories import sequential
from pycram.robot_plans.actions.core.pick_up import PickUpAction


def test_sub_action_expansion(immutable_model_world):
    world, view, context = immutable_model_world

    plan = sequential(
        [
            PickUpAction(
                object_designator=world.get_body_by_name("milk.stl"),
                arm=Arms.RIGHT,
                grasp_description=GraspDescription(
                    ApproachDirection.FRONT,
                    vertical_alignment=VerticalAlignment.NoAlignment,
                    manipulator=view.right_arm.manipulator,
                ),
            )
        ],
        context=context,
    )

    pick_node = plan.children[0]
    pick_node.notify()

    parser = ActionGraphParser(pick_node)
    exapanded_children = parser.expand_sub_actions(pick_node.children)
    assert len(exapanded_children) > 1
