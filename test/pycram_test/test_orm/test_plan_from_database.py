import numpy as np
import pytest
from sqlalchemy import select, text

# The alternative mapping needs to be imported for the stretch to work properly
import pycram.alternative_motion_mappings.stretch_motion_mapping  # type: ignore
import pycram.alternative_motion_mappings.tiago_motion_mapping  # type: ignore
from krrood.ormatic.dao import to_dao, get_dao_class
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import (
    ApproachDirection,
    Arms,
    VerticalAlignment,
    TaskStatus,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PyCramPose, PoseStamped
from pycram.language import SequentialPlan, ParallelPlan
from pycram.motion_executor import simulated_robot
from pycram.orm.ormatic_interface import *
from pycram.robot_plans import (
    MoveTorsoActionDescription,
    ParkArmsAction,
    PlaceActionDescription,
    TransportAction,
    ParkArmsActionDescription,
    TransportActionDescription,
    NavigateActionDescription,
    PickUpActionDescription,
    SetGripperActionDescription,
    OpenActionDescription,
    CloseActionDescription,
    NavigateAction,
    PickUpAction,
    PlaceAction,
)
from semantic_digital_twin.datastructures.definitions import TorsoState, GripperState
from semantic_digital_twin.world import World


def test_pick_up_action_and_replay_from_db(pycram_testing_session, mutable_model_world):
    session = pycram_testing_session
    world, robot_view, context = mutable_model_world
    with simulated_robot:
        sp = SequentialPlan(
            Context.from_world(world),
            NavigateActionDescription(
                PoseStamped.from_list([1.6, 1.9, 0], [0, 0, 0, 1], world.root),
                True,
            ),
            ParkArmsActionDescription(Arms.BOTH),
            PickUpActionDescription(
                world.get_body_by_name("milk.stl"),
                Arms.LEFT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.NoAlignment,
                    robot_view.left_arm.manipulator,
                ),
            ),
            NavigateActionDescription(
                PoseStamped.from_list([1.6, 2.3, 0], [0, 0, 0, 1], world.root),
                True,
            ),
            PlaceActionDescription(
                world.get_body_by_name("milk.stl"),
                PoseStamped.from_list([2.3, 2.5, 1.0], [0, 0, 0, 1], world.root),
                Arms.LEFT,
            ),
        )
        sp.perform()

    dao: PlanMappingDAO = to_dao(sp)

    session.add(dao)
    session.commit()
    result = session.scalars(select(PlanMappingDAO)).first()

    world_from_database: World = result.context.world.from_dao()
    assert len(world_from_database._model_manager.model_modification_blocks) == len(
        world._model_manager.model_modification_blocks
    )
