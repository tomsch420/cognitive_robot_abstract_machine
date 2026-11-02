"""
RoboCasa's own robot, the PandaOmron: a Franka Panda arm with its two-finger gripper on
the lift of an Omron mobile base, described by robosuite's model of it.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from enum import StrEnum
from xml.etree import ElementTree

from robosuite.models.bases import robot_base_factory
from robosuite.models.grippers import gripper_factory
from robosuite.models.robots import create_robot
from robosuite.models.world import MujocoWorldBase
from robosuite.utils.mjcf_utils import xml_path_completion
from typing_extensions import Self

from semantic_digital_twin.adapters.mjcf import MJCFParser
from semantic_digital_twin.collision_checking.collision_rules import (
    AvoidExternalCollisions,
)
from semantic_digital_twin.datastructures.definitions import (
    GripperState,
    JointName,
    StaticJointState,
    TorsoState,
)
from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.robot_part_mixins import (
    HasMobileBase,
    HasOneArm,
    HasTorso,
    HasTwoFingers,
)
from semantic_digital_twin.robots.robot_parts import (
    AbstractRobot,
    Arm,
    EndEffector,
    Finger,
    MobileBase,
    ServoedRobotPart,
    Torso,
)
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.world_description.connection_properties import (
    DriveServos,
    JointDrive,
    ServoGains,
    VelocityServoGains,
)
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)
from krrood.ormatic.utils import classproperty

# %% names in robosuite's model


class PandaOmronJoint(JointName):
    """
    The joints of robosuite's PandaOmron that it commands.
    """

    ARM_1 = "robot0_joint1"
    ARM_2 = "robot0_joint2"
    ARM_3 = "robot0_joint3"
    ARM_4 = "robot0_joint4"
    ARM_5 = "robot0_joint5"
    ARM_6 = "robot0_joint6"
    ARM_7 = "robot0_joint7"

    LEFT_FINGER = "gripper0_right_finger_joint1"
    RIGHT_FINGER = "gripper0_right_finger_joint2"

    LIFT = "mobilebase0_joint_torso_height"

    MOBILE_FORWARD = "mobilebase0_joint_mobile_forward"
    MOBILE_SIDE = "mobilebase0_joint_mobile_side"
    MOBILE_YAW = "mobilebase0_joint_mobile_yaw"

    @classmethod
    def arm(cls) -> list[PandaOmronJoint]:
        """
        :return: The arm's joints, from the shoulder out.
        """
        return [
            cls.ARM_1,
            cls.ARM_2,
            cls.ARM_3,
            cls.ARM_4,
            cls.ARM_5,
            cls.ARM_6,
            cls.ARM_7,
        ]

    @classmethod
    def base_motion(cls) -> list[PandaOmronJoint]:
        """
        :return: The joints robosuite moves the base across the floor with.
        """
        return [cls.MOBILE_FORWARD, cls.MOBILE_SIDE, cls.MOBILE_YAW]


class PandaOmronBody(StrEnum):
    """
    The bodies of robosuite's PandaOmron that its parts begin or end at.
    """

    ROOT = "robot0_base"
    BASE = "mobilebase0_base"
    LIFT_BASE = "mobilebase0_fixed_support"
    ARM_MOUNT = "manipulator_mount"
    ARM_BASE = "robot0_link0"
    HAND = "robot0_right_hand"
    GRIPPER = "gripper0_right_right_gripper"
    TOOL_FRAME = "gripper0_right_eef"
    LEFT_FINGERTIP = "gripper0_right_finger_joint1_tip"
    RIGHT_FINGERTIP = "gripper0_right_finger_joint2_tip"


# %% robosuite's model


class RobosuiteModelName(StrEnum):
    """
    The names robosuite registers the PandaOmron's model and its arm by.
    """

    ROBOT = "PandaOmron"
    """
    The robot.
    """

    ARM = "right"
    """
    Its only arm.
    """


@dataclass
class PandaOmronDescription:
    """
    The MJCF of the PandaOmron as robosuite assembles it, with the base fixed under the
    robot's root and nothing actuated: a drive moves the base, and the robot's parts
    declare what drives their joints.
    """

    identifier: int = 0
    """
    The number robosuite puts in the names of the robot's parts.
    """

    def document(self) -> str:
        """
        :return: The MJCF of the robot, its default base and its default gripper.
        """
        model = create_robot(RobosuiteModelName.ROBOT, idn=self.identifier)
        model.add_base(base=robot_base_factory(model.default_base, idn=self.identifier))
        model.update_joints()
        model.update_actuators()
        arm = RobosuiteModelName.ARM
        model.add_gripper(
            gripper_factory(model.default_gripper[arm], idn=f"{self.identifier}_{arm}"),
            model.eef_name[arm],
        )
        world = MujocoWorldBase()
        world.merge(model)
        document = ElementTree.fromstring(world.get_xml())
        self._remove_base_motion(document)
        for actuators in document.findall("actuator"):
            actuators.clear()
        return ElementTree.tostring(document, encoding="unicode")

    @staticmethod
    def _remove_base_motion(document: ElementTree.Element) -> None:
        """
        Remove the joints robosuite moves the base with, which fixes the base under the
        robot's root.

        :param document: The robot's MJCF.
        """
        base_motion = set(PandaOmronJoint.base_motion())
        for body in document.iter("body"):
            for joint in body.findall("joint"):
                if joint.get("name") in base_motion:
                    body.remove(joint)


# %% the gripper


@dataclass(eq=False)
class PandaOmronLeftFinger(Finger):
    """
    The finger of the Panda gripper on its first finger joint.
    """

    def setup_hardware_interfaces(self) -> None:
        """
        The gripper sets up the finger's joint.
        """

    def setup_joint_states(self) -> list[JointState]:
        """
        The gripper's states hold the finger's.
        """
        return []

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        world = robot_root._world
        return cls(
            root=world.get_body_in_branch_by_name(robot_root, PandaOmronBody.GRIPPER),
            tip=world.get_body_in_branch_by_name(
                robot_root, PandaOmronBody.LEFT_FINGERTIP
            ),
        )


@dataclass(eq=False)
class PandaOmronRightFinger(Finger):
    """
    The finger of the Panda gripper on its second finger joint.
    """

    def setup_hardware_interfaces(self) -> None:
        """
        The gripper sets up the finger's joint.
        """

    def setup_joint_states(self) -> list[JointState]:
        """
        The gripper's states hold the finger's.
        """
        return []

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        world = robot_root._world
        return cls(
            root=world.get_body_in_branch_by_name(robot_root, PandaOmronBody.GRIPPER),
            tip=world.get_body_in_branch_by_name(
                robot_root, PandaOmronBody.RIGHT_FINGERTIP
            ),
        )


@dataclass(eq=False)
class PandaOmronGripper(
    EndEffector,
    HasTwoFingers[PandaOmronLeftFinger, PandaOmronRightFinger],
    ServoedRobotPart,
):
    """
    The Panda's two-finger gripper, its tool frame at robosuite's grip site between the
    fingertips.
    """

    open_finger_travel: float = 0.04
    """
    How far each finger slides out of the closed gripper when it opens, in metres.
    """

    def setup_hardware_interfaces(self) -> None:
        for joint_name in (PandaOmronJoint.LEFT_FINGER, PandaOmronJoint.RIGHT_FINGER):
            self._world.get_connection_by_name(joint_name).has_hardware_interface = True

    def setup_joint_states(self) -> list[JointState]:
        """
        Open and closed, the second finger's joint sliding the other way.
        """
        left = self._world.get_connection_by_name(PandaOmronJoint.LEFT_FINGER)
        right = self._world.get_connection_by_name(PandaOmronJoint.RIGHT_FINGER)
        travel = self.open_finger_travel
        return [
            JointState.from_mapping(
                name=PrefixedName("gripper_open", prefix=self.name.name),
                mapping={left: travel, right: -travel},
                state_type=GripperState.OPEN,
            ),
            JointState.from_mapping(
                name=PrefixedName("gripper_close", prefix=self.name.name),
                mapping={left: 0.0, right: 0.0},
                state_type=GripperState.CLOSE,
            ),
        ]

    @property
    def drive_of_each_joint(self) -> dict[str, JointDrive]:
        """
        Each finger's joint, driven as robosuite's position actuators drive it, the
        joint's own damping damping it.
        """
        finger = JointDrive(
            gains=ServoGains(stiffness=1000.0, damping=0.0, torque_limit=20.0)
        )
        return {
            PandaOmronJoint.LEFT_FINGER: finger,
            PandaOmronJoint.RIGHT_FINGER: finger,
        }

    @property
    def approach_axis(self) -> Vector3:
        return Vector3.Z(reference_frame=self.tool_frame)

    @property
    def closing_axis(self) -> Vector3:
        return Vector3.X(reference_frame=self.tool_frame)

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        world = robot_root._world
        return cls(
            root=world.get_body_in_branch_by_name(robot_root, PandaOmronBody.HAND),
            tool_frame=world.get_body_in_branch_by_name(
                robot_root, PandaOmronBody.TOOL_FRAME
            ),
        )


# %% the arm, the lift and the base


@dataclass(eq=False)
class PandaOmronArm(Arm[PandaOmronGripper], ServoedRobotPart):
    """
    The Franka Panda arm.
    """

    @property
    def drive_of_each_joint(self) -> dict[str, JointDrive]:
        """
        The five joints from the shoulder to the forearm may exert as much torque as
        robosuite lets their motors, and more than the two wrist joints.

        .. note:: The torque limits are robosuite's; the stiffness, damping and armature
            are the PR2 arm's, which are tuned for its simulated joints to follow their
            commanded positions.
        """
        upper = JointDrive(
            gains=ServoGains(stiffness=2000.0, damping=200.0, torque_limit=80.0),
            armature=0.5,
        )
        wrist = JointDrive(
            gains=ServoGains(stiffness=500.0, damping=30.0, torque_limit=12.0),
            armature=0.1,
        )
        arm = PandaOmronJoint.arm()
        return {joint: upper for joint in arm[:5]} | {joint: wrist for joint in arm[5:]}

    def setup_hardware_interfaces(self) -> None:
        for joint_name in PandaOmronJoint.arm():
            self._world.get_connection_by_name(joint_name).has_hardware_interface = True

    def setup_joint_states(self) -> list[JointState]:
        """
        Parked as robosuite starts the PandaOmron's arm, its ``init_qpos``.
        """
        model = create_robot(RobosuiteModelName.ROBOT)
        return [
            JointState.from_mapping(
                name=PrefixedName("park", prefix=self.name.name),
                mapping={
                    self._world.get_connection_by_name(joint_name): float(position)
                    for joint_name, position in zip(
                        PandaOmronJoint.arm(), model.init_qpos
                    )
                },
                state_type=StaticJointState.PARK,
            )
        ]

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        world = robot_root._world
        return cls(
            root=world.get_body_in_branch_by_name(robot_root, PandaOmronBody.ARM_BASE),
            tip=world.get_body_in_branch_by_name(robot_root, PandaOmronBody.HAND),
        )


@dataclass(eq=False)
class PandaOmronLift(Torso, HasOneArm[PandaOmronArm], ServoedRobotPart):
    """
    The lift of the Omron base that carries the arm.
    """

    @property
    def drive_of_each_joint(self) -> dict[str, JointDrive]:
        """
        The lift's joint, which carries the arm.

        .. note:: robosuite drives the lift with a motor of unlimited force; the gains
            are the PR2 torso's.
        """
        return {
            PandaOmronJoint.LIFT: JointDrive(
                gains=ServoGains(
                    stiffness=100000.0, damping=8000.0, torque_limit=10000.0
                ),
                armature=1.0,
            )
        }

    def setup_hardware_interfaces(self) -> None:
        self._world.get_connection_by_name(
            PandaOmronJoint.LIFT
        ).has_hardware_interface = True

    def setup_joint_states(self) -> list[JointState]:
        """
        Low, middle and high: the bottom, the middle and the top of the lift's travel.
        """
        lift = self._world.get_connection_by_name(PandaOmronJoint.LIFT)
        lower = lift.dof.limits.lower.position
        upper = lift.dof.limits.upper.position
        return [
            JointState.from_mapping(
                name=PrefixedName(name, prefix=self.name.name),
                mapping={lift: position},
                state_type=state_type,
            )
            for name, position, state_type in (
                ("torso_low", lower, TorsoState.LOW),
                ("torso_mid", (lower + upper) / 2, TorsoState.MID),
                ("torso_high", upper, TorsoState.HIGH),
            )
        ]

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        world = robot_root._world
        return cls(
            root=world.get_body_in_branch_by_name(robot_root, PandaOmronBody.LIFT_BASE),
            tip=world.get_body_in_branch_by_name(robot_root, PandaOmronBody.ARM_MOUNT),
        )


@dataclass(eq=False)
class PandaOmronMobileBase(MobileBase[OmniDrive], HasTorso[PandaOmronLift]):
    """
    The Omron mobile base, which moves in any direction across the floor and turns.
    """

    @classproperty
    def forward_axis(cls) -> Vector3:
        return Vector3.X()

    def setup_hardware_interfaces(self) -> None:
        """
        The drive moves the base; it has no joints of its own.
        """

    @property
    def drive_servos(self) -> DriveServos:
        """
        robosuite's velocity actuators of the base's forward, sideways and turning
        motion; the forward and sideways ones drive both translations.
        """
        return DriveServos(
            translation_gains=VelocityServoGains(damping=1000.0, torque_limit=600.0),
            rotation_gains=VelocityServoGains(damping=1500.0, torque_limit=600.0),
        )

    def _setup_servos(self) -> None:
        """
        Declare the drive's servos, see :meth:`MobileBase._setup_servos`, and let a
        physical simulation carry the weight of every body standing on the base, as
        robosuite's controllers compensate gravity.
        """
        super()._setup_servos()
        self._compensate_gravity()

    def setup_joint_states(self) -> list[JointState]:
        """
        The base has no joint states.
        """
        return []

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, PandaOmronBody.BASE
            ),
            full_body_controlled=True,
        )


# %% the robot


@dataclass(eq=False)
class PandaOmron(AbstractRobot, HasMobileBase[PandaOmronMobileBase]):
    """
    RoboCasa's own robot: a Franka Panda arm with its two-finger gripper on the lift of
    an Omron mobile base, as robosuite models it.
    """

    @classmethod
    def get_ros_file_path(cls) -> str:
        """
        :return: robosuite's description of the Panda arm, which robosuite assembles
            with its Omron base and Panda gripper into the PandaOmron.
        """
        return xml_path_completion("robots/panda/robot.xml")

    @classmethod
    def from_description(cls) -> Self:
        """
        :return: This robot, read from the MJCF robosuite assembles, without the world
            body MJCF puts above every robot's root.
        """
        world = MJCFParser.from_xml_string(PandaOmronDescription().document()).parse()
        with world.modify_world():
            world.remove_kinematic_structure_entity(world.root)
        return cls.from_world(world)

    @classmethod
    def _get_root_body_name(cls) -> str:
        return PandaOmronBody.ROOT

    def _setup_collision_rules(self) -> None:
        """
        Avoid collisions with everything but the robot itself, whose description states
        which of its own bodies may touch.
        """
        self._world.collision_manager.extend_default_rules(
            [
                AvoidExternalCollisions(
                    buffer_zone_distance=0.05, violated_distance=0.0, robot=self
                ),
            ]
        )

    def _setup_velocity_limits(self) -> None:
        self.tighten_dof_velocity_limits_of_1dof_connections(
            new_limits=defaultdict(lambda: 1.0)
        )
