from __future__ import annotations

import os
from collections import defaultdict
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, replace
from enum import StrEnum
from importlib.resources import files
from pathlib import Path
from typing import Self, List

from typing_extensions import Dict

from krrood.ormatic.utils import classproperty
from semantic_digital_twin.adapters.sensors.lidar import Lidar, LidarSource
from semantic_digital_twin.collision_checking.collision_matrix import (
    MaxAvoidedCollisionsOverride,
)
from semantic_digital_twin.collision_checking.collision_rules import (
    AvoidExternalCollisions,
    AvoidSelfCollisions,
    SelfCollisionMatrixRule,
)
from semantic_digital_twin.datastructures.definitions import (
    GripperState,
    StaticJointState,
    TorsoState,
)
from semantic_digital_twin.datastructures.field_of_view import FieldOfView
from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.datastructures.scan_pattern import ScanPattern
from semantic_digital_twin.robots.robot_part_mixins import (
    HasNeck,
    HasLidar,
    HasOneArm,
    HasTorso,
    HasMobileBase,
    HasTwoFingers,
    HasSensors,
)
from semantic_digital_twin.robots.robot_parts import (
    AbstractRobot,
    AbstractRobotPart,
    Arm,
    Camera,
    Finger,
    Neck,
    Torso,
    MobileBase,
    EndEffector,
)
from semantic_digital_twin.spatial_types import Quaternion, Vector3
from semantic_digital_twin.world_description.connection_properties import (
    DriveServos,
    JointDynamics,
    JointServo,
    ServoGains,
    VelocityServoGains,
)
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    OmniDrive,
)
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)


class HSRBTopic(StrEnum):
    """
    Topics the HSRB publishes the state of its parts on, where it does not follow the
    conventional name.
    """

    ODOMETRY = "laser_odom"
    """
    The topic the mobile base reports its odometry on.
    """


class HSRBJoint(StrEnum):
    """
    Names of the HSRB's commandable connections, as spelled in its URDF.

    Members are usable wherever a connection name is expected, so a configuration keyed by
    them stays a plain mapping of names to positions.

    ..note:: Connections that no controller commands, such as the drive and passive base
        wheels and the hand's spring and distal joints, are left out.
    """

    TORSO_LIFT = "torso_lift_joint"
    HEAD_PAN = "head_pan_joint"
    HEAD_TILT = "head_tilt_joint"

    ARM_LIFT = "arm_lift_joint"
    ARM_FLEX = "arm_flex_joint"
    ARM_ROLL = "arm_roll_joint"
    WRIST_FLEX = "wrist_flex_joint"
    WRIST_ROLL = "wrist_roll_joint"

    HAND_MOTOR = "hand_motor_joint"
    HAND_LEFT_PROXIMAL = "hand_l_proximal_joint"
    HAND_RIGHT_PROXIMAL = "hand_r_proximal_joint"


class HSRBSpringJoint(StrEnum):
    """
    Names of the HSRB's passive finger joints, as spelled in its URDF: springs that let
    a fingertip give way against what it grips, which no controller commands.
    """

    HAND_LEFT_SPRING_PROXIMAL = "hand_l_spring_proximal_joint"
    HAND_RIGHT_SPRING_PROXIMAL = "hand_r_spring_proximal_joint"


OPEN_HAND_ANGLE = 1.2
"""
How far the hand motor turns to open the HSRB's hand, in radians: near the end of its
range, where the fingertips stand 13.5 cm apart, wide enough to close around a carton.
"""

# %% what drives the joints in a physical simulation


@dataclass(frozen=True)
class HSRBJointDrive:
    """
    What drives one degree of freedom of the HSRB in a physical simulation.

    The HSRB's description carries effort limits but no servo gains, and no tuned MuJoCo
    model of it exists to take gains from, so the gains are chosen to hold and track
    poses under the tests in ``test_hsr_mujoco.py``.
    """

    torque_limit: float
    """
    The largest torque, or force for a prismatic joint, the servo may exert: the effort
    limit of the HSRB's description.
    """

    stiffness: float
    """
    How hard the servo pulls towards its commanded position.
    """

    damping: float
    """
    How hard the servo resists the joint's velocity.
    """

    armature: float
    """
    The rotor inertia reflected through the joint's transmission.
    """

    def servo_for(self, connection: ActiveConnection1DOF) -> JointServo:
        """
        :param connection: A joint this drive moves.
        :return: The servo driving ``connection``, keeping the passive damping and
            friction its description gives it.
        """
        return JointServo(
            gains=ServoGains(
                stiffness=self.stiffness,
                damping=self.damping,
                torque_limit=self.torque_limit,
            ),
            dynamics=JointDynamics(
                armature=self.armature,
                damping=connection.dynamics.damping,
                dry_friction=connection.dynamics.dry_friction,
            ),
        )


HSRB_DRIVES_BY_DEGREE_OF_FREEDOM: Dict[str, HSRBJointDrive] = {
    HSRBJoint.ARM_LIFT: HSRBJointDrive(
        torque_limit=300.0, stiffness=10_000.0, damping=1_000.0, armature=1.0
    ),
    HSRBJoint.ARM_FLEX: HSRBJointDrive(
        torque_limit=100.0, stiffness=2_000.0, damping=200.0, armature=0.1
    ),
    HSRBJoint.ARM_ROLL: HSRBJointDrive(
        torque_limit=100.0, stiffness=500.0, damping=50.0, armature=0.1
    ),
    HSRBJoint.WRIST_FLEX: HSRBJointDrive(
        torque_limit=100.0, stiffness=500.0, damping=50.0, armature=0.1
    ),
    HSRBJoint.WRIST_ROLL: HSRBJointDrive(
        torque_limit=100.0, stiffness=500.0, damping=50.0, armature=0.1
    ),
    HSRBJoint.HEAD_PAN: HSRBJointDrive(
        torque_limit=100.0, stiffness=300.0, damping=30.0, armature=0.1
    ),
    HSRBJoint.HEAD_TILT: HSRBJointDrive(
        torque_limit=100.0, stiffness=300.0, damping=30.0, armature=0.1
    ),
    HSRBJoint.HAND_MOTOR: HSRBJointDrive(
        torque_limit=100.0, stiffness=50.0, damping=5.0, armature=0.05
    ),
    HSRBSpringJoint.HAND_LEFT_SPRING_PROXIMAL: HSRBJointDrive(
        torque_limit=10.0, stiffness=10.0, damping=1.0, armature=0.01
    ),
    HSRBSpringJoint.HAND_RIGHT_SPRING_PROXIMAL: HSRBJointDrive(
        torque_limit=10.0, stiffness=10.0, damping=1.0, armature=0.01
    ),
}
"""
What drives each degree of freedom of the HSRB, by the degree of freedom's name.

A joint that mimics another moves on the other's degree of freedom, and so with its
drive.
"""


@dataclass(eq=False)
class ServoedHSRBPart(AbstractRobotPart, ABC):
    """
    A part of the HSRB whose joints are driven by servos in a physical simulation, and
    which carries its own weight there.
    """

    @property
    @abstractmethod
    def servoed_connections(self) -> List[ActiveConnection1DOF]:
        """
        The joints of this part that are driven in a physical simulation.
        """

    def _setup_servos(self) -> None:
        for connection in self.servoed_connections:
            drive = HSRB_DRIVES_BY_DEGREE_OF_FREEDOM[connection.raw_dof.name.name]
            self._declare_servo(connection, drive.servo_for(connection))
        self._compensate_gravity()


@dataclass(eq=False)
class HSRBLeftFinger(Finger):

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "hand_l_proximal_link"
            ),
            tip=robot_root._world.get_body_in_branch_by_name(
                robot_root, "hand_l_distal_link"
            ),
        )


@dataclass(eq=False)
class HSRBRightFinger(Finger):

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "hand_r_proximal_link"
            ),
            tip=robot_root._world.get_body_in_branch_by_name(
                robot_root, "hand_r_distal_link"
            ),
        )


@dataclass(eq=False)
class HSRBGripper(
    ServoedHSRBPart, EndEffector, HasTwoFingers[HSRBLeftFinger, HSRBRightFinger]
):

    @property
    def servoed_connections(self) -> List[ActiveConnection1DOF]:
        return [
            connection
            for connection in self.active_connections
            if isinstance(connection, ActiveConnection1DOF)
        ]

    def setup_hardware_interfaces(self):
        return

    def setup_joint_states(self) -> List[JointState]:
        world = self._world
        gripper_joints = [
            world.get_connection_by_name(HSRBJoint.HAND_LEFT_PROXIMAL),
            world.get_connection_by_name(HSRBJoint.HAND_RIGHT_PROXIMAL),
            world.get_connection_by_name(HSRBJoint.HAND_MOTOR),
        ]

        gripper_open = JointState.from_mapping(
            name=PrefixedName("gripper_open", prefix=self.name.name),
            mapping=dict(zip(gripper_joints, [OPEN_HAND_ANGLE] * len(gripper_joints))),
            state_type=GripperState.OPEN,
        )

        gripper_close = JointState.from_mapping(
            name=PrefixedName("gripper_close", prefix=self.name.name),
            mapping=dict(zip(gripper_joints, [0.0, 0.0, 0.0])),
            state_type=GripperState.CLOSE,
        )

        return [gripper_open, gripper_close]

    @property
    def approach_axis(self) -> Vector3:
        return Vector3.Z(reference_frame=self.tool_frame)

    @property
    def closing_axis(self) -> Vector3:
        return Vector3.NEGATIVE_Y(reference_frame=self.tool_frame)

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "hand_palm_link"
            ),
            tool_frame=robot_root._world.get_body_in_branch_by_name(
                robot_root, "hand_gripper_tool_frame"
            ),
        )


@dataclass(eq=False)
class HSRBHandCamera(Camera):

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    @property
    def forward_facing_axis(self) -> Vector3:
        return Vector3.Z(reference_frame=self.root)

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "hand_camera_frame"
            ),
            field_of_view=FieldOfView(horizontal_angle=0.99483, vertical_angle=0.75049),
            minimal_height=0.75049,
            maximal_height=0.99483,
        )


@dataclass(eq=False)
class HSRBArm(ServoedHSRBPart, Arm[HSRBGripper], HasSensors[HSRBHandCamera]):

    @property
    def servoed_connections(self) -> List[ActiveConnection1DOF]:
        """
        The joints the arm's controller commands, including the lift that carries the
        arm along the torso.
        """
        return [
            self._world.get_connection_by_name(joint_name)
            for joint_name in (
                HSRBJoint.ARM_FLEX,
                HSRBJoint.ARM_LIFT,
                HSRBJoint.ARM_ROLL,
                HSRBJoint.WRIST_FLEX,
                HSRBJoint.WRIST_ROLL,
            )
        ]

    def setup_hardware_interfaces(self):
        for connection in self.servoed_connections:
            connection.has_hardware_interface = True

    def setup_joint_states(self) -> List[JointState]:
        arm_park = JointState.from_mapping(
            name=PrefixedName("arm_park", prefix=self.name.name),
            mapping=dict(
                zip(
                    self.active_connections,
                    [0.0, 1.5, -1.85, 0.0],
                )
            ),
            state_type=StaticJointState.PARK,
        )

        return [arm_park]

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "arm_lift_link"
            ),
            tip=robot_root._world.get_body_in_branch_by_name(
                robot_root, "hand_palm_link"
            ),
        )


@dataclass(eq=False)
class HSRBHeadCenterCamera(Camera):

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    @property
    def forward_facing_axis(self) -> Vector3:
        return Vector3.Z(reference_frame=self.root)

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "head_center_camera_frame"
            ),
            field_of_view=FieldOfView(horizontal_angle=0.99483, vertical_angle=0.75049),
            minimal_height=0.75049,
            maximal_height=0.99483,
            default_camera=True,
        )


@dataclass(eq=False)
class HSRBHeadLeftCamera(Camera):

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    @property
    def forward_facing_axis(self) -> Vector3:
        return Vector3.Z(reference_frame=self.root)

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "head_l_stereo_camera_link"
            ),
            field_of_view=FieldOfView(horizontal_angle=0.99483, vertical_angle=0.75049),
            minimal_height=0.75049,
            maximal_height=0.99483,
        )


@dataclass(eq=False)
class HSRBHeadRightCamera(Camera):

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    @property
    def forward_facing_axis(self) -> Vector3:
        return Vector3.Z(reference_frame=self.root)

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "head_r_stereo_camera_link"
            ),
            field_of_view=FieldOfView(horizontal_angle=0.99483, vertical_angle=0.75049),
            minimal_height=0.75049,
            maximal_height=0.99483,
        )


@dataclass(eq=False)
class HSRBHeadRGBDCamera(Camera):

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    @property
    def forward_facing_axis(self) -> Vector3:
        return Vector3.Z(reference_frame=self.root)

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "head_rgbd_sensor_link"
            ),
            field_of_view=FieldOfView(horizontal_angle=0.99483, vertical_angle=0.75049),
            minimal_height=0.75049,
            maximal_height=0.99483,
            default_camera=True,
        )


@dataclass(eq=False)
class HSRBNeck(
    ServoedHSRBPart,
    Neck[
        HSRBHeadCenterCamera,
        HSRBHeadLeftCamera,
        HSRBHeadRightCamera,
        HSRBHeadRGBDCamera,
    ],
):

    @property
    def servoed_connections(self) -> List[ActiveConnection1DOF]:
        return [
            self._world.get_connection_by_name(joint_name)
            for joint_name in (HSRBJoint.HEAD_PAN, HSRBJoint.HEAD_TILT)
        ]

    def setup_hardware_interfaces(self):
        for connection in self.servoed_connections:
            connection.has_hardware_interface = True

    def setup_joint_states(self) -> List[JointState]:
        return []

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "head_pan_link"
            ),
            tip=robot_root._world.get_body_in_branch_by_name(
                robot_root, "head_tilt_link"
            ),
        )


@dataclass(eq=False)
class HSRBTorso(ServoedHSRBPart, Torso, HasOneArm[HSRBArm], HasNeck[HSRBNeck]):

    @property
    def servoed_connections(self) -> List[ActiveConnection1DOF]:
        """
        The torso lift, which rises with the arm lift it mimics.
        """
        return [
            connection
            for connection in self.active_connections
            if isinstance(connection, ActiveConnection1DOF)
        ]

    def setup_hardware_interfaces(self):
        return

    def setup_joint_states(self) -> List[JointState]:
        torso_joint = self.active_connections
        torso_low = JointState.from_mapping(
            name=PrefixedName("torso_low", prefix=self.name.name),
            mapping=dict(zip(torso_joint, [0.0])),
            state_type=TorsoState.LOW,
        )

        torso_mid = JointState.from_mapping(
            name=PrefixedName("torso_mid", prefix=self.name.name),
            mapping=dict(zip(torso_joint, [0.32 / 2])),
            state_type=TorsoState.MID,
        )

        torso_high = JointState.from_mapping(
            name=PrefixedName("torso_high", prefix=self.name.name),
            mapping=dict(zip(torso_joint, [0.32])),
            state_type=TorsoState.HIGH,
        )

        return [torso_low, torso_mid, torso_high]

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(robot_root, "base_link"),
            tip=robot_root._world.get_body_in_branch_by_name(
                robot_root, "torso_lift_link"
            ),
        )


@dataclass(eq=False)
class HSRBBaseLidar(Lidar):
    """
    The Hokuyo scanner sweeping the floor around the HSRB's base.
    """

    @classmethod
    def with_source(
        cls, robot_root: KinematicStructureEntity, source: LidarSource
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(
                robot_root, "base_range_sensor_link"
            ),
            scan_pattern=ScanPattern(
                minimum_angle=-2.098758,
                maximum_angle=2.098758,
                angle_increment=0.0043633,
                minimum_range=0.012,
                maximum_range=60.0,
            ),
            source=source,
        )


@dataclass(eq=False)
class HSRBMobileBase(
    MobileBase[OmniDrive], HasTorso[HSRBTorso], HasLidar[HSRBBaseLidar]
):

    @classproperty
    def topic_name(cls) -> str:
        """
        The topic the HSRB publishes the pose of its base on.
        """
        return HSRBTopic.ODOMETRY

    @classproperty
    def forward_axis(cls) -> Vector3:
        return Vector3.X()

    full_body_controlled: bool = field(default=True, kw_only=True)

    @classproperty
    def passive_joint_armature(cls) -> float:
        """
        The rotor inertia given to the base's passive joints in a physical simulation.

        The wheels are light and damped, and MuJoCo's RK4 integration of them diverges
        within milliseconds when they have none; this much keeps them stable.
        """
        return 0.01

    @property
    def drive_servos(self) -> DriveServos:
        """
        Servos tuned for the base, which carries the whole robot, to follow its
        commanded velocity; they are not taken from the real robot.
        """
        return DriveServos(
            translation_gains=VelocityServoGains(
                damping=20_000.0, torque_limit=3_000.0
            ),
            rotation_gains=VelocityServoGains(damping=1_000.0, torque_limit=1_000.0),
        )

    @property
    def passive_connections(self) -> List[ActiveConnection1DOF]:
        """
        The joint the base rolls on and every wheel and caster joint below it.

        No
        controller drives them: the base is moved as a whole by its drive.
        """
        roll_link = self._world.get_body_in_branch_by_name(self.root, "base_roll_link")
        return [
            entity.parent_connection
            for entity in self._world.get_kinematic_structure_entities_of_branch(
                roll_link
            )
            if isinstance(entity.parent_connection, ActiveConnection1DOF)
        ]

    @property
    def rolling_bodies(self) -> list[Body]:
        return [
            connection.child
            for connection in self.passive_connections
            if isinstance(connection.child, Body)
        ]

    def _setup_servos(self) -> None:
        super()._setup_servos()
        for connection in self.passive_connections:
            connection.dynamics = replace(
                connection.dynamics, armature=self.passive_joint_armature
            )

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls(
            root=robot_root._world.get_body_in_branch_by_name(robot_root, "base_link"),
        )


@dataclass(eq=False)
class HSRB(AbstractRobot, HasMobileBase[HSRBMobileBase]):
    """
    The HSRB Robot built by Toyota.

    https://robotsguide.com/robots/hsr
    """

    @classmethod
    def get_ros_file_path(cls) -> str:
        return "package://hsr_description/robots/hsrb4s.urdf.xacro"

    @classmethod
    def _get_root_body_name(cls) -> str:
        return "base_footprint"

    def _setup_collision_rules(self):
        srdf_path = os.path.join(
            Path(files("semantic_digital_twin")).parent.parent,
            "resources",
            "collision_configs",
            "hsrb.srdf",
        )
        self._world.collision_manager.add_ignore_collision_rule(
            SelfCollisionMatrixRule.from_collision_srdf(srdf_path, self._world)
        )
        self._world.collision_manager.add_default_rule(
            AvoidExternalCollisions(
                buffer_zone_distance=0.05, violated_distance=0.0, robot=self
            )
        )

        self._world.collision_manager.add_default_rule(
            AvoidExternalCollisions(
                buffer_zone_distance=0.1,
                violated_distance=0.03,
                robot=self,
                body_subset={
                    self._world.get_body_in_branch_by_name(self.root, "base_link")
                },
            )
        )
        self._world.collision_manager.add_default_rule(
            AvoidSelfCollisions(
                buffer_zone_distance=0.03,
                violated_distance=0.0,
                robot=self,
            )
        )

        self._world.collision_manager.max_avoided_bodies_rules.append(
            MaxAvoidedCollisionsOverride(
                2,
                bodies={self._world.get_body_in_branch_by_name(self.root, "base_link")},
            )
        )
        self._world.collision_manager.max_avoided_bodies_rules.append(
            MaxAvoidedCollisionsOverride(
                4,
                bodies=set(
                    self._world.get_direct_child_bodies_with_collision(
                        self._world.get_body_in_branch_by_name(
                            self.root, "wrist_roll_link"
                        )
                    )
                ),
            )
        )

    def _setup_velocity_limits(self):
        vel_limits = defaultdict(lambda: 1.0)
        self.tighten_dof_velocity_limits_of_1dof_connections(new_limits=vel_limits)

    @property
    def end_effector(self) -> HSRBGripper:
        return self.mobile_base.torso.arm.end_effector
