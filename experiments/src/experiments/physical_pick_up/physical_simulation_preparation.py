"""
What a robot description lacks before the robot can be simulated physically, and the
values that fill the gap for the PR2.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
from typing_extensions import Dict, List

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.spatial_types.derivatives import DerivativeMap
from semantic_digital_twin.world_description.connection_properties import (
    JointDynamics,
    JointServo,
    ServoGains,
)
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.inertial_properties import (
    InertiaTensor,
    PrincipalMoments,
)
from semantic_digital_twin.world_description.world_entity import (
    Body,
    GravityCompensation,
    PositionServo,
)

# %% preparing any robot


@dataclass
class PhysicalSimulationPreparation(ABC):
    """
    Adds to a robot what its description leaves out but a physical simulation needs.

    A robot description states the robot's kinematics and geometry. Driving the robot
    through physics additionally needs a servo on every joint that is commanded, limits
    such a servo can be clamped to, inertia a rigid body can actually have, and something
    carrying the links' weight. Each robot states its own servos by subclassing.
    """

    robot: AbstractRobot
    """
    The robot to prepare, already spawned into its world.
    """

    continuous_joint_position_limit: float = 2 * math.pi
    """
    The position, in both directions, a joint without position limits is limited to; a
    servo is clamped to the limits of the joint it drives.
    """

    @abstractmethod
    def servos_by_connection(self) -> Dict[ActiveConnection1DOF, JointServo]:
        """
        :return: The servo driving each joint that is commanded. Joints sharing a degree
            of freedom, such as the coupled joints of a gripper, are driven by the first
            of them listed.
        """

    def apply(self) -> None:
        """
        Prepare :attr:`robot` for physical simulation.

        Has to run before the simulation is built from the robot's world.
        """
        self._repair_inertia_tensors()
        self._declare_servos()
        self._compensate_gravity()

    def _repair_inertia_tensors(self) -> None:
        """
        Shrink the largest principal moment of every body whose moments no rigid body
        can have to the sum of the other two, the largest value that is possible.
        """
        for body in self._bodies_with_inertia():
            moments, axes = body.inertial.inertia.to_principal_moments_and_axes()
            values = moments.data.flatten()
            smallest, middle, largest = np.sort(values)
            if smallest + middle >= largest:
                continue
            repaired = np.where(values == largest, smallest + middle, values)
            body.inertial.inertia = InertiaTensor.from_principal_moments_and_axes(
                PrincipalMoments.from_values(*repaired), axes
            )

    def _bodies_with_inertia(self) -> List[Body]:
        """
        :return: The robot's bodies that state inertial properties.
        """
        return [body for body in self.robot.bodies if body.inertial is not None]

    def _declare_servos(self) -> None:
        """
        Drive every joint of :meth:`servos_by_connection` with its servo.
        """
        world = self.robot._world
        with world.modify_world():
            for connection, servo in self.servos_by_connection().items():
                connection.dynamics = servo.dynamics
                degree_of_freedom = connection.raw_dof
                if any(
                    degree_of_freedom in actuator.dofs for actuator in world.actuators
                ):
                    continue
                self._limit_continuous_joint(connection)
                actuator = PositionServo(
                    name=PrefixedName(
                        f"{degree_of_freedom.name.name}_servo",
                        prefix=degree_of_freedom.name.prefix,
                    ),
                    gains=servo.gains,
                )
                actuator.add_dof(degree_of_freedom)
                world.add_actuator(actuator)

    def _limit_continuous_joint(self, connection: ActiveConnection1DOF) -> None:
        """
        Give a joint without position limits those of
        :attr:`continuous_joint_position_limit`.

        :param connection: The joint; one that has limits already keeps them.
        """
        degree_of_freedom = connection.raw_dof
        if degree_of_freedom.limits.lower.position is not None:
            return
        degree_of_freedom._overwrite_dof_limits(
            new_lower_limits=DerivativeMap(
                -self.continuous_joint_position_limit, None, None, None
            ),
            new_upper_limits=DerivativeMap(
                self.continuous_joint_position_limit, None, None, None
            ),
        )

    def _compensate_gravity(self) -> None:
        """
        Let the simulation carry the weight of the robot's bodies, as the controllers of
        the real robot do.
        """
        for body in self.robot.bodies:
            compensation = body.get_simulator_property_of_type(GravityCompensation)
            if compensation is None:
                body.add_simulator_property(GravityCompensation(fraction=1.0))
                continue
            compensation.fraction = 1.0


# %% preparing the PR2


@dataclass
class PR2PhysicalSimulationPreparation(PhysicalSimulationPreparation):
    """
    Prepares a PR2: servos on its torso, both arms and both grippers.

    The PR2's description is a URDF, which says nothing about what drives its joints,
    and no tuned MuJoCo model of the PR2 is available to read that from. The gains were
    therefore raised by hand until the arms track the commanded motion and the torso
    holds its height.
    """

    robot: PR2

    arm_servo: JointServo = field(
        default_factory=lambda: JointServo(
            gains=ServoGains(stiffness=2_000.0, damping=200.0, torque_limit=100.0),
            dynamics=JointDynamics(armature=0.1, damping=2.0),
        )
    )
    """
    Drives each of the seven joints of an arm.
    """

    torso_servo: JointServo = field(
        default_factory=lambda: JointServo(
            gains=ServoGains(
                stiffness=50_000.0, damping=5_000.0, torque_limit=10_000.0
            ),
            dynamics=JointDynamics(armature=1.0, damping=100.0),
        )
    )
    """
    Drives the torso's lift joint, which carries both arms and the head.
    """

    grip_torque: float = 8.0
    """
    The largest torque the finger servos exert, in newton meters.

    This is the grip strength: fingers commanded to close on an object stall against it
    and press with exactly this torque.
    """

    @property
    def finger_servo(self) -> JointServo:
        """
        Drives the finger joints of a gripper, which are coupled into one degree of
        freedom.
        """
        return JointServo(
            gains=ServoGains(
                stiffness=100.0, damping=5.0, torque_limit=self.grip_torque
            ),
            dynamics=JointDynamics(armature=0.01, damping=0.5),
        )

    def servos_by_connection(self) -> Dict[ActiveConnection1DOF, JointServo]:
        servos = {
            connection: self.torso_servo
            for connection in self.robot.mobile_base.torso.active_connections
        }
        for arm in self.robot.all_arms:
            servos.update(
                {connection: self.arm_servo for connection in arm.active_connections}
            )
            open_gripper = arm.end_effector.get_joint_state_by_type(GripperState.OPEN)
            servos.update(
                {
                    connection: self.finger_servo
                    for connection in open_gripper.connections
                }
            )
        return servos
