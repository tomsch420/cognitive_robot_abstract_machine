pycram.joint_state
==================

.. py:module:: pycram.joint_state


Classes
-------

.. autoapisummary::

   pycram.joint_state.JointState
   pycram.joint_state.ArmState
   pycram.joint_state.GripperState
   pycram.joint_state.JointStateManager


Module Contents
---------------

.. py:class:: JointState

   Represents a named joint state of a robot. For example, the park position of the arms.


   .. py:attribute:: name
      :type:  semantic_digital_twin.datastructures.prefixed_name.PrefixedName

      Name of the joint state



   .. py:attribute:: joint_names
      :type:  List[str]

      Names of the joints in this state



   .. py:attribute:: joint_positions
      :type:  List[float]

      position of the joints in this state, must correspond to the joint_names



   .. py:attribute:: state_type
      :type:  enum.Enum
      :value: None


      Enum type of the joints tate (e.g., Park, Open)



   .. py:method:: apply_to_world(world: semantic_digital_twin.world.World)

      Applies the joint state to the robot in the given world.
      :param world: The world in which the robot is located.



.. py:class:: ArmState

   Bases: :py:obj:`JointState`


   Represents a named joint state of a robot. For example, the park position of the arms.


   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms
      :value: None



.. py:class:: GripperState

   Bases: :py:obj:`JointState`


   Represents the state of a gripper, such as open or closed.


   .. py:attribute:: gripper
      :type:  pycram.datastructures.enums.Arms
      :value: None



.. py:class:: JointStateManager

   Manages joint states for different robot arms and their configurations.


   .. py:attribute:: joint_states
      :type:  Dict[Type[semantic_digital_twin.robots.abstract_robot.AbstractRobot], List[JointState]]

      A list of joint states that can be applied to the robot.



   .. py:method:: get_arm_state(arm: pycram.datastructures.enums.Arms, state_type: pycram.datastructures.enums.StaticJointState, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> Union[ArmState, None]

      Retrieves the joint state for a specific arm and state type.

      :param arm: The arm for which the state is requested.
      :param state_type: The type of state (e.g., Park).
      :param robot_view: The robot view to which the arm belongs.
      :return: The corresponding ArmState or None if not found.



   .. py:method:: get_gripper_state(gripper: pycram.datastructures.enums.Arms, state_type: pycram.datastructures.enums.StaticJointState, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> Union[GripperState, None]

      Retrieves the joint state for a specific gripper and state type.

      :param gripper: The gripper for which the state is requested.
      :param state_type: The type of state (e.g., Open, Close).
      :param robot_view: The robot view to which the gripper belongs.
      :return: The corresponding GripperState or None if not found.



   .. py:method:: get_joint_state(state: enum.Enum, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> List[JointState]

      Retrieves all joint states of a specific type for a given robot.

      :param state: The type of joint state to retrieve (e.g., Park, Open).
      :param robot_view: The robot class for which the joint states are requested.
      :return: A list of JointState objects matching the specified type.



   .. py:method:: add_joint_states(robot: Type[semantic_digital_twin.robots.abstract_robot.AbstractRobot], joint_states: List[JointState])

      Adds joint states for a specific robot type.

      :param robot: The robot class for which the joint states are added.
      :param joint_states: A list of joint states to be added.



