pycram.robot_plans.actions.core.robot_body
==========================================

.. py:module:: pycram.robot_plans.actions.core.robot_body


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.core.robot_body.MoveTorsoActionDescription
   pycram.robot_plans.actions.core.robot_body.SetGripperActionDescription
   pycram.robot_plans.actions.core.robot_body.ParkArmsActionDescription
   pycram.robot_plans.actions.core.robot_body.CarryActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.core.robot_body.MoveTorsoAction
   pycram.robot_plans.actions.core.robot_body.SetGripperAction
   pycram.robot_plans.actions.core.robot_body.ParkArmsAction
   pycram.robot_plans.actions.core.robot_body.CarryAction


Module Contents
---------------

.. py:class:: MoveTorsoAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Move the torso of the robot up and down.


   .. py:attribute:: torso_state
      :type:  pycram.robot_descriptions.hsrb_states.TorsoState

      The state of the torso that should be set



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: datetime.timedelta = timedelta(seconds=2))

      Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.



   .. py:method:: description(torso_state: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.TorsoState], pycram.robot_descriptions.hsrb_states.TorsoState]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[MoveTorsoAction]]
      :classmethod:



.. py:class:: SetGripperAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Set the gripper state of the robot.


   .. py:attribute:: gripper
      :type:  pycram.robot_descriptions.hsrb_states.Arms

      The gripper that should be set 



   .. py:attribute:: motion
      :type:  pycram.robot_descriptions.hsrb_states.GripperStateEnum

      The motion that should be set on the gripper



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: datetime.timedelta = timedelta(seconds=2))

      Needs gripper state to be read or perceived.



   .. py:method:: description(gripper: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.Arms], pycram.robot_descriptions.hsrb_states.Arms], motion: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.GripperState], pycram.robot_descriptions.hsrb_states.GripperState] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[SetGripperAction]]
      :classmethod:



.. py:class:: ParkArmsAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Park the arms of the robot.


   .. py:attribute:: arm
      :type:  pycram.robot_descriptions.hsrb_states.Arms

      Entry from the enum for which arm should be parked.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: get_joint_poses() -> Tuple[List[str], List[float]]

      :return: The joint positions that should be set for the arm to be in the park position.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: datetime.timedelta = timedelta(seconds=2))

      Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.



   .. py:method:: description(arm: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.Arms], pycram.robot_descriptions.hsrb_states.Arms]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[ParkArmsAction]]
      :classmethod:



.. py:class:: CarryAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Parks the robot's arms. And align the arm with the given Axis of a frame.


   .. py:attribute:: arm
      :type:  pycram.robot_descriptions.hsrb_states.Arms

      Entry from the enum for which arm should be parked.



   .. py:attribute:: align
      :type:  typing_extensions.Optional[bool]
      :value: False


      If True, aligns the end-effector with a specified axis.



   .. py:attribute:: tip_link
      :type:  typing_extensions.Optional[str]
      :value: None


      Name of the tip link to align with, e.g the object.



   .. py:attribute:: tip_axis
      :type:  typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier]
      :value: None


      Tip axis of the tip link, that should be aligned.



   .. py:attribute:: root_link
      :type:  typing_extensions.Optional[str]
      :value: None


      Base link of the robot; typically set to the torso.



   .. py:attribute:: root_axis
      :type:  typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier]
      :value: None


      Goal axis of the root link, that should be used to align with.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: get_joint_poses() -> typing_extensions.Dict[str, float]

      :return: The joint positions that should be set for the arm to be in the park position.



   .. py:method:: axis_to_vector3_stamped(axis: pycram.datastructures.enums.AxisIdentifier, link: str = 'base_link') -> pycram.datastructures.pose.Vector3Stamped


   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: datetime.timedelta = timedelta(seconds=2))

      Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.



   .. py:method:: description(arm: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.Arms], pycram.robot_descriptions.hsrb_states.Arms], align: typing_extensions.Optional[bool] = False, tip_link: typing_extensions.Optional[str] = None, tip_axis: typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier] = None, root_link: typing_extensions.Optional[str] = None, root_axis: typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[CarryAction]]
      :classmethod:



.. py:data:: MoveTorsoActionDescription

.. py:data:: SetGripperActionDescription

.. py:data:: ParkArmsActionDescription

.. py:data:: CarryActionDescription

