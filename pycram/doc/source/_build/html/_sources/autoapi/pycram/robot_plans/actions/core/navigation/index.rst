pycram.robot_plans.actions.core.navigation
==========================================

.. py:module:: pycram.robot_plans.actions.core.navigation


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.core.navigation.NavigateActionDescription
   pycram.robot_plans.actions.core.navigation.LookAtActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.core.navigation.NavigateAction
   pycram.robot_plans.actions.core.navigation.LookAtAction


Module Contents
---------------

.. py:class:: NavigateAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Navigates the Robot to a position.


   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Location to which the robot should be navigated



   .. py:attribute:: keep_joint_states
      :type:  bool

      Keep the joint states of the robot the same during the navigation.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], keep_joint_states: typing_extensions.Union[typing_extensions.Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[NavigateAction]]
      :classmethod:



.. py:class:: LookAtAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Lets the robot look at a position.


   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      Position at which the robot should look, given as 6D pose



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if the robot is looking at the target location by spawning a virtual object at the target location and
      creating a ray from the camera and checking if it intersects with the object.



   .. py:method:: description(target: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[LookAtAction]]
      :classmethod:



.. py:data:: NavigateActionDescription

.. py:data:: LookAtActionDescription

