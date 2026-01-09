pycram.robot_plans.actions.composite.facing
===========================================

.. py:module:: pycram.robot_plans.actions.composite.facing


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.composite.facing.FaceAtActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.composite.facing.FaceAtAction


Module Contents
---------------

.. py:class:: FaceAtAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.


   .. py:attribute:: pose
      :type:  pycram.datastructures.pose.PoseStamped

      The pose to face 



   .. py:attribute:: keep_joint_states
      :type:  bool

      Keep the joint states of the robot the same during the navigation.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(pose: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], keep_joint_states: typing_extensions.Union[typing_extensions.Iterable[bool], bool] = ActionConfig.face_at_keep_joint_states) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[FaceAtAction]]
      :classmethod:



.. py:data:: FaceAtActionDescription

