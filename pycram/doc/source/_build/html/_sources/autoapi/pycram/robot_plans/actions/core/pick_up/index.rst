pycram.robot_plans.actions.core.pick_up
=======================================

.. py:module:: pycram.robot_plans.actions.core.pick_up


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.core.pick_up.logger
   pycram.robot_plans.actions.core.pick_up.ReachActionDescription
   pycram.robot_plans.actions.core.pick_up.PickUpActionDescription
   pycram.robot_plans.actions.core.pick_up.GraspingActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.core.pick_up.ReachAction
   pycram.robot_plans.actions.core.pick_up.PickUpAction
   pycram.robot_plans.actions.core.pick_up.GraspingAction


Module Contents
---------------

.. py:data:: logger

.. py:class:: ReachAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Let the robot reach a specific pose.


   .. py:attribute:: target_pose
      :type:  pycram.datastructures.pose.PoseStamped

      Pose that should be reached.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that should be used for pick up



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      The grasp description that should be used for picking up the object



   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body
      :value: None


      Object designator_description describing the object that should be picked up



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if object is contained in the gripper such that it can be grasped and picked up.



   .. py:method:: description(target_pose: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasp_description: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.grasp.GraspDescription], pycram.datastructures.grasp.GraspDescription] = None, object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[ReachAction]]
      :classmethod:



.. py:class:: PickUpAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Let the robot pick up an object.


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be picked up



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that should be used for pick up



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      The GraspDescription that should be used for picking up the object



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if picked up object is in contact with the gripper.



   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasp_description: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.grasp.GraspDescription], pycram.datastructures.grasp.GraspDescription] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[PickUpAction]]
      :classmethod:



.. py:class:: GraspingAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Grasps an object described by the given Object Designator description


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object Designator for the object that should be grasped



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that should be used to grasp



   .. py:attribute:: prepose_distance
      :type:  float

      The distance in meters the gripper should be at before grasping the object



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, prepose_distance: typing_extensions.Union[typing_extensions.Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[GraspingAction]]
      :classmethod:



.. py:data:: ReachActionDescription

.. py:data:: PickUpActionDescription

.. py:data:: GraspingActionDescription

