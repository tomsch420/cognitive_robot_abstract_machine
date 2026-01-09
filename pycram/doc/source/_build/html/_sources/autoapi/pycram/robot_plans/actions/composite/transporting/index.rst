pycram.robot_plans.actions.composite.transporting
=================================================

.. py:module:: pycram.robot_plans.actions.composite.transporting


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.composite.transporting.TransportActionDescription
   pycram.robot_plans.actions.composite.transporting.PickAndPlaceActionDescription
   pycram.robot_plans.actions.composite.transporting.MoveAndPlaceActionDescription
   pycram.robot_plans.actions.composite.transporting.MoveAndPickUpActionDescription
   pycram.robot_plans.actions.composite.transporting.EfficientTransportActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.composite.transporting.TransportAction
   pycram.robot_plans.actions.composite.transporting.PickAndPlaceAction
   pycram.robot_plans.actions.composite.transporting.MoveAndPlaceAction
   pycram.robot_plans.actions.composite.transporting.MoveAndPickUpAction
   pycram.robot_plans.actions.composite.transporting.EfficientTransportAction


Module Contents
---------------

.. py:class:: TransportAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Transports an object to a position using an arm


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be transported.



   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Target Location to which the object should be transported



   .. py:attribute:: arm
      :type:  typing_extensions.Optional[pycram.datastructures.enums.Arms]

      Arm that should be used



   .. py:attribute:: place_rotation_agnostic
      :type:  typing_extensions.Optional[bool]
      :value: False


      If True, the robot will place the object in the same orientation as it is itself, no matter how the object was grasped.



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: inside_container() -> List[semantic_digital_twin.world_description.world_entity.Body]


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, place_rotation_agnostic: typing_extensions.Optional[bool] = False) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[TransportAction]]
      :classmethod:



.. py:class:: PickAndPlaceAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Transports an object to a position using an arm without moving the base of the robot


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be transported.



   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Target Location to which the object should be transported



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      Description of the grasp to pick up the target



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasp_description=GraspDescription) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[PickAndPlaceAction]]
      :classmethod:



.. py:class:: MoveAndPlaceAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Navigate to `standing_position`, then turn towards the object and pick it up.


   .. py:attribute:: standing_position
      :type:  pycram.datastructures.pose.PoseStamped

      The pose to stand before trying to pick up the object



   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object to pick up



   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      The location to place the object.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to use



   .. py:attribute:: keep_joint_states
      :type:  bool

      Keep the joint states of the robot the same during the navigation.



   .. py:method:: execute()

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(standing_position: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, keep_joint_states: typing_extensions.Union[typing_extensions.Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[MoveAndPlaceAction]]
      :classmethod:



.. py:class:: MoveAndPickUpAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Navigate to `standing_position`, then turn towards the object and pick it up.


   .. py:attribute:: standing_position
      :type:  pycram.datastructures.pose.PoseStamped

      The pose to stand before trying to pick up the object



   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object to pick up



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to use



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      The grasp to use



   .. py:attribute:: keep_joint_states
      :type:  bool

      Keep the joint states of the robot the same during the navigation.



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute()

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(standing_position: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], object_designator: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasp_description: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Grasp], pycram.datastructures.enums.Grasp] = None, keep_joint_states: typing_extensions.Union[typing_extensions.Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[MoveAndPickUpAction]]
      :classmethod:



.. py:class:: EfficientTransportAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   To transport an object to a target location by choosing the closest
   available arm using simple Euclidean distance.


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body


   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped


   .. py:method:: _choose_best_arm(robot: semantic_digital_twin.world_description.world_entity.Body, obj: semantic_digital_twin.world_description.world_entity.Body) -> pycram.datastructures.enums.Arms

      Function to find the closest available arm.



   .. py:method:: execute() -> None

      The main plan for the transport action, optimized for a stationary robot.



   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[EfficientTransportAction]]
      :classmethod:



.. py:data:: TransportActionDescription

.. py:data:: PickAndPlaceActionDescription

.. py:data:: MoveAndPlaceActionDescription

.. py:data:: MoveAndPickUpActionDescription

.. py:data:: EfficientTransportActionDescription

