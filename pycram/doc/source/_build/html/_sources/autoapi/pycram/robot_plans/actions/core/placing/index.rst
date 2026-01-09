pycram.robot_plans.actions.core.placing
=======================================

.. py:module:: pycram.robot_plans.actions.core.placing


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.core.placing.PlaceActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.core.placing.PlaceAction


Module Contents
---------------

.. py:class:: PlaceAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Places an Object at a position using an arm.


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be place



   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Pose in the world at which the object should be placed



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that is currently holding the object



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if the object is placed at the target location.



   .. py:method:: validate_loss_of_contact()

      Check if the object is still in contact with the robot after placing it.



   .. py:method:: validate_placement_location()

      Check if the object is placed at the target location.



   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[PlaceAction]]
      :classmethod:



.. py:data:: PlaceActionDescription

