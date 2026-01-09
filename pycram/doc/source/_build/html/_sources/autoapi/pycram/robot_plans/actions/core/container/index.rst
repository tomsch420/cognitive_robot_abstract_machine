pycram.robot_plans.actions.core.container
=========================================

.. py:module:: pycram.robot_plans.actions.core.container


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.core.container.OpenActionDescription
   pycram.robot_plans.actions.core.container.CloseActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.core.container.OpenAction
   pycram.robot_plans.actions.core.container.CloseAction


Functions
---------

.. autoapisummary::

   pycram.robot_plans.actions.core.container.validate_close_open
   pycram.robot_plans.actions.core.container.check_opened
   pycram.robot_plans.actions.core.container.check_closed


Module Contents
---------------

.. py:class:: OpenAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Opens a container like object


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be opened



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used for opening the container



   .. py:attribute:: grasping_prepose_distance
      :type:  float

      The distance in meters the gripper should be at in the x-axis away from the handle.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if the container is opened, this assumes that the container state can be read accurately from the
      real world.



   .. py:method:: description(object_designator_description: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasping_prepose_distance: typing_extensions.Union[typing_extensions.Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[OpenAction]]
      :classmethod:



.. py:class:: CloseAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Closes a container like object.


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be closed



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used for closing



   .. py:attribute:: grasping_prepose_distance
      :type:  float

      The distance in meters between the gripper and the handle before approaching to grasp.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if the container is closed, this assumes that the container state can be read accurately from the
      real world.



   .. py:method:: description(object_designator_description: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasping_prepose_distance: typing_extensions.Union[typing_extensions.Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[CloseAction]]
      :classmethod:



.. py:function:: validate_close_open(object_designator: semantic_digital_twin.world_description.world_entity.Body, arm: pycram.datastructures.enums.Arms, action_type: typing_extensions.Union[typing_extensions.Type[OpenAction], typing_extensions.Type[CloseAction]])

   Validates if the container is opened or closed by checking the joint position of the container.

   :param object_designator: The object designator_description describing the object that should be opened or closed.
   :param arm: The arm that should be used for opening or closing the container.
   :param action_type: The type of the action that should be validated.


.. py:function:: check_opened(joint_obj: semantic_digital_twin.world_description.world_entity.Connection, obj_part: semantic_digital_twin.world_description.world_entity.Body, arm: pycram.datastructures.enums.Arms, upper_limit: float)

.. py:function:: check_closed(joint_obj: semantic_digital_twin.world_description.world_entity.Connection, obj_part: semantic_digital_twin.world_description.world_entity.Body, arm: pycram.datastructures.enums.Arms, lower_limit: float)

.. py:data:: OpenActionDescription

.. py:data:: CloseActionDescription

