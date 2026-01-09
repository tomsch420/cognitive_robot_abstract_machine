pycram.robot_plans.actions.composite.tool_based
===============================================

.. py:module:: pycram.robot_plans.actions.composite.tool_based


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.composite.tool_based.CuttingActionDescription
   pycram.robot_plans.actions.composite.tool_based.PouringActionDescription
   pycram.robot_plans.actions.composite.tool_based.MixingActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.composite.tool_based.MixingAction
   pycram.robot_plans.actions.composite.tool_based.PouringAction
   pycram.robot_plans.actions.composite.tool_based.CuttingAction


Module Contents
---------------

.. py:class:: MixingAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Mixes contents of an object using a tool in a spiral motion.


   .. py:attribute:: object_
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object to be mixed.



   .. py:attribute:: tool
      :type:  semantic_digital_twin.world_description.world_entity.SemanticAnnotation

      The tool to be used for mixing.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to be used for the mixing action.



   .. py:attribute:: technique
      :type:  typing_extensions.Optional[str]
      :value: None


      The technique to be used for mixing, e.g. 'Spiral Mixing'.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: description(object_: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], tool: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.SemanticAnnotation], semantic_digital_twin.world_description.world_entity.SemanticAnnotation], arm: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]] = None, technique: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[str], str]] = None)
      :classmethod:



.. py:class:: PouringAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Performs a pouring action with a tool over an object, typically used for liquids.


   .. py:attribute:: object_
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object over which the pouring action is performed.



   .. py:attribute:: tool
      :type:  semantic_digital_twin.world_description.world_entity.SemanticAnnotation

      The tool used for pouring, e.g., a jug or a bottle.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to be used for the pouring action.



   .. py:attribute:: technique
      :type:  typing_extensions.Optional[str]
      :value: None


      The technique to be used for pouring, e.g., 'Pouring'.



   .. py:attribute:: angle
      :type:  typing_extensions.Optional[float]
      :value: 90


      The angle at which the tool is tilted during the pouring action, in degrees.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: description(object_: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], tool: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.SemanticAnnotation], semantic_digital_twin.world_description.world_entity.SemanticAnnotation], arm: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]] = None, technique: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[str], str]] = None, angle: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[float], float]] = 90)
      :classmethod:



.. py:class:: CuttingAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Performs a cutting action on an object using a specified tool.


   .. py:attribute:: object_
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object to be cut.



   .. py:attribute:: tool
      :type:  semantic_digital_twin.world_description.world_entity.SemanticAnnotation

      The tool used for cutting, e.g., a knife or a saw.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to be used for the cutting action.



   .. py:attribute:: technique
      :type:  typing_extensions.Optional[str]
      :value: None


      The technique to be used for cutting, e.g., 'Slicing', 'Halving', etc.



   .. py:attribute:: slice_thickness
      :type:  typing_extensions.Optional[float]
      :value: 0.03


      The thickness of each slice to be cut from the object, in meters.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: description(object_: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], tool: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.SemanticAnnotation], semantic_digital_twin.world_description.world_entity.SemanticAnnotation], arm: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]] = None, technique: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[str], str]] = None, slice_thickness: typing_extensions.Optional[float] = 0.03)
      :classmethod:



   .. py:method:: calculate_slices(obj_length)


   .. py:method:: perpendicular_pose(slice_pose, angle) -> pycram.datastructures.pose.PoseStamped
      :staticmethod:



   .. py:method:: get_rotation_offset_from_axis_preference(pose_a, pose_b: pycram.datastructures.pose.PoseStamped) -> Tuple[int, float]
      :staticmethod:


      Compute a discrete rotation offset (-90 or 90 degrees) to align this pose's local axes with the direction
      toward a target pose, based on which axis (X or Y) is more aligned.

      :param pose_a: The source pose.
      :param pose_b: The target pose to align with.
      :return: Tuple of (rotation offset in degrees, signed angle difference in radians for Y axis).



.. py:data:: CuttingActionDescription

.. py:data:: PouringActionDescription

.. py:data:: MixingActionDescription

