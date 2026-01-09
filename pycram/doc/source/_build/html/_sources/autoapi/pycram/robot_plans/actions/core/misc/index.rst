pycram.robot_plans.actions.core.misc
====================================

.. py:module:: pycram.robot_plans.actions.core.misc


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.core.misc.DetectActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.core.misc.DetectAction


Module Contents
---------------

.. py:class:: DetectAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Detects an object that fits the object description and returns an object designator_description describing the object.

   If no object is found, an PerceptionObjectNotFound error is raised.


   .. py:attribute:: technique
      :type:  pycram.datastructures.enums.DetectionTechnique

      The technique that should be used for detection



   .. py:attribute:: state
      :type:  typing_extensions.Optional[pycram.datastructures.enums.DetectionState]
      :value: None


      The state of the detection, e.g Start Stop for continues perception



   .. py:attribute:: object_sem_annotation
      :type:  typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]
      :value: None


      The type of the object that should be detected, only considered if technique is equal to Type



   .. py:attribute:: region
      :type:  typing_extensions.Optional[semantic_digital_twin.world_description.world_entity.Region]
      :value: None


      The region in which the object should be detected



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(technique: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.DetectionTechnique], pycram.datastructures.enums.DetectionTechnique], state: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.DetectionState], pycram.datastructures.enums.DetectionState] = None, object_sem_annotation: typing_extensions.Union[typing_extensions.Iterable[typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]], typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]] = None, region: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Region], semantic_digital_twin.world_description.world_entity.Region] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[DetectAction]]
      :classmethod:



.. py:data:: DetectActionDescription

