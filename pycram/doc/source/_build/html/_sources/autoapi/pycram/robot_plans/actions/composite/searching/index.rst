pycram.robot_plans.actions.composite.searching
==============================================

.. py:module:: pycram.robot_plans.actions.composite.searching


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.composite.searching.SearchActionDescription


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.composite.searching.SearchAction


Module Contents
---------------

.. py:class:: SearchAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Searches for a target object around the given location.


   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Location around which to look for a target object.



   .. py:attribute:: object_sem_annotation
      :type:  typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]

      Type of the object which is searched for.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], object_type: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.SemanticAnnotation], semantic_digital_twin.world_description.world_entity.SemanticAnnotation]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[SearchAction]]
      :classmethod:



.. py:data:: SearchActionDescription

