pycram.perception
=================

.. py:module:: pycram.perception


Classes
-------

.. autoapisummary::

   pycram.perception.PerceptionQuery


Module Contents
---------------

.. py:class:: PerceptionQuery

   .. py:attribute:: semantic_annotation
      :type:  typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]

      The semantic annotation for which to perceive



   .. py:attribute:: region
      :type:  semantic_digital_twin.world_description.geometry.BoundingBox

      The region in which the object should be detected



   .. py:attribute:: robot
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot

      '
      Robot annotation of the robot that should perceive the object.



   .. py:attribute:: world
      :type:  semantic_digital_twin.world.World

      The world in which the object should be detected.



   .. py:method:: from_world() -> typing_extensions.List[semantic_digital_twin.world_description.world_entity.Body]


   .. py:method:: from_robokudo()


