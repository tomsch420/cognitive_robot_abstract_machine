pycram.robot_plans.motions.container
====================================

.. py:module:: pycram.robot_plans.motions.container


Classes
-------

.. autoapisummary::

   pycram.robot_plans.motions.container.OpeningMotion
   pycram.robot_plans.motions.container.ClosingMotion


Module Contents
---------------

.. py:class:: OpeningMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Designator for opening container


   .. py:attribute:: object_part
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator for the drawer handle



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: ClosingMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Designator for closing a container


   .. py:attribute:: object_part
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator for the drawer handle



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used



   .. py:method:: perform()


   .. py:property:: _motion_chart


