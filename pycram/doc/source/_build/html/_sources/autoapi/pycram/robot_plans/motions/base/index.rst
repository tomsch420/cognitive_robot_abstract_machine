pycram.robot_plans.motions.base
===============================

.. py:module:: pycram.robot_plans.motions.base


Classes
-------

.. autoapisummary::

   pycram.robot_plans.motions.base.AlternativeMotionMapping
   pycram.robot_plans.motions.base.BaseMotion


Module Contents
---------------

.. py:class:: AlternativeMotionMapping

   Bases: :py:obj:`abc.ABC`


   Helper class that provides a standard way to create an ABC using
   inheritance.


   .. py:attribute:: robot_view
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot


   .. py:attribute:: motion
      :type:  BaseMotion


   .. py:property:: motion_chart
      :type: Task

      :abstractmethod:



   .. py:method:: check_for_alternative(robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot, motion: BaseMotion) -> Optional[Task]
      :staticmethod:



.. py:class:: BaseMotion

   Bases: :py:obj:`pycram.designator.DesignatorDescription`


   .. py:method:: perform()
      :abstractmethod:


      Passes this designator to the process module for execution. Will be overwritten by each motion.



   .. py:property:: motion_chart
      :type: Task



   .. py:property:: _motion_chart
      :type: Task

      :abstractmethod:



   .. py:method:: __post_init__()

      Checks if types are missing or wrong



