pycram.robot_plans.motions.misc
===============================

.. py:module:: pycram.robot_plans.motions.misc


Classes
-------

.. autoapisummary::

   pycram.robot_plans.motions.misc.DetectingMotion


Module Contents
---------------

.. py:class:: DetectingMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Tries to detect an object in the FOV of the robot

   returns: ObjectDesignatorDescription.Object or Error: PerceptionObjectNotFound


   .. py:attribute:: query
      :type:  pycram.perception.PerceptionQuery

      Query for the perception system that should be answered



   .. py:method:: perform()


   .. py:property:: _motion_chart


