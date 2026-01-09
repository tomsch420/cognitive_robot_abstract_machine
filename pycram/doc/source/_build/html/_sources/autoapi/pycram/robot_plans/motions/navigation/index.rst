pycram.robot_plans.motions.navigation
=====================================

.. py:module:: pycram.robot_plans.motions.navigation


Classes
-------

.. autoapisummary::

   pycram.robot_plans.motions.navigation.MoveMotion


Module Contents
---------------

.. py:class:: MoveMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Moves the robot to a designated location


   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      Location to which the robot should be moved



   .. py:attribute:: keep_joint_states
      :type:  bool
      :value: False


      Keep the joint states of the robot during/at the end of the motion



   .. py:method:: perform()


   .. py:property:: _motion_chart


