pycram.robot_plans.motions.robot_body
=====================================

.. py:module:: pycram.robot_plans.motions.robot_body


Classes
-------

.. autoapisummary::

   pycram.robot_plans.motions.robot_body.MoveJointsMotion
   pycram.robot_plans.motions.robot_body.LookingMotion


Module Contents
---------------

.. py:class:: MoveJointsMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Moves any joint on the robot


   .. py:attribute:: names
      :type:  list

      List of joint names that should be moved 



   .. py:attribute:: positions
      :type:  list

      Target positions of joints, should correspond to the list of names



   .. py:attribute:: align
      :type:  Optional[bool]
      :value: False


      If True, aligns the end-effector with a specified axis (optional).



   .. py:attribute:: tip_link
      :type:  Optional[str]
      :value: None


      Name of the tip link to align with, e.g the object (optional).



   .. py:attribute:: tip_normal
      :type:  Optional[pycram.datastructures.pose.Vector3Stamped]
      :value: None


      Normalized vector representing the current orientation axis of the end-effector (optional).



   .. py:attribute:: root_link
      :type:  Optional[str]
      :value: None


      Base link of the robot; typically set to the torso (optional).



   .. py:attribute:: root_normal
      :type:  Optional[pycram.datastructures.pose.Vector3Stamped]
      :value: None


      Normalized vector representing the desired orientation axis to align with (optional).



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: LookingMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Lets the robot look at a point


   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      Target pose to look at



   .. py:method:: perform()


   .. py:property:: _motion_chart


