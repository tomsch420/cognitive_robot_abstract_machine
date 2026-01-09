pycram.robot_plans.motions.gripper
==================================

.. py:module:: pycram.robot_plans.motions.gripper


Classes
-------

.. autoapisummary::

   pycram.robot_plans.motions.gripper.ReachMotion
   pycram.robot_plans.motions.gripper.MoveGripperMotion
   pycram.robot_plans.motions.gripper.MoveTCPMotion
   pycram.robot_plans.motions.gripper.MoveTCPWaypointsMotion


Module Contents
---------------

.. py:class:: ReachMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be picked up



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that should be used for pick up



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      The grasp description that should be used for picking up the object



   .. py:attribute:: movement_type
      :type:  pycram.datastructures.enums.MovementType

      The type of movement that should be performed.



   .. py:attribute:: reverse_pose_sequence
      :type:  bool
      :value: False


      Reverses the sequence of poses, i.e., moves away from the object instead of towards it. Used for placing objects.



   .. py:method:: _calculate_pose_sequence() -> List[pycram.datastructures.pose.PoseStamped]


   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: MoveGripperMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Opens or closes the gripper


   .. py:attribute:: motion
      :type:  pycram.datastructures.enums.GripperState

      Motion that should be performed, either 'open' or 'close'



   .. py:attribute:: gripper
      :type:  pycram.datastructures.enums.Arms

      Name of the gripper that should be moved



   .. py:attribute:: allow_gripper_collision
      :type:  Optional[bool]
      :value: None


      If the gripper is allowed to collide with something



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: MoveTCPMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Moves the Tool center point (TCP) of the robot


   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      Target pose to which the TCP should be moved



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm with the TCP that should be moved to the target



   .. py:attribute:: allow_gripper_collision
      :type:  Optional[bool]
      :value: None


      If the gripper can collide with something



   .. py:attribute:: movement_type
      :type:  Optional[pycram.datastructures.enums.MovementType]

      The type of movement that should be performed.



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: MoveTCPWaypointsMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Moves the Tool center point (TCP) of the robot


   .. py:attribute:: waypoints
      :type:  List[pycram.datastructures.pose.PoseStamped]

      Waypoints the TCP should move along 



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm with the TCP that should be moved to the target



   .. py:attribute:: allow_gripper_collision
      :type:  Optional[bool]
      :value: None


      If the gripper can collide with something



   .. py:attribute:: movement_type
      :type:  pycram.datastructures.enums.WaypointsMovementType

      The type of movement that should be performed.



   .. py:method:: perform()


   .. py:property:: _motion_chart


