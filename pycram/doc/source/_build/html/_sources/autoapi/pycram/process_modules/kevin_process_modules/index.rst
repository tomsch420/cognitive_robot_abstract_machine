pycram.process_modules.kevin_process_modules
============================================

.. py:module:: pycram.process_modules.kevin_process_modules


Classes
-------

.. autoapisummary::

   pycram.process_modules.kevin_process_modules.KevinMoveArmJoints
   pycram.process_modules.kevin_process_modules.KevinManager


Module Contents
---------------

.. py:class:: KevinMoveArmJoints(lock)

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultMoveArmJoints`


   Process module for the simulated Kevin that moves the arm joints of the robot


   .. py:method:: _execute(desig: MoveArmJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: KevinManager

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultManager`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


   .. py:attribute:: robot_name
      :value: 'kevin'



   .. py:attribute:: _navigate_lock


   .. py:method:: navigate()

      Get the Process Module for navigating the robot with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for navigating



   .. py:method:: detecting()

      Get the Process Module for detecting an object with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for detecting an object



   .. py:method:: move_tcp()

      Get the Process Module for moving the Tool Center Point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the TCP



   .. py:method:: move_arm_joints()

      Get the Process Module for moving the joints of the robot arm
      with respect to the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the arm joints



   .. py:method:: world_state_detecting()


   .. py:method:: move_joints()

      Get the Process Module for moving any joint of the robot with respect to the
      :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving joints



   .. py:method:: move_gripper()

      Get the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper



