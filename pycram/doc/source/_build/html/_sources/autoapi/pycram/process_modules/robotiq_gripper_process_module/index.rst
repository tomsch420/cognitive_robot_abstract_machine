pycram.process_modules.robotiq_gripper_process_module
=====================================================

.. py:module:: pycram.process_modules.robotiq_gripper_process_module


Attributes
----------

.. autoapisummary::

   pycram.process_modules.robotiq_gripper_process_module.logger


Classes
-------

.. autoapisummary::

   pycram.process_modules.robotiq_gripper_process_module.RobotiqMoveGripperReal
   pycram.process_modules.robotiq_gripper_process_module.RobotiqManager


Module Contents
---------------

.. py:data:: logger

.. py:class:: RobotiqMoveGripperReal(lock)

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultMoveGripperReal`


   Opens or closes the gripper of the real Robotiq gripper, uses a topic for this instead of giskard


   .. py:method:: _execute(designator: pycram.robot_plans.MoveGripperMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: RobotiqManager

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultManager`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


   .. py:attribute:: robot_name


   .. py:method:: move_gripper()

      Get the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper



