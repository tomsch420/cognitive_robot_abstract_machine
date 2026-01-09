pycram.process_modules.pr2_process_modules
==========================================

.. py:module:: pycram.process_modules.pr2_process_modules


Attributes
----------

.. autoapisummary::

   pycram.process_modules.pr2_process_modules.logger
   pycram.process_modules.pr2_process_modules.Multiverse
   pycram.process_modules.pr2_process_modules.Multiverse


Classes
-------

.. autoapisummary::

   pycram.process_modules.pr2_process_modules.Pr2MoveGripperMultiverse
   pycram.process_modules.pr2_process_modules.Pr2MoveGripperReal
   pycram.process_modules.pr2_process_modules.Pr2Manager


Module Contents
---------------

.. py:data:: logger

.. py:data:: Multiverse

.. py:data:: Multiverse

.. py:class:: Pr2MoveGripperMultiverse(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Opens or closes the gripper of the real PR2, gripper uses an action server for this instead of giskard


   .. py:method:: _execute(designator: pycram.robot_plans.MoveGripperMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2MoveGripperReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Opens or closes the gripper of the real PR2, gripper uses an action server for this instead of giskard


   .. py:method:: _execute(designator: pycram.robot_plans.MoveGripperMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: Pr2Manager

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultManager`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


   .. py:attribute:: robot_name
      :value: 'pr2'



   .. py:method:: move_gripper()

      Get the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper



