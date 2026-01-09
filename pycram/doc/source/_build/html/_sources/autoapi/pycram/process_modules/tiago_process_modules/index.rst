pycram.process_modules.tiago_process_modules
============================================

.. py:module:: pycram.process_modules.tiago_process_modules


Attributes
----------

.. autoapisummary::

   pycram.process_modules.tiago_process_modules.logger


Classes
-------

.. autoapisummary::

   pycram.process_modules.tiago_process_modules.TiagoNavigationReal
   pycram.process_modules.tiago_process_modules.TiagoMoveHeadReal
   pycram.process_modules.tiago_process_modules.TiagoDetectingReal
   pycram.process_modules.tiago_process_modules.TiagoMoveTCPReal
   pycram.process_modules.tiago_process_modules.TiagoManager


Module Contents
---------------

.. py:data:: logger

.. py:class:: TiagoNavigationReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Implementation of process modules. Process modules are the part that communicate with the outer world to execute
    designators.


   .. py:method:: _execute(designator: pycram.robot_plans.MoveMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: TiagoMoveHeadReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Implementation of process modules. Process modules are the part that communicate with the outer world to execute
    designators.


   .. py:method:: _execute(designator: pycram.robot_plans.MoveMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: TiagoDetectingReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Implementation of process modules. Process modules are the part that communicate with the outer world to execute
    designators.


   .. py:method:: _execute(designator: pycram.robot_plans.DetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: TiagoMoveTCPReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Implementation of process modules. Process modules are the part that communicate with the outer world to execute
    designators.


   .. py:method:: _execute(designator: pycram.robot_plans.MoveTCPMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: TiagoManager

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultManager`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


   .. py:attribute:: robot_name
      :value: 'tiago_dual'



   .. py:method:: navigate()

      Get the Process Module for navigating the robot with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for navigating



   .. py:method:: looking()

      Get the Process Module for looking at a point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`
      :return:



   .. py:method:: move_tcp()

      Get the Process Module for moving the Tool Center Point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the TCP



   .. py:method:: move_gripper()

      Get the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper



