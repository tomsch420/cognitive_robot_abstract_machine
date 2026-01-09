pycram.process_modules.stretch_process_modules
==============================================

.. py:module:: pycram.process_modules.stretch_process_modules


Attributes
----------

.. autoapisummary::

   pycram.process_modules.stretch_process_modules.logger


Classes
-------

.. autoapisummary::

   pycram.process_modules.stretch_process_modules.StretchMoveHead
   pycram.process_modules.stretch_process_modules.StretchNavigationReal
   pycram.process_modules.stretch_process_modules.StretchMoveHeadReal
   pycram.process_modules.stretch_process_modules.StretchDetectingReal
   pycram.process_modules.stretch_process_modules.StretchMoveGripperReal
   pycram.process_modules.stretch_process_modules.StretchManager


Module Contents
---------------

.. py:data:: logger

.. py:class:: StretchMoveHead(lock)

   Bases: :py:obj:`pycram.process_modules.default_process_modules.ProcessModule`


   Process module for the simulated Stretch that moves the head such that it looks at the given position


   .. py:method:: _execute(designator: MoveMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: StretchNavigationReal(lock)

   Bases: :py:obj:`pycram.process_modules.default_process_modules.ProcessModule`


   Process module for the real Stretch that sends a cartesian goal to giskard to move the robot base


   .. py:method:: _execute(designator: MoveMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: StretchMoveHeadReal(lock)

   Bases: :py:obj:`pycram.process_modules.default_process_modules.ProcessModule`


   Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
   as the simulated one


   .. py:method:: _execute(desig: LookingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: StretchDetectingReal(lock)

   Bases: :py:obj:`pycram.process_modules.default_process_modules.ProcessModule`


   Process Module for the real Stretch that tries to detect an object fitting the given object description. Uses Robokudo
   for perception of the environment.


   .. py:method:: _execute(designator: DetectingMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: StretchMoveGripperReal(lock)

   Bases: :py:obj:`pycram.process_modules.default_process_modules.ProcessModule`


   Opens or closes the gripper of the real Stretch, gripper uses an action server for this instead of giskard


   .. py:method:: _execute(designator: MoveGripperMotion) -> Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: StretchManager

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultManager`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


   .. py:attribute:: robot_name
      :value: 'stretch_description'



   .. py:method:: navigate()

      Get the Process Module for navigating the robot with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for navigating



   .. py:method:: looking()

      Get the Process Module for looking at a point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`
      :return:



   .. py:method:: move_gripper()

      Get the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper



