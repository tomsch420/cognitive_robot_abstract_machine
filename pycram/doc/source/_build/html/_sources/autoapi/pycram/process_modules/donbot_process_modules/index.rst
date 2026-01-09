pycram.process_modules.donbot_process_modules
=============================================

.. py:module:: pycram.process_modules.donbot_process_modules


Classes
-------

.. autoapisummary::

   pycram.process_modules.donbot_process_modules.DonbotMoveHead
   pycram.process_modules.donbot_process_modules.DonbotManager


Module Contents
---------------

.. py:class:: DonbotMoveHead(lock)

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultMoveHead`


   Moves the head of the iai_donbot robot to look at a specified point in the world coordinate frame.
   This point can be a position or an object, and the orientation is calculated based on the
   robot's base and camera alignment.


   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DonbotManager

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultManager`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


   .. py:attribute:: robot_name
      :value: 'iai_donbot'



   .. py:method:: looking()

      Get the Process Module for looking at a point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`
      :return:



