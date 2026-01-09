pycram.process_modules.boxy_process_modules
===========================================

.. py:module:: pycram.process_modules.boxy_process_modules


Classes
-------

.. autoapisummary::

   pycram.process_modules.boxy_process_modules.BoxyParkArms
   pycram.process_modules.boxy_process_modules.BoxyMoveHead
   pycram.process_modules.boxy_process_modules.BoxyDetecting
   pycram.process_modules.boxy_process_modules.BoxyManager


Functions
---------

.. autoapisummary::

   pycram.process_modules.boxy_process_modules._park_arms


Module Contents
---------------

.. py:function:: _park_arms(arm)

   Defines the joint poses for the parking positions of the arm of Donbot and applies them to the
   in the World defined robot.
   :return: None


.. py:class:: BoxyParkArms(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process module is for moving the arms in a parking position.
   It is currently not used.


   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyMoveHead(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process module moves the head to look at a specific point in the world coordinate frame.
   This point can either be a position or an object.


   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyDetecting(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process module tries to detect an object with the given type. To be detected the object has to be in
   the field of view of the robot.


   .. py:method:: _execute(desig)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: BoxyManager

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultManager`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


   .. py:attribute:: robot_name
      :value: 'boxy'



   .. py:method:: looking()

      Get the Process Module for looking at a point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`
      :return:



   .. py:method:: detecting()

      Get the Process Module for detecting an object with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for detecting an object



