pycram.process_modules.hsrb_process_modules
===========================================

.. py:module:: pycram.process_modules.hsrb_process_modules


Attributes
----------

.. autoapisummary::

   pycram.process_modules.hsrb_process_modules.logger


Classes
-------

.. autoapisummary::

   pycram.process_modules.hsrb_process_modules.HSRBNavigationReal
   pycram.process_modules.hsrb_process_modules.HSRBMoveHeadReal
   pycram.process_modules.hsrb_process_modules.HSRBMoveTCPReal
   pycram.process_modules.hsrb_process_modules.HSRBMoveArmJointsReal
   pycram.process_modules.hsrb_process_modules.HSRBMoveJointsReal
   pycram.process_modules.hsrb_process_modules.HSRBOpenReal
   pycram.process_modules.hsrb_process_modules.HSRBCloseReal
   pycram.process_modules.hsrb_process_modules.HSRBNavigationSemiReal
   pycram.process_modules.hsrb_process_modules.HSRBManager


Module Contents
---------------

.. py:data:: logger

.. py:class:: HSRBNavigationReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base


   .. py:method:: _execute(designator: MoveMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveHeadReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Process module for the real HSRB that sends a pose goal to giskard to move the robot head


   .. py:method:: _execute(desig: LookingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveTCPReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Moves the tool center point of the real HSRB while avoiding all collisions via giskard


   .. py:method:: _execute(designator: MoveTCPMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveArmJointsReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Moves the arm joints of the real HSRB to the given configuration while avoiding all collisions via giskard


   .. py:method:: _execute(designator: MoveArmJointsMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBMoveJointsReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Moves any joint using giskard, avoids all collisions while doint this.


   .. py:method:: _execute(designator: MoveJointsMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBOpenReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process Modules tries to open an already grasped container via giskard


   .. py:method:: _execute(designator: OpeningMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBCloseReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process module executes close a an already grasped container via giskard


   .. py:method:: _execute(designator: ClosingMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBNavigationSemiReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base


   .. py:method:: _execute(designator: MoveMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: HSRBManager

   Bases: :py:obj:`pycram.process_modules.default_process_modules.DefaultManager`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


   .. py:attribute:: robot_name
      :value: 'hsrb'



   .. py:attribute:: _navigate_lock


   .. py:method:: navigate()

      Get the Process Module for navigating the robot with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for navigating



   .. py:method:: looking()

      Get the Process Module for looking at a point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`
      :return:



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



   .. py:method:: move_joints()

      Get the Process Module for moving any joint of the robot with respect to the
      :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving joints



   .. py:method:: open()

      Get the Process Module for opening drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for opening drawers



   .. py:method:: close()

      Get the Process Module for closing drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for closing drawers



