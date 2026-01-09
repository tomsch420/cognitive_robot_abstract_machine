pycram.process_module
=====================

.. py:module:: pycram.process_module

.. autoapi-nested-parse::

   Implementation of process modules.

   Classes:
   ProcessModule -- implementation of process modules.



Attributes
----------

.. autoapisummary::

   pycram.process_module.logger
   pycram.process_module.simulated_robot
   pycram.process_module.real_robot
   pycram.process_module.semi_real_robot


Classes
-------

.. autoapisummary::

   pycram.process_module.ProcessModule
   pycram.process_module.RealRobot
   pycram.process_module.SimulatedRobot
   pycram.process_module.SemiRealRobot
   pycram.process_module.ProcessModuleManager
   pycram.process_module.ManagerBase


Functions
---------

.. autoapisummary::

   pycram.process_module.with_real_robot
   pycram.process_module.with_simulated_robot


Module Contents
---------------

.. py:data:: logger

.. py:class:: ProcessModule(lock)

   Implementation of process modules. Process modules are the part that communicate with the outer world to execute
    designators.


   .. py:attribute:: execution_delay
      :type:  typing_extensions.Optional[datetime.timedelta]

      Adds a delay after executing a process module, to make the execution in simulation more realistic



   .. py:attribute:: _lock


   .. py:method:: _execute(designator: pycram.robot_plans.motions.BaseMotion) -> typing_extensions.Any

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



   .. py:method:: execute(designator: pycram.robot_plans.motions.BaseMotion) -> typing_extensions.Any

      Execute the given designator_description. If there is already another process module of the same kind the `self._lock` will
      lock this thread until the execution of that process module is finished. This implicitly queues the execution of
      process modules.

      :param designator: The designator_description to execute.
      :return: Return of the Process Module if there is any



.. py:class:: RealRobot

   Management class for executing designators on the real robot. This is intended to be used in a with environment.
   When importing this class an instance is imported instead.

   Example:

   .. code-block:: python

       with real_robot:
           some designators


   .. py:attribute:: pre
      :type:  pycram.datastructures.enums.ExecutionType


   .. py:attribute:: pre_delay
      :type:  datetime.timedelta


   .. py:method:: __enter__()

      Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
      sets it to 'real'



   .. py:method:: __exit__(_type, value, traceback)

      Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
      used one.



   .. py:method:: __call__()


.. py:class:: SimulatedRobot

   Management class for executing designators on the simulated robot. This is intended to be used in
   a with environment. When importing this class an instance is imported instead.

   Example:

   .. code-block:: python

       with simulated_robot:
           some designators


   .. py:attribute:: pre
      :type:  pycram.datastructures.enums.ExecutionType


   .. py:method:: __enter__()

      Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
      sets it to 'simulated'



   .. py:method:: __exit__(_type, value, traceback)

      Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
      used one.



   .. py:method:: __call__()


.. py:class:: SemiRealRobot

   Management class for executing designators on the semi-real robot. This is intended to be used in a with environment.
   When importing this class an instance is imported instead.

   Example:

   .. code-block:: python

       with semi_real_robot:
           some designators


   .. py:attribute:: pre
      :type:  pycram.datastructures.enums.ExecutionType


   .. py:method:: __enter__()

      Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
      sets it to 'semi_real'



   .. py:method:: __exit__(type, value, traceback)

      Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
      used one.



   .. py:method:: __call__()


.. py:function:: with_real_robot(func: typing_extensions.Callable) -> typing_extensions.Callable

   Decorator to execute designators in the decorated class on the real robot.

   Example:

   .. code-block:: python

       @with_real_robot
       def plan():
           some designators

   :param func: Function this decorator is annotating
   :return: The decorated function wrapped into the decorator


.. py:function:: with_simulated_robot(func: typing_extensions.Callable) -> typing_extensions.Callable

   Decorator to execute designators in the decorated class on the simulated robot.

   Example:

   .. code-block:: python

       @with_simulated_robot
       def plan():
           some designators

   :param func: Function this decorator is annotating
   :return: The decorated function wrapped into the decorator


.. py:data:: simulated_robot

.. py:data:: real_robot

.. py:data:: semi_real_robot

.. py:class:: ProcessModuleManager

   Bases: :py:obj:`abc.ABC`


   Base class for managing process modules, any new process modules have to implement this class to register the
   Process Modules


   .. py:attribute:: execution_type
      :type:  pycram.datastructures.enums.ExecutionType
      :value: None


      Whether the robot for which the process module is intended for is real or a simulated one



   .. py:attribute:: available_pms
      :type:  typing_extensions.List[ManagerBase]
      :value: []


      List of all available Process Module Managers



   .. py:attribute:: _instance
      :type:  ProcessModuleManager
      :value: None


      Singelton instance of this Process Module Manager



   .. py:attribute:: _initialized
      :type:  bool
      :value: False



   .. py:method:: register_manager(manager: ManagerBase)

      Register a new Process Module Manager for the given robot name.

      :param manager: The Process Module Manager to register



   .. py:method:: get_manager(robot: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> ManagerBase

      Returns the Process Module manager for the currently loaded robot or None if there is no Manager.

      :return: ProcessModuleManager instance of the current robot



   .. py:method:: register_all_process_modules()
      :staticmethod:



.. py:class:: ManagerBase(robot_name: str)

   Bases: :py:obj:`abc.ABC`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


   .. py:attribute:: robot_name


   .. py:attribute:: _navigate_lock


   .. py:attribute:: _pick_up_lock


   .. py:attribute:: _place_lock


   .. py:attribute:: _looking_lock


   .. py:attribute:: _detecting_lock


   .. py:attribute:: _move_tcp_lock


   .. py:attribute:: _move_arm_joints_lock


   .. py:attribute:: _world_state_detecting_lock


   .. py:attribute:: _move_joints_lock


   .. py:attribute:: _move_gripper_lock


   .. py:attribute:: _open_lock


   .. py:attribute:: _close_lock


   .. py:attribute:: _move_tcp_waypoints_lock


   .. py:method:: navigate() -> ProcessModule
      :abstractmethod:


      Get the Process Module for navigating the robot with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for navigating



   .. py:method:: looking() -> ProcessModule
      :abstractmethod:


      Get the Process Module for looking at a point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`
      :return:



   .. py:method:: detecting() -> ProcessModule
      :abstractmethod:


      Get the Process Module for detecting an object with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for detecting an object



   .. py:method:: move_tcp() -> ProcessModule
      :abstractmethod:


      Get the Process Module for moving the Tool Center Point with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the TCP



   .. py:method:: move_arm_joints() -> ProcessModule
      :abstractmethod:


      Get the Process Module for moving the joints of the robot arm
      with respect to the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the arm joints



   .. py:method:: move_joints() -> ProcessModule
      :abstractmethod:


      Get the Process Module for moving any joint of the robot with respect to the
      :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving joints



   .. py:method:: move_gripper() -> ProcessModule
      :abstractmethod:


      Get the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper



   .. py:method:: open() -> ProcessModule
      :abstractmethod:


      Get the Process Module for opening drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for opening drawers



   .. py:method:: close() -> ProcessModule
      :abstractmethod:


      Get the Process Module for closing drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for closing drawers



   .. py:method:: move_tcp_waypoints() -> ProcessModule
      :abstractmethod:


      Get the Process Module for moving the Tool Center Point along a list of waypoints with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the TCP along a list of waypoints



