pycram.process_modules.default_process_modules
==============================================

.. py:module:: pycram.process_modules.default_process_modules


Attributes
----------

.. autoapisummary::

   pycram.process_modules.default_process_modules.logger


Classes
-------

.. autoapisummary::

   pycram.process_modules.default_process_modules.DefaultNavigation
   pycram.process_modules.default_process_modules.DefaultMoveHead
   pycram.process_modules.default_process_modules.DefaultMoveGripper
   pycram.process_modules.default_process_modules.DefaultDetecting
   pycram.process_modules.default_process_modules.DefaultMoveTCP
   pycram.process_modules.default_process_modules.DefaultMoveArmJoints
   pycram.process_modules.default_process_modules.DefaultMoveJoints
   pycram.process_modules.default_process_modules.DefaultOpen
   pycram.process_modules.default_process_modules.DefaultClose
   pycram.process_modules.default_process_modules.DefaultMoveTCPWaypoints
   pycram.process_modules.default_process_modules.DefaultDetectingReal
   pycram.process_modules.default_process_modules.DefaultNavigationReal
   pycram.process_modules.default_process_modules.DefaultMoveHeadReal
   pycram.process_modules.default_process_modules.DefaultMoveTCPReal
   pycram.process_modules.default_process_modules.DefaultMoveArmJointsReal
   pycram.process_modules.default_process_modules.DefaultMoveJointsReal
   pycram.process_modules.default_process_modules.DefaultMoveGripperReal
   pycram.process_modules.default_process_modules.DefaultOpenReal
   pycram.process_modules.default_process_modules.DefaultCloseReal
   pycram.process_modules.default_process_modules.DefaultMoveTCPWaypointsReal
   pycram.process_modules.default_process_modules.DefaultManager


Functions
---------

.. autoapisummary::

   pycram.process_modules.default_process_modules._move_arm_tcp


Module Contents
---------------

.. py:data:: logger

.. py:class:: DefaultNavigation(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   The process module to move the robot from one position to another.


   .. py:method:: _execute(desig: MoveMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveHead(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Moves the robot's head to look at a specified target point in the world coordinate frame.
   The target can be either a position or an object.


   .. py:method:: _execute(desig: LookingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveGripper(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process module controls the gripper of the robot. They can either be opened or closed.
   Furthermore, it can only move one gripper at a time.


   .. py:method:: _execute(desig: MoveGripperMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultDetecting(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process module tries to detect an object with the given type. To be detected the object has to be in
   the field of view of the robot.
   :return: A list of perceived objects.


   .. py:method:: _execute(designator: DetectingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveTCP(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process moves the tool center point of either the right or the left arm.


   .. py:method:: _execute(desig: MoveTCPMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveArmJoints(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process modules moves the joints of either the right or the left arm. The joint states can be given as
   list that should be applied or a pre-defined position can be used, such as "parking"


   .. py:method:: _execute(desig: MoveArmJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveJoints(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Implementation of process modules. Process modules are the part that communicate with the outer world to execute
    designators.


   .. py:method:: _execute(desig: MoveJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultOpen(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.


   .. py:method:: _execute(desig: OpeningMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultClose(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Low-level implementation that lets the robot close a grasped container, in simulation


   .. py:method:: _execute(desig: ClosingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveTCPWaypoints(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   This process moves the tool center point of either the right or the left arm along a list of waypoints.


   .. py:method:: _execute(desig: MoveTCPWaypointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:function:: _move_arm_tcp(target: PoseStamped, robot: semantic_digital_twin.robots.pr2.AbstractRobot, arm: Arms, world: semantic_digital_twin.world.World) -> None

   Calls the ik solver to calculate the inverse kinematics of the arm and then sets the joint states accordingly.

   :param target: Target pose to which the end-effector should move.
   :param robot: Robot object representing the robot.
   :param arm: Which arm to move


.. py:class:: DefaultDetectingReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Implementation of process modules. Process modules are the part that communicate with the outer world to execute
    designators.


   .. py:method:: _execute(designator: DetectingMotion) -> List[Body]

      Perform a query based on the detection technique and state defined in the designator.

      :return: A list of perceived objects.



.. py:class:: DefaultNavigationReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Process module for the real robot that sends a cartesian goal to giskard to move the robot base


   .. py:method:: _execute(designator: MoveMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveHeadReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Process module for controlling the real robot's head to look at a specified position.
   Uses the same calculations as the simulated version to orient the head.


   .. py:method:: _execute(desig: LookingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveTCPReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Moves the tool center point of the real robot while avoiding all collisions


   .. py:method:: _execute(designator: MoveTCPMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveArmJointsReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Moves the arm joints of the real robot to the given configuration while avoiding all collisions


   .. py:method:: _execute(designator: MoveArmJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveJointsReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Moves any joint using giskard, avoids all collisions while doint this.


   .. py:method:: _execute(designator: MoveJointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveGripperReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Opens or closes the gripper of the real robot, gripper uses an action server for this instead of giskard


   .. py:method:: _execute(designator: MoveGripperMotion)
      :abstractmethod:


      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultOpenReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Tries to open an already grasped container


   .. py:method:: _execute(designator: OpeningMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultCloseReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Tries to close an already grasped container


   .. py:method:: _execute(designator: ClosingMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultMoveTCPWaypointsReal(lock)

   Bases: :py:obj:`pycram.process_module.ProcessModule`


   Moves the tool center point of the real robot along a list of waypoints while avoiding all collisions


   .. py:method:: _execute(designator: MoveTCPWaypointsMotion)

      Helper method for internal usage only.
      This method is to be overwritten instead of the execute method.



.. py:class:: DefaultManager

   Bases: :py:obj:`pycram.process_module.ManagerBase`


   Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
   It is intended to be used as a base class for all Process Module Managers.


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



   .. py:method:: move_gripper()

      Get the Process Module for moving the gripper with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the gripper



   .. py:method:: open()

      Get the Process Module for opening drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for opening drawers



   .. py:method:: close()

      Get the Process Module for closing drawers with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for closing drawers



   .. py:method:: move_tcp_waypoints()

      Get the Process Module for moving the Tool Center Point along a list of waypoints with respect to
       the :py:attr:`~ProcessModuleManager.execution_type`

      :return: The Process Module for moving the TCP along a list of waypoints



