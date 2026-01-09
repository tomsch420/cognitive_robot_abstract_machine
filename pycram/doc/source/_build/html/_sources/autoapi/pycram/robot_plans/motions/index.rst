pycram.robot_plans.motions
==========================

.. py:module:: pycram.robot_plans.motions


Submodules
----------

.. toctree::
   :maxdepth: 1

   /autoapi/pycram/robot_plans/motions/base/index
   /autoapi/pycram/robot_plans/motions/container/index
   /autoapi/pycram/robot_plans/motions/gripper/index
   /autoapi/pycram/robot_plans/motions/misc/index
   /autoapi/pycram/robot_plans/motions/navigation/index
   /autoapi/pycram/robot_plans/motions/robot_body/index


Classes
-------

.. autoapisummary::

   pycram.robot_plans.motions.BaseMotion
   pycram.robot_plans.motions.Arms
   pycram.robot_plans.motions.ProcessModuleManager
   pycram.robot_plans.motions.ViewManager
   pycram.robot_plans.motions.OpeningMotion
   pycram.robot_plans.motions.ClosingMotion
   pycram.robot_plans.motions.BaseMotion
   pycram.robot_plans.motions.Arms
   pycram.robot_plans.motions.GripperState
   pycram.robot_plans.motions.MovementType
   pycram.robot_plans.motions.WaypointsMovementType
   pycram.robot_plans.motions.GraspDescription
   pycram.robot_plans.motions.PoseStamped
   pycram.robot_plans.motions.JointStateManager
   pycram.robot_plans.motions.ViewManager
   pycram.robot_plans.motions.ReachMotion
   pycram.robot_plans.motions.MoveGripperMotion
   pycram.robot_plans.motions.MoveTCPMotion
   pycram.robot_plans.motions.MoveTCPWaypointsMotion
   pycram.robot_plans.motions.PerceptionQuery
   pycram.robot_plans.motions.BaseMotion
   pycram.robot_plans.motions.ProcessModuleManager
   pycram.robot_plans.motions.DetectingMotion
   pycram.robot_plans.motions.BaseMotion
   pycram.robot_plans.motions.PoseStamped
   pycram.robot_plans.motions.ProcessModuleManager
   pycram.robot_plans.motions.MoveMotion
   pycram.robot_plans.motions.BaseMotion
   pycram.robot_plans.motions.Vector3Stamped
   pycram.robot_plans.motions.PoseStamped
   pycram.robot_plans.motions.ProcessModuleManager
   pycram.robot_plans.motions.ViewManager
   pycram.robot_plans.motions.MoveJointsMotion
   pycram.robot_plans.motions.LookingMotion


Functions
---------

.. autoapisummary::

   pycram.robot_plans.motions.translate_pose_along_local_axis


Package Contents
----------------

.. py:class:: BaseMotion

   Bases: :py:obj:`pycram.designator.DesignatorDescription`


   .. py:method:: perform()
      :abstractmethod:


      Passes this designator to the process module for execution. Will be overwritten by each motion.



   .. py:property:: motion_chart
      :type: Task



   .. py:property:: _motion_chart
      :type: Task

      :abstractmethod:



   .. py:method:: __post_init__()

      Checks if types are missing or wrong



.. py:class:: Arms

   Bases: :py:obj:`enum.IntEnum`


   Enum for Arms.


   .. py:attribute:: LEFT
      :value: 0



   .. py:attribute:: RIGHT
      :value: 1



   .. py:attribute:: BOTH
      :value: 2



   .. py:method:: __str__()

      Return str(self).



   .. py:method:: __repr__()

      Return repr(self).



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



.. py:class:: ViewManager

   .. py:method:: get_end_effector_view(arm: pycram.datastructures.enums.Arms, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.Manipulator]
      :staticmethod:



   .. py:method:: get_arm_view(arm: pycram.datastructures.enums.Arms, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.KinematicChain]
      :staticmethod:


      Get the arm view for a given arm and robot view.

      :param arm: The arm to get the view for.
      :param robot_view: The robot view to search in.
      :return: The Manipulator object representing the arm.



   .. py:method:: get_neck_view(robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.Neck]
      :staticmethod:


      Get the neck view for a given robot view.

      :param robot_view: The robot view to search in.
      :return: The Neck object representing the neck.



.. py:class:: OpeningMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Designator for opening container


   .. py:attribute:: object_part
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator for the drawer handle



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: ClosingMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Designator for closing a container


   .. py:attribute:: object_part
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator for the drawer handle



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: BaseMotion

   Bases: :py:obj:`pycram.designator.DesignatorDescription`


   .. py:method:: perform()
      :abstractmethod:


      Passes this designator to the process module for execution. Will be overwritten by each motion.



   .. py:property:: motion_chart
      :type: Task



   .. py:property:: _motion_chart
      :type: Task

      :abstractmethod:



   .. py:method:: __post_init__()

      Checks if types are missing or wrong



.. py:class:: Arms

   Bases: :py:obj:`enum.IntEnum`


   Enum for Arms.


   .. py:attribute:: LEFT
      :value: 0



   .. py:attribute:: RIGHT
      :value: 1



   .. py:attribute:: BOTH
      :value: 2



   .. py:method:: __str__()

      Return str(self).



   .. py:method:: __repr__()

      Return repr(self).



.. py:class:: GripperState(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different motions of the gripper.


   .. py:attribute:: OPEN


   .. py:attribute:: CLOSE


   .. py:attribute:: MEDIUM


   .. py:method:: __str__()


   .. py:method:: __repr__()


.. py:class:: MovementType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different movement types of the robot.


   .. py:attribute:: STRAIGHT_TRANSLATION


   .. py:attribute:: STRAIGHT_CARTESIAN


   .. py:attribute:: TRANSLATION


   .. py:attribute:: CARTESIAN


.. py:class:: WaypointsMovementType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different movement types of the robot.


   .. py:attribute:: ENFORCE_ORIENTATION_STRICT


   .. py:attribute:: ENFORCE_ORIENTATION_FINAL_POINT


.. py:class:: GraspDescription

   Bases: :py:obj:`pycram.has_parameters.HasParameters`


   Represents a grasp description with a side grasp, top face, and orientation alignment.


   .. py:attribute:: approach_direction
      :type:  pycram.datastructures.enums.ApproachDirection

      The primary approach direction. 



   .. py:attribute:: vertical_alignment
      :type:  pycram.datastructures.enums.VerticalAlignment

      The vertical alignment when grasping the pose



   .. py:attribute:: rotate_gripper
      :type:  bool
      :value: False


      Indicates if the gripper should be rotated by 90°. Must be a boolean.



   .. py:method:: __hash__()


   .. py:method:: as_list() -> typing_extensions.List[typing_extensions.Union[pycram.datastructures.enums.Grasp, typing_extensions.Optional[pycram.datastructures.enums.Grasp], bool]]

      :return: A list representation of the grasp description.



   .. py:method:: get_grasp_pose(end_effector: semantic_digital_twin.robots.abstract_robot.Manipulator, body: semantic_digital_twin.world_description.world_entity.Body, translate_rim_offset: bool = False) -> pycram.datastructures.pose.PoseStamped

      Translates the grasp pose of the object using the desired grasp description and object knowledge.
      Leaves the orientation untouched.
      Returns the translated grasp pose.

      :param end_effector: The end effector that will be used to grasp the object.
      :param body: The body of the object to be grasped.
      :param translate_rim_offset: If True, the grasp pose will be translated along the rim offset.

      :return: The grasp pose of the object.



   .. py:method:: calculate_grasp_orientation(front_orientation: numpy.ndarray) -> typing_extensions.List[float]

      Calculates the grasp orientation based on the approach axis and the grasp description.

      :param front_orientation: The front-facing orientation of the end effector as a numpy array.

      :return: The calculated orientation as a quaternion.



   .. py:method:: calculate_grasp_descriptions(robot: semantic_digital_twin.robots.abstract_robot.AbstractRobot, pose: pycram.datastructures.pose.PoseStamped, grasp_alignment: typing_extensions.Optional[PreferredGraspAlignment] = None) -> typing_extensions.List[GraspDescription]
      :staticmethod:


      This method determines the possible grasp configurations (approach axis and vertical alignment) of the body,
      taking into account the bodies orientation, position, and whether the gripper should be rotated by 90°.

      :param robot: The robot for which the grasp configurations are being calculated.
      :param grasp_alignment: An optional PreferredGraspAlignment object that specifies preferred grasp axis,
      :param pose: The pose of the object to be grasped.

      :return: A sorted list of GraspDescription instances representing all grasp permutations.



   .. py:method:: calculate_closest_faces(pose_to_robot_vector: pycram.datastructures.pose.PyCramVector3, specified_grasp_axis: pycram.datastructures.enums.AxisIdentifier = AxisIdentifier.Undefined) -> typing_extensions.Union[Tuple[pycram.datastructures.enums.ApproachDirection, pycram.datastructures.enums.ApproachDirection], Tuple[pycram.datastructures.enums.VerticalAlignment, pycram.datastructures.enums.VerticalAlignment]]
      :staticmethod:


      Determines the faces of the object based on the input vector.

      If `specified_grasp_axis` is None, it calculates the primary and secondary faces based on the vector's magnitude
      determining which sides of the object are most aligned with the robot. This will either be the x, y plane for side faces
      or the z axis for top/bottom faces.
      If `specified_grasp_axis` is provided, it only considers the specified axis and calculates the faces aligned
      with that axis.

      :param pose_to_robot_vector: A 3D vector representing one of the robot's axes in the pose's frame, with
                            irrelevant components set to np.nan.
      :param specified_grasp_axis: Specifies a specific axis (e.g., X, Y, Z) to focus on.

      :return: A tuple of two Grasp enums representing the primary and secondary faces.



.. py:class:: PoseStamped

   Bases: :py:obj:`pycram.has_parameters.HasParameters`


   A pose in 3D space with a timestamp.


   .. py:attribute:: pose
      :type:  PyCramPose


   .. py:attribute:: header
      :type:  Header


   .. py:property:: position


   .. py:property:: orientation


   .. py:property:: frame_id


   .. py:method:: __repr__()


   .. py:method:: ros_message()

      Convert the pose to a ROS message of type PoseStamped.

      :return: The ROS message.



   .. py:method:: from_ros_message(message: ROSPoseStamped) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a ROS message.

      :param message: The PoseStamped ROS message.
      :return: A new PoseStamped object created from the ROS message.



   .. py:method:: from_list(position: typing_extensions.Optional[typing_extensions.List[float]] = None, orientation: typing_extensions.Optional[typing_extensions.List[float]] = None, frame: typing_extensions.Optional[semantic_digital_twin.world_description.world_entity.Body] = None) -> typing_extensions.Self
      :classmethod:


      Factory to create a PoseStamped from a list of position and orientation.

      :param position: Position as a list of [x, y, z].
      :param orientation: Orientation as a list of [x, y, z, w].
      :param frame: Frame in which the pose is defined.
      :return: A new PoseStamped object.



   .. py:method:: from_matrix(matrix: numpy.ndarray, frame: semantic_digital_twin.world_description.world_entity.Body) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a 4x4 transformation matrix and a frame.

      :param matrix: A 4x4 transformation matrix as numpy array.
      :param frame: The frame in which the pose is defined.
      :return: A PoseStamped object created from the matrix and frame.



   .. py:method:: from_spatial_type(spatial_type: semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a SpatialTransformationMatrix and a frame.

      :param spatial_type: A SpatialTransformationMatrix object.
      :return: A PoseStamped object created from the spatial type and frame.



   .. py:method:: to_transform_stamped(child_link_id: semantic_digital_twin.world_description.world_entity.Body) -> TransformStamped

      Converts the PoseStamped to a TransformStamped given a frame to which the transform is pointing.

      :param child_link_id: Frame to which the transform is pointing.
      :return: A TransformStamped object.



   .. py:method:: to_spatial_type() -> semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix

      Converts the PoseStamped to a SpatialTransformationMatrix.

      :return: A SpatialTransformationMatrix object representing the pose in 3D space.



   .. py:method:: round(decimals: int = 4)

      Rounds the components of the pose (position and orientation) to the specified number of decimal places.

      :param decimals: Number of decimal places to round to.



   .. py:method:: to_list()

      Convert the pose to a list of [position, orientation, frame_id].

      :return: A list of [pose, frame_id].



   .. py:method:: almost_equal(other: PoseStamped, position_tolerance: float = 1e-06, orientation_tolerance: float = 1e-05) -> bool

      Check if two PoseStamped objects are almost equal within given tolerances for position and orientation and if the
      frame_id is the same.

      :param other: The other PoseStamped object to compare to.
      :param position_tolerance: Tolerance for position comparison as number of decimal places.
      :param orientation_tolerance: Tolerance for orientation comparison as number of decimal places.
      :return: True if the PoseStamped objects are almost equal, False otherwise.



   .. py:method:: rotate_by_quaternion(quaternion: typing_extensions.List[float])

      Rotates the orientation of the pose by a given quaternion.

      :param quaternion: A list representing the quaternion [x, y, z, w].



   .. py:method:: is_facing_2d_axis(pose_b: PoseStamped, axis: typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier] = AxisIdentifier.X, threshold_deg=82) -> typing_extensions.Tuple[bool, float]

      Check if this pose is facing another pose along a specific axis (X or Y) within a given angular threshold.

      :param pose_b: The target pose to compare against.
      :param axis: The axis to check alignment with ('x' or 'y'). Defaults to 'x'.
      :param threshold_deg: The maximum angular difference in degrees to consider as 'facing'. Defaults to 82 degrees.
      :return: Tuple of (True/False if facing, signed angular difference in radians).



   .. py:method:: is_facing_x_or_y(pose_b: PoseStamped) -> bool

      Check if this pose is facing another pose along either the X or Y axis within a default angular threshold.

      :param pose_b: The target pose to compare against.
      :return: True if this pose is facing the target along either X or Y axis, False otherwise.



.. py:class:: JointStateManager

   Manages joint states for different robot arms and their configurations.


   .. py:attribute:: joint_states
      :type:  Dict[Type[semantic_digital_twin.robots.abstract_robot.AbstractRobot], List[JointState]]

      A list of joint states that can be applied to the robot.



   .. py:method:: get_arm_state(arm: pycram.datastructures.enums.Arms, state_type: pycram.datastructures.enums.StaticJointState, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> Union[ArmState, None]

      Retrieves the joint state for a specific arm and state type.

      :param arm: The arm for which the state is requested.
      :param state_type: The type of state (e.g., Park).
      :param robot_view: The robot view to which the arm belongs.
      :return: The corresponding ArmState or None if not found.



   .. py:method:: get_gripper_state(gripper: pycram.datastructures.enums.Arms, state_type: pycram.datastructures.enums.StaticJointState, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> Union[GripperState, None]

      Retrieves the joint state for a specific gripper and state type.

      :param gripper: The gripper for which the state is requested.
      :param state_type: The type of state (e.g., Open, Close).
      :param robot_view: The robot view to which the gripper belongs.
      :return: The corresponding GripperState or None if not found.



   .. py:method:: get_joint_state(state: enum.Enum, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> List[JointState]

      Retrieves all joint states of a specific type for a given robot.

      :param state: The type of joint state to retrieve (e.g., Park, Open).
      :param robot_view: The robot class for which the joint states are requested.
      :return: A list of JointState objects matching the specified type.



   .. py:method:: add_joint_states(robot: Type[semantic_digital_twin.robots.abstract_robot.AbstractRobot], joint_states: List[JointState])

      Adds joint states for a specific robot type.

      :param robot: The robot class for which the joint states are added.
      :param joint_states: A list of joint states to be added.



.. py:class:: ViewManager

   .. py:method:: get_end_effector_view(arm: pycram.datastructures.enums.Arms, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.Manipulator]
      :staticmethod:



   .. py:method:: get_arm_view(arm: pycram.datastructures.enums.Arms, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.KinematicChain]
      :staticmethod:


      Get the arm view for a given arm and robot view.

      :param arm: The arm to get the view for.
      :param robot_view: The robot view to search in.
      :return: The Manipulator object representing the arm.



   .. py:method:: get_neck_view(robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.Neck]
      :staticmethod:


      Get the neck view for a given robot view.

      :param robot_view: The robot view to search in.
      :return: The Neck object representing the neck.



.. py:function:: translate_pose_along_local_axis(pose: pycram.datastructures.pose.PoseStamped, axis: Union[typing_extensions.List, numpy.ndarray], distance: float) -> pycram.datastructures.pose.PoseStamped

   Translate a pose along a given 3d vector (axis) by a given distance. The axis is given in the local coordinate
   frame of the pose. The axis is normalized and then scaled by the distance.

   :param pose: The pose that should be translated
   :param axis: The local axis along which the translation should be performed
   :param distance: The distance by which the pose should be translated

   :return: The translated pose


.. py:class:: ReachMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be picked up



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that should be used for pick up



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      The grasp description that should be used for picking up the object



   .. py:attribute:: movement_type
      :type:  pycram.datastructures.enums.MovementType

      The type of movement that should be performed.



   .. py:attribute:: reverse_pose_sequence
      :type:  bool
      :value: False


      Reverses the sequence of poses, i.e., moves away from the object instead of towards it. Used for placing objects.



   .. py:method:: _calculate_pose_sequence() -> List[pycram.datastructures.pose.PoseStamped]


   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: MoveGripperMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Opens or closes the gripper


   .. py:attribute:: motion
      :type:  pycram.datastructures.enums.GripperState

      Motion that should be performed, either 'open' or 'close'



   .. py:attribute:: gripper
      :type:  pycram.datastructures.enums.Arms

      Name of the gripper that should be moved



   .. py:attribute:: allow_gripper_collision
      :type:  Optional[bool]
      :value: None


      If the gripper is allowed to collide with something



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: MoveTCPMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Moves the Tool center point (TCP) of the robot


   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      Target pose to which the TCP should be moved



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm with the TCP that should be moved to the target



   .. py:attribute:: allow_gripper_collision
      :type:  Optional[bool]
      :value: None


      If the gripper can collide with something



   .. py:attribute:: movement_type
      :type:  Optional[pycram.datastructures.enums.MovementType]

      The type of movement that should be performed.



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: MoveTCPWaypointsMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Moves the Tool center point (TCP) of the robot


   .. py:attribute:: waypoints
      :type:  List[pycram.datastructures.pose.PoseStamped]

      Waypoints the TCP should move along 



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm with the TCP that should be moved to the target



   .. py:attribute:: allow_gripper_collision
      :type:  Optional[bool]
      :value: None


      If the gripper can collide with something



   .. py:attribute:: movement_type
      :type:  pycram.datastructures.enums.WaypointsMovementType

      The type of movement that should be performed.



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: PerceptionQuery

   .. py:attribute:: semantic_annotation
      :type:  typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]

      The semantic annotation for which to perceive



   .. py:attribute:: region
      :type:  semantic_digital_twin.world_description.geometry.BoundingBox

      The region in which the object should be detected



   .. py:attribute:: robot
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot

      '
      Robot annotation of the robot that should perceive the object.



   .. py:attribute:: world
      :type:  semantic_digital_twin.world.World

      The world in which the object should be detected.



   .. py:method:: from_world() -> typing_extensions.List[semantic_digital_twin.world_description.world_entity.Body]


   .. py:method:: from_robokudo()


.. py:class:: BaseMotion

   Bases: :py:obj:`pycram.designator.DesignatorDescription`


   .. py:method:: perform()
      :abstractmethod:


      Passes this designator to the process module for execution. Will be overwritten by each motion.



   .. py:property:: motion_chart
      :type: Task



   .. py:property:: _motion_chart
      :type: Task

      :abstractmethod:



   .. py:method:: __post_init__()

      Checks if types are missing or wrong



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



.. py:class:: DetectingMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Tries to detect an object in the FOV of the robot

   returns: ObjectDesignatorDescription.Object or Error: PerceptionObjectNotFound


   .. py:attribute:: query
      :type:  pycram.perception.PerceptionQuery

      Query for the perception system that should be answered



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: BaseMotion

   Bases: :py:obj:`pycram.designator.DesignatorDescription`


   .. py:method:: perform()
      :abstractmethod:


      Passes this designator to the process module for execution. Will be overwritten by each motion.



   .. py:property:: motion_chart
      :type: Task



   .. py:property:: _motion_chart
      :type: Task

      :abstractmethod:



   .. py:method:: __post_init__()

      Checks if types are missing or wrong



.. py:class:: PoseStamped

   Bases: :py:obj:`pycram.has_parameters.HasParameters`


   A pose in 3D space with a timestamp.


   .. py:attribute:: pose
      :type:  PyCramPose


   .. py:attribute:: header
      :type:  Header


   .. py:property:: position


   .. py:property:: orientation


   .. py:property:: frame_id


   .. py:method:: __repr__()


   .. py:method:: ros_message()

      Convert the pose to a ROS message of type PoseStamped.

      :return: The ROS message.



   .. py:method:: from_ros_message(message: ROSPoseStamped) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a ROS message.

      :param message: The PoseStamped ROS message.
      :return: A new PoseStamped object created from the ROS message.



   .. py:method:: from_list(position: typing_extensions.Optional[typing_extensions.List[float]] = None, orientation: typing_extensions.Optional[typing_extensions.List[float]] = None, frame: typing_extensions.Optional[semantic_digital_twin.world_description.world_entity.Body] = None) -> typing_extensions.Self
      :classmethod:


      Factory to create a PoseStamped from a list of position and orientation.

      :param position: Position as a list of [x, y, z].
      :param orientation: Orientation as a list of [x, y, z, w].
      :param frame: Frame in which the pose is defined.
      :return: A new PoseStamped object.



   .. py:method:: from_matrix(matrix: numpy.ndarray, frame: semantic_digital_twin.world_description.world_entity.Body) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a 4x4 transformation matrix and a frame.

      :param matrix: A 4x4 transformation matrix as numpy array.
      :param frame: The frame in which the pose is defined.
      :return: A PoseStamped object created from the matrix and frame.



   .. py:method:: from_spatial_type(spatial_type: semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a SpatialTransformationMatrix and a frame.

      :param spatial_type: A SpatialTransformationMatrix object.
      :return: A PoseStamped object created from the spatial type and frame.



   .. py:method:: to_transform_stamped(child_link_id: semantic_digital_twin.world_description.world_entity.Body) -> TransformStamped

      Converts the PoseStamped to a TransformStamped given a frame to which the transform is pointing.

      :param child_link_id: Frame to which the transform is pointing.
      :return: A TransformStamped object.



   .. py:method:: to_spatial_type() -> semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix

      Converts the PoseStamped to a SpatialTransformationMatrix.

      :return: A SpatialTransformationMatrix object representing the pose in 3D space.



   .. py:method:: round(decimals: int = 4)

      Rounds the components of the pose (position and orientation) to the specified number of decimal places.

      :param decimals: Number of decimal places to round to.



   .. py:method:: to_list()

      Convert the pose to a list of [position, orientation, frame_id].

      :return: A list of [pose, frame_id].



   .. py:method:: almost_equal(other: PoseStamped, position_tolerance: float = 1e-06, orientation_tolerance: float = 1e-05) -> bool

      Check if two PoseStamped objects are almost equal within given tolerances for position and orientation and if the
      frame_id is the same.

      :param other: The other PoseStamped object to compare to.
      :param position_tolerance: Tolerance for position comparison as number of decimal places.
      :param orientation_tolerance: Tolerance for orientation comparison as number of decimal places.
      :return: True if the PoseStamped objects are almost equal, False otherwise.



   .. py:method:: rotate_by_quaternion(quaternion: typing_extensions.List[float])

      Rotates the orientation of the pose by a given quaternion.

      :param quaternion: A list representing the quaternion [x, y, z, w].



   .. py:method:: is_facing_2d_axis(pose_b: PoseStamped, axis: typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier] = AxisIdentifier.X, threshold_deg=82) -> typing_extensions.Tuple[bool, float]

      Check if this pose is facing another pose along a specific axis (X or Y) within a given angular threshold.

      :param pose_b: The target pose to compare against.
      :param axis: The axis to check alignment with ('x' or 'y'). Defaults to 'x'.
      :param threshold_deg: The maximum angular difference in degrees to consider as 'facing'. Defaults to 82 degrees.
      :return: Tuple of (True/False if facing, signed angular difference in radians).



   .. py:method:: is_facing_x_or_y(pose_b: PoseStamped) -> bool

      Check if this pose is facing another pose along either the X or Y axis within a default angular threshold.

      :param pose_b: The target pose to compare against.
      :return: True if this pose is facing the target along either X or Y axis, False otherwise.



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



.. py:class:: MoveMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Moves the robot to a designated location


   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      Location to which the robot should be moved



   .. py:attribute:: keep_joint_states
      :type:  bool
      :value: False


      Keep the joint states of the robot during/at the end of the motion



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: BaseMotion

   Bases: :py:obj:`pycram.designator.DesignatorDescription`


   .. py:method:: perform()
      :abstractmethod:


      Passes this designator to the process module for execution. Will be overwritten by each motion.



   .. py:property:: motion_chart
      :type: Task



   .. py:property:: _motion_chart
      :type: Task

      :abstractmethod:



   .. py:method:: __post_init__()

      Checks if types are missing or wrong



.. py:class:: Vector3Stamped

   Bases: :py:obj:`PyCramVector3`


   A Vector3 with an attached ROS Header (timestamp and frame).
   Inherits all vector operations and adds frame/time metadata.


   .. py:attribute:: header
      :type:  Header


   .. py:property:: frame_id


   .. py:method:: __repr__()


   .. py:method:: ros_message()

      Convert to a ROS Vector3Stamped message.

      :return: The ROS message.



   .. py:method:: from_ros_message(message)
      :classmethod:


      Create a Vector3Stamped from a ROS message.

      :param message: The Vector3Stamped ROS message.
      :return: A new Vector3Stamped object.



   .. py:method:: to_spatial_type() -> semantic_digital_twin.spatial_types.spatial_types.Vector3


.. py:class:: PoseStamped

   Bases: :py:obj:`pycram.has_parameters.HasParameters`


   A pose in 3D space with a timestamp.


   .. py:attribute:: pose
      :type:  PyCramPose


   .. py:attribute:: header
      :type:  Header


   .. py:property:: position


   .. py:property:: orientation


   .. py:property:: frame_id


   .. py:method:: __repr__()


   .. py:method:: ros_message()

      Convert the pose to a ROS message of type PoseStamped.

      :return: The ROS message.



   .. py:method:: from_ros_message(message: ROSPoseStamped) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a ROS message.

      :param message: The PoseStamped ROS message.
      :return: A new PoseStamped object created from the ROS message.



   .. py:method:: from_list(position: typing_extensions.Optional[typing_extensions.List[float]] = None, orientation: typing_extensions.Optional[typing_extensions.List[float]] = None, frame: typing_extensions.Optional[semantic_digital_twin.world_description.world_entity.Body] = None) -> typing_extensions.Self
      :classmethod:


      Factory to create a PoseStamped from a list of position and orientation.

      :param position: Position as a list of [x, y, z].
      :param orientation: Orientation as a list of [x, y, z, w].
      :param frame: Frame in which the pose is defined.
      :return: A new PoseStamped object.



   .. py:method:: from_matrix(matrix: numpy.ndarray, frame: semantic_digital_twin.world_description.world_entity.Body) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a 4x4 transformation matrix and a frame.

      :param matrix: A 4x4 transformation matrix as numpy array.
      :param frame: The frame in which the pose is defined.
      :return: A PoseStamped object created from the matrix and frame.



   .. py:method:: from_spatial_type(spatial_type: semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a SpatialTransformationMatrix and a frame.

      :param spatial_type: A SpatialTransformationMatrix object.
      :return: A PoseStamped object created from the spatial type and frame.



   .. py:method:: to_transform_stamped(child_link_id: semantic_digital_twin.world_description.world_entity.Body) -> TransformStamped

      Converts the PoseStamped to a TransformStamped given a frame to which the transform is pointing.

      :param child_link_id: Frame to which the transform is pointing.
      :return: A TransformStamped object.



   .. py:method:: to_spatial_type() -> semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix

      Converts the PoseStamped to a SpatialTransformationMatrix.

      :return: A SpatialTransformationMatrix object representing the pose in 3D space.



   .. py:method:: round(decimals: int = 4)

      Rounds the components of the pose (position and orientation) to the specified number of decimal places.

      :param decimals: Number of decimal places to round to.



   .. py:method:: to_list()

      Convert the pose to a list of [position, orientation, frame_id].

      :return: A list of [pose, frame_id].



   .. py:method:: almost_equal(other: PoseStamped, position_tolerance: float = 1e-06, orientation_tolerance: float = 1e-05) -> bool

      Check if two PoseStamped objects are almost equal within given tolerances for position and orientation and if the
      frame_id is the same.

      :param other: The other PoseStamped object to compare to.
      :param position_tolerance: Tolerance for position comparison as number of decimal places.
      :param orientation_tolerance: Tolerance for orientation comparison as number of decimal places.
      :return: True if the PoseStamped objects are almost equal, False otherwise.



   .. py:method:: rotate_by_quaternion(quaternion: typing_extensions.List[float])

      Rotates the orientation of the pose by a given quaternion.

      :param quaternion: A list representing the quaternion [x, y, z, w].



   .. py:method:: is_facing_2d_axis(pose_b: PoseStamped, axis: typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier] = AxisIdentifier.X, threshold_deg=82) -> typing_extensions.Tuple[bool, float]

      Check if this pose is facing another pose along a specific axis (X or Y) within a given angular threshold.

      :param pose_b: The target pose to compare against.
      :param axis: The axis to check alignment with ('x' or 'y'). Defaults to 'x'.
      :param threshold_deg: The maximum angular difference in degrees to consider as 'facing'. Defaults to 82 degrees.
      :return: Tuple of (True/False if facing, signed angular difference in radians).



   .. py:method:: is_facing_x_or_y(pose_b: PoseStamped) -> bool

      Check if this pose is facing another pose along either the X or Y axis within a default angular threshold.

      :param pose_b: The target pose to compare against.
      :return: True if this pose is facing the target along either X or Y axis, False otherwise.



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



.. py:class:: ViewManager

   .. py:method:: get_end_effector_view(arm: pycram.datastructures.enums.Arms, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.Manipulator]
      :staticmethod:



   .. py:method:: get_arm_view(arm: pycram.datastructures.enums.Arms, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.KinematicChain]
      :staticmethod:


      Get the arm view for a given arm and robot view.

      :param arm: The arm to get the view for.
      :param robot_view: The robot view to search in.
      :return: The Manipulator object representing the arm.



   .. py:method:: get_neck_view(robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.Neck]
      :staticmethod:


      Get the neck view for a given robot view.

      :param robot_view: The robot view to search in.
      :return: The Neck object representing the neck.



.. py:class:: MoveJointsMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Moves any joint on the robot


   .. py:attribute:: names
      :type:  list

      List of joint names that should be moved 



   .. py:attribute:: positions
      :type:  list

      Target positions of joints, should correspond to the list of names



   .. py:attribute:: align
      :type:  Optional[bool]
      :value: False


      If True, aligns the end-effector with a specified axis (optional).



   .. py:attribute:: tip_link
      :type:  Optional[str]
      :value: None


      Name of the tip link to align with, e.g the object (optional).



   .. py:attribute:: tip_normal
      :type:  Optional[pycram.datastructures.pose.Vector3Stamped]
      :value: None


      Normalized vector representing the current orientation axis of the end-effector (optional).



   .. py:attribute:: root_link
      :type:  Optional[str]
      :value: None


      Base link of the robot; typically set to the torso (optional).



   .. py:attribute:: root_normal
      :type:  Optional[pycram.datastructures.pose.Vector3Stamped]
      :value: None


      Normalized vector representing the desired orientation axis to align with (optional).



   .. py:method:: perform()


   .. py:property:: _motion_chart


.. py:class:: LookingMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Lets the robot look at a point


   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      Target pose to look at



   .. py:method:: perform()


   .. py:property:: _motion_chart


