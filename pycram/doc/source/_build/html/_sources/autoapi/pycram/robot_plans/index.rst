pycram.robot_plans
==================

.. py:module:: pycram.robot_plans


Submodules
----------

.. toctree::
   :maxdepth: 1

   /autoapi/pycram/robot_plans/actions/index
   /autoapi/pycram/robot_plans/motions/index


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.LookAtActionDescription
   pycram.robot_plans.NavigateActionDescription
   pycram.robot_plans.FaceAtActionDescription
   pycram.robot_plans.DetectActionDescription
   pycram.robot_plans.SearchActionDescription
   pycram.robot_plans.CuttingActionDescription
   pycram.robot_plans.PouringActionDescription
   pycram.robot_plans.MixingActionDescription
   pycram.robot_plans.ParkArmsActionDescription
   pycram.robot_plans.PickUpActionDescription
   pycram.robot_plans.PlaceActionDescription
   pycram.robot_plans.OpenActionDescription
   pycram.robot_plans.TransportActionDescription
   pycram.robot_plans.PickAndPlaceActionDescription
   pycram.robot_plans.MoveAndPlaceActionDescription
   pycram.robot_plans.MoveAndPickUpActionDescription
   pycram.robot_plans.EfficientTransportActionDescription
   pycram.robot_plans.GraspingActionDescription
   pycram.robot_plans.CloseActionDescription
   pycram.robot_plans.logger
   pycram.robot_plans.ReachActionDescription
   pycram.robot_plans.MoveTorsoActionDescription
   pycram.robot_plans.SetGripperActionDescription
   pycram.robot_plans.CarryActionDescription
   pycram.robot_plans.right_park
   pycram.robot_plans.left_park
   pycram.robot_plans.both_park
   pycram.robot_plans.left_gripper_open
   pycram.robot_plans.left_gripper_close
   pycram.robot_plans.right_gripper_open
   pycram.robot_plans.right_gripper_close
   pycram.robot_plans.torso_low
   pycram.robot_plans.torso_mid
   pycram.robot_plans.torso_high
   pycram.robot_plans.left_arm
   pycram.robot_plans.both_arm


Exceptions
----------

.. autoapisummary::

   pycram.robot_plans.PerceptionObjectNotFound
   pycram.robot_plans.ObjectUnfetchable
   pycram.robot_plans.ConfigurationNotReached
   pycram.robot_plans.ContainerManipulationError
   pycram.robot_plans.ObjectNotGraspedError
   pycram.robot_plans.ObjectNotInGraspingArea
   pycram.robot_plans.LookAtGoalNotReached
   pycram.robot_plans.NavigationGoalNotReachedError
   pycram.robot_plans.ObjectNotPlacedAtTargetLocation
   pycram.robot_plans.ObjectStillInContact
   pycram.robot_plans.TorsoGoalNotReached


Classes
-------

.. autoapisummary::

   pycram.robot_plans.ActionConfig
   pycram.robot_plans.PartialDesignator
   pycram.robot_plans.PoseStamped
   pycram.robot_plans.SequentialPlan
   pycram.robot_plans.ActionDescription
   pycram.robot_plans.FaceAtAction
   pycram.robot_plans.DetectionTechnique
   pycram.robot_plans.CostmapLocation
   pycram.robot_plans.TryInOrderPlan
   pycram.robot_plans.SearchAction
   pycram.robot_plans.MoveTCPMotion
   pycram.robot_plans.Arms
   pycram.robot_plans.AxisIdentifier
   pycram.robot_plans.Grasp
   pycram.robot_plans.ApproachDirection
   pycram.robot_plans.VerticalAlignment
   pycram.robot_plans.RobotDescription
   pycram.robot_plans.MixingAction
   pycram.robot_plans.PouringAction
   pycram.robot_plans.CuttingAction
   pycram.robot_plans.GraspDescription
   pycram.robot_plans.ProbabilisticCostmapLocation
   pycram.robot_plans.GiskardLocation
   pycram.robot_plans.BelieveObject
   pycram.robot_plans.TransportAction
   pycram.robot_plans.PickAndPlaceAction
   pycram.robot_plans.MoveAndPlaceAction
   pycram.robot_plans.MoveAndPickUpAction
   pycram.robot_plans.EfficientTransportAction
   pycram.robot_plans.OpeningMotion
   pycram.robot_plans.ClosingMotion
   pycram.robot_plans.MoveGripperMotion
   pycram.robot_plans.GripperState
   pycram.robot_plans.ContainerManipulationType
   pycram.robot_plans.OpenAction
   pycram.robot_plans.CloseAction
   pycram.robot_plans.MovementType
   pycram.robot_plans.FindBodyInRegionMethod
   pycram.robot_plans.ViewManager
   pycram.robot_plans.ReachAction
   pycram.robot_plans.PickUpAction
   pycram.robot_plans.GraspingAction
   pycram.robot_plans.PerceptionQuery
   pycram.robot_plans.DetectionState
   pycram.robot_plans.DetectAction
   pycram.robot_plans.LookingMotion
   pycram.robot_plans.MoveMotion
   pycram.robot_plans.PoseErrorChecker
   pycram.robot_plans.NavigateAction
   pycram.robot_plans.LookAtAction
   pycram.robot_plans.ReachMotion
   pycram.robot_plans.PlaceAction
   pycram.robot_plans.Vector3Stamped
   pycram.robot_plans.MoveJointsMotion
   pycram.robot_plans.MoveTorsoAction
   pycram.robot_plans.SetGripperAction
   pycram.robot_plans.ParkArmsAction
   pycram.robot_plans.CarryAction
   pycram.robot_plans.StaticJointState
   pycram.robot_plans.GripperStateEnum
   pycram.robot_plans.TorsoState
   pycram.robot_plans.JointState
   pycram.robot_plans.ArmState
   pycram.robot_plans.JointStateManager
   pycram.robot_plans.BaseMotion
   pycram.robot_plans.Arms
   pycram.robot_plans.ProcessModuleManager
   pycram.robot_plans.ViewManager
   pycram.robot_plans.OpeningMotion
   pycram.robot_plans.ClosingMotion
   pycram.robot_plans.GripperState
   pycram.robot_plans.MovementType
   pycram.robot_plans.WaypointsMovementType
   pycram.robot_plans.GraspDescription
   pycram.robot_plans.PoseStamped
   pycram.robot_plans.JointStateManager
   pycram.robot_plans.ReachMotion
   pycram.robot_plans.MoveGripperMotion
   pycram.robot_plans.MoveTCPMotion
   pycram.robot_plans.MoveTCPWaypointsMotion
   pycram.robot_plans.PerceptionQuery
   pycram.robot_plans.DetectingMotion
   pycram.robot_plans.MoveMotion
   pycram.robot_plans.Vector3Stamped
   pycram.robot_plans.MoveJointsMotion
   pycram.robot_plans.LookingMotion


Functions
---------

.. autoapisummary::

   pycram.robot_plans.has_parameters
   pycram.robot_plans.quaternion_from_euler
   pycram.robot_plans.validate_close_open
   pycram.robot_plans.check_opened
   pycram.robot_plans.check_closed
   pycram.robot_plans.translate_pose_along_local_axis
   pycram.robot_plans.create_multiple_joint_goal_validator
   pycram.robot_plans.translate_pose_along_local_axis


Package Contents
----------------

.. py:data:: LookAtActionDescription

.. py:data:: NavigateActionDescription

.. py:class:: ActionConfig

   .. py:attribute:: pick_up_prepose_distance
      :value: 0.03



   .. py:attribute:: grasping_prepose_distance
      :value: 0.03



   .. py:attribute:: navigate_keep_joint_states
      :value: True



   .. py:attribute:: face_at_keep_joint_states
      :value: True



   .. py:attribute:: execution_delay
      :type:  datetime.timedelta

      The delay between the execution of actions/motions to imitate real world execution time.



.. py:class:: PartialDesignator(performable: T, *args, **kwargs)

   Bases: :py:obj:`typing_extensions.Iterable`\ [\ :py:obj:`T`\ ]


   A partial designator_description is somewhat between a DesignatorDescription and a specified designator_description. Basically it is a
   partially initialized specified designator_description which can take a list of input arguments (like a DesignatorDescription)
   and generate a list of specified designators with all possible permutations of the input arguments.

   PartialDesignators are designed as generators, as such they need to be iterated over to yield the possible specified
   designators. Please also keep in mind that at the time of iteration all parameter of the specified designator_description need
   to be filled, otherwise a TypeError will be raised, see the example below for usage.

   .. code-block:: python

           # Example usage
           partial_designator = PartialDesignator(PickUpAction, milk_object_designator, arm=[Arms.RIGHT, Arms.LEFT])
           for performable in partial_designator(Grasp.FRONT):
               performable.perform()


   .. py:attribute:: performable
      :type:  T
      :value: None


      Reference to the performable class that should be initialized



   .. py:attribute:: args
      :type:  typing_extensions.Tuple[typing_extensions.Any, Ellipsis]
      :value: None


      Arguments that are passed to the performable



   .. py:attribute:: kwargs
      :type:  typing_extensions.Dict[str, typing_extensions.Any]
      :value: None


      Keyword arguments that are passed to the performable



   .. py:attribute:: _plan_node
      :type:  pycram.plan.PlanNode
      :value: None


      Reference to the PlanNode that is used to execute the performable



   .. py:method:: __call__(*fargs, **fkwargs)

      Creates a new PartialDesignator with the given arguments and keyword arguments added. Existing arguments will
      be prioritized over the new arguments.

      :param fargs: Additional arguments that should be added to the new PartialDesignator
      :param fkwargs: Additional keyword arguments that should be added to the new PartialDesignator
      :return: A new PartialDesignator with the given arguments and keyword arguments added



   .. py:method:: __iter__() -> typing_extensions.Iterator[T]

      Iterates over all possible permutations of the arguments and keyword arguments and creates a new performable
      object for each permutation. In case there are conflicting parameters the args will be used over the keyword
      arguments.

      :return: A new performable object for each permutation of arguments and keyword arguments



   .. py:method:: generate_permutations() -> typing_extensions.Iterator[typing_extensions.Dict[str, typing_extensions.Any]]

      Generates the cartesian product of the given arguments. Arguments can also be a list of lists of arguments.

      :yields: A list with a possible permutation of the given arguments



   .. py:method:: missing_parameter() -> typing_extensions.List[str]

      Returns a list of all parameters that are missing for the performable to be initialized.

      :return: A list of parameter names that are missing from the performable



   .. py:method:: resolve() -> T

      Returns the Designator with the first set of parameters

      :return: A fully parametrized Designator



   .. py:method:: to_dict()


   .. py:method:: flatten() -> typing_extensions.List[pycram.has_parameters.leaf_types]

      Flattens a partial designator, very similar to HasParameters.flatten but this method can deal with parameters
      thet are None.

      :return: A list of flattened field values from the object.



   .. py:method:: flatten_parameters() -> typing_extensions.Dict[str, pycram.has_parameters.leaf_types]

      The flattened parameter types of the performable.

      :return: A dict with the flattened parameter types of the performable.



   .. py:property:: plan_node
      :type: pycram.plan.PlanNode


      Returns the PlanNode that is used to execute the performable.

      :return: The PlanNode that is used to execute the performable.



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



.. py:function:: has_parameters(target_class: T) -> T

   Insert parameters of a class post construction.
   Use this when dataclasses should be combined with HasParameters.

   :param target_class: The class to get the parameters from.
   :return: The updated class


.. py:class:: SequentialPlan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`LanguagePlan`


   Creates a plan which executes its children in sequential order


.. py:class:: ActionDescription

   Bases: :py:obj:`pycram.designator.DesignatorDescription`, :py:obj:`pycram.has_parameters.HasParameters`


   Base class for everything that contains potentially parameters for a plan.


   .. py:attribute:: _pre_perform_callbacks
      :value: []



   .. py:attribute:: _post_perform_callbacks
      :value: []



   .. py:method:: __post_init__()


   .. py:method:: perform() -> typing_extensions.Any

      Full execution: pre-check, plan, post-check



   .. py:method:: execute() -> typing_extensions.Any
      :abstractmethod:


      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate_precondition()
      :abstractmethod:


      Symbolic/world state precondition validation.



   .. py:method:: validate_postcondition(result: typing_extensions.Optional[typing_extensions.Any] = None)
      :abstractmethod:


      Symbolic/world state postcondition validation.



   .. py:method:: pre_perform(func) -> typing_extensions.Callable
      :classmethod:



   .. py:method:: post_perform(func) -> typing_extensions.Callable
      :classmethod:



.. py:function:: quaternion_from_euler(ai, aj, ak, axes='sxyz')

   Return quaternion from Euler angles and axis sequence.

   ai, aj, ak : Euler's roll, pitch and yaw angles
   axes : One of 24 axis sequences as string or encoded tuple

   >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
   >>> numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
   True



.. py:class:: FaceAtAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.


   .. py:attribute:: pose
      :type:  pycram.datastructures.pose.PoseStamped

      The pose to face 



   .. py:attribute:: keep_joint_states
      :type:  bool

      Keep the joint states of the robot the same during the navigation.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(pose: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], keep_joint_states: typing_extensions.Union[typing_extensions.Iterable[bool], bool] = ActionConfig.face_at_keep_joint_states) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[FaceAtAction]]
      :classmethod:



.. py:data:: FaceAtActionDescription

.. py:data:: DetectActionDescription

.. py:class:: DetectionTechnique

   Bases: :py:obj:`int`, :py:obj:`enum.Enum`


   Enum for techniques for detection tasks.


   .. py:attribute:: ALL
      :value: 0



   .. py:attribute:: HUMAN
      :value: 1



   .. py:attribute:: TYPES
      :value: 2



   .. py:attribute:: REGION
      :value: 3



   .. py:attribute:: HUMAN_ATTRIBUTES
      :value: 4



   .. py:attribute:: HUMAN_WAVING
      :value: 5



.. py:class:: CostmapLocation(target: typing_extensions.Union[pycram.datastructures.pose.PoseStamped, semantic_digital_twin.world_description.world_entity.Body], reachable_for: semantic_digital_twin.robots.abstract_robot.AbstractRobot = None, visible_for: semantic_digital_twin.robots.abstract_robot.AbstractRobot = None, reachable_arm: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]] = None, ignore_collision_with: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body]] = None, grasp_descriptions: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.grasp.GraspDescription], pycram.datastructures.grasp.GraspDescription]] = None, rotation_agnostic: bool = False)

   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`


   Uses Costmaps to create locations for complex constrains


   .. py:attribute:: target
      :type:  typing_extensions.Union[pycram.datastructures.pose.PoseStamped, semantic_digital_twin.world_description.world_entity.Body]


   .. py:attribute:: reachable_for
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot
      :value: None



   .. py:attribute:: visible_for
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot
      :value: None



   .. py:attribute:: reachable_arm
      :type:  typing_extensions.Optional[pycram.datastructures.enums.Arms]
      :value: None



   .. py:attribute:: ignore_collision_with


   .. py:attribute:: grasps
      :type:  typing_extensions.List[typing_extensions.Optional[pycram.datastructures.enums.Grasp]]


   .. py:method:: ground() -> pycram.datastructures.pose.PoseStamped

      Default specialized_designators which returns the first result from the iterator of this instance.

      :return: A resolved location



   .. py:method:: setup_costmaps(target: pycram.datastructures.pose.PoseStamped, visible_for, reachable_for) -> pycram.costmaps.Costmap

      Sets up the costmaps for the given target and robot. The costmaps are merged and stored in the final_map





   .. py:method:: __iter__() -> typing_extensions.Iterator[pycram.datastructures.pose.PoseStamped]

      Generates positions for a given set of constrains from a costmap and returns
      them. The generation is based of a costmap which itself is the product of
      merging costmaps, each for a different purpose. In any case an occupancy costmap
      is used as the base, then according to the given constrains a visibility or
      gaussian costmap is also merged with this. Once the costmaps are merged,
      a generator generates pose candidates from the costmap. Each pose candidate
      is then validated against the constraints given by the designator if all validators
      pass the pose is considered valid and yielded.

         :yield: An instance of CostmapLocation.Location with a valid position that satisfies the given constraints



.. py:exception:: PerceptionObjectNotFound(obj_type: typing_extensions.Type[PhysicalObject], technique: pycram.datastructures.enums.DetectionTechnique, region: pycram.datastructures.pose.PoseStamped, *args, **kwargs)

   Bases: :py:obj:`PerceptionLowLevelFailure`


   Thrown when an attempt to find an object by perception fails -- and this can still be interpreted as the robot
   not looking in the right direction, as opposed to the object being absent.


.. py:class:: TryInOrderPlan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`LanguagePlan`


   Creates a plan that executes all children in sequential order but does not stop if one of them throws an error


.. py:class:: SearchAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Searches for a target object around the given location.


   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Location around which to look for a target object.



   .. py:attribute:: object_sem_annotation
      :type:  typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]

      Type of the object which is searched for.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], object_type: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.SemanticAnnotation], semantic_digital_twin.world_description.world_entity.SemanticAnnotation]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[SearchAction]]
      :classmethod:



.. py:data:: SearchActionDescription

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



.. py:class:: AxisIdentifier(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for translating the axis name to a vector along that axis.


   .. py:attribute:: X
      :value: (1, 0, 0)



   .. py:attribute:: Y
      :value: (0, 1, 0)



   .. py:attribute:: Z
      :value: (0, 0, 1)



   .. py:attribute:: Undefined
      :value: (0, 0, 0)



   .. py:method:: from_tuple(axis_tuple)
      :classmethod:



.. py:class:: Grasp

   Base class for grasp enums.


   .. py:method:: __hash__()


   .. py:method:: from_axis_direction(axis: AxisIdentifier, direction: int)
      :classmethod:


      Get the Grasp face from an axis-index tuple



.. py:class:: ApproachDirection(*args, **kwds)

   Bases: :py:obj:`Grasp`, :py:obj:`enum.Enum`


   Enum for the approach direction of a gripper.
   The AxisIdentifier is used to identify the axis of the gripper, and the int is used to identify the direction along
    that axis.


   .. py:attribute:: FRONT


   .. py:attribute:: BACK


   .. py:attribute:: RIGHT


   .. py:attribute:: LEFT


   .. py:property:: axis
      :type: AxisIdentifier


      Returns the axis of the approach direction.



.. py:class:: VerticalAlignment(*args, **kwds)

   Bases: :py:obj:`Grasp`, :py:obj:`enum.Enum`


   Enum for the vertical alignment of a gripper.
   The AxisIdentifier is used to identify the axis of the gripper, and the int is used to identify the direction along
    that axis.


   .. py:attribute:: TOP


   .. py:attribute:: BOTTOM


   .. py:attribute:: NoAlignment


.. py:class:: RobotDescription(name: str, base_link: str, torso_link: str, torso_joint: str, urdf_path: str, virtual_mobile_base_joints: typing_extensions.Optional[pycram.datastructures.dataclasses.VirtualMobileBaseJoints] = None, mjcf_path: typing_extensions.Optional[str] = None, ignore_joints: typing_extensions.Optional[typing_extensions.List[str]] = None, gripper_name: typing_extensions.Optional[str] = None)

   Base class of a robot description. Contains all necessary information about a robot, like the URDF, the base link,
   the torso link and joint, the kinematic chains and cameras.


   .. py:attribute:: current_robot_description
      :type:  RobotDescription
      :value: None


      The currently loaded robot description.



   .. py:attribute:: name
      :type:  str

      Name of the robot



   .. py:attribute:: base_link
      :type:  str

      Base link of the robot



   .. py:attribute:: torso_link
      :type:  str

      Torso link of the robot



   .. py:attribute:: torso_joint
      :type:  str

      Torso joint of the robot



   .. py:attribute:: urdf_object
      :type:  urdf_parser_py.urdf.URDF

      Parsed URDF of the robot



   .. py:attribute:: kinematic_chains
      :type:  typing_extensions.Dict[str, KinematicChainDescription]

      All kinematic chains defined for this robot



   .. py:attribute:: cameras
      :type:  typing_extensions.Dict[str, CameraDescription]

      All cameras defined for this robot



   .. py:attribute:: links
      :type:  typing_extensions.List[str]

      All links defined in the URDF



   .. py:attribute:: joints
      :type:  typing_extensions.List[str]

      All joints defined in the URDF, by default fixed joints are not included



   .. py:attribute:: virtual_mobile_base_joints
      :type:  typing_extensions.Optional[pycram.datastructures.dataclasses.VirtualMobileBaseJoints]
      :value: None


      Virtual mobile base joint names for mobile robots, these joints are not part of the URDF, however they are used to
      move the robot in the simulation (e.g. set_pose for the robot would actually move these joints)



   .. py:attribute:: gripper_name
      :type:  typing_extensions.Optional[str]
      :value: None


      Name of the gripper of the robot if it has one, this is used when the gripper is a different Object with its own
      description file outside the robot description file.



   .. py:attribute:: neck
      :type:  typing_extensions.Dict[str, typing_extensions.List[str]]

      Dictionary of neck links and joints. Keys are yaw, pitch and roll, values are [link, joint]



   .. py:attribute:: ignore_joints


   .. py:attribute:: joint_types


   .. py:attribute:: joint_actuators
      :type:  typing_extensions.Optional[typing_extensions.Dict]


   .. py:method:: add_arm(end_link: str, arm_type: pycram.datastructures.enums.Arms = Arms.RIGHT, arm_name: str = 'manipulator', arm_home_values: typing_extensions.Optional[typing_extensions.Dict[str, float]] = None, arm_start: typing_extensions.Optional[str] = None) -> KinematicChainDescription

      Creates and adds an arm to the RobotDescription.

      :param end_link: Last link of the arm
      :param arm_type: Type of the arm
      :param arm_name: Name of the arm
      :param arm_home_values: Dictionary of joint names and their home values (default configuration) (e.g. park arms)
      :param arm_start: Start link of the arm



   .. py:property:: has_actuators

      Property to check if the robot has actuators defined in the MJCF file.

      :return: True if the robot has actuators, False otherwise



   .. py:method:: get_actuator_for_joint(joint: str) -> typing_extensions.Optional[str]

      Get the actuator name for a given joint.

      :param joint: Name of the joint
      :return: Name of the actuator



   .. py:method:: add_kinematic_chain_description(chain: KinematicChainDescription)

      Adds a KinematicChainDescription object to the RobotDescription. The chain is stored with the name of the chain
      as key.

      :param chain: KinematicChainDescription object to add



   .. py:method:: add_kinematic_chain(name: str, start_link: str, end_link: str)

      Creates and adds a KinematicChainDescription object to the RobotDescription.

      :param name: Name of the KinematicChainDescription object
      :param start_link: First link of the chain
      :param end_link: Last link of the chain



   .. py:method:: add_camera_description(camera: CameraDescription)

      Adds a CameraDescription object to the RobotDescription. The camera is stored with the name of the camera as key.
      :param camera: The CameraDescription object to add



   .. py:method:: add_camera(name: str, camera_link: str, minimal_height: float, maximal_height: float)

      Creates and adds a CameraDescription object to the RobotDescription. Minimal and maximal height of the camera are
      relevant if the robot has a moveable torso or the camera is mounted on a moveable part of the robot. Otherwise
      both values can be the same.

      :param name: Name of the CameraDescription object
      :param camera_link: Link of the camera in the URDF
      :param minimal_height: Minimal height of the camera
      :param maximal_height: Maximal height of the camera
      :return:



   .. py:method:: get_manipulator_chains() -> typing_extensions.List[KinematicChainDescription]

      Get a list of all manipulator chains of the robot which posses an end effector.

      :return: A list of KinematicChainDescription objects



   .. py:method:: get_camera_frame(robot_object_name: str = None) -> str

      Quick method to get the name of a link of a camera. Uses the first camera in the list of cameras.

      :return: A name of the link of a camera



   .. py:method:: get_camera_link() -> str

      Quick method to get the name of a link of a camera. Uses the first camera in the list of cameras.

      :return: A name of the link of a camera



   .. py:method:: get_default_camera() -> CameraDescription

      Get the first camera in the list of cameras.

      :return: A CameraDescription object



   .. py:method:: get_static_joint_chain(kinematic_chain_name: str, configuration_name: typing_extensions.Union[str, enum.Enum])

      Get the static joint states of a kinematic chain for a specific configuration. When trying to access one of
      the robot arms the function `:func: get_arm_chain` should be used.

      :param kinematic_chain_name:
      :param configuration_name:
      :return:



   .. py:method:: get_offset(name: str) -> typing_extensions.Optional[pycram.datastructures.pose.PoseStamped]

      Returns the offset of a Joint in the URDF.

      :param name: The name of the Joint for which the offset will be returned.
      :return: The offset of the Joint



   .. py:method:: get_parent(name: str) -> str

      Get the parent of a link or joint in the URDF. Always returns the immediate parent, for a link this is a joint
      and vice versa.

      :param name: Name of the link or joint in the URDF
      :return: Name of the parent link or joint



   .. py:method:: get_child(name: str, return_multiple_children: bool = False) -> typing_extensions.Union[str, typing_extensions.List[str]]

      Get the child of a link or joint in the URDF. Always returns the immediate child, for a link this is a joint
      and vice versa. Since a link can have multiple children, the return_multiple_children parameter can be set to
      True to get a list of all children.

      :param name: Name of the link or joint in the URDF
      :param return_multiple_children: If True, a list of all children is returned
      :return: Name of the child link or joint or a list of all children



   .. py:method:: get_arm_tool_frame(arm: pycram.datastructures.enums.Arms) -> str

      Get the name of the tool frame of a specific arm.

      :param arm: Arm for which the tool frame should be returned
      :return: The name of the link of the tool frame in the URDF.



   .. py:method:: get_arm_chain(arm: pycram.datastructures.enums.Arms) -> typing_extensions.Union[KinematicChainDescription, typing_extensions.List[KinematicChainDescription]]

      Get the kinematic chain of a specific arm. If the arm is set to BOTH, all kinematic chains are returned.

      :param arm: Arm for which the chain should be returned
      :return: KinematicChainDescription object of the arm



   .. py:method:: set_neck(yaw_joint: typing_extensions.Optional[str] = None, pitch_joint: typing_extensions.Optional[str] = None, roll_joint: typing_extensions.Optional[str] = None)

      Defines the neck configuration of the robot by setting the yaw, pitch, and roll
      joints along with their corresponding links.

      :param yaw_joint: The joint name for the yaw movement of the neck.
      :param pitch_joint: The joint name for the pitch movement of the neck.
      :param roll_joint: The joint name for the roll movement of the neck.



   .. py:method:: get_neck() -> typing_extensions.Dict[str, typing_extensions.List[typing_extensions.Optional[str]]]

      Retrieves the neck configuration of the robot, including links and joints for yaw,
      pitch, and roll.

      :return: A dictionary containing the neck configuration. Keys are yaw, pitch, and roll. Values are [link, joint].



   .. py:method:: load()

      Loads the robot description in the robot description manager, can be overridden to take more parameter into
      account.



   .. py:method:: unload()

      Unloads the robot description in the robot description manager, can be overridden to take more parameter into
      account.



.. py:class:: MixingAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Mixes contents of an object using a tool in a spiral motion.


   .. py:attribute:: object_
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object to be mixed.



   .. py:attribute:: tool
      :type:  semantic_digital_twin.world_description.world_entity.SemanticAnnotation

      The tool to be used for mixing.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to be used for the mixing action.



   .. py:attribute:: technique
      :type:  typing_extensions.Optional[str]
      :value: None


      The technique to be used for mixing, e.g. 'Spiral Mixing'.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: description(object_: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], tool: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.SemanticAnnotation], semantic_digital_twin.world_description.world_entity.SemanticAnnotation], arm: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]] = None, technique: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[str], str]] = None)
      :classmethod:



.. py:class:: PouringAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Performs a pouring action with a tool over an object, typically used for liquids.


   .. py:attribute:: object_
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object over which the pouring action is performed.



   .. py:attribute:: tool
      :type:  semantic_digital_twin.world_description.world_entity.SemanticAnnotation

      The tool used for pouring, e.g., a jug or a bottle.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to be used for the pouring action.



   .. py:attribute:: technique
      :type:  typing_extensions.Optional[str]
      :value: None


      The technique to be used for pouring, e.g., 'Pouring'.



   .. py:attribute:: angle
      :type:  typing_extensions.Optional[float]
      :value: 90


      The angle at which the tool is tilted during the pouring action, in degrees.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: description(object_: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], tool: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.SemanticAnnotation], semantic_digital_twin.world_description.world_entity.SemanticAnnotation], arm: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]] = None, technique: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[str], str]] = None, angle: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[float], float]] = 90)
      :classmethod:



.. py:class:: CuttingAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Performs a cutting action on an object using a specified tool.


   .. py:attribute:: object_
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object to be cut.



   .. py:attribute:: tool
      :type:  semantic_digital_twin.world_description.world_entity.SemanticAnnotation

      The tool used for cutting, e.g., a knife or a saw.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to be used for the cutting action.



   .. py:attribute:: technique
      :type:  typing_extensions.Optional[str]
      :value: None


      The technique to be used for cutting, e.g., 'Slicing', 'Halving', etc.



   .. py:attribute:: slice_thickness
      :type:  typing_extensions.Optional[float]
      :value: 0.03


      The thickness of each slice to be cut from the object, in meters.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: description(object_: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], tool: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.SemanticAnnotation], semantic_digital_twin.world_description.world_entity.SemanticAnnotation], arm: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]] = None, technique: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[str], str]] = None, slice_thickness: typing_extensions.Optional[float] = 0.03)
      :classmethod:



   .. py:method:: calculate_slices(obj_length)


   .. py:method:: perpendicular_pose(slice_pose, angle) -> pycram.datastructures.pose.PoseStamped
      :staticmethod:



   .. py:method:: get_rotation_offset_from_axis_preference(pose_a, pose_b: pycram.datastructures.pose.PoseStamped) -> Tuple[int, float]
      :staticmethod:


      Compute a discrete rotation offset (-90 or 90 degrees) to align this pose's local axes with the direction
      toward a target pose, based on which axis (X or Y) is more aligned.

      :param pose_a: The source pose.
      :param pose_b: The target pose to align with.
      :return: Tuple of (rotation offset in degrees, signed angle difference in radians for Y axis).



.. py:data:: CuttingActionDescription

.. py:data:: PouringActionDescription

.. py:data:: MixingActionDescription

.. py:data:: ParkArmsActionDescription

.. py:data:: PickUpActionDescription

.. py:data:: PlaceActionDescription

.. py:data:: OpenActionDescription

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



.. py:class:: ProbabilisticCostmapLocation(target: typing_extensions.Union[pycram.datastructures.pose.PoseStamped, semantic_digital_twin.world_description.world_entity.Body], reachable_for: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.robots.abstract_robot.AbstractRobot], semantic_digital_twin.robots.abstract_robot.AbstractRobot]] = None, visible_for: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.robots.abstract_robot.AbstractRobot], semantic_digital_twin.robots.abstract_robot.AbstractRobot]] = None, reachable_arm: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]] = None, ignore_collision_with: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body]] = None, grasp_descriptions: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.grasp.GraspDescription], pycram.datastructures.grasp.GraspDescription]] = None, object_in_hand: typing_extensions.Optional[typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body]] = None, rotation_agnostic: bool = False, number_of_samples: int = 300, costmap_resolution: float = 0.1)

   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`


   Uses Costmaps to create locations for complex constrains.
   The construction of the probabilistic circuit can be quite time-consuming, but the majority of the computational load
   is done during the first iteration only. In this case the exact time is highly dependent on the number of
   links in the world.


   .. py:attribute:: target_x

      Variable representing the x coordinate of the target pose



   .. py:attribute:: target_y

      Variable representing the y coordinate of the target pose



   .. py:attribute:: number_of_samples
      :value: 300



   .. py:attribute:: costmap_resolution
      :value: 0.05



   .. py:attribute:: target
      :type:  typing_extensions.Union[pycram.datastructures.pose.PoseStamped, semantic_digital_twin.world_description.world_entity.Body]


   .. py:attribute:: reachable_for
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot
      :value: None



   .. py:attribute:: visible_for
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot
      :value: None



   .. py:attribute:: reachable_arm
      :type:  typing_extensions.Optional[pycram.datastructures.enums.Arms]
      :value: None



   .. py:attribute:: ignore_collision_with


   .. py:attribute:: grasps
      :type:  typing_extensions.List[typing_extensions.Optional[pycram.datastructures.enums.Grasp]]


   .. py:method:: ground() -> pycram.datastructures.pose.PoseStamped

      Default specialized_designators which returns the first element of the iterator of this instance.

      :return: A resolved location



   .. py:method:: _calculate_room_event(world: semantic_digital_twin.world.World, free_space_graph: semantic_digital_twin.world_description.graph_of_convex_sets.GraphOfConvexSets, target_position: pycram.datastructures.pose.PyCramVector3) -> random_events.product_algebra.Event
      :staticmethod:


      Calculates an event for the free space inside the room around the target position is located in, in 2d.

      :param world: The current world
      :param free_space_graph: The free space graph that is used as basis for the room calculation
      :param target_position: The position of the target in the world

      :return: An Event that describes the room around the target position in 2d



   .. py:method:: _create_free_space_conditions(world: semantic_digital_twin.world.World, target_position: pycram.datastructures.pose.PyCramVector3, search_distance: float = 1.5) -> typing_extensions.Tuple[random_events.product_algebra.Event, random_events.product_algebra.Event, random_events.product_algebra.Event]

      Creates the conditions for the free space around the target position.
      1. reachable_space_condition: The condition that describes the reachable space around the target position in 3d
      2. navigation_space_condition: The condition that describes the navigation space around the target position in 2d
      3. room_condition: The condition that describes the room around the target position in 2d

      :param world: The current world
      :param target_position: The position of the target in the world
      :param search_distance: The distance around the target position to search for free space, defaults to 1.5 meters

      :return: A tuple containing the reachable space condition, navigation space condition and room condition



   .. py:method:: _create_navigation_circuit(target_position: pycram.datastructures.pose.PyCramVector3) -> probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit

      Creates a probabilistic circuit that samples navigation poses around the target position.



   .. py:method:: __iter__() -> typing_extensions.Iterator[pycram.datastructures.pose.PoseStamped]

      Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
      which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
      is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

      :yield: An instance of ProbabilisticSemanticLocation.Location with the found valid position of the Costmap.



.. py:class:: GiskardLocation

   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`


   Finds a standing pose for a robot such that the TCP of the given arm can reach the target_pose. This Location
   Designator uses Giskard and full body control to find a pose for the robot base.


   .. py:attribute:: target_pose
      :type:  pycram.datastructures.pose.PoseStamped

      Target pose for which a standing pose should be found. 



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm which should read the target pose



   .. py:attribute:: grasp_description
      :type:  typing_extensions.Optional[pycram.datastructures.grasp.GraspDescription]
      :value: None


      The grasp description which should be used to grasp the target pose, used if there is a body at the pose



   .. py:attribute:: threshold
      :type:  float
      :value: 0.02


      Threshold between the TCP of the arm and the target pose at which a stand pose if deemed successfull



   .. py:method:: __post_init__()


   .. py:method:: ground() -> pycram.datastructures.pose.PoseStamped

      Default specialized_designators which returns the first element of the iterator of this instance.

      :return: A resolved location



   .. py:method:: setup_costmap(pose: pycram.datastructures.pose.PoseStamped) -> pycram.costmaps.Costmap

      Setup the reachability costmap for initial pose estimation.



   .. py:method:: setup_giskard_executor(pose_sequence: typing_extensions.List[pycram.datastructures.pose.PoseStamped], world: semantic_digital_twin.world.World, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot, end_effector: semantic_digital_twin.world_description.world_entity.Body) -> giskardpy.executor.Executor

      Setup the Giskard executor for a specific pose sequence and a given world.

      :param pose_sequence: The pose sequence which the end_effector should follow
      :param world: The world in which the pose sequence should be executed
      :param robot_view: The robot view of the robot which should be used for the execution, needs to fit the world
      :param end_effector: The end effector which should be controlled by Giskard
      :return: The Giskard executor for the pose sequence



   .. py:method:: __iter__() -> typing_extensions.Iterator[pycram.datastructures.pose.GraspPose]

      Iterates over all possible permutations of the arguments and keyword arguments and creates a new performable
      object for each permutation. In case there are conflicting parameters the args will be used over the keyword
      arguments.

      :return: A new performable object for each permutation of arguments and keyword arguments



.. py:class:: BelieveObject(names: typing_extensions.Optional[typing_extensions.List[str]] = None)

   Bases: :py:obj:`pycram.external_interfaces.robokudo.ObjectDesignatorDescription`


   Description for Objects that are only believed in.


.. py:exception:: ObjectUnfetchable(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when no base positioning can assure a reachable pose to grasp the object from.


.. py:exception:: ConfigurationNotReached(goal_validator: pycram.validation.goal_validator.MultiJointPositionGoalValidator, configuration_type: pycram.datastructures.enums.StaticJointState, *args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


   .. py:attribute:: goal_validator
      :type:  pycram.validation.goal_validator.MultiJointPositionGoalValidator

      The goal validator that was used to check if the goal was reached.



   .. py:attribute:: configuration_type
      :type:  pycram.datastructures.enums.StaticJointState

      The configuration type that should be reached.



.. py:class:: TransportAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Transports an object to a position using an arm


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be transported.



   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Target Location to which the object should be transported



   .. py:attribute:: arm
      :type:  typing_extensions.Optional[pycram.datastructures.enums.Arms]

      Arm that should be used



   .. py:attribute:: place_rotation_agnostic
      :type:  typing_extensions.Optional[bool]
      :value: False


      If True, the robot will place the object in the same orientation as it is itself, no matter how the object was grasped.



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: inside_container() -> List[semantic_digital_twin.world_description.world_entity.Body]


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, place_rotation_agnostic: typing_extensions.Optional[bool] = False) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[TransportAction]]
      :classmethod:



.. py:class:: PickAndPlaceAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Transports an object to a position using an arm without moving the base of the robot


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be transported.



   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Target Location to which the object should be transported



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      Description of the grasp to pick up the target



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasp_description=GraspDescription) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[PickAndPlaceAction]]
      :classmethod:



.. py:class:: MoveAndPlaceAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Navigate to `standing_position`, then turn towards the object and pick it up.


   .. py:attribute:: standing_position
      :type:  pycram.datastructures.pose.PoseStamped

      The pose to stand before trying to pick up the object



   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object to pick up



   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      The location to place the object.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to use



   .. py:attribute:: keep_joint_states
      :type:  bool

      Keep the joint states of the robot the same during the navigation.



   .. py:method:: execute()

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(standing_position: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, keep_joint_states: typing_extensions.Union[typing_extensions.Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[MoveAndPlaceAction]]
      :classmethod:



.. py:class:: MoveAndPickUpAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Navigate to `standing_position`, then turn towards the object and pick it up.


   .. py:attribute:: standing_position
      :type:  pycram.datastructures.pose.PoseStamped

      The pose to stand before trying to pick up the object



   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      The object to pick up



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm to use



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      The grasp to use



   .. py:attribute:: keep_joint_states
      :type:  bool

      Keep the joint states of the robot the same during the navigation.



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute()

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(standing_position: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], object_designator: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasp_description: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Grasp], pycram.datastructures.enums.Grasp] = None, keep_joint_states: typing_extensions.Union[typing_extensions.Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[MoveAndPickUpAction]]
      :classmethod:



.. py:class:: EfficientTransportAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   To transport an object to a target location by choosing the closest
   available arm using simple Euclidean distance.


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body


   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped


   .. py:method:: _choose_best_arm(robot: semantic_digital_twin.world_description.world_entity.Body, obj: semantic_digital_twin.world_description.world_entity.Body) -> pycram.datastructures.enums.Arms

      Function to find the closest available arm.



   .. py:method:: execute() -> None

      The main plan for the transport action, optimized for a stationary robot.



   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[EfficientTransportAction]]
      :classmethod:



.. py:data:: TransportActionDescription

.. py:data:: PickAndPlaceActionDescription

.. py:data:: MoveAndPlaceActionDescription

.. py:data:: MoveAndPickUpActionDescription

.. py:data:: EfficientTransportActionDescription

.. py:data:: GraspingActionDescription

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


.. py:class:: GripperState(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different motions of the gripper.


   .. py:attribute:: OPEN


   .. py:attribute:: CLOSE


   .. py:attribute:: MEDIUM


   .. py:method:: __str__()


   .. py:method:: __repr__()


.. py:class:: ContainerManipulationType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different types of container manipulation.


   .. py:attribute:: Opening

      The Opening type is used to open a container.



   .. py:attribute:: Closing

      The Closing type is used to close a container.



.. py:exception:: ContainerManipulationError(robot: Object, arms: typing_extensions.List[pycram.datastructures.enums.Arms], body: PhysicalBody, container_joint: Joint, manipulation_type: pycram.datastructures.enums.ContainerManipulationType, *args, **kwargs)

   Bases: :py:obj:`ManipulationLowLevelFailure`, :py:obj:`abc.ABC`


   Thrown when container manipulation fails.


   .. py:attribute:: container_joint
      :type:  Joint

      The joint of the container that should be manipulated.



   .. py:attribute:: manipulation_type
      :type:  pycram.datastructures.enums.ContainerManipulationType

      The type of manipulation that should be performed on the container.



.. py:class:: OpenAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Opens a container like object


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be opened



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used for opening the container



   .. py:attribute:: grasping_prepose_distance
      :type:  float

      The distance in meters the gripper should be at in the x-axis away from the handle.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if the container is opened, this assumes that the container state can be read accurately from the
      real world.



   .. py:method:: description(object_designator_description: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasping_prepose_distance: typing_extensions.Union[typing_extensions.Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[OpenAction]]
      :classmethod:



.. py:class:: CloseAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Closes a container like object.


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be closed



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that should be used for closing



   .. py:attribute:: grasping_prepose_distance
      :type:  float

      The distance in meters between the gripper and the handle before approaching to grasp.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if the container is closed, this assumes that the container state can be read accurately from the
      real world.



   .. py:method:: description(object_designator_description: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasping_prepose_distance: typing_extensions.Union[typing_extensions.Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[CloseAction]]
      :classmethod:



.. py:function:: validate_close_open(object_designator: semantic_digital_twin.world_description.world_entity.Body, arm: pycram.datastructures.enums.Arms, action_type: typing_extensions.Union[typing_extensions.Type[OpenAction], typing_extensions.Type[CloseAction]])

   Validates if the container is opened or closed by checking the joint position of the container.

   :param object_designator: The object designator_description describing the object that should be opened or closed.
   :param arm: The arm that should be used for opening or closing the container.
   :param action_type: The type of the action that should be validated.


.. py:function:: check_opened(joint_obj: semantic_digital_twin.world_description.world_entity.Connection, obj_part: semantic_digital_twin.world_description.world_entity.Body, arm: pycram.datastructures.enums.Arms, upper_limit: float)

.. py:function:: check_closed(joint_obj: semantic_digital_twin.world_description.world_entity.Connection, obj_part: semantic_digital_twin.world_description.world_entity.Body, arm: pycram.datastructures.enums.Arms, lower_limit: float)

.. py:data:: CloseActionDescription

.. py:class:: MovementType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different movement types of the robot.


   .. py:attribute:: STRAIGHT_TRANSLATION


   .. py:attribute:: STRAIGHT_CARTESIAN


   .. py:attribute:: TRANSLATION


   .. py:attribute:: CARTESIAN


.. py:class:: FindBodyInRegionMethod(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different methods to find a body in a region.


   .. py:attribute:: FingerToCentroid

      The FingerToCentroid method is used to find the body in a region by casting a ray from each finger to the
       centroid of the region.



   .. py:attribute:: Centroid

      The Centroid method is used to find the body in a region by calculating the centroid of the region and
      casting two rays from opposite sides of the region to the centroid.



   .. py:attribute:: MultiRay

      The MultiRay method is used to find the body in a region by casting multiple rays covering the region.



.. py:exception:: ObjectNotGraspedError(obj: Object, robot: Object, arm: pycram.datastructures.enums.Arms, grasp=None, *args, **kwargs)

   Bases: :py:obj:`Grasping`


.. py:exception:: ObjectNotInGraspingArea(obj: Object, robot: Object, arm: pycram.datastructures.enums.Arms, grasp, *args, **kwargs)

   Bases: :py:obj:`ReachabilityFailure`


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


.. py:data:: logger

.. py:class:: ReachAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Let the robot reach a specific pose.


   .. py:attribute:: target_pose
      :type:  pycram.datastructures.pose.PoseStamped

      Pose that should be reached.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that should be used for pick up



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      The grasp description that should be used for picking up the object



   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body
      :value: None


      Object designator_description describing the object that should be picked up



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if object is contained in the gripper such that it can be grasped and picked up.



   .. py:method:: description(target_pose: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasp_description: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.grasp.GraspDescription], pycram.datastructures.grasp.GraspDescription] = None, object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[ReachAction]]
      :classmethod:



.. py:class:: PickUpAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Let the robot pick up an object.


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be picked up



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that should be used for pick up



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription

      The GraspDescription that should be used for picking up the object



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if picked up object is in contact with the gripper.



   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, grasp_description: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.grasp.GraspDescription], pycram.datastructures.grasp.GraspDescription] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[PickUpAction]]
      :classmethod:



.. py:class:: GraspingAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Grasps an object described by the given Object Designator description


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object Designator for the object that should be grasped



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that should be used to grasp



   .. py:attribute:: prepose_distance
      :type:  float

      The distance in meters the gripper should be at before grasping the object



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, prepose_distance: typing_extensions.Union[typing_extensions.Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[GraspingAction]]
      :classmethod:



.. py:data:: ReachActionDescription

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


.. py:class:: DetectionState

   Bases: :py:obj:`int`, :py:obj:`enum.Enum`


   Enum for the state of the detection task.


   .. py:attribute:: START
      :value: 0



   .. py:attribute:: STOP
      :value: 1



   .. py:attribute:: PAUSE
      :value: 2



.. py:class:: DetectAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Detects an object that fits the object description and returns an object designator_description describing the object.

   If no object is found, an PerceptionObjectNotFound error is raised.


   .. py:attribute:: technique
      :type:  pycram.datastructures.enums.DetectionTechnique

      The technique that should be used for detection



   .. py:attribute:: state
      :type:  typing_extensions.Optional[pycram.datastructures.enums.DetectionState]
      :value: None


      The state of the detection, e.g Start Stop for continues perception



   .. py:attribute:: object_sem_annotation
      :type:  typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]
      :value: None


      The type of the object that should be detected, only considered if technique is equal to Type



   .. py:attribute:: region
      :type:  typing_extensions.Optional[semantic_digital_twin.world_description.world_entity.Region]
      :value: None


      The region in which the object should be detected



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(technique: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.DetectionTechnique], pycram.datastructures.enums.DetectionTechnique], state: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.DetectionState], pycram.datastructures.enums.DetectionState] = None, object_sem_annotation: typing_extensions.Union[typing_extensions.Iterable[typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]], typing_extensions.Type[semantic_digital_twin.world_description.world_entity.SemanticAnnotation]] = None, region: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Region], semantic_digital_twin.world_description.world_entity.Region] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[DetectAction]]
      :classmethod:



.. py:class:: LookingMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Lets the robot look at a point


   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      Target pose to look at



   .. py:method:: perform()


   .. py:property:: _motion_chart


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


.. py:exception:: LookAtGoalNotReached(robot: Object, target: pycram.datastructures.pose.PoseStamped, *args, **kwargs)

   Bases: :py:obj:`LookingHighLevelFailure`


   Thrown when the look at goal is not reached.


.. py:exception:: NavigationGoalNotReachedError(current_pose: pycram.datastructures.pose.PoseStamped, goal_pose: pycram.datastructures.pose.PoseStamped, *args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Thrown when the navigation goal is not reached.


   .. py:attribute:: current_pose
      :type:  pycram.datastructures.pose.PoseStamped

      The current pose of the robot.



   .. py:attribute:: goal_pose
      :type:  pycram.datastructures.pose.PoseStamped

      The goal pose of the robot.



.. py:class:: PoseErrorChecker(acceptable_error: typing_extensions.Union[typing_extensions.Tuple[float], typing_extensions.Iterable[typing_extensions.Tuple[float]]] = (0.001, np.pi / 180), is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`ErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


   .. py:method:: _calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> typing_extensions.List[float]

      Calculate the error between two poses.

      :param value_1: The first pose.
      :param value_2: The second pose.



.. py:class:: NavigateAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Navigates the Robot to a position.


   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Location to which the robot should be navigated



   .. py:attribute:: keep_joint_states
      :type:  bool

      Keep the joint states of the robot the same during the navigation.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)


   .. py:method:: description(target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], keep_joint_states: typing_extensions.Union[typing_extensions.Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[NavigateAction]]
      :classmethod:



.. py:class:: LookAtAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Lets the robot look at a position.


   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      Position at which the robot should look, given as 6D pose



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if the robot is looking at the target location by spawning a virtual object at the target location and
      creating a ray from the camera and checking if it intersects with the object.



   .. py:method:: description(target: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[LookAtAction]]
      :classmethod:



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


.. py:exception:: ObjectNotPlacedAtTargetLocation(obj: Object, placing_pose: pycram.datastructures.pose.PoseStamped, robot: Object, arm: pycram.datastructures.enums.Arms, *args, **kwargs)

   Bases: :py:obj:`ObjectPlacingError`


   Thrown when the object was not placed at the target location.


.. py:exception:: ObjectStillInContact(obj: Object, contact_links: typing_extensions.List[Link], placing_pose: pycram.datastructures.pose.PoseStamped, robot: Object, arm: pycram.datastructures.enums.Arms, *args, **kwargs)

   Bases: :py:obj:`ObjectPlacingError`


   Thrown when the object is still in contact with the robot after placing.


   .. py:attribute:: contact_links
      :type:  typing_extensions.List[Link]

      The links of the robot that are still in contact with the object.



.. py:class:: PlaceAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Places an Object at a position using an arm.


   .. py:attribute:: object_designator
      :type:  semantic_digital_twin.world_description.world_entity.Body

      Object designator_description describing the object that should be place



   .. py:attribute:: target_location
      :type:  pycram.datastructures.pose.PoseStamped

      Pose in the world at which the object should be placed



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      Arm that is currently holding the object



   .. py:attribute:: _pre_perform_callbacks
      :value: []


      List to save the callbacks which should be called before performing the action.



   .. py:method:: __post_init__()


   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: typing_extensions.Optional[datetime.timedelta] = None)

      Check if the object is placed at the target location.



   .. py:method:: validate_loss_of_contact()

      Check if the object is still in contact with the robot after placing it.



   .. py:method:: validate_placement_location()

      Check if the object is placed at the target location.



   .. py:method:: description(object_designator: typing_extensions.Union[typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body], semantic_digital_twin.world_description.world_entity.Body], target_location: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.pose.PoseStamped], pycram.datastructures.pose.PoseStamped], arm: typing_extensions.Union[typing_extensions.Iterable[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[PlaceAction]]
      :classmethod:



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


.. py:exception:: TorsoGoalNotReached(goal_validator: typing_extensions.Optional[pycram.validation.goal_validator.MultiJointPositionGoalValidator] = None, *args, **kwargs)

   Bases: :py:obj:`TorsoLowLevelFailure`


   Thrown when the torso moved as a result of a torso action but the goal was not reached.


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


.. py:function:: create_multiple_joint_goal_validator(robot: Object, joint_positions: typing_extensions.Union[typing_extensions.Dict[Joint, float], typing_extensions.Dict[str, float]]) -> MultiJointPositionGoalValidator

   Validate the multiple joint goals, and wait until the goal is achieved.

   :param robot: The robot object.
   :param joint_positions: The joint positions to validate.


.. py:class:: MoveTorsoAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Move the torso of the robot up and down.


   .. py:attribute:: torso_state
      :type:  pycram.robot_descriptions.hsrb_states.TorsoState

      The state of the torso that should be set



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: datetime.timedelta = timedelta(seconds=2))

      Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.



   .. py:method:: description(torso_state: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.TorsoState], pycram.robot_descriptions.hsrb_states.TorsoState]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[MoveTorsoAction]]
      :classmethod:



.. py:class:: SetGripperAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Set the gripper state of the robot.


   .. py:attribute:: gripper
      :type:  pycram.robot_descriptions.hsrb_states.Arms

      The gripper that should be set 



   .. py:attribute:: motion
      :type:  pycram.robot_descriptions.hsrb_states.GripperStateEnum

      The motion that should be set on the gripper



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: datetime.timedelta = timedelta(seconds=2))

      Needs gripper state to be read or perceived.



   .. py:method:: description(gripper: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.Arms], pycram.robot_descriptions.hsrb_states.Arms], motion: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.GripperState], pycram.robot_descriptions.hsrb_states.GripperState] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[SetGripperAction]]
      :classmethod:



.. py:class:: ParkArmsAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Park the arms of the robot.


   .. py:attribute:: arm
      :type:  pycram.robot_descriptions.hsrb_states.Arms

      Entry from the enum for which arm should be parked.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: get_joint_poses() -> Tuple[List[str], List[float]]

      :return: The joint positions that should be set for the arm to be in the park position.



   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: datetime.timedelta = timedelta(seconds=2))

      Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.



   .. py:method:: description(arm: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.Arms], pycram.robot_descriptions.hsrb_states.Arms]) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[ParkArmsAction]]
      :classmethod:



.. py:class:: CarryAction

   Bases: :py:obj:`pycram.robot_plans.actions.base.ActionDescription`


   Parks the robot's arms. And align the arm with the given Axis of a frame.


   .. py:attribute:: arm
      :type:  pycram.robot_descriptions.hsrb_states.Arms

      Entry from the enum for which arm should be parked.



   .. py:attribute:: align
      :type:  typing_extensions.Optional[bool]
      :value: False


      If True, aligns the end-effector with a specified axis.



   .. py:attribute:: tip_link
      :type:  typing_extensions.Optional[str]
      :value: None


      Name of the tip link to align with, e.g the object.



   .. py:attribute:: tip_axis
      :type:  typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier]
      :value: None


      Tip axis of the tip link, that should be aligned.



   .. py:attribute:: root_link
      :type:  typing_extensions.Optional[str]
      :value: None


      Base link of the robot; typically set to the torso.



   .. py:attribute:: root_axis
      :type:  typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier]
      :value: None


      Goal axis of the root link, that should be used to align with.



   .. py:method:: execute() -> None

      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: get_joint_poses() -> typing_extensions.Dict[str, float]

      :return: The joint positions that should be set for the arm to be in the park position.



   .. py:method:: axis_to_vector3_stamped(axis: pycram.datastructures.enums.AxisIdentifier, link: str = 'base_link') -> pycram.datastructures.pose.Vector3Stamped


   .. py:method:: validate(result: typing_extensions.Optional[typing_extensions.Any] = None, max_wait_time: datetime.timedelta = timedelta(seconds=2))

      Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.



   .. py:method:: description(arm: typing_extensions.Union[typing_extensions.Iterable[pycram.robot_descriptions.hsrb_states.Arms], pycram.robot_descriptions.hsrb_states.Arms], align: typing_extensions.Optional[bool] = False, tip_link: typing_extensions.Optional[str] = None, tip_axis: typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier] = None, root_link: typing_extensions.Optional[str] = None, root_axis: typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier] = None) -> pycram.datastructures.partial_designator.PartialDesignator[typing_extensions.Type[CarryAction]]
      :classmethod:



.. py:data:: MoveTorsoActionDescription

.. py:data:: SetGripperActionDescription

.. py:data:: CarryActionDescription

.. py:class:: StaticJointState(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Create a collection of name/value pairs.

   Example enumeration:

   >>> class Color(Enum):
   ...     RED = 1
   ...     BLUE = 2
   ...     GREEN = 3

   Access them by:

   - attribute access:

     >>> Color.RED
     <Color.RED: 1>

   - value lookup:

     >>> Color(1)
     <Color.RED: 1>

   - name lookup:

     >>> Color['RED']
     <Color.RED: 1>

   Enumerations can be iterated over, and know how many members they have:

   >>> len(Color)
   3

   >>> list(Color)
   [<Color.RED: 1>, <Color.BLUE: 2>, <Color.GREEN: 3>]

   Methods can be added to enumerations, and members can have their own
   attributes -- see the documentation for details.


   .. py:attribute:: Park
      :value: 'park'



.. py:class:: GripperStateEnum(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different motions of the gripper.


   .. py:attribute:: OPEN


   .. py:attribute:: CLOSE


   .. py:attribute:: MEDIUM


   .. py:method:: __str__()


   .. py:method:: __repr__()


.. py:class:: TorsoState

   Bases: :py:obj:`enum.IntEnum`


   Enum for the different states of the torso.


   .. py:attribute:: HIGH


   .. py:attribute:: MID


   .. py:attribute:: LOW


.. py:class:: JointState

   Represents a named joint state of a robot. For example, the park position of the arms.


   .. py:attribute:: name
      :type:  semantic_digital_twin.datastructures.prefixed_name.PrefixedName

      Name of the joint state



   .. py:attribute:: joint_names
      :type:  List[str]

      Names of the joints in this state



   .. py:attribute:: joint_positions
      :type:  List[float]

      position of the joints in this state, must correspond to the joint_names



   .. py:attribute:: state_type
      :type:  enum.Enum
      :value: None


      Enum type of the joints tate (e.g., Park, Open)



   .. py:method:: apply_to_world(world: semantic_digital_twin.world.World)

      Applies the joint state to the robot in the given world.
      :param world: The world in which the robot is located.



.. py:class:: ArmState

   Bases: :py:obj:`JointState`


   Represents a named joint state of a robot. For example, the park position of the arms.


   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms
      :value: None



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



.. py:data:: right_park

.. py:data:: left_park

.. py:data:: both_park

.. py:data:: left_gripper_open

.. py:data:: left_gripper_close

.. py:data:: right_gripper_open

.. py:data:: right_gripper_close

.. py:data:: torso_low

.. py:data:: torso_mid

.. py:data:: torso_high

.. py:data:: left_arm

.. py:data:: both_arm

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


.. py:class:: DetectingMotion

   Bases: :py:obj:`pycram.robot_plans.motions.base.BaseMotion`


   Tries to detect an object in the FOV of the robot

   returns: ObjectDesignatorDescription.Object or Error: PerceptionObjectNotFound


   .. py:attribute:: query
      :type:  pycram.perception.PerceptionQuery

      Query for the perception system that should be answered



   .. py:method:: perform()


   .. py:property:: _motion_chart


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


