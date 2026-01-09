pycram.robot_plans.actions.composite
====================================

.. py:module:: pycram.robot_plans.actions.composite


Submodules
----------

.. toctree::
   :maxdepth: 1

   /autoapi/pycram/robot_plans/actions/composite/facing/index
   /autoapi/pycram/robot_plans/actions/composite/searching/index
   /autoapi/pycram/robot_plans/actions/composite/tool_based/index
   /autoapi/pycram/robot_plans/actions/composite/transporting/index


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.composite.LookAtActionDescription
   pycram.robot_plans.actions.composite.NavigateActionDescription
   pycram.robot_plans.actions.composite.FaceAtActionDescription
   pycram.robot_plans.actions.composite.DetectActionDescription
   pycram.robot_plans.actions.composite.LookAtActionDescription
   pycram.robot_plans.actions.composite.NavigateActionDescription
   pycram.robot_plans.actions.composite.SearchActionDescription
   pycram.robot_plans.actions.composite.CuttingActionDescription
   pycram.robot_plans.actions.composite.PouringActionDescription
   pycram.robot_plans.actions.composite.MixingActionDescription
   pycram.robot_plans.actions.composite.FaceAtActionDescription
   pycram.robot_plans.actions.composite.ParkArmsActionDescription
   pycram.robot_plans.actions.composite.NavigateActionDescription
   pycram.robot_plans.actions.composite.PickUpActionDescription
   pycram.robot_plans.actions.composite.PlaceActionDescription
   pycram.robot_plans.actions.composite.OpenActionDescription
   pycram.robot_plans.actions.composite.TransportActionDescription
   pycram.robot_plans.actions.composite.PickAndPlaceActionDescription
   pycram.robot_plans.actions.composite.MoveAndPlaceActionDescription
   pycram.robot_plans.actions.composite.MoveAndPickUpActionDescription
   pycram.robot_plans.actions.composite.EfficientTransportActionDescription


Exceptions
----------

.. autoapisummary::

   pycram.robot_plans.actions.composite.PerceptionObjectNotFound
   pycram.robot_plans.actions.composite.ObjectUnfetchable
   pycram.robot_plans.actions.composite.ConfigurationNotReached


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.composite.ActionConfig
   pycram.robot_plans.actions.composite.PartialDesignator
   pycram.robot_plans.actions.composite.PoseStamped
   pycram.robot_plans.actions.composite.SequentialPlan
   pycram.robot_plans.actions.composite.ActionDescription
   pycram.robot_plans.actions.composite.FaceAtAction
   pycram.robot_plans.actions.composite.DetectionTechnique
   pycram.robot_plans.actions.composite.PartialDesignator
   pycram.robot_plans.actions.composite.PoseStamped
   pycram.robot_plans.actions.composite.CostmapLocation
   pycram.robot_plans.actions.composite.TryInOrderPlan
   pycram.robot_plans.actions.composite.SequentialPlan
   pycram.robot_plans.actions.composite.ActionDescription
   pycram.robot_plans.actions.composite.SearchAction
   pycram.robot_plans.actions.composite.MoveTCPMotion
   pycram.robot_plans.actions.composite.PoseStamped
   pycram.robot_plans.actions.composite.PartialDesignator
   pycram.robot_plans.actions.composite.Arms
   pycram.robot_plans.actions.composite.AxisIdentifier
   pycram.robot_plans.actions.composite.Grasp
   pycram.robot_plans.actions.composite.ApproachDirection
   pycram.robot_plans.actions.composite.VerticalAlignment
   pycram.robot_plans.actions.composite.RobotDescription
   pycram.robot_plans.actions.composite.ActionDescription
   pycram.robot_plans.actions.composite.MixingAction
   pycram.robot_plans.actions.composite.PouringAction
   pycram.robot_plans.actions.composite.CuttingAction
   pycram.robot_plans.actions.composite.ActionConfig
   pycram.robot_plans.actions.composite.Arms
   pycram.robot_plans.actions.composite.Grasp
   pycram.robot_plans.actions.composite.VerticalAlignment
   pycram.robot_plans.actions.composite.GraspDescription
   pycram.robot_plans.actions.composite.PartialDesignator
   pycram.robot_plans.actions.composite.PoseStamped
   pycram.robot_plans.actions.composite.ProbabilisticCostmapLocation
   pycram.robot_plans.actions.composite.CostmapLocation
   pycram.robot_plans.actions.composite.GiskardLocation
   pycram.robot_plans.actions.composite.BelieveObject
   pycram.robot_plans.actions.composite.SequentialPlan
   pycram.robot_plans.actions.composite.RobotDescription
   pycram.robot_plans.actions.composite.ActionDescription
   pycram.robot_plans.actions.composite.TransportAction
   pycram.robot_plans.actions.composite.PickAndPlaceAction
   pycram.robot_plans.actions.composite.MoveAndPlaceAction
   pycram.robot_plans.actions.composite.MoveAndPickUpAction
   pycram.robot_plans.actions.composite.EfficientTransportAction


Functions
---------

.. autoapisummary::

   pycram.robot_plans.actions.composite.has_parameters
   pycram.robot_plans.actions.composite.quaternion_from_euler
   pycram.robot_plans.actions.composite.has_parameters
   pycram.robot_plans.actions.composite.has_parameters
   pycram.robot_plans.actions.composite.has_parameters


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

.. py:data:: LookAtActionDescription

.. py:data:: NavigateActionDescription

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


.. py:function:: has_parameters(target_class: T) -> T

   Insert parameters of a class post construction.
   Use this when dataclasses should be combined with HasParameters.

   :param target_class: The class to get the parameters from.
   :return: The updated class


.. py:class:: TryInOrderPlan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`LanguagePlan`


   Creates a plan that executes all children in sequential order but does not stop if one of them throws an error


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

.. py:data:: FaceAtActionDescription

.. py:data:: ParkArmsActionDescription

.. py:data:: NavigateActionDescription

.. py:data:: PickUpActionDescription

.. py:data:: PlaceActionDescription

.. py:data:: OpenActionDescription

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



.. py:class:: Grasp

   Base class for grasp enums.


   .. py:method:: __hash__()


   .. py:method:: from_axis_direction(axis: AxisIdentifier, direction: int)
      :classmethod:


      Get the Grasp face from an axis-index tuple



.. py:class:: VerticalAlignment(*args, **kwds)

   Bases: :py:obj:`Grasp`, :py:obj:`enum.Enum`


   Enum for the vertical alignment of a gripper.
   The AxisIdentifier is used to identify the axis of the gripper, and the int is used to identify the direction along
    that axis.


   .. py:attribute:: TOP


   .. py:attribute:: BOTTOM


   .. py:attribute:: NoAlignment


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



.. py:function:: has_parameters(target_class: T) -> T

   Insert parameters of a class post construction.
   Use this when dataclasses should be combined with HasParameters.

   :param target_class: The class to get the parameters from.
   :return: The updated class


.. py:class:: SequentialPlan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`LanguagePlan`


   Creates a plan which executes its children in sequential order


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

