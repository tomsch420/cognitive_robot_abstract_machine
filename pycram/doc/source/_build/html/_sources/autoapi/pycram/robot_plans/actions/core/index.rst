pycram.robot_plans.actions.core
===============================

.. py:module:: pycram.robot_plans.actions.core


Submodules
----------

.. toctree::
   :maxdepth: 1

   /autoapi/pycram/robot_plans/actions/core/container/index
   /autoapi/pycram/robot_plans/actions/core/misc/index
   /autoapi/pycram/robot_plans/actions/core/navigation/index
   /autoapi/pycram/robot_plans/actions/core/pick_up/index
   /autoapi/pycram/robot_plans/actions/core/placing/index
   /autoapi/pycram/robot_plans/actions/core/robot_body/index


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.core.GraspingActionDescription
   pycram.robot_plans.actions.core.OpenActionDescription
   pycram.robot_plans.actions.core.CloseActionDescription
   pycram.robot_plans.actions.core.logger
   pycram.robot_plans.actions.core.ReachActionDescription
   pycram.robot_plans.actions.core.PickUpActionDescription
   pycram.robot_plans.actions.core.GraspingActionDescription
   pycram.robot_plans.actions.core.DetectActionDescription
   pycram.robot_plans.actions.core.NavigateActionDescription
   pycram.robot_plans.actions.core.LookAtActionDescription
   pycram.robot_plans.actions.core.ReachActionDescription
   pycram.robot_plans.actions.core.PlaceActionDescription
   pycram.robot_plans.actions.core.MoveTorsoActionDescription
   pycram.robot_plans.actions.core.SetGripperActionDescription
   pycram.robot_plans.actions.core.ParkArmsActionDescription
   pycram.robot_plans.actions.core.CarryActionDescription
   pycram.robot_plans.actions.core.right_park
   pycram.robot_plans.actions.core.left_park
   pycram.robot_plans.actions.core.both_park
   pycram.robot_plans.actions.core.left_gripper_open
   pycram.robot_plans.actions.core.left_gripper_close
   pycram.robot_plans.actions.core.right_gripper_open
   pycram.robot_plans.actions.core.right_gripper_close
   pycram.robot_plans.actions.core.torso_low
   pycram.robot_plans.actions.core.torso_mid
   pycram.robot_plans.actions.core.torso_high
   pycram.robot_plans.actions.core.left_arm
   pycram.robot_plans.actions.core.both_arm


Exceptions
----------

.. autoapisummary::

   pycram.robot_plans.actions.core.ContainerManipulationError
   pycram.robot_plans.actions.core.ObjectNotGraspedError
   pycram.robot_plans.actions.core.ObjectNotInGraspingArea
   pycram.robot_plans.actions.core.LookAtGoalNotReached
   pycram.robot_plans.actions.core.NavigationGoalNotReachedError
   pycram.robot_plans.actions.core.ObjectNotPlacedAtTargetLocation
   pycram.robot_plans.actions.core.ObjectStillInContact
   pycram.robot_plans.actions.core.TorsoGoalNotReached
   pycram.robot_plans.actions.core.ConfigurationNotReached


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.core.OpeningMotion
   pycram.robot_plans.actions.core.ClosingMotion
   pycram.robot_plans.actions.core.MoveGripperMotion
   pycram.robot_plans.actions.core.ActionConfig
   pycram.robot_plans.actions.core.Arms
   pycram.robot_plans.actions.core.GripperState
   pycram.robot_plans.actions.core.ContainerManipulationType
   pycram.robot_plans.actions.core.PartialDesignator
   pycram.robot_plans.actions.core.SequentialPlan
   pycram.robot_plans.actions.core.ActionDescription
   pycram.robot_plans.actions.core.OpenAction
   pycram.robot_plans.actions.core.CloseAction
   pycram.robot_plans.actions.core.MoveGripperMotion
   pycram.robot_plans.actions.core.MoveTCPMotion
   pycram.robot_plans.actions.core.ActionConfig
   pycram.robot_plans.actions.core.Arms
   pycram.robot_plans.actions.core.GripperState
   pycram.robot_plans.actions.core.MovementType
   pycram.robot_plans.actions.core.FindBodyInRegionMethod
   pycram.robot_plans.actions.core.GraspDescription
   pycram.robot_plans.actions.core.PartialDesignator
   pycram.robot_plans.actions.core.PoseStamped
   pycram.robot_plans.actions.core.SequentialPlan
   pycram.robot_plans.actions.core.RobotDescription
   pycram.robot_plans.actions.core.ViewManager
   pycram.robot_plans.actions.core.ActionDescription
   pycram.robot_plans.actions.core.ReachAction
   pycram.robot_plans.actions.core.PickUpAction
   pycram.robot_plans.actions.core.GraspingAction
   pycram.robot_plans.actions.core.PerceptionQuery
   pycram.robot_plans.actions.core.DetectionTechnique
   pycram.robot_plans.actions.core.DetectionState
   pycram.robot_plans.actions.core.PartialDesignator
   pycram.robot_plans.actions.core.ActionDescription
   pycram.robot_plans.actions.core.DetectAction
   pycram.robot_plans.actions.core.ActionDescription
   pycram.robot_plans.actions.core.LookingMotion
   pycram.robot_plans.actions.core.MoveMotion
   pycram.robot_plans.actions.core.ActionConfig
   pycram.robot_plans.actions.core.PartialDesignator
   pycram.robot_plans.actions.core.PoseStamped
   pycram.robot_plans.actions.core.SequentialPlan
   pycram.robot_plans.actions.core.PoseErrorChecker
   pycram.robot_plans.actions.core.NavigateAction
   pycram.robot_plans.actions.core.LookAtAction
   pycram.robot_plans.actions.core.ActionConfig
   pycram.robot_plans.actions.core.MoveTCPMotion
   pycram.robot_plans.actions.core.MoveGripperMotion
   pycram.robot_plans.actions.core.ReachMotion
   pycram.robot_plans.actions.core.Arms
   pycram.robot_plans.actions.core.GripperState
   pycram.robot_plans.actions.core.ApproachDirection
   pycram.robot_plans.actions.core.VerticalAlignment
   pycram.robot_plans.actions.core.GraspDescription
   pycram.robot_plans.actions.core.PartialDesignator
   pycram.robot_plans.actions.core.PoseStamped
   pycram.robot_plans.actions.core.SequentialPlan
   pycram.robot_plans.actions.core.ViewManager
   pycram.robot_plans.actions.core.ActionDescription
   pycram.robot_plans.actions.core.PoseErrorChecker
   pycram.robot_plans.actions.core.PlaceAction
   pycram.robot_plans.actions.core.AxisIdentifier
   pycram.robot_plans.actions.core.PartialDesignator
   pycram.robot_plans.actions.core.Vector3Stamped
   pycram.robot_plans.actions.core.SequentialPlan
   pycram.robot_plans.actions.core.RobotDescription
   pycram.robot_plans.actions.core.ActionDescription
   pycram.robot_plans.actions.core.MoveGripperMotion
   pycram.robot_plans.actions.core.MoveJointsMotion
   pycram.robot_plans.actions.core.MoveTorsoAction
   pycram.robot_plans.actions.core.SetGripperAction
   pycram.robot_plans.actions.core.ParkArmsAction
   pycram.robot_plans.actions.core.CarryAction
   pycram.robot_plans.actions.core.StaticJointState
   pycram.robot_plans.actions.core.Arms
   pycram.robot_plans.actions.core.GripperStateEnum
   pycram.robot_plans.actions.core.TorsoState
   pycram.robot_plans.actions.core.JointState
   pycram.robot_plans.actions.core.ArmState
   pycram.robot_plans.actions.core.GripperState
   pycram.robot_plans.actions.core.JointStateManager


Functions
---------

.. autoapisummary::

   pycram.robot_plans.actions.core.has_parameters
   pycram.robot_plans.actions.core.validate_close_open
   pycram.robot_plans.actions.core.check_opened
   pycram.robot_plans.actions.core.check_closed
   pycram.robot_plans.actions.core.has_parameters
   pycram.robot_plans.actions.core.translate_pose_along_local_axis
   pycram.robot_plans.actions.core.has_parameters
   pycram.robot_plans.actions.core.has_parameters
   pycram.robot_plans.actions.core.has_parameters
   pycram.robot_plans.actions.core.translate_pose_along_local_axis
   pycram.robot_plans.actions.core.has_parameters
   pycram.robot_plans.actions.core.create_multiple_joint_goal_validator


Package Contents
----------------

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



.. py:exception:: ContainerManipulationError(robot: Object, arms: typing_extensions.List[pycram.datastructures.enums.Arms], body: PhysicalBody, container_joint: Joint, manipulation_type: pycram.datastructures.enums.ContainerManipulationType, *args, **kwargs)

   Bases: :py:obj:`ManipulationLowLevelFailure`, :py:obj:`abc.ABC`


   Thrown when container manipulation fails.


   .. py:attribute:: container_joint
      :type:  Joint

      The joint of the container that should be manipulated.



   .. py:attribute:: manipulation_type
      :type:  pycram.datastructures.enums.ContainerManipulationType

      The type of manipulation that should be performed on the container.



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

.. py:data:: OpenActionDescription

.. py:data:: CloseActionDescription

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



.. py:exception:: ObjectNotGraspedError(obj: Object, robot: Object, arm: pycram.datastructures.enums.Arms, grasp=None, *args, **kwargs)

   Bases: :py:obj:`Grasping`


.. py:exception:: ObjectNotInGraspingArea(obj: Object, robot: Object, arm: pycram.datastructures.enums.Arms, grasp, *args, **kwargs)

   Bases: :py:obj:`ReachabilityFailure`


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

.. py:data:: PickUpActionDescription

.. py:data:: GraspingActionDescription

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



.. py:class:: DetectionState

   Bases: :py:obj:`int`, :py:obj:`enum.Enum`


   Enum for the state of the detection task.


   .. py:attribute:: START
      :value: 0



   .. py:attribute:: STOP
      :value: 1



   .. py:attribute:: PAUSE
      :value: 2



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



.. py:function:: has_parameters(target_class: T) -> T

   Insert parameters of a class post construction.
   Use this when dataclasses should be combined with HasParameters.

   :param target_class: The class to get the parameters from.
   :return: The updated class


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



.. py:data:: DetectActionDescription

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



.. py:function:: has_parameters(target_class: T) -> T

   Insert parameters of a class post construction.
   Use this when dataclasses should be combined with HasParameters.

   :param target_class: The class to get the parameters from.
   :return: The updated class


.. py:class:: SequentialPlan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`LanguagePlan`


   Creates a plan which executes its children in sequential order


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



.. py:data:: NavigateActionDescription

.. py:data:: LookAtActionDescription

.. py:data:: ReachActionDescription

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



.. py:exception:: ObjectNotPlacedAtTargetLocation(obj: Object, placing_pose: pycram.datastructures.pose.PoseStamped, robot: Object, arm: pycram.datastructures.enums.Arms, *args, **kwargs)

   Bases: :py:obj:`ObjectPlacingError`


   Thrown when the object was not placed at the target location.


.. py:exception:: ObjectStillInContact(obj: Object, contact_links: typing_extensions.List[Link], placing_pose: pycram.datastructures.pose.PoseStamped, robot: Object, arm: pycram.datastructures.enums.Arms, *args, **kwargs)

   Bases: :py:obj:`ObjectPlacingError`


   Thrown when the object is still in contact with the robot after placing.


   .. py:attribute:: contact_links
      :type:  typing_extensions.List[Link]

      The links of the robot that are still in contact with the object.



.. py:function:: has_parameters(target_class: T) -> T

   Insert parameters of a class post construction.
   Use this when dataclasses should be combined with HasParameters.

   :param target_class: The class to get the parameters from.
   :return: The updated class


.. py:class:: SequentialPlan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`LanguagePlan`


   Creates a plan which executes its children in sequential order


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



.. py:function:: translate_pose_along_local_axis(pose: pycram.datastructures.pose.PoseStamped, axis: Union[typing_extensions.List, numpy.ndarray], distance: float) -> pycram.datastructures.pose.PoseStamped

   Translate a pose along a given 3d vector (axis) by a given distance. The axis is given in the local coordinate
   frame of the pose. The axis is normalized and then scaled by the distance.

   :param pose: The pose that should be translated
   :param axis: The local axis along which the translation should be performed
   :param distance: The distance by which the pose should be translated

   :return: The translated pose


.. py:class:: PoseErrorChecker(acceptable_error: typing_extensions.Union[typing_extensions.Tuple[float], typing_extensions.Iterable[typing_extensions.Tuple[float]]] = (0.001, np.pi / 180), is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`ErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


   .. py:method:: _calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> typing_extensions.List[float]

      Calculate the error between two poses.

      :param value_1: The first pose.
      :param value_2: The second pose.



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



.. py:data:: PlaceActionDescription

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

.. py:data:: ParkArmsActionDescription

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



.. py:class:: GripperState

   Bases: :py:obj:`JointState`


   Represents the state of a gripper, such as open or closed.


   .. py:attribute:: gripper
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

