pycram.failures
===============

.. py:module:: pycram.failures


Exceptions
----------

.. autoapisummary::

   pycram.failures.PlanFailure
   pycram.failures.KnowledgeNotAvailable
   pycram.failures.NotALanguageExpression
   pycram.failures.FailureDiagnosis
   pycram.failures.LowLevelFailure
   pycram.failures.ActionlibActionTimeout
   pycram.failures.HighLevelFailure
   pycram.failures.SensorMonitoringCondition
   pycram.failures.DeliveringFailed
   pycram.failures.ManipulationLowLevelFailure
   pycram.failures.EnvironmentManipulationGoalNotReached
   pycram.failures.ContainerManipulationError
   pycram.failures.EnvironmentManipulationImpossible
   pycram.failures.EnvironmentUnreachable
   pycram.failures.FetchingFailed
   pycram.failures.GripperLowLevelFailure
   pycram.failures.GripperIsNotOpen
   pycram.failures.GripperClosedCompletely
   pycram.failures.GripperGoalNotReached
   pycram.failures.GripperOccupied
   pycram.failures.LookingHighLevelFailure
   pycram.failures.LookAtGoalNotReached
   pycram.failures.ManipulationGoalInCollision
   pycram.failures.ManipulationGoalNotReached
   pycram.failures.RobotInCollision
   pycram.failures.IKError
   pycram.failures.ManipulationPoseUnreachable
   pycram.failures.NavigationHighLevelFailure
   pycram.failures.NavigationGoalInCollision
   pycram.failures.NavigationLowLevelFailure
   pycram.failures.NavigationGoalNotReached
   pycram.failures.NavigationPoseUnreachable
   pycram.failures.ObjectNotVisible
   pycram.failures.ObjectNotFound
   pycram.failures.LinkNotFound
   pycram.failures.ObjectUndeliverable
   pycram.failures.ObjectPlacingError
   pycram.failures.ObjectStillInContact
   pycram.failures.ObjectNotPlacedAtTargetLocation
   pycram.failures.ObjectUnfetchable
   pycram.failures.ObjectUnreachable
   pycram.failures.PerceptionLowLevelFailure
   pycram.failures.PerceptionObjectNotFound
   pycram.failures.PerceptionObjectNotInWorld
   pycram.failures.SearchingFailed
   pycram.failures.TorsoLowLevelFailure
   pycram.failures.TorsoGoalNotReached
   pycram.failures.TorsoGoalUnreachable
   pycram.failures.Task
   pycram.failures.Grasping
   pycram.failures.ObjectNotGraspedError
   pycram.failures.Looking
   pycram.failures.ObjectPoseMissestimation
   pycram.failures.SuccessfulCompletion
   pycram.failures.LocomotorFailure
   pycram.failures.ArmFailure
   pycram.failures.ObjectLost
   pycram.failures.SensorFailure
   pycram.failures.IllPosedGoalFailure
   pycram.failures.CapabilityAbsenceFailure
   pycram.failures.ReachabilityFailure
   pycram.failures.ObjectNotInGraspingArea
   pycram.failures.TorsoFailure
   pycram.failures.ConfigurationNotReached
   pycram.failures.Timeout
   pycram.failures.EndEffectorFailure
   pycram.failures.ObjectUnavailable
   pycram.failures.SustainedFailure
   pycram.failures.ReasoningError
   pycram.failures.CollisionError
   pycram.failures.NavigationGoalNotReachedError
   pycram.failures.ToolPoseNotReachedError
   pycram.failures.MultiverseFailedAPIResponse
   pycram.failures.ProspectionObjectNotFound
   pycram.failures.ObjectAlreadyExists
   pycram.failures.ObjectDescriptionNotFound
   pycram.failures.WorldMismatchErrorBetweenAttachedObjects
   pycram.failures.ObjectFrameNotFoundError
   pycram.failures.MultiplePossibleTipLinks
   pycram.failures.UnsupportedFileExtension
   pycram.failures.ObjectDescriptionUndefined
   pycram.failures.UnsupportedJointType
   pycram.failures.LinkHasNoGeometry
   pycram.failures.LinkGeometryHasNoMesh


Module Contents
---------------

.. py:exception:: PlanFailure(*args, **kwargs)

   Bases: :py:obj:`Exception`


   Implementation of plan failures.


.. py:exception:: KnowledgeNotAvailable(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Thrown when a knowledge source can not provide the information for a query.


.. py:exception:: NotALanguageExpression(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: FailureDiagnosis(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: LowLevelFailure(*args, **kwargs)

   Bases: :py:obj:`FailureDiagnosis`


   Failure thrown by low-level modules: robot or projection PMs.


.. py:exception:: ActionlibActionTimeout(*args, **kwargs)

   Bases: :py:obj:`LowLevelFailure`


   Failure thrown by low-level modules: robot or projection PMs.


.. py:exception:: HighLevelFailure(*args, **kwargs)

   Bases: :py:obj:`FailureDiagnosis`


   Failure thrown by high-level modules, i.e. plans.


.. py:exception:: SensorMonitoringCondition(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Thrown when a sensor monitoring condition is met.


.. py:exception:: DeliveringFailed(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when delivering plan completely gives up.


.. py:exception:: ManipulationLowLevelFailure(robot: Object, arms: typing_extensions.List[pycram.datastructures.enums.Arms], body: PhysicalBody, *args, **kwargs)

   Bases: :py:obj:`LowLevelFailure`


   Thrown when a low-level, i.e. hardware related, failure is detected in a manipulation action.


   .. py:attribute:: robot
      :type:  Object

      The robot that the manipulation action was performed with.



   .. py:attribute:: arm
      :type:  typing_extensions.List[pycram.datastructures.enums.Arms]

      The arm(s) that the manipulation action was performed with.



   .. py:attribute:: body
      :type:  PhysicalBody

      The body that the manipulation action was performed on.



.. py:exception:: EnvironmentManipulationGoalNotReached(*args, **kwargs)

   Bases: :py:obj:`ManipulationLowLevelFailure`


   Thrown when door / drawer opening / closing goal is still not reached.


.. py:exception:: ContainerManipulationError(robot: Object, arms: typing_extensions.List[pycram.datastructures.enums.Arms], body: PhysicalBody, container_joint: Joint, manipulation_type: pycram.datastructures.enums.ContainerManipulationType, *args, **kwargs)

   Bases: :py:obj:`ManipulationLowLevelFailure`, :py:obj:`abc.ABC`


   Thrown when container manipulation fails.


   .. py:attribute:: container_joint
      :type:  Joint

      The joint of the container that should be manipulated.



   .. py:attribute:: manipulation_type
      :type:  pycram.datastructures.enums.ContainerManipulationType

      The type of manipulation that should be performed on the container.



.. py:exception:: EnvironmentManipulationImpossible(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when environment manipulation cannot be achieved.


.. py:exception:: EnvironmentUnreachable(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when environment manipulation in collision or unreachable.


.. py:exception:: FetchingFailed(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when fetching plan completely gives up.


.. py:exception:: GripperLowLevelFailure(robot: Object, arm: pycram.datastructures.enums.Arms, *args, **kwargs)

   Bases: :py:obj:`LowLevelFailure`


   Thrown when a failure involving the gripper hardware occurs.


   .. py:attribute:: robot
      :type:  Object

      The robot that the gripper belongs to.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that the gripper belongs to.



.. py:exception:: GripperIsNotOpen(robot: Object, arm: pycram.datastructures.enums.Arms, *args, **kwargs)

   Bases: :py:obj:`GripperLowLevelFailure`


   Thrown when the gripper is not open when it should be open.


.. py:exception:: GripperClosedCompletely(*args, **kwargs)

   Bases: :py:obj:`GripperLowLevelFailure`


   Thrown when the gripper closed completely, despite not being expected to do so (e.g. because it should have
   grasped something).


.. py:exception:: GripperGoalNotReached(*args, **kwargs)

   Bases: :py:obj:`GripperLowLevelFailure`


   Thrown when the gripper does not reach its goal.


.. py:exception:: GripperOccupied(*args, **kwargs)

   Bases: :py:obj:`GripperLowLevelFailure`


   Thrown when the gripper is occupied by some object.


.. py:exception:: LookingHighLevelFailure(robot: Object, target: pycram.datastructures.pose.PoseStamped, *args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   High-level failure produced when looking for an object, i.e. it is not a hardware issue but one relating to
   the looking task, its parameters, and how they relate to the environment.


   .. py:attribute:: robot
      :type:  Object

      The robot that performed the look at action.



   .. py:attribute:: target
      :type:  pycram.datastructures.pose.PoseStamped

      The target pose that the robot was supposed to look at.



.. py:exception:: LookAtGoalNotReached(robot: Object, target: pycram.datastructures.pose.PoseStamped, *args, **kwargs)

   Bases: :py:obj:`LookingHighLevelFailure`


   Thrown when the look at goal is not reached.


.. py:exception:: ManipulationGoalInCollision(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when executing a manipulation action results in a collision.


.. py:exception:: ManipulationGoalNotReached(*args, **kwargs)

   Bases: :py:obj:`ManipulationLowLevelFailure`


   Thrown when after executing the action, goal is still not reached.


.. py:exception:: RobotInCollision(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Thrown when the robot is in collision with the environment.


.. py:exception:: IKError(pose, base_frame, tip_frame, robot_pose, joint_positions=None)

   Bases: :py:obj:`PlanFailure`


   Thrown when no inverse kinematics solution could be found


   .. py:attribute:: pose
      :type:  pycram.datastructures.pose.PoseStamped

      The pose for which no IK solution could be found.



   .. py:attribute:: base_frame
      :type:  str

      The base frame in which the pose was given.



   .. py:attribute:: tip_frame
      :type:  str

      The robot tip frame that should reach the pose.



   .. py:attribute:: robot_pose


   .. py:attribute:: message


.. py:exception:: ManipulationPoseUnreachable(*args, **kwargs)

   Bases: :py:obj:`ManipulationLowLevelFailure`


   Thrown when no IK solution can be found.


.. py:exception:: NavigationHighLevelFailure(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   High-level failure produced while navigating the robot, i.e. it is not a hardware issue but one relating to
   the navigation task, its parameters, and how they relate to the environment.


.. py:exception:: NavigationGoalInCollision(*args, **kwargs)

   Bases: :py:obj:`NavigationHighLevelFailure`


   Navigation goal cannot be reached because the goal itself is already occupied by some other object.


.. py:exception:: NavigationLowLevelFailure(*args, **kwargs)

   Bases: :py:obj:`LowLevelFailure`


   Low-level failure produced while navigating the robot, i.e. some kind of hardware issue.


.. py:exception:: NavigationGoalNotReached(*args, **kwargs)

   Bases: :py:obj:`NavigationLowLevelFailure`


   Thrown when the base moved as a result of the navigation action but the goal was not reached.


.. py:exception:: NavigationPoseUnreachable(*args, **kwargs)

   Bases: :py:obj:`NavigationLowLevelFailure`


   Thrown when the goal pose for navigation is computed to be unreachable.


.. py:exception:: ObjectNotVisible(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when the robot cannot see an object of a given description in its surroundings.


.. py:exception:: ObjectNotFound(object_name: str)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when the robot cannot find an object of a given description in its surroundings.


.. py:exception:: LinkNotFound(link_name: str, of_object: typing_extensions.Optional[str] = None)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when the robot cannot find a link of a given description


   .. py:attribute:: link_name
      :type:  str

      The name of the link that couldn't be found



   .. py:attribute:: of_object
      :type:  typing_extensions.Optional[str]

      The name of the object that the link should belong to



.. py:exception:: ObjectUndeliverable(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when no base positioning can assure a reachable pose to place the object from.


.. py:exception:: ObjectPlacingError(obj: Object, placing_pose: pycram.datastructures.pose.PoseStamped, robot: Object, arm: pycram.datastructures.enums.Arms, *args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when the placing of the object fails.


   .. py:attribute:: obj
      :type:  Object

      The object that should be placed.



   .. py:attribute:: placing_pose
      :type:  pycram.datastructures.pose.PoseStamped

      The target pose at which the object should be placed.



   .. py:attribute:: robot
      :type:  Object

      The robot that placed the object.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The robot arm used to place the object.



.. py:exception:: ObjectStillInContact(obj: Object, contact_links: typing_extensions.List[Link], placing_pose: pycram.datastructures.pose.PoseStamped, robot: Object, arm: pycram.datastructures.enums.Arms, *args, **kwargs)

   Bases: :py:obj:`ObjectPlacingError`


   Thrown when the object is still in contact with the robot after placing.


   .. py:attribute:: contact_links
      :type:  typing_extensions.List[Link]

      The links of the robot that are still in contact with the object.



.. py:exception:: ObjectNotPlacedAtTargetLocation(obj: Object, placing_pose: pycram.datastructures.pose.PoseStamped, robot: Object, arm: pycram.datastructures.enums.Arms, *args, **kwargs)

   Bases: :py:obj:`ObjectPlacingError`


   Thrown when the object was not placed at the target location.


.. py:exception:: ObjectUnfetchable(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when no base positioning can assure a reachable pose to grasp the object from.


.. py:exception:: ObjectUnreachable(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when no IK found for particular base pose.


.. py:exception:: PerceptionLowLevelFailure(object_description: pycram.designator.ObjectDesignatorDescription, technique: pycram.datastructures.enums.DetectionTechnique, region: typing_extensions.Optional[pycram.designators.location_designator.Location] = None, *args, **kwargs)

   Bases: :py:obj:`LowLevelFailure`


   Low-level failure produced while perceiving, i.e. some kind of hardware issue.


   .. py:attribute:: object_description
      :type:  pycram.designator.ObjectDesignatorDescription

      The object description that was used to search for the object.



   .. py:attribute:: technique
      :type:  pycram.datastructures.enums.DetectionTechnique

      The detection technique that was used to search for the object.



   .. py:attribute:: region
      :type:  typing_extensions.Optional[pycram.designators.location_designator.Location]
      :value: None


      The suggested region in which the object was searched.



.. py:exception:: PerceptionObjectNotFound(obj_type: typing_extensions.Type[PhysicalObject], technique: pycram.datastructures.enums.DetectionTechnique, region: pycram.datastructures.pose.PoseStamped, *args, **kwargs)

   Bases: :py:obj:`PerceptionLowLevelFailure`


   Thrown when an attempt to find an object by perception fails -- and this can still be interpreted as the robot
   not looking in the right direction, as opposed to the object being absent.


.. py:exception:: PerceptionObjectNotInWorld(*args, **kwargs)

   Bases: :py:obj:`PerceptionLowLevelFailure`


   Thrown when an attempt to find an object by perception fails -- and this is because the object can be assumed
   absent or perhaps is known absent because of the setup of a simulation.


.. py:exception:: SearchingFailed(*args, **kwargs)

   Bases: :py:obj:`HighLevelFailure`


   Thrown when searching plan completely gives up.


.. py:exception:: TorsoLowLevelFailure(goal_validator: typing_extensions.Optional[pycram.validation.goal_validator.MultiJointPositionGoalValidator] = None, *args, **kwargs)

   Bases: :py:obj:`LowLevelFailure`


   Low-level failure produced while moving the torso, i.e. some kind of hardware issue.


   .. py:attribute:: goal_validator
      :type:  typing_extensions.Optional[pycram.validation.goal_validator.MultiJointPositionGoalValidator]
      :value: None


      The goal validator that was used to check if the goal was reached.



.. py:exception:: TorsoGoalNotReached(goal_validator: typing_extensions.Optional[pycram.validation.goal_validator.MultiJointPositionGoalValidator] = None, *args, **kwargs)

   Bases: :py:obj:`TorsoLowLevelFailure`


   Thrown when the torso moved as a result of a torso action but the goal was not reached.


.. py:exception:: TorsoGoalUnreachable(goal_validator: typing_extensions.Optional[pycram.validation.goal_validator.MultiJointPositionGoalValidator] = None, *args, **kwargs)

   Bases: :py:obj:`TorsoLowLevelFailure`


   Thrown when the goal for the torso is computed to be unreachable.


.. py:exception:: Task(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: Grasping(obj: Object, robot: Object, arm: pycram.datastructures.enums.Arms, grasp: typing_extensions.Optional[pycram.datastructures.enums.Grasp] = None, *args, **kwargs)

   Bases: :py:obj:`Task`


   .. py:attribute:: obj
      :type:  Object

      The object to be grasped.



   .. py:attribute:: robot
      :type:  Object

      The robot that should grasp the object.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm used to grasp the object.



   .. py:attribute:: grasp
      :type:  typing_extensions.Optional[pycram.datastructures.enums.Grasp]

      The grasp type used to grasp the object.



.. py:exception:: ObjectNotGraspedError(obj: Object, robot: Object, arm: pycram.datastructures.enums.Arms, grasp=None, *args, **kwargs)

   Bases: :py:obj:`Grasping`


.. py:exception:: Looking(*args, **kwargs)

   Bases: :py:obj:`Task`


.. py:exception:: ObjectPoseMissestimation(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: SuccessfulCompletion(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: LocomotorFailure(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: ArmFailure(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: ObjectLost(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: SensorFailure(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: IllPosedGoalFailure(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: CapabilityAbsenceFailure(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: ReachabilityFailure(obj: Object, robot: Object, arm: pycram.datastructures.enums.Arms, grasp: pycram.datastructures.enums.Grasp, *args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


   .. py:attribute:: obj
      :type:  Object

      The object that should be reachable.



   .. py:attribute:: robot
      :type:  Object

      The robot that should reach the object.



   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms

      The arm that should reach the object.



   .. py:attribute:: grasp
      :type:  typing_extensions.Optional[pycram.datastructures.enums.Grasp]
      :value: None


      The grasp/gripper orientation that should be used to reach the object.



.. py:exception:: ObjectNotInGraspingArea(obj: Object, robot: Object, arm: pycram.datastructures.enums.Arms, grasp, *args, **kwargs)

   Bases: :py:obj:`ReachabilityFailure`


.. py:exception:: TorsoFailure(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: ConfigurationNotReached(goal_validator: pycram.validation.goal_validator.MultiJointPositionGoalValidator, configuration_type: pycram.datastructures.enums.StaticJointState, *args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


   .. py:attribute:: goal_validator
      :type:  pycram.validation.goal_validator.MultiJointPositionGoalValidator

      The goal validator that was used to check if the goal was reached.



   .. py:attribute:: configuration_type
      :type:  pycram.datastructures.enums.StaticJointState

      The configuration type that should be reached.



.. py:exception:: Timeout(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: EndEffectorFailure(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: ObjectUnavailable(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: SustainedFailure(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: ReasoningError(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: CollisionError(*args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Implementation of plan failures.


.. py:exception:: NavigationGoalNotReachedError(current_pose: pycram.datastructures.pose.PoseStamped, goal_pose: pycram.datastructures.pose.PoseStamped, *args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Thrown when the navigation goal is not reached.


   .. py:attribute:: current_pose
      :type:  pycram.datastructures.pose.PoseStamped

      The current pose of the robot.



   .. py:attribute:: goal_pose
      :type:  pycram.datastructures.pose.PoseStamped

      The goal pose of the robot.



.. py:exception:: ToolPoseNotReachedError(current_pose: pycram.datastructures.pose.PoseStamped, goal_pose: pycram.datastructures.pose.PoseStamped, *args, **kwargs)

   Bases: :py:obj:`PlanFailure`


   Thrown when the tool pose is not reached.


   .. py:attribute:: current_pose
      :type:  pycram.datastructures.pose.PoseStamped

      The current pose of the tool.



   .. py:attribute:: goal_pose
      :type:  pycram.datastructures.pose.PoseStamped

      The goal pose of the tool.



.. py:exception:: MultiverseFailedAPIResponse(api_response: typing_extensions.List[str], api_name: pycram.datastructures.enums.MultiverseAPIName, *args, **kwargs)

   Bases: :py:obj:`Exception`


   Exception raised when a Multiverse API call fails.


   .. py:attribute:: api_response
      :type:  typing_extensions.List[str]

      The response of the API call that failed.



   .. py:attribute:: api_name
      :type:  pycram.datastructures.enums.MultiverseAPIName

      The name of the API that failed.



.. py:exception:: ProspectionObjectNotFound(obj: Object)

   Bases: :py:obj:`KeyError`


   Exception raised when an object was not found in the prospection world.


   .. py:attribute:: obj
      :type:  Object

      The object that was not found in the prospection world.



.. py:exception:: ObjectAlreadyExists(obj: Object)

   Bases: :py:obj:`Exception`


   Exception raised when an object with the same name already exists in the world.


   .. py:attribute:: obj
      :type:  Object

      The object that already exists in the world.



.. py:exception:: ObjectDescriptionNotFound(object_name: str, path: str, extension: str)

   Bases: :py:obj:`KeyError`


   Exception raised when the description of an object was not found.


   .. py:attribute:: object_name
      :type:  str

      The name of the object whose description was not found.



   .. py:attribute:: path
      :type:  str

      The path of the object whose description was not found.



   .. py:attribute:: extension
      :type:  str

      The description extension of the object whose description was not found.



.. py:exception:: WorldMismatchErrorBetweenAttachedObjects(obj_1: Object, obj_2: Object)

   Bases: :py:obj:`Exception`


   Exception raised when two objects that are attached to each other have a mismatch in the world they belong to.


   .. py:attribute:: obj_1
      :type:  Object

      The first object that has a mismatch in the world.



   .. py:attribute:: obj_2
      :type:  Object

      The second object that has a mismatch in the world.



.. py:exception:: ObjectFrameNotFoundError(frame_name: str)

   Bases: :py:obj:`KeyError`


   Exception raised when a tf frame of an object is not found.


   .. py:attribute:: frame_name
      :type:  str

      The name of the frame that was not found.



.. py:exception:: MultiplePossibleTipLinks(object_name: str, start_link: str, tip_links: typing_extensions.List[str])

   Bases: :py:obj:`Exception`


   Exception raised when multiple tip links are found for an object.


   .. py:attribute:: object_name
      :type:  str

      The name of the object that has multiple tip links.



   .. py:attribute:: start_link
      :type:  str

      The start link of the object that has multiple tip links.



   .. py:attribute:: tip_links
      :type:  typing_extensions.List[str]

      The list of tip links that are found for the object.



.. py:exception:: UnsupportedFileExtension(object_name: str, path: str)

   Bases: :py:obj:`Exception`


   Exception raised when an object mesh/description has an unsupported file extension.


   .. py:attribute:: object_name
      :type:  str

      The name of the object that has an unsupported file extension.



   .. py:attribute:: path
      :type:  str

      The path of the object description/mesh that has an unsupported file extension.



   .. py:attribute:: extension
      :type:  str

      The unsupported file extension of the object description/mesh.



.. py:exception:: ObjectDescriptionUndefined(object_name: str)

   Bases: :py:obj:`Exception`


   Exception raised when the given object description type is not defined or couldn't be resolved to a known type.


   .. py:attribute:: object_name
      :type:  str

      The name of the object that has an undefined description.



.. py:exception:: UnsupportedJointType(joint_type: pycram.datastructures.enums.JointType)

   Bases: :py:obj:`Exception`


   Exception raised when an unsupported joint type is used.


   .. py:attribute:: joint_type
      :type:  pycram.datastructures.enums.JointType

      The unsupported joint type that was used.



.. py:exception:: LinkHasNoGeometry(link_name: str)

   Bases: :py:obj:`Exception`


   Exception raised when a link has no geometry (i.e. no visual or collision elements).


   .. py:attribute:: link_name
      :type:  str

      The name of the link that has no geometry.



.. py:exception:: LinkGeometryHasNoMesh(link_name: str, geometry_type: str)

   Bases: :py:obj:`Exception`


   Exception raised when a link geometry has no mesh or is not of type mesh.


   .. py:attribute:: link_name
      :type:  str

      The name of the link that has no mesh.



   .. py:attribute:: geometry_type
      :type:  str

      The type of the link geometry.



