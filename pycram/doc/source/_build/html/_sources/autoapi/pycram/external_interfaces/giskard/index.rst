pycram.external_interfaces.giskard
==================================

.. py:module:: pycram.external_interfaces.giskard


Attributes
----------

.. autoapisummary::

   pycram.external_interfaces.giskard.logger
   pycram.external_interfaces.giskard.giskard_wrapper
   pycram.external_interfaces.giskard.is_init
   pycram.external_interfaces.giskard.giskard_lock
   pycram.external_interfaces.giskard.giskard_rlock
   pycram.external_interfaces.giskard.par_threads


Functions
---------

.. autoapisummary::

   pycram.external_interfaces.giskard.thread_safe
   pycram.external_interfaces.giskard.init_giskard_interface
   pycram.external_interfaces.giskard.achieve_joint_goal
   pycram.external_interfaces.giskard.set_joint_goal
   pycram.external_interfaces.giskard.achieve_cartesian_goal
   pycram.external_interfaces.giskard.achieve_straight_cartesian_goal
   pycram.external_interfaces.giskard.achieve_translation_goal
   pycram.external_interfaces.giskard.achieve_straight_translation_goal
   pycram.external_interfaces.giskard.achieve_rotation_goal
   pycram.external_interfaces.giskard.achieve_align_planes_goal
   pycram.external_interfaces.giskard.achieve_open_container_goal
   pycram.external_interfaces.giskard.achieve_close_container_goal
   pycram.external_interfaces.giskard.achieve_cartesian_waypoints_goal
   pycram.external_interfaces.giskard.projection_cartesian_goal
   pycram.external_interfaces.giskard.projection_cartesian_goal_with_approach
   pycram.external_interfaces.giskard.projection_joint_goal
   pycram.external_interfaces.giskard.allow_gripper_collision
   pycram.external_interfaces.giskard.allow_all_collision
   pycram.external_interfaces.giskard.get_gripper_group_names
   pycram.external_interfaces.giskard.add_gripper_groups
   pycram.external_interfaces.giskard.avoid_all_collisions
   pycram.external_interfaces.giskard.allow_self_collision
   pycram.external_interfaces.giskard.avoid_collisions
   pycram.external_interfaces.giskard.make_point_stamped
   pycram.external_interfaces.giskard.make_quaternion_stamped
   pycram.external_interfaces.giskard.make_vector_stamped
   pycram.external_interfaces.giskard.set_straight_cart_goal
   pycram.external_interfaces.giskard.set_cart_goal
   pycram.external_interfaces.giskard.execute


Module Contents
---------------

.. py:data:: logger

.. py:data:: giskard_wrapper
   :value: None


.. py:data:: is_init
   :value: False


.. py:data:: giskard_lock

.. py:data:: giskard_rlock

.. py:data:: par_threads

.. py:function:: thread_safe(func: typing_extensions.Callable) -> typing_extensions.Callable

   Adds thread safety to a function via a decorator. This uses the giskard_lock

   :param func: Function that should be thread safe
   :return: A function with thread safety


.. py:function:: init_giskard_interface(func: typing_extensions.Callable) -> typing_extensions.Callable

   Checks if the ROS messages are available and if giskard is running, if that is the case the interface will be
   initialized.

   :param func: Function this decorator should be wrapping
   :return: A callable function which initializes the interface and then calls the wrapped function


.. py:function:: achieve_joint_goal(goal_poses: typing_extensions.Dict[str, float])

   Takes a dictionary of joint position that should be achieved, the keys in the dictionary are the joint names and
   values are the goal joint positions.

   :param goal_poses: Dictionary with joint names and position goals
   :return: MoveResult message for this goal


.. py:function:: set_joint_goal(goal_poses: typing_extensions.Dict[str, float]) -> None

   Takes a dictionary of joint position that should be achieved, the keys in the dictionary are the joint names and
   values are the goal joint positions.

   :param goal_poses: Dictionary with joint names and position goals


.. py:function:: achieve_cartesian_goal(goal_pose: pycram.datastructures.pose.PoseStamped, tip_link: str, root_link: str, position_threshold: float = 0.02, orientation_threshold: float = 0.02, use_monitor: bool = True, grippers_that_can_collide: typing_extensions.Optional[pycram.datastructures.enums.Arms] = None)

   Takes a cartesian position and tries to move the tip_link to this position using the chain defined by
   tip_link and root_link.

   :param goal_pose: The position which should be achieved with tip_link
   :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
   :param root_link: The starting link of the chain which should be used to achieve this goal
   :param position_threshold: Position distance at which the goal is successfully reached
   :param orientation_threshold: Orientation distance at which the goal is successfully reached
   :param use_monitor: Whether to use a monitor for this goal or not.
   :param grippers_that_can_collide: The gripper(s) that should be allowed to collide.
   :return: MoveResult message for this goal


.. py:function:: achieve_straight_cartesian_goal(goal_pose: pycram.datastructures.pose.PoseStamped, tip_link: str, root_link: str, grippers_that_can_collide: typing_extensions.Optional[pycram.datastructures.enums.Arms] = None)

   Takes a cartesian position and tries to move the tip_link to this position in a straight line, using the chain
   defined by tip_link and root_link.

   :param goal_pose: The position which should be achieved with tip_link
   :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
   :param root_link: The starting link of the chain which should be used to achieve this goal
   :param grippers_that_can_collide: The gripper(s) that should be allowed to collide.
   :return: MoveResult message for this goal


.. py:function:: achieve_translation_goal(goal_point: typing_extensions.List[float], tip_link: str, root_link: str)

   Tries to move the tip_link to the position defined by goal_point using the chain defined by root_link and
   tip_link. Since goal_point only defines the position but no rotation, rotation is not taken into account.

   :param goal_point: The goal position of the tip_link
   :param tip_link: The link which should be moved to goal_point as well as the end of the used chain
   :param root_link: The start link of the chain
   :return: MoveResult message for this goal


.. py:function:: achieve_straight_translation_goal(goal_point: typing_extensions.List[float], tip_link: str, root_link: str)

   Tries to move the tip_link to the position defined by goal_point in a straight line, using the chain defined by
   root_link and tip_link. Since goal_point only defines the position but no rotation, rotation is not taken into account.

   :param goal_point: The goal position of the tip_link
   :param tip_link: The link which should be moved to goal_point as well as the end of the used chain
   :param root_link: The start link of the chain
   :return: MoveResult message for this goal


.. py:function:: achieve_rotation_goal(quat: typing_extensions.List[float], tip_link: str, root_link: str)

   Tries to bring the tip link into the rotation defined by quat using the chain defined by root_link and
   tip_link.

   :param quat: The rotation that should be achieved, given as a quaternion
   :param tip_link: The link that should be in the rotation defined by quat
   :param root_link: The start link of the chain
   :return: MoveResult message for this goal


.. py:function:: achieve_align_planes_goal(goal_normal: typing_extensions.List[float], tip_link: str, tip_normal: typing_extensions.List[float], root_link: str)

   Tries to align the plane defined by tip normal with goal_normal using the chain between root_link and
   tip_link.

   :param goal_normal: The goal plane, given as a list of XYZ
   :param tip_link: The end link of the chain that should be used.
   :param tip_normal: The plane that should be aligned with goal_normal, given as a list of XYZ
   :param root_link: The starting link of the chain that should be used.
   :return: MoveResult message for this goal


.. py:function:: achieve_open_container_goal(tip_link: str, environment_link: str)

   Tries to open a container in an environment, this only works if the container was added as a URDF. This goal assumes
   that the handle was already grasped. Can only handle container with 1 DOF

   :param tip_link: The End effector that should open the container
   :param environment_link: The name of the handle for this container.
   :return: MoveResult message for this goal


.. py:function:: achieve_close_container_goal(tip_link: str, environment_link: str)

   Tries to close a container, this only works if the container was added as a URDF. Assumes that the handle of the
   container was already grasped. Can only handle container with 1 DOF.

   :param tip_link: Link name that should be used to close the container.
   :param environment_link: Name of the handle
   :return: MoveResult message for this goal


.. py:function:: achieve_cartesian_waypoints_goal(waypoints: typing_extensions.List[pycram.datastructures.pose.PoseStamped], tip_link: str, root_link: str, enforce_final_orientation: bool = True)

   Tries to achieve each waypoint in the given sequence of waypoints.
   If :param enforce_final_orientation is False, each waypoint needs a corresponding orientation. If it is True only
   the last waypoint needs to have an orientation.

   :param waypoints: The sequence of waypoints as poses to achieve.
   :param tip_link: The endeffector link of the chain that should be used.
   :param root_link: The root link of the chain that should be used.
   :param enforce_final_orientation: If true, only achieve the orientation of the last waypoint. If false, achieve the orientation of each waypoint.
   :return: MoveResult message for this goal


.. py:function:: projection_cartesian_goal(goal_pose: pycram.datastructures.pose.PoseStamped, tip_link: str, root_link: str)

   Tries to move the tip_link to the position defined by goal_pose using the chain defined by tip_link and root_link.
   The goal_pose is projected to the closest point on the robot's workspace.

   :param goal_pose: The position which should be achieved with tip_link
   :param tip_link: The end link of the chain as well as the link which should achieve the goal_pose
   :param root_link: The starting link of the chain which should be used to achieve this goal
   :return: MoveResult message for this goal


.. py:function:: projection_cartesian_goal_with_approach(approach_pose: pycram.datastructures.pose.PoseStamped, goal_pose: pycram.datastructures.pose.PoseStamped, tip_link: str, root_link: str, robot_base_link: str)

   Tries to achieve the goal_pose using the chain defined by tip_link and root_link. The approach_pose is used to drive
   the robot to a pose close the actual goal pose, the robot_base_link is used to define the base link of the robot.

   :param approach_pose: Pose near the goal_pose
   :param goal_pose: Pose to which the tip_link should be moved
   :param tip_link: The link which should be moved to goal_pose, usually the tool frame
   :param root_link: The start of the link chain which should be used for planning
   :param robot_base_link: The base link of the robot
   :return: A trajectory calculated to move the tip_link to the goal_pose


.. py:function:: projection_joint_goal(goal_poses: typing_extensions.Dict[str, float], allow_collisions: bool = False)

   Tries to achieve the joint goal defined by goal_poses, the goal_poses are projected to the closest point on the
   robot's workspace.

   :param goal_poses: Dictionary with joint names and position goals
   :param allow_collisions: If all collisions should be allowed for this goal
   :return: MoveResult message for this goal


.. py:function:: allow_gripper_collision(gripper: pycram.datastructures.enums.Arms, at_goal: bool = False) -> None

   Allows the specified gripper to collide with anything.

   :param gripper: The gripper which can collide, either 'Arms.RIGHT', 'Arms.LEFT' or 'Arms.BOTH'
   :param at_goal: If the collision should be allowed only for this motion goal.


.. py:function:: allow_all_collision()

.. py:function:: get_gripper_group_names() -> typing_extensions.List[str]

   :return: The list of groups that are registered in giskard which have 'gripper' in their name.


.. py:function:: add_gripper_groups() -> None

   Add the gripper links as a group for collision avoidance.

   :return: Response of the RegisterGroup Service


.. py:function:: avoid_all_collisions() -> None

   Will avoid all collision for the next goal.


.. py:function:: allow_self_collision() -> None

   Will allow the robot collision with itself.


.. py:function:: avoid_collisions(object1: semantic_digital_twin.world_description.world_entity.Body, object2: semantic_digital_twin.world_description.world_entity.Body) -> None

   Will avoid collision between the two objects for the next goal.

   :param object1: The first World Object
   :param object2: The second World Object


.. py:function:: make_point_stamped(point: typing_extensions.List[float]) -> geometry_msgs.msg.PointStamped

   Creates a PointStamped message for the given position in world coordinate frame.

   :param point: XYZ coordinates of the point
   :return: A PointStamped message


.. py:function:: make_quaternion_stamped(quaternion: typing_extensions.List[float]) -> geometry_msgs.msg.QuaternionStamped

   Creates a QuaternionStamped message for the given quaternion.

   :param quaternion: The quaternion as a list of xyzw
   :return: A QuaternionStamped message


.. py:function:: make_vector_stamped(vector: typing_extensions.List[float]) -> geometry_msgs.msg.Vector3Stamped

   Creates a Vector3Stamped message, this is similar to PointStamped but represents a vector instead of a point.

   :param vector: The vector given as xyz in world frame
   :return: A Vector3Stamped message


.. py:function:: set_straight_cart_goal(goal_pose: pycram.datastructures.pose.PoseStamped, tip_link: str, root_link: str, tip_group: typing_extensions.Optional[str] = '', root_group: typing_extensions.Optional[str] = '', reference_linear_velocity: typing_extensions.Optional[float] = None, reference_angular_velocity: typing_extensions.Optional[float] = None, weight: typing_extensions.Optional[float] = None, **kwargs)

.. py:function:: set_cart_goal(goal_pose: pycram.datastructures.pose.PoseStamped, tip_link: str, root_link: str, tip_group: typing_extensions.Optional[str] = '', root_group: typing_extensions.Optional[str] = '', reference_linear_velocity: typing_extensions.Optional[float] = None, reference_angular_velocity: typing_extensions.Optional[float] = None, weight: typing_extensions.Optional[float] = None, add_monitor: bool = True, **kwargs)

.. py:function:: execute(add_default=True)

