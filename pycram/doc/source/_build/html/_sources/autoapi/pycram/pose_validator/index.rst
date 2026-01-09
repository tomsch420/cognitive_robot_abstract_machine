pycram.pose_validator
=====================

.. py:module:: pycram.pose_validator


Attributes
----------

.. autoapisummary::

   pycram.pose_validator.logger


Functions
---------

.. autoapisummary::

   pycram.pose_validator.visibility_validator
   pycram.pose_validator.reachability_validator
   pycram.pose_validator.pose_sequence_reachability_validator
   pycram.pose_validator.collision_check
   pycram.pose_validator.create_collision_matrix


Module Contents
---------------

.. py:data:: logger

.. py:function:: visibility_validator(robot: semantic_digital_twin.robots.abstract_robot.AbstractRobot, object_or_pose: typing_extensions.Union[semantic_digital_twin.world_description.world_entity.Body, pycram.datastructures.pose.PoseStamped], world: semantic_digital_twin.world.World) -> bool

   This method validates if the robot can see the target position from a given
   pose candidate. The target position can either be a position, in world coordinate
   system, or an object in the World. The validation is done by shooting a
   ray from the camera to the target position and checking that it does not collide
   with anything else.

   :param robot: The robot object for which this should be validated
   :param object_or_pose: The target position or object for which the pose candidate should be validated.
   :param world: The world in which the visibility should be validated.
   :return: True if the target is visible for the robot, None in any other case.


.. py:function:: reachability_validator(target_pose: pycram.datastructures.pose.PoseStamped, tip_link: semantic_digital_twin.world_description.world_entity.KinematicStructureEntity, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot, world: semantic_digital_twin.world.World, use_fullbody_ik: bool = False) -> bool

   Evaluates if a pose can be reached with the tip_link in the given world. This uses giskard motion state charts
   for testing.

   :param target_pose: The sequence of poses which the tip_link needs to reach
   :param tip_link: The tip link which should be used for reachability
   :param robot_view: The semantic annotation of the robot which should be evaluated for reachability
   :param world: The world in which the visibility should be validated.
   :param use_fullbody_ik: If true the base will be used in trying to reach the poses


.. py:function:: pose_sequence_reachability_validator(target_sequence: typing_extensions.List[pycram.datastructures.pose.PoseStamped], tip_link: semantic_digital_twin.world_description.world_entity.KinematicStructureEntity, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot, world: semantic_digital_twin.world.World, use_fullbody_ik: bool = False) -> bool

   Evaluates the pose sequence by executing the pose sequence with giskard motion state charts.

   :param target_sequence: The sequence of poses which the tip_link needs to reach
   :param tip_link: The tip link which should be used for reachability
   :param robot_view: The semantic annotation of the robot which should be evaluated for reachability
   :param world: The world in which the visibility should be validated.
   :param use_fullbody_ik: If true the base will be used in trying to reach the poses


.. py:function:: collision_check(robot: semantic_digital_twin.robots.abstract_robot.AbstractRobot, allowed_collision: typing_extensions.List[semantic_digital_twin.world_description.world_entity.Body], world: semantic_digital_twin.world.World) -> typing_extensions.List[semantic_digital_twin.collision_checking.collision_detector.Collision]

   This method checks if a given robot collides with any object within the world
   which it is not allowed to collide with.
   This is done checking iterating over every object within the world and checking
   if the robot collides with it. Careful the floor will be ignored.
   If there is a collision with an object that was not within the allowed collision
   list the function will raise a RobotInCollision exception.

   :param robot: The robot object in the (Bullet)World where it should be checked if it collides with something
   :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates to a list of links of which this object consists
   :param world: The world in which collision should be checked
   :raises: RobotInCollision if the robot collides with an object it is not allowed to collide with.


.. py:function:: create_collision_matrix(ignore_collision_with: typing_extensions.List[semantic_digital_twin.world_description.world_entity.Body], world: semantic_digital_twin.world.World, robot: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.List[semantic_digital_twin.collision_checking.collision_detector.CollisionCheck]

   CCreates a list of collision checks that should be performed

   :param ignore_collision_with: List of objects for which collision should be ignored
   :param world: The world in which the collision check should be performed
   :param robot: The robot for which the collision check should be performed
   :return: A list of collision checks that should be performed


