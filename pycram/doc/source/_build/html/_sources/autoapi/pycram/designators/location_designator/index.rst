pycram.designators.location_designator
======================================

.. py:module:: pycram.designators.location_designator


Attributes
----------

.. autoapisummary::

   pycram.designators.location_designator.logger


Classes
-------

.. autoapisummary::

   pycram.designators.location_designator.Location
   pycram.designators.location_designator.CostmapLocation
   pycram.designators.location_designator.AccessingLocation
   pycram.designators.location_designator.SemanticCostmapLocation
   pycram.designators.location_designator.ProbabilisticSemanticLocation
   pycram.designators.location_designator.ProbabilisticCostmapLocation
   pycram.designators.location_designator.GiskardLocation


Functions
---------

.. autoapisummary::

   pycram.designators.location_designator._create_target_sequence


Module Contents
---------------

.. py:data:: logger

.. py:class:: Location(pose: pycram.datastructures.pose.PoseStamped)

   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`


   Default location designator which only wraps a pose.


   .. py:attribute:: pose
      :type:  pycram.datastructures.pose.PoseStamped


   .. py:method:: ground() -> pycram.datastructures.pose.PoseStamped

      Default specialized_designators which returns a resolved designator which contains the pose given in init.

      :return: A resolved designator



.. py:function:: _create_target_sequence(grasp_description: pycram.datastructures.grasp.GraspDescription, target: typing_extensions.Union[pycram.datastructures.pose.PoseStamped, semantic_digital_twin.world_description.world_entity.Body], robot: semantic_digital_twin.robots.abstract_robot.AbstractRobot, object_in_hand: semantic_digital_twin.world_description.world_entity.Body, reachable_arm: pycram.datastructures.enums.Arms, world: semantic_digital_twin.world.World, rotation_agnostic: bool = False) -> typing_extensions.List[pycram.datastructures.pose.PoseStamped]

   Creates the sequence of poses that need to be reachable for the robot to grasp the target.
   For pickup this would be retract pose, target pose and lift pose.
   For place this would be lift pose, target pose and retract pose.


   :param grasp_description: Grasp description to be used for grasping
   :param target: The target of reachability, either a pose or an object
   :param robot: The robot that should be checked for reachability
   :param object_in_hand: An object that is held if any
   :param reachable_arm: The arm which should be checked for reachability
   :return: A list of poses that need to be reachable in this order


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



.. py:class:: AccessingLocation(handle: typing_extensions.Union[semantic_digital_twin.world_description.world_entity.Body, typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body]], robot_desig: typing_extensions.Union[semantic_digital_twin.robots.abstract_robot.AbstractRobot, typing_extensions.Iterable[semantic_digital_twin.robots.abstract_robot.AbstractRobot]], arm: typing_extensions.Union[typing_extensions.List[pycram.datastructures.enums.Arms], pycram.datastructures.enums.Arms] = None, prepose_distance: float = ActionConfig.grasping_prepose_distance)

   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`


   Location designator which describes poses used for opening drawers


   .. py:attribute:: handle
      :type:  semantic_digital_twin.world_description.world_entity.Body


   .. py:attribute:: robot
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot


   .. py:attribute:: prepose_distance
      :value: 0.03



   .. py:attribute:: arm


   .. py:method:: ground() -> pycram.datastructures.pose.PoseStamped

      Default specialized_designators for this location designator, just returns the first element from the iteration

      :return: A location designator for a pose from which the drawer can be opened



   .. py:method:: adjust_map_for_drawer_opening(cost_map: pycram.costmaps.Costmap, init_pose: pycram.datastructures.pose.PoseStamped, goal_pose: pycram.datastructures.pose.PoseStamped, width: float = 0.2)
      :staticmethod:


      Adjust the cost map for opening a drawer. This is done by removing all locations between the initial and final
      pose of the drawer/container.

      :param cost_map: Costmap that should be adjusted.
      :param init_pose: Pose of the drawer/container when it is fully closed.
      :param goal_pose: Pose of the drawer/container when it is fully opened.
      :param width: Width of the drawer/container.



   .. py:method:: setup_costmaps(handle: semantic_digital_twin.world_description.world_entity.Body) -> pycram.costmaps.Costmap

      Sets up the costmaps for the given handle and robot. The costmaps are merged and stored in the final_map.



   .. py:method:: create_target_sequence(params_box: box.Box, final_map: pycram.costmaps.Costmap) -> typing_extensions.List[pycram.datastructures.pose.PoseStamped]

      Creates the sequence of target poses

      :param params_box:
      :param final_map:
      :return:



   .. py:method:: __iter__() -> typing_extensions.Iterator[pycram.datastructures.pose.PoseStamped]

      Creates poses from which the robot can open the drawer specified by the ObjectPart designator describing the
      handle. Poses are validated by checking if the robot can grasp the handle while the drawer is closed and if
      the handle can be grasped if the drawer is open.

      :yield: A location designator containing the pose and the arms that can be used.



.. py:class:: SemanticCostmapLocation(body, for_object: semantic_digital_twin.world_description.world_entity.Body = None, edges_only: bool = False, horizontal_edges_only: bool = False, edge_size_in_meters: float = 0.06, height_offset: float = 0.0)

   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`


   Locations over semantic entities, like a table surface


   .. py:attribute:: body
      :type:  semantic_digital_twin.world_description.world_entity.Body


   .. py:attribute:: for_object
      :type:  typing_extensions.Optional[semantic_digital_twin.world_description.world_entity.Body]
      :value: None



   .. py:attribute:: edges_only
      :type:  bool
      :value: False



   .. py:attribute:: horizontal_edges_only
      :type:  bool
      :value: False



   .. py:attribute:: edge_size_in_meters
      :type:  float
      :value: 0.06



   .. py:attribute:: sem_costmap
      :type:  typing_extensions.Optional[pycram.costmaps.SemanticCostmap]
      :value: None



   .. py:method:: ground() -> pycram.datastructures.pose.PoseStamped

      Default specialized_designators which returns the first element of the iterator of this instance.

      :return: A resolved location



   .. py:method:: __iter__() -> typing_extensions.Iterator[pycram.datastructures.pose.PoseStamped]

      Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
      which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
      is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

      :yield: An instance of SemanticCostmapLocation.Location with the found valid position of the Costmap.



.. py:class:: ProbabilisticSemanticLocation(bodies: typing_extensions.List[semantic_digital_twin.world_description.world_entity.Body], for_object: semantic_digital_twin.world_description.world_entity.Body = None, link_is_center_link: bool = False, number_of_samples: int = 1000, sort_sampels: bool = False, uniform_sampling: bool = False, highlight_used_surfaces: bool = False)

   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`


   Location designator which samples poses from a semantic costmap with a probability distribution.
   The construction of the probabilistic circuit can be quite time-consuming, but the majority of the computational load
   is done during the first iteration only. In this case the exact time is highly dependent on the number of
   links in the world, and the number of links we are sampling from.


   .. py:attribute:: surface_x

      Variable representing the x coordinate on a surface



   .. py:attribute:: surface_y

      Variable representing the y coordinate on a surface



   .. py:attribute:: sort_samples
      :value: False



   .. py:attribute:: uniform_sampling
      :value: False



   .. py:attribute:: number_of_samples
      :value: 1000



   .. py:attribute:: bodies
      :type:  typing_extensions.List[semantic_digital_twin.world_description.world_entity.Body]


   .. py:attribute:: link_is_center_link
      :type:  bool
      :value: False



   .. py:attribute:: for_object
      :type:  typing_extensions.Optional[semantic_digital_twin.world_description.world_entity.Body]
      :value: None



   .. py:attribute:: highlight_used_surfaces
      :type:  bool
      :value: False



   .. py:method:: ground() -> pycram.datastructures.pose.PoseStamped

      Default specialized_designators which returns the first element of the iterator of this instance.

      :return: A resolved location



   .. py:method:: _create_link_circuit(surface_samples: typing_extensions.List[typing_extensions.Tuple[float, float]]) -> probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit

      Creates a probabilistic circuit that samples navigation poses on a surface defined by the given surface samples.
      The circuit will sample poses that are close to the surface samples and have the given link id as true.
      The circuit will also sample the x and y coordinates of the poses from a Gaussian distribution centered around
      the surface samples with a scale of 1.0.

      :param surface_samples: A list of surface samples, each sample is a tuple of (x, y) coordinates.

      :return: A ProbabilisticCircuit that samples navigation poses on the surface defined by the surface samples.



   .. py:method:: _create_surface_event(body: semantic_digital_twin.world_description.world_entity.Body, search_space: semantic_digital_twin.world_description.shape_collection.BoundingBoxCollection, wall_bloat: float) -> typing_extensions.Optional[random_events.product_algebra.Event]
      :staticmethod:


      Creates an event that describes the surface of the link we want to sample from.
      The surface event is constructed from the bounding box of the link, and the walls and doors are cut out to
      ensure that the target poses are not too close to walls or doors.

      :param body: The link for which the surface event should be created.
      :param search_space: The search space in which the surface event should be created.
      :param wall_bloat: The amount by which the walls should be bloated to ensure the target poses are not too close
      to walls or doors.

      :return: An Event that describes the surface of the link, or None if the surface event is empty.



   .. py:method:: _create_navigation_space_event(free_space: semantic_digital_twin.world_description.graph_of_convex_sets.GraphOfConvexSets, body: semantic_digital_twin.world_description.world_entity.Body) -> typing_extensions.Optional[random_events.product_algebra.Event]

      Creates an event that describes the navigation space for the link we want to sample from.
      The navigation space is the free space around the link, which is used to sample navigation poses.
      The navigation space is constructed from the free space graph, which is a subgraph of the free space that is
      reachable from the target node, which is the node in the free space graph that is above the link.

      :param free_space: The free space graph that is used to sample navigation poses.
      :param body: The link for which the navigation space event should be created.

      :return: An Event that describes the navigation space for the link, or None if the navigation space event is empty.



   .. py:method:: _create_distribution_for_link(body: semantic_digital_twin.world_description.world_entity.Body) -> typing_extensions.Tuple[typing_extensions.Optional[probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit], float]

      Creates a distribution for the given link, which is a probabilistic circuit that samples navigation poses on the
      surface of the link. The distribution is conditioned on the navigation space event, which is the free space
      around the link, and the surface event, which is the surface of the link we want to sample from.

      :param body: The link for which the distribution should be created.

      :return: A tuple containing the conditioned link circuit and the probability of the condition.



   .. py:method:: _calculate_surface_z_coord(test_robot: semantic_digital_twin.robots.abstract_robot.AbstractRobot, surface_coords: typing_extensions.Tuple[float, float]) -> typing_extensions.Optional[float]
      :staticmethod:


      Calculates the z-coordinate of the surface at the given surface coordinates on the link.

      :param test_robot: The robot that is used to test the surface coordinates.
      :param surface_coords: The coordinates on the surface where the z-coordinate should be calculated.

      :return: The z-coordinate of the surface at the given surface coordinates, or None if the link is not hit.



   .. py:method:: __iter__() -> typing_extensions.Iterator[pycram.datastructures.pose.PoseStamped]

      Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
      which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
      is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

      :yield: A PoseStamped with the found valid position of the Costmap.



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



