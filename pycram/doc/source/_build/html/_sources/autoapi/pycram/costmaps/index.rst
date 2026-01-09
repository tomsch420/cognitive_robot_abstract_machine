pycram.costmaps
===============

.. py:module:: pycram.costmaps


Attributes
----------

.. autoapisummary::

   pycram.costmaps.logger
   pycram.costmaps.cmap


Classes
-------

.. autoapisummary::

   pycram.costmaps.OrientationGenerator
   pycram.costmaps.Rectangle
   pycram.costmaps.Costmap
   pycram.costmaps.OccupancyCostmap
   pycram.costmaps.VisibilityCostmap
   pycram.costmaps.GaussianCostmap
   pycram.costmaps.RingCostmap
   pycram.costmaps.SemanticCostmap
   pycram.costmaps.AlgebraicSemanticCostmap


Functions
---------

.. autoapisummary::

   pycram.costmaps.plot_grid


Module Contents
---------------

.. py:data:: logger

.. py:class:: OrientationGenerator

   Provides methods to generate orientations for pose candidates.


   .. py:method:: generate_origin_orientation(position: typing_extensions.List[float], origin: pycram.datastructures.pose.PoseStamped) -> typing_extensions.List[float]
      :staticmethod:


      Generates an orientation such that the robot faces the origin of the costmap.

      :param position: The position in the costmap, already converted to the world coordinate frame.
      :param origin: The origin of the costmap, the point which the robot should face.
      :return: A quaternion of the calculated orientation.



   .. py:method:: generate_random_orientation(*_, rng: random.Random = random.Random(42)) -> typing_extensions.List[float]
      :staticmethod:


      Generates a random orientation rotated around the z-axis (yaw).
      A random angle is sampled using a provided RNG instance to ensure reproducibility.

      :param _: Ignored parameters to maintain compatibility with other orientation generators.
      :param rng: Random number generator instance for reproducible sampling.

      :return: A quaternion of the randomly generated orientation.



.. py:class:: Rectangle

   A rectangle that is described by a lower and upper x and y value.


   .. py:attribute:: x_lower
      :type:  float


   .. py:attribute:: x_upper
      :type:  float


   .. py:attribute:: y_lower
      :type:  float


   .. py:attribute:: y_upper
      :type:  float


   .. py:method:: translate(x: float, y: float)

      Translate the rectangle by x and y



   .. py:method:: scale(x_factor: float, y_factor: float)

      Scale the rectangle by x_factor and y_factor



.. py:class:: Costmap

   The base class of all Costmaps which implements the visualization of costmaps
   in the World.


   .. py:attribute:: resolution
      :type:  float

      The distance in metre in the real-world which is represented by a single entry in the costmap. 



   .. py:attribute:: height
      :type:  int

      Height of the costmap.



   .. py:attribute:: width
      :type:  int

      Width of the costmap.



   .. py:attribute:: origin
      :type:  pycram.datastructures.pose.PoseStamped

      Origin pose of the costmap.



   .. py:attribute:: map
      :type:  numpy.ndarray

      Numpy array to save the costmap distribution

      Costmaps represent the 2D distribution in a numpy array where axis 0 is the X-Axis of the coordinate system and axis 1 
      is the Y-Axis of the coordinate system. An increase in the index of the axis of the numpy array corresponds to an increase in the 
      value of the spatial axis. The factor by how the value of the index of the numpy corresponds to the spatial coordinate 
      system is given by the resolution. 

      Furthermore, there is a difference in the origin of the two representations while the numpy arrays start from the top left 
      corner, the origin given as PoseStamped is placed in the middle of the array. The costmap is build around the origin and 
      since the array start from 0, 0 in the corner this conversion is necessary. 

                  y-axis      0, 10
          0,0 ------------------
              ------------------
              ------------------
      x-axis  ------------------
              ------------------
              ------------------
        10, 0 ------------------



   .. py:attribute:: world
      :type:  semantic_digital_twin.world.World

      The world from which this costmap was created.



   .. py:attribute:: vis_ids
      :type:  typing_extensions.List[int]
      :value: []



   .. py:attribute:: number_of_samples
      :type:  int
      :value: 200


      Number of samples to return at max



   .. py:attribute:: sample_randomly
      :type:  bool
      :value: False


      If the sampling should randomly pick valid entries



   .. py:attribute:: orientation_generator
      :type:  typing_extensions.Callable[pycram.datastructures.pose.PoseStamped, pycram.datastructures.pose.PoseStamped, [float]]
      :value: None


      An optional orientatoin generator to use to generate the orientation for a sampled pose



   .. py:method:: visualize() -> None

      Visualizes a costmap in the BulletWorld, the visualisation works by
      subdividing the costmap in rectangles which are then visualized as pybullet
      visual shapes.



   .. py:method:: _chunks(lst: typing_extensions.List, n: int) -> typing_extensions.Iterator[typing_extensions.List]

      Yield successive n-sized chunks from lst.

      :param lst: The list from which chunks should be yielded
      :param n: Size of the chunks
      :return: A list of size n from lst



   .. py:method:: close_visualization() -> None

      Removes the visualization from the World.



   .. py:method:: _find_consectuive_line(start: typing_extensions.Tuple[int, int], map: numpy.ndarray) -> int

      Finds the number of consecutive entries in the costmap which are greater
      than zero.

      :param start: The indices in the costmap from which the consecutive line should be found.
      :param map: The costmap in which the line should be found.
      :return: The length of the consecutive line of entries greater than zero.



   .. py:method:: _find_max_box_height(start: typing_extensions.Tuple[int, int], length: int, map: numpy.ndarray) -> int

      Finds the maximal height for a rectangle with a given width in a costmap.
      The method traverses one row at a time and checks if all entries for the
      given width are greater than zero. If an entry is less or equal than zero
      the height is returned.

      :param start: The indices in the costmap from which the method should start.
      :param length: The given width for the rectangle
      :param map: The costmap in which should be searched.
      :return: The height of the rectangle.



   .. py:method:: merge(other_cm: Costmap) -> Costmap

      Merges the values of two costmaps and returns a new costmap that has for
      every cell the merged values of both inputs. To merge two costmaps they
      need to fulfill 3 constrains:

      1. They need to have the same size
      2. They need to have the same x and y coordinates in the origin
      3. They need to have the same resolution

      If any of these constrains is not fulfilled a ValueError will be raised.

      :param other_cm: The other costmap with which this costmap should be merged.
      :return: A new costmap that contains the merged values



   .. py:method:: __add__(other: Costmap) -> Costmap

      Overloading of the "+" operator for merging of Costmaps. Furthermore, checks if 'other' is actual a Costmap and
      raises a ValueError if this is not the case. Please check :func:`~Costmap.merge` for further information of merging.

      :param other: Another Costmap
      :return: A new Costmap that contains the merged values from this Costmap and the other Costmap



   .. py:method:: partitioning_rectangles() -> typing_extensions.List[Rectangle]

      Partition the map attached to this costmap into rectangles. The rectangles are axis aligned, exhaustive and
      disjoint sets.

      :return: A list containing the partitioning rectangles



   .. py:method:: __iter__() -> typing_extensions.Iterator[pycram.datastructures.pose.PoseStamped]

      A generator that crates pose candidates from a given costmap. The generator
      selects the highest 100 values and returns the corresponding positions.
      Orientations are calculated such that the Robot faces the center of the costmap.

      :Yield: A tuple of position and orientation



   .. py:method:: segment_map() -> typing_extensions.List[numpy.ndarray]

      Finds partitions in the costmap and isolates them, a partition is a number of entries in the costmap which are
      neighbours. Returns a list of numpy arrays with one partition per array.

      :return: A list of numpy arrays with one partition per array



.. py:class:: OccupancyCostmap

   Bases: :py:obj:`Costmap`


   The occupancy Costmap represents a map of the environment where obstacles or
   positions which are inaccessible for a robot have a value of -1.


   .. py:attribute:: distance_to_obstacle
      :type:  float


   .. py:attribute:: robot_view
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot


   .. py:attribute:: _distance_to_obstacle_index
      :type:  int
      :value: None



   .. py:method:: __post_init__()


   .. py:method:: create_ray_mask_around_origin()

      Determines the occupied space around the origin position using ray testing. A ray is cast from the ground
      straight up 10m and if it hits something the position is considered occupied.

      :return: A 2d numpy array of the occupied space



   .. py:method:: inflate_obstacles(map: numpy.ndarray)

      Inflates found obstacles in the environment by the distance_to_obstacle factor.

      :param map: Map of obstacles to inflate.
      :return: The map with inflated obstacles.



   .. py:method:: _create_from_world() -> numpy.ndarray

      Creates an Occupancy Costmap for the specified World.
      This map marks every position as valid that has no object above it. After
      creating the costmap the distance to obstacle parameter is applied.



.. py:class:: VisibilityCostmap

   Bases: :py:obj:`Costmap`


   A costmap that represents the visibility of a specific point for every position around
   this point. For a detailed explanation on how the creation of the costmap works
   please look here: `PhD Thesis (page 173) <https://mediatum.ub.tum.de/doc/1239461/1239461.pdf>`_


   .. py:attribute:: min_height
      :type:  float


   .. py:attribute:: max_height
      :type:  float


   .. py:attribute:: target_object
      :type:  typing_extensions.Optional[Union[semantic_digital_twin.world_description.world_entity.Body, pycram.datastructures.pose.PoseStamped]]
      :value: None



   .. py:method:: __post_init__()


   .. py:method:: _create_images() -> typing_extensions.List[numpy.ndarray]

      Creates four depth images in every direction around the point
      for which the costmap should be created. The depth images are converted
      to metre, meaning that every entry in the depth images represents the
      distance to the next object in metre.

      :return: A list of four depth images, the images are represented as 2D arrays.



   .. py:method:: _generate_map()

      This method generates the resulting density map by using the algorithm explained
      in Lorenz Mösenlechners `PhD Thesis (page 178) <https://mediatum.ub.tum.de/doc/1239461/1239461.pdf>`_
      The resulting map is then saved to :py:attr:`self.map`



.. py:class:: GaussianCostmap(mean: int, sigma: float, world: semantic_digital_twin.world.World, resolution: typing_extensions.Optional[float] = 0.02, origin: typing_extensions.Optional[pycram.datastructures.pose.PoseStamped] = None)

   Bases: :py:obj:`Costmap`


   Gaussian Costmaps are 2D gaussian distributions around the origin with the given mean and sigma


   .. py:attribute:: gau
      :type:  numpy.ndarray


   .. py:attribute:: map
      :type:  numpy.ndarray

      Numpy array to save the costmap distribution

      Costmaps represent the 2D distribution in a numpy array where axis 0 is the X-Axis of the coordinate system and axis 1 
      is the Y-Axis of the coordinate system. An increase in the index of the axis of the numpy array corresponds to an increase in the 
      value of the spatial axis. The factor by how the value of the index of the numpy corresponds to the spatial coordinate 
      system is given by the resolution. 

      Furthermore, there is a difference in the origin of the two representations while the numpy arrays start from the top left 
      corner, the origin given as PoseStamped is placed in the middle of the array. The costmap is build around the origin and 
      since the array start from 0, 0 in the corner this conversion is necessary. 

                  y-axis      0, 10
          0,0 ------------------
              ------------------
              ------------------
      x-axis  ------------------
              ------------------
              ------------------
        10, 0 ------------------



   .. py:attribute:: size
      :type:  float


   .. py:attribute:: width

      Width of the costmap.



   .. py:attribute:: height

      Height of the costmap.



   .. py:attribute:: resolution
      :type:  float
      :value: 0.02


      The distance in metre in the real-world which is represented by a single entry in the costmap. 



   .. py:attribute:: world

      The world from which this costmap was created.



   .. py:attribute:: origin
      :type:  pycram.datastructures.pose.PoseStamped

      Origin pose of the costmap.



   .. py:method:: _gaussian_window(mean: int, std: float) -> numpy.ndarray

      This method creates a window of values with a gaussian distribution of
      size "mean" and standart deviation "std".
      Code from `Scipy <https://github.com/scipy/scipy/blob/v0.14.0/scipy/signal/windows.py#L976>`_



.. py:class:: RingCostmap

   Bases: :py:obj:`Costmap`


   Creates a ring costmap, similar to the gaussian costmap but this looks more like a donut. Can be used to create poses
   for reaching a point for the robot.


   .. py:attribute:: std
      :type:  int

      Standard deviation of the gaussian distribution that makes up the ring.



   .. py:attribute:: distance
      :type:  float

      Distance between the center of the costmap and the center of the ring. A distance of 0 results in a gaussian costmap



   .. py:method:: __post_init__()


   .. py:method:: ring() -> numpy.ndarray


.. py:class:: SemanticCostmap(body: semantic_digital_twin.world_description.world_entity.Body, resolution: float = 0.02)

   Bases: :py:obj:`Costmap`


   Semantic Costmaps represent a 2D distribution over a link of an Object. An example of this would be a Costmap for a
   table surface.


   .. py:attribute:: world
      :type:  semantic_digital_twin.world.World

      The world from which this costmap was created.



   .. py:attribute:: body
      :type:  semantic_digital_twin.world_description.world_entity.Body


   .. py:attribute:: resolution
      :type:  float
      :value: 0.02


      The distance in metre in the real-world which is represented by a single entry in the costmap. 



   .. py:attribute:: origin
      :type:  pycram.datastructures.pose.PoseStamped

      Origin pose of the costmap.



   .. py:attribute:: height
      :type:  int
      :value: 0


      Height of the costmap.



   .. py:attribute:: width
      :type:  int
      :value: 0


      Width of the costmap.



   .. py:attribute:: map
      :type:  numpy.ndarray

      Numpy array to save the costmap distribution

      Costmaps represent the 2D distribution in a numpy array where axis 0 is the X-Axis of the coordinate system and axis 1 
      is the Y-Axis of the coordinate system. An increase in the index of the axis of the numpy array corresponds to an increase in the 
      value of the spatial axis. The factor by how the value of the index of the numpy corresponds to the spatial coordinate 
      system is given by the resolution. 

      Furthermore, there is a difference in the origin of the two representations while the numpy arrays start from the top left 
      corner, the origin given as PoseStamped is placed in the middle of the array. The costmap is build around the origin and 
      since the array start from 0, 0 in the corner this conversion is necessary. 

                  y-axis      0, 10
          0,0 ------------------
              ------------------
              ------------------
      x-axis  ------------------
              ------------------
              ------------------
        10, 0 ------------------



   .. py:method:: get_edges_map(margin_in_meters: float, horizontal_only: bool = False) -> Costmap

      Return a Costmap with only the edges of the original Costmap marked as possible positions.

      :param margin_in_meters: The edge thickness in meters that should be marked as possible positions.
      :param horizontal_only: If True only the horizontal edges will be marked as possible positions.
      :return: The modified Costmap.



   .. py:method:: generate_map() -> None

      Generates the semantic costmap according to the provided parameters. To do this the axis aligned bounding box (AABB)
      for the link name will be used. Height and width of the final Costmap will be the x and y sizes of the AABB.



   .. py:method:: points_in_poly(points, poly)
      :staticmethod:



.. py:class:: AlgebraicSemanticCostmap(body: semantic_digital_twin.world_description.world_entity.Body, number_of_samples=1000)

   Bases: :py:obj:`SemanticCostmap`


   Class for a semantic costmap that is based on an algebraic set-description of the valid area.


   .. py:attribute:: x
      :type:  random_events.variable.Continuous

      The variable for height.



   .. py:attribute:: y
      :type:  random_events.variable.Continuous

      The variable for width.



   .. py:attribute:: original_valid_area
      :type:  typing_extensions.Optional[random_events.product_algebra.SimpleEvent]

      The original rectangle of the valid area.



   .. py:attribute:: valid_area
      :type:  typing_extensions.Optional[random_events.product_algebra.Event]

      A description of the valid positions as set.



   .. py:attribute:: number_of_samples
      :type:  int

      The number of samples to generate for the iter.



   .. py:attribute:: world

      The world from which this costmap was created.



   .. py:method:: check_valid_area_exists()


   .. py:method:: left(margin=0.0) -> random_events.product_algebra.Event

      Create an event left of the origins Y-Coordinate.
      :param margin: The margin of the events left bound.
      :return: The left event.



   .. py:method:: right(margin=0.0) -> random_events.product_algebra.Event

      Create an event right of the origins Y-Coordinate.
      :param margin: The margin of the events right bound.
      :return: The right event.



   .. py:method:: top(margin=0.0) -> random_events.product_algebra.Event

      Create an event above the origins X-Coordinate.
      :param margin: The margin of the events upper bound.
      :return: The top event.



   .. py:method:: bottom(margin=0.0) -> random_events.product_algebra.Event

      Create an event below the origins X-Coordinate.
      :param margin: The margin of the events lower bound.
      :return: The bottom event.



   .. py:method:: inner(margin=0.2)


   .. py:method:: border(margin=0.2)


   .. py:method:: generate_map() -> None

      Generates the semantic costmap according to the provided parameters. To do this the axis aligned bounding box (AABB)
      for the link name will be used. Height and width of the final Costmap will be the x and y sizes of the AABB.



   .. py:method:: as_distribution() -> probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit


   .. py:method:: sample_to_pose(sample: numpy.ndarray) -> pycram.datastructures.pose.PoseStamped

      Convert a sample from the costmap to a pose.

      :param sample: The sample to convert
      :return: The pose corresponding to the sample



   .. py:method:: __iter__() -> typing_extensions.Iterator[pycram.datastructures.pose.PoseStamped]

      A generator that crates pose candidates from a given costmap. The generator
      selects the highest 100 values and returns the corresponding positions.
      Orientations are calculated such that the Robot faces the center of the costmap.

      :Yield: A tuple of position and orientation



.. py:data:: cmap

.. py:function:: plot_grid(data: numpy.ndarray) -> None

   An auxiliary method only used for debugging, it will plot a 2D numpy array using MatplotLib.


