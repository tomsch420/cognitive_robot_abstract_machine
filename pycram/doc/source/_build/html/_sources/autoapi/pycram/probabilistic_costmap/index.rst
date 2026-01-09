pycram.probabilistic_costmap
============================

.. py:module:: pycram.probabilistic_costmap


Attributes
----------

.. autoapisummary::

   pycram.probabilistic_costmap.logger


Classes
-------

.. autoapisummary::

   pycram.probabilistic_costmap.Filter
   pycram.probabilistic_costmap.ProbabilisticCostmap


Module Contents
---------------

.. py:data:: logger

.. py:class:: Filter(*args, **kwds)

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


   .. py:attribute:: OCCUPANCY


   .. py:attribute:: VISIBILITY


.. py:class:: ProbabilisticCostmap(origin: pycram.datastructures.pose.PoseStamped, size: pint.Quantity = 2 * meter, max_cells=10000, costmap_type: typing_extensions.Type[pycram.costmaps.Costmap] = OccupancyCostmap, world: typing_extensions.Optional[semantic_digital_twin.world.World] = None, robot: semantic_digital_twin.robots.abstract_robot.AbstractRobot = None)

   A Costmap that uses probability distributions for representation.


   .. py:attribute:: x
      :type:  random_events.variable.Continuous

      The variable for the x-axis (height) in meters.



   .. py:attribute:: y
      :type:  random_events.variable.Continuous

      The variable for the y-axis (width) in meters.



   .. py:attribute:: costmap
      :type:  pycram.costmaps.Costmap

      The legacy costmap.



   .. py:attribute:: origin
      :type:  pycram.datastructures.pose.PoseStamped

      The origin of the costmap.



   .. py:attribute:: size
      :type:  pint.Quantity

      The side length of the costmap. The costmap is a square.



   .. py:attribute:: distribution
      :type:  typing_extensions.Optional[probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit]
      :value: None


      The distribution associated with the costmap.



   .. py:attribute:: world
      :value: None



   .. py:property:: publisher


   .. py:method:: create_event_from_map() -> random_events.product_algebra.Event

      :return: The event that is encoded by the costmaps map.



   .. py:method:: create_distribution()

      Create a probabilistic circuit from the costmap.



   .. py:method:: sample_to_pose(sample: numpy.ndarray) -> pycram.datastructures.pose.PoseStamped

      Convert a sample from the costmap to a pose.

      :param sample: The sample to convert
      :return: The pose corresponding to the sample



   .. py:method:: visualize()

      Visualize the costmap for rviz.



