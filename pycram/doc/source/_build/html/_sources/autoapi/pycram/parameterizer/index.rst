pycram.parameterizer
====================

.. py:module:: pycram.parameterizer


Classes
-------

.. autoapisummary::

   pycram.parameterizer.Parameterizer


Functions
---------

.. autoapisummary::

   pycram.parameterizer.collision_free_event
   pycram.parameterizer.update_variables_of_simple_event
   pycram.parameterizer.update_variables_of_event
   pycram.parameterizer.leaf_type_to_variable


Module Contents
---------------

.. py:class:: Parameterizer

   Class that generates possible parameters for a plan.

   This class can be used to generate possible parameters for a plan from probability distributions.


   .. py:attribute:: plan
      :type:  pycram.plan.Plan

      The plan to generate the parameters for.



   .. py:attribute:: parameters
      :type:  Dict[pycram.plan.DesignatorNode, Any]

      A dictionary that maps all nodes in the plan that hold actions to their parameters.



   .. py:attribute:: variables_of_node
      :type:  Dict[pycram.plan.DesignatorNode, List[random_events.variable.Variable]]

      A dictionary that maps all nodes in the plan that hold actions to the variables that describe that nodes parameters.



   .. py:method:: __post_init__()


   .. py:property:: variables
      :type: List[random_events.variable.Variable]


      :return: The variables for all parameters in the plan.



   .. py:method:: get_variable(name: str) -> random_events.variable.Variable

      :param name: The name of the variable.
      :return: The variable.



   .. py:method:: make_parameters()

      Create the parameters for all relevant nodes in the plan.



   .. py:method:: make_variables()

      Create the variables for all relevant parameters in the plan.



   .. py:method:: plan_from_sample(model: probabilistic_model.probabilistic_model.ProbabilisticModel, sample: numpy.ndarray, world: semantic_digital_twin.world.World) -> pycram.plan.Plan

      Create a sequential plan from a sample of all parameters.

      :param model: The model that generated the sample.
      :param sample: The sample to generate the plan from.
      :param world: The world to create the plan in.
      :return: The executable, sequential plan



   .. py:method:: create_fully_factorized_distribution() -> probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit

      :return: a fully factorized distribution for the plan.



   .. py:method:: create_restrictions() -> random_events.product_algebra.SimpleEvent

      :return: The restrictions present in the plan as random event.



.. py:function:: collision_free_event(world: semantic_digital_twin.world.World, search_space: Optional[semantic_digital_twin.world_description.shape_collection.BoundingBoxCollection] = None) -> random_events.product_algebra.Event

   Create an event that describes the free space of the world.
   :param world: The world to create the event from.
   :param search_space: The search space to limit the collision free event to.
   :return: An event that describes the free space.


.. py:function:: update_variables_of_simple_event(event: random_events.product_algebra.SimpleEvent, new_variables: Dict[random_events.variable.Variable, random_events.variable.Variable]) -> random_events.product_algebra.SimpleEvent

.. py:function:: update_variables_of_event(event: random_events.product_algebra.Event, new_variables: Dict[random_events.variable.Variable, random_events.variable.Variable]) -> random_events.product_algebra.Event

.. py:function:: leaf_type_to_variable(name: str, leaf_type: Type) -> random_events.variable.Variable

   Convert a leaf type to a random events variable.
   :param name: The name of the variable.
   :param leaf_type: The leaf type.
   :return: The variable.


