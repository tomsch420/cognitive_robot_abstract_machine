pycram.designators.specialized_designators.probabilistic.probabilistic_action
=============================================================================

.. py:module:: pycram.designators.specialized_designators.probabilistic.probabilistic_action


Classes
-------

.. autoapisummary::

   pycram.designators.specialized_designators.probabilistic.probabilistic_action.Variables
   pycram.designators.specialized_designators.probabilistic.probabilistic_action.ProbabilisticAction
   pycram.designators.specialized_designators.probabilistic.probabilistic_action.MoveAndPickUpVariables
   pycram.designators.specialized_designators.probabilistic.probabilistic_action.MoveAndPickUpParameterizer


Module Contents
---------------

.. py:class:: Variables(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Abstract enum for variables in a parameterizer.


   .. py:method:: all()
      :classmethod:



.. py:class:: ProbabilisticAction(policy: typing_extensions.Optional[probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit] = None)

   Abstract class for performables that have a probabilistic parametrization.


   .. py:attribute:: policy
      :type:  probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit

      The policy that is used to determine the parameters.



   .. py:attribute:: variables
      :type:  Type[Variables]


   .. py:method:: default_policy() -> probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit

      :return: The default policy for the action.



.. py:class:: MoveAndPickUpVariables(*args, **kwds)

   Bases: :py:obj:`Variables`


   Abstract enum for variables in a parameterizer.


   .. py:attribute:: arm


   .. py:attribute:: keep_joint_states


   .. py:attribute:: rotate_gripper


   .. py:attribute:: x


   .. py:attribute:: y


   .. py:attribute:: approach_direction


   .. py:attribute:: vertical_alignment


.. py:class:: MoveAndPickUpParameterizer(policy: typing_extensions.Optional[probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit] = None)

   Bases: :py:obj:`ProbabilisticAction`


   Action that moves the agent to an object and picks it up using probability tools to parameterize.


   .. py:attribute:: variables


   .. py:attribute:: partial
      :type:  pycram.datastructures.partial_designator.PartialDesignator[pycram.robot_plans.MoveAndPickUpAction]


   .. py:attribute:: world
      :type:  semantic_digital_twin.world.World


   .. py:method:: collision_free_condition_for_object(obj: semantic_digital_twin.world_description.world_entity.Body)


   .. py:method:: accessing_distribution_for_object(obj: semantic_digital_twin.world_description.world_entity.Body, object_variable: random_events.variable.Symbolic) -> probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit


   .. py:property:: object_variable
      :type: random_events.variable.Symbolic



   .. py:method:: create_distribution()


   .. py:method:: create_actions(amount: int = 100) -> typing_extensions.List[pycram.robot_plans.MoveAndPickUpAction]


   .. py:method:: sample_to_action(sample: typing_extensions.List, model: probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit.ProbabilisticCircuit) -> pycram.robot_plans.MoveAndPickUpAction


   .. py:method:: create_action()


   .. py:method:: query_for_database()


