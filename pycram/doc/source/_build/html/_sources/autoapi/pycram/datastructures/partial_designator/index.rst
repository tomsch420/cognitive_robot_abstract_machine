pycram.datastructures.partial_designator
========================================

.. py:module:: pycram.datastructures.partial_designator


Attributes
----------

.. autoapisummary::

   pycram.datastructures.partial_designator.T


Classes
-------

.. autoapisummary::

   pycram.datastructures.partial_designator.PartialDesignator


Module Contents
---------------

.. py:data:: T

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



