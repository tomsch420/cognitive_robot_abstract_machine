pycram.has_parameters
=====================

.. py:module:: pycram.has_parameters


Attributes
----------

.. autoapisummary::

   pycram.has_parameters.logger
   pycram.has_parameters.HasParameters
   pycram.has_parameters.LeafTypes
   pycram.has_parameters.leaf_types
   pycram.has_parameters.T


Classes
-------

.. autoapisummary::

   pycram.has_parameters.HasParametersMeta
   pycram.has_parameters.HasParameters


Functions
---------

.. autoapisummary::

   pycram.has_parameters.has_parameters


Module Contents
---------------

.. py:data:: logger

.. py:data:: HasParameters

.. py:data:: LeafTypes

.. py:data:: leaf_types

.. py:class:: HasParametersMeta(*args, **kwargs)

   Bases: :py:obj:`type`


   Metaclass for flattening and reconstructing nested Python datastructures until they reach something mentioned in leaf_types.
   This is very similar to JAX PyTrees `https://docs.jax.dev/en/latest/pytrees.html#pytrees`_.
   Will scan the constructor of the class for fields to be used and not class variables.


   .. py:attribute:: _parameters
      :type:  ParameterDict
      :value: None


      A dictionary that maps field names to their types, including nested types.
      The keys are the names of the variables. The values are the types of the variables or nested types.



   .. py:method:: create_parameters(target_class)
      :classmethod:


      Creates the flattened parameters for the given class.

      :param target_class: Class for which to create the parameters.



.. py:class:: HasParameters

   Base class for everything that contains potentially parameters for a plan.


   .. py:method:: flatten() -> typing_extensions.List

      Flattens the object into a list of field values.

      :return: A list of flattened field values from the object.

      :raises TypeError: If the object is not an instance of the target class.



   .. py:method:: flattened_parameters() -> typing_extensions.Dict[str, leaf_types]
      :classmethod:


      Returns a dictionary of all flattened fields and their types.

      :return: A dictionary mapping field names to their types.



   .. py:method:: number_of_fields() -> int
      :classmethod:


      :return: The total number of fields in the flattened list.



   .. py:method:: field_indices() -> typing_extensions.Dict[str, typing_extensions.Tuple[int, int]]
      :classmethod:


      :return: A dictionary mapping field names to their indices in the flattened list.



   .. py:method:: reconstruct(flattened: typing_extensions.List)
      :classmethod:


      Reconstructs the object from a flattened list of field values.

      :param flattened: The flattened list of field values.

      :return: An instance of the target class with the reconstructed field values.

      :raises TypeError: If the object is not a list or if the length of the list does not match the number of fields.



   .. py:method:: define_parameters() -> ParameterDict
      :classmethod:

      :abstractmethod:


      Override this method if you want to define a custom parameter dict for this class.

      :return: Dict like cls._parameters



.. py:data:: T

.. py:function:: has_parameters(target_class: T) -> T

   Insert parameters of a class post construction.
   Use this when dataclasses should be combined with HasParameters.

   :param target_class: The class to get the parameters from.
   :return: The updated class


