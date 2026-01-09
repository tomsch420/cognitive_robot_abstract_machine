pycram.designators.object_designator
====================================

.. py:module:: pycram.designators.object_designator


Classes
-------

.. autoapisummary::

   pycram.designators.object_designator.BelieveObject
   pycram.designators.object_designator.ResolutionStrategyObject


Module Contents
---------------

.. py:class:: BelieveObject(names: typing_extensions.Optional[typing_extensions.List[str]] = None)

   Bases: :py:obj:`pycram.external_interfaces.robokudo.ObjectDesignatorDescription`


   Description for Objects that are only believed in.


.. py:class:: ResolutionStrategyObject(strategy: typing_extensions.Union[pycram.external_interfaces.robokudo.Callable, typing_extensions.Iterable])

   Bases: :py:obj:`pycram.external_interfaces.robokudo.ObjectDesignatorDescription`


   Class for object designator_description descriptions.
   Descriptions hold possible parameter ranges for object designators.


   .. py:attribute:: strategy


   .. py:method:: create_iterator(resolution_strategy: typing_extensions.Union[pycram.external_interfaces.robokudo.Callable, typing_extensions.Iterable])

      Creates an iterator for the given method. If the method is iterable it will be used as is, otherwise it will
      be called as a function.

      :param resolution_strategy: The method to create an iterator for.
      :return: An iterator for the given method.



   .. py:method:: __iter__() -> typing_extensions.Iterable[semantic_digital_twin.world_description.world_entity.Body]

      Iterates through every possible solution for the given solution strategy.

      :return: A resolved object designator



