pycram.robot_plans.actions.base
===============================

.. py:module:: pycram.robot_plans.actions.base


Attributes
----------

.. autoapisummary::

   pycram.robot_plans.actions.base.logger


Classes
-------

.. autoapisummary::

   pycram.robot_plans.actions.base.ActionDescription


Module Contents
---------------

.. py:data:: logger

.. py:class:: ActionDescription

   Bases: :py:obj:`pycram.designator.DesignatorDescription`, :py:obj:`pycram.has_parameters.HasParameters`


   Base class for everything that contains potentially parameters for a plan.


   .. py:attribute:: _pre_perform_callbacks
      :value: []



   .. py:attribute:: _post_perform_callbacks
      :value: []



   .. py:method:: __post_init__()


   .. py:method:: perform() -> typing_extensions.Any

      Full execution: pre-check, plan, post-check



   .. py:method:: execute() -> typing_extensions.Any
      :abstractmethod:


      Symbolic plan. Should only call motions or sub-actions.



   .. py:method:: validate_precondition()
      :abstractmethod:


      Symbolic/world state precondition validation.



   .. py:method:: validate_postcondition(result: typing_extensions.Optional[typing_extensions.Any] = None)
      :abstractmethod:


      Symbolic/world state postcondition validation.



   .. py:method:: pre_perform(func) -> typing_extensions.Callable
      :classmethod:



   .. py:method:: post_perform(func) -> typing_extensions.Callable
      :classmethod:



