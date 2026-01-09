pycram.failure_handling
=======================

.. py:module:: pycram.failure_handling


Classes
-------

.. autoapisummary::

   pycram.failure_handling.FailureHandling
   pycram.failure_handling.Retry
   pycram.failure_handling.RetryMonitor


Functions
---------

.. autoapisummary::

   pycram.failure_handling.try_action
   pycram.failure_handling.try_motion
   pycram.failure_handling.try_method


Module Contents
---------------

.. py:class:: FailureHandling(plan: typing_extensions.Optional[pycram.plan.Plan] = None)

   Base class for failure handling mechanisms in automated systems or workflows.

   This class provides a structure for implementing different strategies to handle
   failures that may occur during the execution of a plan or process. It is designed
   to be extended by subclasses that implement specific failure handling behaviors.


   .. py:attribute:: plan
      :value: None



   .. py:method:: perform()
      :abstractmethod:


      Abstract method to perform the failure handling mechanism.

      This method should be overridden in subclasses to implement the specific
      behavior for handling failures.

      :raises NotImplementedError: If the method is not implemented in a subclass.



.. py:class:: Retry(plan: pycram.language.MonitorPlan, max_tries: int = 3)

   Bases: :py:obj:`FailureHandling`


   A subclass of FailureHandling that implements a retry mechanism.

   This class represents a specific failure handling strategy where the system
   attempts to retry a failed action a certain number of times before giving up.


   .. py:attribute:: max_tries
      :type:  int

      The maximum number of attempts to retry the action.



   .. py:method:: perform()

      Implementation of the retry mechanism.

      This method attempts to perform the action specified in the designator_description.
      If the action fails, it is retried up to max_tries times. If all attempts fail,
      the last exception is raised.

      :raises PlanFailure: If all retry attempts fail.



.. py:class:: RetryMonitor(monitor_plan: pycram.plan.Plan, max_tries: int = 3, recovery: dict = None)

   Bases: :py:obj:`FailureHandling`


   A subclass of FailureHandling that implements a retry mechanism that works with a Monitor.

   This class represents a specific failure handling strategy that allows us to retry a demo that is
   being monitored, in case that monitoring condition is triggered.


   .. py:attribute:: max_tries
      :type:  int

      The maximum number of attempts to retry the action.



   .. py:attribute:: recovery
      :type:  dict

      A dictionary that maps exception types to recovery actions



   .. py:attribute:: lock


   .. py:method:: perform() -> typing_extensions.List[typing_extensions.Any]

      This method attempts to perform the Monitor + plan specified in the designator_description. If the action
      fails, it is retried up to max_tries times. If all attempts fail, the last exception is raised. In every
      loop, we need to clear the kill_event, and set all relevant 'interrupted' variables to False, to make sure
      the Monitor and plan are executed properly again.

      :raises PlanFailure: If all retry attempts fail.

      :return: The state of the execution performed, as well as a flattened list of the
      results, in the correct order



.. py:function:: try_action(action: typing_extensions.Any, failure_type: typing_extensions.Type[Exception], max_tries: int = 3)

   A generic function to retry an action a certain number of times before giving up, with a specific failure type.

   :param action: The action to be performed, it must have a perform() method.
   :param failure_type: The type of exception to catch.
   :param max_tries: The maximum number of attempts to retry the action. Defaults to 3.


.. py:function:: try_motion(motion: pycram.process_module.ProcessModule, motion_designator_instance: pycram.robot_plans.BaseMotion, failure_type: typing_extensions.Type[Exception], max_tries: int = 3)

   A generic function to retry a motion a certain number of times before giving up, with a specific exception.

   :param motion: The motion to be executed.
   :param motion_designator_instance: The instance of the motion designator that has the description of the motion.
   :param failure_type: The type of exception to catch.
   :param max_tries: The maximum number of attempts to retry the motion.


.. py:function:: try_method(method: typing_extensions.Callable, failure_type: typing_extensions.Type[Exception], max_tries: int = 3, name: str = 'method')

   A generic function to retry a method a certain number of times before giving up, with a specific exception.

   :param method: The method to be called.
   :param failure_type: The type of exception to catch.
   :param max_tries: The maximum number
   :param name: The name of the method to be called.


