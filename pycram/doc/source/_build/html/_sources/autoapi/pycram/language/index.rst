pycram.language
===============

.. py:module:: pycram.language


Attributes
----------

.. autoapisummary::

   pycram.language.logger


Classes
-------

.. autoapisummary::

   pycram.language.LanguagePlan
   pycram.language.SequentialPlan
   pycram.language.ParallelPlan
   pycram.language.TryInOrderPlan
   pycram.language.TryAllPLan
   pycram.language.RepeatPlan
   pycram.language.MonitorPlan
   pycram.language.CodePlan
   pycram.language.LanguageNode
   pycram.language.SequentialNode
   pycram.language.ParallelNode
   pycram.language.RepeatNode
   pycram.language.MonitorNode
   pycram.language.TryInOrderNode
   pycram.language.TryAllNode
   pycram.language.CodeNode


Module Contents
---------------

.. py:data:: logger

.. py:class:: LanguagePlan(root: LanguageNode, context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion, pycram.robot_plans.actions.base.ActionDescription])

   Bases: :py:obj:`pycram.plan.Plan`


   Base class for language plans


   .. py:method:: simplify_language_nodes()

      Traverses the plan and merges LanguageNodes of the same type



.. py:class:: SequentialPlan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`LanguagePlan`


   Creates a plan which executes its children in sequential order


.. py:class:: ParallelPlan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion], root: LanguageNode = None)

   Bases: :py:obj:`LanguagePlan`


   Creates a plan which executes all children in parallel in seperate threads


   .. py:attribute:: parallel_blocklist
      :value: ['PickUpAction', 'PlaceAction', 'OpenAction', 'CloseAction', 'TransportAction', 'GraspingAction']


      A list of Actions which can't be part of a Parallel plan



.. py:class:: TryInOrderPlan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`LanguagePlan`


   Creates a plan that executes all children in sequential order but does not stop if one of them throws an error


.. py:class:: TryAllPLan(context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`ParallelPlan`


   Creates a plan which executes all children in parallel but does not abort if one throws an error


.. py:class:: RepeatPlan(context: pycram.datastructures.dataclasses.Context, repeat=1, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion])

   Bases: :py:obj:`LanguagePlan`


   A plan which repeats all children a number of times


.. py:class:: MonitorPlan(condition, context: pycram.datastructures.dataclasses.Context, *children: typing_extensions.Union[pycram.plan.Plan, pycram.datastructures.partial_designator.PartialDesignator, pycram.robot_plans.BaseMotion], behavior=MonitorBehavior.INTERRUPT)

   Bases: :py:obj:`LanguagePlan`


   A plan which monitors a condition and upon the condition becoming true interrupts all children. Monitors can have
   different behaviors, they can Interrupt, Pause or Resume the execution of the children. If the behavior is set to
   resume the plan will be paused until the condition is met.

   :param condition: A condition which should be monitored
   :behavior: The behavior of the monitor, either :py:attr:`~MonitorBehavior.INTERRUPT`, :py:attr:`~MonitorBehavior.PAUSE` or :py:attr:`~MonitorBehavior.RESUME`


.. py:class:: CodePlan(context: pycram.datastructures.dataclasses.Context, func: typing_extensions.Callable, kwargs: typing_extensions.Dict[str, typing_extensions.Any] = None)

   Bases: :py:obj:`LanguagePlan`


   A Plan that contains a function to be executed. Mainly intended for debugging purposes


.. py:class:: LanguageNode

   Bases: :py:obj:`pycram.plan.PlanNode`


   .. py:attribute:: designator_type
      :type:  typing_extensions.Type[LanguageNode]

      Superclass for language nodes in a plan. Used to distinguish language nodes from other types of nodes.



.. py:class:: SequentialNode

   Bases: :py:obj:`LanguageNode`


   .. py:attribute:: designator_type
      :type:  typing_extensions.Type[SequentialNode]

      Executes all children sequentially, an exception while executing a child does not terminate the whole process.
      Instead, the exception is saved to a list of all exceptions thrown during execution and returned.

      Behaviour:
          Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from each
          child's perform() method. The state is :py:attr:`~TaskStatus.SUCCEEDED` *iff* all children are executed without
          exception. In any other case the State :py:attr:`~TaskStatus.FAILED` will be returned.



   .. py:method:: perform()

      Behaviour of Sequential, calls perform() on each child sequentially

      :return: The state and list of results according to the behaviour described in :func:`Sequential`



   .. py:method:: perform_sequential(nodes: typing_extensions.List[pycram.plan.PlanNode], raise_exceptions=True) -> typing_extensions.Any

      Behavior of the sequential node, performs all children in sequence and raises error if they occur.

      :param nodes: A list of nodes which should be performed in sequence
      :param raise_exceptions: If True (default) errors will be raised



   .. py:method:: __hash__()


   .. py:method:: __repr__()


.. py:class:: ParallelNode

   Bases: :py:obj:`LanguageNode`


   Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
   exceptions during execution will be caught, saved to a list and returned upon end.

   Behaviour:
       Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from
       each child's perform() method. The state is :py:attr:`~TaskStatus.SUCCEEDED` *iff* all children could be executed without
       an exception. In any other case the State :py:attr:`~TaskStatus.FAILED` will be returned.



   .. py:method:: perform()

      Behaviour of Parallel, creates a new thread for each child and calls perform() of the child in the respective
      thread.

      :return: The state and list of results according to the behaviour described in :func:`Parallel`




   .. py:method:: perform_parallel(nodes: typing_extensions.List[pycram.plan.PlanNode])

      Behaviour of the parallel node performs the given nodes in parallel in different threads.

      :param nodes: A list of nodes which should be performed in parallel



   .. py:method:: _lang_call(node: pycram.plan.PlanNode)

      Wrapper method which is executed in the thread. Wraps the given node in a try catch and performs it

      :param node: The node which is to be performed



   .. py:method:: __hash__()


   .. py:method:: __repr__()


.. py:class:: RepeatNode

   Bases: :py:obj:`SequentialNode`


   .. py:attribute:: repeat
      :type:  int
      :value: 1


      Executes all children a given number of times in sequential order.



   .. py:method:: perform()

      Behaviour of repeat, executes all children in a loop as often as stated on initialization.

      :return:



   .. py:method:: __hash__()


.. py:class:: MonitorNode(condition: typing_extensions.Union[typing_extensions.Callable, pycram.fluent.Fluent] = None, behavior: typing_extensions.Optional[pycram.datastructures.enums.MonitorBehavior] = MonitorBehavior.INTERRUPT)

   Bases: :py:obj:`SequentialNode`


   Monitors a Language Expression and interrupts it when the given condition is evaluated to True.

   Behaviour:
       Monitors start a new Thread which checks the condition while performing the nodes below it. Monitors can have
       different behaviors, they can Interrupt, Pause or Resume the execution of the children.
       If the behavior is set to Resume the plan will be paused until the condition is met.


   .. py:attribute:: kill_event


   .. py:attribute:: exception_queue


   .. py:attribute:: behavior


   .. py:attribute:: monitor_thread


   .. py:method:: perform()

      Behavior of the Monitor, starts a new Thread which checks the condition and then performs the attached language
      expression

      :return: The state of the attached language expression, as well as a list of the results of the children



   .. py:method:: monitor()


   .. py:method:: __hash__()


.. py:class:: TryInOrderNode

   Bases: :py:obj:`SequentialNode`


   Executes all children sequentially, an exception while executing a child does not terminate the whole process.
   Instead, the exception is saved to a list of all exceptions thrown during execution and returned.

   Behaviour:
       Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from each
       child's perform() method. The state is :py:attr:`~TaskStatus.SUCCEEDED` if one or more children are executed without
       exception. In the case that all children could not be executed the State :py:attr:`~TaskStatus.FAILED` will be returned.


   .. py:method:: perform()

      Behaviour of TryInOrder, calls perform() on each child sequentially and catches raised exceptions.

      :return: The state and list of results according to the behaviour described in :func:`TryInOrder`



   .. py:method:: __hash__()


.. py:class:: TryAllNode

   Bases: :py:obj:`ParallelNode`


   Executes all children in parallel by creating a thread per children and executing them in the respective thread. All
   exceptions during execution will be caught, saved to a list and returned upon end.

   Behaviour:
       Returns a tuple containing the final state of execution (SUCCEEDED, FAILED) and a list of results from each
       child's perform() method. The state is :py:attr:`~TaskStatus.SUCCEEDED` if one or more children could be executed
       without raising an exception. If all children fail the State :py:attr:`~TaskStatus.FAILED` will be returned.


   .. py:method:: perform()

      Behaviour of TryAll, creates a new thread for each child and executes all children in their respective threads.

      :return: The state and list of results according to the behaviour described in :func:`TryAll`



   .. py:method:: __hash__()


.. py:class:: CodeNode(function: typing_extensions.Optional[typing_extensions.Callable] = None, kwargs: typing_extensions.Optional[typing_extensions.Dict] = None)

   Bases: :py:obj:`LanguageNode`


   Executable code block in a plan.

   :ivar function: The function (plan) that was called
   :ivar kwargs: Dictionary holding the keyword arguments of the function


   .. py:attribute:: designator_type
      :type:  typing_extensions.Type[LanguageNode]

      Superclass for language nodes in a plan. Used to distinguish language nodes from other types of nodes.



   .. py:attribute:: function
      :type:  typing_extensions.Callable
      :value: None



   .. py:attribute:: kwargs
      :type:  typing_extensions.Dict[str, typing_extensions.Any]
      :value: None



   .. py:attribute:: perform


   .. py:attribute:: performable


   .. py:attribute:: action


   .. py:method:: execute() -> typing_extensions.Any

      Execute the code with its arguments

      :returns: Anything that the function associated with this object will return.



   .. py:method:: resolve() -> typing_extensions.Self


   .. py:method:: __hash__()


