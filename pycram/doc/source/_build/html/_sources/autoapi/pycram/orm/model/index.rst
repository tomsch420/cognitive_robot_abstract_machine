pycram.orm.model
================

.. py:module:: pycram.orm.model


Classes
-------

.. autoapisummary::

   pycram.orm.model.PyCRAMQuaternionMapping
   pycram.orm.model.PlanNodeMapping
   pycram.orm.model.DesignatorNodeMapping
   pycram.orm.model.ActionNodeMapping
   pycram.orm.model.MotionNodeMapping
   pycram.orm.model.ResolvedActionNodeMapping
   pycram.orm.model.PlanEdge
   pycram.orm.model.PlanMapping
   pycram.orm.model.NumpyType


Module Contents
---------------

.. py:class:: PyCRAMQuaternionMapping

   Bases: :py:obj:`krrood.ormatic.dao.AlternativeMapping`\ [\ :py:obj:`pycram.datastructures.pose.PyCramQuaternion`\ ]


   .. py:attribute:: x
      :type:  float
      :value: 0



   .. py:attribute:: y
      :type:  float
      :value: 0



   .. py:attribute:: z
      :type:  float
      :value: 0



   .. py:attribute:: w
      :type:  float
      :value: 1



   .. py:method:: from_domain_object(obj: krrood.ormatic.dao.T)
      :classmethod:



   .. py:method:: to_domain_object() -> krrood.ormatic.dao.T


.. py:class:: PlanNodeMapping

   Bases: :py:obj:`krrood.ormatic.dao.AlternativeMapping`\ [\ :py:obj:`pycram.plan.PlanNode`\ ]


   Helper class that provides a standard way to create an ABC using
   inheritance.


   .. py:attribute:: status
      :type:  pycram.datastructures.enums.TaskStatus


   .. py:attribute:: start_time
      :type:  typing_extensions.Optional[datetime.datetime]
      :value: None



   .. py:attribute:: end_time
      :type:  typing_extensions.Optional[datetime.datetime]
      :value: None



   .. py:attribute:: reason
      :type:  typing_extensions.Optional[pycram.failures.PlanFailure]
      :value: None



   .. py:method:: from_domain_object(obj: pycram.plan.PlanNode)
      :classmethod:


      Convert a PlanNode to a PlanNodeDAO.



   .. py:method:: to_domain_object() -> krrood.ormatic.dao.T
      :abstractmethod:



.. py:class:: DesignatorNodeMapping

   Bases: :py:obj:`PlanNodeMapping`, :py:obj:`krrood.ormatic.dao.AlternativeMapping`\ [\ :py:obj:`pycram.plan.DesignatorNode`\ ]


   Helper class that provides a standard way to create an ABC using
   inheritance.


   .. py:attribute:: designator_ref
      :type:  pycram.designator.DesignatorDescription
      :value: None



   .. py:attribute:: designator_type
      :type:  Type[pycram.designator.DesignatorDescription]
      :value: None



   .. py:method:: from_domain_object(obj: pycram.plan.DesignatorNode)
      :classmethod:


      Convert a DesignatorNode to a DesignatorNodeDAO.



   .. py:method:: to_domain_object() -> krrood.ormatic.dao.T
      :abstractmethod:



.. py:class:: ActionNodeMapping

   Bases: :py:obj:`DesignatorNodeMapping`, :py:obj:`krrood.ormatic.dao.AlternativeMapping`\ [\ :py:obj:`pycram.plan.ActionNode`\ ]


   Helper class that provides a standard way to create an ABC using
   inheritance.


   .. py:method:: from_domain_object(obj: pycram.plan.ActionNode)
      :classmethod:


      Convert an ActionNode to an ActionNodeDAO.



   .. py:method:: to_domain_object() -> krrood.ormatic.dao.T
      :abstractmethod:



.. py:class:: MotionNodeMapping

   Bases: :py:obj:`DesignatorNodeMapping`, :py:obj:`krrood.ormatic.dao.AlternativeMapping`\ [\ :py:obj:`pycram.plan.MotionNode`\ ]


   Helper class that provides a standard way to create an ABC using
   inheritance.


   .. py:method:: to_domain_object() -> krrood.ormatic.dao.T
      :abstractmethod:



.. py:class:: ResolvedActionNodeMapping

   Bases: :py:obj:`DesignatorNodeMapping`, :py:obj:`krrood.ormatic.dao.AlternativeMapping`\ [\ :py:obj:`pycram.plan.ResolvedActionNode`\ ]


   Helper class that provides a standard way to create an ABC using
   inheritance.


   .. py:attribute:: designator_ref
      :type:  pycram.robot_plans.ActionDescription
      :value: None



   .. py:attribute:: execution_data
      :type:  pycram.datastructures.dataclasses.ExecutionData
      :value: None



   .. py:method:: from_domain_object(obj: pycram.plan.ResolvedActionNode)
      :classmethod:


      Convert a ResolvedActionNode to a ResolvedActionNodeDAO.



   .. py:method:: to_domain_object() -> krrood.ormatic.dao.T
      :abstractmethod:



.. py:class:: PlanEdge

   .. py:attribute:: parent
      :type:  pycram.plan.PlanNode


   .. py:attribute:: child
      :type:  pycram.plan.PlanNode


.. py:class:: PlanMapping

   Bases: :py:obj:`krrood.ormatic.dao.AlternativeMapping`\ [\ :py:obj:`pycram.plan.Plan`\ ]


   Helper class that provides a standard way to create an ABC using
   inheritance.


   .. py:attribute:: nodes
      :type:  List[pycram.plan.PlanNode]


   .. py:attribute:: edges
      :type:  List[PlanEdge]


   .. py:method:: from_domain_object(obj: pycram.plan.Plan)
      :classmethod:



   .. py:method:: to_domain_object() -> krrood.ormatic.dao.T
      :abstractmethod:



.. py:class:: NumpyType(*args: Any, **kwargs: Any)

   Bases: :py:obj:`sqlalchemy.TypeDecorator`


   Type that casts field which are of numpy nd array type


   .. py:attribute:: impl


   .. py:method:: process_bind_param(value: numpy.ndarray, dialect)

      Receive a bound parameter value to be converted.

      Custom subclasses of :class:`_types.TypeDecorator` should override
      this method to provide custom behaviors for incoming data values.
      This method is called at **statement execution time** and is passed
      the literal Python data value which is to be associated with a bound
      parameter in the statement.

      The operation could be anything desired to perform custom
      behavior, such as transforming or serializing data.
      This could also be used as a hook for validating logic.

      :param value: Data to operate upon, of any type expected by
       this method in the subclass.  Can be ``None``.
      :param dialect: the :class:`.Dialect` in use.

      .. seealso::

          :ref:`types_typedecorator`

          :meth:`_types.TypeDecorator.process_result_value`




   .. py:method:: process_result_value(value: impl, dialect) -> typing_extensions.Optional[numpy.ndarray]

      Receive a result-row column value to be converted.

      Custom subclasses of :class:`_types.TypeDecorator` should override
      this method to provide custom behaviors for data values
      being received in result rows coming from the database.
      This method is called at **result fetching time** and is passed
      the literal Python data value that's extracted from a database result
      row.

      The operation could be anything desired to perform custom
      behavior, such as transforming or deserializing data.

      :param value: Data to operate upon, of any type expected by
       this method in the subclass.  Can be ``None``.
      :param dialect: the :class:`.Dialect` in use.

      .. seealso::

          :ref:`types_typedecorator`

          :meth:`_types.TypeDecorator.process_bind_param`





