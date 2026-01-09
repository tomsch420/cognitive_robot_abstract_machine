pycram.designator
=================

.. py:module:: pycram.designator


Exceptions
----------

.. autoapisummary::

   pycram.designator.DesignatorError
   pycram.designator.ResolutionError


Classes
-------

.. autoapisummary::

   pycram.designator.DesignatorDescription
   pycram.designator.LocationDesignatorDescription
   pycram.designator.ObjectDesignatorDescription
   pycram.designator.EQLObjectDesignator
   pycram.designator.NamedObject


Module Contents
---------------

.. py:exception:: DesignatorError(*args, **kwargs)

   Bases: :py:obj:`Exception`


   Implementation of designator_description errors.


.. py:exception:: ResolutionError(missing_properties: typing_extensions.List[str], wrong_type: typing_extensions.Dict, current_type: typing_extensions.Any, designator: DesignatorDescription)

   Bases: :py:obj:`Exception`


   Common base class for all non-exit exceptions.


   .. py:attribute:: error
      :value: Multiline-String

      .. raw:: html

         <details><summary>Show Value</summary>

      .. code-block:: python

         """
         Some required properties where missing or had the wrong type when grounding the Designator: Uninferable.
         """

      .. raw:: html

         </details>




   .. py:attribute:: missing
      :value: Multiline-String

      .. raw:: html

         <details><summary>Show Value</summary>

      .. code-block:: python

         """The missing properties where: Uninferable
         """

      .. raw:: html

         </details>




   .. py:attribute:: wrong
      :value: Multiline-String

      .. raw:: html

         <details><summary>Show Value</summary>

      .. code-block:: python

         """The properties with the wrong type along with the current -and right type :
         """

      .. raw:: html

         </details>




   .. py:attribute:: head
      :value: Multiline-String

      .. raw:: html

         <details><summary>Show Value</summary>

      .. code-block:: python

         """Property   |   Current Type    |     Right Type
         -------------------------------------------------------------
         """

      .. raw:: html

         </details>




   .. py:attribute:: tab
      :value: ''



   .. py:attribute:: message
      :value: Multiline-String

      .. raw:: html

         <details><summary>Show Value</summary>

      .. code-block:: python

         """
         Some required properties where missing or had the wrong type when grounding the Designator: Uninferable.
         """

      .. raw:: html

         </details>




.. py:class:: DesignatorDescription

   .. py:attribute:: plan_node
      :type:  pycram.plan.PlanNode
      :value: None


      The plan node to which this designator_description belongs. This is assigned as soon as the designator_description is added to a plan.



   .. py:property:: plan
      :type: pycram.plan.Plan


      Returns the plan that this designator_description is part of.



   .. py:property:: robot_view
      :type: semantic_digital_twin.robots.abstract_robot.AbstractRobot


      Returns the robot that this designator_description is part of.



   .. py:property:: world
      :type: semantic_digital_twin.world.World


      Returns the world that this designator_description is part of.



   .. py:property:: context
      :type: pycram.datastructures.dataclasses.Context



   .. py:method:: resolve()


   .. py:method:: ground() -> typing_extensions.Any

      Should be overwritten with an actual grounding function which infers missing properties.



   .. py:method:: copy() -> DesignatorDescription


   .. py:method:: get_optional_parameter() -> typing_extensions.List[str]

      Returns a list of optional parameter names of this designator_description description.



   .. py:method:: get_all_parameter() -> typing_extensions.List[str]

      Returns a list of all parameter names of this designator_description description.



   .. py:method:: get_type_hints() -> typing_extensions.Dict[str, typing_extensions.Any]
      :classmethod:


      Returns the type hints of the __init__ method of this designator_description description.

      :return:



.. py:class:: LocationDesignatorDescription

   Bases: :py:obj:`DesignatorDescription`, :py:obj:`pycram.datastructures.partial_designator.PartialDesignator`


   Parent class of location designator_description descriptions.


   .. py:method:: ground() -> pycram.datastructures.pose.PoseStamped
      :abstractmethod:


      Find a location that satisfies all constrains.



.. py:class:: ObjectDesignatorDescription(names: typing_extensions.Optional[typing_extensions.List[str]] = None)

   Bases: :py:obj:`DesignatorDescription`, :py:obj:`pycram.datastructures.partial_designator.PartialDesignator`


   Class for object designator_description descriptions.
   Descriptions hold possible parameter ranges for object designators.


   .. py:attribute:: names
      :type:  typing_extensions.Optional[typing_extensions.List[str]]
      :value: None



   .. py:method:: ground() -> semantic_digital_twin.world_description.world_entity.Body

      Return the first object from the world that fits the description.

      :return: A executed object designator_description



   .. py:method:: __iter__() -> typing_extensions.Iterator[semantic_digital_twin.world_description.world_entity.Body]

      Iterate through all possible objects fitting this description

      :yield: A executed object designator_description



   .. py:method:: flatten() -> typing_extensions.List


.. py:class:: EQLObjectDesignator(eql_query)

   Bases: :py:obj:`DesignatorDescription`


   Description for objects found via an EQL query.


   .. py:attribute:: eql_query


   .. py:method:: __iter__() -> typing_extensions.Iterator[semantic_digital_twin.world_description.world_entity.Body]


.. py:class:: NamedObject(name: typing_extensions.Union[typing_extensions.Iterable[str], str])

   Bases: :py:obj:`ObjectDesignatorDescription`, :py:obj:`pycram.datastructures.partial_designator.PartialDesignator`


   Description for objects with a specific name.


   .. py:method:: __iter__() -> typing_extensions.Iterator[semantic_digital_twin.world_description.world_entity.Body]

      Iterate through all possible objects fitting this description

      :yield: A executed object designator_description



