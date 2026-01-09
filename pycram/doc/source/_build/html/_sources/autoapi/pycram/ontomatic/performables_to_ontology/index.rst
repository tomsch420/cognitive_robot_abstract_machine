pycram.ontomatic.performables_to_ontology
=========================================

.. py:module:: pycram.ontomatic.performables_to_ontology


Classes
-------

.. autoapisummary::

   pycram.ontomatic.performables_to_ontology.ParameterDigest
   pycram.ontomatic.performables_to_ontology.ActionAbstractDigest


Functions
---------

.. autoapisummary::

   pycram.ontomatic.performables_to_ontology.create_ontology_from_performables


Module Contents
---------------

.. py:class:: ParameterDigest

   Encapsulation of meta information about a parameter.


   .. py:attribute:: clazz
      :type:  typing_extensions.Any

      Class of the parameter.



   .. py:attribute:: name
      :type:  str

      Name of the parameter.



   .. py:attribute:: docstring
      :type:  str

      Docstring of the parameter itself (individual to each performable).



   .. py:attribute:: default_value
      :type:  typing_extensions.Optional[typing_extensions.Any]

      Holds the default value of the parameter if set.



   .. py:property:: docstring_of_clazz
      :type: str



   .. py:property:: is_enum
      :type: bool



   .. py:property:: is_optional
      :type: bool



   .. py:method:: get_default_value() -> typing_extensions.Optional[typing_extensions.List[str]]

      :return: A list containing the string representation of the default value or
          `None` if no default value exists.



.. py:class:: ActionAbstractDigest(clazz: typing_extensions.Type[pycram.robot_plans.actions.base.ActionDescription])

   Wrap all information about an action abstract class that are necessary for the created ontology.


   .. py:attribute:: clazz
      :type:  typing_extensions.Type[pycram.robot_plans.actions.base.ActionDescription]


   .. py:attribute:: full_name
      :type:  str


   .. py:attribute:: classname
      :type:  str


   .. py:attribute:: docstring
      :type:  str


   .. py:attribute:: parameters
      :type:  typing_extensions.Optional[typing_extensions.List[ParameterDigest]]
      :value: []



   .. py:method:: extract_dataclass_parameter_information() -> typing_extensions.List[ParameterDigest]

      Extract information about dataclass parameters from a dataclass.

      :return: List of parameter information.



.. py:function:: create_ontology_from_performables(output_path: pathlib.Path = './performables.owl', abstract_actions_to_parse: typing_extensions.Union[typing_extensions.List[typing_extensions.Type[pycram.robot_plans.actions.base.ActionDescription]], typing_extensions.Type[pycram.robot_plans.actions.base.ActionDescription]] = None) -> None

   Create an ontology from the performables.

   :param output_path: Path of the output ontology file.
   :param abstract_actions_to_parse: ActionAbstract classes to parse.
   If not set, all subclasses of ActionAbstract will be parsed.


