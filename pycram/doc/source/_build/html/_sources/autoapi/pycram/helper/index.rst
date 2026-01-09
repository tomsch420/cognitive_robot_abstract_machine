pycram.helper
=============

.. py:module:: pycram.helper

.. autoapi-nested-parse::

   Implementation of helper functions and classes for internal usage only.

   Classes:
   Singleton -- implementation of singleton metaclass



Attributes
----------

.. autoapisummary::

   pycram.helper.logger


Classes
-------

.. autoapisummary::

   pycram.helper.Singleton


Functions
---------

.. autoapisummary::

   pycram.helper.get_robot_urdf_and_mjcf_file_paths
   pycram.helper.parse_mjcf_actuators
   pycram.helper.get_robot_description_path
   pycram.helper.find_multiverse_resources_path
   pycram.helper.find_multiverse_path
   pycram.helper.perform
   pycram.helper.an


Module Contents
---------------

.. py:data:: logger

.. py:class:: Singleton

   Bases: :py:obj:`type`


   Metaclass for singletons


   .. py:attribute:: _instances

      Dictionary of singleton child classes inheriting from this metaclass, keyed by child class objects.



   .. py:method:: __call__(*args, **kwargs)


.. py:function:: get_robot_urdf_and_mjcf_file_paths(robot_name: str, robot_relative_dir: str, multiverse_resources: typing_extensions.Optional[str] = None) -> typing_extensions.Tuple[typing_extensions.Optional[str], typing_extensions.Optional[str]]

   Get the paths to the MJCF and URDF files of a robot from the Multiverse resources directory.

   :param robot_name: The name of the robot.
   :param robot_relative_dir: The relative directory of the robot in the Multiverse resources/robots directory.
   :param multiverse_resources: The path to the Multiverse resources directory.


.. py:function:: parse_mjcf_actuators(file_path: str) -> typing_extensions.Dict[str, str]

   Parse the actuator elements from an MJCF file.

   :param file_path: The path to the MJCF file.


.. py:function:: get_robot_description_path(robot_relative_dir: str, robot_name: str, description_type: pycram.datastructures.enums.DescriptionType = DescriptionType.MJCF, file_name: typing_extensions.Optional[str] = None, resources_dir: typing_extensions.Optional[str] = None) -> typing_extensions.Optional[str]

   Get the path to the description file of a robot.

   :param robot_relative_dir: The relative directory of the robot in the resources/robots directory.
   :param robot_name: The name of the robot.
   :param description_type: The type of the description (URDF, MJCF).
   :param file_name: The name of the XML file of the robot.
   :param resources_dir: The path to the Multiverse resources directory.
   :return: The path to the description file of the robot if it exists, otherwise None.


.. py:function:: find_multiverse_resources_path() -> typing_extensions.Optional[str]

   :return: The path to the Multiverse resources directory.


.. py:function:: find_multiverse_path() -> typing_extensions.Optional[str]

   :return: the path to the Multiverse installation.


.. py:function:: perform(action_instance)

   Executes the perform logic for a given action instance.

   :param action_instance: An instance of an action class.


.. py:function:: an(designator)

   Resolve the first available action from the designator.

   :param designator: The designator description instance.
   :return: The first resolved action instance.


