pycram.ros.no_ros.ros_tools
===========================

.. py:module:: pycram.ros.no_ros.ros_tools


Exceptions
----------

.. autoapisummary::

   pycram.ros.no_ros.ros_tools.ResourceNotFound


Functions
---------

.. autoapisummary::

   pycram.ros.no_ros.ros_tools.get_ros_package_path
   pycram.ros.no_ros.ros_tools.sleep
   pycram.ros.no_ros.ros_tools.get_node_names
   pycram.ros.no_ros.ros_tools.create_ros_pack
   pycram.ros.no_ros.ros_tools.get_parameter
   pycram.ros.no_ros.ros_tools.wait_for_message
   pycram.ros.no_ros.ros_tools.is_master_online
   pycram.ros.no_ros.ros_tools.get_time


Module Contents
---------------

.. py:function:: get_ros_package_path(package_name: str) -> str

   Get the path of a ROS package. Using the os module to avoid importing rospkg.


.. py:function:: sleep(duration: float)

.. py:function:: get_node_names(namespace=None)

   Get the names of all nodes in the ROS system.


.. py:function:: create_ros_pack(ros_paths=None)

   Creates a RosPack instance to search for resources of ros packages.


.. py:exception:: ResourceNotFound

   Bases: :py:obj:`Exception`


   Exception raised when a resource is not found.


.. py:function:: get_parameter(name: str)

   Get a parameter from the ROS parameter server.


.. py:function:: wait_for_message(topic_name: str, msg_type)

   Wait for a message on a topic.


.. py:function:: is_master_online()

   Check if the ROS master is online.


.. py:function:: get_time()

   Get the current time from the ROS system.


