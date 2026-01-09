pycram.ros.no_ros
=================

.. py:module:: pycram.ros.no_ros


Submodules
----------

.. toctree::
   :maxdepth: 1

   /autoapi/pycram/ros/no_ros/action_lib/index
   /autoapi/pycram/ros/no_ros/data_types/index
   /autoapi/pycram/ros/no_ros/publisher/index
   /autoapi/pycram/ros/no_ros/ros_tools/index
   /autoapi/pycram/ros/no_ros/service/index
   /autoapi/pycram/ros/no_ros/subscriber/index


Exceptions
----------

.. autoapisummary::

   pycram.ros.no_ros.ResourceNotFound
   pycram.ros.no_ros.ServiceException


Classes
-------

.. autoapisummary::

   pycram.ros.no_ros.Time


Functions
---------

.. autoapisummary::

   pycram.ros.no_ros.get_ros_package_path
   pycram.ros.no_ros.sleep
   pycram.ros.no_ros.get_node_names
   pycram.ros.no_ros.create_ros_pack
   pycram.ros.no_ros.get_parameter
   pycram.ros.no_ros.wait_for_message
   pycram.ros.no_ros.is_master_online
   pycram.ros.no_ros.get_time
   pycram.ros.no_ros.Duration
   pycram.ros.no_ros.Rate
   pycram.ros.no_ros.get_service_proxy
   pycram.ros.no_ros.wait_for_service
   pycram.ros.no_ros.create_action_client
   pycram.ros.no_ros.create_publisher
   pycram.ros.no_ros.create_subscriber


Package Contents
----------------

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


.. py:class:: Time(time=0.0, nsec=0.0)

   Class to abstract the ROS2 Time, to make it more consistent with the ROS1 Time class.


   .. py:attribute:: time
      :value: 0.0



   .. py:attribute:: nsec
      :value: 0.0



   .. py:method:: now()
      :classmethod:



   .. py:method:: to_sec()


.. py:function:: Duration(duration=0.0)

.. py:function:: Rate(rate)

.. py:exception:: ServiceException(message: str)

   Bases: :py:obj:`Exception`


   Exception class for service exceptions.


   .. py:attribute:: message


.. py:function:: get_service_proxy(topic_name: str, service_message) -> None

   Get a service proxy for a given topic name and service message type.

   :param topic_name: The name of the service.
   :param service_message: The type of the service message.
   :return: A service proxy for the specified topic and message type.


.. py:function:: wait_for_service(topic_name: str) -> None

   Wait for a service to become available.

   :param topic_name: The name of the service.


.. py:function:: create_action_client(topic_name: str, action_message) -> None

   Create an action client for the given topic name and action message type.

   Args:
       topic_name (str): The name of the action topic.
       action_message: The action message type.

   Returns:
       None


.. py:function:: create_publisher(topic, msg_type, queue_size=10, latch: bool = False) -> None

   Create a ROS publisher.

   :param topic: The name of the topic to publish to.
   :param msg_type: The type of message to publish.
   :param queue_size: The size of the message queue.
   :param latch: Whether to latch the last message sent.
   :return: A ROS publisher object.


.. py:function:: create_subscriber(topic, msg_type, callback, queue_size=10)

   Create a ROS subscriber.

   :param topic: The name of the topic to subscribe to.
   :param msg_type: The type of message to subscribe to.
   :param callback: The callback function to call when a message is received.
   :param queue_size: The size of the message queue.
   :return: A ROS subscriber object.


