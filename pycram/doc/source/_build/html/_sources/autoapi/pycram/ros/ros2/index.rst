pycram.ros.ros2
===============

.. py:module:: pycram.ros.ros2


Submodules
----------

.. toctree::
   :maxdepth: 1

   /autoapi/pycram/ros/ros2/action_lib/index
   /autoapi/pycram/ros/ros2/data_types/index
   /autoapi/pycram/ros/ros2/filter/index
   /autoapi/pycram/ros/ros2/publisher/index
   /autoapi/pycram/ros/ros2/ros_tools/index
   /autoapi/pycram/ros/ros2/service/index
   /autoapi/pycram/ros/ros2/subscriber/index
   /autoapi/pycram/ros/ros2/viz_marker_publisher/index


Attributes
----------

.. autoapisummary::

   pycram.ros.ros2.logger
   pycram.ros.ros2.logger
   pycram.ros.ros2.services


Exceptions
----------

.. autoapisummary::

   pycram.ros.ros2.ResourceNotFound
   pycram.ros.ros2.ServiceException


Classes
-------

.. autoapisummary::

   pycram.ros.ros2.Time
   pycram.ros.ros2.ServiceProxy


Functions
---------

.. autoapisummary::

   pycram.ros.ros2.to_sec
   pycram.ros.ros2.Duration
   pycram.ros.ros2.Rate
   pycram.ros.ros2.get_node_names
   pycram.ros.ros2.create_ros_pack
   pycram.ros.ros2.get_ros_package_path
   pycram.ros.ros2.get_parameter
   pycram.ros.ros2.wait_for_message
   pycram.ros.ros2.is_master_online
   pycram.ros.ros2.sleep
   pycram.ros.ros2.get_time
   pycram.ros.ros2.create_timer
   pycram.ros.ros2.create_action_client
   pycram.ros.ros2.get_service_proxy
   pycram.ros.ros2.wait_for_service
   pycram.ros.ros2.create_publisher
   pycram.ros.ros2.create_subscriber


Package Contents
----------------

.. py:function:: to_sec(self)

   Returns the time in seconds from a builtin_interfaces.msg.Time message.

   :return: The time in seconds.


.. py:class:: Time(time=0, nsecs=0)

   Bases: :py:obj:`builtin_interfaces.msg.Time`


   Class to abstract the ROS2 Time, to make it more consistent with the ROS1 Time class.


   .. py:method:: now(node)
      :classmethod:



   .. py:method:: to_sec()


.. py:function:: Duration(duration=0.0)

.. py:function:: Rate(rate)

.. py:data:: logger

.. py:function:: get_node_names(node, namespace=None)

   Get the names of all nodes in the ROS system.

   :param namespace: The namespace to search for nodes.
   :return: A list of node names.


.. py:function:: create_ros_pack(ros_paths=None)

.. py:function:: get_ros_package_path(package_name)

.. py:function:: get_parameter(name, node)

.. py:function:: wait_for_message(topic_name, msg_type, node)

.. py:function:: is_master_online(node)

.. py:function:: sleep(duration, node)

   Sleep for a given duration.

   :param duration: The duration to sleep in seconds.


.. py:function:: get_time(node)

.. py:function:: create_timer(duration, callback, node, oneshot=False)

.. py:exception:: ResourceNotFound(*args, **kwargs)

   Bases: :py:obj:`Exception`


   Common base class for all non-exit exceptions.


.. py:exception:: ServiceException(*args, **kwargs)

   Bases: :py:obj:`Exception`


   Common base class for all non-exit exceptions.


.. py:function:: create_action_client(topic_name: str, action_message, node) -> rclpy.action.ActionClient

.. py:data:: logger

.. py:data:: services

.. py:class:: ServiceProxy(topic_name, service_message, node)

   .. py:attribute:: service


   .. py:attribute:: message_type


   .. py:method:: __call__(*args, **kwargs)


   .. py:method:: wait_for_service(*args, **kwargs)


.. py:function:: get_service_proxy(topic_name: str, service_message) -> rclpy.client.Client

.. py:function:: wait_for_service(topic_name: str, service_message)

.. py:function:: create_publisher(topic, msg_type, node, queue_size=10) -> rclpy.publisher.Publisher

.. py:function:: create_subscriber(topic, msg_type, callback, node, queue_size=10) -> rclpy.subscription.Subscription

