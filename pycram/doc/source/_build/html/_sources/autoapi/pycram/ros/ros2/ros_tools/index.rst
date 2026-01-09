pycram.ros.ros2.ros_tools
=========================

.. py:module:: pycram.ros.ros2.ros_tools


Attributes
----------

.. autoapisummary::

   pycram.ros.ros2.ros_tools.logger


Exceptions
----------

.. autoapisummary::

   pycram.ros.ros2.ros_tools.ResourceNotFound
   pycram.ros.ros2.ros_tools.ServiceException


Functions
---------

.. autoapisummary::

   pycram.ros.ros2.ros_tools.get_node_names
   pycram.ros.ros2.ros_tools.create_ros_pack
   pycram.ros.ros2.ros_tools.get_ros_package_path
   pycram.ros.ros2.ros_tools.get_parameter
   pycram.ros.ros2.ros_tools.wait_for_message
   pycram.ros.ros2.ros_tools.is_master_online
   pycram.ros.ros2.ros_tools.sleep
   pycram.ros.ros2.ros_tools.get_time
   pycram.ros.ros2.ros_tools.create_timer


Module Contents
---------------

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


