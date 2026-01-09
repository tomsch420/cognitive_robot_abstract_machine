pycram.ros.ros1.ros_tools
=========================

.. py:module:: pycram.ros.ros1.ros_tools


Functions
---------

.. autoapisummary::

   pycram.ros.ros1.ros_tools.get_node_names
   pycram.ros.ros1.ros_tools.create_ros_pack
   pycram.ros.ros1.ros_tools.get_ros_package_path
   pycram.ros.ros1.ros_tools.get_parameter
   pycram.ros.ros1.ros_tools.wait_for_message
   pycram.ros.ros1.ros_tools.is_master_online
   pycram.ros.ros1.ros_tools.sleep
   pycram.ros.ros1.ros_tools.get_time
   pycram.ros.ros1.ros_tools.create_timer


Module Contents
---------------

.. py:function:: get_node_names(namespace=None)

.. py:function:: create_ros_pack(ros_paths: typing_extensions.Any = None) -> rospkg.RosPack

   Creates a RosPack instance to search for resources of ros packages.

   :param ros_paths: An ordered list of paths to search for resources.
   :return: An instance of RosPack


.. py:function:: get_ros_package_path(package_name: str) -> str

.. py:function:: get_parameter(name: str) -> typing_extensions.Any

.. py:function:: wait_for_message(topic_name: str, msg_type: typing_extensions.Any)

.. py:function:: is_master_online()

.. py:function:: sleep(duration: float)

.. py:function:: get_time()

.. py:function:: create_timer(duration: rospy.Duration, callback, oneshot=False)

