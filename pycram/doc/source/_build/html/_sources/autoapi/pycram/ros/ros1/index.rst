pycram.ros.ros1
===============

.. py:module:: pycram.ros.ros1


Submodules
----------

.. toctree::
   :maxdepth: 1

   /autoapi/pycram/ros/ros1/action_lib/index
   /autoapi/pycram/ros/ros1/data_types/index
   /autoapi/pycram/ros/ros1/publisher/index
   /autoapi/pycram/ros/ros1/ros_tools/index
   /autoapi/pycram/ros/ros1/service/index
   /autoapi/pycram/ros/ros1/subscriber/index
   /autoapi/pycram/ros/ros1/viz_marker_publisher/index


Classes
-------

.. autoapisummary::

   pycram.ros.ros1.VizMarkerPublisher


Functions
---------

.. autoapisummary::

   pycram.ros.ros1.is_master_online
   pycram.ros.ros1.Time
   pycram.ros.ros1.Duration
   pycram.ros.ros1.Rate
   pycram.ros.ros1.get_node_names
   pycram.ros.ros1.create_ros_pack
   pycram.ros.ros1.get_ros_package_path
   pycram.ros.ros1.get_parameter
   pycram.ros.ros1.wait_for_message
   pycram.ros.ros1.is_master_online
   pycram.ros.ros1.sleep
   pycram.ros.ros1.get_time
   pycram.ros.ros1.create_timer
   pycram.ros.ros1.create_action_client
   pycram.ros.ros1.get_service_proxy
   pycram.ros.ros1.wait_for_service
   pycram.ros.ros1.create_publisher
   pycram.ros.ros1.create_subscriber


Package Contents
----------------

.. py:function:: is_master_online()

.. py:function:: Time(time: int = 0.0, nsecs: int = 0) -> rospy.Time

   Wrapper for rospy.Time to create a Time object.

   :param time: Time in seconds
   :param nsecs: Time in nanoseconds
   :return: Rospy Time object representing the given time


.. py:function:: Duration(duration: float = 0.0) -> rospy.Duration

   Wrapper for rospy.Duration to create a Duration object.

   :param duration: Duration in seconds
   :return: A rospy Duration object representing the given duration


.. py:function:: Rate(rate: float) -> rospy.Rate

   Wrapper for rospy.Rate to create a Rate object.

   :param rate: Rate in Hz
   :return: A rospy Rate object representing the given rate


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

.. py:function:: create_action_client(topic_name: str, action_message) -> actionlib.SimpleActionClient

.. py:function:: get_service_proxy(topic_name: str, service_message) -> rospy.ServiceProxy

.. py:function:: wait_for_service(topic_name: str)

.. py:function:: create_publisher(topic, msg_type, queue_size=10, latch: bool = False) -> rospy.Publisher

.. py:function:: create_subscriber(topic, msg_type, callback, queue_size=10) -> rospy.Subscriber

.. py:class:: VizMarkerPublisher

