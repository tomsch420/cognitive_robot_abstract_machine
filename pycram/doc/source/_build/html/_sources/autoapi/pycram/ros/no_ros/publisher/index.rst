pycram.ros.no_ros.publisher
===========================

.. py:module:: pycram.ros.no_ros.publisher


Functions
---------

.. autoapisummary::

   pycram.ros.no_ros.publisher.create_publisher


Module Contents
---------------

.. py:function:: create_publisher(topic, msg_type, queue_size=10, latch: bool = False) -> None

   Create a ROS publisher.

   :param topic: The name of the topic to publish to.
   :param msg_type: The type of message to publish.
   :param queue_size: The size of the message queue.
   :param latch: Whether to latch the last message sent.
   :return: A ROS publisher object.


