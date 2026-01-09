pycram.ros.no_ros.subscriber
============================

.. py:module:: pycram.ros.no_ros.subscriber


Functions
---------

.. autoapisummary::

   pycram.ros.no_ros.subscriber.create_subscriber


Module Contents
---------------

.. py:function:: create_subscriber(topic, msg_type, callback, queue_size=10)

   Create a ROS subscriber.

   :param topic: The name of the topic to subscribe to.
   :param msg_type: The type of message to subscribe to.
   :param callback: The callback function to call when a message is received.
   :param queue_size: The size of the message queue.
   :return: A ROS subscriber object.


