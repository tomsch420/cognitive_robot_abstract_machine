pycram.ros.ros1.data_types
==========================

.. py:module:: pycram.ros.ros1.data_types


Functions
---------

.. autoapisummary::

   pycram.ros.ros1.data_types.Time
   pycram.ros.ros1.data_types.Duration
   pycram.ros.ros1.data_types.Rate


Module Contents
---------------

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


