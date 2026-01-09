pycram.ros.ros2.data_types
==========================

.. py:module:: pycram.ros.ros2.data_types


Classes
-------

.. autoapisummary::

   pycram.ros.ros2.data_types.Time


Functions
---------

.. autoapisummary::

   pycram.ros.ros2.data_types.Duration
   pycram.ros.ros2.data_types.Rate


Module Contents
---------------

.. py:class:: Time(time=0, nsecs=0)

   Bases: :py:obj:`builtin_interfaces.msg.Time`


   Class to abstract the ROS2 Time, to make it more consistent with the ROS1 Time class.


   .. py:method:: now(node)
      :classmethod:



   .. py:method:: to_sec()


.. py:function:: Duration(duration=0.0)

.. py:function:: Rate(rate)

