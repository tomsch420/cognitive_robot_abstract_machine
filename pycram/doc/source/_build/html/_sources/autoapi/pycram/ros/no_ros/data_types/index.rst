pycram.ros.no_ros.data_types
============================

.. py:module:: pycram.ros.no_ros.data_types


Exceptions
----------

.. autoapisummary::

   pycram.ros.no_ros.data_types.ServiceException


Classes
-------

.. autoapisummary::

   pycram.ros.no_ros.data_types.Time


Functions
---------

.. autoapisummary::

   pycram.ros.no_ros.data_types.Duration
   pycram.ros.no_ros.data_types.Rate


Module Contents
---------------

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


