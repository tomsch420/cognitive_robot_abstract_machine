pycram.ros.ros2.service
=======================

.. py:module:: pycram.ros.ros2.service


Attributes
----------

.. autoapisummary::

   pycram.ros.ros2.service.logger
   pycram.ros.ros2.service.services


Classes
-------

.. autoapisummary::

   pycram.ros.ros2.service.ServiceProxy


Functions
---------

.. autoapisummary::

   pycram.ros.ros2.service.get_service_proxy
   pycram.ros.ros2.service.wait_for_service


Module Contents
---------------

.. py:data:: logger

.. py:data:: services

.. py:class:: ServiceProxy(topic_name, service_message, node)

   .. py:attribute:: service


   .. py:attribute:: message_type


   .. py:method:: __call__(*args, **kwargs)


   .. py:method:: wait_for_service(*args, **kwargs)


.. py:function:: get_service_proxy(topic_name: str, service_message) -> rclpy.client.Client

.. py:function:: wait_for_service(topic_name: str, service_message)

