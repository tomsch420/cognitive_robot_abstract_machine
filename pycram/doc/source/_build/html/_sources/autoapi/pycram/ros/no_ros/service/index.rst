pycram.ros.no_ros.service
=========================

.. py:module:: pycram.ros.no_ros.service


Functions
---------

.. autoapisummary::

   pycram.ros.no_ros.service.get_service_proxy
   pycram.ros.no_ros.service.wait_for_service


Module Contents
---------------

.. py:function:: get_service_proxy(topic_name: str, service_message) -> None

   Get a service proxy for a given topic name and service message type.

   :param topic_name: The name of the service.
   :param service_message: The type of the service message.
   :return: A service proxy for the specified topic and message type.


.. py:function:: wait_for_service(topic_name: str) -> None

   Wait for a service to become available.

   :param topic_name: The name of the service.


