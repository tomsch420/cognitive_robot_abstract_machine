pycram.external_interfaces.robokudo
===================================

.. py:module:: pycram.external_interfaces.robokudo


Attributes
----------

.. autoapisummary::

   pycram.external_interfaces.robokudo.logger
   pycram.external_interfaces.robokudo.robokudo_found
   pycram.external_interfaces.robokudo.is_init
   pycram.external_interfaces.robokudo.client
   pycram.external_interfaces.robokudo.number_of_par_goals
   pycram.external_interfaces.robokudo.robokudo_lock
   pycram.external_interfaces.robokudo.robokudo_rlock
   pycram.external_interfaces.robokudo.par_threads


Functions
---------

.. autoapisummary::

   pycram.external_interfaces.robokudo.thread_safe
   pycram.external_interfaces.robokudo.init_robokudo_interface
   pycram.external_interfaces.robokudo.send_query
   pycram.external_interfaces.robokudo.query_all_objects
   pycram.external_interfaces.robokudo.query_object
   pycram.external_interfaces.robokudo.query_human
   pycram.external_interfaces.robokudo.stop_query
   pycram.external_interfaces.robokudo.query_specific_region
   pycram.external_interfaces.robokudo.query_human_attributes
   pycram.external_interfaces.robokudo.query_waving_human


Module Contents
---------------

.. py:data:: logger

.. py:data:: robokudo_found
   :value: False


.. py:data:: is_init
   :value: False


.. py:data:: client
   :value: None


.. py:data:: number_of_par_goals
   :value: 0


.. py:data:: robokudo_lock

.. py:data:: robokudo_rlock

.. py:data:: par_threads

.. py:function:: thread_safe(func: typing_extensions.Callable) -> typing_extensions.Callable

   Adds thread safety to a function via a decorator. This uses the robokudo_lock

   :param func: Function that should be thread safe
   :return: A function with thread safety


.. py:function:: init_robokudo_interface(func: typing_extensions.Callable) -> typing_extensions.Callable

   Checks if the ROS messages are available and if Robokudo is running, if that is the case the interface will be
   initialized.

   :param func: Function this decorator should be wrapping
   :return: A callable function which initializes the interface and then calls the wrapped function


.. py:function:: send_query(obj_type: typing_extensions.Optional[str] = None, region: typing_extensions.Optional[str] = None, attributes: typing_extensions.Optional[typing_extensions.List[str]] = None) -> Any

   Generic function to send a query to RoboKudo.


.. py:function:: query_all_objects() -> dict

   Query RoboKudo for all objects.


.. py:function:: query_object(obj_desc: pycram.designator.ObjectDesignatorDescription) -> dict

   Query RoboKudo for an object that fits the description.


.. py:function:: query_human() -> PointStamped

   Query RoboKudo for human detection and return the detected human's pose.


.. py:function:: stop_query()

   Stop any ongoing query to RoboKudo.


.. py:function:: query_specific_region(region: str) -> Any

   Query RoboKudo to scan a specific region.


.. py:function:: query_human_attributes() -> Any

   Query RoboKudo for human attributes like brightness of clothes, headgear, and gender.


.. py:function:: query_waving_human() -> pycram.datastructures.pose.PoseStamped

   Query RoboKudo for detecting a waving human.


