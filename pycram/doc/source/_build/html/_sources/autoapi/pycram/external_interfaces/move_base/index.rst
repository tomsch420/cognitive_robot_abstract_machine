pycram.external_interfaces.move_base
====================================

.. py:module:: pycram.external_interfaces.move_base


Attributes
----------

.. autoapisummary::

   pycram.external_interfaces.move_base.logger
   pycram.external_interfaces.move_base.nav_action_client
   pycram.external_interfaces.move_base.is_init


Functions
---------

.. autoapisummary::

   pycram.external_interfaces.move_base.create_nav_action_client
   pycram.external_interfaces.move_base.init_nav_interface
   pycram.external_interfaces.move_base.query_pose_nav
   pycram.external_interfaces.move_base.cancel_nav


Module Contents
---------------

.. py:data:: logger

.. py:data:: nav_action_client
   :value: None


.. py:data:: is_init
   :value: False


.. py:function:: create_nav_action_client()

   Creates a new action client for the move_base interface.


.. py:function:: init_nav_interface(func: Callable) -> Callable

   Ensures initialization of the navigation interface before function execution.


.. py:function:: query_pose_nav(navpose: PoseStamped)

   Sends a goal to the move_base service, initiating robot navigation to a given pose.


.. py:function:: cancel_nav()

   Cancels the current navigation goal.


