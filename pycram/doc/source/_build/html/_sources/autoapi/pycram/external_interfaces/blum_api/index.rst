pycram.external_interfaces.blum_api
===================================

.. py:module:: pycram.external_interfaces.blum_api


Attributes
----------

.. autoapisummary::

   pycram.external_interfaces.blum_api.logger


Functions
---------

.. autoapisummary::

   pycram.external_interfaces.blum_api.init_kitchen_interface
   pycram.external_interfaces.blum_api.authenticate_user
   pycram.external_interfaces.blum_api.call_kitchen_service
   pycram.external_interfaces.blum_api.open_cabinet
   pycram.external_interfaces.blum_api.close_cabinet
   pycram.external_interfaces.blum_api.modules_show
   pycram.external_interfaces.blum_api.kitchen_state


Module Contents
---------------

.. py:data:: logger

.. py:function:: init_kitchen_interface(func: Callable) -> Callable

   Tries to import the messages and initialize interface.


.. py:function:: authenticate_user()

   Authenticates a user at the bridge


.. py:function:: call_kitchen_service(command, argument)

.. py:function:: open_cabinet(kitchen_element)

   Opens a cabinet of the apartment kitchen

   :param kitchen_element: The cabinet which should be opened


.. py:function:: close_cabinet(kitchen_element)

   Closes a cabinet of the apartment kitchen, currently this is only possible for the "oberschrank"

   :param kitchen_element: Cabinet which should be closed


.. py:function:: modules_show()

   Shows all possible cabinet modules that can be called
   :return:


.. py:function:: kitchen_state() -> typing_extensions.Dict[str, typing_extensions.Dict[str, str]]

   Returns the state of the whole kitchen as a dictionary. The mapping is "name": {"state", "type"}

   :return: A dict describing the kitchen state


