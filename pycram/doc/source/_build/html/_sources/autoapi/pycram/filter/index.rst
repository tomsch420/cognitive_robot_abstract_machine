pycram.filter
=============

.. py:module:: pycram.filter


Classes
-------

.. autoapisummary::

   pycram.filter.Filter
   pycram.filter.Butterworth


Module Contents
---------------

.. py:class:: Filter

   Abstract class to ensure that every supported filter needs to implement the filter method.

   :method filter: Abstract method to filter the given data.


   .. py:method:: filter(data)
      :abstractmethod:



.. py:class:: Butterworth(order=4, cutoff=10, fs=60)

   Bases: :py:obj:`Filter`


   Implementation for a Butterworth filter.

   :param order: The order of the filter (default is 4).
   :param cutoff: The cutoff frequency of the filter (default is 10).
   :param fs: The sampling frequency of the data (default is 60).


   .. py:attribute:: order
      :value: 4



   .. py:attribute:: cutoff
      :value: 10



   .. py:attribute:: fs
      :value: 60



   .. py:method:: filter(data: list)

      Filters the given data using a Butterworth filter.

      :param data: The data to be filtered.

      :return: The filtered data.



