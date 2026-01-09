pouring
=======

.. py:module:: pouring


Attributes
----------

.. autoapisummary::

   pouring.sparql
   pouring.prefix
   pouring.verb
   pouring.foodobject


Functions
---------

.. autoapisummary::

   pouring.get_needed_tool
   pouring.get_min_angle
   pouring.get_max_angle
   pouring.get_min_duration
   pouring.get_max_duration
   pouring.query_var


Module Contents
---------------

.. py:data:: sparql

.. py:data:: prefix
   :value: Multiline-String

   .. raw:: html

      <details><summary>Show Value</summary>

   .. code-block:: python

      """
       PREFIX owl: <http://www.w3.org/2002/07/owl#>
       prefix dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#>
       PREFIX pour: <http://www.ease-crc.org/ont/meals#>
       PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
       PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
       PREFIX foodon: <http://purl.obolibrary.org/obo/>
       PREFIX soma: <http://www.ease-crc.org/ont/SOMA.owl#>
       PREFIX sit_aware: <http://www.ease-crc.org/ont/situation_awareness#>
       PREFIX obo: <http://purl.obolibrary.org/obo/>
       PREFIX qudt: <http://qudt.org/schema/qudt#>
       """

   .. raw:: html

      </details>



.. py:function:: get_needed_tool(verb)

.. py:function:: get_min_angle(foodobject)

.. py:function:: get_max_angle(foodobject)

.. py:function:: get_min_duration(foodobject)

.. py:function:: get_max_duration(foodobject)

.. py:function:: query_var(verb, foodobject)

.. py:data:: verb
   :value: 'pour:Draining'


.. py:data:: foodobject
   :value: 'obo:FOODON_03301304'


