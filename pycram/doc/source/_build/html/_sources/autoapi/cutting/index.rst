cutting
=======

.. py:module:: cutting


Attributes
----------

.. autoapisummary::

   cutting.sparql
   cutting.prefix
   cutting.verb
   cutting.foodobject


Functions
---------

.. autoapisummary::

   cutting.check_food_part
   cutting.get_prior_task
   cutting.get_cutting_tool
   cutting.get_cutting_position
   cutting.get_repetition
   cutting.get_peel_tool
   cutting.query_var


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
       PREFIX cut: <http://www.ease-crc.org/ont/meals#>
       PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
       PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
       PREFIX foodon: <http://purl.obolibrary.org/obo/>
       PREFIX soma: <http://www.ease-crc.org/ont/SOMA.owl#>
       PREFIX sit_aware: <http://www.ease-crc.org/ont/situation_awareness#>
       """

   .. raw:: html

      </details>



.. py:function:: check_food_part(food, part)

.. py:function:: get_prior_task(verb)

.. py:function:: get_cutting_tool(foodobject)

.. py:function:: get_cutting_position(verb)

.. py:function:: get_repetition(verb)

.. py:function:: get_peel_tool(foodobject)

.. py:function:: query_var(verb, foodobject)

.. py:data:: verb
   :value: 'cut:Quartering'


.. py:data:: foodobject
   :value: 'FOODON_00003523'


