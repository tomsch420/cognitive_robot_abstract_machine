pycram.motion_executor
======================

.. py:module:: pycram.motion_executor


Attributes
----------

.. autoapisummary::

   pycram.motion_executor.logger


Classes
-------

.. autoapisummary::

   pycram.motion_executor.MotionExecutor


Module Contents
---------------

.. py:data:: logger

.. py:class:: MotionExecutor

   .. py:attribute:: motions
      :type:  List[giskardpy.motion_statechart.graph_node.Task]

      The motions to execute



   .. py:attribute:: world
      :type:  semantic_digital_twin.world.World

      The world in which the motions should be executed.



   .. py:attribute:: motion_state_chart
      :type:  giskardpy.motion_statechart.motion_statechart.MotionStatechart

      Giskard's motion state chart that is created from the motions.



   .. py:attribute:: ros_node
      :type:  Any
      :value: None


      ROS node that should be used for communication. Only relevant for real execution.



   .. py:method:: construct_msc()


   .. py:method:: execute()

      Executes the constructed motion state chart in the given world.



   .. py:method:: _execute_for_simulation()

      Creates an executor and executes the motion state chart until it is done.



   .. py:method:: _execute_for_real()


