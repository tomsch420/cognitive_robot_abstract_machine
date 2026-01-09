pycram.ros_utils.joint_state_publisher
======================================

.. py:module:: pycram.ros_utils.joint_state_publisher


Classes
-------

.. autoapisummary::

   pycram.ros_utils.joint_state_publisher.JointStatePublisher


Module Contents
---------------

.. py:class:: JointStatePublisher(world: semantic_digital_twin.world.World, node, joint_state_topic='/pycram/joint_state', interval=0.1)

   Joint state publisher for all robots currently loaded in the World


   .. py:attribute:: world


   .. py:attribute:: joint_state_pub


   .. py:attribute:: node


   .. py:attribute:: interval
      :value: 0.1



   .. py:attribute:: kill_event


   .. py:attribute:: thread


   .. py:method:: _publish() -> None

      Publishes the current joint states of the :py:attr:`~pycram.world.World.robot` in an infinite loop.
      The joint states are published as long as the kill_event is not set by :py:meth:`~JointStatePublisher._stop_publishing`



   .. py:method:: _stop_publishing() -> None

      Sets the kill_event to terminate the publishing thread and joins the thread.



