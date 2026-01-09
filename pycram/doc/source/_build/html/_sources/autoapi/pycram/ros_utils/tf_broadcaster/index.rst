pycram.ros_utils.tf_broadcaster
===============================

.. py:module:: pycram.ros_utils.tf_broadcaster


Classes
-------

.. autoapisummary::

   pycram.ros_utils.tf_broadcaster.TFBroadcaster


Module Contents
---------------

.. py:class:: TFBroadcaster(world: semantic_digital_twin.world.World, node, projection_namespace=ExecutionType.SIMULATED, odom_frame='odom', interval=0.1)

   Broadcaster that publishes TF frames for every object in the World.


   .. py:attribute:: world


   .. py:attribute:: node


   .. py:attribute:: tf_static_publisher


   .. py:attribute:: tf_publisher


   .. py:attribute:: thread


   .. py:attribute:: kill_event


   .. py:attribute:: interval
      :value: 0.1



   .. py:attribute:: projection_namespace


   .. py:attribute:: odom_frame
      :value: 'odom'



   .. py:method:: update()

      Updates the TFs for the static odom frame and all objects currently in the World.



   .. py:method:: _update_objects() -> None

      Publishes the current pose of all objects in the World. As well as the poses of all links of these objects.



   .. py:method:: _update_static_odom() -> None

      Publishes a static odom frame to the tf_static topic.



   .. py:method:: _publish_pose(child_frame_id: str, pose: pycram.datastructures.pose.PoseStamped, static=False) -> None

      Publishes the given pose to the ROS TF topic. First the pose is converted to a Transform between pose.frame and
      the given child_frame_id. Afterward, the frames of the Transform are prefixed with the projection namespace.

      :param child_frame_id: Name of the TF frame which the pose points to
      :param pose: Pose that should be published
      :param static: If the pose should be published to the tf_static topic



   .. py:method:: _publish() -> None

      Constantly publishes the positions of all objects in the World.



   .. py:method:: _stop_publishing() -> None

      Called when the process ends, sets the kill_event which terminates the thread that publishes to the TF topic.



