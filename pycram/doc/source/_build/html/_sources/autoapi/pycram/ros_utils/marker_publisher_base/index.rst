pycram.ros_utils.marker_publisher_base
======================================

.. py:module:: pycram.ros_utils.marker_publisher_base


Classes
-------

.. autoapisummary::

   pycram.ros_utils.marker_publisher_base.MarkerPublisherBase


Module Contents
---------------

.. py:class:: MarkerPublisherBase

   Bases: :py:obj:`abc.ABC`


   Base class for publishing visualization markers of specific data in ROS/RViz.

   This class provides common utilities for creating and publishing various types of
   markers, such as arrows, cubes, lines, and geometry-based visualizations. Subclasses
   must implement the `visualize` method to define how their specific data is converted
   into `Marker` objects.


   .. py:attribute:: topic
      :type:  str

      The name of the topic to publish to



   .. py:property:: publisher
      :type: rclpy.publisher.Publisher


      Create a publisher that publishes data to topic.

      returns: The created publisher.



   .. py:method:: visualize(data: object) -> None
      :abstractmethod:


      Subclasses must implement this to create and publish a MarkerArray



   .. py:method:: _create_arrow_marker(p1: geometry_msgs.msg.Point, p2: geometry_msgs.msg.Point, index: int, frame_id: str, color: Optional[List] = (1.0, 0.0, 1.0, 1.0), scale: Optional[List] = (0.01, 0.02, 0.02), duration: Optional[int] = 60) -> visualization_msgs.msg.Marker

      Create an arrow marker.

      :param p1: first point of the arrow
      :param p2: second point of the arrow
      :param index: index of the created arrow
      :param frame_id: frame id of the created arrow marker
      :param color: color of the created arrow marker
      :param scale: scale of the created arrow marker
      :param duration: duration of visualizing the arrow

      returns: The created arrow marker.



   .. py:method:: _create_cube_marker(box: pycram.datastructures.dataclasses.BoundingBox, index: int, duration: Optional[int] = 60) -> visualization_msgs.msg.Marker

      Create a cube marker.

      :param box: the bounding box the cube marker should be drawn on
      :param index: index of the created cube
      :param duration: duration of visualizing the cube marker

      returns: The created cube marker.



   .. py:method:: _create_line_marker(pose: pycram.datastructures.pose.PoseStamped, axis: List[float], color_rgba: List[float], index: int, duration: Optional[int] = 60, length: Optional[float] = 0.1, width: Optional[float] = 0.02) -> visualization_msgs.msg.Marker

      Create a line marker.

      :param pose: the pose of the line
      :param axis: the given axis
      :param color_rgba: the color of the created line marker
      :param index: id of the created line marker
      :param duration: duration of visualizing the line marker
      :param length: length of the axis
      :param width: width of the axis

      returns: The created line marker.



   .. py:method:: _create_geometry_marker(geom: pycram.datastructures.dataclasses.MeshVisualShape | pycram.datastructures.dataclasses.CylinderVisualShape | pycram.datastructures.dataclasses.BoxVisualShape | pycram.datastructures.dataclasses.SphereVisualShape, obj: semantic_digital_twin.world_description.world_entity.Body, link: str, i: int, link_pose_with_origin: pycram.datastructures.pose.TransformStamped, reference_frame: str, use_prospection_world: Optional[bool] = False) -> visualization_msgs.msg.Marker

      Creates a Marker for the given geometry type.

      :param geom: the geometry to create the marker for
      :param obj: the object to create the marker for
      :param link: the link to create the marker for
      :param i: the id of the created marker
      :param link_pose_with_origin: the pose of the link
      :param reference_frame: the reference frame of the link
      :param use_prospection_world: uif prospection world is used or not

      returns: the created marker.



   .. py:method:: _create_object_marker(pose: pycram.datastructures.pose.PoseStamped, name: str, path: str, current_id: int) -> visualization_msgs.msg.Marker

      Create a marker for an object.

      :param pose: the pose of the object
      :param name: the name of the object
      :param path: the path of the object
      :param current_id: the id of the current object

      returns: The created marker.



