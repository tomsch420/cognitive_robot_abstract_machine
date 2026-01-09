pycram.datastructures.grasp
===========================

.. py:module:: pycram.datastructures.grasp


Classes
-------

.. autoapisummary::

   pycram.datastructures.grasp.GraspDescription
   pycram.datastructures.grasp.PreferredGraspAlignment


Module Contents
---------------

.. py:class:: GraspDescription

   Bases: :py:obj:`pycram.has_parameters.HasParameters`


   Represents a grasp description with a side grasp, top face, and orientation alignment.


   .. py:attribute:: approach_direction
      :type:  pycram.datastructures.enums.ApproachDirection

      The primary approach direction. 



   .. py:attribute:: vertical_alignment
      :type:  pycram.datastructures.enums.VerticalAlignment

      The vertical alignment when grasping the pose



   .. py:attribute:: rotate_gripper
      :type:  bool
      :value: False


      Indicates if the gripper should be rotated by 90°. Must be a boolean.



   .. py:method:: __hash__()


   .. py:method:: as_list() -> typing_extensions.List[typing_extensions.Union[pycram.datastructures.enums.Grasp, typing_extensions.Optional[pycram.datastructures.enums.Grasp], bool]]

      :return: A list representation of the grasp description.



   .. py:method:: get_grasp_pose(end_effector: semantic_digital_twin.robots.abstract_robot.Manipulator, body: semantic_digital_twin.world_description.world_entity.Body, translate_rim_offset: bool = False) -> pycram.datastructures.pose.PoseStamped

      Translates the grasp pose of the object using the desired grasp description and object knowledge.
      Leaves the orientation untouched.
      Returns the translated grasp pose.

      :param end_effector: The end effector that will be used to grasp the object.
      :param body: The body of the object to be grasped.
      :param translate_rim_offset: If True, the grasp pose will be translated along the rim offset.

      :return: The grasp pose of the object.



   .. py:method:: calculate_grasp_orientation(front_orientation: numpy.ndarray) -> typing_extensions.List[float]

      Calculates the grasp orientation based on the approach axis and the grasp description.

      :param front_orientation: The front-facing orientation of the end effector as a numpy array.

      :return: The calculated orientation as a quaternion.



   .. py:method:: calculate_grasp_descriptions(robot: semantic_digital_twin.robots.abstract_robot.AbstractRobot, pose: pycram.datastructures.pose.PoseStamped, grasp_alignment: typing_extensions.Optional[PreferredGraspAlignment] = None) -> typing_extensions.List[GraspDescription]
      :staticmethod:


      This method determines the possible grasp configurations (approach axis and vertical alignment) of the body,
      taking into account the bodies orientation, position, and whether the gripper should be rotated by 90°.

      :param robot: The robot for which the grasp configurations are being calculated.
      :param grasp_alignment: An optional PreferredGraspAlignment object that specifies preferred grasp axis,
      :param pose: The pose of the object to be grasped.

      :return: A sorted list of GraspDescription instances representing all grasp permutations.



   .. py:method:: calculate_closest_faces(pose_to_robot_vector: pycram.datastructures.pose.PyCramVector3, specified_grasp_axis: pycram.datastructures.enums.AxisIdentifier = AxisIdentifier.Undefined) -> typing_extensions.Union[Tuple[pycram.datastructures.enums.ApproachDirection, pycram.datastructures.enums.ApproachDirection], Tuple[pycram.datastructures.enums.VerticalAlignment, pycram.datastructures.enums.VerticalAlignment]]
      :staticmethod:


      Determines the faces of the object based on the input vector.

      If `specified_grasp_axis` is None, it calculates the primary and secondary faces based on the vector's magnitude
      determining which sides of the object are most aligned with the robot. This will either be the x, y plane for side faces
      or the z axis for top/bottom faces.
      If `specified_grasp_axis` is provided, it only considers the specified axis and calculates the faces aligned
      with that axis.

      :param pose_to_robot_vector: A 3D vector representing one of the robot's axes in the pose's frame, with
                            irrelevant components set to np.nan.
      :param specified_grasp_axis: Specifies a specific axis (e.g., X, Y, Z) to focus on.

      :return: A tuple of two Grasp enums representing the primary and secondary faces.



.. py:class:: PreferredGraspAlignment

   Description of the preferred grasp alignment for an object.


   .. py:attribute:: preferred_axis
      :type:  typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier]

      The preferred axis, X, Y, or Z, for grasping the object, or None if not specified.



   .. py:attribute:: with_vertical_alignment
      :type:  bool

      Indicates if the object should be grasped with a vertical alignment.



   .. py:attribute:: with_rotated_gripper
      :type:  bool

      Indicates if the gripper should be rotated by 90° around X.



