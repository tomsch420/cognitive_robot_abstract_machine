pycram.datastructures.pose
==========================

.. py:module:: pycram.datastructures.pose


Attributes
----------

.. autoapisummary::

   pycram.datastructures.pose.Point


Classes
-------

.. autoapisummary::

   pycram.datastructures.pose.PyCramVector3
   pycram.datastructures.pose.PyCramQuaternion
   pycram.datastructures.pose.PyCramPose
   pycram.datastructures.pose.Header
   pycram.datastructures.pose.Vector3Stamped
   pycram.datastructures.pose.PoseStamped
   pycram.datastructures.pose.Transform
   pycram.datastructures.pose.TransformStamped
   pycram.datastructures.pose.GraspPose


Module Contents
---------------

.. py:class:: PyCramVector3

   Bases: :py:obj:`pycram.has_parameters.HasParameters`


   A 3D vector with x, y and z coordinates.


   .. py:attribute:: x
      :type:  float
      :value: 0



   .. py:attribute:: y
      :type:  float
      :value: 0



   .. py:attribute:: z
      :type:  float
      :value: 0



   .. py:method:: euclidean_distance(other: typing_extensions.Self) -> float

      The euclidian distance between this vector and another vector.

      :param other: The other vector to calculate the distance to.
      :return: The euclidian distance



   .. py:method:: ros_message()

      Convert the vector to a ROS message of type Vector3.

      :return: The ROS message.



   .. py:method:: to_list() -> typing_extensions.List[float]

      Convert the vector to a list.

      :return: A list containing the x, y and z coordinates.



   .. py:method:: to_spatial_type(reference_frame: semantic_digital_twin.world_description.world_entity.Body = None) -> semantic_digital_twin.spatial_types.spatial_types.Vector3


   .. py:method:: round(decimals: int = 4)

      Rounds the coordinates of the vector to the specified number of decimal places.

      :param decimals: Number of decimal places to round to.



   .. py:method:: almost_equal(other: typing_extensions.Self, tolerance: float = 1e-06) -> bool

      Check if two vectors are almost equal within a given tolerance.

      :param other: The other vector to compare to.
      :param tolerance: The tolerance for the comparison as number of decimal places.
      :return: True if the vectors are almost equal, False otherwise.



   .. py:method:: vector_to_position(other: typing_extensions.Self) -> PyCramVector3

      Calculates a vector from this vector to another vector.

      :param other: The vector to calculate the vector to.
      :return: A new vector between this vector and the other vector.



   .. py:method:: to_numpy() -> numpy.ndarray

      Convert the vector to a numpy array.

      :return: A numpy array containing the x, y and z coordinates.



   .. py:method:: __add__(other: typing_extensions.Self) -> PyCramVector3

      Adds two vectors together.

      :param other: The other vector to add.
      :return: A new vector that is the sum of this vector and the other vector.



   .. py:method:: __sub__(other: typing_extensions.Self) -> PyCramVector3

      Subtracts two vectors.

      :param other: The other vector to subtract.
      :return: A new vector that is the difference of this vector and the other vector.



   .. py:method:: __mul__(other: float) -> PyCramVector3

      Multiplies the vector by a scalar.

      :param other: The scalar to multiply by.
      :return: A new vector that is the product of this vector and the scalar.



   .. py:method:: __rmul__(other: float) -> PyCramVector3

      Multiplies the vector by a scalar (right multiplication).

      :param other: The scalar to multiply by.
      :return: A new vector that is the product of this vector and the scalar.



   .. py:method:: from_list(vector: typing_extensions.List[float]) -> typing_extensions.Self
      :classmethod:


      Factory to create a Vector3 from a list of coordinates.

      :param vector: The list of coordinates.
      :return: A new Vector3 object.



.. py:class:: PyCramQuaternion

   Bases: :py:obj:`pycram.has_parameters.HasParameters`


   A quaternion with x, y, z and w components.


   .. py:attribute:: x
      :type:  float
      :value: 0



   .. py:attribute:: y
      :type:  float
      :value: 0



   .. py:attribute:: z
      :type:  float
      :value: 0



   .. py:attribute:: w
      :type:  float
      :value: 1



   .. py:method:: __post_init__()


   .. py:method:: normalize()

      Normalize the quaternion in-place.



   .. py:method:: ros_message()


   .. py:method:: to_list() -> typing_extensions.List[float]

      Convert the quaternion to a list.

      :return: A list containing the x, y, z and w components.



   .. py:method:: to_numpy() -> numpy.ndarray

      Convert the quaternion to a numpy array.

      :return: A numpy array containing the x, y, z and w components.



   .. py:method:: to_spatial_type() -> semantic_digital_twin.spatial_types.spatial_types.Quaternion

      Convert the quaternion to a SpatialQuaternion.

      :return: A SpatialQuaternion object containing the x, y, z and w components.



   .. py:method:: round(decimals: int = 4)

      Rounds the components of the quaternion to the specified number of decimal places.

      :param decimals: The number of decimal places to round to.



   .. py:method:: almost_equal(other: typing_extensions.Self, tolerance: float = 1e-06) -> bool

      Check if two quaternions are almost equal within a given tolerance.

      :param other: The other quaternion to compare to.
      :param tolerance: The tolerance for the comparison as number of decimal places.
      :return: True if the quaternions are almost equal, False otherwise.



   .. py:method:: __mul__(other: typing_extensions.Self) -> PyCramQuaternion

      Multiplies two quaternions together.

      :param other: The other quaternion to multiply with.
      :return: A new quaternion that is the product of this quaternion and the other quaternion.



   .. py:method:: from_list(quaternion: typing_extensions.List[float]) -> typing_extensions.Self
      :classmethod:


      Factory to create a Quaternion from a list of components.

      :param quaternion: A list of components [x, y, z, w].
      :return: A new Quaternion object.



   .. py:method:: from_matrix(matrix: numpy.ndarray) -> typing_extensions.Self
      :classmethod:


      Create a Quaternion from a 3x3 rotation matrix.

      :param matrix: A 3x3 rotation matrix as numpy array.
      :return: A Quaternion object created from the matrix.



   .. py:method:: __eq__(other)


.. py:class:: PyCramPose

   Bases: :py:obj:`pycram.has_parameters.HasParameters`


   A pose in 3D space.


   .. py:attribute:: position
      :type:  PyCramVector3


   .. py:attribute:: orientation
      :type:  PyCramQuaternion


   .. py:method:: __repr__()


   .. py:method:: ros_message()

      Convert the pose to a ROS message of type Pose.

      :return: The ROS message.



   .. py:method:: to_list()

      Convert the pose to a list of [position, orientation].

      :return: A list containing the position and orientation of this pose.



   .. py:method:: to_spatial_type() -> semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix


   .. py:method:: round(decimals: int = 4)

      Rounds the components of the pose (position and orientation) to the specified number of decimal places.

      :param decimals: The number of decimal places to round to.



   .. py:method:: almost_equal(other: PyCramPose, position_tolerance: float = 1e-06, orientation_tolerance: float = 1e-05) -> bool

      Check if two poses are almost equal within given tolerances for position and orientation.

      :param other: The other pose to compare to.
      :param position_tolerance: Tolerance for position comparison as number of decimal places.
      :param orientation_tolerance: Tolerance for orientation comparison as number of decimal places.
      :return:  True if the poses are almost equal, False otherwise.



   .. py:method:: __eq__(other: typing_extensions.Self) -> bool

      Check if two poses are equal. Uses almost_equal with a tolerance of 1e-4 for both position and orientation.

      :param other: The other pose to compare to.
      :return: True if the poses are equal, False otherwise.



   .. py:method:: from_matrix(matrix: numpy.ndarray) -> typing_extensions.Self
      :classmethod:


      Create a Pose from a 4x4 transformation matrix.

      :param matrix: A 4x4 transformation matrix as numpy array.
      :return: A pose object created from the matrix.



   .. py:method:: from_list(position: typing_extensions.List[float], orientation: typing_extensions.List[float]) -> typing_extensions.Self
      :classmethod:


      Factory to create a Pose from a list of position and orientation.

      :param position: List of position [x, y, z].
      :param orientation: List of orientation [x, y, z, w].
      :return: A new Pose object.



.. py:class:: Header

   A header with a timestamp.


   .. py:attribute:: frame_id
      :type:  semantic_digital_twin.world_description.world_entity.Body
      :value: None



   .. py:attribute:: stamp
      :type:  datetime.datetime


   .. py:attribute:: sequence
      :type:  int
      :value: 0



   .. py:method:: ros_message()

      Convert the header to a ROS message of type Header.

      :return: The ROS message.



   .. py:method:: __deepcopy__(memo)


.. py:class:: Vector3Stamped

   Bases: :py:obj:`PyCramVector3`


   A Vector3 with an attached ROS Header (timestamp and frame).
   Inherits all vector operations and adds frame/time metadata.


   .. py:attribute:: header
      :type:  Header


   .. py:property:: frame_id


   .. py:method:: __repr__()


   .. py:method:: ros_message()

      Convert to a ROS Vector3Stamped message.

      :return: The ROS message.



   .. py:method:: from_ros_message(message)
      :classmethod:


      Create a Vector3Stamped from a ROS message.

      :param message: The Vector3Stamped ROS message.
      :return: A new Vector3Stamped object.



   .. py:method:: to_spatial_type() -> semantic_digital_twin.spatial_types.spatial_types.Vector3


.. py:class:: PoseStamped

   Bases: :py:obj:`pycram.has_parameters.HasParameters`


   A pose in 3D space with a timestamp.


   .. py:attribute:: pose
      :type:  PyCramPose


   .. py:attribute:: header
      :type:  Header


   .. py:property:: position


   .. py:property:: orientation


   .. py:property:: frame_id


   .. py:method:: __repr__()


   .. py:method:: ros_message()

      Convert the pose to a ROS message of type PoseStamped.

      :return: The ROS message.



   .. py:method:: from_ros_message(message: ROSPoseStamped) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a ROS message.

      :param message: The PoseStamped ROS message.
      :return: A new PoseStamped object created from the ROS message.



   .. py:method:: from_list(position: typing_extensions.Optional[typing_extensions.List[float]] = None, orientation: typing_extensions.Optional[typing_extensions.List[float]] = None, frame: typing_extensions.Optional[semantic_digital_twin.world_description.world_entity.Body] = None) -> typing_extensions.Self
      :classmethod:


      Factory to create a PoseStamped from a list of position and orientation.

      :param position: Position as a list of [x, y, z].
      :param orientation: Orientation as a list of [x, y, z, w].
      :param frame: Frame in which the pose is defined.
      :return: A new PoseStamped object.



   .. py:method:: from_matrix(matrix: numpy.ndarray, frame: semantic_digital_twin.world_description.world_entity.Body) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a 4x4 transformation matrix and a frame.

      :param matrix: A 4x4 transformation matrix as numpy array.
      :param frame: The frame in which the pose is defined.
      :return: A PoseStamped object created from the matrix and frame.



   .. py:method:: from_spatial_type(spatial_type: semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix) -> typing_extensions.Self
      :classmethod:


      Create a PoseStamped from a SpatialTransformationMatrix and a frame.

      :param spatial_type: A SpatialTransformationMatrix object.
      :return: A PoseStamped object created from the spatial type and frame.



   .. py:method:: to_transform_stamped(child_link_id: semantic_digital_twin.world_description.world_entity.Body) -> TransformStamped

      Converts the PoseStamped to a TransformStamped given a frame to which the transform is pointing.

      :param child_link_id: Frame to which the transform is pointing.
      :return: A TransformStamped object.



   .. py:method:: to_spatial_type() -> semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix

      Converts the PoseStamped to a SpatialTransformationMatrix.

      :return: A SpatialTransformationMatrix object representing the pose in 3D space.



   .. py:method:: round(decimals: int = 4)

      Rounds the components of the pose (position and orientation) to the specified number of decimal places.

      :param decimals: Number of decimal places to round to.



   .. py:method:: to_list()

      Convert the pose to a list of [position, orientation, frame_id].

      :return: A list of [pose, frame_id].



   .. py:method:: almost_equal(other: PoseStamped, position_tolerance: float = 1e-06, orientation_tolerance: float = 1e-05) -> bool

      Check if two PoseStamped objects are almost equal within given tolerances for position and orientation and if the
      frame_id is the same.

      :param other: The other PoseStamped object to compare to.
      :param position_tolerance: Tolerance for position comparison as number of decimal places.
      :param orientation_tolerance: Tolerance for orientation comparison as number of decimal places.
      :return: True if the PoseStamped objects are almost equal, False otherwise.



   .. py:method:: rotate_by_quaternion(quaternion: typing_extensions.List[float])

      Rotates the orientation of the pose by a given quaternion.

      :param quaternion: A list representing the quaternion [x, y, z, w].



   .. py:method:: is_facing_2d_axis(pose_b: PoseStamped, axis: typing_extensions.Optional[pycram.datastructures.enums.AxisIdentifier] = AxisIdentifier.X, threshold_deg=82) -> typing_extensions.Tuple[bool, float]

      Check if this pose is facing another pose along a specific axis (X or Y) within a given angular threshold.

      :param pose_b: The target pose to compare against.
      :param axis: The axis to check alignment with ('x' or 'y'). Defaults to 'x'.
      :param threshold_deg: The maximum angular difference in degrees to consider as 'facing'. Defaults to 82 degrees.
      :return: Tuple of (True/False if facing, signed angular difference in radians).



   .. py:method:: is_facing_x_or_y(pose_b: PoseStamped) -> bool

      Check if this pose is facing another pose along either the X or Y axis within a default angular threshold.

      :param pose_b: The target pose to compare against.
      :return: True if this pose is facing the target along either X or Y axis, False otherwise.



.. py:class:: Transform

   Bases: :py:obj:`PyCramPose`


   A pose in 3D space.


   .. py:property:: translation


   .. py:property:: rotation


   .. py:method:: to_matrix() -> numpy.ndarray

      Converts the transform to a 4x4 transformation matrix.

      :return: A numpy array representing the transformation matrix.



   .. py:method:: __invert__() -> Transform

      Inverts the transform, returning a new Transform object.

      :return: The inverted Transform object.



   .. py:method:: __mul__(other: Transform) -> Transform

      Multiplies two transforms together, returning a new Transform object.

      :param other: The other Transform object to multiply with.
      :return: A new Transform object that is the product of this transform and the other transform.



   .. py:method:: from_pose(pose: PyCramPose) -> typing_extensions.Self
      :classmethod:


      Create a Transform from a Pose object.

      :param pose: The pose to convert to a Transform.
      :return: A new Transform object created from the Pose.



   .. py:method:: ros_message()

      Convert the transform to a ROS message of type Transform.

      :return: The ROS message.



.. py:class:: TransformStamped

   Bases: :py:obj:`PoseStamped`


   A pose in 3D space with a timestamp.


   .. py:attribute:: child_frame_id
      :type:  semantic_digital_twin.world_description.world_entity.Body
      :value: ''


      Target frame id of the transform.



   .. py:attribute:: pose
      :type:  Transform

      The transform of the transform.



   .. py:property:: transform
      :type: Transform



   .. py:property:: translation


   .. py:property:: rotation


   .. py:method:: __invert__() -> typing_extensions.Self

      Inverts the transform, returning a new TransformStamped object which points from child_frame_id to frame_id.

      :return: A new TransformStamped object that is the inverse of this transform.



   .. py:method:: __mul__(other) -> typing_extensions.Self

      Multiplies two TransformStamped objects together, returning a new TransformStamped object.

      :param other: The other TransformStamped object to multiply with.
      :return: A new TransformStamped object that is the product of this transform and the other transform.



   .. py:method:: __deepcopy__(memo)


   .. py:method:: from_list(translation: typing_extensions.List[float] = None, rotation: typing_extensions.List[float] = None, frame: semantic_digital_twin.world_description.world_entity.Body = None, child_frame_id: semantic_digital_twin.world_description.world_entity.Body = None) -> typing_extensions.Self
      :classmethod:


      Factory to create a TransformStamped from a list of position and orientation.

      :param translation: Translation as a list of [x, y, z].
      :param rotation: Rotation as a list of [x, y, z, w].
      :param frame: Original frame in which the transform is defined.
      :param child_frame_id: Target frame id of the transform.
      :return: A new TransformStamped object.



   .. py:method:: ros_message()

      Convert the TransformStamped to a ROS message of type TransformStamped.

      :return: The ROS message.



   .. py:method:: to_pose_stamped() -> PoseStamped

      Converts the TransformStamped to a PoseStamped object.

      :return: A PoseStamped object created from the TransformStamped.



   .. py:method:: to_spatial_type() -> semantic_digital_twin.spatial_types.spatial_types.HomogeneousTransformationMatrix

      Converts the TransformStamped to a SpatialTransformationMatrix.

      :return: A SpatialTransformationMatrix object representing the transform in 3D space.



   .. py:method:: inverse_times(other: TransformStamped) -> typing_extensions.Self

      Multiply this TransformStamped by the inverse of another TransformStamped. Essentially, this is the same as
      "subtracting" another TransformStamped from this one.

      :param other: The other TransformStamped to subtract.
      :return: A new TransformStamped object that is the result of this transform minus the other transform.



.. py:class:: GraspPose

   Bases: :py:obj:`PoseStamped`


   A pose from which a grasp can be performed along with the respective arm and grasp description.


   .. py:attribute:: arm
      :type:  pycram.datastructures.enums.Arms
      :value: None


      Arm corresponding to the grasp pose.



   .. py:attribute:: grasp_description
      :type:  pycram.datastructures.grasp.GraspDescription
      :value: None


      Grasp description corresponding to the grasp pose.



.. py:data:: Point

