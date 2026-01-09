pycram.validation.error_checkers
================================

.. py:module:: pycram.validation.error_checkers


Classes
-------

.. autoapisummary::

   pycram.validation.error_checkers.ErrorChecker
   pycram.validation.error_checkers.PoseErrorChecker
   pycram.validation.error_checkers.PositionErrorChecker
   pycram.validation.error_checkers.OrientationErrorChecker
   pycram.validation.error_checkers.SingleValueErrorChecker
   pycram.validation.error_checkers.RevoluteJointPositionErrorChecker
   pycram.validation.error_checkers.PrismaticJointPositionErrorChecker
   pycram.validation.error_checkers.IterableErrorChecker
   pycram.validation.error_checkers.MultiJointPositionErrorChecker


Functions
---------

.. autoapisummary::

   pycram.validation.error_checkers.calculate_pose_error
   pycram.validation.error_checkers.calculate_position_error
   pycram.validation.error_checkers.calculate_orientation_error
   pycram.validation.error_checkers.calculate_joint_position_error
   pycram.validation.error_checkers.is_error_acceptable
   pycram.validation.error_checkers.calculate_angle_between_quaternions
   pycram.validation.error_checkers.calculate_quaternion_difference


Module Contents
---------------

.. py:class:: ErrorChecker(acceptable_error: typing_extensions.Union[float, typing_extensions.Iterable[float]], is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`abc.ABC`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


   .. py:attribute:: _acceptable_error
      :type:  numpy.ndarray


   .. py:attribute:: tiled_acceptable_error
      :type:  typing_extensions.Optional[numpy.ndarray]
      :value: None



   .. py:attribute:: is_iterable
      :value: False



   .. py:method:: reset() -> None

      Reset the error checker.



   .. py:property:: acceptable_error
      :type: numpy.ndarray



   .. py:method:: update_acceptable_error(new_acceptable_error: typing_extensions.Optional[typing_extensions.Iterable[float]] = None, tile_to_match: typing_extensions.Optional[typing_extensions.Sized] = None) -> None

      Update the acceptable error with a new value, and tile it to match the length of the error if needed.

      :param new_acceptable_error: The new acceptable error.
      :param tile_to_match: The iterable to match the length of the error with.



   .. py:method:: update_tiled_acceptable_error(tile_to_match: typing_extensions.Sized) -> None

      Tile the acceptable error to match the length of the error.

      :param tile_to_match: The object to match the length of the error.
      :return: The tiled acceptable error.



   .. py:method:: _calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> typing_extensions.Union[float, typing_extensions.List[float]]
      :abstractmethod:


      Calculate the error between two values.

      :param value_1: The first value.
      :param value_2: The second value.
      :return: The error between the two values.



   .. py:method:: calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> typing_extensions.Union[float, typing_extensions.List[float]]

      Calculate the error between two values.

      :param value_1: The first value.
      :param value_2: The second value.
      :return: The error between the two values.



   .. py:method:: is_error_acceptable(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> bool

      Check if the error is acceptable.

      :param value_1: The first value.
      :param value_2: The second value.
      :return: Whether the error is acceptable.



.. py:class:: PoseErrorChecker(acceptable_error: typing_extensions.Union[typing_extensions.Tuple[float], typing_extensions.Iterable[typing_extensions.Tuple[float]]] = (0.001, np.pi / 180), is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`ErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


   .. py:method:: _calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> typing_extensions.List[float]

      Calculate the error between two poses.

      :param value_1: The first pose.
      :param value_2: The second pose.



.. py:class:: PositionErrorChecker(acceptable_error: typing_extensions.Optional[float] = 0.001, is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`ErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


   .. py:method:: _calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> float

      Calculate the error between two positions.

      :param value_1: The first position.
      :param value_2: The second position.
      :return: The error between the two positions.



.. py:class:: OrientationErrorChecker(acceptable_error: typing_extensions.Optional[float] = np.pi / 180, is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`ErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


   .. py:method:: _calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> float

      Calculate the error between two quaternions.

      :param value_1: The first quaternion.
      :param value_2: The second quaternion.
      :return: The error between the two quaternions.



.. py:class:: SingleValueErrorChecker(acceptable_error: typing_extensions.Optional[float] = 0.001, is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`ErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


   .. py:method:: _calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> float

      Calculate the error between two values.

      :param value_1: The first value.
      :param value_2: The second value.
      :return: The error between the two values.



.. py:class:: RevoluteJointPositionErrorChecker(acceptable_error: typing_extensions.Optional[float] = np.pi / 180, is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`SingleValueErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


.. py:class:: PrismaticJointPositionErrorChecker(acceptable_error: typing_extensions.Optional[float] = 0.001, is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`SingleValueErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


.. py:class:: IterableErrorChecker(acceptable_error: typing_extensions.Optional[typing_extensions.Iterable[float]] = None)

   Bases: :py:obj:`ErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


   .. py:method:: _calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> float

      Calculate the error between two values.

      :param value_1: The first value.
      :param value_2: The second value.
      :return: The error between the two values.



.. py:class:: MultiJointPositionErrorChecker(joint_types: typing_extensions.List[pycram.datastructures.enums.JointType], acceptable_error: typing_extensions.Optional[typing_extensions.Iterable[float]] = None)

   Bases: :py:obj:`IterableErrorChecker`


   An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
   two values and another for checking if the error is acceptable.


   .. py:attribute:: joint_types


   .. py:method:: _calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> float

      Calculate the error between two joint positions.

      :param value_1: The first joint position.
      :param value_2: The second joint position.
      :return: The error between the two joint positions.



.. py:function:: calculate_pose_error(pose_1: pycram.datastructures.pose.PoseStamped, pose_2: pycram.datastructures.pose.PoseStamped) -> typing_extensions.List[float]

   Calculate the error between two poses.

   :param pose_1: The first pose.
   :param pose_2: The second pose.
   :return: The error between the two poses.


.. py:function:: calculate_position_error(position_1: typing_extensions.List[float], position_2: typing_extensions.List[float]) -> float

   Calculate the error between two positions.

   :param position_1: The first position.
   :param position_2: The second position.
   :return: The error between the two positions.


.. py:function:: calculate_orientation_error(quat_1: typing_extensions.List[float], quat_2: typing_extensions.List[float]) -> float

   Calculate the error between two quaternions.

   :param quat_1: The first quaternion.
   :param quat_2: The second quaternion.
   :return: The error between the two quaternions.


.. py:function:: calculate_joint_position_error(joint_position_1: float, joint_position_2: float) -> float

   Calculate the error between two joint positions.

   :param joint_position_1: The first joint position.
   :param joint_position_2: The second joint position.
   :return: The error between the two joint positions.


.. py:function:: is_error_acceptable(error: typing_extensions.Union[float, typing_extensions.Iterable[float]], acceptable_error: typing_extensions.Union[float, typing_extensions.Iterable[float]]) -> bool

   Check if the error is acceptable.

   :param error: The error.
   :param acceptable_error: The acceptable error.
   :return: Whether the error is acceptable.


.. py:function:: calculate_angle_between_quaternions(quat_1: typing_extensions.List[float], quat_2: typing_extensions.List[float]) -> float

   Calculates the angle between two quaternions.

   :param quat_1: The first quaternion.
   :param quat_2: The second quaternion.
   :return: A float value that represents the angle between the two quaternions.


.. py:function:: calculate_quaternion_difference(quat_1: typing_extensions.List[float], quat_2: typing_extensions.List[float]) -> typing_extensions.List[float]

   Calculates the quaternion difference.

   :param quat_1: The quaternion of the object at the first time step.
   :param quat_2: The quaternion of the object at the second time step.
   :return: A list of float values that represent the quaternion difference.


