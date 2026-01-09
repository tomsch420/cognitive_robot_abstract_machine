pycram.validation.goal_validator
================================

.. py:module:: pycram.validation.goal_validator


Attributes
----------

.. autoapisummary::

   pycram.validation.goal_validator.logger
   pycram.validation.goal_validator.OptionalArgCallable


Classes
-------

.. autoapisummary::

   pycram.validation.goal_validator.GoalValidator
   pycram.validation.goal_validator.PoseGoalValidator
   pycram.validation.goal_validator.MultiPoseGoalValidator
   pycram.validation.goal_validator.PositionGoalValidator
   pycram.validation.goal_validator.MultiPositionGoalValidator
   pycram.validation.goal_validator.OrientationGoalValidator
   pycram.validation.goal_validator.MultiOrientationGoalValidator
   pycram.validation.goal_validator.JointPositionGoalValidator
   pycram.validation.goal_validator.MultiJointPositionGoalValidator


Functions
---------

.. autoapisummary::

   pycram.validation.goal_validator.validate_object_pose
   pycram.validation.goal_validator.validate_multiple_object_poses
   pycram.validation.goal_validator.validate_joint_position
   pycram.validation.goal_validator.validate_multiple_joint_positions
   pycram.validation.goal_validator.create_multiple_joint_goal_validator


Module Contents
---------------

.. py:data:: logger

.. py:data:: OptionalArgCallable

.. py:class:: GoalValidator(error_checker: pycram.validation.error_checkers.ErrorChecker, current_value_getter: OptionalArgCallable, acceptable_percentage_of_goal_achieved: typing_extensions.Optional[float] = 0.8)

   A class to validate the goal by tracking the goal achievement progress.


   .. py:attribute:: raise_error
      :type:  typing_extensions.Optional[bool]
      :value: False


      Whether to raise an error if the goal is not achieved.



   .. py:attribute:: total_wait_time
      :type:  typing_extensions.Optional[datetime.timedelta]
      :value: None


      The total wait time that was spent waiting for the goal to be achieved.



   .. py:attribute:: error_checker
      :type:  pycram.validation.error_checkers.ErrorChecker


   .. py:attribute:: current_value_getter
      :type:  typing_extensions.Callable[[typing_extensions.Optional[typing_extensions.Any]], typing_extensions.Any]


   .. py:attribute:: acceptable_percentage_of_goal_achieved
      :type:  typing_extensions.Optional[float]
      :value: 0.8



   .. py:attribute:: goal_value
      :type:  typing_extensions.Optional[typing_extensions.Any]
      :value: None



   .. py:attribute:: initial_error
      :type:  typing_extensions.Optional[numpy.ndarray]
      :value: None



   .. py:attribute:: current_value_getter_input
      :type:  typing_extensions.Optional[typing_extensions.Any]
      :value: None



   .. py:method:: register_goal_and_wait_until_achieved(goal_value: typing_extensions.Any, current_value_getter_input: typing_extensions.Optional[typing_extensions.Any] = None, initial_value: typing_extensions.Optional[typing_extensions.Any] = None, acceptable_error: typing_extensions.Optional[typing_extensions.Union[float, typing_extensions.Iterable[float]]] = None, max_wait_time: typing_extensions.Optional[float] = 1, time_per_read: typing_extensions.Optional[float] = 0.01) -> None

      Register the goal value and wait until the target is reached.

      :param goal_value: The goal value.
      :param current_value_getter_input: The values that are used as input to the current value getter.
      :param initial_value: The initial value.
      :param acceptable_error: The acceptable error.
      :param max_wait_time: The maximum time to wait.
      :param time_per_read: The time to wait between each read.



   .. py:method:: wait_until_goal_is_achieved(max_wait_time: typing_extensions.Optional[datetime.timedelta] = timedelta(seconds=2), time_per_read: typing_extensions.Optional[datetime.timedelta] = timedelta(milliseconds=10)) -> None

      Wait until the target is reached.

      :param max_wait_time: The maximum time to wait.
      :param time_per_read: The time to wait between each read.



   .. py:method:: reset() -> None

      Reset the goal validator.



   .. py:property:: _acceptable_error
      :type: numpy.ndarray


      The acceptable error.



   .. py:property:: acceptable_error
      :type: numpy.ndarray


      The acceptable error.



   .. py:property:: tiled_acceptable_error
      :type: typing_extensions.Optional[numpy.ndarray]


      The tiled acceptable error.



   .. py:method:: register_goal(goal_value: typing_extensions.Any, current_value_getter_input: typing_extensions.Optional[typing_extensions.Any] = None, initial_value: typing_extensions.Optional[typing_extensions.Any] = None, acceptable_error: typing_extensions.Optional[typing_extensions.Union[float, typing_extensions.Iterable[float]]] = None)

      Register the goal value.

      :param goal_value: The goal value.
      :param current_value_getter_input: The values that are used as input to the current value getter.
      :param initial_value: The initial value.
      :param acceptable_error: The acceptable error.



   .. py:method:: update_initial_error(goal_value: typing_extensions.Any, initial_value: typing_extensions.Optional[typing_extensions.Any] = None) -> None

      Calculate the initial error.

      :param goal_value: The goal value.
      :param initial_value: The initial value.



   .. py:property:: current_value
      :type: typing_extensions.Any


      The current value of the monitored variable.



   .. py:property:: current_error
      :type: numpy.ndarray


      The current error.



   .. py:method:: calculate_error(value_1: typing_extensions.Any, value_2: typing_extensions.Any) -> numpy.ndarray

      Calculate the error between two values.

      :param value_1: The first value.
      :param value_2: The second value.
      :return: The error.



   .. py:property:: percentage_of_goal_achieved
      :type: float


      The relative (relative to the acceptable error) achieved percentage of goal.



   .. py:property:: actual_percentage_of_goal_achieved
      :type: float


      The percentage of goal achieved.



   .. py:property:: relative_current_error
      :type: numpy.ndarray


      The relative current error (relative to the acceptable error).



   .. py:property:: relative_initial_error
      :type: numpy.ndarray


      The relative initial error (relative to the acceptable error).



   .. py:method:: get_relative_error(error: typing_extensions.Any, threshold: typing_extensions.Optional[float] = 0.001) -> numpy.ndarray

      Get the relative error by comparing the error with the acceptable error and filtering out the errors that are
      less than the threshold.

      :param error: The error.
      :param threshold: The threshold.
      :return: The relative error.



   .. py:property:: goal_achieved
      :type: bool


      Check if the goal is achieved.



   .. py:property:: is_current_error_acceptable
      :type: bool


      Check if the error is acceptable.



   .. py:property:: goal_not_achieved_message

      Message to be displayed when the goal is not achieved.



.. py:class:: PoseGoalValidator(current_pose_getter: OptionalArgCallable = None, acceptable_error: typing_extensions.Union[typing_extensions.Tuple[float], typing_extensions.Iterable[typing_extensions.Tuple[float]]] = (0.001, np.pi / 180), acceptable_percentage_of_goal_achieved: typing_extensions.Optional[float] = 0.8, is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`GoalValidator`


   A class to validate the pose goal by tracking the goal achievement progress.


.. py:class:: MultiPoseGoalValidator(current_poses_getter: OptionalArgCallable = None, acceptable_error: typing_extensions.Union[typing_extensions.Tuple[float], typing_extensions.Iterable[typing_extensions.Tuple[float]]] = (0.01, 5 * np.pi / 180), acceptable_percentage_of_goal_achieved: typing_extensions.Optional[float] = 0.8)

   Bases: :py:obj:`PoseGoalValidator`


   A class to validate the multi-pose goal by tracking the goal achievement progress.


.. py:class:: PositionGoalValidator(current_position_getter: OptionalArgCallable = None, acceptable_error: typing_extensions.Optional[float] = 0.001, acceptable_percentage_of_goal_achieved: typing_extensions.Optional[float] = 0.8, is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`GoalValidator`


   A class to validate the position goal by tracking the goal achievement progress.


.. py:class:: MultiPositionGoalValidator(current_positions_getter: OptionalArgCallable = None, acceptable_error: typing_extensions.Optional[float] = 0.001, acceptable_percentage_of_goal_achieved: typing_extensions.Optional[float] = 0.8)

   Bases: :py:obj:`PositionGoalValidator`


   A class to validate the multi-position goal by tracking the goal achievement progress.


.. py:class:: OrientationGoalValidator(current_orientation_getter: OptionalArgCallable = None, acceptable_error: typing_extensions.Optional[float] = np.pi / 180, acceptable_percentage_of_goal_achieved: typing_extensions.Optional[float] = 0.8, is_iterable: typing_extensions.Optional[bool] = False)

   Bases: :py:obj:`GoalValidator`


   A class to validate the orientation goal by tracking the goal achievement progress.


.. py:class:: MultiOrientationGoalValidator(current_orientations_getter: OptionalArgCallable = None, acceptable_error: typing_extensions.Optional[float] = np.pi / 180, acceptable_percentage_of_goal_achieved: typing_extensions.Optional[float] = 0.8)

   Bases: :py:obj:`OrientationGoalValidator`


   A class to validate the multi-orientation goal by tracking the goal achievement progress.


.. py:class:: JointPositionGoalValidator(current_position_getter: OptionalArgCallable = None, acceptable_error: typing_extensions.Optional[float] = None, acceptable_revolute_joint_position_error: float = np.pi / 180, acceptable_prismatic_joint_position_error: float = 0.001, acceptable_percentage_of_goal_achieved: float = 0.8, is_iterable: bool = False)

   Bases: :py:obj:`GoalValidator`


   A class to validate the joint position goal by tracking the goal achievement progress.


   .. py:attribute:: acceptable_orientation_error


   .. py:attribute:: acceptable_position_error
      :value: 0.001



   .. py:method:: register_goal(goal_value: typing_extensions.Any, joint_type: pycram.datastructures.enums.JointType, current_value_getter_input: typing_extensions.Optional[typing_extensions.Any] = None, initial_value: typing_extensions.Optional[typing_extensions.Any] = None, acceptable_error: typing_extensions.Optional[float] = None)

      Register the goal value.

      :param goal_value: The goal value.
      :param joint_type: The joint type (e.g. REVOLUTE, PRISMATIC).
      :param current_value_getter_input: The values that are used as input to the current value getter.
      :param initial_value: The initial value.
      :param acceptable_error: The acceptable error.



.. py:class:: MultiJointPositionGoalValidator(current_positions_getter: OptionalArgCallable = None, acceptable_error: typing_extensions.Optional[typing_extensions.Iterable[float]] = None, acceptable_revolute_joint_position_error: float = np.pi / 180, acceptable_prismatic_joint_position_error: float = 0.001, acceptable_percentage_of_goal_achieved: float = 0.8)

   Bases: :py:obj:`GoalValidator`


   A class to validate the multi-joint position goal by tracking the goal achievement progress.


   .. py:attribute:: acceptable_orientation_error


   .. py:attribute:: acceptable_position_error
      :value: 0.001



   .. py:method:: register_goal(goal_value: typing_extensions.Any, joint_type: typing_extensions.Iterable[pycram.datastructures.enums.JointType], current_value_getter_input: typing_extensions.Optional[typing_extensions.Any] = None, initial_value: typing_extensions.Optional[typing_extensions.Any] = None, acceptable_error: typing_extensions.Optional[typing_extensions.Iterable[float]] = None)

      Register the goal value.

      :param goal_value: The goal value.
      :param current_value_getter_input: The values that are used as input to the current value getter.
      :param initial_value: The initial value.
      :param acceptable_error: The acceptable error.



.. py:function:: validate_object_pose(pose_setter_func)

   A decorator to validate the object pose.

   :param pose_setter_func: The function to set the pose of the object.


.. py:function:: validate_multiple_object_poses(pose_setter_func)

   A decorator to validate multiple object poses.

   :param pose_setter_func: The function to set multiple poses of the objects.


.. py:function:: validate_joint_position(position_setter_func)

   A decorator to validate the joint position.

   :param position_setter_func: The function to set the joint position.


.. py:function:: validate_multiple_joint_positions(position_setter_func)

   A decorator to validate the joint positions, this function does not validate the virtual joints,
   as in multiverse the virtual joints take command velocities and not positions, so after their goals
   are set, they are zeroed thus can't be validated. (They are actually validated by the robot pose in case
   of virtual mobile base joints)

   :param position_setter_func: The function to set the joint positions.


.. py:function:: create_multiple_joint_goal_validator(robot: Object, joint_positions: typing_extensions.Union[typing_extensions.Dict[Joint, float], typing_extensions.Dict[str, float]]) -> MultiJointPositionGoalValidator

   Validate the multiple joint goals, and wait until the goal is achieved.

   :param robot: The robot object.
   :param joint_positions: The joint positions to validate.


