pycram.datastructures.dataclasses
=================================

.. py:module:: pycram.datastructures.dataclasses


Classes
-------

.. autoapisummary::

   pycram.datastructures.dataclasses.Context
   pycram.datastructures.dataclasses.ExecutionData
   pycram.datastructures.dataclasses.ManipulatorData
   pycram.datastructures.dataclasses.Color
   pycram.datastructures.dataclasses.Colors
   pycram.datastructures.dataclasses.CollisionCallbacks
   pycram.datastructures.dataclasses.LateralFriction
   pycram.datastructures.dataclasses.TextAnnotation
   pycram.datastructures.dataclasses.VirtualJoint
   pycram.datastructures.dataclasses.Rotations
   pycram.datastructures.dataclasses.VirtualMobileBaseJoints
   pycram.datastructures.dataclasses.MultiverseMetaData


Functions
---------

.. autoapisummary::

   pycram.datastructures.dataclasses.get_point_as_list


Module Contents
---------------

.. py:class:: Context

   A dataclass for storing the context of a plan


   .. py:attribute:: world
      :type:  semantic_digital_twin.world.World

      The world in which the plan is executed



   .. py:attribute:: robot
      :type:  semantic_digital_twin.robots.abstract_robot.AbstractRobot

      The semantic robot annotation which should execute the plan



   .. py:attribute:: super_plan
      :type:  typing_extensions.Optional[pycram.plan.Plan]
      :value: None


      The plan of which this plan/designator is a part of



   .. py:attribute:: ros_node
      :type:  typing_extensions.Optional[typing_extensions.Any]
      :value: None


      A ROS node that should be used for communication in this plan



   .. py:method:: from_world(world: semantic_digital_twin.world.World, super_plan: typing_extensions.Optional[pycram.plan.Plan] = None)
      :classmethod:


      Create a context from a world by getting the first robot in the world. There is no super plan in this case.

      :param world: The world for which to create the context
      :param super_plan: An optional super plan
      :return: A context with the first robot in the world and no super plan



   .. py:method:: from_plan(plan: pycram.plan.Plan)
      :classmethod:


      Create a context from a plan by getting the context information from the plan and setting the super plan to
      the given plan.

      :param plan: Plan from which to create the context
      :return: A new context with the world and robot from the plan and the super plan set to the given plan



.. py:class:: ExecutionData

   A dataclass for storing the information of an execution that is used for creating a robot description for that
   execution. An execution is a Robot with a virtual mobile base that can be used to move the robot in the environment.


   .. py:attribute:: execution_start_pose
      :type:  pycram.datastructures.pose.PoseStamped

      Start of the robot at the start of execution of an action designator



   .. py:attribute:: execution_start_world_state
      :type:  numpy.ndarray

      The world state at the start of execution of an action designator



   .. py:attribute:: execution_end_pose
      :type:  typing_extensions.Optional[pycram.datastructures.pose.PoseStamped]
      :value: None


      The pose of the robot at the end of executing an action designator



   .. py:attribute:: execution_end_world_state
      :type:  typing_extensions.Optional[numpy.ndarray]
      :value: None


      The world state at the end of executing an action designator



   .. py:attribute:: added_world_modifications
      :type:  typing_extensions.List[semantic_digital_twin.world_description.world_modification.WorldModelModificationBlock]
      :value: []


      A list of World modification blocks that were added during the execution of the action designator



   .. py:attribute:: manipulated_body_pose_start
      :type:  typing_extensions.Optional[pycram.datastructures.pose.PoseStamped]
      :value: None


      Start pose of the manipulated Body if there was one



   .. py:attribute:: manipulated_body_pose_end
      :type:  typing_extensions.Optional[pycram.datastructures.pose.PoseStamped]
      :value: None


      End pose of the manipulated Body if there was one



   .. py:attribute:: manipulated_body
      :type:  typing_extensions.Optional[semantic_digital_twin.world_description.world_entity.Body]
      :value: None


      Reference to the manipulated body 



   .. py:attribute:: manipulated_body_name
      :type:  typing_extensions.Optional[str]
      :value: None


      Name of the manipulated body



.. py:class:: ManipulatorData

   A dataclass for storing the information of a manipulator that is used for creating a robot description for that
   manipulator. A manipulator is an Arm with an end-effector that can be used to interact with the environment.


   .. py:attribute:: name
      :type:  str

      Name of the Manipulator.



   .. py:attribute:: base_link
      :type:  str

      Manipulator's base link.



   .. py:attribute:: arm_end_link
      :type:  str

      Manipulator's arm end link.



   .. py:attribute:: joint_names
      :type:  typing_extensions.List[str]

      List of joint names.



   .. py:attribute:: home_joint_values
      :type:  typing_extensions.List[float]

      List of joint values for the home position. (default position)



   .. py:attribute:: gripper_name
      :type:  str

      Name of the gripper at the end of the arm.



   .. py:attribute:: gripper_tool_frame
      :type:  str

      Name of the frame of the gripper tool.



   .. py:attribute:: gripper_joint_names
      :type:  typing_extensions.List[str]

      List of gripper joint names.



   .. py:attribute:: closed_joint_values
      :type:  typing_extensions.List[float]

      List of joint values for the gripper in the closed position.



   .. py:attribute:: open_joint_values
      :type:  typing_extensions.List[float]

      List of joint values for the gripper in the open position.



   .. py:attribute:: opening_distance
      :type:  float

      The opening distance of the gripper.



   .. py:attribute:: fingers_link_names
      :type:  typing_extensions.Optional[typing_extensions.List[str]]
      :value: None


      List of link names for the fingers of the gripper.



   .. py:attribute:: relative_dir
      :type:  str
      :value: ''


      Relative directory of the manipulator description file in the resources directory.



   .. py:attribute:: gripper_cmd_topic
      :type:  typing_extensions.Optional[str]
      :value: None


      Gripper command topic in ROS if it has one.



   .. py:attribute:: gripper_open_cmd_value
      :type:  typing_extensions.Optional[float]
      :value: None


      Grip open command value.



   .. py:attribute:: gripper_close_cmd_value
      :type:  typing_extensions.Optional[float]
      :value: None


      Grip close command value.



   .. py:attribute:: gripper_relative_dir
      :type:  typing_extensions.Optional[str]
      :value: None


      Relative directory of the gripper description file in the resources directory if it has one and is not part of the
       manipulator description file.



.. py:function:: get_point_as_list(point: pycram.datastructures.pose.Point) -> typing_extensions.List[float]

   Return the point as a list.

   :param point: The point.
   :return: The point as a list


.. py:class:: Color

   Dataclass for storing rgba_color as an RGBA value.
   The values are stored as floats between 0 and 1.
   The default rgba_color is white. 'A' stands for the opacity.


   .. py:attribute:: R
      :type:  float
      :value: 1



   .. py:attribute:: G
      :type:  float
      :value: 1



   .. py:attribute:: B
      :type:  float
      :value: 1



   .. py:attribute:: A
      :type:  float
      :value: 1



   .. py:method:: from_list(color: typing_extensions.List[float])
      :classmethod:


      Set the rgba_color from a list of RGBA values.

      :param color: The list of RGBA values



   .. py:method:: from_rgb(rgb: typing_extensions.List[float])
      :classmethod:


      Set the rgba_color from a list of RGB values.

      :param rgb: The list of RGB values



   .. py:method:: from_rgba(rgba: typing_extensions.List[float])
      :classmethod:


      Set the rgba_color from a list of RGBA values.

      :param rgba: The list of RGBA values



   .. py:method:: get_rgba() -> typing_extensions.List[float]

      Return the rgba_color as a list of RGBA values.

      :return: The rgba_color as a list of RGBA values



   .. py:method:: get_rgb() -> typing_extensions.List[float]

      Return the rgba_color as a list of RGB values.

      :return: The rgba_color as a list of RGB values



.. py:class:: Colors(*args, **kwds)

   Bases: :py:obj:`Color`, :py:obj:`enum.Enum`


   Enum for easy access to some common colors.


   .. py:attribute:: PINK
      :value: (1, 0, 1, 1)



   .. py:attribute:: BLACK
      :value: (0, 0, 0, 1)



   .. py:attribute:: WHITE
      :value: (1, 1, 1, 1)



   .. py:attribute:: RED
      :value: (1, 0, 0, 1)



   .. py:attribute:: GREEN
      :value: (0, 1, 0, 1)



   .. py:attribute:: BLUE
      :value: (0, 0, 1, 1)



   .. py:attribute:: YELLOW
      :value: (1, 1, 0, 1)



   .. py:attribute:: CYAN
      :value: (0, 1, 1, 1)



   .. py:attribute:: MAGENTA
      :value: (1, 0, 1, 1)



   .. py:attribute:: GREY
      :value: (0.5, 0.5, 0.5, 1)



   .. py:method:: from_string(color: str) -> Color
      :classmethod:


      Set the rgba_color from a string. If the string is not a valid color, it will return the color WHITE.

      :param color: The string of the color



.. py:class:: CollisionCallbacks

   Dataclass for storing the collision callbacks which are callables that get called when there is a collision
   or when a collision is no longer there.


   .. py:attribute:: on_collision_cb
      :type:  typing_extensions.Callable


   .. py:attribute:: no_collision_cb
      :type:  typing_extensions.Optional[typing_extensions.Callable]
      :value: None



.. py:class:: LateralFriction

   Dataclass for storing the information of the lateral friction.


   .. py:attribute:: lateral_friction
      :type:  float


   .. py:attribute:: lateral_friction_direction
      :type:  typing_extensions.List[float]


.. py:class:: TextAnnotation

   Dataclass for storing text annotations that can be displayed in the simulation.


   .. py:attribute:: text
      :type:  str


   .. py:attribute:: position
      :type:  typing_extensions.List[float]


   .. py:attribute:: id
      :type:  int


   .. py:attribute:: color
      :type:  Color


   .. py:attribute:: size
      :type:  float
      :value: 0.1



.. py:class:: VirtualJoint

   A virtual (not real) joint that is most likely used for simulation purposes.


   .. py:attribute:: name
      :type:  str


   .. py:attribute:: type_
      :type:  pycram.datastructures.enums.JointType


   .. py:attribute:: axes
      :type:  typing_extensions.Optional[pycram.datastructures.pose.Point]
      :value: None



   .. py:property:: type


   .. py:property:: is_virtual


   .. py:method:: __hash__()


.. py:class:: Rotations

   Bases: :py:obj:`typing_extensions.Dict`\ [\ :py:obj:`typing_extensions.Optional`\ [\ :py:obj:`typing_extensions.Union`\ [\ :py:obj:`pycram.datastructures.enums.Grasp`\ , :py:obj:`bool`\ ]\ ]\ , :py:obj:`typing_extensions.List`\ [\ :py:obj:`float`\ ]\ ]


   A dictionary that defines standard quaternions for different grasps and orientations. This is mainly used
   to automatically calculate all grasp descriptions of a robot gripper for the robot description.

   SIDE_ROTATIONS: The quaternions for the different approach directions (front, back, left, right)
   VERTICAL_ROTATIONS: The quaternions for the different vertical alignments, in case the object requires for
   example a top grasp
   HORIZONTAL_ROTATIONS: The quaternions for the different horizontal alignments, in case the gripper needs to roll
   90°


   .. py:attribute:: SIDE_ROTATIONS


   .. py:attribute:: VERTICAL_ROTATIONS


   .. py:attribute:: HORIZONTAL_ROTATIONS


.. py:class:: VirtualMobileBaseJoints

   Dataclass for storing the names, types and axes of the virtual mobile base joints of a mobile robot.


   .. py:attribute:: translation_x
      :type:  typing_extensions.Optional[VirtualJoint]


   .. py:attribute:: translation_y
      :type:  typing_extensions.Optional[VirtualJoint]


   .. py:attribute:: angular_z
      :type:  typing_extensions.Optional[VirtualJoint]


   .. py:property:: names
      :type: typing_extensions.List[str]


      Return the names of the virtual mobile base joints.



   .. py:method:: get_types() -> typing_extensions.Dict[str, pycram.datastructures.enums.JointType]

      Return the joint types of the virtual mobile base joints.



   .. py:method:: get_axes() -> typing_extensions.Dict[str, pycram.datastructures.pose.Point]

      Return the axes (i.e. The axis on which the joint moves) of the virtual mobile base joints.



.. py:class:: MultiverseMetaData

   Meta data for the Multiverse Client, the simulation_name should be non-empty and unique for each simulation


   .. py:attribute:: world_name
      :type:  str
      :value: 'world'



   .. py:attribute:: simulation_name
      :type:  str
      :value: 'cram'



   .. py:attribute:: length_unit
      :type:  str
      :value: 'm'



   .. py:attribute:: angle_unit
      :type:  str
      :value: 'rad'



   .. py:attribute:: mass_unit
      :type:  str
      :value: 'kg'



   .. py:attribute:: time_unit
      :type:  str
      :value: 's'



   .. py:attribute:: handedness
      :type:  str
      :value: 'rhs'



