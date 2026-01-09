pycram.robot_description
========================

.. py:module:: pycram.robot_description


Attributes
----------

.. autoapisummary::

   pycram.robot_description.logger


Classes
-------

.. autoapisummary::

   pycram.robot_description.RobotDescriptionManager
   pycram.robot_description.RobotDescription
   pycram.robot_description.KinematicChainDescription
   pycram.robot_description.CameraDescription
   pycram.robot_description.EndEffectorDescription
   pycram.robot_description.ViewManager


Functions
---------

.. autoapisummary::

   pycram.robot_description.create_manipulator_description


Module Contents
---------------

.. py:data:: logger

.. py:class:: RobotDescriptionManager

   Singleton class to manage multiple robot descriptions. Stores all robot descriptions and loads a robot description
   according to the name of the loaded robot.


   .. py:attribute:: _instance
      :value: None



   .. py:attribute:: descriptions
      :type:  typing_extensions.Dict[str, RobotDescription]


   .. py:attribute:: _initialized
      :value: True



   .. py:method:: load_description(name: str)

      Loads a robot description according to the name of the robot. This required that a robot description with the
      corresponding name is registered.

      :param name: Name of the robot to which the description should be loaded.
      :return: The loaded robot description.



   .. py:method:: register_description(description: RobotDescription)

      Register a robot description to the RobotDescriptionManager. The description is stored with the name of the
      description as key. This will later be used to load the description.

      :param description: RobotDescription to register.



   .. py:method:: register_all_descriptions()
      :staticmethod:



.. py:class:: RobotDescription(name: str, base_link: str, torso_link: str, torso_joint: str, urdf_path: str, virtual_mobile_base_joints: typing_extensions.Optional[pycram.datastructures.dataclasses.VirtualMobileBaseJoints] = None, mjcf_path: typing_extensions.Optional[str] = None, ignore_joints: typing_extensions.Optional[typing_extensions.List[str]] = None, gripper_name: typing_extensions.Optional[str] = None)

   Base class of a robot description. Contains all necessary information about a robot, like the URDF, the base link,
   the torso link and joint, the kinematic chains and cameras.


   .. py:attribute:: current_robot_description
      :type:  RobotDescription
      :value: None


      The currently loaded robot description.



   .. py:attribute:: name
      :type:  str

      Name of the robot



   .. py:attribute:: base_link
      :type:  str

      Base link of the robot



   .. py:attribute:: torso_link
      :type:  str

      Torso link of the robot



   .. py:attribute:: torso_joint
      :type:  str

      Torso joint of the robot



   .. py:attribute:: urdf_object
      :type:  urdf_parser_py.urdf.URDF

      Parsed URDF of the robot



   .. py:attribute:: kinematic_chains
      :type:  typing_extensions.Dict[str, KinematicChainDescription]

      All kinematic chains defined for this robot



   .. py:attribute:: cameras
      :type:  typing_extensions.Dict[str, CameraDescription]

      All cameras defined for this robot



   .. py:attribute:: links
      :type:  typing_extensions.List[str]

      All links defined in the URDF



   .. py:attribute:: joints
      :type:  typing_extensions.List[str]

      All joints defined in the URDF, by default fixed joints are not included



   .. py:attribute:: virtual_mobile_base_joints
      :type:  typing_extensions.Optional[pycram.datastructures.dataclasses.VirtualMobileBaseJoints]
      :value: None


      Virtual mobile base joint names for mobile robots, these joints are not part of the URDF, however they are used to
      move the robot in the simulation (e.g. set_pose for the robot would actually move these joints)



   .. py:attribute:: gripper_name
      :type:  typing_extensions.Optional[str]
      :value: None


      Name of the gripper of the robot if it has one, this is used when the gripper is a different Object with its own
      description file outside the robot description file.



   .. py:attribute:: neck
      :type:  typing_extensions.Dict[str, typing_extensions.List[str]]

      Dictionary of neck links and joints. Keys are yaw, pitch and roll, values are [link, joint]



   .. py:attribute:: ignore_joints


   .. py:attribute:: joint_types


   .. py:attribute:: joint_actuators
      :type:  typing_extensions.Optional[typing_extensions.Dict]


   .. py:method:: add_arm(end_link: str, arm_type: pycram.datastructures.enums.Arms = Arms.RIGHT, arm_name: str = 'manipulator', arm_home_values: typing_extensions.Optional[typing_extensions.Dict[str, float]] = None, arm_start: typing_extensions.Optional[str] = None) -> KinematicChainDescription

      Creates and adds an arm to the RobotDescription.

      :param end_link: Last link of the arm
      :param arm_type: Type of the arm
      :param arm_name: Name of the arm
      :param arm_home_values: Dictionary of joint names and their home values (default configuration) (e.g. park arms)
      :param arm_start: Start link of the arm



   .. py:property:: has_actuators

      Property to check if the robot has actuators defined in the MJCF file.

      :return: True if the robot has actuators, False otherwise



   .. py:method:: get_actuator_for_joint(joint: str) -> typing_extensions.Optional[str]

      Get the actuator name for a given joint.

      :param joint: Name of the joint
      :return: Name of the actuator



   .. py:method:: add_kinematic_chain_description(chain: KinematicChainDescription)

      Adds a KinematicChainDescription object to the RobotDescription. The chain is stored with the name of the chain
      as key.

      :param chain: KinematicChainDescription object to add



   .. py:method:: add_kinematic_chain(name: str, start_link: str, end_link: str)

      Creates and adds a KinematicChainDescription object to the RobotDescription.

      :param name: Name of the KinematicChainDescription object
      :param start_link: First link of the chain
      :param end_link: Last link of the chain



   .. py:method:: add_camera_description(camera: CameraDescription)

      Adds a CameraDescription object to the RobotDescription. The camera is stored with the name of the camera as key.
      :param camera: The CameraDescription object to add



   .. py:method:: add_camera(name: str, camera_link: str, minimal_height: float, maximal_height: float)

      Creates and adds a CameraDescription object to the RobotDescription. Minimal and maximal height of the camera are
      relevant if the robot has a moveable torso or the camera is mounted on a moveable part of the robot. Otherwise
      both values can be the same.

      :param name: Name of the CameraDescription object
      :param camera_link: Link of the camera in the URDF
      :param minimal_height: Minimal height of the camera
      :param maximal_height: Maximal height of the camera
      :return:



   .. py:method:: get_manipulator_chains() -> typing_extensions.List[KinematicChainDescription]

      Get a list of all manipulator chains of the robot which posses an end effector.

      :return: A list of KinematicChainDescription objects



   .. py:method:: get_camera_frame(robot_object_name: str = None) -> str

      Quick method to get the name of a link of a camera. Uses the first camera in the list of cameras.

      :return: A name of the link of a camera



   .. py:method:: get_camera_link() -> str

      Quick method to get the name of a link of a camera. Uses the first camera in the list of cameras.

      :return: A name of the link of a camera



   .. py:method:: get_default_camera() -> CameraDescription

      Get the first camera in the list of cameras.

      :return: A CameraDescription object



   .. py:method:: get_static_joint_chain(kinematic_chain_name: str, configuration_name: typing_extensions.Union[str, enum.Enum])

      Get the static joint states of a kinematic chain for a specific configuration. When trying to access one of
      the robot arms the function `:func: get_arm_chain` should be used.

      :param kinematic_chain_name:
      :param configuration_name:
      :return:



   .. py:method:: get_offset(name: str) -> typing_extensions.Optional[pycram.datastructures.pose.PoseStamped]

      Returns the offset of a Joint in the URDF.

      :param name: The name of the Joint for which the offset will be returned.
      :return: The offset of the Joint



   .. py:method:: get_parent(name: str) -> str

      Get the parent of a link or joint in the URDF. Always returns the immediate parent, for a link this is a joint
      and vice versa.

      :param name: Name of the link or joint in the URDF
      :return: Name of the parent link or joint



   .. py:method:: get_child(name: str, return_multiple_children: bool = False) -> typing_extensions.Union[str, typing_extensions.List[str]]

      Get the child of a link or joint in the URDF. Always returns the immediate child, for a link this is a joint
      and vice versa. Since a link can have multiple children, the return_multiple_children parameter can be set to
      True to get a list of all children.

      :param name: Name of the link or joint in the URDF
      :param return_multiple_children: If True, a list of all children is returned
      :return: Name of the child link or joint or a list of all children



   .. py:method:: get_arm_tool_frame(arm: pycram.datastructures.enums.Arms) -> str

      Get the name of the tool frame of a specific arm.

      :param arm: Arm for which the tool frame should be returned
      :return: The name of the link of the tool frame in the URDF.



   .. py:method:: get_arm_chain(arm: pycram.datastructures.enums.Arms) -> typing_extensions.Union[KinematicChainDescription, typing_extensions.List[KinematicChainDescription]]

      Get the kinematic chain of a specific arm. If the arm is set to BOTH, all kinematic chains are returned.

      :param arm: Arm for which the chain should be returned
      :return: KinematicChainDescription object of the arm



   .. py:method:: set_neck(yaw_joint: typing_extensions.Optional[str] = None, pitch_joint: typing_extensions.Optional[str] = None, roll_joint: typing_extensions.Optional[str] = None)

      Defines the neck configuration of the robot by setting the yaw, pitch, and roll
      joints along with their corresponding links.

      :param yaw_joint: The joint name for the yaw movement of the neck.
      :param pitch_joint: The joint name for the pitch movement of the neck.
      :param roll_joint: The joint name for the roll movement of the neck.



   .. py:method:: get_neck() -> typing_extensions.Dict[str, typing_extensions.List[typing_extensions.Optional[str]]]

      Retrieves the neck configuration of the robot, including links and joints for yaw,
      pitch, and roll.

      :return: A dictionary containing the neck configuration. Keys are yaw, pitch, and roll. Values are [link, joint].



   .. py:method:: load()

      Loads the robot description in the robot description manager, can be overridden to take more parameter into
      account.



   .. py:method:: unload()

      Unloads the robot description in the robot description manager, can be overridden to take more parameter into
      account.



.. py:class:: KinematicChainDescription(name: str, start_link: str, end_link: str, urdf_object: urdf_parser_py.urdf.URDF, arm_type: pycram.datastructures.enums.Arms = None, include_fixed_joints=False)

   Represents a kinematic chain of a robot. A Kinematic chain is a chain of links and joints that are connected to each
   other and can be moved.

   This class contains all necessary information about the chain, like the start and end
   link, the URDF object and the joints of the chain.


   .. py:attribute:: name
      :type:  str

      Name of the chain



   .. py:attribute:: start_link
      :type:  str

      First link of the chain



   .. py:attribute:: end_link
      :type:  str

      Last link of the chain



   .. py:attribute:: urdf_object
      :type:  urdf_parser_py.urdf.URDF

      Parsed URDF of the robot



   .. py:attribute:: include_fixed_joints
      :type:  bool

      If True, fixed joints are included in the chain



   .. py:attribute:: link_names
      :type:  typing_extensions.List[str]

      List of all links in the chain



   .. py:attribute:: joint_names
      :type:  typing_extensions.List[str]

      List of all joints in the chain



   .. py:attribute:: end_effector
      :type:  EndEffectorDescription

      End effector of the chain, if there is one



   .. py:attribute:: arm_type
      :type:  pycram.datastructures.enums.Arms

      Type of the arm, if the chain is an arm



   .. py:attribute:: static_joint_states
      :type:  typing_extensions.Dict[typing_extensions.Union[str, enum.Enum], typing_extensions.Dict[str, float]]

      Dictionary of static joint states for the chain



   .. py:method:: _init_links()

      Initializes the links of the chain by getting the chain from the URDF object.



   .. py:method:: _init_joints()

      Initializes the joints of the chain by getting the chain from the URDF object.



   .. py:method:: create_end_effector(name: str, tool_frame, opened_joint_values: typing_extensions.Dict[str, float], closed_joint_values: typing_extensions.Dict[str, float], relative_dir: typing_extensions.Optional[str] = None, resources_dir: typing_extensions.Optional[str] = None, description_name: str = 'gripper', opening_distance: typing_extensions.Optional[float] = None) -> EndEffectorDescription

      Create a gripper end effector description.

      :param name: The name of the gripper.
      :param tool_frame: The name of the tool frame.
      :param opened_joint_values: The joint values when the gripper is open.
      :param closed_joint_values: The joint values when the gripper is closed.
      :param relative_dir: The relative directory of the gripper in the Multiverse resources/robots directory.
      :param resources_dir: The path to the resources directory.
      :param description_name: The name of the gripper description.
      :param opening_distance: The openning distance of the gripper.
      :return: The gripper end effector description.



   .. py:method:: get_joints() -> typing_extensions.List[str]

      Get a list of all joints of the chain.

      :return: List of joint names



   .. py:method:: get_links() -> typing_extensions.List[str]

      :return: A list of all links of the chain.



   .. py:property:: links
      :type: typing_extensions.List[str]


      Property to get the links of the chain.

      :return: List of link names



   .. py:property:: joints
      :type: typing_extensions.List[str]


      Property to get the joints of the chain.

      :return: List of joint names



   .. py:method:: add_static_joint_states(name: typing_extensions.Union[str, enum.Enum], states: dict)

      Adds static joint states to the chain. These define a specific configuration of the chain.

      :param name: Name of the static joint states
      :param states: Dictionary of joint names and their values



   .. py:method:: get_static_joint_states(name: typing_extensions.Union[str, enum.Enum]) -> typing_extensions.Dict[str, float]

      Get the dictionary of static joint states for a given name of the static joint states.

      :param name: Name of the static joint states
      :return: Dictionary of joint names and their values



   .. py:method:: get_tool_frame() -> str

      Get the name of the tool frame of the end effector of this chain, if it has an end effector.

      :return: The name of the link of the tool frame in the URDF.



   .. py:method:: get_static_gripper_state(state: pycram.datastructures.enums.GripperState) -> typing_extensions.Dict[str, float]

      Get the static joint states for the gripper of the chain.

      :param state: Name of the static joint states
      :return: Dictionary of joint names and their values



.. py:class:: CameraDescription(name: str, link_name: str, minimal_height: float, maximal_height: float, horizontal_angle: float = 20, vertical_angle: float = 20, front_facing_axis: typing_extensions.List[float] = None)

   Represents a camera mounted on a robot. Contains all necessary information about the camera, like the link name,
   minimal and maximal height, horizontal and vertical angle and the front facing axis.


   .. py:attribute:: name
      :type:  str

      Name of the camera



   .. py:attribute:: link_name
      :type:  str

      Name of the link in the URDF



   .. py:attribute:: minimal_height
      :type:  float

      Minimal height the camera can be at



   .. py:attribute:: maximal_height
      :type:  float

      Maximal height the camera can be at



   .. py:attribute:: horizontal_angle
      :type:  float

      Horizontal opening angle of the camera



   .. py:attribute:: vertical_angle
      :type:  float

      Vertical opening angle of the camera



   .. py:attribute:: front_facing_axis
      :type:  typing_extensions.List[int]

      Axis along which the camera is taking the image



.. py:class:: EndEffectorDescription(name: str, start_link: str, tool_frame: str, urdf_object: urdf_parser_py.urdf.URDF, gripper_object_name: typing_extensions.Optional[str] = None, opening_distance: typing_extensions.Optional[float] = None, fingers_link_names: typing_extensions.Optional[typing_extensions.List[str]] = None)

   Describes an end effector of robot. Contains all necessary information about the end effector, like the
   base link, the tool frame, the URDF object and the static joint states.


   .. py:attribute:: name
      :type:  str

      Name of the end effector



   .. py:attribute:: start_link
      :type:  str

      Root link of the end effector, every link below this link in the URDF is part of the end effector



   .. py:attribute:: tool_frame
      :type:  str

      Name of the tool frame link in the URDf



   .. py:attribute:: urdf_object
      :type:  urdf_parser_py.urdf.URDF

      Parsed URDF of the robot



   .. py:attribute:: link_names
      :type:  typing_extensions.List[str]

      List of all links in the end effector



   .. py:attribute:: joint_names
      :type:  typing_extensions.List[str]

      List of all joints in the end effector



   .. py:attribute:: static_joint_states
      :type:  typing_extensions.Dict[pycram.datastructures.enums.GripperState, typing_extensions.Dict[str, float]]

      Dictionary of static joint states for the end effector



   .. py:attribute:: end_effector_type
      :type:  pycram.datastructures.enums.GripperType

      Type of the gripper



   .. py:attribute:: opening_distance
      :type:  float

      Distance the gripper can open, in cm



   .. py:attribute:: gripper_object_name
      :type:  typing_extensions.Optional[str]
      :value: None


      Name of the gripper of the robot if it has one, this is used when the gripper is a different Object with its own
      description file outside the robot description file.



   .. py:attribute:: fingers_link_names
      :type:  typing_extensions.Optional[typing_extensions.List[str]]
      :value: None


      List of all links of the fingers of the gripper



   .. py:attribute:: grasps
      :type:  typing_extensions.Dict[pycram.datastructures.grasp.GraspDescription, typing_extensions.List[float]]

      Dictionary of all grasp orientations of the end effector



   .. py:attribute:: approach_axis
      :type:  typing_extensions.List[float]

      Relative axis along which the end effector is approaching an object



   .. py:method:: _init_links_joints()

      Traverses the URDF object to get all links and joints of the end effector below the start link.1



   .. py:method:: add_static_joint_states(name: pycram.datastructures.enums.GripperState, states: dict)

      Adds static joint states to the end effector. These define a specific configuration of the end effector. Like
      open and close configurations of a gripper.

      :param name: Name of the static joint states
      :param states: Dictionary of joint names and their values



   .. py:property:: links
      :type: typing_extensions.List[str]


      Property to get the links of the chain.

      :return: List of link names



   .. py:property:: joints
      :type: typing_extensions.List[str]


      Property to get the joints of the chain.

      :return: List of joint names



   .. py:method:: update_all_grasp_orientations(front_orientation: typing_extensions.List[float])

      Generates all grasp quaternion orientations based on a given front-facing quaternion orientation in-place,
      covering combinations of side grasps (front, back, left, right),
      top/bottom grasps, and horizontal rotation options.

      :param front_orientation: A quaternion representing the front-facing orientation as [x, y, z, w] quaternion.



   .. py:method:: get_grasp(approach_direction: pycram.datastructures.enums.ApproachDirection, vertical_alignment: pycram.datastructures.enums.VerticalAlignment = VerticalAlignment.NoAlignment, rotate_gripper: bool = False) -> typing_extensions.List[float]

      Retrieves the quaternion orientation of the end effector for a specific grasp.

      :param approach_direction: The approach direction of the end effector.
      :param vertical_alignment: The vertical alignment of the end effector.
      :param rotate_gripper: True, the gripper should be rotated 90°.

      :return: List of floats representing the quaternion orientation of the end effector



   .. py:method:: set_approach_axis(axis: typing_extensions.List[float])

      Sets the approach axis for the robot's palm.

      :param axis: A list representing the approach axis.



   .. py:method:: get_approach_axis() -> typing_extensions.List[float]

      Retrieves the approach axis.

      :return: A list representing the approach axis.



.. py:function:: create_manipulator_description(data: pycram.datastructures.dataclasses.ManipulatorData, urdf_filename: str, mjcf_filename: typing_extensions.Optional[str] = None) -> RobotDescription

   Create a robot description from a ManipulatorData object.

   :param data: ManipulatorData object containing all necessary information about the manipulator.
   :param urdf_filename: Path to the URDF file of the robot.
   :param mjcf_filename: Path to the MJCF file of the robot.
   :return: A RobotDescription object


.. py:class:: ViewManager

   .. py:method:: get_end_effector_view(arm: pycram.datastructures.enums.Arms, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.Manipulator]
      :staticmethod:



   .. py:method:: get_arm_view(arm: pycram.datastructures.enums.Arms, robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.KinematicChain]
      :staticmethod:


      Get the arm view for a given arm and robot view.

      :param arm: The arm to get the view for.
      :param robot_view: The robot view to search in.
      :return: The Manipulator object representing the arm.



   .. py:method:: get_neck_view(robot_view: semantic_digital_twin.robots.abstract_robot.AbstractRobot) -> typing_extensions.Optional[semantic_digital_twin.robots.abstract_robot.Neck]
      :staticmethod:


      Get the neck view for a given robot view.

      :param robot_view: The robot view to search in.
      :return: The Neck object representing the neck.



