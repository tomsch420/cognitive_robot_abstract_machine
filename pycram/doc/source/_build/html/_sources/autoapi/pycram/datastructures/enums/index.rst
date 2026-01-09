pycram.datastructures.enums
===========================

.. py:module:: pycram.datastructures.enums

.. autoapi-nested-parse::

   Module holding all enums of PyCRAM.



Attributes
----------

.. autoapisummary::

   pycram.datastructures.enums.MJCFBodyType


Classes
-------

.. autoapisummary::

   pycram.datastructures.enums.AdjacentBodyMethod
   pycram.datastructures.enums.ContainerManipulationType
   pycram.datastructures.enums.FindBodyInRegionMethod
   pycram.datastructures.enums.StaticJointState
   pycram.datastructures.enums.DescriptionType
   pycram.datastructures.enums.ExecutionType
   pycram.datastructures.enums.Arms
   pycram.datastructures.enums.TaskStatus
   pycram.datastructures.enums.JointType
   pycram.datastructures.enums.AxisIdentifier
   pycram.datastructures.enums.Grasp
   pycram.datastructures.enums.ApproachDirection
   pycram.datastructures.enums.VerticalAlignment
   pycram.datastructures.enums.ObjectType
   pycram.datastructures.enums.Shape
   pycram.datastructures.enums.TorsoState
   pycram.datastructures.enums.WorldMode
   pycram.datastructures.enums.GripperState
   pycram.datastructures.enums.GripperType
   pycram.datastructures.enums.ImageEnum
   pycram.datastructures.enums.DetectionTechnique
   pycram.datastructures.enums.DetectionState
   pycram.datastructures.enums.LoggerLevel
   pycram.datastructures.enums.VirtualMobileBaseJointName
   pycram.datastructures.enums.MJCFGeomType
   pycram.datastructures.enums.MJCFJointType
   pycram.datastructures.enums.MovementType
   pycram.datastructures.enums.WaypointsMovementType
   pycram.datastructures.enums.MultiverseAPIName
   pycram.datastructures.enums.MultiverseProperty
   pycram.datastructures.enums.MultiverseBodyProperty
   pycram.datastructures.enums.MultiverseJointProperty
   pycram.datastructures.enums.MultiverseJointPosition
   pycram.datastructures.enums.MultiverseJointCMD
   pycram.datastructures.enums.FilterConfig
   pycram.datastructures.enums.MonitorBehavior


Module Contents
---------------

.. py:class:: AdjacentBodyMethod(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Create a collection of name/value pairs.

   Example enumeration:

   >>> class Color(Enum):
   ...     RED = 1
   ...     BLUE = 2
   ...     GREEN = 3

   Access them by:

   - attribute access:

     >>> Color.RED
     <Color.RED: 1>

   - value lookup:

     >>> Color(1)
     <Color.RED: 1>

   - name lookup:

     >>> Color['RED']
     <Color.RED: 1>

   Enumerations can be iterated over, and know how many members they have:

   >>> len(Color)
   3

   >>> list(Color)
   [<Color.RED: 1>, <Color.BLUE: 2>, <Color.GREEN: 3>]

   Methods can be added to enumerations, and members can have their own
   attributes -- see the documentation for details.


   .. py:attribute:: ClosestPoints

      The ClosestPoints method is used to find the closest points in other bodies to the body.



   .. py:attribute:: RayCasting

      The RayCasting method is used to find the points in other bodies that are intersected by rays cast
       from the body bounding box to 6 directions (up, down, left, right, front, back).



.. py:class:: ContainerManipulationType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different types of container manipulation.


   .. py:attribute:: Opening

      The Opening type is used to open a container.



   .. py:attribute:: Closing

      The Closing type is used to close a container.



.. py:class:: FindBodyInRegionMethod(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different methods to find a body in a region.


   .. py:attribute:: FingerToCentroid

      The FingerToCentroid method is used to find the body in a region by casting a ray from each finger to the
       centroid of the region.



   .. py:attribute:: Centroid

      The Centroid method is used to find the body in a region by calculating the centroid of the region and
      casting two rays from opposite sides of the region to the centroid.



   .. py:attribute:: MultiRay

      The MultiRay method is used to find the body in a region by casting multiple rays covering the region.



.. py:class:: StaticJointState(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Create a collection of name/value pairs.

   Example enumeration:

   >>> class Color(Enum):
   ...     RED = 1
   ...     BLUE = 2
   ...     GREEN = 3

   Access them by:

   - attribute access:

     >>> Color.RED
     <Color.RED: 1>

   - value lookup:

     >>> Color(1)
     <Color.RED: 1>

   - name lookup:

     >>> Color['RED']
     <Color.RED: 1>

   Enumerations can be iterated over, and know how many members they have:

   >>> len(Color)
   3

   >>> list(Color)
   [<Color.RED: 1>, <Color.BLUE: 2>, <Color.GREEN: 3>]

   Methods can be added to enumerations, and members can have their own
   attributes -- see the documentation for details.


   .. py:attribute:: Park
      :value: 'park'



.. py:class:: DescriptionType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Create a collection of name/value pairs.

   Example enumeration:

   >>> class Color(Enum):
   ...     RED = 1
   ...     BLUE = 2
   ...     GREEN = 3

   Access them by:

   - attribute access:

     >>> Color.RED
     <Color.RED: 1>

   - value lookup:

     >>> Color(1)
     <Color.RED: 1>

   - name lookup:

     >>> Color['RED']
     <Color.RED: 1>

   Enumerations can be iterated over, and know how many members they have:

   >>> len(Color)
   3

   >>> list(Color)
   [<Color.RED: 1>, <Color.BLUE: 2>, <Color.GREEN: 3>]

   Methods can be added to enumerations, and members can have their own
   attributes -- see the documentation for details.


   .. py:attribute:: URDF
      :value: 'urdf'



   .. py:attribute:: MJCF
      :value: 'mjcf'



   .. py:method:: get_file_extension()


.. py:class:: ExecutionType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for Execution Process Module types.


   .. py:attribute:: REAL


   .. py:attribute:: SIMULATED


   .. py:attribute:: SEMI_REAL


.. py:class:: Arms

   Bases: :py:obj:`enum.IntEnum`


   Enum for Arms.


   .. py:attribute:: LEFT
      :value: 0



   .. py:attribute:: RIGHT
      :value: 1



   .. py:attribute:: BOTH
      :value: 2



   .. py:method:: __str__()

      Return str(self).



   .. py:method:: __repr__()

      Return repr(self).



.. py:class:: TaskStatus

   Bases: :py:obj:`int`, :py:obj:`enum.Enum`


   Enum for readable descriptions of a tasks' status.


   .. py:attribute:: CREATED
      :value: 0



   .. py:attribute:: RUNNING
      :value: 1



   .. py:attribute:: SUCCEEDED
      :value: 2



   .. py:attribute:: FAILED
      :value: 3



   .. py:attribute:: INTERRUPTED
      :value: 4



   .. py:attribute:: SLEEPING
      :value: 5



.. py:class:: JointType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for readable joint types.


   .. py:attribute:: REVOLUTE
      :value: 0



   .. py:attribute:: PRISMATIC
      :value: 1



   .. py:attribute:: SPHERICAL
      :value: 2



   .. py:attribute:: PLANAR
      :value: 3



   .. py:attribute:: FIXED
      :value: 4



   .. py:attribute:: UNKNOWN
      :value: 5



   .. py:attribute:: CONTINUOUS
      :value: 6



   .. py:attribute:: FLOATING
      :value: 7



.. py:class:: AxisIdentifier(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for translating the axis name to a vector along that axis.


   .. py:attribute:: X
      :value: (1, 0, 0)



   .. py:attribute:: Y
      :value: (0, 1, 0)



   .. py:attribute:: Z
      :value: (0, 0, 1)



   .. py:attribute:: Undefined
      :value: (0, 0, 0)



   .. py:method:: from_tuple(axis_tuple)
      :classmethod:



.. py:class:: Grasp

   Base class for grasp enums.


   .. py:method:: __hash__()


   .. py:method:: from_axis_direction(axis: AxisIdentifier, direction: int)
      :classmethod:


      Get the Grasp face from an axis-index tuple



.. py:class:: ApproachDirection(*args, **kwds)

   Bases: :py:obj:`Grasp`, :py:obj:`enum.Enum`


   Enum for the approach direction of a gripper.
   The AxisIdentifier is used to identify the axis of the gripper, and the int is used to identify the direction along
    that axis.


   .. py:attribute:: FRONT


   .. py:attribute:: BACK


   .. py:attribute:: RIGHT


   .. py:attribute:: LEFT


   .. py:property:: axis
      :type: AxisIdentifier


      Returns the axis of the approach direction.



.. py:class:: VerticalAlignment(*args, **kwds)

   Bases: :py:obj:`Grasp`, :py:obj:`enum.Enum`


   Enum for the vertical alignment of a gripper.
   The AxisIdentifier is used to identify the axis of the gripper, and the int is used to identify the direction along
    that axis.


   .. py:attribute:: TOP


   .. py:attribute:: BOTTOM


   .. py:attribute:: NoAlignment


.. py:class:: ObjectType

   Bases: :py:obj:`int`, :py:obj:`enum.Enum`


   Enum for Object types to easier identify different objects


   .. py:attribute:: METALMUG


   .. py:attribute:: PRINGLES


   .. py:attribute:: MILK


   .. py:attribute:: SPOON


   .. py:attribute:: BOWL


   .. py:attribute:: BREAKFAST_CEREAL


   .. py:attribute:: JEROEN_CUP


   .. py:attribute:: ROBOT


   .. py:attribute:: GRIPPER


   .. py:attribute:: ENVIRONMENT


   .. py:attribute:: GENERIC_OBJECT


   .. py:attribute:: HUMAN


   .. py:attribute:: IMAGINED_SURFACE


.. py:class:: Shape(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for visual shapes of objects


   .. py:attribute:: SPHERE
      :value: 2



   .. py:attribute:: BOX
      :value: 3



   .. py:attribute:: CYLINDER
      :value: 4



   .. py:attribute:: MESH
      :value: 5



   .. py:attribute:: PLANE
      :value: 6



   .. py:attribute:: CAPSULE
      :value: 7



.. py:class:: TorsoState

   Bases: :py:obj:`enum.IntEnum`


   Enum for the different states of the torso.


   .. py:attribute:: HIGH


   .. py:attribute:: MID


   .. py:attribute:: LOW


.. py:class:: WorldMode(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different modes of the world.


   .. py:attribute:: GUI
      :value: 'GUI'



   .. py:attribute:: DIRECT
      :value: 'DIRECT'



.. py:class:: GripperState(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different motions of the gripper.


   .. py:attribute:: OPEN


   .. py:attribute:: CLOSE


   .. py:attribute:: MEDIUM


   .. py:method:: __str__()


   .. py:method:: __repr__()


.. py:class:: GripperType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different types of grippers.


   .. py:attribute:: PARALLEL


   .. py:attribute:: SUCTION


   .. py:attribute:: FINGER


   .. py:attribute:: HYDRAULIC


   .. py:attribute:: PNEUMATIC


   .. py:attribute:: CUSTOM


.. py:class:: ImageEnum(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for image switch view on hsrb display.


   .. py:attribute:: HI
      :value: 0



   .. py:attribute:: TALK
      :value: 1



   .. py:attribute:: DISH
      :value: 2



   .. py:attribute:: DONE
      :value: 3



   .. py:attribute:: DROP
      :value: 4



   .. py:attribute:: HANDOVER
      :value: 5



   .. py:attribute:: ORDER
      :value: 6



   .. py:attribute:: PICKING
      :value: 7



   .. py:attribute:: PLACING
      :value: 8



   .. py:attribute:: REPEAT
      :value: 9



   .. py:attribute:: SEARCH
      :value: 10



   .. py:attribute:: WAVING
      :value: 11



   .. py:attribute:: FOLLOWING
      :value: 12



   .. py:attribute:: DRIVINGBACK
      :value: 13



   .. py:attribute:: PUSHBUTTONS
      :value: 14



   .. py:attribute:: FOLLOWSTOP
      :value: 15



   .. py:attribute:: JREPEAT
      :value: 16



   .. py:attribute:: SOFA
      :value: 17



   .. py:attribute:: INSPECT
      :value: 18



   .. py:attribute:: CHAIR
      :value: 37



.. py:class:: DetectionTechnique

   Bases: :py:obj:`int`, :py:obj:`enum.Enum`


   Enum for techniques for detection tasks.


   .. py:attribute:: ALL
      :value: 0



   .. py:attribute:: HUMAN
      :value: 1



   .. py:attribute:: TYPES
      :value: 2



   .. py:attribute:: REGION
      :value: 3



   .. py:attribute:: HUMAN_ATTRIBUTES
      :value: 4



   .. py:attribute:: HUMAN_WAVING
      :value: 5



.. py:class:: DetectionState

   Bases: :py:obj:`int`, :py:obj:`enum.Enum`


   Enum for the state of the detection task.


   .. py:attribute:: START
      :value: 0



   .. py:attribute:: STOP
      :value: 1



   .. py:attribute:: PAUSE
      :value: 2



.. py:class:: LoggerLevel(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different logger levels.


   .. py:attribute:: DEBUG
      :value: 'debug'



   .. py:attribute:: INFO
      :value: 'info'



   .. py:attribute:: WARN
      :value: 'warn'



   .. py:attribute:: ERROR
      :value: 'error'



   .. py:attribute:: FATAL
      :value: 'fatal'



.. py:class:: VirtualMobileBaseJointName(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the joint names of the virtual mobile base.


   .. py:attribute:: LINEAR_X
      :value: 'odom_vel_lin_x_joint'



   .. py:attribute:: LINEAR_Y
      :value: 'odom_vel_lin_y_joint'



   .. py:attribute:: ANGULAR_Z
      :value: 'odom_vel_ang_z_joint'



.. py:class:: MJCFGeomType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different geom types in a MuJoCo XML file.


   .. py:attribute:: BOX
      :value: 'box'



   .. py:attribute:: CYLINDER
      :value: 'cylinder'



   .. py:attribute:: CAPSULE
      :value: 'capsule'



   .. py:attribute:: SPHERE
      :value: 'sphere'



   .. py:attribute:: PLANE
      :value: 'plane'



   .. py:attribute:: MESH
      :value: 'mesh'



   .. py:attribute:: ELLIPSOID
      :value: 'ellipsoid'



   .. py:attribute:: HFIELD
      :value: 'hfield'



   .. py:attribute:: SDF
      :value: 'sdf'



.. py:data:: MJCFBodyType

   Alias for MJCFGeomType. As the body type is the same as the geom type.


.. py:class:: MJCFJointType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different joint types in a MuJoCo XML file.


   .. py:attribute:: FREE
      :value: 'free'



   .. py:attribute:: BALL
      :value: 'ball'



   .. py:attribute:: SLIDE
      :value: 'slide'



   .. py:attribute:: HINGE
      :value: 'hinge'



   .. py:attribute:: FIXED
      :value: 'fixed'



.. py:class:: MovementType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different movement types of the robot.


   .. py:attribute:: STRAIGHT_TRANSLATION


   .. py:attribute:: STRAIGHT_CARTESIAN


   .. py:attribute:: TRANSLATION


   .. py:attribute:: CARTESIAN


.. py:class:: WaypointsMovementType(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different movement types of the robot.


   .. py:attribute:: ENFORCE_ORIENTATION_STRICT


   .. py:attribute:: ENFORCE_ORIENTATION_FINAL_POINT


.. py:class:: MultiverseAPIName(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different APIs of the Multiverse.


   .. py:attribute:: GET_CONTACT_POINTS
      :value: 'get_contact_points'



   .. py:attribute:: GET_CONTACT_BODIES
      :value: 'get_contact_bodies'



   .. py:attribute:: GET_CONTACT_BODIES_AND_POINTS
      :value: 'get_contact_bodies_and_points'



   .. py:attribute:: GET_CONSTRAINT_EFFORT
      :value: 'get_constraint_effort'



   .. py:attribute:: GET_BOUNDING_BOX
      :value: 'get_bounding_box'



   .. py:attribute:: ATTACH
      :value: 'attach'



   .. py:attribute:: DETACH
      :value: 'detach'



   .. py:attribute:: GET_RAYS
      :value: 'get_rays'



   .. py:attribute:: EXIST
      :value: 'exist'



   .. py:attribute:: PAUSE
      :value: 'pause'



   .. py:attribute:: UNPAUSE
      :value: 'unpause'



   .. py:attribute:: SAVE
      :value: 'save'



   .. py:attribute:: LOAD
      :value: 'load'



.. py:class:: MultiverseProperty(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Create a collection of name/value pairs.

   Example enumeration:

   >>> class Color(Enum):
   ...     RED = 1
   ...     BLUE = 2
   ...     GREEN = 3

   Access them by:

   - attribute access:

     >>> Color.RED
     <Color.RED: 1>

   - value lookup:

     >>> Color(1)
     <Color.RED: 1>

   - name lookup:

     >>> Color['RED']
     <Color.RED: 1>

   Enumerations can be iterated over, and know how many members they have:

   >>> len(Color)
   3

   >>> list(Color)
   [<Color.RED: 1>, <Color.BLUE: 2>, <Color.GREEN: 3>]

   Methods can be added to enumerations, and members can have their own
   attributes -- see the documentation for details.


   .. py:method:: __str__()


.. py:class:: MultiverseBodyProperty(*args, **kwds)

   Bases: :py:obj:`MultiverseProperty`


   Enum for the different properties of a body the Multiverse.


   .. py:attribute:: POSITION
      :value: 'position'



   .. py:attribute:: ORIENTATION
      :value: 'quaternion'



   .. py:attribute:: RELATIVE_VELOCITY
      :value: 'relative_velocity'



.. py:class:: MultiverseJointProperty(*args, **kwds)

   Bases: :py:obj:`MultiverseProperty`


   Create a collection of name/value pairs.

   Example enumeration:

   >>> class Color(Enum):
   ...     RED = 1
   ...     BLUE = 2
   ...     GREEN = 3

   Access them by:

   - attribute access:

     >>> Color.RED
     <Color.RED: 1>

   - value lookup:

     >>> Color(1)
     <Color.RED: 1>

   - name lookup:

     >>> Color['RED']
     <Color.RED: 1>

   Enumerations can be iterated over, and know how many members they have:

   >>> len(Color)
   3

   >>> list(Color)
   [<Color.RED: 1>, <Color.BLUE: 2>, <Color.GREEN: 3>]

   Methods can be added to enumerations, and members can have their own
   attributes -- see the documentation for details.


.. py:class:: MultiverseJointPosition(*args, **kwds)

   Bases: :py:obj:`MultiverseJointProperty`


   Enum for the Position names of the different joint types in the Multiverse.


   .. py:attribute:: REVOLUTE_JOINT_POSITION
      :value: 'joint_rvalue'



   .. py:attribute:: PRISMATIC_JOINT_POSITION
      :value: 'joint_tvalue'



   .. py:method:: from_pycram_joint_type(joint_type: JointType) -> MultiverseJointPosition
      :classmethod:



.. py:class:: MultiverseJointCMD(*args, **kwds)

   Bases: :py:obj:`MultiverseJointProperty`


   Enum for the Command names of the different joint types in the Multiverse.


   .. py:attribute:: REVOLUTE_JOINT_CMD
      :value: 'cmd_joint_rvalue'



   .. py:attribute:: PRISMATIC_JOINT_CMD
      :value: 'cmd_joint_tvalue'



   .. py:method:: from_pycram_joint_type(joint_type: JointType) -> MultiverseJointCMD
      :classmethod:



.. py:class:: FilterConfig(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Declare existing filter methods.
   Currently supported: Butterworth


   .. py:attribute:: butterworth
      :value: 1



.. py:class:: MonitorBehavior(*args, **kwds)

   Bases: :py:obj:`enum.Enum`


   Enum for the different monitor behaviors.


   .. py:attribute:: INTERRUPT

      Interrupt the task when the condition is met.



   .. py:attribute:: PAUSE

      Pause the task when the condition is met.



   .. py:attribute:: RESUME

      Resume the task when the condition is met.



