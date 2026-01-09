pycram.utils
============

.. py:module:: pycram.utils

.. autoapi-nested-parse::

   Implementation of helper functions and classes for internal usage only.

   Functions:
   _block -- wrap multiple statements into a single block.

   Classes:
   GeneratorList -- implementation of generator list wrappers.



Classes
-------

.. autoapisummary::

   pycram.utils.bcolors
   pycram.utils.GeneratorList
   pycram.utils.suppress_stdout_stderr
   pycram.utils.ClassPropertyDescriptor


Functions
---------

.. autoapisummary::

   pycram.utils.link_pose_for_joint_config
   pycram.utils.get_rays_from_min_max
   pycram.utils.chunks
   pycram.utils.axis_angle_to_quaternion
   pycram.utils.adjust_camera_pose_based_on_target
   pycram.utils.get_quaternion_between_camera_and_target
   pycram.utils.transform_vector_using_pose
   pycram.utils.apply_quaternion_to_pose
   pycram.utils.get_quaternion_between_two_vectors
   pycram.utils.get_axis_angle_between_two_vectors
   pycram.utils.wxyz_to_xyzw
   pycram.utils.xyzw_to_wxyz
   pycram.utils.wxyz_to_xyzw_arr
   pycram.utils.xyzw_to_wxyz_arr
   pycram.utils.classproperty
   pycram.utils.is_iterable
   pycram.utils.lazy_product
   pycram.utils.translate_pose_along_local_axis


Module Contents
---------------

.. py:function:: link_pose_for_joint_config(obj: semantic_digital_twin.world_description.world_entity.Body, joint_config: typing_extensions.Dict[str, float]) -> pycram.datastructures.pose.PoseStamped

   Get the pose a link would be in if the given joint configuration would be applied to the object.
   This is done by using the respective object in the prospection world and applying the joint configuration
   to this one. After applying the joint configuration the link position is taken from there.

   :param obj: The body for which the pose should be calculated
   :param joint_config: Dict with the goal joint configuration
   :return: The pose of the link after applying the joint configuration


.. py:function:: get_rays_from_min_max(min_bound: typing_extensions.Sequence[float], max_bound: typing_extensions.Sequence[float], step_size_in_meters: float = 0.01) -> numpy.ndarray

   Get rays from min and max bounds as an array of start and end 3D points.
   Note: The rays are not steped in the x direction as the rays are cast parallel to the x-axis.

   Example:
   >>> min_bound = [0, 0, 0]
   >>> max_bound = [1, 2, 3]
   >>> rays = get_rays_from_min_max(min_bound, max_bound, 1)
   >>> rays.shape
   (6, 3, 2)
   >>> rays
   array([
   [[0. , 1. ],
    [0. , 0. ],
    [0. , 0. ]],
   [[0. , 1. ],
    [0. , 0. ],
    [1.5, 1.5]],
   [[0. , 1. ],
    [0. , 0. ],
    [3. , 3. ]],
   [[0. , 1. ],
    [2. , 2. ],
    [0. , 0. ]],
   [[0. , 1. ],
    [2. , 2. ],
    [1.5, 1.5]],
   [[0. , 1. ],
    [2. , 2. ],
    [3. , 3. ]]
    ])

   :param min_bound: The minimum bound of the rays, a sequence of 3 floats.
   :param max_bound: The maximum bound of the rays, a sequence of 3 floats.
   :param step_size_in_meters: The step size in meters between the rays.
   :return: The rays as an array of shape (n, 3, 2) where n is number of rays, 3 is because each point has x, y, and z,
   and 2 is for the start and end points of the rays.


.. py:function:: chunks(lst: Union[typing_extensions.List, numpy.ndarray], n: int) -> Iterator[typing_extensions.List]

   Yield successive n-sized chunks from lst.

   :param lst: The list from which chunks should be yielded
   :param n: Size of the chunks
   :return: A list of size n from lst


.. py:class:: bcolors

   Color codes which can be used to highlight Text in the Terminal. For example,
   for warnings.
   Usage:
   Firstly import the class into the file.
   print(f'{bcolors.WARNING} Some Text {bcolors.ENDC}')


   .. py:attribute:: HEADER
      :value: '\x1b[95m'



   .. py:attribute:: OKBLUE
      :value: '\x1b[94m'



   .. py:attribute:: OKCYAN
      :value: '\x1b[96m'



   .. py:attribute:: OKGREEN
      :value: '\x1b[92m'



   .. py:attribute:: WARNING
      :value: '\x1b[93m'



   .. py:attribute:: FAIL
      :value: '\x1b[91m'



   .. py:attribute:: ENDC
      :value: '\x1b[0m'



   .. py:attribute:: BOLD
      :value: '\x1b[1m'



   .. py:attribute:: UNDERLINE
      :value: '\x1b[4m'



.. py:class:: GeneratorList(generator: typing_extensions.Callable)

   Implementation of generator list wrappers.

   Generator lists store the elements of a generator, so these can be fetched multiple times.

   Methods:
   get -- get the element at a specific index.
   has -- check if an element at a specific index exists.


   .. py:attribute:: _generated
      :value: []



   .. py:method:: get(index: int = 0)

      Get the element at a specific index or raise StopIteration if it doesn't exist.

      Arguments:
      index -- the index to get the element of.



   .. py:method:: has(index: int) -> bool

      Check if an element at a specific index exists and return True or False.

      Arguments:
      index -- the index to check for.



.. py:function:: axis_angle_to_quaternion(axis: typing_extensions.List, angle: float) -> typing_extensions.Tuple

   Convert axis-angle to quaternion.

   :param axis: (x, y, z) tuple representing rotation axis.
   :param angle: rotation angle in degree
   :return: The quaternion representing the axis angle


.. py:class:: suppress_stdout_stderr

   Bases: :py:obj:`object`


   A context manager for doing a "deep suppression" of stdout and stderr in
   Python, i.e. will suppress all prints, even if the print originates in a
   compiled C/Fortran sub-function.

   This will not suppress raised exceptions, since exceptions are printed
   to stderr just before a script exits, and after the context manager has
   exited (at least, I think that is why it lets exceptions through).
   Copied from https://stackoverflow.com/questions/11130156/suppress-stdout-stderr-print-from-python-functions


   .. py:attribute:: null_fds


   .. py:attribute:: save_fds


   .. py:method:: __enter__()


   .. py:method:: __exit__(*_)


.. py:function:: adjust_camera_pose_based_on_target(cam_pose: pycram.datastructures.pose.PoseStamped, target_pose: pycram.datastructures.pose.PoseStamped, camera_description: pycram.robot_description.CameraDescription) -> pycram.datastructures.pose.PoseStamped

   Adjust the given cam_pose orientation such that it is facing the target_pose, which partly depends on the
    front_facing_axis of the that is defined in the camera_description.

   :param cam_pose: The camera pose.
   :param target_pose: The target pose.
   :param camera_description: The camera description.
   :return: The adjusted camera pose.


.. py:function:: get_quaternion_between_camera_and_target(cam_pose: pycram.datastructures.pose.PoseStamped, target_pose: pycram.datastructures.pose.PoseStamped, camera_description: pycram.robot_description.CameraDescription) -> numpy.ndarray

   Get the quaternion between the camera and the target.

   :param cam_pose: The camera pose.
   :param target_pose: The target pose.
   :param camera_description: The camera description.
   :return: The quaternion between the camera and the target.


.. py:function:: transform_vector_using_pose(vector: typing_extensions.Sequence, pose) -> numpy.ndarray

   Transform a vector using a pose.

   :param vector: The vector.
   :param pose: The pose.
   :return: The transformed vector.


.. py:function:: apply_quaternion_to_pose(pose: pycram.datastructures.pose.PoseStamped, quaternion: numpy.ndarray) -> pycram.datastructures.pose.PoseStamped

   Apply a quaternion to a pose.

   :param pose: The pose.
   :param quaternion: The quaternion.
   :return: The new pose.


.. py:function:: get_quaternion_between_two_vectors(v1: numpy.ndarray, v2: numpy.ndarray) -> numpy.ndarray

   Get the quaternion between two vectors.

   :param v1: The first vector.
   :param v2: The second vector.
   :return: The quaternion between the two vectors.


.. py:function:: get_axis_angle_between_two_vectors(v1: numpy.ndarray, v2: numpy.ndarray) -> typing_extensions.Tuple[numpy.ndarray, float]

   Get the axis and angle between two vectors.

   :param v1: The first vector.
   :param v2: The second vector.
   :return: The axis and angle between the two vectors.


.. py:function:: wxyz_to_xyzw(wxyz: typing_extensions.List[float]) -> typing_extensions.List[float]

   Convert a quaternion from WXYZ to XYZW format.


.. py:function:: xyzw_to_wxyz(xyzw: typing_extensions.List[float]) -> typing_extensions.List[float]

   Convert a quaternion from XYZW to WXYZ format.

   :param xyzw: The quaternion in XYZW format.


.. py:function:: wxyz_to_xyzw_arr(wxyz: numpy.ndarray) -> numpy.ndarray

   Convert a quaternion from WXYZ to XYZW format.

   :param wxyz: The quaternion in WXYZ format.


.. py:function:: xyzw_to_wxyz_arr(xyzw: numpy.ndarray) -> numpy.ndarray

   Convert a quaternion from XYZW to WXYZ format.

   :param xyzw: The quaternion in XYZW format.


.. py:class:: ClassPropertyDescriptor(fget, fset=None)

   A helper that can be used to define properties of a class like the built-in ones but does not require the class
   to be instantiated.


   .. py:attribute:: fget


   .. py:attribute:: fset
      :value: None



   .. py:method:: __get__(obj, klass=None)


   .. py:method:: __set__(obj, value)


   .. py:method:: setter(func)


.. py:function:: classproperty(func)

.. py:function:: is_iterable(obj: typing_extensions.Any) -> bool

   Checks if the given object is iterable.

   :param obj: The object that should be checked
   :return: True if the object is iterable, False otherwise


.. py:function:: lazy_product(*iterables: typing_extensions.Iterable, iter_names: typing_extensions.List[str] = None) -> typing_extensions.Iterable[typing_extensions.Tuple]

   Lazily generate the cartesian product of the iterables.

   :param iterables: Iterable of iterables to construct product for.
   :param iter_names: Optional names for the iterables for better error messages.
   :return: Iterable of tuples in the cartesian product.


.. py:function:: translate_pose_along_local_axis(pose: pycram.datastructures.pose.PoseStamped, axis: Union[typing_extensions.List, numpy.ndarray], distance: float) -> pycram.datastructures.pose.PoseStamped

   Translate a pose along a given 3d vector (axis) by a given distance. The axis is given in the local coordinate
   frame of the pose. The axis is normalized and then scaled by the distance.

   :param pose: The pose that should be translated
   :param axis: The local axis along which the translation should be performed
   :param distance: The distance by which the pose should be translated

   :return: The translated pose


