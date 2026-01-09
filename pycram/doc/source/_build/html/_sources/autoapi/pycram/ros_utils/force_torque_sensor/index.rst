pycram.ros_utils.force_torque_sensor
====================================

.. py:module:: pycram.ros_utils.force_torque_sensor


Attributes
----------

.. autoapisummary::

   pycram.ros_utils.force_torque_sensor.logger


Classes
-------

.. autoapisummary::

   pycram.ros_utils.force_torque_sensor.ForceTorqueSensorSimulated
   pycram.ros_utils.force_torque_sensor.ForceTorqueSensor


Module Contents
---------------

.. py:data:: logger

.. py:class:: ForceTorqueSensorSimulated(joint_name, world: semantic_digital_twin.world.World, fts_topic='/pycram/fts', interval=0.1)

   Simulated force-torque sensor for a joint with a given name.
   Reads simulated forces and torques at that joint from world and publishes geometry_msgs/Wrench messages
   to the given topic.


   .. py:attribute:: world


   .. py:attribute:: fts_joint_idx
      :value: None



   .. py:attribute:: joint_name


   .. py:attribute:: fts_pub


   .. py:attribute:: interval
      :value: 0.1



   .. py:attribute:: kill_event


   .. py:attribute:: thread


   .. py:method:: _publish() -> None

      Continuously publishes the force-torque values for the simulated joint. Values are published as long as the
      kill_event is not set.



   .. py:method:: _stop_publishing() -> None

      Sets the kill_event and therefore terminates the Thread publishing the force-torque values as well as join the
      threads.



.. py:class:: ForceTorqueSensor(robot_name, filter_config=FilterConfig.butterworth, filter_order=4, custom_topic=None, debug=False)

   Monitor a force-torque sensor of a supported robot and save relevant data.

   Apply a specified filter and save this data as well.
   Default filter is the low pass filter 'Butterworth'

   Can also calculate the derivative of (un-)filtered data

   :param robot_name: Name of the robot
   :param filter_config: Desired filter (default: Butterworth)
   :param filter_order: Order of the filter. Declares the number of elements that delay the sampling
   :param custom_topic: Declare a custom topic if the default topics do not fit


   .. py:attribute:: filtered
      :value: 'filtered'



   .. py:attribute:: unfiltered
      :value: 'unfiltered'



   .. py:attribute:: robot_name


   .. py:attribute:: filter_config


   .. py:attribute:: filter


   .. py:attribute:: debug
      :value: False



   .. py:attribute:: wrench_topic_name
      :value: None



   .. py:attribute:: force_torque_subscriber
      :value: None



   .. py:attribute:: init_data
      :value: True



   .. py:attribute:: whole_data
      :value: None



   .. py:attribute:: prev_values
      :value: None



   .. py:attribute:: order
      :value: 4



   .. py:method:: _setup()


   .. py:method:: _get_robot_parameters()


   .. py:method:: _get_rospy_data(data_compensated: geometry_msgs.msg.WrenchStamped)


   .. py:method:: _get_filter(order=4, cutoff=10, fs=60)


   .. py:method:: _filter_data(current_wrench_data: geometry_msgs.msg.WrenchStamped) -> geometry_msgs.msg.WrenchStamped


   .. py:method:: subscribe()

      Subscribe to the specified wrench topic.

      This will automatically be called on setup.
      Only use this if you already unsubscribed before.



   .. py:method:: unsubscribe()

      Unsubscribe from the specified topic



   .. py:method:: get_last_value(is_filtered=True) -> geometry_msgs.msg.WrenchStamped

      Get the most current data values.

      :param is_filtered: Decides about using filtered or raw data

      :return: A list containing the most current values (newest are first)



   .. py:method:: get_derivative(is_filtered=True) -> geometry_msgs.msg.WrenchStamped

      Calculate the derivative of current data.

      :param is_filtered: Decides about using filtered or raw data.
      :return: The derivative as a WrenchStamped object. Returns a zeroed derivative if only one data point exists.



   .. py:method:: human_touch_monitoring(plan)


