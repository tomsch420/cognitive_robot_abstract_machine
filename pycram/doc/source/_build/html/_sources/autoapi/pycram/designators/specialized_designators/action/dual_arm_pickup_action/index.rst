pycram.designators.specialized_designators.action.dual_arm_pickup_action
========================================================================

.. py:module:: pycram.designators.specialized_designators.action.dual_arm_pickup_action


Classes
-------

.. autoapisummary::

   pycram.designators.specialized_designators.action.dual_arm_pickup_action.DualArmPickupAction


Module Contents
---------------

.. py:class:: DualArmPickupAction(object_designator_description: typing_extensions.Union[pycram.designator.ObjectDesignatorDescription, pycram.designator.ObjectDesignatorDescription.Object], grasps: typing_extensions.List[pycram.datastructures.enums.Grasp], resolver=None)

   Bases: :py:obj:`pycram.robot_plans.PickUpAction`


   Specialization version of the PickUpAction designator which uses heuristics to solve for a dual pickup solution.


   .. py:attribute:: object_designator_description
      :type:  typing_extensions.Union[pycram.designator.ObjectDesignatorDescription, pycram.designator.ObjectDesignatorDescription.Object]


   .. py:attribute:: gripper_list
      :type:  typing_extensions.List[pycram.robot_description.KinematicChainDescription]


   .. py:method:: ground() -> pycram.robot_plans.PickUpAction


