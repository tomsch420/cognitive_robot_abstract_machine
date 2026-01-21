import pytest
from tf2_py import LookupException

from semantic_digital_twin.adapters.ros.msg_converter import (
    HomogeneousTransformationMatrixROS2Converter,
)
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.tfwrapper import TFWrapper


def test_tf_publisher(rclpy_node, pr2_world_state_reset):
    r_gripper_tool_frame = (
        pr2_world_state_reset.get_kinematic_structure_entities_by_name(
            "r_gripper_tool_frame"
        )
    )
    tf_wrapper = TFWrapper(node=rclpy_node)
    tf_publisher = TFPublisher(
        node=rclpy_node,
        world=pr2_world_state_reset,
        ignored_kinematic_structure_entities=r_gripper_tool_frame,
    )

    transform = tf_wrapper.lookup_transform("odom_combined", "base_footprint")
    odom_combined = pr2_world_state_reset.get_kinematic_structure_entities_by_name(
        "odom_combined"
    )[0]
    base_footprint = pr2_world_state_reset.get_kinematic_structure_entities_by_name(
        "base_footprint"
    )[0]
    fk = pr2_world_state_reset.compute_forward_kinematics(odom_combined, base_footprint)
    transform2 = HomogeneousTransformationMatrixROS2Converter.to_ros2_message(fk)
    assert transform.transform == transform2.transform

    with pytest.raises(LookupException):
        tf_wrapper.lookup_transform("odom_combined", "r_gripper_tool_frame")
