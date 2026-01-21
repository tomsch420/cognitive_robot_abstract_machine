from geometry_msgs.msg import TransformStamped

from semantic_digital_twin.adapters.ros.msg_converter import (
    HomogeneousTransformationMatrixROS2Converter,
)


def test_convert_transform():
    transform = TransformStamped()
    transform.transform.translation.x = 1.0
    transform.transform.rotation.x = 1.0
    transform.transform.rotation.w = 0.0
    transformation_matrix = (
        HomogeneousTransformationMatrixROS2Converter.from_ros2_message(transform)
    )
    position = transformation_matrix.to_position().evaluate()
    rotation = transformation_matrix.to_quaternion().evaluate()
    assert position[0] == transform.transform.translation.x
    assert position[1] == transform.transform.translation.y
    assert position[2] == transform.transform.translation.z

    assert rotation[0] == transform.transform.rotation.x
    assert rotation[1] == transform.transform.rotation.y
    assert rotation[2] == transform.transform.rotation.z
    assert rotation[3] == transform.transform.rotation.w

    transform2 = HomogeneousTransformationMatrixROS2Converter.to_ros2_message(
        transformation_matrix
    )
    assert transform == transform2
