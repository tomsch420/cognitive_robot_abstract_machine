from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Type

import geometry_msgs.msg as geometry_msgs
import giskard_msgs.msg as giskard_msgs
import std_msgs.msg as std_msgs
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import TransformStamped, PointStamped, Vector3Stamped
from typing_extensions import Generic, TypeVar, ClassVar

import semantic_digital_twin.spatial_types.spatial_types as cas
from giskardpy.data_types.exceptions import (
    GiskardException,
)
from giskardpy_ros.ros2.visualization_mode import VisualizationMode
from krrood.utils import recursive_subclasses, DataclassException
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
    Point3,
    Vector3,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import (
    Shape,
    Box,
    Cylinder,
    Sphere,
    Mesh,
    Color,
    TriangleMesh,
    FileMesh,
)
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

OurType = TypeVar("OurType")
Ros2Type = TypeVar("Ros2Type")


@dataclass
class ROS2ConversionError(DataclassException): ...


@dataclass
class CannotConvertToRos2Error(ROS2ConversionError): ...


@dataclass
class CannotConvertFromRos2Error(ROS2ConversionError): ...


@dataclass
class ROS2MessageConverter(ABC, Generic[OurType, Ros2Type]):
    our_registry: ClassVar[Dict[Type, Type[ROS2MessageConverter]]] = field(default={})
    ros2_registry: ClassVar[Dict[Type, Type[ROS2MessageConverter]]] = field(default={})

    our_type: Type
    ros2_type: Type

    def __post_init__(self):
        self.our_registry[self.__class__] = self.__class__

    @classmethod
    def get_to_ros2_converter(cls, our_type: Type) -> Type[ROS2MessageConverter]:
        for sub_class in recursive_subclasses(cls):
            if sub_class.our_type == our_type:
                return sub_class

        raise CannotConvertToRos2Error()

    @classmethod
    def get_to_our_converter(cls, ros2_type: Type) -> Type[ROS2MessageConverter]:
        for sub_class in recursive_subclasses(cls):
            if sub_class.ros2_type == ros2_type:
                return sub_class
        raise CannotConvertFromRos2Error()

    @classmethod
    def to_ros2_message(cls, data: OurType) -> Ros2Type:
        return ROS2MessageConverter.get_to_ros2_converter(type(data))._to_ros2_message(
            data
        )

    @classmethod
    @abstractmethod
    def _to_ros2_message(cls, data: OurType) -> Ros2Type:
        pass

    @classmethod
    def from_ros2_message(cls, data: Ros2Type) -> OurType:
        return ROS2MessageConverter.get_to_our_converter(type(data))._from_ros2_message(
            data
        )

    @classmethod
    @abstractmethod
    def _from_ros2_message(cls, data: Ros2Type, world: World) -> OurType:
        pass


@dataclass
class HomogeneousTransformationMatrixROS2Converter(
    ROS2MessageConverter[HomogeneousTransformationMatrix, TransformStamped]
):
    our_type = HomogeneousTransformationMatrix
    ros2_type = TransformStamped

    @classmethod
    def _to_ros2_message(
        cls, data: HomogeneousTransformationMatrix
    ) -> TransformStamped:
        result = TransformStamped()
        if data.reference_frame is not None:
            result.header.frame_id = str(data.reference_frame.name)
        if data.child_frame is not None:
            result.child_frame_id = str(data.child_frame.name)
        position = data.to_position().to_np()
        orientation = data.to_rotation_matrix().to_quaternion().to_np()
        result.transform.translation = geometry_msgs.Vector3(
            x=position[0], y=position[1], z=position[2]
        )
        result.transform.rotation = geometry_msgs.Quaternion(
            x=orientation[0],
            y=orientation[1],
            z=orientation[2],
            w=orientation[3],
        )
        return result

    @classmethod
    def _from_ros2_message(
        cls, data: TransformStamped, world: World
    ) -> HomogeneousTransformationMatrix:
        result = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=data.transform.translation.x,
            pos_y=data.transform.translation.y,
            pos_z=data.transform.translation.z,
            quat_x=data.transform.rotation.x,
            quat_y=data.transform.rotation.y,
            quat_z=data.transform.rotation.z,
            quat_w=data.transform.rotation.w,
            reference_frame=world.get_kinematic_structure_entity_by_name(
                data.header.frame_id
            ),
            child_frame=world.get_kinematic_structure_entity_by_name(
                data.child_frame_id
            ),
        )
        return result


@dataclass
class Point3ROS2Converter(ROS2MessageConverter[Point3, PointStamped]):
    our_type = Point3
    ros2_type = PointStamped

    @classmethod
    def _to_ros2_message(cls, data: Point3) -> PointStamped:
        point_stamped = PointStamped()
        if data.reference_frame is not None:
            point_stamped.header.frame_id = str(data.reference_frame.name)
        position = data.evaluate()
        point_stamped.point = geometry_msgs.Point(
            x=position[0], y=position[1], z=position[2]
        )
        return point_stamped

    @classmethod
    def _from_ros2_message(cls, data: Ros2Type, world: World) -> OurType:
        return Point3(
            data.point.x,
            data.point.y,
            data.point.z,
            reference_frame=world.get_kinematic_structure_entity_by_name(
                data.header.frame_id
            ),
        )


@dataclass
class Vector3ROS2Converter(ROS2MessageConverter[Vector3, Vector3Stamped]):
    our_type = Vector3
    ros2_type = Vector3Stamped

    @classmethod
    def _to_ros2_message(cls, data: Vector3) -> Vector3Stamped:
        vector_stamped = Vector3Stamped()
        if data.reference_frame is not None:
            vector_stamped.header.frame_id = str(data.reference_frame.name)
        vector = data.evaluate()
        vector_stamped.vector = geometry_msgs.Point(
            x=vector[0], y=vector[1], z=vector[2]
        )
        return vector_stamped

    @classmethod
    def _from_ros2_message(cls, data: Ros2Type, world: World) -> Vector3:
        return Vector3(
            data.point.x,
            data.point.y,
            data.point.z,
            reference_frame=world.get_kinematic_structure_entity_by_name(
                data.header.frame_id
            ),
        )


# %% to ro


def to_visualization_marker(data):
    if isinstance(data, Shape):
        return link_geometry_to_visualization_marker(data)


def link_to_visualization_marker(
    data: Body, mode: VisualizationMode
) -> visualization_msgs.MarkerArray:
    markers = visualization_msgs.MarkerArray()
    if mode.is_visual():
        geometries = data.visual
    else:
        geometries = data.collision
    for collision in geometries:
        if isinstance(collision, Box):
            marker = link_geometry_box_to_visualization_marker(collision)
        elif isinstance(collision, Cylinder):
            marker = link_geometry_cylinder_to_visualization_marker(collision)
        elif isinstance(collision, Sphere):
            marker = link_geometry_sphere_to_visualization_marker(collision)
        elif isinstance(collision, Mesh):
            marker = link_geometry_mesh_to_visualization_marker(collision, mode)
            if mode.is_visual():
                marker.mesh_use_embedded_materials = True
                marker.color = std_msgs.ColorRGBA()
        else:
            raise GiskardException(
                f"Can't convert {type(collision)} to visualization marker."
            )
        markers.markers.append(marker)
    return markers


def link_geometry_to_visualization_marker(data: Shape) -> visualization_msgs.Marker:
    marker = visualization_msgs.Marker()
    marker.color = color_rgba_to_ros_msg(data.color)
    marker.pose = to_ros_message(data.origin).pose
    return marker


def link_geometry_sphere_to_visualization_marker(
    data: Sphere,
) -> visualization_msgs.Marker:
    marker = link_geometry_to_visualization_marker(data)
    marker.type = visualization_msgs.Marker.SPHERE
    marker.scale.x = data.radius * 2
    marker.scale.y = data.radius * 2
    marker.scale.z = data.radius * 2
    return marker


def link_geometry_cylinder_to_visualization_marker(
    data: Cylinder,
) -> visualization_msgs.Marker:
    marker = link_geometry_to_visualization_marker(data)
    marker.type = visualization_msgs.Marker.CYLINDER
    marker.scale.x = data.width
    marker.scale.y = data.width
    marker.scale.z = data.height
    return marker


def link_geometry_box_to_visualization_marker(data: Box) -> visualization_msgs.Marker:
    marker = link_geometry_to_visualization_marker(data)
    marker.type = visualization_msgs.Marker.CUBE
    marker.scale.x = data.scale.x
    marker.scale.y = data.scale.y
    marker.scale.z = data.scale.z
    return marker


def link_geometry_mesh_to_visualization_marker(
    data: Mesh, mode: VisualizationMode
) -> visualization_msgs.Marker:
    marker = link_geometry_to_visualization_marker(data)
    marker.type = visualization_msgs.Marker.MESH_RESOURCE
    if mode.is_collision_decomposed():
        marker.mesh_resource = "file://" + data.collision_file_name_absolute
    elif isinstance(data, TriangleMesh):
        marker.mesh_resource = "file://" + data.file.name
    elif isinstance(data, FileMesh):
        marker.mesh_resource = "file://" + data.filename
    marker.scale.x = data.scale.x
    marker.scale.y = data.scale.y
    marker.scale.z = data.scale.z
    marker.mesh_use_embedded_materials = False
    return marker


def color_rgba_to_ros_msg(data: Color) -> std_msgs.ColorRGBA:
    return std_msgs.ColorRGBA(r=data.R, g=data.G, b=data.B, a=data.A)


# %% from ros


def link_name_msg_to_body(
    msg: giskard_msgs.LinkName, world: World
) -> KinematicStructureEntity:
    if msg.group_name == "" and msg.name == "":
        return world.root
    if msg.group_name == "":
        return world.get_kinematic_structure_entity_by_name(msg.name)
    return world.get_kinematic_structure_entity_by_name(
        PrefixedName(msg.name, msg.group_name)
    )


def quaternion_stamped_to_quaternion(
    msg: geometry_msgs.QuaternionStamped, world: World
) -> cas.RotationMatrix:
    return cas.Quaternion(
        msg.quaternion.x,
        msg.quaternion.y,
        msg.quaternion.z,
        msg.quaternion.w,
        reference_frame=world.get_kinematic_structure_entity_by_name(
            msg.header.frame_id
        ),
    ).to_rotation_matrix()
