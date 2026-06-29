"""Annotator for writing a configured static camera transform into the CAS.
Use this in scenarios where you do not have a world to camera transform published
or when you develop the perception system in isolation."""

from __future__ import annotations

from timeit import default_timer

from py_trees.common import Status

from robokudo import world as rk_world
from robokudo.annotators.core import BaseAnnotator
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix


class StaticCameraTransformAnnotator(BaseAnnotator):
    """Write a fixed camera-to-world transform to the current CAS."""

    class Descriptor(BaseAnnotator.Descriptor):
        class Parameters:
            def __init__(self) -> None:
                self.world_frame: str = "map"
                self.camera_frame: str = "camera"
                self.translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
                self.rotation_xyzw: tuple[float, float, float, float] = (
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                )

        parameters = Parameters()

    def __init__(
        self,
        name: str = "StaticCameraTransform",
        descriptor: StaticCameraTransformAnnotator.Descriptor = Descriptor(),
    ) -> None:
        super().__init__(name=name, descriptor=descriptor)

    def update(self) -> Status:
        start_timer = default_timer()

        cas = self.get_cas()
        params = self.descriptor.parameters

        if len(params.translation) != 3:
            raise ValueError("translation must contain exactly 3 values")
        if len(params.rotation_xyzw) != 4:
            raise ValueError("rotation_xyzw must contain exactly 4 values")

        rk_world.setup_world_for_camera_frame(
            world_frame=params.world_frame,
            camera_frame=params.camera_frame,
        )

        sem_world = rk_world.world_instance()
        world_body = sem_world.get_body_by_name(params.world_frame)
        camera_body = sem_world.get_body_by_name(params.camera_frame)

        translation = tuple(float(value) for value in params.translation)
        rotation = tuple(float(value) for value in params.rotation_xyzw)

        camera_to_world_transform = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=translation[0],
            pos_y=translation[1],
            pos_z=translation[2],
            quat_x=rotation[0],
            quat_y=rotation[1],
            quat_z=rotation[2],
            quat_w=rotation[3],
            reference_frame=world_body,
            child_frame=camera_body,
        )

        cas.world_frame = params.world_frame
        cas.cam_frame = params.camera_frame
        cas.cam_to_world_transform = camera_to_world_transform

        rk_world.update_connection_transform(
            to_name=world_body.name,
            from_name=camera_body.name,
            transform=camera_to_world_transform,
        )

        end_timer = default_timer()
        self.feedback_message = f"Processing took {(end_timer - start_timer):.4f}s"
        return Status.SUCCESS
