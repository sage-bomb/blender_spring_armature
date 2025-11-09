"""Handlers for the Spring Bones add-on."""

from __future__ import annotations

import bpy
from bpy.app.handlers import persistent

from .simulation import spring_bone


@persistent
def spring_bone_frame_post(scene: bpy.types.Scene) -> None:
    """Advance the spring simulation once per animation frame."""

    if getattr(scene, "sb_global_spring_frame", False):
        depsgraph = bpy.context.view_layer.depsgraph
        spring_bone(scene, depsgraph=depsgraph)


def register() -> None:
    handlers = bpy.app.handlers.frame_change_post
    if spring_bone_frame_post not in handlers:
        handlers.append(spring_bone_frame_post)


def unregister() -> None:
    handlers = bpy.app.handlers.frame_change_post
    if spring_bone_frame_post in handlers:
        handlers.remove(spring_bone_frame_post)


__all__ = ["spring_bone_frame_post", "register", "unregister"]
