"""Handlers for the Spring Bones add-on."""

from __future__ import annotations

import bpy
from bpy.app.handlers import persistent

from .simulation import spring_bone


@persistent
def spring_bone_frame_mode(_scene):
    if bpy.context.scene.sb_global_spring_frame is True:
        spring_bone(_scene)


def register():
    if spring_bone_frame_mode not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(spring_bone_frame_mode)


def unregister():
    try:
        bpy.app.handlers.frame_change_post.remove(spring_bone_frame_mode)
    except ValueError:
        pass


__all__ = ["spring_bone_frame_mode"]
