"""Handlers for the Spring Bones add-on."""

from __future__ import annotations

import bpy
from bpy.app.handlers import persistent

from .simulation import spring_bone


@persistent
def spring_bone_depsgraph_post(scene, depsgraph):
    """Advance the spring simulation after Blender finishes evaluating."""

    # Only run when the animation system is stepping and your mode is on
    if getattr(scene, "sb_global_spring_frame", False):
        spring_bone(scene, depsgraph=depsgraph)


def register():
    handlers = bpy.app.handlers.depsgraph_update_post
    if spring_bone_depsgraph_post not in handlers:
        handlers.append(spring_bone_depsgraph_post)


def unregister():
    handlers = bpy.app.handlers.depsgraph_update_post
    if spring_bone_depsgraph_post in handlers:
        handlers.remove(spring_bone_depsgraph_post)


__all__ = ["spring_bone_frame_mode"]
