"""Handlers for the Spring Bones add-on."""

from __future__ import annotations

import bpy
from bpy.app.handlers import persistent
from .simulation import spring_bone

@persistent
def spring_bone_depsgraph_pre(scene, depsgraph):
    # Only run when the animation system is stepping and your mode is on
    if getattr(scene, "sb_global_spring_frame", False):
        spring_bone(scene, depsgraph=depsgraph)

def register():
    h = bpy.app.handlers.depsgraph_update_pre
    if spring_bone_depsgraph_pre not in h:
        h.append(spring_bone_depsgraph_pre)

def unregister():
    h = bpy.app.handlers.depsgraph_update_pre
    if spring_bone_depsgraph_pre in h:
        h.remove(spring_bone_depsgraph_pre)

__all__ = ["spring_bone_frame_mode"]
