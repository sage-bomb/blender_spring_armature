"""Utility helpers for the Spring Bones add-on."""

from __future__ import annotations

import bpy


def get_pose_bone(name: str, context=None):
    """Return the active object's pose bone by name, if available."""
    ctx = context or bpy.context
    try:
        return ctx.object.pose.bones[name]
    except Exception:
        return None


__all__ = ["get_pose_bone"]
