"""Viewport drawing helpers for Spring Bones."""

from __future__ import annotations

import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from mathutils import Vector

_draw_handle = None


def _collect_force_segments(scene):
    """Collect world-space line segments for force/torque visualization."""
    pos_segments = []
    rot_segments = []
    ref_segments = []

    for item in scene.sb_spring_bones:
        head = bpy.data.objects.get(item.name + '_spring')
        tail = bpy.data.objects.get(item.name + '_spring_tail')
        if head is None or tail is None:
            continue

        head_pos = head.matrix_world.translation.copy()
        tail_pos = tail.matrix_world.translation.copy()

        ref_segments.append((head_pos, tail_pos))

        force = Vector(item.force_smooth)
        if force.length > 0.0:
            end = head_pos + force * scene.sb_force_scale
            pos_segments.append((head_pos, end))

        torque = Vector(item.torque_smooth)
        if torque.length > 0.0:
            end = head_pos + torque * scene.sb_torque_scale
            rot_segments.append((head_pos, end))

    return pos_segments, rot_segments, ref_segments


def _draw_forces_callback():
    scene = bpy.context.scene
    if not scene or not getattr(scene, "sb_vis_forces", False):
        return

    pos_segments, rot_segments, ref_segments = _collect_force_segments(scene)
    if not pos_segments and not rot_segments and not ref_segments:
        return

    shader = gpu.shader.from_builtin('UNIFORM_COLOR')

    try:
        gpu.state.line_width_set(2.0)
    except Exception:
        pass

    if ref_segments:
        verts = []
        for start, end in ref_segments:
            verts.extend([start, end])
        batch = batch_for_shader(shader, 'LINES', {"pos": verts})
        shader.bind()
        shader.uniform_float("color", (0.2, 1.0, 1.0, 1.0))
        batch.draw(shader)

    if pos_segments:
        verts = []
        for start, end in pos_segments:
            verts.extend([start, end])
        batch = batch_for_shader(shader, 'LINES', {"pos": verts})
        shader.bind()
        shader.uniform_float("color", (1.0, 0.2, 0.2, 1.0))
        batch.draw(shader)

    if rot_segments:
        verts = []
        for start, end in rot_segments:
            verts.extend([start, end])
        batch = batch_for_shader(shader, 'LINES', {"pos": verts})
        shader.bind()
        shader.uniform_float("color", (0.2, 0.4, 1.0, 1.0))
        batch.draw(shader)

    try:
        gpu.state.line_width_set(1.0)
    except Exception:
        pass


def toggle_force_viz(context) -> bool:
    """Toggle the viewport force visualization handler."""
    global _draw_handle
    scene = context.scene

    scene.sb_vis_forces = not scene.sb_vis_forces

    if scene.sb_vis_forces:
        if _draw_handle is None:
            _draw_handle = bpy.types.SpaceView3D.draw_handler_add(
                _draw_forces_callback, (), 'WINDOW', 'POST_VIEW'
            )
    else:
        if _draw_handle is not None:
            bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, 'WINDOW')
            _draw_handle = None

    for window in context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()

    return scene.sb_vis_forces


def diagnose_force_viz(scene):
    """Print diagnostic information about the current visualization state."""
    bones = list(scene.sb_spring_bones)
    print(f"[SB] sb_vis_forces={scene.sb_vis_forces}, bones={len(bones)}")
    any_head = any_tail = any_force = any_torque = False
    for item in bones:
        head = bpy.data.objects.get(item.name + '_spring')
        tail = bpy.data.objects.get(item.name + '_spring_tail')
        force_len = Vector(item.force_smooth).length
        torque_len = Vector(item.torque_smooth).length
        print(
            f"  - {item.name}: head={'ok' if head else 'MISSING'} "
            f"tail={'ok' if tail else 'MISSING'} "
            f"|F|={force_len:.4f} |T|={torque_len:.4f}"
        )
        any_head |= bool(head)
        any_tail |= bool(tail)
        any_force |= force_len > 0.0
        any_torque |= torque_len > 0.0
    if not bones:
        print("  (!!) No items in scene.sb_spring_bones. Start Interactive/Animation mode to populate.")
    if bones and not any_head:
        print("  (!!) No _spring empties found.")
    if bones and not any_tail:
        print("  (!!) No _spring_tail empties found.")
    if bones and (not any_force and not any_torque):
        print("  (i) Forces/torques are zero—move the rig and/or raise scales.")


def remove_force_viz_handler():
    """Ensure the draw handler is removed (used during unregister)."""
    global _draw_handle
    if _draw_handle is not None:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, 'WINDOW')
        except Exception:
            pass
        _draw_handle = None


__all__ = [
    "toggle_force_viz",
    "diagnose_force_viz",
    "remove_force_viz_handler",
]
