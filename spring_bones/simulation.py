"""Simulation and setup logic for the Spring Bones add-on."""

from __future__ import annotations

import math
import time

import bpy
from mathutils import Matrix, Vector

# Tunables (per-step, not per-second) for force/torque tracking viz only
K_POS = 1.0    # position "spring" weight
C_POS = 0.5    # position "damping" on relative velocity
ALPHA_POS = 0.25  # smoothing for displayed force (0..1)

K_ROT = 1.0    # rotation "spring" weight (radians per step)
C_ROT = 0.5    # rotation "damping" on angular velocity
ALPHA_ROT = 0.25  # smoothing for displayed torque

EPS = 1e-8


def _ema(prev: Vector, cur: Vector, alpha: float) -> Vector:
    return prev.lerp(cur, max(0.0, min(1.0, alpha)))


def _ema_vec(prev: Vector, cur: Vector, alpha: float) -> Vector:
    # clamp for safety
    if alpha < 0.0:
        alpha = 0.0
    elif alpha > 1.0:
        alpha = 1.0
    return prev.lerp(cur, alpha)


def _safe_normalize(vec: Vector) -> Vector:
    length = vec.length
    if length < EPS:
        return Vector((0.0, 0.0, 0.0))
    return vec / length


def track_forces_for_bone(item, armature, pose_bone, emp_head, emp_tail):
    """Track pseudo force/torque for a spring bone for visualization."""
    target = emp_tail.matrix_world.translation
    head = emp_head.matrix_world.translation.copy()

    error = target - head
    v_target = target - Vector(item.prev_target_loc)
    v_head = head - Vector(item.prev_head_loc)

    force_raw = error * K_POS + (v_target - v_head) * C_POS
    force_smooth = _ema(Vector(item.force_smooth), force_raw, ALPHA_POS)

    item.force_raw = force_raw
    item.force_smooth = force_smooth
    item.prev_target_loc = target
    item.prev_head_loc = head

    # Rotation tracking
    current_aim = _safe_normalize(pose_bone.y_axis.copy())
    desired_vec = head - target
    desired_aim = _safe_normalize(desired_vec)

    cross = current_aim.cross(desired_aim)
    sin_term = cross.length
    dot_term = max(-1.0, min(1.0, current_aim.dot(desired_aim)))
    angle = math.atan2(sin_term, dot_term)
    axis = _safe_normalize(cross) if sin_term > EPS else Vector((0.0, 0.0, 0.0))
    theta_vec = axis * angle

    prev_aim = _safe_normalize(Vector(item.prev_aim_vec))
    cross_step = prev_aim.cross(current_aim)
    sin_step = cross_step.length
    dot_step = max(-1.0, min(1.0, prev_aim.dot(current_aim)))
    angle_step = math.atan2(sin_step, dot_step)
    axis_step = _safe_normalize(cross_step) if sin_step > EPS else Vector((0.0, 0.0, 0.0))
    omega_est = axis_step * angle_step

    torque_raw = theta_vec * K_ROT - omega_est * C_ROT
    torque_smooth = _ema(Vector(item.torque_smooth), torque_raw, ALPHA_ROT)

    item.torque_raw = torque_raw
    item.torque_smooth = torque_smooth
    item.prev_aim_vec = current_aim


def lerp_vec(vec_a: Vector, vec_b: Vector, t: float) -> Vector:
    """Legacy linear interpolation used by the original add-on."""
    return vec_a * t + vec_b * (1 - t)


def _current_dt(scene) -> float:
    """Match current timing model: interactive ≈ 0.02s; animation = 1/fps."""
    if getattr(scene, "sb_global_spring", False):
        return 0.02
    fps = max(1, scene.render.fps)
    fps_base = getattr(scene.render, "fps_base", 1.0) or 1.0
    return fps_base / fps


def _current_frame_time(scene) -> float:
    """Return the current frame + sub-frame as a float."""
    frame_final = getattr(scene, "frame_current_final", None)
    if frame_final is not None:
        return float(frame_final)
    return float(scene.frame_current + getattr(scene, "frame_subframe", 0.0))


def _spring_damper_step(
    x: Vector,
    v: Vector,
    x_target: Vector,
    v_target: Vector,
    freq_hz: float,
    zeta: float,
    dt: float,
    external_accel: Vector,
):
    """Semi-implicit Euler step for a classic 2nd-order system."""
    omega = 2.0 * math.pi * max(1e-4, freq_hz)
    dx = x - x_target
    dv = v - v_target
    accel = (-omega * omega) * dx + (-2.0 * zeta * omega) * dv + external_accel
    v_next = v + accel * dt
    x_next = x + v_next * dt
    return x_next, v_next


def spring_bone(scene, depsgraph=None):
    """Advance the simulation for all registered spring bones."""
    if scene is None:
        scene = bpy.context.scene
    elif hasattr(scene, "scene") and scene.scene is not None:
        scene = scene.scene
    if depsgraph is None:
        depsgraph = bpy.context.view_layer.depsgraph
    if scene is None:
        return None

    base_dt = _current_dt(scene)
    dt = base_dt
    if getattr(scene, "sb_global_spring_frame", False):
        frame_time = _current_frame_time(scene)
        last_frame_time = getattr(scene, "sb_last_eval_subframe", -1.0)
        if last_frame_time >= 0.0:
            delta_frames = frame_time - last_frame_time
            if delta_frames == 0.0:
                return None
            if delta_frames < 0.0:
                dt = base_dt
            else:
                dt = base_dt * delta_frames
        scene.sb_last_eval_subframe = frame_time

    for bone in scene.sb_spring_bones:
        armature = bpy.data.objects.get(bone.armature)
        if armature is None:
            continue

        armature_eval = armature.evaluated_get(depsgraph)
        pose_bone = armature.pose.bones.get(bone.name)
        pose_bone_eval = armature_eval.pose.bones.get(bone.name)
        if pose_bone is None or pose_bone_eval is None:
            continue

        if pose_bone.sb_global_influence == 0.0:
            continue

        emp_tail = bpy.data.objects.get(bone.name + '_spring_tail')
        emp_head = bpy.data.objects.get(bone.name + '_spring')
        if emp_tail is None or emp_head is None:
            continue

        if getattr(scene, "sb_debug", False):
            print("[SB] eval?", armature is armature_eval, " (expect False)")

        target_basis = pose_bone_eval.tail if bone.sb_bone_rot else pose_bone_eval.head
        target_world = armature_eval.matrix_world @ target_basis
        target_world_vec = Vector(target_world)

        if getattr(scene, "sb_debug", False):
            print(
                "[SB] tgt_world_len",
                (target_world_vec - Vector(armature_eval.location)).length,
            )

        raw_target = target_world_vec.copy()
        prev_target = Vector(bone.prev_target_loc)

        # Use scene prop if present; otherwise default to the stable old value (0.2)
        target_alpha = getattr(scene, "sb_target_alpha", 0.2)
        target_smooth = _ema_vec(prev_target, raw_target, target_alpha)

        # IMPORTANT: derive v_target from the SMOOTHED series
        v_target = (target_smooth - prev_target) / max(1e-4, dt)
        external_accel = Vector((0.0, 0.0, -pose_bone.sb_gravity))

        freq_hz = getattr(pose_bone, "sb_phys_stiffness", 4.0)
        zeta = getattr(pose_bone, "sb_phys_damping", 0.7)

        x = emp_head.location.copy()
        v = Vector(bone.speed)
        x_next, v_next = _spring_damper_step(
            x=x,
            v=v,
            x_target=target_smooth,    # use smoothed target
            v_target=v_target,         # velocity from smoothed series
            freq_hz=freq_hz,
            zeta=zeta,
            dt=dt,
            external_accel=external_accel,
        )

        # NOTE: write to original ID, evaluated is copy-on-write
        emp_head.location = x_next
        bone.speed = v_next

        emp_head.location = lerp_vec(
            emp_head.location, target_smooth, pose_bone.sb_global_influence
        )

        bone.prev_target_loc = target_smooth

        emp_head.update_tag()
        armature.update_tag()

        track_forces_for_bone(bone, armature_eval, pose_bone_eval, emp_head, emp_tail)

    return None


def update_bone(_self, context):
    """Register the current selection of spring bones and ensure helpers exist."""
    print("Updating data...")
    time_start = time.time()
    scene = context.scene
    armature = context.active_object

    scene.sb_last_eval_subframe = -1.0

    if len(scene.sb_spring_bones) > 0:
        for i in range(len(scene.sb_spring_bones) - 1, -1, -1):
            scene.sb_spring_bones.remove(i)

    for pbone in armature.pose.bones:
        if len(pbone.keys()) == 0:
            continue
        if 'sb_bone_spring' not in pbone.keys():
            continue

        rotation_enabled = bool(pbone.get("sb_bone_rot", False))
        is_spring_bone = bool(pbone.get("sb_bone_spring", False))

        if not is_spring_bone:
            spring_cns = pbone.constraints.get("spring")
            if spring_cns:
                pbone.constraints.remove(spring_cns)
            continue

        bone_tail = armature.matrix_world @ pbone.tail
        bone_head = armature.matrix_world @ pbone.head

        item = scene.sb_spring_bones.add()
        item.name = pbone.name
        print("registering", pbone.name)
        item.last_loc = bone_head
        item.armature = armature.name
        parent_name = pbone.parent.name if pbone.parent else ""

        item.sb_bone_rot = rotation_enabled

        initial_loc = bone_tail if rotation_enabled else bone_head
        item.prev_target_loc = initial_loc
        item.prev_head_loc = initial_loc
        item.prev_aim_vec = (0.0, 1.0, 0.0)
        item.force_raw = (0.0, 0.0, 0.0)
        item.force_smooth = (0.0, 0.0, 0.0)
        item.torque_raw = (0.0, 0.0, 0.0)
        item.torque_smooth = (0.0, 0.0, 0.0)

        empty_radius = 1.0
        head_name = item.name + '_spring'
        tail_name = item.name + '_spring_tail'

        if not bpy.data.objects.get(head_name):
            head_empty = bpy.data.objects.new(head_name, None)
            scene.collection.objects.link(head_empty)
            head_empty.empty_display_size = empty_radius
            head_empty.empty_display_type = 'PLAIN_AXES'
            head_empty.location = bone_tail if rotation_enabled else bone_head
            head_empty.hide_set(True)
            head_empty.hide_select = True

        if not bpy.data.objects.get(tail_name):
            tail_empty = bpy.data.objects.new(tail_name, None)
            scene.collection.objects.link(tail_empty)
            tail_empty.empty_display_size = empty_radius
            tail_empty.empty_display_type = 'PLAIN_AXES'
            tail_empty.matrix_world = Matrix.Translation(
                bone_tail if rotation_enabled else bone_head
            )
            tail_empty.hide_set(True)
            tail_empty.hide_select = True
            matrix = tail_empty.matrix_world.copy()
            tail_empty.parent = armature
            tail_empty.parent_type = 'BONE'
            tail_empty.parent_bone = parent_name
            tail_empty.matrix_world = matrix

        if pbone.get('sb_bone_spring', False):
            spring_cns = pbone.constraints.get("spring")
            if spring_cns:
                pbone.constraints.remove(spring_cns)
            target_obj = bpy.data.objects[head_name]
            if rotation_enabled:
                constraint = pbone.constraints.new('DAMPED_TRACK')
                constraint.target = target_obj
            else:
                constraint = pbone.constraints.new('COPY_LOCATION')
                constraint.target = target_obj
            constraint.name = 'spring'

    print("Updated in", round(time.time() - time_start, 1), "seconds.")


def end_spring_bone(context, _operator):
    """Stop the simulation and remove helper objects."""
    if context.scene.sb_global_spring:
        context.scene.sb_global_spring = False

    context.scene.sb_last_eval_subframe = -1.0

    for item in context.scene.sb_spring_bones:
        active_bone = context.active_object.pose.bones.get(item.name)
        if active_bone is None:
            continue
        constraint = active_bone.constraints.get('spring')
        if constraint:
            active_bone.constraints.remove(constraint)
        emp1 = bpy.data.objects.get(active_bone.name + '_spring')
        emp2 = bpy.data.objects.get(active_bone.name + '_spring_tail')
        if emp1:
            bpy.data.objects.remove(emp1)
        if emp2:
            bpy.data.objects.remove(emp2)

    print("--End--")


__all__ = [
    "spring_bone",
    "update_bone",
    "end_spring_bone",
    "track_forces_for_bone",
    "lerp_vec",
]
