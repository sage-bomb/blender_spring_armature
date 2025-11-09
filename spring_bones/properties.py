"""Property definitions for the Spring Bones add-on."""

from __future__ import annotations

import bpy
from mathutils import Matrix


class SpringBoneSettings(bpy.types.PropertyGroup):
    armature: bpy.props.StringProperty(default="")
    last_loc: bpy.props.FloatVectorProperty(
        name="Loc", subtype='DIRECTION', default=(0, 0, 0), size=3
    )
    speed: bpy.props.FloatVectorProperty(
        name="Speed", subtype='DIRECTION', default=(0, 0, 0), size=3
    )
    dist: bpy.props.FloatProperty(name="distance", default=1.0)
    target_offset: bpy.props.FloatVectorProperty(
        name="TargetLoc", subtype='DIRECTION', default=(0, 0, 0), size=3
    )
    sb_bone_rot: bpy.props.BoolProperty(name="Bone Rot", default=False)
    matrix_offset = Matrix()
    initial_matrix = Matrix()

    prev_target_loc: bpy.props.FloatVectorProperty(
        name="Prev Target", subtype='XYZ', default=(0, 0, 0), size=3
    )
    prev_head_loc: bpy.props.FloatVectorProperty(
        name="Prev Head", subtype='XYZ', default=(0, 0, 0), size=3
    )

    force_raw: bpy.props.FloatVectorProperty(
        name="Force Raw", subtype='XYZ', default=(0, 0, 0), size=3
    )
    force_smooth: bpy.props.FloatVectorProperty(
        name="Force Smooth", subtype='XYZ', default=(0, 0, 0), size=3
    )

    prev_aim_vec: bpy.props.FloatVectorProperty(
        name="Prev Aim", subtype='DIRECTION', default=(0, 1, 0), size=3
    )
    torque_raw: bpy.props.FloatVectorProperty(
        name="Torque Raw", subtype='XYZ', default=(0, 0, 0), size=3
    )
    torque_smooth: bpy.props.FloatVectorProperty(
        name="Torque Smooth", subtype='XYZ', default=(0, 0, 0), size=3
    )


_SCENE_PROPS = {
    "sb_spring_bones": bpy.props.CollectionProperty(type=SpringBoneSettings),
    "sb_global_spring": bpy.props.BoolProperty(name="Enable spring", default=False),
    "sb_global_spring_frame": bpy.props.BoolProperty(
        name="Enable Spring",
        description="Enable Spring on frame change only",
        default=False,
    ),
    "sb_vis_forces": bpy.props.BoolProperty(
        name="Show Forces",
        description="Draw spring forces/torques in the 3D View",
        default=False,
    ),
    "sb_last_eval_subframe": bpy.props.FloatProperty(
        name="Last Evaluated Sub-frame",
        description="Internal: last sub-frame that evaluated the animation mode",
        default=-1.0,
        options={'HIDDEN'},
    ),
    "sb_force_scale": bpy.props.FloatProperty(
        name="Force Scale",
        description="Scale for positional force debug lines",
        default=0.1,
        min=0.0,
        soft_max=10.0,
    ),
    "sb_torque_scale": bpy.props.FloatProperty(
        name="Torque Scale",
        description="Scale for rotational torque debug lines",
        default=0.1,
        min=0.0,
        soft_max=10.0,
    ),
    "sb_use_physics": bpy.props.BoolProperty(
        name="Physics Mode",
        description="Use spring–damper physics update instead of legacy integrator",
        default=False,
    ),
    "sb_target_alpha": bpy.props.FloatProperty(
        name="Target Smooth",
        description="EMA smoothing applied to the target before physics",
        default=0.2,
        min=0.0,
        max=1.0,
    ),
}

_POSE_BONE_PROPS = {
    "sb_bone_spring": bpy.props.BoolProperty(
        name="Enabled",
        default=False,
        description="Enable spring effect on this bone",
    ),
    "sb_stiffness": bpy.props.FloatProperty(
        name="Stiffness",
        default=0.5,
        min=0.01,
        max=1.0,
        description="Bouncy/elasticity value, higher values lead to more bounciness",
    ),
    "sb_damp": bpy.props.FloatProperty(
        name="Damp",
        default=0.7,
        min=0.0,
        max=10.0,
        description="Speed/damping force applied to the bone to go back to its initial position",
    ),
    "sb_phys_freq": bpy.props.FloatProperty(
        name="Physics Freq (Hz)",
        description="Natural frequency of the bone's spring",
        default=4.0,
        min=0.01,
        soft_max=20.0,
    ),
    "sb_phys_zeta": bpy.props.FloatProperty(
        name="Physics Damp",
        description="0 = none, 1 = critical, >1 = over-damped",
        default=0.7,
        min=0.0,
        soft_max=2.0,
    ),
    "sb_gravity": bpy.props.FloatProperty(
        name="Gravity",
        description="Additional vertical force to simulate gravity",
        default=0.0,
        min=-100.0,
        max=100.0,
    ),
    "sb_bone_rot": bpy.props.BoolProperty(
        name="Rotation",
        default=False,
        description="The spring effect will apply on the bone rotation instead of location",
    ),
    "sb_global_influence": bpy.props.FloatProperty(
        name="Influence",
        default=1.0,
        min=0.0,
        max=1.0,
        description="Global influence of spring motion",
    ),
}


def register():
    from bpy.utils import register_class

    register_class(SpringBoneSettings)

    for name, prop in _SCENE_PROPS.items():
        setattr(bpy.types.Scene, name, prop)

    for name, prop in _POSE_BONE_PROPS.items():
        setattr(bpy.types.PoseBone, name, prop)


def unregister():
    from bpy.utils import unregister_class

    for name in _SCENE_PROPS.keys():
        delattr(bpy.types.Scene, name)

    for name in _POSE_BONE_PROPS.keys():
        delattr(bpy.types.PoseBone, name)

    unregister_class(SpringBoneSettings)


__all__ = ["SpringBoneSettings"]
