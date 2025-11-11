"""UI panel for the Spring Bones add-on."""

from __future__ import annotations

import bpy


class SB_PT_ui(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Spring Bones'
    bl_label = "Spring Bones"

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        if context.mode == "POSE" and bpy.context.active_pose_bone:
            active_bone = bpy.context.active_pose_bone

            col = layout.column(align=True)
            if context.scene.sb_global_spring is False:
                col.operator("sb.spring_bone", text="Start - Interactive Mode", icon='PLAY')
            else:
                col.operator("sb.spring_bone_stop", text="Stop", icon='PAUSE')
            col.enabled = not context.scene.sb_global_spring_frame

            col = layout.column(align=True)
            if context.scene.sb_global_spring_frame is False:
                col.operator("sb.spring_bone_frame", text="Start - Animation Mode", icon='PLAY')
            else:
                col.operator("sb.spring_bone_frame", text="Stop", icon='PAUSE')
            col.enabled = not context.scene.sb_global_spring

            col = layout.column(align=True)
            col.label(text='Bone Parameters:')
            col.prop(active_bone, 'sb_bone_spring', text="Spring")
            col.prop(active_bone, 'sb_bone_rot', text="Rotation")
            col.prop(active_bone, 'sb_stiffness', text="Bouncy")
            col.prop(active_bone, 'sb_damp', text="Speed")
            col.prop(active_bone, 'sb_gravity', text="Gravity")
            col.prop(active_bone, 'sb_global_influence', text="Influence")

            layout.separator()
            box = layout.box()
            box.label(text="Force Debug Draw:")
            row = box.row(align=True)
            row.operator(
                "sb.toggle_force_viz",
                text=("Hide" if scene.sb_vis_forces else "Show"),
                icon=('HIDE_OFF' if scene.sb_vis_forces else 'HIDE_ON'),
            )
            row = box.row(align=True)
            row.prop(scene, "sb_force_scale")
            row.prop(scene, "sb_torque_scale")
            row = box.row(align=True)
            row.operator("sb.diag_force_viz", text="Diagnose", icon='INFO')

            layout.separator()
            box = layout.box()
            box.label(text="Physics Mode:")
            row = box.row(align=True)
            row.prop(scene, "sb_use_physics", text="Enable")
            col = box.column(align=True)
            col.label(text="Interactive Mode:")
            row = col.row(align=True)
            row.prop(scene, "sb_phys_freq_interactive")
            row.prop(scene, "sb_phys_zeta_interactive")
            col = box.column(align=True)
            col.label(text="Animation Mode:")
            row = col.row(align=True)
            row.prop(scene, "sb_phys_freq_animation")
            row.prop(scene, "sb_phys_zeta_animation")
            row = box.row(align=True)
            row.prop(scene, "sb_target_alpha")


def register():
    from bpy.utils import register_class

    register_class(SB_PT_ui)


def unregister():
    from bpy.utils import unregister_class

    unregister_class(SB_PT_ui)


__all__ = ["SB_PT_ui"]
