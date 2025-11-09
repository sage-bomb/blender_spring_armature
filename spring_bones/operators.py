"""Operator definitions for the Spring Bones add-on."""

from __future__ import annotations

import bpy

from .draw import diagnose_force_viz, toggle_force_viz
from .simulation import end_spring_bone, spring_bone, update_bone
from .utils import get_pose_bone


class SB_OT_spring_modal(bpy.types.Operator):
    """Spring Bones, interactive mode"""

    bl_idname = "sb.spring_bone"
    bl_label = "spring_bone"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return context is not None and context.window is not None and context.scene is not None

    def invoke(self, context, _event):
        wm = context.window_manager
        if not getattr(self, "timer_handler", None):
            self.timer_handler = wm.event_timer_add(0.02, window=context.window)
        wm.modal_handler_add(self)
        context.scene.sb_global_spring = True
        update_bone(self, context)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type == "ESC" or context.scene.sb_global_spring is False:
            self.cancel(context)
            return {'FINISHED'}

        if event.type == 'TIMER':
            spring_bone(context)
            return {'PASS_THROUGH'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        if context.scene.sb_global_spring is False:
            return self.invoke(context, None)
        self.cancel(context)
        return {'FINISHED'}

    def cancel(self, context):
        wm = context.window_manager
        if getattr(self, "timer_handler", None):
            wm.event_timer_remove(self.timer_handler)
            self.timer_handler = None

        context.scene.sb_global_spring = False

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


class SB_OT_spring(bpy.types.Operator):
    """Spring Bones, animation mode. Support baking."""

    bl_idname = "sb.spring_bone_frame"
    bl_label = "spring_bone_frame"

    def execute(self, context):
        if context.scene.sb_global_spring_frame is False:
            context.scene.sb_global_spring_frame = True
            update_bone(self, context)
        else:
            end_spring_bone(context, self)
            context.scene.sb_global_spring_frame = False
        return {'FINISHED'}


class SB_OT_select_bone(bpy.types.Operator):
    """Select this bone"""

    bl_idname = "sb.select_bone"
    bl_label = "select_bone"

    bone_name: bpy.props.StringProperty(default="")

    def execute(self, context):
        pose_bone = get_pose_bone(self.bone_name, context)
        if pose_bone is None:
            return {'CANCELLED'}
        data_bone = pose_bone.bone
        context.active_object.data.bones.active = data_bone
        data_bone.select = True
        for index, layer_enabled in enumerate(data_bone.layers):
            if layer_enabled and context.active_object.data.layers[index] is False:
                context.active_object.data.layers[index] = True
        return {'FINISHED'}


class SB_OT_spring_stop(bpy.types.Operator):
    """Stop Spring Bones interactive mode"""

    bl_idname = "sb.spring_bone_stop"
    bl_label = "spring_bone_stop"

    def execute(self, context):
        context.scene.sb_global_spring = False
        return {'FINISHED'}


class SB_OT_toggle_force_viz(bpy.types.Operator):
    """Toggle drawing of spring forces/torques in the viewport"""

    bl_idname = "sb.toggle_force_viz"
    bl_label = "Toggle Force Viz"

    def execute(self, context):
        toggle_force_viz(context)
        return {'FINISHED'}


class SB_OT_diag_force_viz(bpy.types.Operator):
    """Print diagnostic info for Spring Bones force viz"""

    bl_idname = "sb.diag_force_viz"
    bl_label = "Diagnose Force Viz"

    def execute(self, context):
        diagnose_force_viz(context.scene)
        return {'FINISHED'}


_CLASSES = (
    SB_OT_spring_modal,
    SB_OT_spring,
    SB_OT_select_bone,
    SB_OT_spring_stop,
    SB_OT_toggle_force_viz,
    SB_OT_diag_force_viz,
)


def register():
    from bpy.utils import register_class

    for cls in _CLASSES:
        register_class(cls)


def unregister():
    from bpy.utils import unregister_class

    for cls in reversed(_CLASSES):
        unregister_class(cls)


__all__ = [cls.__name__ for cls in _CLASSES]
