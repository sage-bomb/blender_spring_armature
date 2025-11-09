bl_info = {
    "name": "Spring Bones",
    "author": "sage-bomb",
    "version": (0, 9),
    "blender": (4, 4, 0),
    "location": "3D Viewport > Sidebar (N) > Spring Bones",
    "description": "Add a spring dynamic effect to a single/multiple bones",
    "category": "Animation"
}

import bpy, time, math
from bpy.app.handlers import persistent
from mathutils import *
from mathutils import Vector

# --- GPU drawing for force viz ---
import gpu
from gpu_extras.batch import batch_for_shader

# ------------- helpers ----------------

def set_active_object(object_name):
    bpy.context.view_layer.objects.active = bpy.data.objects[object_name]
    bpy.data.objects[object_name].select_set(state=1)

def get_pose_bone(name):
    try:
        return bpy.context.object.pose.bones[name]
    except:
        return None

# Tunables (per-step, not per-second) for force/torque tracking viz only
K_POS = 1.0    # position “spring” weight
C_POS = 0.5    # position “damping” on relative velocity
ALPHA_POS = 0.25  # smoothing for displayed force (0..1)

K_ROT = 1.0    # rotation “spring” weight (radians per step)
C_ROT = 0.5    # rotation “damping” on angular velocity
ALPHA_ROT = 0.25  # smoothing for displayed torque

EPS = 1e-8

def _ema(prev: Vector, cur: Vector, alpha: float) -> Vector:
    return prev.lerp(cur, max(0.0, min(1.0, alpha)))

def _safe_normalize(v: Vector) -> Vector:
    l = v.length
    if l < EPS:
        return Vector((0.0, 0.0, 0.0))
    return v / l

def track_forces_for_bone(item, armature, pose_bone, emp_head, emp_tail):
    # --- Position tracking ---
    target = emp_tail.matrix_world.translation
    head   = emp_head.matrix_world.translation.copy()  # world-safe

    error = target - head
    v_target = target - Vector(item.prev_target_loc)
    v_head   = head   - Vector(item.prev_head_loc)

    F_raw = error * K_POS + (v_target - v_head) * C_POS
    F_s   = _ema(Vector(item.force_smooth), F_raw, ALPHA_POS)

    item.force_raw    = F_raw
    item.force_smooth = F_s
    item.prev_target_loc = target
    item.prev_head_loc   = head

    # --- Rotation tracking ---
    a_now = _safe_normalize(pose_bone.y_axis.copy())  # current aim (world)
    des_vec = head - target                           # desired aim (world)
    a_des = _safe_normalize(des_vec)

    c = a_now.cross(a_des)
    s = c.length
    d = max(-1.0, min(1.0, a_now.dot(a_des)))
    angle = math.atan2(s, d)  # 0..pi
    axis  = _safe_normalize(c) if s > EPS else Vector((0.0, 0.0, 0.0))
    theta_vec = axis * angle   # rotational error vector

    prev_aim = _safe_normalize(Vector(item.prev_aim_vec))
    c2 = prev_aim.cross(a_now)
    s2 = c2.length
    d2 = max(-1.0, min(1.0, prev_aim.dot(a_now)))
    angle_step = math.atan2(s2, d2)
    axis_step  = _safe_normalize(c2) if s2 > EPS else Vector((0.0, 0.0, 0.0))
    omega_est  = axis_step * angle_step  # approx angular vel per step

    tau_raw = theta_vec * K_ROT - omega_est * C_ROT
    tau_s   = _ema(Vector(item.torque_smooth), tau_raw, ALPHA_ROT)

    item.torque_raw    = tau_raw
    item.torque_smooth = tau_s
    item.prev_aim_vec  = a_now

@persistent
def spring_bone_frame_mode(foo):
    if bpy.context.scene.sb_global_spring_frame == True:
        spring_bone(foo)

def lerp_vec(vec_a, vec_b, t):
    # NOTE: legacy "inverted" lerp kept for compatibility with existing rigs.
    return vec_a * t + vec_b * (1 - t)

# -------------------- PHYSICS HELPERS --------------------

def _current_dt(scene) -> float:
    """Match current timing model: interactive ≈ 0.02s; animation = 1/fps."""
    if getattr(scene, "sb_global_spring", False):
        return 0.02
    fps = max(1, scene.render.fps)
    return 1.0 / fps

def _ema_vec(prev: Vector, cur: Vector, alpha: float) -> Vector:
    return prev.lerp(cur, max(0.0, min(1.0, alpha)))

def _spring_damper_step(x: Vector, v: Vector, x_target: Vector, v_target: Vector,
                        freq_hz: float, zeta: float, dt: float,
                        external_accel: Vector):
    """
    Semi-implicit Euler for classic 2nd-order system:
      a = -ω^2 (x - x*) - 2ζω (v - v*) + external
    where ω = 2π f

    Returns (x_next, v_next)
    """
    omega = 2.0 * math.pi * max(1e-4, freq_hz)
    dx = x - x_target
    dv = v - v_target
    a = (-omega * omega) * dx + (-2.0 * zeta * omega) * dv + external_accel
    v_next = v + a * dt
    x_next = x + v_next * dt
    return x_next, v_next

# -------------------- MAIN STEP --------------------

def spring_bone(foo):
    scene = bpy.context.scene
    use_physics = getattr(scene, "sb_use_physics", False)
    dt = _current_dt(scene)

    # cache knobs
    target_alpha = getattr(scene, "sb_target_alpha", 0.2)
    freq_hz = getattr(scene, "sb_phys_freq", 4.0)
    zeta = getattr(scene, "sb_phys_zeta", 0.7)

    for bone in scene.sb_spring_bones:
        armature = bpy.data.objects[bone.armature]
        pose_bone = armature.pose.bones.get(bone.name)
        if pose_bone is None:
            continue

        # skip if influence is zero
        if pose_bone.sb_global_influence == 0.0:
            continue

        emp_tail = bpy.data.objects.get(bone.name + '_spring_tail')
        emp_head = bpy.data.objects.get(bone.name + '_spring')
        if emp_tail is None or emp_head is None:
            continue

        # world position of tail empty
        emp_tail_loc, rot, scale = emp_tail.matrix_world.decompose()

        if not use_physics:
            # -------------------------
            # LEGACY (unchanged)
            # -------------------------
            base_pos_dir = Vector((0, 0, -pose_bone.sb_gravity))
            base_pos_dir += (emp_tail_loc - emp_head.location)
            bone.speed += base_pos_dir * pose_bone.sb_stiffness
            bone.speed *= pose_bone.sb_damp
            emp_head.location += bone.speed
            emp_head.location = lerp_vec(emp_head.location, emp_tail_loc, pose_bone.sb_global_influence)

        else:
            # -------------------------
            # PHYSICS (spring–damper)
            # -------------------------

            # 1) Smooth the target to preserve the "soft input" feel
            raw_target = Vector(emp_tail_loc)
            prev_target = Vector(bone.prev_target_loc)
            target_smooth = _ema_vec(prev_target, raw_target, target_alpha)

            # 2) Estimate target velocity (for relative damping term)
            v_target = (target_smooth - prev_target) / max(1e-4, dt)

            # 3) External acceleration (gravity)
            a_ext = Vector((0.0, 0.0, -pose_bone.sb_gravity))

            # 4) Integrate spring–damper against smoothed target
            x = emp_head.location.copy()
            v = Vector(bone.speed)
            x_next, v_next = _spring_damper_step(
                x=x, v=v,
                x_target=target_smooth, v_target=v_target,
                freq_hz=freq_hz, zeta=zeta, dt=dt,
                external_accel=a_ext
            )

            # 5) Apply
            emp_head.location = x_next
            bone.speed = v_next

            # 6) Optional global blend (parity with legacy "Influence")
            emp_head.location = lerp_vec(emp_head.location, target_smooth, pose_bone.sb_global_influence)

            # 7) Keep tracker in sync with the smoothed target
            bone.prev_target_loc = target_smooth

        # ------------------------------------------------------------
        # TRACKING: record pseudo-force and pseudo-torque (for viz)
        # ------------------------------------------------------------
        track_forces_for_bone(bone, armature, pose_bone, emp_head, emp_tail)

    return None

def update_bone(self, context):
    print("Updating data...")
    time_start = time.time()
    scene = bpy.context.scene
    armature = bpy.context.active_object

    # remove items safely (descending indices)
    if len(scene.sb_spring_bones) > 0:
        for i in range(len(scene.sb_spring_bones) - 1, -1, -1):
            scene.sb_spring_bones.remove(i)

    for pbone in armature.pose.bones:
        # Skip bones without any custom props at all
        if len(pbone.keys()) == 0:
            continue
        # We only care about spring bones now
        if 'sb_bone_spring' not in pbone.keys():
            continue

        rotation_enabled = bool(pbone.get("sb_bone_rot", False))
        is_spring_bone = bool(pbone.get("sb_bone_spring", False))

        # If the spring flag is off, make sure the old constraint is gone and skip
        if not is_spring_bone:
            spring_cns = pbone.constraints.get("spring")
            if spring_cns:
                pbone.constraints.remove(spring_cns)
            continue

        # -------- from here on: spring bone only --------

        # World-space head/tail for this pose bone
        bone_tail = armature.matrix_world @ pbone.tail
        bone_head = armature.matrix_world @ pbone.head

        # Register a new per-bone item
        item = scene.sb_spring_bones.add()
        item.name = pbone.name
        print("registering", pbone.name)
        item.last_loc = bone_head
        item.armature = armature.name
        parent_name = pbone.parent.name if pbone.parent else ""

        item.sb_bone_rot = rotation_enabled

        # ---- initialize tracking fields (per-bone) ----
        initial_loc = bone_tail if rotation_enabled else bone_head
        item.prev_target_loc = initial_loc
        item.prev_head_loc   = initial_loc
        item.prev_aim_vec    = (0.0, 1.0, 0.0)
        item.force_raw       = (0.0, 0.0, 0.0)
        item.force_smooth    = (0.0, 0.0, 0.0)
        item.torque_raw      = (0.0, 0.0, 0.0)
        item.torque_smooth   = (0.0, 0.0, 0.0)

        # ---- create empty helpers if missing ----
        empty_radius = 1.0

        head_name = item.name + '_spring'
        tail_name = item.name + '_spring_tail'

        if not bpy.data.objects.get(head_name):
            o = bpy.data.objects.new(head_name, None)
            bpy.context.scene.collection.objects.link(o)
            o.empty_display_size = empty_radius
            o.empty_display_type = 'PLAIN_AXES'
            o.location = bone_tail if rotation_enabled else bone_head
            o.hide_set(True)
            o.hide_select = True

        if not bpy.data.objects.get(tail_name):
            empty = bpy.data.objects.new(tail_name, None)
            bpy.context.scene.collection.objects.link(empty)
            empty.empty_display_size = empty_radius
            empty.empty_display_type = 'PLAIN_AXES'
            empty.matrix_world = Matrix.Translation(bone_tail if rotation_enabled else bone_head)
            empty.hide_set(True)
            empty.hide_select = True
            # parent to the armature's parent bone to follow the rig
            mat = empty.matrix_world.copy()
            empty.parent = armature
            empty.parent_type = 'BONE'
            empty.parent_bone = parent_name
            empty.matrix_world = mat

        # ---- create/refresh constraint ----
        if pbone.get('sb_bone_spring', False):
            spring_cns = pbone.constraints.get("spring")
            if spring_cns:
                pbone.constraints.remove(spring_cns)
            target_obj = bpy.data.objects[head_name]
            if rotation_enabled:
                cns = pbone.constraints.new('DAMPED_TRACK')
                cns.target = target_obj
            else:
                cns = pbone.constraints.new('COPY_LOCATION')
                cns.target = target_obj
            cns.name = 'spring'

    print("Updated in", round(time.time() - time_start, 1), "seconds.")

def end_spring_bone(context, self):
    if context.scene.sb_global_spring:
        wm = context.window_manager
        # timer is removed in cancel(); here we just flip the flag
        context.scene.sb_global_spring = False

    for item in context.scene.sb_spring_bones:
        active_bone = bpy.context.active_object.pose.bones.get(item.name)
        if active_bone == None:
            continue
        cns = active_bone.constraints.get('spring')
        if cns:
            active_bone.constraints.remove(cns)
        emp1 = bpy.data.objects.get(active_bone.name + '_spring')
        emp2 = bpy.data.objects.get(active_bone.name + '_spring_tail')
        if emp1:
            bpy.data.objects.remove(emp1)
        if emp2:
            bpy.data.objects.remove(emp2)

    print("--End--")

class SB_OT_spring_modal(bpy.types.Operator):
    """Spring Bones, interactive mode"""
    bl_idname = "sb.spring_bone"
    bl_label = "spring_bone"
    bl_options = {'REGISTER'}  # UNDO not necessary for modal toggle

    @classmethod
    def poll(cls, context):
        return context is not None and context.window is not None and context.scene is not None

    def invoke(self, context, event):
        wm = context.window_manager
        # Avoid duplicate timers
        if not getattr(self, "timer_handler", None):
            self.timer_handler = wm.event_timer_add(0.02, window=context.window)
        wm.modal_handler_add(self)
        context.scene.sb_global_spring = True
        update_bone(self, context)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        # Stop via ESC or property flip
        if event.type == "ESC" or context.scene.sb_global_spring is False:
            self.cancel(context)
            return {'FINISHED'}

        if event.type == 'TIMER':
            spring_bone(context)
            # Let other UI events through so buttons/menus keep working
            return {'PASS_THROUGH'}

        # Do not consume non-timer events
        return {'PASS_THROUGH'}

    def execute(self, context):
        # Toggle behavior: If off -> start (invoke), if on -> stop
        if context.scene.sb_global_spring is False:
            return self.invoke(context, None)
        else:
            self.cancel(context)
            return {'FINISHED'}

    def cancel(self, context):
        wm = context.window_manager
        if getattr(self, "timer_handler", None):
            wm.event_timer_remove(self.timer_handler)
            self.timer_handler = None

        context.scene.sb_global_spring = False

        for item in context.scene.sb_spring_bones:
            active_bone = bpy.context.active_object.pose.bones.get(item.name)
            if active_bone is None:
                continue
            cns = active_bone.constraints.get('spring')
            if cns:
                active_bone.constraints.remove(cns)
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
        if context.scene.sb_global_spring_frame == False:
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
        data_bone = get_pose_bone(self.bone_name).bone
        bpy.context.active_object.data.bones.active = data_bone
        data_bone.select = True
        for i, l in enumerate(data_bone.layers):
            if l == True and bpy.context.active_object.data.layers[i] == False:
                bpy.context.active_object.data.layers[i] = True
        return {'FINISHED'}

# NEW: tiny dedicated Stop operator that only flips the flag
class SB_OT_spring_stop(bpy.types.Operator):
    """Stop Spring Bones interactive mode"""
    bl_idname = "sb.spring_bone_stop"
    bl_label = "spring_bone_stop"

    def execute(self, context):
        context.scene.sb_global_spring = False
        return {'FINISHED'}

# ----------------- FORCE VIZ -----------------

_draw_handle = None

def _collect_force_segments(scene):
    """Collect world-space line segments for forces and torques and a reference head->tail ray."""
    pos_segs = []   # (start, end) positional force (red)
    rot_segs = []   # (start, end) torque vector (blue)
    ref_segs = []   # (start, end) reference head->tail (cyan)

    for item in scene.sb_spring_bones:
        head = bpy.data.objects.get(item.name + '_spring')
        tail = bpy.data.objects.get(item.name + '_spring_tail')
        if head is None or tail is None:
            continue

        head_pos = head.matrix_world.translation.copy()
        tail_pos = tail.matrix_world.translation.copy()

        # Always draw a reference line so we know the handler is active
        ref_segs.append((head_pos, tail_pos))

        # positional force
        f = Vector(item.force_smooth)
        if f.length > 0.0:
            end = head_pos + f * scene.sb_force_scale
            pos_segs.append((head_pos, end))

        # rotational torque
        t = Vector(item.torque_smooth)
        if t.length > 0.0:
            end = head_pos + t * scene.sb_torque_scale
            rot_segs.append((head_pos, end))

    return pos_segs, rot_segs, ref_segs

def _draw_forces_callback():
    scene = bpy.context.scene
    if not scene or not getattr(scene, "sb_vis_forces", False):
        return

    pos_segs, rot_segs, ref_segs = _collect_force_segments(scene)
    if not pos_segs and not rot_segs and not ref_segs:
        return

    # Blender 4.x builtin shader name
    shader = gpu.shader.from_builtin('UNIFORM_COLOR')

    try:
        gpu.state.line_width_set(2.0)
    except:
        pass

    # Reference (cyan): head -> tail
    if ref_segs:
        verts = []
        for s, e in ref_segs:
            verts.extend([s, e])
        batch = batch_for_shader(shader, 'LINES', {"pos": verts})
        shader.bind()
        shader.uniform_float("color", (0.2, 1.0, 1.0, 1.0))
        batch.draw(shader)

    # Positional force (red)
    if pos_segs:
        verts = []
        for s, e in pos_segs:
            verts.extend([s, e])
        batch = batch_for_shader(shader, 'LINES', {"pos": verts})
        shader.bind()
        shader.uniform_float("color", (1.0, 0.2, 0.2, 1.0))
        batch.draw(shader)

    # Torque (blue)
    if rot_segs:
        verts = []
        for s, e in rot_segs:
            verts.extend([s, e])
        batch = batch_for_shader(shader, 'LINES', {"pos": verts})
        shader.bind()
        shader.uniform_float("color", (0.2, 0.4, 1.0, 1.0))
        batch.draw(shader)

    try:
        gpu.state.line_width_set(1.0)
    except:
        pass

class SB_OT_toggle_force_viz(bpy.types.Operator):
    """Toggle drawing of spring forces/torques in the viewport"""
    bl_idname = "sb.toggle_force_viz"
    bl_label = "Toggle Force Viz"

    def execute(self, context):
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

        # tag all 3D views for redraw
        for w in context.window_manager.windows:
            for area in w.screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()

        return {'FINISHED'}

class SB_OT_diag_force_viz(bpy.types.Operator):
    """Print diagnostic info for Spring Bones force viz"""
    bl_idname = "sb.diag_force_viz"
    bl_label = "Diagnose Force Viz"

    def execute(self, context):
        scene = context.scene
        bones = list(scene.sb_spring_bones)
        print(f"[SB] sb_vis_forces={scene.sb_vis_forces}, bones={len(bones)}")
        any_head = any_tail = any_force = any_torque = False
        for item in bones:
            head = bpy.data.objects.get(item.name + '_spring')
            tail = bpy.data.objects.get(item.name + '_spring_tail')
            fl = Vector(item.force_smooth).length
            tl = Vector(item.torque_smooth).length
            print(f"  - {item.name}: head={'ok' if head else 'MISSING'} "
                  f"tail={'ok' if tail else 'MISSING'} "
                  f"|F|={fl:.4f} |T|={tl:.4f}")
            any_head |= bool(head)
            any_tail |= bool(tail)
            any_force |= fl > 0.0
            any_torque |= tl > 0.0
        if not bones:
            print("  (!!) No items in scene.sb_spring_bones. Start Interactive/Animation mode to populate.")
        if bones and not any_head:
            print("  (!!) No _spring empties found.")
        if bones and not any_tail:
            print("  (!!) No _spring_tail empties found.")
        if bones and (not any_force and not any_torque):
            print("  (i) Forces/torques are zero—move the rig and/or raise scales.")
        return {'FINISHED'}

###########  UI PANEL  ###################

class SB_PT_ui(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Spring Bones'
    bl_label = "Spring Bones"

    @classmethod
    def poll(cls, context):
        return context.active_object

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        if context.mode == "POSE" and bpy.context.active_pose_bone:
            active_bone = bpy.context.active_pose_bone

            # Interactive mode buttons (use dedicated stop operator)
            col = layout.column(align=True)
            if context.scene.sb_global_spring is False:
                col.operator(SB_OT_spring_modal.bl_idname, text="Start - Interactive Mode", icon='PLAY')
            else:
                col.operator("sb.spring_bone_stop", text="Stop", icon='PAUSE')
            col.enabled = not context.scene.sb_global_spring_frame

            # Animation mode buttons
            col = layout.column(align=True)
            if context.scene.sb_global_spring_frame == False:
                col.operator(SB_OT_spring.bl_idname, text="Start - Animation Mode", icon='PLAY')
            else:
                col.operator(SB_OT_spring.bl_idname, text="Stop", icon='PAUSE')
            col.enabled = not context.scene.sb_global_spring

            # Bone params
            col = layout.column(align=True)
            col.label(text='Bone Parameters:')
            col.prop(active_bone, 'sb_bone_spring', text="Spring")
            col.prop(active_bone, 'sb_bone_rot', text="Rotation")
            col.prop(active_bone, 'sb_stiffness', text="Bouncy")
            col.prop(active_bone, 'sb_damp', text="Speed")
            col.prop(active_bone, 'sb_gravity', text="Gravity")
            col.prop(active_bone, 'sb_global_influence', text="Influence")

            # Force Viz UI
            layout.separator()
            box = layout.box()
            box.label(text="Force Debug Draw:")
            row = box.row(align=True)
            row.operator("sb.toggle_force_viz",
                         text=("Hide" if scene.sb_vis_forces else "Show"),
                         icon=('HIDE_OFF' if scene.sb_vis_forces else 'HIDE_ON'))
            row = box.row(align=True)
            row.prop(scene, "sb_force_scale")
            row.prop(scene, "sb_torque_scale")
            row = box.row(align=True)
            row.operator("sb.diag_force_viz", text="Diagnose", icon='INFO')

            # Physics Mode UI
            layout.separator()
            box = layout.box()
            box.label(text="Physics Mode:")
            row = box.row(align=True)
            row.prop(scene, "sb_use_physics", text="Enable")
            row = box.row(align=True)
            row.prop(scene, "sb_phys_freq")
            row.prop(scene, "sb_phys_zeta")
            row = box.row(align=True)
            row.prop(scene, "sb_target_alpha")

#### REGISTER #############

class bones_collec(bpy.types.PropertyGroup):
    armature: bpy.props.StringProperty(default="")
    last_loc: bpy.props.FloatVectorProperty(name="Loc", subtype='DIRECTION', default=(0, 0, 0), size=3)
    speed: bpy.props.FloatVectorProperty(name="Speed", subtype='DIRECTION', default=(0, 0, 0), size=3)
    dist: bpy.props.FloatProperty(name="distance", default=1.0)
    target_offset: bpy.props.FloatVectorProperty(name="TargetLoc", subtype='DIRECTION', default=(0, 0, 0), size=3)
    sb_bone_rot: bpy.props.BoolProperty(name="Bone Rot", default=False)
    matrix_offset = Matrix()
    initial_matrix = Matrix()

    # tracking fields (per-bone)
    prev_target_loc: bpy.props.FloatVectorProperty(name="Prev Target", subtype='XYZ', default=(0,0,0), size=3)
    prev_head_loc:   bpy.props.FloatVectorProperty(name="Prev Head",   subtype='XYZ', default=(0,0,0), size=3)

    force_raw:    bpy.props.FloatVectorProperty(name="Force Raw",    subtype='XYZ', default=(0,0,0), size=3)
    force_smooth: bpy.props.FloatVectorProperty(name="Force Smooth", subtype='XYZ', default=(0,0,0), size=3)

    prev_aim_vec: bpy.props.FloatVectorProperty(name="Prev Aim", subtype='DIRECTION', default=(0,1,0), size=3)
    torque_raw:    bpy.props.FloatVectorProperty(name="Torque Raw",    subtype='XYZ', default=(0,0,0), size=3)
    torque_smooth: bpy.props.FloatVectorProperty(name="Torque Smooth", subtype='XYZ', default=(0,0,0), size=3)

classes = (
    SB_PT_ui,
    bones_collec,
    SB_OT_spring_modal, SB_OT_spring, SB_OT_select_bone,
    SB_OT_spring_stop,
    SB_OT_toggle_force_viz,
    SB_OT_diag_force_viz,
)

def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
    bpy.app.handlers.frame_change_post.append(spring_bone_frame_mode)

    bpy.types.Scene.sb_spring_bones = bpy.props.CollectionProperty(type=bones_collec)
    bpy.types.Scene.sb_global_spring = bpy.props.BoolProperty(name="Enable spring", default=False)
    bpy.types.Scene.sb_global_spring_frame = bpy.props.BoolProperty(name="Enable Spring", description="Enable Spring on frame change only", default=False)

    bpy.types.PoseBone.sb_bone_spring = bpy.props.BoolProperty(name="Enabled", default=False, description="Enable spring effect on this bone")
    bpy.types.PoseBone.sb_stiffness = bpy.props.FloatProperty(name="Stiffness", default=0.5, min=0.01, max=1.0, description="Bouncy/elasticity value, higher values lead to more bounciness")
    bpy.types.PoseBone.sb_damp = bpy.props.FloatProperty(name="Damp", default=0.7, min=0.0, max=10.0, description="Speed/damping force applied to the bone to go back to it initial position")
    bpy.types.PoseBone.sb_gravity = bpy.props.FloatProperty(name="Gravity", description="Additional vertical force to simulate gravity", default=0.0, min=-100.0, max=100.0)
    bpy.types.PoseBone.sb_bone_rot = bpy.props.BoolProperty(name="Rotation", default=False, description="The spring effect will apply on the bone rotation instead of location")
    bpy.types.PoseBone.sb_global_influence = bpy.props.FloatProperty(name="Influence", default=1.0, min=0.0, max=1.0, description="Global influence of spring motion")

    # Scene props for visualization
    bpy.types.Scene.sb_vis_forces = bpy.props.BoolProperty(
        name="Show Forces",
        description="Draw spring forces/torques in the 3D View",
        default=False,
    )
    bpy.types.Scene.sb_force_scale = bpy.props.FloatProperty(
        name="Force Scale",
        description="Scale for positional force debug lines",
        default=0.1, min=0.0, soft_max=10.0
    )
    bpy.types.Scene.sb_torque_scale = bpy.props.FloatProperty(
        name="Torque Scale",
        description="Scale for rotational torque debug lines",
        default=0.1, min=0.0, soft_max=10.0
    )

    # Scene props for physics mode
    bpy.types.Scene.sb_use_physics = bpy.props.BoolProperty(
        name="Physics Mode",
        description="Use spring–damper physics update instead of legacy integrator",
        default=False,
    )
    bpy.types.Scene.sb_phys_freq = bpy.props.FloatProperty(
        name="Freq (Hz)",
        description="Natural frequency of the spring",
        default=4.0, min=0.01, soft_max=20.0
    )
    bpy.types.Scene.sb_phys_zeta = bpy.props.FloatProperty(
        name="Damping Ratio",
        description="0 = none, 1 = critical, >1 = over-damped",
        default=0.7, min=0.0, soft_max=2.0
    )
    bpy.types.Scene.sb_target_alpha = bpy.props.FloatProperty(
        name="Target Smooth",
        description="EMA smoothing applied to the target before physics",
        default=0.2, min=0.0, max=1.0
    )

def unregister():
    from bpy.utils import unregister_class

    # remove draw handler if still present
    global _draw_handle
    if _draw_handle is not None:
        try:
            bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, 'WINDOW')
        except:
            pass
        _draw_handle = None

    for cls in reversed(classes):
        unregister_class(cls)

    # Safely remove handler if present
    try:
        bpy.app.handlers.frame_change_post.remove(spring_bone_frame_mode)
    except ValueError:
        pass

    del bpy.types.Scene.sb_spring_bones
    del bpy.types.Scene.sb_global_spring
    del bpy.types.Scene.sb_global_spring_frame

    del bpy.types.PoseBone.sb_bone_spring
    del bpy.types.PoseBone.sb_stiffness
    del bpy.types.PoseBone.sb_damp
    del bpy.types.PoseBone.sb_gravity
    del bpy.types.PoseBone.sb_bone_rot
    del bpy.types.PoseBone.sb_global_influence

    # remove viz props
    del bpy.types.Scene.sb_vis_forces
    del bpy.types.Scene.sb_force_scale
    del bpy.types.Scene.sb_torque_scale

    # remove physics props
    del bpy.types.Scene.sb_use_physics
    del bpy.types.Scene.sb_phys_freq
    del bpy.types.Scene.sb_phys_zeta
    del bpy.types.Scene.sb_target_alpha

if __name__ == "__main__":
    register()
