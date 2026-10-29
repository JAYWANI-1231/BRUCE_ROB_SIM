#!/usr/bin/env python3
"""
================================================================================
BRUCE Humanoid Robot - GTA-Style Third-Person Character Controller
================================================================================
Controls BRUCE like a video game character:
  - W / S       : Walk forward / backward (camera-relative)
  - A / D       : Strafe / turn left / right (camera-relative)
  - Left Shift  : Sprint / Run in active direction
  - Left Ctrl   : Crouch / Sit (lowers center of mass)
  - Space       : Dynamic jump (crouch, explosive thrust, flight, touchdown)
  - Mouse Move  : Orbit third-person camera (360° yaw, pitch up/down)
  - Mouse Wheel : Zoom camera in / out (0.8m to 4.5m)
  - Left Click  : Strike / Kick dynamic soccer ball (or key 'F')
  - Right Click : Push power stance to push boxes / obstacles (or key 'E')
  - G           : Wave arm / friendly greeting
  - R           : Reset robot & environment objects
  - Tab / Esc   : Toggle mouse cursor lock / unlock
  - M           : Toggle Manipulation Mode (drag objects / set nav goal)
                    Left-drag  : Drag a dynamic object (ball, box, barrel…)
                    Dbl-click  : Plant navigation goal on arena floor
                    Right-click: Clear navigation goal
  - N           : Toggle Autonomous Navigation toward current goal
================================================================================
"""

import os
import sys
import math
import time
import glfw
import numpy as np
import mujoco
from multiprocessing import shared_memory

try:
    from scipy.ndimage import binary_dilation
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False

    def binary_dilation(arr, iterations=1):
        """Fallback manual binary dilation when scipy is unavailable."""
        result = arr.copy()
        for _ in range(iterations):
            tmp = result.copy()
            tmp[1:, :] |= result[:-1, :]
            tmp[:-1, :] |= result[1:, :]
            tmp[:, 1:] |= result[:, :-1]
            tmp[:, :-1] |= result[:, 1:]
            result = tmp
        return result

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_XML = os.path.join(SCRIPT_DIR, "interactive_scene.xml")


class BruceGameController:
    def __init__(self, scene_path=SCENE_XML):
        if not os.path.exists(scene_path):
            raise FileNotFoundError(f"Scene file not found: {scene_path}")

        self.scene_path = scene_path
        self.model = mujoco.MjModel.from_xml_path(self.scene_path)
        self.data = mujoco.MjData(self.model)

        # 16 Actuated Joints Mapping
        self.joint_names = [
            "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r",
            "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
            "shoulder_pitch_r", "shoulder_roll_r", "elbow_pitch_r",
            "shoulder_pitch_l", "shoulder_roll_l", "elbow_pitch_l"
        ]

        self.qpos_adr = [self.model.joint(name).qposadr[0] for name in self.joint_names]
        self.qvel_adr = [self.model.joint(name).dofadr[0] for name in self.joint_names]
        self.actuator_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.joint_names]

        # Physics Object Body IDs
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.ball_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        self.box_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target_box")
        self.obstacle_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "obstacle_block")

        # Dynamic body IDs available for mouse-drag in Manipulation Mode
        self.dynamic_body_ids = []
        for _dyn_name in [
            "soccer_ball", "target_box", "target_box_2", "barrel_1", "obstacle_block", "pickup_crate",
            "chair_1", "chair_2", "chair_3", "chair_4", "office_chair_1", "office_chair_2"
        ]:
            _bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, _dyn_name)
            if _bid >= 0:
                self.dynamic_body_ids.append(_bid)

        # All robot limb body IDs to exclude from LiDAR occupancy grid (prevent self-occlusion / scanning own feet)
        self.robot_body_ids = set()
        for _r_name in [
            "base_link",
            "hip_yaw_link_r", "hip_roll_link_r", "hip_pitch_link_r", "knee_pitch_link_r", "ankle_pitch_link_r",
            "hip_yaw_link_l", "hip_roll_link_l", "hip_pitch_link_l", "knee_pitch_link_l", "ankle_pitch_link_l",
            "shoulder_pitch_link_r", "shoulder_roll_link_r", "elbow_pitch_link_r",
            "shoulder_pitch_link_l", "shoulder_roll_link_l", "elbow_pitch_link_l"
        ]:
            _rbid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, _r_name)
            if _rbid >= 0:
                self.robot_body_ids.add(_rbid)

        # Object Initial Poses (Placed in front to allow open walking and approach space)
        self.ball_init_pos = np.array([1.25, -0.076, 0.06])
        if self.ball_body_id != -1:
            self.ball_qpos_adr = self.model.jnt_qposadr[self.model.body_jntadr[self.ball_body_id]]
            self.ball_qvel_adr = self.model.jnt_dofadr[self.model.body_jntadr[self.ball_body_id]]

        if self.box_body_id != -1:
            self.box_qpos_adr = self.model.jnt_qposadr[self.model.body_jntadr[self.box_body_id]]
            self.box_qvel_adr = self.model.jnt_dofadr[self.model.body_jntadr[self.box_body_id]]
            self.box_init_pos = np.array([1.90, 0.35, 0.06])

        # PD Gains for Joints (Stiff hips, knees, and ankles for stable bipedal gait; compliant arms for fluid motion)
        self.kp = np.array([
            160, 160, 180, 220, 160,   # right leg
            160, 160, 180, 220, 160,   # left leg
             35,  35,  35,             # right arm (smooth, human-like compliance)
             35,  35,  35              # left arm (smooth, human-like compliance)
        ], dtype=float)

        self.kd = np.array([
            10, 10, 12, 16, 10,
            10, 10, 12, 16, 10,
             2.5, 2.5, 2.5,
             2.5, 2.5, 2.5
        ], dtype=float)

        # Nominal Standing Pose (Natural Human Stance)
        self.q_nominal = np.zeros(16, dtype=float)
        # Legs: stable, balanced bipedal stance
        self.q_nominal[2] = 0.25    # hip_pitch_r
        self.q_nominal[3] = -0.50   # knee_pitch_r
        self.q_nominal[4] = 0.25    # ankle_pitch_r

        self.q_nominal[7] = 0.25    # hip_pitch_l
        self.q_nominal[8] = -0.50   # knee_pitch_l
        self.q_nominal[9] = 0.25    # ankle_pitch_l

        # Relaxed Human Arm Posture: arms hang naturally beside torso with gentle elbow bend
        self.q_nominal[10] = 0.05   # shoulder_pitch_r (slight relaxed forward angle)
        self.q_nominal[11] = 1.38   # shoulder_roll_r (dropped down against right ribs)
        self.q_nominal[12] = 0.50   # elbow_pitch_r (relaxed forward ~28° elbow bend)

        self.q_nominal[13] = 0.05   # shoulder_pitch_l (slight relaxed forward angle)
        self.q_nominal[14] = -1.38  # shoulder_roll_l (dropped down against left ribs)
        self.q_nominal[15] = -0.50  # elbow_pitch_l (relaxed forward ~28° elbow bend)

        # Target and Current Joint Angles
        self.q_current = self.q_nominal.copy()
        self.q_target = self.q_nominal.copy()

        # Motion & Locomotion State
        self.active_state = "STAND"
        self.action_start_time = 0.0
        self.sim_time = 0.0
        self.gait_phase = 0.0

        # Locomotion Speeds
        self.walk_speed = 0.24       # m/s
        self.sprint_speed = 0.48     # m/s
        self.active_speed = 0.0
        self.heading_yaw = 0.0       # desired robot heading (radians)
        self.target_yaw_rate = 0.0

        # Key States
        self.key_w = False
        self.key_s = False
        self.key_a = False
        self.key_d = False
        self.key_shift = False
        self.key_ctrl = False

        # One-shot Action Triggers
        self.trigger_jump = False
        self.trigger_kick = False
        self.trigger_push = False
        self.trigger_wave = False
        self.jump_phase = 0          # 0: idle, 1: crouch, 2: thrust, 3: flight, 4: land
        self.jump_start_time = 0.0

        # Mouse & Camera State
        self.mouse_locked = True
        self.first_mouse = True
        self.last_mouse_x = 0.0
        self.last_mouse_y = 0.0
        self.mouse_sensitivity = 0.25

        # Third-Person Follow Camera (Positioned directly behind BRUCE facing +X forward)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.lookat[:] = [0.0, 0.0, 0.45]
        self.cam.distance = 1.85     # meters behind character
        self.cam.azimuth = 0.0       # degrees (0 = look along +X directly behind BRUCE)
        self.cam.elevation = -14.0   # degrees (downward angle)

        # MuJoCo Visualization Contexts
        self.opt = mujoco.MjvOption()
        self.scene = mujoco.MjvScene(self.model, maxgeom=1000)
        self.context = None
        self.window = None

        # Reset Robot and Objects to Initial State
        self.reset_simulation()

        # ── Manipulation Mode ──────────────────────────────────────────────
        self.manipulation_mode = False
        self.dragging_body_id = -1
        self.drag_target_pos = None
        self._drag_obj_z_offset = 0.05
        self._last_click_time = 0.0
        self._last_click_pos = (0, 0)

        # ── Navigation Goal & Autonomous Nav ──────────────────────────────
        self.nav_goal = None
        self.nav_goal_active = False
        self.nav_active = False
        self.nav_waypoints = []
        self._nav_replan_counter = 0
        self.nav_vx_override = 0.0
        self.nav_vy_override = 0.0

        # ── Fixed-Origin World Occupancy Grid (30m x 30m, 10cm resolution) ──
        self.occ_grid_res = 0.10
        self.occ_grid_size = 300
        self.occ_grid_origin = np.array([-15.0, -15.0])
        self.occ_grid = np.zeros((self.occ_grid_size, self.occ_grid_size), dtype=np.uint8)
        self._occ_decay_counter = 0
        self._lidar_step_counter = 0

        # Shared Memory State Export for ROS 2 Perception Bridge
        self.shm = None
        self._init_shared_memory()

    def _init_shared_memory(self):
        """Initializes POSIX shared memory buffer for streaming to ROS 2 perception node."""
        shm_name = "bruce_sim_state"
        # Total doubles: sim_time(1) + nq(1) + nv(1) + qpos(nq) + qvel(nv) + nav_goal(3)
        total_doubles = 3 + self.model.nq + self.model.nv + 3
        self.shm_size = total_doubles * 8
        try:
            old_shm = shared_memory.SharedMemory(name=shm_name)
            old_shm.close()
            old_shm.unlink()
        except FileNotFoundError:
            pass
        try:
            self.shm = shared_memory.SharedMemory(name=shm_name, create=True, size=self.shm_size)
            self.shm_buf = np.ndarray((total_doubles,), dtype=np.float64, buffer=self.shm.buf)
            self.shm_buf[0] = 0.0
            self.shm_buf[1] = float(self.model.nq)
            self.shm_buf[2] = float(self.model.nv)
        except Exception as e:
            print(f"Warning: Could not create shared memory ({e})")
            self.shm = None

    def reset_simulation(self):
        """Resets robot and all scene dynamic objects."""
        mujoco.mj_resetData(self.model, self.data)

        # Place robot base in nominal upright pose
        self.data.qpos[0:3] = [0.0, 0.0, 0.45]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]

        # Apply nominal joint angles
        for idx, adr in enumerate(self.qpos_adr):
            self.data.qpos[adr] = self.q_nominal[idx]

        # Reset soccer ball
        if self.ball_body_id != -1:
            self.data.qpos[self.ball_qpos_adr:self.ball_qpos_adr+3] = self.ball_init_pos
            self.data.qpos[self.ball_qpos_adr+3:self.ball_qpos_adr+7] = [1.0, 0.0, 0.0, 0.0]
            self.data.qvel[self.ball_qvel_adr:self.ball_qvel_adr+6] = 0.0

        # Reset target box
        if self.box_body_id != -1:
            self.data.qpos[self.box_qpos_adr:self.box_qpos_adr+3] = self.box_init_pos
            self.data.qpos[self.box_qpos_adr+3:self.box_qpos_adr+7] = [1.0, 0.0, 0.0, 0.0]
            self.data.qvel[self.box_qvel_adr:self.box_qvel_adr+6] = 0.0

        mujoco.mj_forward(self.model, self.data)
        self.q_target = self.q_nominal.copy()
        self.active_state = "STAND"
        self.heading_yaw = 0.0
        self.gait_phase = 0.0
        self.jump_phase = 0

    # =========================================================================
    # GLFW INPUT CALLBACKS (MOUSE & KEYBOARD)
    # =========================================================================

    def on_key(self, window, key, scancode, action, mods):
        """Handles keyboard events for WASD, Shift, Ctrl, Space, etc."""
        is_pressed = (action == glfw.PRESS or action == glfw.REPEAT)
        is_down = (action != glfw.RELEASE)

        # Locomotion Keys
        if key == glfw.KEY_W:
            self.key_w = is_down
        elif key == glfw.KEY_S:
            self.key_s = is_down
        elif key == glfw.KEY_A:
            self.key_a = is_down
        elif key == glfw.KEY_D:
            self.key_d = is_down
        elif key in (glfw.KEY_LEFT_SHIFT, glfw.KEY_RIGHT_SHIFT):
            self.key_shift = is_down
        elif key in (glfw.KEY_LEFT_CONTROL, glfw.KEY_RIGHT_CONTROL):
            self.key_ctrl = is_down

        # Action Triggers on Single Press
        if action == glfw.PRESS:
            if key == glfw.KEY_SPACE:
                if self.jump_phase == 0:
                    self.trigger_jump = True
            elif key == glfw.KEY_F:
                self.trigger_kick = True
            elif key == glfw.KEY_E:
                self.trigger_push = True
            elif key == glfw.KEY_G:
                self.trigger_wave = True
            elif key == glfw.KEY_R:
                self.reset_simulation()
            elif key in (glfw.KEY_TAB, glfw.KEY_ESCAPE):
                self.toggle_mouse_lock()
            elif key == glfw.KEY_M:
                self.manipulation_mode = not self.manipulation_mode
                if self.manipulation_mode:
                    self.mouse_locked = False
                    glfw.set_input_mode(self.window, glfw.CURSOR, glfw.CURSOR_NORMAL)
                    print("🖱️  Manipulation Mode: Left-drag objects | Double-click floor = Nav Goal | Right-click = Clear Goal")
                else:
                    self.mouse_locked = True
                    glfw.set_input_mode(self.window, glfw.CURSOR, glfw.CURSOR_DISABLED)
                    self.first_mouse = True
                    self.dragging_body_id = -1
                    print("🎮 Game Mode restored.")
            elif key == glfw.KEY_N:
                if self.nav_goal_active:
                    self.nav_active = not self.nav_active
                    if self.nav_active:
                        self.nav_waypoints = []
                        self._nav_replan_counter = 0
                        print(f"🤖 Autonomous navigation STARTED → goal {self.nav_goal}")
                    else:
                        print("🤖 Autonomous navigation PAUSED")
                else:
                    print("⚠️  Set a goal first: M-mode then double-click on floor")

    def on_mouse_move(self, window, xpos, ypos):
        """Orbits the third-person camera based on mouse movement."""
        # In manipulation mode, update drag target along floor or table surfaces
        if self.manipulation_mode and self.dragging_body_id >= 0:
            w, h = glfw.get_framebuffer_size(self.window)
            cam_pos, ray_dir = self._cursor_to_ray(xpos, ypos, w, h)

            # Cast ray excluding the dragged body to find the exact supporting surface (table top or floor)
            geom_id = np.zeros(1, dtype=np.int32)
            dist = mujoco.mj_ray(self.model, self.data, cam_pos, ray_dir,
                                 None, 1, self.dragging_body_id, geom_id)

            if dist > 0 and geom_id[0] >= 0:
                # Direct surface hit (table, desk, chair, floor)
                surface_pt = cam_pos + dist * ray_dir
                target_x = float(np.clip(surface_pt[0], -6.2, 6.2))
                target_y = float(np.clip(surface_pt[1], -6.2, 6.2))
                # Gentle 15mm hover clearance while carrying so it glides smoothly without scraping
                target_z = float(np.clip(surface_pt[2] + self._drag_obj_z_offset + 0.015, 0.02, 1.2))
                self.drag_target_pos = np.array([target_x, target_y, target_z])
            elif ray_dir[2] < -0.05:
                # Floor plane intersection (Z=0)
                t = -cam_pos[2] / ray_dir[2]
                floor_pt = cam_pos + t * ray_dir
                target_x = float(np.clip(floor_pt[0], -6.2, 6.2))
                target_y = float(np.clip(floor_pt[1], -6.2, 6.2))
                target_z = float(self._drag_obj_z_offset + 0.015)
                self.drag_target_pos = np.array([target_x, target_y, target_z])
            else:
                # Pointing toward horizon/sky: keep distance from camera, clamp height
                target_pt = cam_pos + 2.5 * ray_dir
                target_x = float(np.clip(target_pt[0], -6.2, 6.2))
                target_y = float(np.clip(target_pt[1], -6.2, 6.2))
                target_z = float(np.clip(target_pt[2], 0.05, 0.8))
                self.drag_target_pos = np.array([target_x, target_y, target_z])
            return

        if not self.mouse_locked:
            return

        if self.first_mouse:
            self.last_mouse_x = xpos
            self.last_mouse_y = ypos
            self.first_mouse = False
            return

        dx = xpos - self.last_mouse_x
        dy = ypos - self.last_mouse_y
        self.last_mouse_x = xpos
        self.last_mouse_y = ypos

        # Update Camera Azimuth (Yaw) and Elevation (Pitch)
        self.cam.azimuth -= dx * self.mouse_sensitivity
        self.cam.elevation = np.clip(
            self.cam.elevation - dy * self.mouse_sensitivity,
            -75.0,  # Look downward from above
            20.0    # Look upward from ground
        )

    def on_mouse_button(self, window, button, action, mods):
        """Handles mouse click actions (Left Click: Kick/Grab, Right Click: Push/Clear)."""
        # ── Manipulation Mode ────────────────────────────────────────────────
        if self.manipulation_mode:
            if action == glfw.PRESS:
                cx, cy = glfw.get_cursor_pos(self.window)
                w, h = glfw.get_framebuffer_size(self.window)
                if button == glfw.MOUSE_BUTTON_LEFT:
                    # Double-click detection
                    now = time.perf_counter()
                    is_double = (now - self._last_click_time) < 0.35
                    self._last_click_time = now

                    # Ray cast to find clicked body or floor
                    cam_pos, ray_dir = self._cursor_to_ray(cx, cy, w, h)
                    geom_id = np.zeros(1, dtype=np.int32)
                    dist = mujoco.mj_ray(self.model, self.data, cam_pos, ray_dir,
                                         None, 1, -1, geom_id)

                    hit_body = -1
                    if dist > 0 and geom_id[0] >= 0:
                        hit_body = self.model.geom_bodyid[geom_id[0]]

                    if hit_body in self.dynamic_body_ids:
                        self.dragging_body_id = hit_body
                        jadr = self.model.body_jntadr[hit_body]
                        vadr = self.model.jnt_dofadr[jadr]
                        # Zero velocity immediately so object doesn't fly away
                        self.data.qvel[vadr:vadr+6] = 0.0
                        self.data.xfrc_applied[hit_body, :] = 0.0
                        
                        body_z = self.data.xpos[hit_body][2]
                        self._drag_obj_z_offset = max(0.02, body_z)
                        self.drag_target_pos = self.data.xpos[hit_body].copy()
                        print(f"✋ Picked up {self.model.body(hit_body).name}")
                    elif is_double:
                        # Plant goal on floor
                        if abs(ray_dir[2]) > 1e-6:
                            t = -cam_pos[2] / ray_dir[2]
                            floor_pt = cam_pos + t * ray_dir
                            self.nav_goal = floor_pt[:2].copy()
                            self.nav_goal_active = True
                            self.nav_waypoints = []
                            print(f"🎯 Nav goal set at ({self.nav_goal[0]:.2f}, {self.nav_goal[1]:.2f})")

                elif button == glfw.MOUSE_BUTTON_RIGHT:
                    self.nav_goal_active = False
                    self.nav_active = False
                    self.nav_goal = None
                    print("🎯 Nav goal cleared.")

            elif action == glfw.RELEASE and button == glfw.MOUSE_BUTTON_LEFT:
                if self.dragging_body_id >= 0:
                    bid = self.dragging_body_id
                    jadr = self.model.body_jntadr[bid]
                    vadr = self.model.jnt_dofadr[jadr]
                    # Zero out all linear and angular velocities so object settles gently with 0 flying
                    self.data.qvel[vadr:vadr+6] = 0.0
                    self.data.xfrc_applied[bid, :] = 0.0
                    pos = self.data.xpos[bid]
                    print(f"📦 Placed {self.model.body(bid).name} at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
                self.dragging_body_id = -1
                self.drag_target_pos = None
            return  # Don't process game actions in manipulation mode

        # ── Game Mode ────────────────────────────────────────────────────────
        if action == glfw.PRESS:
            if not self.mouse_locked:
                # Re-lock mouse cursor on clicking inside window
                self.toggle_mouse_lock()
                return

            if button == glfw.MOUSE_BUTTON_LEFT:
                self.trigger_kick = True
            elif button == glfw.MOUSE_BUTTON_RIGHT:
                self.trigger_push = True

    def on_scroll(self, window, xoffset, yoffset):
        """Zooms third-person camera distance with mouse wheel."""
        self.cam.distance = float(np.clip(self.cam.distance - yoffset * 0.15, 0.8, 5.0))

    def toggle_mouse_lock(self):
        """Toggles between locked camera-look mode and unlocked free-cursor mode."""
        self.mouse_locked = not self.mouse_locked
        if self.mouse_locked:
            glfw.set_input_mode(self.window, glfw.CURSOR, glfw.CURSOR_DISABLED)
            self.first_mouse = True
        else:
            glfw.set_input_mode(self.window, glfw.CURSOR, glfw.CURSOR_NORMAL)

    # =========================================================================
    # CURSOR-TO-WORLD RAY CASTING, LIDAR GRID & AUTONOMOUS NAVIGATION
    # =========================================================================

    def _cursor_to_ray(self, cx, cy, width, height):
        """Converts a GLFW window cursor position to a world-space ray (origin, direction)."""
        x_ndc = (2.0 * cx / width) - 1.0
        y_ndc = 1.0 - (2.0 * cy / height)

        az_rad = math.radians(self.cam.azimuth)
        el_rad = math.radians(self.cam.elevation)

        fwd = np.array([
            math.cos(el_rad) * math.cos(az_rad),
            math.cos(el_rad) * math.sin(az_rad),
            math.sin(el_rad)
        ])
        right = np.array([math.sin(az_rad), -math.cos(az_rad), 0.0])
        up = np.cross(right, fwd)

        cam_pos = np.array(self.cam.lookat) - fwd * self.cam.distance

        fov_y = math.radians(45.0)
        aspect = width / height
        ray_dir = (fwd
                   + math.tan(fov_y / 2) * y_ndc * up
                   + math.tan(fov_y / 2) * aspect * x_ndc * right)
        ray_dir = ray_dir / np.linalg.norm(ray_dir)
        return cam_pos, ray_dir

    def _update_occupancy_grid(self):
        """
        Multi-elevation 360° LiDAR sweep into a fixed-origin world occupancy grid:
        - Scans at 3 vertical heights: Z=0.05m (floor items: ball, crates, blocks),
          Z=0.16m (medium crates), and Z=0.35m (tables, barrels, walls).
        - Accumulates into a fixed world-frame grid (30m x 30m, 10cm/cell).
        - Gradually decays unvisited cells so moved objects are cleared.
        """
        robot_pos = self.data.xpos[self.base_body_id].copy()
        origin = self.occ_grid_origin
        res = self.occ_grid_res
        sz = self.occ_grid_size

        # Slow decay of old cells every 3 cycles
        self._occ_decay_counter += 1
        if self._occ_decay_counter >= 3:
            self._occ_decay_counter = 0
            self.occ_grid = np.maximum(0, self.occ_grid.astype(np.int16) - 1).astype(np.uint8)

        geom_id = np.zeros(1, dtype=np.int32)
        heights = [0.05, 0.16, 0.35]
        azimuths = np.linspace(0, 2 * math.pi, 90, endpoint=False)

        for h in heights:
            ray_origin = np.array([robot_pos[0], robot_pos[1], h])
            for az in azimuths:
                ray_dir = np.array([math.cos(az), math.sin(az), 0.0])
                dist = mujoco.mj_ray(self.model, self.data, ray_origin, ray_dir,
                                      None, 1, self.base_body_id, geom_id)
                # Ignore self-hits on robot's own limbs and within robot's 28cm body radius
                if 0.28 < dist < 12.0 and geom_id[0] >= 0:
                    hit_body = self.model.geom_bodyid[geom_id[0]]
                    if hit_body not in self.robot_body_ids:
                        hit = ray_origin[:2] + ray_dir[:2] * dist
                        gx = int((hit[0] - origin[0]) / res)
                        gy = int((hit[1] - origin[1]) / res)
                        if 0 <= gx < sz and 0 <= gy < sz:
                            self.occ_grid[gy, gx] = min(255, int(self.occ_grid[gy, gx]) + 120)

    def _plan_path_astar(self, start_world, goal_world):
        """
        A* path planner on the inflated fixed-origin occupancy grid.
        Returns a collision-free list of waypoints from start to goal.
        """
        import heapq
        origin = self.occ_grid_origin
        res = self.occ_grid_res
        sz = self.occ_grid_size

        def w2g(wx, wy):
            return int((wx - origin[0]) / res), int((wy - origin[1]) / res)

        def g2w(gx, gy):
            return origin[0] + (gx + 0.5) * res, origin[1] + (gy + 0.5) * res

        # 4 iterations of 10cm cells = 40cm safe obstacle clearance
        occ_bool = self.occ_grid > 40
        occ_inflated = binary_dilation(occ_bool, iterations=4)

        def get_nearest_free(x, y):
            if not (0 <= x < sz and 0 <= y < sz):
                return x, y
            if not occ_inflated[y, x]:
                return x, y
            for r in range(1, 15):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < sz and 0 <= ny < sz and not occ_inflated[ny, nx]:
                            return nx, ny
            return x, y

        sx, sy = get_nearest_free(*w2g(float(start_world[0]), float(start_world[1])))
        gx, gy = get_nearest_free(*w2g(float(goal_world[0]), float(goal_world[1])))

        if not (0 <= sx < sz and 0 <= sy < sz and 0 <= gx < sz and 0 <= gy < sz):
            return []

        open_set = []
        heapq.heappush(open_set, (0.0, sx, sy))
        came_from = {}
        g_score = {(sx, sy): 0.0}

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            if cx == gx and cy == gy:
                path = []
                node = (gx, gy)
                while node in came_from:
                    path.append(g2w(*node))
                    node = came_from[node]
                path.reverse()
                # Subsample waypoints every 3 cells (0.3m spacing) + exact goal
                simplified = [np.array(p) for p in path[::3]]
                simplified.append(np.array(goal_world[:2], dtype=float))
                return simplified

            for dx, dy in neighbors:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < sz and 0 <= ny < sz):
                    continue
                if occ_inflated[ny, nx]:
                    continue
                step_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
                new_g = g_score[(cx, cy)] + step_cost
                if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_g
                    h = math.sqrt((nx - gx) ** 2 + (ny - gy) ** 2)
                    heapq.heappush(open_set, (new_g + h, nx, ny))
                    came_from[(nx, ny)] = (cx, cy)
        return []

    def _update_autonomous_nav(self):
        """
        Pure-pursuit lookahead waypoint tracker with dynamic replanning.
        """
        if not self.nav_goal_active or self.nav_goal is None:
            return

        robot_pos = self.data.xpos[self.base_body_id][:2].copy()
        dist_to_goal = np.linalg.norm(self.nav_goal - robot_pos)

        # Check goal reached
        if dist_to_goal < 0.30:
            self.nav_active = False
            self.active_state = "STAND"
            self.nav_waypoints = []
            print("✅ Navigation goal reached!")
            return

        # Periodic replan or replan if path empty
        replan_needed = (not self.nav_waypoints or self._nav_replan_counter <= 0)

        # Dynamic obstacle check: replan if upcoming waypoints are now blocked
        if not replan_needed and self.nav_waypoints:
            origin = self.occ_grid_origin
            res = self.occ_grid_res
            for wp in self.nav_waypoints[:4]:
                gx = int((wp[0] - origin[0]) / res)
                gy = int((wp[1] - origin[1]) / res)
                if 0 <= gx < self.occ_grid_size and 0 <= gy < self.occ_grid_size:
                    if self.occ_grid[gy, gx] > 80:
                        replan_needed = True
                        break

        if replan_needed:
            self.nav_waypoints = self._plan_path_astar(robot_pos, self.nav_goal)
            self._nav_replan_counter = 80  # replan every 80 steps (~80 ms)

        self._nav_replan_counter -= 1

        # Advance waypoints that the robot has already reached (within 0.35m)
        while len(self.nav_waypoints) > 1 and np.linalg.norm(self.nav_waypoints[0] - robot_pos) < 0.35:
            self.nav_waypoints.pop(0)

        # Pure-pursuit lookahead target: pick waypoint at least 0.45m ahead along path
        target_wp = self.nav_waypoints[0] if self.nav_waypoints else self.nav_goal
        for wp in self.nav_waypoints:
            if np.linalg.norm(wp - robot_pos) >= 0.45:
                target_wp = wp
                break

        direction = target_wp - robot_pos
        dist = np.linalg.norm(direction)
        if dist > 0.01:
            direction = direction / dist

        self.nav_vx_override = float(direction[0])
        self.nav_vy_override = float(direction[1])

    # =========================================================================
    # CAMERA-RELATIVE LOCOMOTION & ACTION TRAJECTORY
    # =========================================================================

    def compute_motion_intent(self):
        """
        Calculates world velocity vector and target heading based on WASD keys
        and the current camera azimuth.
        """
        fwd_input = (1.0 if self.key_w else 0.0) - (1.0 if self.key_s else 0.0)
        lat_input = (1.0 if self.key_d else 0.0) - (1.0 if self.key_a else 0.0)

        is_moving = (abs(fwd_input) > 0.1 or abs(lat_input) > 0.1)

        if not is_moving:
            # Autonomous navigation override — used when no WASD keys are held
            if self.nav_active and hasattr(self, 'nav_vx_override'):
                vx = self.nav_vx_override * self.walk_speed
                vy = self.nav_vy_override * self.walk_speed
                target_heading = math.atan2(vy, vx)
                return vx, vy, target_heading, True
            return 0.0, 0.0, 0.0, False

        # Convert Camera Azimuth from degrees to radians
        # Camera azimuth: 0 = +X, 90 = +Y
        cam_yaw = math.radians(self.cam.azimuth)

        # Camera Forward and Right Unit Vectors in World XY
        cam_forward = np.array([math.cos(cam_yaw), math.sin(cam_yaw)])
        cam_right = np.array([math.sin(cam_yaw), -math.cos(cam_yaw)])

        # Desired World Velocity Direction
        move_dir = fwd_input * cam_forward + lat_input * cam_right
        norm = np.linalg.norm(move_dir)
        if norm > 1e-4:
            move_dir /= norm

        # Determine Target Speed
        if self.key_ctrl:
            target_speed = self.walk_speed * 0.6  # Crouch-walk speed
        elif self.key_shift:
            target_speed = self.sprint_speed     # Sprint speed
        else:
            target_speed = self.walk_speed       # Normal walk speed

        target_heading = math.atan2(move_dir[1], move_dir[0])
        return move_dir[0] * target_speed, move_dir[1] * target_speed, target_heading, True

    def compute_joint_targets(self, dt):
        """Computes joint trajectory targets based on the active state machine."""
        q = self.q_nominal.copy()

        # Check Active State Overrides
        if self.jump_phase > 0 or self.trigger_jump:
            self._update_jump_state(q, dt)
        elif self.trigger_kick:
            self._update_kick_state(q, dt)
        elif self.trigger_push:
            self._update_push_state(q, dt)
        elif self.trigger_wave:
            self._update_wave_state(q, dt)
        elif self.key_ctrl:
            self._update_crouch_state(q, dt)
        else:
            # Standard Locomotion (Stand / Walk / Sprint)
            vx_world, vy_world, target_heading, is_moving = self.compute_motion_intent()

            if is_moving:
                self.active_state = "SPRINT" if self.key_shift else "WALK"
                self._update_gait_state(q, vx_world, vy_world, target_heading, dt)
            else:
                self.active_state = "STAND"
                self._update_stand_state(q, dt)

        return q

    def _update_stand_state(self, q, dt):
        """Active standing posture with subtle human breathing and organic micro-sway."""
        t = self.sim_time
        # Subtle organic breathing cycle (~0.45 Hz)
        breath = 0.012 * math.sin(2 * math.pi * 0.45 * t)
        chest_pitch = 0.006 * math.sin(2 * math.pi * 0.25 * t)

        # Torso / leg micro-compliance
        q[2] += chest_pitch
        q[4] -= chest_pitch
        q[7] += chest_pitch
        q[9] -= chest_pitch

        # Subtle arm breathing expansion/relaxation
        q[11] += breath * 0.5   # right shoulder roll
        q[12] += breath         # right elbow
        q[14] -= breath * 0.5   # left shoulder roll
        q[15] -= breath         # left elbow

    def _update_gait_state(self, q, vx_w, vy_w, target_heading, dt):
        """
        Omnidirectional gait generator with clean foot clearance, anti-toe-stubbing,
        and continuous forward propulsion aligned with camera look direction.
        """
        # Robot's current torso heading in world frame
        R_base = self.data.xmat[self.base_body_id].reshape(3, 3)
        current_yaw = math.atan2(R_base[1, 0], R_base[0, 0])

        # Angular error between current heading and movement direction
        yaw_err = math.atan2(math.sin(target_heading - current_yaw), math.cos(target_heading - current_yaw))

        # Gait Frequency
        gait_freq = 1.5 if self.active_state == "SPRINT" else 1.25
        self.gait_phase += 2 * math.pi * gait_freq * dt
        phi = self.gait_phase

        # Dynamic Joint Amplitudes
        pitch_amp = 0.42 if self.active_state == "SPRINT" else 0.32
        lift_amp = 0.58 if self.active_state == "SPRINT" else 0.52
        sway_amp = 0.065

        # Lateral Sway for Center-of-Mass Weight Shifting
        sway = sway_amp * math.sin(phi)
        q[1] = -sway
        q[6] = -sway

        # Leg Pitching and High Ground Clearance Lifting
        step_r = pitch_amp * math.sin(phi)
        step_l = -pitch_amp * math.sin(phi)
        lift_r = lift_amp * max(0.0, math.sin(phi))
        lift_l = lift_amp * max(0.0, -math.sin(phi))

        # Yaw Steering via Hip Yaw Joints
        steer_yaw = np.clip(yaw_err * 0.55, -0.40, 0.40)
        q[0] = steer_yaw * math.sin(phi)
        q[5] = -steer_yaw * math.sin(phi)

        # Right leg (hip pitch, knee flexion lift, ankle dorsiflexion level compensation)
        q[2] += step_r
        q[3] -= lift_r
        q[4] += (lift_r * 0.85 - step_r)

        # Left leg
        q[7] += step_l
        q[8] -= lift_l
        q[9] += (lift_l * 0.85 - step_l)

        # Human-like Biomechanical Arm Swing in Opposition to Legs
        arm_amp = 0.52 if self.active_state == "SPRINT" else 0.36
        swing = arm_amp * math.sin(phi)

        # Right Arm (swings opposite to right leg)
        # When right leg swings forward (sin(phi) > 0), right arm swings back:
        q[10] = self.q_nominal[10] - swing
        # Dynamic elbow flexion: flexes forward more when swinging forward, relaxes when swinging back
        q[12] = self.q_nominal[12] + 0.30 * max(0.0, -swing) - 0.12 * max(0.0, swing)
        q[11] = self.q_nominal[11] + 0.03 * math.sin(phi)

        # Left Arm (swings opposite to left leg)
        q[13] = self.q_nominal[13] + swing
        q[15] = self.q_nominal[15] - 0.30 * max(0.0, swing) + 0.12 * max(0.0, -swing)
        q[14] = self.q_nominal[14] - 0.03 * math.sin(phi)

        # Forward Propulsion Force with Alignment Scaling (CRITICAL FOR OBSTACLE AVOIDANCE)
        # Slows forward drive when heading error is large, allowing the robot to pivot smoothly
        forward_dir = R_base[:, 0]
        base_propel = 52.0 if self.active_state == "SPRINT" else 40.0
        align_factor = max(0.0, math.cos(yaw_err))
        propel_force = base_propel * (align_factor ** 2)
        self.data.qfrc_applied[0] = forward_dir[0] * propel_force
        self.data.qfrc_applied[1] = forward_dir[1] * propel_force

        # Responsive Heading Steering Torque
        w_base = self.data.qvel[3:6]
        self.data.qfrc_applied[5] = np.clip(65.0 * yaw_err - 10.0 * w_base[2], -20.0, 20.0)

    def _update_crouch_state(self, q, dt):
        """Crouching / Sitting down with balanced humanoid arm posture."""
        self.active_state = "CROUCH"
        q[2] = 0.85    # hip pitch forward
        q[3] = -1.55   # knee pitch deep bend
        q[4] = 0.70    # ankle pitch compensation
        q[7] = 0.85
        q[8] = -1.55
        q[9] = 0.70

        # Humanoid Crouch: arms tucked defensively forward near chest
        q[10] = -0.45   # right shoulder pitch forward
        q[11] = 0.95    # right shoulder roll tucked
        q[12] = 1.10    # right elbow flexed forward in guard stance
        q[13] = -0.45   # left shoulder pitch forward
        q[14] = -0.95   # left shoulder roll tucked
        q[15] = -1.10   # left elbow flexed forward in guard stance

        # If WASD is held, allow slow crouch-walking
        vx_w, vy_w, target_heading, is_moving = self.compute_motion_intent()
        if is_moving:
            self.active_state = "CROUCH-WALK"
            self.data.qfrc_applied[0] = vx_w * 18.0
            self.data.qfrc_applied[1] = vy_w * 18.0

    def _update_jump_state(self, q, dt):
        """4-Phase Dynamic Jump: Crouch -> Thrust -> Flight -> Touchdown."""
        self.active_state = "JUMP"

        if self.trigger_jump:
            self.trigger_jump = False
            self.jump_phase = 1
            self.jump_start_time = self.sim_time

        t_jump = self.sim_time - self.jump_start_time

        if self.jump_phase == 1:
            # Phase 1: Pre-jump Compression (Crouch dip)
            alpha = min(1.0, t_jump / 0.15)
            q[2] += 0.35 * alpha
            q[3] -= 0.65 * alpha
            q[4] += 0.30 * alpha
            q[7] += 0.35 * alpha
            q[8] -= 0.65 * alpha
            q[9] += 0.30 * alpha
            # Arms pull back ready to swing upward
            q[10] = self.q_nominal[10] + 0.30 * alpha
            q[13] = self.q_nominal[13] + 0.30 * alpha
            if t_jump >= 0.15:
                self.jump_phase = 2

        elif self.jump_phase == 2:
            # Phase 2: Explosive Extension Thrust Upward
            q[2] = 0.05
            q[3] = -0.10
            q[4] = 0.05
            q[7] = 0.05
            q[8] = -0.10
            q[9] = 0.05
            # Dynamic upward arm swing for momentum
            q[10] = -0.85
            q[12] = 0.80
            q[13] = -0.85
            q[15] = -0.80
            # Upward thrust impulse
            self.data.qfrc_applied[2] = 160.0

            # Forward impulse if walking
            vx_w, vy_w, _, is_moving = self.compute_motion_intent()
            if is_moving:
                self.data.qfrc_applied[0] = vx_w * 25.0
                self.data.qfrc_applied[1] = vy_w * 25.0

            if t_jump >= 0.30:
                self.jump_phase = 3

        elif self.jump_phase == 3:
            # Phase 3: Airborne Flight (Tuck legs slightly & stabilize arms)
            q[2] = 0.20
            q[3] = -0.40
            q[4] = 0.20
            q[7] = 0.20
            q[8] = -0.40
            q[9] = 0.20
            # Natural flight balance pose
            q[10] = -0.55
            q[11] = 1.15
            q[12] = 0.65
            q[13] = -0.55
            q[14] = -1.15
            q[15] = -0.65

            if t_jump >= 0.65:
                self.jump_phase = 4

        elif self.jump_phase == 4:
            # Phase 4: Touchdown Compliance & Balance Recovery
            alpha = min(1.0, (t_jump - 0.65) / 0.35)
            # Smoothly restore nominal standing angles
            q[:] = self.q_nominal.copy()
            if t_jump >= 1.0:
                self.jump_phase = 0

    def _update_kick_state(self, q, dt):
        """Kicks dynamic soccer ball forward with right foot and counter-balanced arms."""
        self.active_state = "KICK"
        if not hasattr(self, "_kick_start_time"):
            self._kick_start_time = self.sim_time

        t_kick = self.sim_time - self._kick_start_time

        # Stance left leg bends for stability
        q[7] = 0.30; q[8] = -0.55; q[9] = 0.25

        # Dynamic upper-body counter balance (left arm forward, right arm back/out)
        q[10] = 0.25; q[11] = 1.45; q[12] = 0.55
        q[13] = -0.65; q[14] = -1.15; q[15] = -0.85

        R_base = self.data.xmat[self.base_body_id].reshape(3, 3)
        forward_dir = R_base[:, 0]

        if t_kick < 0.30:
            # Wind-up: leg pulls back
            alpha = t_kick / 0.30
            q[2] = 0.25 - 0.50 * alpha
            q[3] = -0.50 - 0.45 * alpha
            q[4] = 0.25
        elif t_kick < 0.65:
            # Explosive strike forward!
            alpha = (t_kick - 0.30) / 0.35
            q[2] = -0.25 + 1.35 * math.sin(alpha * math.pi * 0.5)
            q[3] = -0.95 + 0.95 * math.sin(alpha * math.pi * 0.5)
            q[4] = 0.35
            # Dynamic propulsion step into the ball
            if 0.32 < t_kick < 0.58:
                self.data.qfrc_applied[0] = forward_dir[0] * 55.0
                self.data.qfrc_applied[1] = forward_dir[1] * 55.0
        elif t_kick < 1.05:
            # Recovery to stance
            alpha = (t_kick - 0.65) / 0.40
            q[2] = 1.0 * (1.0 - alpha) + 0.25 * alpha
            q[3] = -0.05 * (1.0 - alpha) + (-0.50) * alpha
            q[4] = 0.35 * (1.0 - alpha) + 0.25 * alpha
            # Arms blend back to nominal
            q[10:16] = self.q_nominal[10:16] * alpha + q[10:16] * (1.0 - alpha)
        else:
            self.trigger_kick = False
            delattr(self, "_kick_start_time")

    def _update_push_state(self, q, dt):
        """Forward power stance with natural pushing arms reaching forward."""
        self.active_state = "PUSH"
        if not hasattr(self, "_push_start_time"):
            self._push_start_time = self.sim_time

        t_push = self.sim_time - self._push_start_time

        # Power forward stance
        q[2] = 0.35; q[3] = -0.65; q[4] = 0.30
        q[7] = 0.15; q[8] = -0.35; q[9] = 0.20

        # Arms reaching forward with elbows flexed to push
        q[10] = -1.25; q[11] = 0.35; q[12] = 0.40
        q[13] = -1.25; q[14] = -0.35; q[15] = -0.40

        # Drive forward with physical pushing force along robot heading
        R_base = self.data.xmat[self.base_body_id].reshape(3, 3)
        forward_dir = R_base[:, 0]
        self.data.qfrc_applied[0] = forward_dir[0] * 35.0
        self.data.qfrc_applied[1] = forward_dir[1] * 35.0

        if t_push >= 1.2:
            self.trigger_push = False
            delattr(self, "_push_start_time")

    def _update_wave_state(self, q, dt):
        """Friendly arm wave gesture with relaxed left arm."""
        self.active_state = "WAVE"
        if not hasattr(self, "_wave_start_time"):
            self._wave_start_time = self.sim_time

        t_wave = self.sim_time - self._wave_start_time
        wave_osc = 0.45 * math.sin(2 * math.pi * 2.5 * t_wave)

        # Right arm waves
        q[10] = -1.60              # shoulder pitch up
        q[11] = 0.85               # shoulder roll outward
        q[12] = 1.10 + wave_osc    # elbow wave

        # Left arm remains naturally relaxed by side
        q[13] = self.q_nominal[13]
        q[14] = self.q_nominal[14]
        q[15] = self.q_nominal[15]

        if t_wave >= 1.8:
            self.trigger_wave = False
            delattr(self, "_wave_start_time")

    # =========================================================================
    # PHYSICS STEPPING & BALANCE STABILIZATION
    # =========================================================================

    def step_physics(self):
        """Computes PD torques and applies active stabilization at 1000 Hz."""
        dt = self.model.opt.timestep
        self.sim_time += dt

        # Reset applied forces from previous step
        self.data.qfrc_applied[:] = 0.0

        # ── 1. LiDAR occupancy grid update every 20 steps (~20 ms) ───────────
        self._lidar_step_counter += 1
        if self._lidar_step_counter >= 20:
            self._lidar_step_counter = 0
            self._update_occupancy_grid()

        # ── 2. Autonomous navigation motion injection (BEFORE computing targets)
        if self.nav_active and not (self.key_w or self.key_s or self.key_a or self.key_d):
            self._update_autonomous_nav()

        # Target joint positions
        q_target = self.compute_joint_targets(dt)
        self.q_target = q_target

        # Current Joint Angles & Velocities
        q_curr = np.array([self.data.qpos[adr] for adr in self.qpos_adr])
        v_curr = np.array([self.data.qvel[adr] for adr in self.qvel_adr])

        # PD Torque Calculation
        torques = self.kp * (q_target - q_curr) - self.kd * v_curr
        torques = np.clip(torques, -160.0, 160.0)

        for i, act_id in enumerate(self.actuator_ids):
            self.data.ctrl[act_id] = torques[i]

        # Active Torso Roll & Pitch Stabilization
        R_base = self.data.xmat[self.base_body_id].reshape(3, 3)
        pitch = -math.asin(max(-1.0, min(1.0, R_base[2, 0])))
        roll = math.atan2(R_base[2, 1], R_base[2, 2])
        w_base = self.data.qvel[3:6]

        # Dynamic counter-pitch compensation to prevent forward tipping torque during walking/sprinting
        propel_mag = 50.0 if self.active_state == "SPRINT" else (38.0 if self.active_state == "WALK" else 0.0)
        counter_pitch = propel_mag * 0.22 if self.active_state in ("WALK", "SPRINT") else 0.0

        self.data.qfrc_applied[3] = -90.0 * roll - 10.0 * w_base[0]
        self.data.qfrc_applied[4] = -120.0 * pitch - 16.0 * w_base[1] - counter_pitch

        # Ground Height Maintenance (Skip during jump flight)
        if self.jump_phase not in (2, 3):
            target_z = 0.31 if "CROUCH" in self.active_state else 0.44
            z_err = target_z - self.data.xpos[self.base_body_id][2]
            self.data.qfrc_applied[2] = 95.0 * max(0.0, z_err) - 12.0 * self.data.qvel[2]

        # ── Apply smooth mouse manipulation (Zero Catapult / Zero Flying) ────
        if self.dragging_body_id >= 0 and self.drag_target_pos is not None:
            bid = self.dragging_body_id
            jadr = self.model.body_jntadr[bid]
            if jadr >= 0:
                qadr = self.model.jnt_qposadr[jadr]
                vadr = self.model.jnt_dofadr[jadr]

                # Smoothly track target position without accumulating velocity
                curr_pos = self.data.qpos[qadr : qadr + 3]
                self.data.qpos[qadr : qadr + 3] += 0.25 * (self.drag_target_pos - curr_pos)

                # Zero out linear and angular velocity while being carried so kinetic energy never builds up
                self.data.qvel[vadr : vadr + 6] = 0.0
                self.data.xfrc_applied[bid, :] = 0.0
        else:
            # Clear any residual forces on dynamic bodies
            for bid in self.dynamic_body_ids:
                self.data.xfrc_applied[bid, :] = 0.0

        # Step MuJoCo Physics
        mujoco.mj_step(self.model, self.data)

        # Export updated state to Shared Memory for ROS 2 Perception Bridge
        if self.shm is not None:
            self.shm_buf[0] = self.sim_time
            nq = self.model.nq
            nv = self.model.nv
            self.shm_buf[3 : 3 + nq] = self.data.qpos[:]
            self.shm_buf[3 + nq : 3 + nq + nv] = self.data.qvel[:]
            # Nav goal fields: [nav_goal_x, nav_goal_y, nav_goal_active]
            nav_offset = 3 + nq + nv
            if self.nav_goal_active and self.nav_goal is not None:
                self.shm_buf[nav_offset]     = float(self.nav_goal[0])
                self.shm_buf[nav_offset + 1] = float(self.nav_goal[1])
                self.shm_buf[nav_offset + 2] = 1.0
            else:
                self.shm_buf[nav_offset + 2] = 0.0

    # =========================================================================
    # MAIN APPLICATION RUNNER (GLFW RENDER + HUD)
    # =========================================================================

    def run(self):
        """Initializes GLFW window and runs real-time game loop."""
        if not glfw.init():
            print("Failed to initialize GLFW.")
            return

        window_width, window_height = 1280, 720
        self.window = glfw.create_window(
            window_width, window_height,
            "BRUCE Humanoid - GTA Character Controller",
            None, None
        )

        if not self.window:
            glfw.terminate()
            print("Failed to create GLFW window.")
            return

        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # Register Input Callbacks
        glfw.set_key_callback(self.window, self.on_key)
        glfw.set_cursor_pos_callback(self.window, self.on_mouse_move)
        glfw.set_mouse_button_callback(self.window, self.on_mouse_button)
        glfw.set_scroll_callback(self.window, self.on_scroll)

        # Lock Mouse by Default
        glfw.set_input_mode(self.window, glfw.CURSOR, glfw.CURSOR_DISABLED)
        self.mouse_locked = True

        # Initialize MuJoCo Rendering Context
        self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        print("\n" + "="*70)
        print("  🎮 BRUCE HUMANOID - GTA CHARACTER CONTROLLER ACTIVE! 🎮")
        print("="*70)
        print("  • WASD        : Walk forward / backward / strafe (Camera-relative)")
        print("  • Shift (hold): SPRINT in movement direction")
        print("  • Ctrl (hold) : CROUCH / SIT DOWN (WASD = Crouch-walk)")
        print("  • Space       : JUMP")
        print("  • Mouse Move  : Orbit 3rd-person follow camera (Yaw & Pitch)")
        print("  • Mouse Wheel : Zoom camera in / out")
        print("  • Left Click  : KICK soccer ball (or key 'F')")
        print("  • Right Click : PUSH obstacles / target box (or key 'E')")
        print("  • G           : Wave hello")
        print("  • R           : Reset robot & arena objects")
        print("  • Tab / Esc   : Toggle mouse cursor lock / unlock")
        print("="*70 + "\n")

        last_render_time = time.perf_counter()

        while not glfw.window_should_close(self.window):
            # Run 1000 Hz physics steps to catch up to real-time
            dt_physics = self.model.opt.timestep
            now = time.perf_counter()
            elapsed = now - last_render_time

            # Advance physics in multiple sub-steps per frame (~60 FPS render)
            num_steps = max(1, min(20, int(elapsed / dt_physics)))
            for _ in range(num_steps):
                self.step_physics()

            last_render_time = now

            # Camera follows BRUCE's Torso
            robot_pos = self.data.xpos[self.base_body_id]
            self.cam.lookat[:] = [robot_pos[0], robot_pos[1], robot_pos[2] + 0.08]

            # Get Window Framebuffer Dimensions
            width, height = glfw.get_framebuffer_size(self.window)
            viewport = mujoco.MjrRect(0, 0, width, height)

            # Update Scene and Render
            mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.cam, mujoco.mjtCatBit.mjCAT_ALL, self.scene)

            # ── Goal Marker: pulsing green sphere + flat ring cylinder ────────
            if self.nav_goal_active and self.nav_goal is not None:
                t_pulse = time.perf_counter()
                pulse = 0.5 + 0.5 * math.sin(t_pulse * 4.0)
                rgba_sphere = np.array([0.05, 0.85 + 0.15 * pulse, 0.25, 0.8], dtype=np.float32)
                if self.scene.ngeom < self.scene.maxgeom:
                    g = self.scene.geoms[self.scene.ngeom]
                    mujoco.mjv_initGeom(
                        g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.12 + 0.03 * pulse, 0.0, 0.0]),
                        np.array([self.nav_goal[0], self.nav_goal[1], 0.12 + 0.04 * pulse]),
                        np.eye(3).flatten().astype(np.float64),
                        rgba_sphere
                    )
                    self.scene.ngeom += 1
                if self.scene.ngeom < self.scene.maxgeom:
                    g2 = self.scene.geoms[self.scene.ngeom]
                    mujoco.mjv_initGeom(
                        g2, mujoco.mjtGeom.mjGEOM_CYLINDER,
                        np.array([0.30, 0.30, 0.005]),
                        np.array([self.nav_goal[0], self.nav_goal[1], 0.005]),
                        np.eye(3).flatten().astype(np.float64),
                        np.array([0.05, 0.9, 0.3, 0.4], dtype=np.float32)
                    )
                    self.scene.ngeom += 1

            # ── Render Planned Path as Cyan Waypoint Markers ──────────────────
            if self.nav_active and self.nav_waypoints:
                for wp in self.nav_waypoints:
                    if self.scene.ngeom < self.scene.maxgeom:
                        g_wp = self.scene.geoms[self.scene.ngeom]
                        mujoco.mjv_initGeom(
                            g_wp, mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([0.035, 0.0, 0.0]),
                            np.array([wp[0], wp[1], 0.035]),
                            np.eye(3).flatten().astype(np.float64),
                            np.array([0.0, 0.85, 1.0, 0.75], dtype=np.float32)
                        )
                        self.scene.ngeom += 1

            mujoco.mjr_render(viewport, self.scene, self.context)

            # ── HUD Overlays ─────────────────────────────────────────────────
            mouse_status = "LOCKED (Tab to free)" if self.mouse_locked else "FREE (Click to lock)"
            mode_tag = " | MODE: MANIPULATION" if self.manipulation_mode else ""
            state_label = f"STATE: [{self.active_state}] | MOUSE: {mouse_status}{mode_tag}"
            pos_label = f"XYZ: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f}) | CAM YAW: {self.cam.azimuth:.1f}°"

            # Navigation status line
            if self.nav_active and self.nav_goal is not None:
                dist_to_goal = np.linalg.norm(self.nav_goal - self.data.xpos[self.base_body_id][:2])
                nav_label = f"NAV: [ACTIVE → dist {dist_to_goal:.1f}m]"
            elif self.nav_goal_active:
                nav_label = "NAV: [GOAL SET — press N to start]"
            else:
                nav_label = "NAV: [OFF]"

            mujoco.mjr_overlay(
                mujoco.mjtFontScale.mjFONTSCALE_150,
                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                viewport,
                "BRUCE 3RD-PERSON CONTROLLER",
                f"{state_label}\n{pos_label}\n{nav_label}",
                self.context
            )

            mujoco.mjr_overlay(
                mujoco.mjtFontScale.mjFONTSCALE_100,
                mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                viewport,
                "CONTROLS",
                "WASD: Move | Shift: Sprint | Ctrl: Crouch | Space: Jump | L-Click: Kick | R-Click: Push | Tab: Mouse | M: Manipulation Mode | N: Toggle Auto-Nav",
                self.context
            )

            glfw.swap_buffers(self.window)
            glfw.poll_events()

        glfw.destroy_window(self.window)
        glfw.terminate()

        # Clean up shared memory segment
        if self.shm is not None:
            try:
                self.shm.close()
                self.shm.unlink()
            except Exception:
                pass

        print("Simulation closed gracefully.")


if __name__ == "__main__":
    controller = BruceGameController()
    controller.run()
