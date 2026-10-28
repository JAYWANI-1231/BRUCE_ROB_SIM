#!/usr/bin/env python3
"""
BRUCE Humanoid Robot - Interactive MuJoCo Simulation Environment
================================================================
Control BRUCE with English instructions, interact with physics (soccer ball,
obstacle boxes, ground contact), and visualize real-time 3D simulation.
"""

import os
import sys
import time
import math
import select
import threading
import argparse
import numpy as np
import mujoco
import mujoco.viewer

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_XML = os.path.join(SCRIPT_DIR, "interactive_scene.xml")

class BruceInteractiveSim:
    def __init__(self, scene_path=SCENE_XML):
        if not os.path.exists(scene_path):
            raise FileNotFoundError(f"Scene file not found: {scene_path}")
        
        self.scene_path = scene_path
        self.model = mujoco.MjModel.from_xml_path(self.scene_path)
        self.data = mujoco.MjData(self.model)
        
        # 16 Actuated Joints mapping
        self.joint_names = [
            "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r",
            "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
            "shoulder_pitch_r", "shoulder_roll_r", "elbow_pitch_r",
            "shoulder_pitch_l", "shoulder_roll_l", "elbow_pitch_l"
        ]
        
        self.qpos_adr = [self.model.joint(name).qposadr[0] for name in self.joint_names]
        self.qvel_adr = [self.model.joint(name).dofadr[0] for name in self.joint_names]
        self.actuator_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.joint_names]
        
        # Physics object IDs
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.ball_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "soccer_ball")
        self.box_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target_box")
        
        # Ball initial pose
        self.ball_init_pos = np.array([0.22, -0.076, 0.06])
        if self.ball_body_id != -1:
            self.ball_qpos_adr = self.model.jnt_qposadr[self.model.body_jntadr[self.ball_body_id]]
            self.ball_qvel_adr = self.model.jnt_dofadr[self.model.body_jntadr[self.ball_body_id]]
        
        # PD Gains
        self.kp = np.array([
            140, 140, 150, 200, 150,   # right leg
            140, 140, 150, 200, 150,   # left leg
             45,  45,  45,             # right arm
             45,  45,  45              # left arm
        ], dtype=float)
        
        self.kd = np.array([
            8, 8, 8, 12, 8,
            8, 8, 8, 12, 8,
            3, 3, 3,
            3, 3, 3
        ], dtype=float)
        
        # Nominal Standing Posture (slight knee flexion for stability)
        self.q_nominal = np.zeros(16, dtype=float)
        self.q_nominal[2] = 0.25    # hip_pitch_r
        self.q_nominal[3] = -0.50   # knee_pitch_r
        self.q_nominal[4] = 0.25    # ankle_pitch_r
        
        self.q_nominal[7] = 0.25    # hip_pitch_l
        self.q_nominal[8] = -0.50   # knee_pitch_l
        self.q_nominal[9] = 0.25    # ankle_pitch_l

        # Relaxed Human Arm Posture: arms hang naturally beside torso with gentle elbow bend
        self.q_nominal[10] = 0.05   # shoulder_pitch_r
        self.q_nominal[11] = 1.38   # shoulder_roll_r
        self.q_nominal[12] = 0.50   # elbow_pitch_r
        self.q_nominal[13] = 0.05   # shoulder_pitch_l
        self.q_nominal[14] = -1.38  # shoulder_roll_l
        self.q_nominal[15] = -0.50  # elbow_pitch_l
        
        # Current and target joint positions
        self.q_current = self.q_nominal.copy()
        self.q_target = self.q_nominal.copy()
        
        # State variables
        self.active_action = "stand"
        self.action_start_time = 0.0
        self.sim_time = 0.0
        self.gait_phase = 0.0
        self.gait_freq = 1.0  # Hz
        
        self.walk_speed_x = 0.0   # forward/backward
        self.walk_speed_y = 0.0   # lateral sidestep
        self.walk_yaw_rate = 0.0  # turning
        
        self.kick_leg = "right"
        self.running = True
        self.lock = threading.Lock()
        
        # Initialize robot in nominal standing posture
        self.reset_robot()

    def reset_robot(self):
        """Resets the robot to upright nominal posture and resets objects."""
        mujoco.mj_resetData(self.model, self.data)
        
        # Set base link position
        self.data.qpos[0:3] = [0.0, 0.0, 0.45]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]  # identity quaternion
        
        # Set nominal joint positions
        for idx, adr in enumerate(self.qpos_adr):
            self.data.qpos[adr] = self.q_nominal[idx]
            
        # Reset ball position
        self.reset_ball()
        
        mujoco.mj_forward(self.model, self.data)
        self.q_target = self.q_nominal.copy()
        self.active_action = "stand"
        self.walk_speed_x = 0.0
        self.walk_speed_y = 0.0
        self.walk_yaw_rate = 0.0

    def reset_ball(self):
        """Resets the soccer ball in front of BRUCE's right foot."""
        if self.ball_body_id != -1:
            self.data.qpos[self.ball_qpos_adr:self.ball_qpos_adr+3] = self.ball_init_pos
            self.data.qpos[self.ball_qpos_adr+3:self.ball_qpos_adr+7] = [1.0, 0.0, 0.0, 0.0]
            self.data.qvel[self.ball_qvel_adr:self.ball_qvel_adr+6] = 0.0

    def set_instruction(self, text):
        """Parses English instruction and sets robot action."""
        cmd = text.strip().lower()
        if not cmd:
            return "Empty command."

        with self.lock:
            self.action_start_time = self.sim_time

            # 1. WALKING & LOCOMOTION
            if any(k in cmd for k in ["walk forward", "go forward", "move forward", "march forward", "forward"]):
                self.active_action = "walk"
                self.walk_speed_x = 0.22
                self.walk_speed_y = 0.0
                self.walk_yaw_rate = 0.0
                return " Walking forward."

            elif any(k in cmd for k in ["walk backward", "go backward", "step back", "backward", "reverse"]):
                self.active_action = "walk"
                self.walk_speed_x = -0.18
                self.walk_speed_y = 0.0
                self.walk_yaw_rate = 0.0
                return " Walking backward."

            elif any(k in cmd for k in ["turn left", "rotate left", "spin left"]):
                self.active_action = "walk"
                self.walk_speed_x = 0.08
                self.walk_speed_y = 0.0
                self.walk_yaw_rate = 0.35
                return " Turning left."

            elif any(k in cmd for k in ["turn right", "rotate right", "spin right"]):
                self.active_action = "walk"
                self.walk_speed_x = 0.08
                self.walk_speed_y = 0.0
                self.walk_yaw_rate = -0.35
                return " Turning right."

            elif any(k in cmd for k in ["step left", "side step left", "sidestep left"]):
                self.active_action = "walk"
                self.walk_speed_x = 0.0
                self.walk_speed_y = 0.12
                self.walk_yaw_rate = 0.0
                return " Sidestepping left."

            elif any(k in cmd for k in ["step right", "side step right", "sidestep right"]):
                self.active_action = "walk"
                self.walk_speed_x = 0.0
                self.walk_speed_y = -0.12
                self.walk_yaw_rate = 0.0
                return " Sidestepping right."

            elif any(k in cmd for k in ["walk in place", "stepping", "march in place", "in place"]):
                self.active_action = "walk"
                self.walk_speed_x = 0.0
                self.walk_speed_y = 0.0
                self.walk_yaw_rate = 0.0
                return " Walking in place."

            elif cmd == "walk" or "start walking" in cmd:
                self.active_action = "walk"
                self.walk_speed_x = 0.20
                self.walk_speed_y = 0.0
                self.walk_yaw_rate = 0.0
                return " Walking forward."

            elif any(k in cmd for k in ["stop", "halt", "freeze", "pause"]):
                self.active_action = "stand"
                self.walk_speed_x = 0.0
                self.walk_speed_y = 0.0
                self.walk_yaw_rate = 0.0
                return " Stopping and stabilizing standing posture."

            # 2. PHYSICAL INTERACTION (KICK, PUSH, RESET BALL)
            elif any(k in cmd for k in ["kick ball", "kick right", "strike ball", "shoot", "kick"]):
                self.active_action = "kick"
                self.kick_leg = "right"
                return " Kicking the soccer ball with right foot!"

            elif any(k in cmd for k in ["kick left", "kick left foot"]):
                self.active_action = "kick"
                self.kick_leg = "left"
                return " Kicking with left foot!"

            elif any(k in cmd for k in ["push", "push forward", "shove"]):
                self.active_action = "push"
                return " Driving forward with arms outstretched to push."

            elif any(k in cmd for k in ["reset ball", "spawn ball", "new ball"]):
                self.reset_ball()
                return " Soccer ball reset in front of right foot."

            # 3. GESTURES & POSTURES
            elif any(k in cmd for k in ["wave right", "wave hand", "say hello", "hello", "hi", "wave"]):
                self.active_action = "wave"
                return " Waving right arm: Hello!"

            elif any(k in cmd for k in ["wave left", "wave left arm"]):
                self.active_action = "wave_left"
                return " Waving left arm!"

            elif any(k in cmd for k in ["squat", "crouch", "bend knees", "sit"]):
                self.active_action = "squat"
                return " Crouching down in deep squat."

            elif any(k in cmd for k in ["hands up", "raise arms", "celebrate", "victory", "cheer"]):
                self.active_action = "celebrate"
                return " Raising arms up in victory!"

            elif any(k in cmd for k in ["t-pose", "tpose", "stretch", "open arms"]):
                self.active_action = "t_pose"
                return " Extending arms in T-pose."

            elif any(k in cmd for k in ["bow", "lean forward", "respect"]):
                self.active_action = "bow"
                return " Bowing forward in greeting."

            elif any(k in cmd for k in ["dance", "groove"]):
                self.active_action = "dance"
                return " Dancing to the rhythm!"

            elif any(k in cmd for k in ["stand", "balance", "nominal", "upright"]):
                self.active_action = "stand"
                return " Restoring nominal standing posture."

            elif any(k in cmd for k in ["reset", "restart"]):
                self.reset_robot()
                return " Full simulation reset."

            elif any(k in cmd for k in ["help", "commands", "?"]):
                return self.get_help_text()

            else:
                return f"❓ Unrecognized instruction '{cmd}'. Type 'help' to see available English commands."

    def get_help_text(self):
        return (
            "\n" + "="*60 + "\n"
            " AVAILABLE ENGLISH INSTRUCTIONS FOR BRUCE:\n"
            "="*60 + "\n"
            "  • LOCOMOTION:\n"
            "    - 'walk' / 'walk forward'  : Walk forward\n"
            "    - 'walk backward'          : Walk backward\n"
            "    - 'turn left' / 'turn right': Turn while stepping\n"
            "    - 'step left' / 'step right': Sidestep laterally\n"
            "    - 'walk in place'          : Step in place\n"
            "    - 'stop' / 'halt'          : Return to standing balance\n\n"
            "  • PHYSICAL INTERACTION:\n"
            "    - 'kick' / 'kick ball'     : Dynamic soccer kick with right foot\n"
            "    - 'kick left'              : Kick with left foot\n"
            "    - 'push' / 'push forward'  : Forward push stance\n"
            "    - 'reset ball'             : Teleport ball back in front of foot\n\n"
            "  • GESTURES & POSTURES:\n"
            "    - 'wave' / 'say hello'     : Friendly arm wave\n"
            "    - 'squat' / 'crouch'       : Lower torso in deep squat\n"
            "    - 'hands up' / 'celebrate' : Victory arms in the air\n"
            "    - 'bow'                    : Respectful bow\n"
            "    - 't-pose'                 : Full lateral arm stretch\n"
            "    - 'dance'                  : Rhythmic dancing motion\n"
            "    - 'stand'                  : Stable upright balance\n"
            "    - 'reset'                  : Reset whole simulation\n"
            "="*60
        )

    def compute_action_trajectory(self, dt):
        """Computes joint targets based on active action and elapsed time."""
        t_action = self.sim_time - self.action_start_time
        q = self.q_nominal.copy()

        if self.active_action == "stand":
            # Breathing / subtle swaying
            sway = 0.01 * math.sin(2 * math.pi * 0.5 * self.sim_time)
            q[2] += sway
            q[4] -= sway
            q[7] += sway
            q[9] -= sway
            # Arm breathing
            breath = 0.012 * math.sin(2 * math.pi * 0.45 * self.sim_time)
            q[11] += breath * 0.5
            q[12] += breath
            q[14] -= breath * 0.5
            q[15] -= breath

        elif self.active_action == "walk":
            self.gait_phase += 2 * math.pi * self.gait_freq * dt
            phi = self.gait_phase
            
            # Lateral sway for weight shifting
            sway = 0.04 * math.sin(phi) + self.walk_speed_y * 0.2
            q[1] = -sway
            q[6] = -sway
            
            # Step pitch and lift
            step_r = (self.walk_speed_x * 0.8) * math.sin(phi)
            step_l = -(self.walk_speed_x * 0.8) * math.sin(phi)
            
            lift_r = 0.28 * max(0.0, math.sin(phi))
            lift_l = 0.28 * max(0.0, -math.sin(phi))
            
            # Turning offset via hip yaw
            turn_r = self.walk_yaw_rate * 0.4 * math.sin(phi)
            turn_l = -self.walk_yaw_rate * 0.4 * math.sin(phi)
            
            q[0] = turn_r      # hip_yaw_r
            q[2] += step_r     # hip_pitch_r
            q[3] -= lift_r     # knee_pitch_r
            q[4] += (lift_r - step_r)  # ankle_pitch_r
            
            q[5] = turn_l      # hip_yaw_l
            q[7] += step_l     # hip_pitch_l
            q[8] -= lift_l     # knee_pitch_l
            q[9] += (lift_l - step_l)  # ankle_pitch_l
            
            # Human-like Arm swing in opposition with dynamic elbow flexion
            swing = (self.walk_speed_x * 1.2) * math.sin(phi)
            q[10] = self.q_nominal[10] - swing
            q[12] = self.q_nominal[12] + 0.30 * max(0.0, -swing) - 0.12 * max(0.0, swing)
            q[13] = self.q_nominal[13] + swing
            q[15] = self.q_nominal[15] - 0.30 * max(0.0, swing) + 0.12 * max(0.0, -swing)
            
            # Forward torso propulsion
            self.data.qfrc_applied[0] = self.walk_speed_x * 35.0
            self.data.qfrc_applied[1] = self.walk_speed_y * 30.0
            self.data.qfrc_applied[5] = self.walk_yaw_rate * 8.0

        elif self.active_action == "kick":
            # 3-phase kick sequence
            is_right = (self.kick_leg == "right")
            hip_p = 2 if is_right else 7
            knee_p = 3 if is_right else 8
            ankle_p = 4 if is_right else 9
            opp_hip = 7 if is_right else 2
            
            # Stance leg bends slightly for support
            q[opp_hip] = 0.30
            
            if t_action < 0.40:
                # Phase 1: Wind-up (pull leg back, bend knee)
                alpha = t_action / 0.40
                q[hip_p] = 0.25 - 0.50 * alpha
                q[knee_p] = -0.50 - 0.45 * alpha
                q[ankle_p] = 0.25
                # Balance arms
                q[10] = 0.3 * alpha
                q[13] = -0.3 * alpha
            elif t_action < 0.80:
                # Phase 2: Dynamic Strike forward!
                alpha = (t_action - 0.40) / 0.40
                # Fast explosive strike
                q[hip_p] = -0.25 + 1.15 * math.sin(alpha * math.pi * 0.5)
                q[knee_p] = -0.95 + 0.85 * math.sin(alpha * math.pi * 0.5)
                q[ankle_p] = 0.25 + 0.15 * alpha
                # Propel forward slightly toward ball
                if t_action > 0.50 and t_action < 0.65:
                    self.data.qfrc_applied[0] = 40.0
            elif t_action < 1.30:
                # Phase 3: Return smoothly to standing posture
                alpha = (t_action - 0.80) / 0.50
                q[hip_p] = 0.90 * (1.0 - alpha) + 0.25 * alpha
                q[knee_p] = -0.10 * (1.0 - alpha) + (-0.50) * alpha
                q[ankle_p] = 0.40 * (1.0 - alpha) + 0.25 * alpha
            else:
                self.active_action = "stand"

        elif self.active_action == "push":
            # Forward stance with arms thrust forward
            q[2] = 0.35; q[3] = -0.65; q[4] = 0.30
            q[7] = 0.15; q[8] = -0.35; q[9] = 0.20
            # Both arms stiff and extended horizontally
            q[10] = -1.45; q[11] = 0.10; q[12] = 0.05
            q[13] = -1.45; q[14] = -0.10; q[15] = 0.05
            self.data.qfrc_applied[0] = 25.0

        elif self.active_action == "wave":
            # Right arm waving
            wave_osc = 0.45 * math.sin(2 * math.pi * 2.2 * t_action)
            q[10] = -1.60              # shoulder pitch up
            q[11] = 0.85               # shoulder roll outward
            q[12] = 1.10 + wave_osc    # elbow waving back and forth

        elif self.active_action == "wave_left":
            # Left arm waving
            wave_osc = 0.45 * math.sin(2 * math.pi * 2.2 * t_action)
            q[13] = -1.60
            q[14] = -0.85
            q[15] = -1.10 - wave_osc

        elif self.active_action == "squat":
            # Lowering base from 0.45m down to 0.32m
            depth = min(1.0, t_action / 0.8)
            q[2] = 0.25 + 0.45 * depth
            q[3] = -0.50 - 0.70 * depth
            q[4] = 0.25 + 0.35 * depth
            q[7] = 0.25 + 0.45 * depth
            q[8] = -0.50 - 0.70 * depth
            q[9] = 0.25 + 0.35 * depth
            # Arms forward for counterbalance
            q[10] = -0.8 * depth
            q[13] = -0.8 * depth

        elif self.active_action == "celebrate":
            # Arms raised high in victory
            lift = min(1.0, t_action / 0.6)
            q[10] = -2.60 * lift
            q[11] = 0.40 * lift
            q[12] = 0.60 * lift
            q[13] = -2.60 * lift
            q[14] = -0.40 * lift
            q[15] = -0.60 * lift

        elif self.active_action == "t_pose":
            lift = min(1.0, t_action / 0.6)
            q[10] = 0.0
            q[11] = 1.57 * lift
            q[12] = 0.0
            q[13] = 0.0
            q[14] = -1.57 * lift
            q[15] = 0.0

        elif self.active_action == "bow":
            if t_action < 1.0:
                depth = math.sin(t_action / 1.0 * math.pi)
                q[2] = 0.25 + 0.50 * depth
                q[4] = 0.25 - 0.30 * depth
                q[7] = 0.25 + 0.50 * depth
                q[9] = 0.25 - 0.30 * depth
                q[10] = 0.4 * depth
                q[13] = 0.4 * depth
            else:
                self.active_action = "stand"

        elif self.active_action == "dance":
            phi = 2 * math.pi * 1.5 * t_action
            q[1] = 0.06 * math.sin(phi)
            q[6] = 0.06 * math.sin(phi)
            q[3] = -0.50 + 0.15 * math.sin(phi * 2)
            q[8] = -0.50 + 0.15 * math.sin(phi * 2 + math.pi)
            q[10] = -1.2 + 0.6 * math.sin(phi)
            q[13] = -1.2 - 0.6 * math.sin(phi)

        return q

    def step_physics(self):
        """Advances physics by one step using PD torque control and active stabilization."""
        dt = self.model.opt.timestep
        self.sim_time += dt

        # Generate target joint angles
        q_target = self.compute_action_trajectory(dt)
        self.q_target = q_target

        # Current positions and velocities
        q_curr = np.array([self.data.qpos[adr] for adr in self.qpos_adr])
        v_curr = np.array([self.data.qvel[adr] for adr in self.qvel_adr])

        # Compute PD torques
        torques = self.kp * (q_target - q_curr) - self.kd * v_curr
        torques = np.clip(torques, -150.0, 150.0)

        # Apply torques to actuators
        for i, act_id in enumerate(self.actuator_ids):
            self.data.ctrl[act_id] = torques[i]

        # Active postural stabilization on torso freejoint (root attitude)
        # Keeps humanoid torso upright during dynamic interactions
        R_base = self.data.xmat[self.base_body_id].reshape(3, 3)
        pitch = -math.asin(max(-1.0, min(1.0, R_base[2, 0])))
        roll = math.atan2(R_base[2, 1], R_base[2, 2])
        w_base = self.data.qvel[3:6]

        self.data.qfrc_applied[3] = -75.0 * roll - 8.0 * w_base[0]
        self.data.qfrc_applied[4] = -75.0 * pitch - 8.0 * w_base[1]

        # Ground height maintenance
        target_z = 0.44 if self.active_action != "squat" else 0.33
        z_err = target_z - self.data.xpos[self.base_body_id][2]
        self.data.qfrc_applied[2] = 85.0 * max(0.0, z_err) - 10.0 * self.data.qvel[2]

        # Step MuJoCo
        mujoco.mj_step(self.model, self.data)

    def run_simulation(self, with_viewer=True, demo_mode=False):
        """Runs the main simulation loop with optional viewer and interactive thread."""
        print("\n" + "="*65)
        print("  🤖 BRUCE HUMANOID INTERACTIVE MUJOCO SIMULATION 🤖")
        print("="*65)
        print("Scene:       ", self.scene_path)
        print("Objects:      BRUCE Humanoid, Dynamic Soccer Ball, Obstacle Box")
        print("Physics:      1000 Hz Contact Physics, Friction, Restitution")
        print("Interactivity: Type English instructions in terminal below!")
        print("="*65 + "\n")

        # Start background command input thread
        if not demo_mode:
            input_thread = threading.Thread(target=self._command_listener_thread, daemon=True)
            input_thread.start()
        else:
            demo_thread = threading.Thread(target=self._demo_sequence_thread, daemon=True)
            demo_thread.start()

        if with_viewer:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                last_sync = time.perf_counter()
                
                while viewer.is_running() and self.running:
                    step_start = time.perf_counter()
                    
                    with self.lock:
                        self.step_physics()
                    
                    now = time.perf_counter()
                    if now - last_sync >= 0.016:  # ~60 FPS
                        viewer.sync()
                        last_sync = now
                    
                    elapsed = time.perf_counter() - step_start
                    if self.model.opt.timestep > elapsed:
                        time.sleep(self.model.opt.timestep - elapsed)
        else:
            print("Running in headless mode. Press Ctrl+C to stop.")
            while self.running:
                with self.lock:
                    self.step_physics()
                time.sleep(self.model.opt.timestep)

        print("\nSimulation ended.")

    def _command_listener_thread(self):
        """Listens for English instructions from user in terminal."""
        time.sleep(1.0)
        print(self.get_help_text())
        print("\n👉 Type an English instruction below (e.g. 'walk forward', 'kick ball', 'wave'):")
        
        # Check if stdin is an interactive terminal
        is_interactive = sys.stdin.isatty()

        while self.running:
            try:
                if is_interactive:
                    sys.stdout.write("\nBRUCE> ")
                    sys.stdout.flush()
                
                line = sys.stdin.readline()
                if not line:
                    # In non-interactive mode or on EOF, sleep rather than spin
                    time.sleep(0.5)
                    continue
                
                cmd = line.strip()
                if not cmd:
                    continue
                if cmd in ["exit", "quit", "q"]:
                    print("Exiting simulation...")
                    self.running = False
                    break
                
                response = self.set_instruction(cmd)
                print(response)
                
                # Print current robot state summary
                base_pos = self.data.xpos[self.base_body_id]
                ball_pos = self.data.xpos[self.ball_body_id] if self.ball_body_id != -1 else [0, 0, 0]
                print(f"   [State: Action='{self.active_action.upper()}', CoM Pos=({base_pos[0]:.2f}, {base_pos[1]:.2f}, {base_pos[2]:.2f}), Ball Pos=({ball_pos[0]:.2f}, {ball_pos[1]:.2f})]")
            
            except (KeyboardInterrupt, EOFError):
                self.running = False
                break

    def _demo_sequence_thread(self):
        """Automated showcase sequence demonstrating all capabilities."""
        time.sleep(1.5)
        demo_script = [
            ("wave", "👋 'wave' - Greeting the user with a friendly hand wave", 3.0),
            ("walk forward", "🚶 'walk forward' - Walking toward the soccer ball", 4.0),
            ("kick ball", "⚽ 'kick ball' - Dynamic kick launching the soccer ball!", 3.0),
            ("celebrate", "🎉 'celebrate' - Celebrating the goal with arms raised!", 2.5),
            ("turn left", "🔄 'turn left' - Turning toward the obstacle box", 3.5),
            ("squat", "🏋️ 'squat' - Deep knee squat balancing posture", 2.5),
            ("dance", "💃 'dance' - Performing rhythmic victory dance", 4.0),
            ("stand", "🧍 'stand' - Returning to stable standing balance", 2.0),
        ]
        
        print("\n🎬 STARTING AUTOMATED SHOWCASE DEMO...")
        for cmd, desc, duration in demo_script:
            if not self.running:
                break
            print(f"\n[DEMO] {desc}")
            self.set_instruction(cmd)
            time.sleep(duration)
            
        print("\n✨ Demo sequence complete! Entering interactive mode...")
        self._command_listener_thread()

def main():
    parser = argparse.ArgumentParser(description="Interactive BRUCE Humanoid Simulation in MuJoCo")
    parser.add_argument("--demo", action="store_true", help="Run automated showcase demo")
    parser.add_argument("--command", type=str, default="", help="Run a specific initial instruction")
    parser.add_argument("--no-viewer", action="store_true", help="Run without graphical viewer")
    args = parser.parse_args()

    sim = BruceInteractiveSim()
    if args.command:
        sim.set_instruction(args.command)
    
    sim.run_simulation(with_viewer=not args.no_viewer, demo_mode=args.demo)

if __name__ == "__main__":
    main()
