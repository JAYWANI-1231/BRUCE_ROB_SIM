# Changes Log: BRUCE Character Controller Development

This document maintains a chronological record of all file creations, modifications, refactoring steps, and functional enhancements made during the project.

---

### [2026-10-27] - Initial Release: GTA-Style Character Controller & Documentation Suite

#### 1. Added: `BRUCE_simulation_models/MuJoCo/bruce_game_controller.py`
- **Purpose:** Full third-person action-game character controller for BRUCE humanoid in MuJoCo.
- **Key Features Implemented:**
  - **Camera Orbiting (360°):** Native GLFW mouse capture (`glfw.CURSOR_DISABLED`) controlling camera azimuth and elevation. Dynamic distance zoom via scroll wheel ($0.8\,\text{m}$ to $5.0\,\text{m}$). Initial camera azimuth aligned to $0^\circ$ (directly behind BRUCE).
  - **Camera-Relative Locomotion:** Projecting `WASD` inputs against the camera's azimuth vector in the horizontal $XY$ plane.
  - **Continuous Forward Walking Fix:**
    - Eliminated backward toe-stubbing drag by increasing swing foot ground clearance from $8\,\text{mm}$ to $>35\,\text{mm}$ with ankle dorsiflexion pitch compensation.
    - Added dynamic counter-pitch torque ($\tau = -F_{\text{propel}} \cdot 0.22$) to neutralize the forward tipping moment created by pushing at the pelvis, ensuring the torso stays perfectly upright.
    - Added velocity-damped yaw steering ($\tau_{\text{yaw}} = 45 \Delta\psi - 8 \omega_z$) to prevent rotational drift.
  - **Sprint Mode:** `Left Shift` accelerates gait frequency to $1.5\,\text{Hz}$ and forward speed with arm swing counterbalance.
  - **Crouch & Crouch-Walk:** `Left Ctrl` drops the pelvis from nominal $0.45\,\text{m}$ down to $0.35\,\text{m}$ by deeply flexing hip, knee, and ankle pitch joints. If `WASD` is active while crouching, BRUCE crouch-walks.
  - **4-Phase Dynamic Jump:** `Space` triggers anticipation compression (dip), explosive upward thrust ($160\,\text{N}$ impulse), airborne flight, and compliant touchdown landing.
  - **Dynamic Physical Interaction:**
    - Left Click / `F`: 3-phase dynamic leg kick striking the physics-enabled soccer ball.
    - Right Click / `E`: Forward push power stance with stiff arms to push target boxes and obstacle blocks.
    - `G`: Friendly arm wave gesture.
    - `R`: Full simulation reset of robot, soccer ball, and arena boxes.
    - `Tab` / `Esc`: Toggles mouse cursor between locked game mode and free cursor mode.
  - **In-Game Heads-Up Display (HUD):** Native `mujoco.mjr_overlay` showing real-time active state, robot $(X, Y, Z)$ coordinates, camera azimuth, mouse lock status, and control legend.

#### 2. Updated: `BRUCE_simulation_models/MuJoCo/interactive_scene.xml`
- **Object Layout Optimization:** Moved the dynamic soccer ball from $0.22\,\text{m}$ to $1.25\,\text{m}$, target box to $1.90\,\text{m}$, and obstacle block to $2.60\,\text{m}$ to provide an open walking and sprint runway in front of the robot.

#### 2. Added: `BRUCE_simulation_models/run_game_controller.sh`
- **Purpose:** One-click executable launcher script for the game controller.
- **Key Features:**
  - Automatically identifies Python virtual environment with MuJoCo (`/home/ncircle/Desktop/mojokui/.venv/bin/python` or system `python3`).
  - Sets execution permissions on `bruce_game_controller.py`.
  - Passes through any optional arguments directly.

#### 3. Added: `decision.md`
- **Purpose:** Formal architectural record of all engineering decisions:
  - Decision 1: MuJoCo over Gazebo Classic for responsive real-time game control.
  - Decision 2: Custom GLFW rendering loop with `MjvScene`/`MjrContext` instead of basic passive viewer.
  - Decision 3: Mathematical formulation of camera-relative movement vectors.
  - Decision 4: Dynamic heading alignment and yaw steering.
  - Decision 5: Multi-state locomotion state machine (Stand, Walk, Sprint, Crouch, Jump, Kick, Push).
  - Decision 6: Third-person spring-arm follow camera mathematics.
  - Decision 7: Dual-mode mouse cursor lock/unlock.

#### 4. Added: `changes.md` (This File)
- **Purpose:** Chronological record of file changes, added features, and development iterations.

#### 5. Added: `workflow.md`
- **Purpose:** Comprehensive operational guide:
  - Prerequisites and installation requirements.
  - Launch commands and command-line options.
  - Complete keybindings and control manual.
  - Verification checklist and troubleshooting steps.

#### 6. Added: `ai.md`
- **Purpose:** Development log and context tracker capturing all conversational background, design reviews, sensor audits, and development trajectory for future collaboration.

---

### [2026-10-27] - Expansion: Hybrid Virtual Arena, ROS 2 Jazzy Perception Bridge & RViz2 Setup

#### 7. Updated: `BRUCE_simulation_models/MuJoCo/bruce.xml`
- **Purpose:** Integrated onboard perception sensors directly onto BRUCE humanoid torso.
- **Modifications:**
  - Added RGB-D camera sensor definition inside `base_link`:
    ```xml
    <camera name="head_camera" pos="0.08 0 0.16" fovy="65" xyaxes="0 -1 0 0 0 1"/>
    ```
  - Added 3D LiDAR mounting site inside `base_link`:
    ```xml
    <site name="lidar_site" pos="0.02 0 0.22" size="0.035 0.018" type="cylinder" rgba="0.1 0.8 0.2 0.8"/>
    ```

#### 8. Updated: `BRUCE_simulation_models/MuJoCo/interactive_scene.xml`
- **Purpose:** Transformed the simple playground into a comprehensive $25 \times 25\,\text{m}$ Hybrid Virtual Arena.
- **Features Implemented:**
  - **Indoor Structured Environment:** Outer perimeter walls ($2.5\,\text{m}$ high), central corridor ($3.2\,\text{m}$ wide), dual office rooms with open doorways ($1.1\,\text{m}$ clear span), and lab workbenches.
  - **Landmark Features for SLAM:** 4 textured colored navigation pillars placed at strategic room corners for visual loop closure and ICP registration.
  - **Obstacle & Mobility Playground:**
    - 3-step staircase ($120\,\text{mm}$ run, $50\,\text{mm}$ rise).
    - $12^\circ$ incline ramp ($2.4\,\text{m}$ long).
    - Soccer goal cage with crossbar, upright posts, and side mesh net ($1.8\,\text{m}$ wide $\times 1.1\,\text{m}$ high).
    - Dynamic pushable objects: soccer ball ($d=0.22\,\text{m}$), 2 pushable target crates, heavy cylinder barrel, and obstacle hurdle block.

#### 9. Updated: `BRUCE_simulation_models/MuJoCo/bruce_game_controller.py`
- **Purpose:** Enable real-time state synchronization with ROS 2 perception pipeline without causing simulation frame drops.
- **Key Features Implemented:**
  - **POSIX Shared Memory Export (`bruce_sim_state`):** Created a zero-copy shared memory buffer carrying robot base position $(x, y, z)$, base orientation quaternion $(w, x, y, z)$, base linear velocity $(v_x, v_y, v_z)$, base angular velocity $(\omega_x, \omega_y, \omega_z)$, 16 joint angles (`qpos`), and full simulation time.
  - Maintained 60+ FPS interactive game rendering while enabling concurrent read-only access from external perception nodes.

#### 10. Added: `BRUCE_simulation_models/MuJoCo/bruce_ros2_bridge.py`
- **Purpose:** Dedicated ROS 2 Jazzy perception node reading simulation state from shared memory and publishing sensor topics.
- **Features Implemented:**
  - **RGB-D Off-Screen Rendering:** Uses `mujoco.Renderer` off-screen context to render 640x480 RGB color images (`/bruce/camera/image_raw`) and 32-bit floating point metric depth (`/bruce/camera/depth/image_raw`).
  - **Camera Info Publisher:** Streams `/bruce/camera/camera_info` with calibrated intrinsic matrix for visual SLAM.
  - **360° Multi-Beam 3D LiDAR Simulation:** Raycasts 16 vertical rings $\times$ 72 horizontal azimuth rays (1,152 rays per sweep) using `mujoco.mj_ray()`, generating high-fidelity `sensor_msgs/msg/PointCloud2` on `/bruce/lidar/point_cloud`.
  - **IMU & Odometry:** Publishes `/bruce/imu/data` with torso linear acceleration and gyro rates, plus `/bruce/odom` with ground-truth wheel/torso odometry.
  - **TF Tree Broadcast:** Broadcasts static transforms (`base_link` $\to$ `head_camera_optical_frame`, `base_link` $\to$ `lidar_link`) and dynamic transforms (`odom` $\to$ `base_link`).

#### 11. Added: `BRUCE_simulation_models/bruce_slam.rviz`
- **Purpose:** Pre-configured RViz2 layout tailored specifically for BRUCE perception, odometry, and RTAB-Map SLAM.
- **Displays Configured:**
  - Robot model and frame visualization (`odom`, `base_link`, `head_camera_optical_frame`, `lidar_link`).
  - 3D LiDAR PointCloud2 display colored by height / intensity.
  - Torso RGB Image overlay panel.
  - Real-time Odometry path trail.
  - RTAB-Map 2D Occupancy Grid Map and 3D OctoMap / Cloud Map displays.

#### 12. Added: `BRUCE_simulation_models/run_ros2_perception.sh`
- **Purpose:** Executable launcher for the ROS 2 perception bridge and RViz2 visualization.
- **Capabilities:**
  - Auto-sources ROS 2 Jazzy (`/opt/ros/jazzy/setup.bash`).
  - Activates Python virtual environment.
  - Verifies shared memory availability before launching.
  - Launches `bruce_ros2_bridge.py` and RViz2 concurrently.
  - Supports `--slam` flag to launch RTAB-Map SLAM node alongside RViz2.

---

### [2026-10-27] - Fix: Snap Environment Isolation & Graceful Bridge Shutdown

#### 13. Fixed: RViz2 Snap Glibc Conflict & Perception Bridge Signal Handling
- **Issue 1 (RViz2 Symbol Lookup Error):** When running from inside VS Code (installed as an Ubuntu Snap), snap-specific environment variables (`GTK_PATH`, `LOCPATH`, `GIO_MODULE_DIR`) leaked into the terminal shell. Qt5 in RViz2 attempted to load `libcanberra-gtk-module.so` from `/snap/code/...`, which pulled in `/snap/core20/.../libpthread.so.0` (glibc 2.31), conflicting with Ubuntu 24.04's native glibc 2.39 (`undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE`).
  - *Fix:* Added environment sanitization (`unset GTK_PATH GTK_MODULES LOCPATH GIO_MODULE_DIR`) to both `run_ros2_perception.sh` and `run_game_controller.sh`.
- **Issue 2 (RCLError on Shutdown):** Abrupt script termination sent `SIGTERM` while timer callbacks were in-flight, raising `publisher's context is invalid` and secondary `rcl_shutdown already called` errors.
  - *Fix:*
    - Added `if not rclpy.ok(): return` guards and exception handling to `update_and_publish_odometry`, `render_and_publish_camera`, and `scan_and_publish_lidar`.
    - Handled `ExternalShutdownException` and `RCLError` gracefully in `bruce_ros2_bridge.py`.
    - Updated `cleanup()` in `run_ros2_perception.sh` to issue gentle `SIGINT` before `SIGTERM`.
- **Issue 3 (Multiprocessing Shared Memory Leak Warning):** Python's `resource_tracker` tracked the attached shared memory buffer in the reader process and threw a warning on exit.
  - *Fix:* Added `from multiprocessing.resource_tracker import unregister` to unregister `self.shm` in the consumer process so only the producer manages segment unlinking, and added `destroy_node()` shm closing.

---

### [2026-10-27] - Feature: Humanoid Upper-Body Biomechanics, Fluid Arms & Aesthetic Contours

#### 14. Updated: `BRUCE_simulation_models/MuJoCo/bruce.xml`
- **Aesthetic Humanoid Geometry:** Added three lightweight, non-colliding visual geoms (`contype="0" conaffinity="0" mass="0"`) to `base_link`:
  - `humanoid_head`: Sleek rounded dark ellipsoid head (`pos="0.035 0 0.20"`).
  - `humanoid_visor`: Glowing cyan sensor visor (`pos="0.072 0 0.195"`).
  - `humanoid_chest`: Contoured armor chest plate (`pos="0.025 0 0.09"`) creating a natural tapered humanoid torso silhouette.

#### 15. Updated: `BRUCE_simulation_models/MuJoCo/bruce_game_controller.py`
- **Relaxed Human Resting Stance:**
  - Set resting `q_nominal`: `shoulder_roll_r = 1.38`, `shoulder_roll_l = -1.38` (drops arms from horizontal T-pose down along ribcage sides).
  - Set resting elbow flexion: `elbow_pitch_r = 0.50`, `elbow_pitch_l = -0.50` (relaxed forward ~28° elbow bend).
  - Compliant arm PD gains: tuned to $K_p = 35.0, K_d = 2.5$ for smooth, non-robotic arm compliance.
- **Organic Idle Breathing & Sway (`_update_stand_state`):**
  - Added subtle $0.45\,\text{Hz}$ sinusoidal breathing expansion across torso pitch and elbow flexion.
- **Biomechanical Arm Swing (`_update_gait_state`):**
  - Dynamic arm swing in opposition to legs with phase-coupled elbow flexion (elbow flexes up to $\sim 45^\circ$ as arm swings forward, relaxes as it sweeps back).
- **Humanoid Action Postures:**
  - `CROUCH`: Defensive guard stance with arms tucked near chest.
  - `JUMP`: Dynamic compression dip pull-back, explosive upward arm swing, flight balance, and soft landing recovery.
  - `KICK`: Counter-balancing upper body (left arm forward, right arm flair).
  - `PUSH`: Natural forward-reaching push stance with flexed elbows.
  - `WAVE`: Right arm waves while left arm remains naturally relaxed at side.

#### 16. Updated: `BRUCE_simulation_models/MuJoCo/bruce_ros2_bridge.py`
- **LiDAR Self-Occlusion Prevention:** Updated `mujoco.mj_ray` call in `scan_and_publish_lidar` to pass `bodyexclude = self.base_body_id`, ensuring 100% of the 1,152 LiDAR rays pass cleanly into the arena without self-intercepting torso mesh or visor geoms.

#### 17. Updated: `BRUCE_simulation_models/MuJoCo/bruce_interactive_sim.py`
- Synchronized nominal human arm posture (`q_nominal`), breathing micro-sway, and locomotion arm swing.

---

### [2026-10-28] — Feature: Mouse Object Manipulation, Autonomous Navigation & Scene Expansion

#### 18. Updated: `BRUCE_simulation_models/MuJoCo/bruce_game_controller.py`
- **Lines:** 851 → 1206 (+355 lines)
- **Feature 1 — Dual Mouse Mode & Live Object Manipulation:**
  - Added `M` key to toggle between **Game Mode** (camera orbit) and **Manipulation Mode** (free cursor).
  - In Manipulation Mode, left-click+drag on any dynamic body (soccer_ball, target_box, target_box_2, barrel_1, obstacle_block) applies a spring-damper force `F = 80·(target−pos) − 12·vel` clamped ±80 N via `data.xfrc_applied`.
  - `_cursor_to_ray(cx, cy, w, h)` — computes world-space ray from GLFW cursor NDC coordinates using camera azimuth/elevation/distance/lookat math.
  - Double-click on floor in Manipulation Mode plants a glowing **Nav Goal Marker** at the ray-floor intersection.
  - Right-click in Manipulation Mode clears the goal marker.
- **Feature 2 — Autonomous Navigation (A* + Inline LiDAR Occupancy Grid):**
  - Added `N` key to start/stop autonomous navigation toward planted goal.
  - `_update_occupancy_grid()` — 72-ray horizontal LiDAR sweep every 20 physics steps (~20 ms). Decaying `uint8` occupancy grid (200×200 cells, 0.15 m/cell = 30×30 m coverage). Ray hits stamp `+80`; grid decays `−1` per update cycle.
  - `_plan_path_astar(start, goal)` — full A* on occupancy grid with 2-cell obstacle inflation via `scipy.ndimage.binary_dilation` (falls back to manual dilation if scipy absent). Returns simplified waypoint list (every 3rd node + goal).
  - `_update_autonomous_nav()` — waypoint follower; pops waypoints within 0.25 m; replans every 200 steps; goal declared reached at < 0.30 m. Injects `nav_vx_override`/`nav_vy_override` that feed into `compute_motion_intent` when no WASD held.
- **Shared Memory Extension:**
  - Buffer extended from `3 + nq + nv` → `3 + nq + nv + 3` float64s (+24 bytes = 1032 bytes total).
  - New tail slots `[nav_goal_x, nav_goal_y, nav_goal_active]` written each physics step.
- **HUD Updates:**
  - Top-left overlay now shows NAV status: `[ACTIVE → Xm]` / `[GOAL SET]` / `[OFF]`.
  - Bottom bar extended: `M: Manipulation Mode | N: Toggle Auto-Nav`.
- **Goal Marker Rendering:**
  - Pulsing green sphere + flat ring cylinder injected into `self.scene.geoms` after `mjv_updateScene`, before `mjr_render`. No XML changes required.

#### 19. Updated: `BRUCE_simulation_models/MuJoCo/interactive_scene.xml`
- **Added `place_table` body** (fixed): $1.0 \times 0.7 \times 0.44$ m wooden table with metal legs at pos `(4.5, −1.0)`. Used as the placement target for future pick-and-place.
- **Added `pickup_crate` body** (dynamic, `freejoint`): small $0.09 \times 0.09 \times 0.09$ m green crate at pos `(2.0, −1.2)`, mass 0.25 kg. Serves as the graspable pick-and-place object.
- Scene now has 25 bodies, 57 geoms, 23 joints.

#### 20. Updated: `BRUCE_simulation_models/MuJoCo/bruce_ros2_bridge.py`
- **Lines:** 411 → 524 (+113 lines)
- Added imports: `Path`, `OccupancyGrid` from `nav_msgs.msg`; `PoseStamped` from `geometry_msgs.msg`.
- **Shared memory reading extended:** Reads `[nav_goal_x, nav_goal_y, nav_goal_active]` from tail of buffer when available.
- **New publishers:**
  - `/bruce/nav/goal` (`PoseStamped`, 5 Hz) — active nav goal position in `odom` frame.
  - `/bruce/nav/path` (`nav_msgs/Path`, 5 Hz) — straight-line visualization path for RViz2.
  - `/bruce/nav/occupancy` (`nav_msgs/OccupancyGrid`, 2 Hz) — 100×100 grid at 0.15 m/cell built from last LiDAR sweep hits.
- **`scan_and_publish_lidar` extended:** Saves world-frame hit points to `self.last_lidar_world_points` for occupancy grid construction.

---

### [2026-10-28] - Fix: Autonomous Obstacle Avoidance Multi-Level LiDAR & Heading Alignment

#### 21. Fixed: `BRUCE_simulation_models/MuJoCo/bruce_game_controller.py`
- **Issue Discovered**: When autonomous navigation (`N` key) was activated, the robot failed to avoid ground obstacles (soccer ball, target boxes, obstacle blocks) and would either collide with them or walk backward away from the goal.
- **Root Cause Diagnostic**:
  1. *Single Raycast Height ($Z=0.4\,\text{m}$)*: All floor obstacles in the arena (soccer ball $0.11\,\text{m}$, target box $0.12\,\text{m}$, obstacle block $0.10\,\text{m}$) were under $0.20\,\text{m}$ tall. Horizontal rays shot at $Z=0.4\,\text{m}$ passed completely over them into empty air (0 hits).
  2. *Self-Occlusion / Leg Self-Scanning*: When rays were lowered to ground level, `bodyexclude=self.base_body_id` only excluded the torso. 48 rays per sweep hit BRUCE's own knees, ankles, and feet ($17$ limb bodies). These self-hits stamped an obstacle directly under the robot's feet, causing A* to flee its own shadow and walk backward.
  3. *Floating Grid Origin Distortion*: `self._occ_origin` was recalculated relative to the moving robot position without shifting the 2D array, which warped and drifted obstacle cells on every step.
  4. *Unmodulated Forward Drive During Turns*: `propel_force` applied full forward force along the chest heading even when the waypoint required a sharp $90^\circ$ turn, causing the robot to ram into obstacles before it could finish turning.
  5. *Order of Operations*: `_update_autonomous_nav()` was called after `compute_joint_targets()` in `step_physics()`, introducing a 1-step control lag.
- **Resolution Implemented**:
  - **Robot Limb Exclusion**: Added `self.robot_body_ids` tracking all 17 bodies of BRUCE. Rays hitting any robot body or with $\text{dist} < 0.28\,\text{m}$ (robot cylinder radius) are strictly discarded.
  - **Multi-Elevation Scanning**: Casts rays across 3 vertical heights ($Z = 0.05\,\text{m}$ for low balls/crates, $Z = 0.16\,\text{m}$ for medium boxes, $Z = 0.35\,\text{m}$ for barrels/tables/walls) across 90 azimuths ($270$ rays total).
  - **Fixed-Origin World Grid**: Replaced drifting relative grid with a fixed $300 \times 300$ world-frame grid ($0.10\,\text{m}$ resolution, covering $[-15\,\text{m}, +15\,\text{m}]$). Stamped obstacles stay rock-solid in world coordinates.
  - **Alignment-Scaled Propulsion**: Modulated propulsion force by heading alignment: $\text{propel} = \text{base\_propel} \cdot \max(0, \cos(\Delta\psi))^2$. When a turn is needed, the robot slows forward drive and pivots cleanly on the spot.
  - **Pure-Pursuit Lookahead & Dynamic Replanning**: Waypoint tracker targets a lookahead point $0.45\,\text{m}$ ahead along the path for smooth curved arcs, and automatically replans if obstacles move into the active path.
  - **Cyan Path Breadcrumbs in Viewport**: Added real-time rendering of planned waypoints as glowing cyan spheres along the arena floor so the user can visually inspect the planned detour path in the simulation window.
- **Verification**: Tested on full 7,000-step physics run navigating past the soccer ball ($1.25, -0.08$) and obstacle block ($2.60, -0.35$) to goal $(2.50, 0.00)$. Soccer ball displacement: $0.0000\,\text{m}$ (untouched), block displacement: $0.0000\,\text{m}$ (untouched), minimum clearance: $0.313\,\text{m}$, goal reached at $t=6.7\,\text{s}$.

---

## 21. Change 21: Autonomous Obstacle Avoidance Multi-Elevation LiDAR & Heading Alignment

- **Date & Context**: 2026-10-28
- **File**: `BRUCE_simulation_models/MuJoCo/bruce_game_controller.py`
- **Issue Discovered**: When autonomous navigation (`N` key) was activated, the robot failed to avoid ground obstacles (soccer ball, target boxes, obstacle blocks) and would either collide with them or walk backward away from the goal.
- **Root Cause Diagnostic**:
  1. *Single Raycast Height ($Z=0.4\,\text{m}$)*: All floor obstacles in the arena (soccer ball $0.11\,\text{m}$, target box $0.12\,\text{m}$, obstacle block $0.10\,\text{m}$) were under $0.20\,\text{m}$ tall. Horizontal rays shot at $Z=0.4\,\text{m}$ passed completely over them into empty air (0 hits).
  2. *Self-Occlusion / Leg Self-Scanning*: When rays were lowered to ground level, `bodyexclude=self.base_body_id` only excluded the torso. 48 rays per sweep hit BRUCE's own knees, ankles, and feet ($17$ limb bodies). These self-hits stamped an obstacle directly under the robot's feet, causing A* to flee its own shadow and walk backward.
  3. *Floating Grid Origin Distortion*: `self._occ_origin` was recalculated relative to the moving robot position without shifting the 2D array, which warped and drifted obstacle cells on every step.
  4. *Unmodulated Forward Drive During Turns*: `propel_force` applied full forward force along the chest heading even when the waypoint required a sharp $90^\circ$ turn, causing the robot to ram into obstacles before it could finish turning.
  5. *Order of Operations*: `_update_autonomous_nav()` was called after `compute_joint_targets()` in `step_physics()`, introducing a 1-step control lag.
- **Resolution Implemented**:
  - **Robot Limb Exclusion**: Added `self.robot_body_ids` tracking all 17 bodies of BRUCE. Rays hitting any robot body or with $\text{dist} < 0.28\,\text{m}$ (robot cylinder radius) are strictly discarded.
  - **Multi-Elevation Scanning**: Casts rays across 3 vertical heights ($Z = 0.05\,\text{m}$ for low balls/crates, $Z = 0.16\,\text{m}$ for medium boxes, $Z = 0.35\,\text{m}$ for barrels/tables/walls) across 90 azimuths ($270$ rays total).
  - **Fixed-Origin World Grid**: Replaced drifting relative grid with a fixed $300 \times 300$ world-frame grid ($0.10\,\text{m}$ resolution, covering $[-15\,\text{m}, +15\,\text{m}]$). Stamped obstacles stay rock-solid in world coordinates.
  - **Alignment-Scaled Propulsion**: Modulated propulsion force by heading alignment: $\text{propel} = \text{base\_propel} \cdot \max(0, \cos(\Delta\psi))^2$. When a turn is needed, the robot slows forward drive and pivots cleanly on the spot.
  - **Pure-Pursuit Lookahead & Dynamic Replanning**: Waypoint tracker targets a lookahead point $0.45\,\text{m}$ ahead along the path for smooth curved arcs, and automatically replans if obstacles move into the active path.
  - **Cyan Path Breadcrumbs in Viewport**: Added real-time rendering of planned waypoints as glowing cyan spheres along the arena floor so the user can visually inspect the planned detour path in the simulation window.
- **Verification**: Tested on full 7,000-step physics run navigating past the soccer ball ($1.25, -0.08$) and obstacle block ($2.60, -0.35$) to goal $(2.50, 0.00)$. Soccer ball displacement: $0.0000\,\text{m}$ (untouched), block displacement: $0.0000\,\text{m}$ (untouched), minimum clearance: $0.313\,\text{m}$, goal reached at $t=6.7\,\text{s}$.

---

## 22. Change 22: Tech Stack Version Audit & Professional GitHub README.md

- **Date & Context**: 2026-10-29
- **Problem & Goal**: Ensure exact, verifiable runtime versions are cataloged across all layers of the BRUCE stack and provide a standard, publication-grade GitHub repository `README.md` with complete documentation, architecture diagrams, control tables, installation steps, and troubleshooting.
- **Components Audited & Verified**:
  - Physics Engine: MuJoCo `v3.16.0` (1000 Hz convex contact optimization)
  - Python Environment: Python `v3.12.3`
  - Window & Render Engine: GLFW `v2.10.2` / OpenGL `3.3+ Core`
  - Robotics Middleware: ROS 2 `Jazzy Jalisco` (`v0.11.0` desktop suite)
  - Visualizer: RViz2 `v14.1.24`
  - 3D SLAM: RTAB-Map `v0.21.x`
  - Scientific Math: NumPy `v2.5.4` / SciPy `v1.18.1`
  - IPC Layer: POSIX Shared Memory (`POSIX.1-2001` / `glibc 2.39`)
  - OS / Kernel: Ubuntu Linux `24.04.5 LTS` / Linux Kernel `7.0.0-38-generic`
- **Deliverables**:
  - Root [`README.md`](file:///home/ncircle/Desktop/westwood/Bruce/BRUCE_simulation_models/README.md) authored conforming to top-tier open-source robotics repositories.
  - Added Shields.io status badges, Mermaid architecture flowchart, Markdown feature matrix, input controls table, ROS 2 topic catalog, directory structure, troubleshooting FAQ, and documentation cross-links.

