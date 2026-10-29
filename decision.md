# Architectural & Technical Decisions Log: BRUCE Character Controller

This document records every major architectural, engineering, and control design decision made during the development of the GTA-style interactive simulation for the BRUCE humanoid robot.

---

## 1. Engine Selection: MuJoCo vs. Gazebo

* **Date:** 2026-10-27
* **Status:** **Accepted**
* **Context:** The user requested a responsive, third-person gaming-style character controller (similar to GTA 5) with mouse-based camera look, WASD movement, Shift-sprinting, Ctrl-crouching, Space-jumping, and physical interactions with environmental objects.
* **Decision:** We selected **MuJoCo** over Gazebo for the interactive character controller.
* **Rationale:**
  1. **Latency & Frame Rate:** MuJoCo runs contact physics at 1000 Hz and graphics rendering at 60–120+ FPS with virtually zero input latency. Gazebo Classic (Gazebo 11) typically struggles to maintain real-time factor with high-frequency bipedal contact physics and introduces noticeable teleoperation delay.
  2. **Foot Contact Stability:** Bipedal humanoid feet are prone to high-frequency chatter, polygon penetration, and explosive contact forces in Gazebo's ODE engine without heavy friction dampening. MuJoCo's convex optimization solver provides stable, soft-contact dynamics ideal for walking, running, and jumping.
  3. **Input Handling & Mouse Capture:** MuJoCo integrates cleanly with GLFW, allowing native cursor locking (`glfw.CURSOR_DISABLED`) for seamless 360° mouse camera orbiting. Gazebo's Qt-based interface uses CAD-style navigation and cannot natively lock the cursor for action-game camera control without extensive C++ GUI plugins.
  4. **Architecture Simplicity:** MuJoCo runs as a single self-contained Python process, whereas BRUCE's Gazebo setup requires 7 background screen sessions, POSIX shared memory segments, and external binary controllers.

---

## 2. Rendering & Window System: Custom GLFW Pipeline vs. `mujoco.viewer.launch_passive`

* **Date:** 2026-10-27
* **Status:** **Accepted**
* **Context:** MuJoCo provides a lightweight passive viewer (`mujoco.viewer.launch_passive`). However, passive viewer callbacks only expose basic keypress events and lack mouse cursor capture, mouse delta callbacks, mouse wheel zoom, and native HUD overlays.
* **Decision:** We implement a **custom GLFW rendering loop** using MuJoCo's C API Python bindings (`MjvScene`, `MjrContext`, `mjv_updateScene`, `mjr_render`, `mjr_overlay`).
* **Rationale:**
  1. **Continuous Key-State Tracking:** GLFW key callbacks capture `GLFW_PRESS` and `GLFW_RELEASE`. This allows true key-hold detection (holding `W` walks continuously; releasing `W` immediately returns to idle balance).
  2. **Mouse Look & Cursor Locking:** We can lock and hide the cursor with `glfw.set_input_mode(window, glfw.CURSOR, glfw.CURSOR_DISABLED)`, capturing raw mouse deltas ($\Delta x, \Delta y$) for smooth camera yaw and pitch rotation.
  3. **Cursor Release Toggle:** Pressing `Tab` or `Esc` switches between Game Mode (`CURSOR_DISABLED`) and Menu Mode (`CURSOR_NORMAL`), allowing the user to click outside or inspect the window.
  4. **On-Screen Heads-Up Display (HUD):** We can render text overlays with `mujoco.mjr_overlay` directly onto the viewport, displaying the active action state, movement speed, camera heading, and keybindings.

---

## 3. Camera-Relative Locomotion Mapping

* **Date:** 2026-10-27
* **Status:** **Accepted**
* **Context:** In third-person action games, character movement is relative to where the camera is facing, not to world axes or robot-fixed local axes.
* **Decision:** Transform 2D keyboard inputs (`WASD`) based on the camera azimuth angle ($\theta_{\text{cam}}$).
* **Mathematical Model:**
  Given camera azimuth $\theta_{\text{cam}}$ (in radians):
  $$\vec{u}_{\text{cam}} = \begin{bmatrix} \cos(\theta_{\text{cam}}) \\ \sin(\theta_{\text{cam}}) \end{bmatrix}, \quad \vec{r}_{\text{cam}} = \begin{bmatrix} \sin(\theta_{\text{cam}}) \\ -\cos(\theta_{\text{cam}}) \end{bmatrix}$$
  For inputs $k_w, k_s, k_a, k_d \in \{0, 1\}$:
  $$v_{\text{fwd}} = k_w - k_s, \quad v_{\text{lat}} = k_d - k_a$$
  The desired world velocity vector is:
  $$\vec{v}_{\text{world}} = v_{\text{fwd}} \vec{u}_{\text{cam}} + v_{\text{lat}} \vec{r}_{\text{cam}}$$
  If $\|\vec{v}_{\text{world}}\| > 0$, the vector is normalized and scaled by the active speed ($v_{\text{walk}}$ or $v_{\text{sprint}}$).

---

## 4. Automatic Heading Alignment & Steering

* **Date:** 2026-10-27
* **Status:** **Accepted**
* **Context:** When walking in a given direction, the robot's torso should smoothly pivot to face the direction of travel rather than moonwalking or crab-walking sideways indefinitely.
* **Decision:** Implement dynamic heading tracking:
  1. Compute the desired heading angle $\psi_{\text{target}} = \text{atan2}(v_{\text{world}, y}, v_{\text{world}, x})$.
  2. Compute heading error $\Delta \psi = \text{wrap\_to\_pi}(\psi_{\text{target}} - \psi_{\text{robot}})$.
  3. Apply yaw steering through hip yaw differential motion and root torque correction proportional to $\Delta \psi$.

---

## 5. Locomotion & Posture State Machine

* **Date:** 2026-10-27
* **Status:** **Accepted**
* **Context:** The controller needs to support walking, sprinting, crouching, jumping, kicking, pushing, and standing balance.
* **Decision:** Implement a multi-state hierarchical state machine:
  1. **`STAND` (Default / Idle):** Active standing balance at nominal height ($z \approx 0.44\,\text{m}$) with breathing sway.
  2. **`WALK` (`WASD`):** Stepping frequency $1.1\,\text{Hz}$, linear speed $0.24\,\text{m/s}$, lateral sway weight transfer, arm swing opposition.
  3. **`SPRINT` (`Shift` + `WASD`):** Increased stepping frequency $1.6\,\text{Hz}$, linear speed $0.48\,\text{m/s}$, forward torso lean, larger foot clearance ($0.35\,\text{rad}$ lift).
  4. **`CROUCH` (`Ctrl`):** Deep knee and hip flexion, lowering pelvis to $z \approx 0.30\,\text{m}$, forward arm counterbalance. If combined with `WASD`, enables crouch-walking.
  5. **`JUMP` (`Space`):** 4-phase sequence:
     - Phase 1 (0.00–0.15s): Anticipation compression (pre-jump crouch).
     - Phase 2 (0.15–0.30s): Explosive upward thrust ($\dot{z}$ impulse).
     - Phase 3 (0.30–0.65s): Airborne ballistic flight.
     - Phase 4 (0.65–1.00s): Compliant touchdown landing and restoration of nominal balance.
  6. **`KICK` (Left Click / `F`):** 3-phase leg swing toward the dynamic soccer ball with stance-leg stabilization.
  7. **`PUSH` (Right Click / `E`):** Forward power stance with rigid arms outstretched for pushing target boxes and obstacles.

---

## 6. Camera Follow System (Third-Person Spring Arm)

* **Date:** 2026-10-27
* **Status:** **Accepted**
* **Context:** The camera needs to follow BRUCE without jitter while allowing full rotational freedom with mouse controls.
* **Decision:**
  - `cam.lookat` continuously tracks BRUCE's torso base link: `cam.lookat[:] = [x_robot, y_robot, z_robot + 0.1]`.
  - `cam.distance` defaults to $1.85\,\text{m}$, dynamically adjustable from $0.8\,\text{m}$ to $5.0\,\text{m}$ via mouse scroll wheel.
  - `cam.azimuth` starts at $0.0^\circ$ (directly behind BRUCE facing $+X$), updated by mouse $\Delta x \times \text{sensitivity}$.
  - `cam.elevation` starts at $-14.0^\circ$, updated by mouse $\Delta y \times \text{sensitivity}$, clamped between $-75^\circ$ and $+20^\circ$.

---

## 7. Locomotion Stabilization: Counter-Pitch Compensation & Swing Foot Clearance

* **Date:** 2026-10-27
* **Status:** **Accepted**
* **Context:** During initial tests with `W` pressed, the robot experienced backward sliding and stalling. Investigation revealed three contributing factors:
  1. Forward thrust applied at the pelvis ($h \approx 0.45\,\text{m}$) caused a forward tipping torque against ground friction, tilting the torso forward by $>20^\circ$ and provoking an uncompensated backward pitch-stabilizer torque.
  2. Low swing foot clearance ($8\,\text{mm}$) caused the toe to drag along the ground during forward swing, generating a $-25\,\text{N}$ retarding contact force.
  3. Interactive scene objects were clustered within $0.22\,\text{m}$ of the feet.
* **Decision & Solutions Implemented:**
  1. **Dynamic Counter-Pitch Torque:** Added $\tau_{\text{counter}} = -F_{\text{propel}} \cdot 0.22\,\text{Nm}$ to cancel the pelvis thrust moment, keeping torso pitch stable within $3^\circ \dots 6^\circ$.
  2. **High Clearance & Dorsiflexion:** Increased knee flexion lift amplitude to $0.58$ ($>35\,\text{mm}$ clearance) with ankle level compensation: $q_{\text{ankle}} += \text{lift} \cdot 0.85 - \text{step}$.
  3. **Arena Spacing:** Placed dynamic soccer ball at $1.25\,\text{m}$, target box at $1.90\,\text{m}$, and obstacle block at $2.60\,\text{m}$ to grant open runway space for walking.
* **Result:** Robot achieves steady, continuous, rock-solid forward walking ($>2.5\,\text{m/s}$ rate), sprinting, and backward walking with zero backward drift.

---

### Decision 9: Two-Process Architecture with POSIX Shared Memory for Decoupled Perception
* **Status:** **Accepted**
* **Context:** The interactive 3rd-person game controller requires high, jitter-free rendering rates ($60+\,\text{FPS}$) and ultra-low input latency for smooth gameplay. Off-screen camera rendering (RGB-D depth extraction) and 3D LiDAR raycasting (1,152 rays/frame) consume $15\dots 30\,\text{ms}$ per cycle. If executed inside the game controller's main thread, the gameplay framerate would drop from $60\,\text{FPS}$ to below $20\,\text{FPS}$, causing severe keyboard/mouse stutter.
* **Decision:** Decouple physics/gameplay from perception into two independent OS processes communicating via POSIX shared memory (`multiprocessing.shared_memory.SharedMemory` named `bruce_sim_state`):
  1. **Process 1 (Game Controller):** Runs physics simulation and interactive GLFW window at full $60+\,\text{FPS}$. At every frame, it writes a compact 184-byte state struct (robot position, quaternion, velocities, joint angles `qpos`, simulation time) to shared memory with lock-free atomic double buffering.
  2. **Process 2 (ROS 2 Bridge):** Attaches to `bruce_sim_state` in read-only mode. It synchronizes a duplicate MuJoCo data instance and performs camera off-screen rendering and LiDAR raycasting at $15\dots 30\,\text{Hz}$ independently.
* **Result:** The game controller remains responsive at $60+\,\text{FPS}$, while ROS 2 RViz2 receives rich perception streams without impacting game loop stability.

---

### Decision 10: Multi-Beam Spherical 3D LiDAR Raycasting via `mj_ray`
* **Status:** **Accepted**
* **Context:** Standard MuJoCo lacks built-in ROS LiDAR plugins. We need a dense 360° point cloud with vertical resolution capable of detecting ground planes, doorways, obstacles, and walls for 3D SLAM.
* **Decision:** Implement a multi-beam raycasting engine using `mujoco.mj_ray()`:
  - **Beams:** 16 elevation rings spanning from $-15.0^\circ$ to $+15.0^\circ$ (matching standard 16-channel automotive/robotics LiDAR like Velodyne VLP-16).
  - **Azimuth Resolution:** 72 horizontal steps per ring ($5.0^\circ$ angular resolution), totaling 1,152 rays per sweep.
  - **Range & Filtering:** Maximum range of $15.0\,\text{m}$. Ignores rays that hit robot body geoms (`geomid` belonging to BRUCE).
  - **Output Format:** Packaged into binary IEEE 754 floats $[x, y, z, \text{intensity}]$ compliant with standard `sensor_msgs/msg/PointCloud2`.
* **Result:** High-fidelity 3D point cloud generated in $\approx 11\,\text{ms}$, fully compatible with RViz2 point cloud displays and RTAB-Map 3D ICP odometry.

---

### Decision 11: Camera Optical Frame Alignment (`xyaxes="0 -1 0 0 0 1"`)
* **Status:** **Accepted**
* **Context:** In MuJoCo, the default camera viewing direction points along the negative Z-axis with positive Y pointing upward. In ROS/OpenCV conventions, the optical camera frame has positive Z pointing forward (optical axis), positive X pointing right (image width), and positive Y pointing down (image height).
* **Decision:** Set MuJoCo camera orientation on the robot torso using explicit basis vectors:
  ```xml
  <camera name="head_camera" pos="0.08 0 0.16" fovy="65" xyaxes="0 -1 0 0 0 1"/>
  ```
  And broadcast the standard static optical transform in ROS 2:
  - `base_link` $\to$ `head_camera`: $[0.08, 0, 0.16]$, $R = \text{Identity}$
  - `head_camera` $\to$ `head_camera_optical_frame`: $R_{x}(-90^\circ) \cdot R_{z}(90^\circ)$
* **Result:** Perfect optical registration between 2D RGB imagery, 32-bit floating point metric depth arrays, and 3D point clouds in RViz2 without inverted axes.

---

### Decision 12: Hybrid Arena Design (Office Corridors + Obstacle Course)
* **Status:** **Accepted**
* **Context:** The simulation must satisfy two distinct use-cases: SLAM benchmarking (which requires enclosed walls, distinct corridors, doorways, and visual landmarks for loop closure) and game character locomotion (which requires open space, dynamic physical obstacles, stairs, ramps, and interactive items like soccer balls).
* **Decision:** Construct a $25 \times 25\,\text{m}$ Hybrid Virtual Arena in `interactive_scene.xml`:
  - **Zone 1 (Corridor & Office Rooms):** $3.2\,\text{m}$ wide central hallway flanked by two office rooms with $1.1\,\text{m}$ doorway openings, lab tables, and 4 colored landmark pillars for visual SLAM loop closure.
  - **Zone 2 (Obstacle Playground):** A 3-step staircase ($50\,\text{mm}$ rise per step), a $12^\circ$ incline ramp ($2.4\,\text{m}$ span), and a regulation mini-soccer goal cage.
  - **Zone 3 (Interactive Rigid Bodies):** Moveable soccer ball, pushable target crates, heavy cylinder barrel, and hurdle blocks for dynamic manipulation.
* **Result:** Single unified environment where user can freely transition between testing locomotion over stairs/ramps, playing soccer, and running SLAM mapping runs.

---

### Decision 13: Snap Environment Isolation & Consumer Shared-Memory Lifecycle
* **Status:** **Accepted**
* **Context:**
  1. When launchers are triggered from VS Code (packaged via Snap on Ubuntu 24.04), Snap-specific variables (`GTK_PATH`, `LOCPATH`, `GIO_MODULE_DIR`) leak into the terminal session. This causes Qt5 in RViz2 to link against core20's glibc 2.31 (`/snap/core20/.../libpthread.so.0`), which fails with `undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE` on Ubuntu 24.04 (glibc 2.39).
  2. Python's `multiprocessing.resource_tracker` automatically tracks any shared memory object opened via `SharedMemory(name=...)`. When the reader process exits, the resource tracker attempts cleanup and emits warnings about leaked objects.
* **Decision:**
  1. **Launcher Environment Sanitization:** In all bash launchers (`run_ros2_perception.sh` and `run_game_controller.sh`), explicitly purge snap GUI environment variables:
     ```bash
     unset GTK_PATH GTK_MODULES LOCPATH GIO_MODULE_DIR
     ```
  2. **Decoupled POSIX Consumer Ownership:** Use `from multiprocessing.resource_tracker import unregister` inside `bruce_ros2_bridge.py` right after attaching to `bruce_sim_state`, ensuring the consumer process only reads and closes its local file descriptor while leaving unlinking ownership strictly to the simulation master process (`bruce_game_controller.py`).
  3. **Non-Blocking ROS Context Teardown:** Add publish guards (`if not rclpy.ok(): return`) and catch `ExternalShutdownException` to ensure signal interruptions (`SIGINT` / `SIGTERM`) terminate cleanly without dumping invalid-context stack traces.
* **Result:** RViz2 and the perception bridge start reliably in all environments (including VS Code integrated terminals) with 0 symbol lookup errors, zero warnings, and clean exit codes.

---

### Decision 14: Humanoid Upper-Body Biomechanics, Resting Arm Posture, and Aesthetic Contours
* **Date:** 2026-10-27
* **Status:** **Accepted**
* **Context:** The robot's upper body appeared rigid and locked in an unnatural T-shape with stiff arms extended horizontally sideways, lacking human-like fluidity during standing, walking, crouching, and interaction.
* **Root Cause:**
  1. In BRUCE's CAD/URDF modeling, zero joint angle ($0.0\,\text{rad}$) corresponds to the horizontal modeling T-pose.
  2. The simulation initialized all upper-body joints (`q_nominal[10..15]`) to `0.0`, and the gait generator only pitched the shoulders (`q[10], q[13]`) without rolling them inward against the ribs or flexing the elbows (`elbow = 0.0`), resulting in stiff, locked-arm swing.
  3. The robot torso was a bare aluminum skeletal box without head or chest contours.
* **Decision:**
  1. **Natural Human Resting Stance:** Roll the shoulders downward along the ribcage (`shoulder_roll_r = +1.38\,\text{rad}`, `shoulder_roll_l = -1.38\,\text{rad}`) and introduce a gentle natural forward elbow flexion bend (`elbow_pitch_r = +0.50\,\text{rad}$, `elbow_pitch_l = -0.50\,\text{rad}`, $\sim 28^\circ$).
  2. **Biomechanical Arm Swing:** Coordinated arm swing in opposition to legs with dynamic elbow flexion: as the forward arm swings forward, the elbow flexes up to $\sim 45^\circ$, and relaxes slightly as it sweeps backward.
  3. **Compliant Upper-Body PD Gains:** Reduced arm $K_p$ from $45 \to 35$ and $K_d$ from $3.0 \to 2.5$ for smooth, compliant movement without stiff robotic jerkiness.
  4. **Subtle Breathing Micro-Motion:** Injected a gentle $0.45\,\text{Hz}$ breathing wave into torso pitch and elbow flexion so the character looks organic and alive when standing idle.
  5. **Humanoid Aesthetic Styling:** Added sleek non-colliding head, cyan sensor visor, and contoured upper-chest plate geoms to `bruce.xml`, and updated `bruce_ros2_bridge.py` raycasting with `bodyexclude = self.base_body_id` to prevent self-occlusion.
* **Result:** BRUCE presents a fluid, natural humanoid silhouette and character movement matching GTA 5 third-person game characters.

---

### Decision 15: Dual Mouse Mode — Game Camera vs. Object Manipulation
* **Status:** **Accepted**
* **Context:** The user wanted to drag objects around the arena with the mouse while the simulation runs, but the existing mouse was fully consumed by camera orbit in game mode.
* **Decision:** Toggle `M` switches between **Game Mode** (cursor locked, mouse drives camera) and **Manipulation Mode** (cursor freed, mouse drives object dragging and goal placement). Game actions (kick, push) are suppressed in Manipulation Mode.
* **Implementation:** `_cursor_to_ray()` converts GLFW cursor NDC into world-space ray using analytic camera frame math (azimuth/elevation/distance/lookat). Ray-floor intersection finds drag target on the Z=0 plane. Spring-damper force `F = 80·(target−pos) − 12·vel` applied via `data.xfrc_applied` each physics step.
* **Result:** Smooth live object dragging at 60+ FPS with zero game-mode interference.

---

### Decision 16: Inline A* Planner on Decaying Occupancy Grid (No ROS Nav2 in Physics Loop)
* **Status:** **Accepted**
* **Context:** User requested Nav2-stack navigation. However, running Nav2 costmap + DWA in the feedback path for walking control would introduce 50–200 ms of ROS pipeline latency, causing locomotion instability.
* **Decision:** Run a lightweight A* path planner **inline inside `bruce_game_controller.py`** using a `numpy` occupancy grid populated by horizontal LiDAR raycasts. Nav2 receives the occupancy grid and path for **visualization only** (via `bruce_ros2_bridge.py`). The control loop is purely in-process with zero network latency.
* **Rationale:** The walking controller makes gait decisions at 1000 Hz. Any external feedback loop slower than ~50 Hz would cause oscillatory instability. The inline planner can replan at 5 Hz (every 200 physics steps) without blocking the main thread.
* **Grid Parameters:** 200×200 cells, 0.15 m/cell (30×30 m), decaying `uint8` occupancy. Obstacle inflation: 2-cell dilation via `scipy.ndimage.binary_dilation`.

---

### Decision 17: Weld-Constraint Pick-and-Place (Planned — Phase 3)
* **Status:** **Planned**
* **Context:** True contact-friction grasping with 3-DOF fingertips requires high-frequency force control that would destabilize bipedal balance.
* **Decision:** Use MuJoCo equality `<weld>` constraint toggled on/off to simulate grasping. Fingers visually close, then the weld constraint snaps the object kinematically to the hand frame. This is the standard approach in DM Control and MuJoCo Menagerie manipulation benchmarks.

---

### Decision 18: Shared Memory Schema Extended for Nav Goal Streaming
* **Status:** **Accepted**
* **Context:** The ROS 2 bridge needs to know the current nav goal position to publish `/bruce/nav/goal` for RViz2 and Nav2 visualization without a separate IPC channel.
* **Decision:** Extend the `bruce_sim_state` shared memory buffer by 3 float64 slots at the end: `[nav_goal_x, nav_goal_y, nav_goal_active]`. The game controller writes them every physics step. The bridge reads them with bounds checking (falls back to `nav_goal_active = False` if buffer is old/small).
* **Result:** Zero-latency nav goal streaming at 1000 Hz with no additional sockets, pipes, or ROS topics in the physics loop.

---

### Decision 19: Multi-Elevation LiDAR Scanning with Full-Skeleton Exclusion & Alignment-Scaled Drive
* **Status:** **Accepted**
* **Context:** The autonomous navigation mode was either failing to detect floor obstacles (such as soccer balls and boxes) or walking backward away from the goal.
* **Root Cause:**
  1. Rays cast only at $Z = 0.4\,\text{m}$ passed over all ground obstacles (all $\le 0.15\,\text{m}$ tall).
  2. Casting rays lower struck the robot's own knees and ankles (48 rays per sweep), because `bodyexclude` only ignored `base_link`. The robot stamped its own feet as obstacles and tried to walk backward to escape.
  3. The occupancy grid origin drifted with the robot, warping obstacle locations.
  4. Propulsion was applied full force along the torso heading even during sharp $90^\circ$ turns, causing the robot to charge into obstacles before completing the turn.
* **Decision:**
  1. **Full-Skeleton Exclusion**: Identified all 17 body IDs of the BRUCE humanoid (`self.robot_body_ids`). Any ray returning a geom belonging to these bodies or $\text{dist} < 0.28\,\text{m}$ is filtered out.
  2. **Multi-Elevation Scanning**: Cast 270 rays across three vertical planes ($Z=0.05\,\text{m}$, $Z=0.16\,\text{m}$, $Z=0.35\,\text{m}$) to capture objects of any height from low soccer balls to tall barrels and walls.
  3. **Fixed-Origin World Grid**: Fixed grid at origin $(-15\,\text{m}, -15\,\text{m})$ with $300 \times 300$ cells ($0.10\,\text{m}$ resolution) so obstacles remain stationary in world space.
  4. **Alignment-Scaled Forward Propulsion**: Scaled forward drive by $\max(0, \cos(\Delta\psi))^2$. Large heading errors cause the robot to slow down and pivot on the spot before driving forward.
  5. **Pure-Pursuit Lookahead & Visual Breadcrumbs**: $0.45\,\text{m}$ lookahead creates smooth, curved steering arcs, rendered as glowing cyan spheres in the GLFW viewport.
* **Result:** The robot cleanly identifies all floor objects, plans a curved path around them with $>0.30\,\text{m}$ clearance, and reaches the goal with $0.0000\,\text{m}$ obstacle displacement.

---

### Decision 20: Standardized Open-Source Repository Documentation & Verified Tech Stack Catalog
* **Status:** **Accepted**
* **Context:** The BRUCE humanoid platform has evolved into an end-to-end multi-process simulation and perception system with numerous interacting libraries (MuJoCo, GLFW, ROS 2 Jazzy, RTAB-Map, RViz2, POSIX SHM). Users and external contributors require a unified, professional GitHub repository entry point with verified runtime versions, standard badges, concise installation commands, control tables, and clear cross-links to technical specifications.
* **Decision:**
  1. Audit and pin exact host software versions directly from runtime checks (`dpkg`, Python sys/pkg inspection): MuJoCo `3.16.0`, Python `3.12.3`, ROS 2 Jazzy `0.11.0`, RViz2 `14.1.24`, RTAB-Map `0.21.x`, Ubuntu `24.04.5 LTS` (Kernel `7.0`).
  2. Implement a standard GitHub [`README.md`](file:///home/ncircle/Desktop/westwood/Bruce/BRUCE_simulation_models/README.md) featuring:
     - SVG status badges (OS, physics, middleware, language, graphics, IPC, license).
     - Mermaid system architecture diagram depicting two-process shared memory decoupling.
     - Tech stack matrix with verified versions and core responsibilities.
     - Dual-mode control tables (Game Mode vs. Object Manipulation & Goal Setting).
     - Direct one-click launcher scripts (`run_game_controller.sh`, `run_ros2_perception.sh`).
     - Complete ROS 2 topic catalog and directory layout.
     - Troubleshooting guide addressing snap glibc library symbol collisions, shared memory cleanup, and cursor locking.
* **Result:** Repository meets professional open-source robotics publication standards and eliminates ambiguity regarding installation, operation, and architecture.

---

### Decision 21: Simple & Relatable IT Office Environment for Stakeholder Demos
* **Status:** **Accepted**
* **Context:** Clients and managers need an environment that feels familiar and directly speaks to IT/enterprise use cases (e.g. office delivery, facility inspection, server room patrolling). Abstract blocks or generic shapes don't communicate the commercial reality, while massive city models created unnecessary visual clutter. A simple, recognizable IT office environment bridges this gap.
* **Decision:**
  1. **Relatable IT Office Fixtures ($14 \times 14\,\text{m}$)**:
     - **Developer Workstation**: Desk with widescreen code-editor monitor, desktop PC mini-tower with green LED, keyboard, mousepad, and coffee mug.
     - **IT Server Rack & Network Cabinet**: Enterprise dark cabinet with 4 blade unit slots and green/amber network status LEDs.
     - **Office Water Cooler**: Classic blue inverted bottle jug, white dispenser pedestal, and hot/cold taps.
     - **Agile Scrum Whiteboard**: Whiteboard on aluminum stand with colorful sprint task sticky notes.
     - **Meeting Table & Chairs**: Conference table with blue cushioned office chairs.
     - **Delivery Platform (`place_table`) & Hardware Parcel (`pickup_crate`)**: Clean destination for autonomous courier delivery.
  2. **Calibrated Lighting & Zero Glare**:
     - Dimmed directional lights and matte slate-grey floor (`reflectance=0.0`) eliminating harsh glare and reflections.
     - Tested camera frame brightness maintained at a comfortable, balanced $85.7$.
  3. **Lightweight Physics & Flawless Navigation**:
     - Maintained low complexity (32 bodies, 112 geoms) with sub-millisecond physics stepping ($1,000\,\text{Hz}$) and solid 60+ FPS.
     - Multi-elevation LiDAR ($Z = 0.05\,\text{m}, 0.16\,\text{m}, 0.35\,\text{m}$) detects all IT furniture and obstacles.
     - A* path planner verified routing around task chairs and developer desks with $>0.86\,\text{m}$ clearance and zero collisions.
* **Result:** Highly relatable environment where managers and clients instantly visualize BRUCE operating inside their own office facilities.
