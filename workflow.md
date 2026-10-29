# Workflow & User Guide: BRUCE Character Controller

This document explains how to launch, operate, test, and troubleshoot the third-person GTA-style character controller for the BRUCE humanoid robot.

---

## 1. Quick Launch

Open a terminal and run the launcher script from the project:

```bash
cd /home/ncircle/Desktop/westwood/Bruce/BRUCE_simulation_models
./run_game_controller.sh
```

Alternatively, you can run the Python script directly:

```bash
cd /home/ncircle/Desktop/westwood/Bruce/BRUCE_simulation_models/MuJoCo
/home/ncircle/Desktop/mojokui/.venv/bin/python bruce_game_controller.py
```

---

## 2. Controls & Keymapping Reference

The controller behaves like an action-adventure game (e.g., GTA 5, Assassin's Creed):

### Locomotion & Posture
| Key / Input | Action | Description |
| :--- | :--- | :--- |
| **`W`** | **Walk Forward** | Moves in the exact direction the camera is facing |
| **`S`** | **Walk Backward** | Steps backward away from camera view |
| **`A`** | **Strafe Left** | Steps laterally left relative to camera |
| **`D`** | **Strafe Right** | Steps laterally right relative to camera |
| **`Shift` + `WASD`** | **Sprint / Run** | Accelerates stepping frequency to $1.6\,\text{Hz}$ ($0.48\,\text{m/s}$) with forward torso lean |
| **`Ctrl` (Hold)** | **Crouch / Sit** | Flexes knees and hips, lowering center of mass to $0.31\,\text{m}$ |
| **`Ctrl` + `WASD`** | **Crouch-Walk** | Low-profile stealth walk while staying crouched |
| **`Space`** | **Jump** | 4-phase dynamic vertical jump with landing compliance |

### Camera Orbit & View
| Input | Action | Description |
| :--- | :--- | :--- |
| **Mouse Move** | **Orbit Camera** | $360^\circ$ horizontal yaw and $-75^\circ$ to $+20^\circ$ pitch |
| **Mouse Wheel** | **Camera Zoom** | Zooms follow distance in/out ($0.8\,\text{m}$ to $5.0\,\text{m}$) |
| **`Tab` / `Esc`** | **Toggle Mouse Lock** | Releases cursor so you can click outside the window; click window to re-lock |

### Physical Interactions & Gestures
| Input | Action | Description |
| :--- | :--- | :--- |
| **Left Click** or **`F`** | **Kick Soccer Ball** | 3-phase dynamic leg swing to strike the soccer ball forward |
| **Right Click** or **`E`** | **Push Stance** | Drives forward with natural outstretched pushing arms to push boxes/obstacles |
| **`G`** | **Wave Arm** | Friendly greeting wave gesture with right arm while left arm rests relaxed |
| **`R`** | **Reset Simulation** | Respawns BRUCE upright and resets the ball, boxes, and obstacles |

### Upper-Body Biomechanics & Humanoid Silhouette
- **Relaxed Human Posture:** Arms hang naturally down along the ribcage sides with a gentle forward elbow bend ($\sim 28^\circ$), eliminating the unnatural horizontal T-pose.
- **Organic Idle Breathing:** In Stand mode, subtle $0.45\,\text{Hz}$ breathing oscillations keep the robot looking organic and alive.
- **Dynamic Arm Swing:** In Walk and Sprint modes, arms swing smoothly opposite to legs with phase-coupled elbow bending (bending up to $\sim 45^\circ$ on forward swings, relaxing on backswings).
- **Humanoid Aesthetic Styling:** Features a contoured upper chest armor plate, rounded head, and glowing cyan sensor visor.

---

## 3. Physical Interaction Guide

The arena ([`interactive_scene.xml`](file:///home/ncircle/Desktop/westwood/Bruce/BRUCE_simulation_models/MuJoCo/interactive_scene.xml)) contains several interactive physics objects spaced out along the forward runway:

1. **Dynamic Soccer Ball (Red Sphere):**
   - Positioned $1.25\,\text{m}$ in front of BRUCE.
   - Walk up to the ball with `W` and **Left Click** (or press `F`) to launch it across the arena floor!
2. **Dynamic Target Box (Blue Cube):**
   - Located at $1.90\,\text{m}$ in front, slightly to the right.
   - Walk up to the box, press **Right Click** (or `E`) to push it along the ground.
3. **Obstacle Step Block (Yellow Slab):**
   - Placed at $2.60\,\text{m}$ down the runway to test foot clearance and jumping.
   - Run toward it and press **`Space`** to leap over it.
4. **Respawning:**
   - If an object gets kicked too far, press **`R`** to instantly teleport the robot, ball, and obstacles back to their starting positions.

---

## 4. Verification & Testing Checklist

To verify that all components are functioning as expected, run this quick check:

- [x] **Headless Test:** Run `python -c "from bruce_game_controller import BruceGameController; c = BruceGameController(); c.step_physics()"` $\rightarrow$ Ensure no NaNs.
- [x] **Locomotion Verification:** Hold `W` $\rightarrow$ Robot walks forward along camera view.
- [x] **Sprint Verification:** Hold `Shift` + `W` $\rightarrow$ Robot visibly accelerates and swings arms with wider strides.
- [x] **Crouch Verification:** Hold `Ctrl` $\rightarrow$ Robot sits into deep knee bend.
- [x] **Jump Verification:** Tap `Space` $\rightarrow$ Robot crouches, leaps into the air, and sticks landing upright.
- [x] **Kick Verification:** Click Left Mouse Button $\rightarrow$ Leg kicks dynamic ball forward.
- [x] **Mouse Orbit:** Move mouse $\rightarrow$ Camera smoothly orbits around BRUCE.
- [x] **Mouse Release:** Press `Tab` $\rightarrow$ Cursor is freed; click into window to re-capture.

---

---

## 5. Dual-Window Perception & SLAM Workflow

BRUCE features a high-performance decoupled dual-window setup. You can run the interactive game controller in one window and live RViz2 perception (RGB-D Camera + 3D LiDAR + Odometry + SLAM) in a second window simultaneously.

```
+------------------------------------+          POSIX Shared Memory          +--------------------------------------+
|     WINDOW 1: GAME CONTROLLER      |         (bruce_sim_state)             |     WINDOW 2: ROS 2 PERCEPTION       |
|  - 60+ FPS MuJoCo Interactive      | ====================================> |  - Offscreen RGB-D Camera (15 Hz)    |
|  - GTA 5 3rd-Person Camera         |   [qpos, pos, quat, vel, omega, time]  |  - 360° Multi-Beam LiDAR (15 Hz)     |
|  - WASD, Mouse Look, Jump, Kick    |                                       |  - RViz2 3D PointCloud & SLAM Stream |
+------------------------------------+                                       +--------------------------------------+
```

### Launch Instructions

#### Step 1: Launch Window 1 (Interactive Game Controller)
In your first terminal:
```bash
cd /home/ncircle/Desktop/westwood/Bruce/BRUCE_simulation_models
./run_game_controller.sh
```
- Click into the window to lock the mouse cursor.
- Use `WASD` to move, mouse to look, `Shift` to sprint, `Ctrl` to crouch, `Space` to jump.

#### Step 2: Launch Window 2 (ROS 2 Perception & RViz2)
In your second terminal:
```bash
cd /home/ncircle/Desktop/westwood/Bruce/BRUCE_simulation_models
./run_ros2_perception.sh
```
- This script automatically attaches to the shared memory buffer from Window 1, starts the ROS 2 sensor publishers, and launches RViz2 pre-configured with PointCloud2, Camera Image, and Odometry displays.

#### Optional: Launching with RTAB-Map SLAM
To run 3D SLAM mapping concurrently:
```bash
cd /home/ncircle/Desktop/westwood/Bruce/BRUCE_simulation_models
./run_ros2_perception.sh --slam
```
As you walk BRUCE through the arena corridors and rooms in Window 1, RTAB-Map builds a 3D point cloud map and 2D occupancy grid map live in RViz2.

---

## 6. ROS 2 Sensor Topics & Specifications

| Topic Name | Message Type | Rate | Description |
| :--- | :--- | :--- | :--- |
| `/bruce/camera/image_raw` | `sensor_msgs/msg/Image` | $15\,\text{Hz}$ | $640 \times 480$ RGB color image from torso camera |
| `/bruce/camera/depth/image_raw` | `sensor_msgs/msg/Image` | $15\,\text{Hz}$ | $640 \times 480$ 32-bit floating point metric depth |
| `/bruce/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | $15\,\text{Hz}$ | Calibrated camera intrinsic parameters |
| `/bruce/lidar/point_cloud` | `sensor_msgs/msg/PointCloud2` | $15\,\text{Hz}$ | $360^\circ$ 3D LiDAR point cloud (16 rings $\times$ 72 rays = 1,152 rays) |
| `/bruce/imu/data` | `sensor_msgs/msg/Imu` | $50\,\text{Hz}$ | Torso linear acceleration and angular velocity |
| `/bruce/odom` | `nav_msgs/msg/Odometry` | $50\,\text{Hz}$ | Robot ground-truth position, orientation, and twist |
| `/tf` / `/tf_static` | `tf2_msgs/msg/TFMessage` | $50\,\text{Hz}$ | Coordinate tree (`odom` $\to$ `base_link` $\to$ `head_camera_optical_frame` / `lidar_link`) |

---

## 7. Troubleshooting

| Issue | Cause | Solution |
| :--- | :--- | :--- |
| **Window does not open (`Cannot connect to display`)** | Remote SSH session without X11 forwarding | Ensure `DISPLAY=:0` or run from a local desktop terminal. |
| **Mouse doesn't orbit camera** | Mouse cursor is currently in "Free" mode | Click anywhere inside the window or press `Tab` to lock the cursor. |
| **Robot stumbles on jump landing** | Landing on dynamic block edge | Jump on flat floor or press `R` to reset upright pose. |
| **Camera distance too close/far** | Scroll wheel offset | Scroll the mouse wheel to adjust camera distance smoothly between $0.8\,\text{m}$ and $5.0\,\text{m}$. |
| **ROS 2 bridge: `Shared memory not found`** | Game controller not running yet | Start `./run_game_controller.sh` first in Terminal 1, then run `./run_ros2_perception.sh` in Terminal 2. |
| **RViz2 displays "No tf data"** | Bridge node still initializing | Wait $1\dots 2$ seconds for static transforms to broadcast, or verify bridge is running without error. |
| **`undefined symbol: __libc_pthread_init`** | Terminal inherited Snap environment variables (VS Code Snap) | Fixed automatically in launcher; manually run `unset GTK_PATH GTK_MODULES LOCPATH GIO_MODULE_DIR`. |
| **Stale shared memory block** | Previous run terminated abnormally | Clean up with `python3 -c "from multiprocessing import shared_memory; m = shared_memory.SharedMemory('bruce_sim_state'); m.close(); m.unlink()"` |

---

## 8. Mouse Object Manipulation & Autonomous Navigation

### New Keys & Mouse Controls

| Input | Mode | Action |
|---|---|---|
| **`M`** | Any | Toggle **Manipulation Mode** (cursor free) / **Game Mode** (cursor locked) |
| **Left-click + drag** | Manipulation | Grab and drag any dynamic object (ball, crates, barrel, block) across the floor |
| **Double-click floor** | Manipulation | Plant **Navigation Goal Marker** (glowing pulsing green sphere) |
| **Right-click** | Manipulation | Clear navigation goal |
| **`N`** | Any | Toggle autonomous navigation ON/OFF (requires goal to be set first) |

### Quick Navigation Workflow

```
Step 1: Press M            → cursor unlocks (Manipulation Mode)
Step 2: Double-click floor → green goal marker appears
Step 3: Press M again      → cursor re-locks (Game Mode)
Step 4: Press N            → BRUCE autonomously walks to goal
         WASD at any time → overrides nav and resumes manual control
         N again           → pauses navigation
```

### What Happens During Autonomous Navigation
1. Every ~20 ms, 72 horizontal LiDAR rays sweep 360° from the robot torso.
2. Hits mark cells in a 200×200 decaying occupancy grid (0.15 m/cell).
3. Every 200 steps (~0.2 s), A* replans a path from current robot position to goal, inflating obstacles by 2 cells (0.30 m robot radius).
4. Waypoints pop as the robot comes within 0.25 m of each.
5. On reaching within 0.30 m of the goal, robot stops and returns to STAND mode.

### Object Dragging Tips
- In Manipulation Mode you can **stack objects** by dragging one on top of another — MuJoCo physics handles the stacking dynamics.
- Drag the soccer ball into the robot's path before pressing `N` to test obstacle avoidance replanning.
- The goal marker glows brighter when `N` navigation is active.

---

## 9. ROS 2 Navigation Topics (New)

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/bruce/nav/goal` | `geometry_msgs/PoseStamped` | 5 Hz | Current nav goal position in `odom` frame |
| `/bruce/nav/path` | `nav_msgs/Path` | 5 Hz | Straight-line visualization path to goal |
| `/bruce/nav/occupancy` | `nav_msgs/OccupancyGrid` | 2 Hz | 100×100 local occupancy grid (0.15 m/cell) built from LiDAR |

These topics appear automatically in RViz2 when `run_ros2_perception.sh` is running alongside the game controller.

### RViz2 Additions for Nav Visualization
In RViz2 add the following displays:
- **Displays → Add → By Topic → `/bruce/nav/goal` → Pose** (shows goal arrow)
- **Displays → Add → By Topic → `/bruce/nav/path` → Path** (shows path line)
- **Displays → Add → By Topic → `/bruce/nav/occupancy` → Map** (shows 2D cost map)
