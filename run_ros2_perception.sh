#!/usr/bin/env bash
# ==============================================================================
# BRUCE Humanoid - ROS 2 Perception, LiDAR, & SLAM Visualizer Launcher
# ==============================================================================
# Usage:
#   ./run_ros2_perception.sh
#
# Requirements:
#   Make sure the 3rd-person game simulation is already running:
#     ./run_game_controller.sh
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="/home/ncircle/Desktop/mojokui/.venv/bin/python"

# Clean up snap environment pollution that breaks native GUI apps (e.g. RViz2 / Qt)
unset GTK_PATH
unset GTK_MODULES
unset LOCPATH
unset GIO_MODULE_DIR

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
else
    echo "[ERROR] ROS 2 Jazzy setup.bash not found in /opt/ros/jazzy!"
    exit 1
fi

if [ ! -f "$PYTHON_BIN" ]; then
    PYTHON_BIN="python3"
fi

echo "======================================================================"
echo "  📡 STARTING BRUCE ROS 2 PERCEPTION BRIDGE & RVIZ2 (ROS 2 JAZZY) 📡"
echo "======================================================================"
echo "Topics Published:"
echo "  • /bruce/camera/image_raw       (RGB Camera, 30 Hz)"
echo "  • /bruce/camera/depth/image_raw (Depth Camera, 30 Hz)"
echo "  • /bruce/camera/camera_info     (Camera Intrinsics)"
echo "  • /bruce/lidar/point_cloud      (360° 3D LiDAR PointCloud2, 15 Hz)"
echo "  • /bruce/imu/data               (6-Axis IMU, 50 Hz)"
echo "  • /bruce/odom                   (Odometry & TF tree)"
echo "======================================================================"

# Check for optional SLAM flag
ENABLE_SLAM=false
for arg in "$@"; do
    if [ "$arg" == "--slam" ]; then
        ENABLE_SLAM=true
    fi
done

# 1. Start ROS 2 Perception Bridge Node in Background
cd "$SCRIPT_DIR/MuJoCo"
"$PYTHON_BIN" bruce_ros2_bridge.py &
BRIDGE_PID=$!

SLAM_PID=""
# Optional: Launch RTAB-Map SLAM if requested
if [ "$ENABLE_SLAM" = true ]; then
    if ros2 pkg list | grep -q "rtabmap_launch"; then
        echo "Starting RTAB-Map SLAM Node..."
        ros2 launch rtabmap_launch rtabmap.launch.py \
            rgb_topic:=/bruce/camera/image_raw \
            depth_topic:=/bruce/camera/depth/image_raw \
            camera_info_topic:=/bruce/camera/camera_info \
            scan_cloud_topic:=/bruce/lidar/point_cloud \
            odom_topic:=/bruce/odom \
            frame_id:=base_link \
            approx_sync:=true &
        SLAM_PID=$!
    else
        echo "[INFO] To enable RTAB-Map SLAM, install the ROS 2 Jazzy package:"
        echo "       sudo apt install ros-jazzy-rtabmap-launch ros-jazzy-rtabmap-ros"
    fi
fi

# Ensure Bridge & SLAM are terminated on script exit
cleanup() {
    echo ""
    echo "Stopping ROS 2 perception bridge (PID: $BRIDGE_PID)..."
    kill -INT "$BRIDGE_PID" 2>/dev/null
    if [ -n "$SLAM_PID" ]; then
        kill -INT "$SLAM_PID" 2>/dev/null
    fi
    sleep 0.3
    kill -TERM "$BRIDGE_PID" 2>/dev/null
    wait "$BRIDGE_PID" 2>/dev/null
    echo "Perception bridge stopped."
}
trap cleanup EXIT INT TERM

sleep 1.5

# 2. Launch RViz2 with pre-configured BRUCE SLAM display
cd "$SCRIPT_DIR"
if command -v rviz2 &> /dev/null; then
    echo "Launching RViz2..."
    rviz2 -d bruce_slam.rviz
else
    echo "[WARN] rviz2 command not found. Perception bridge is running in background."
    echo "Press Ctrl+C to terminate."
    wait "$BRIDGE_PID"
fi
