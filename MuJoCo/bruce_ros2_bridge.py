#!/usr/bin/env python3
"""
================================================================================
BRUCE Humanoid Robot - ROS 2 Perception & SLAM Bridge (ROS 2 Jazzy)
================================================================================
Streams simulated sensor data from MuJoCo to standard ROS 2 perception topics:
  • /bruce/camera/image_raw       (sensor_msgs/msg/Image - RGB8, 30 Hz)
  • /bruce/camera/depth/image_raw (sensor_msgs/msg/Image - 32FC1, 30 Hz)
  • /bruce/camera/camera_info     (sensor_msgs/msg/CameraInfo, 30 Hz)
  • /bruce/lidar/point_cloud      (sensor_msgs/msg/PointCloud2 - 360° 3D LiDAR, 15 Hz)
  • /bruce/imu/data               (sensor_msgs/msg/Imu, 50 Hz)
  • /bruce/odom                   (nav_msgs/msg/Odometry, 50 Hz)
  • /tf & /tf_static              (tf2_msgs/msg/TFMessage)
================================================================================
"""

import os
import sys
import time
import math
import numpy as np
from multiprocessing import shared_memory

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Point, PoseStamped
import tf2_ros
import sensor_msgs_py.point_cloud2 as pc2

# MuJoCo
import mujoco

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_XML = os.path.join(SCRIPT_DIR, "interactive_scene.xml")


class BruceRos2Bridge(Node):
    def __init__(self):
        super().__init__("bruce_ros2_bridge")

        self.get_logger().info("Initializing BRUCE ROS 2 Perception Bridge...")

        # Load MuJoCo model for off-screen perception rendering
        if not os.path.exists(SCENE_XML):
            self.get_logger().error(f"Scene XML not found: {SCENE_XML}")
            raise FileNotFoundError(SCENE_XML)

        self.model = mujoco.MjModel.from_xml_path(SCENE_XML)
        self.data = mujoco.MjData(self.model)

        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
        self.lidar_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "lidar_site")

        # Camera Intrinsics Configuration (640x480, fovy=65°)
        self.cam_width = 640
        self.cam_height = 480
        self.cam_fovy = 65.0  # degrees
        focal_y = (self.cam_height / 2.0) / math.tan(math.radians(self.cam_fovy / 2.0))
        focal_x = focal_y
        cx = self.cam_width / 2.0
        cy = self.cam_height / 2.0

        self.camera_info = CameraInfo()
        self.camera_info.width = self.cam_width
        self.camera_info.height = self.cam_height
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.k = [
            focal_x, 0.0, cx,
            0.0, focal_y, cy,
            0.0, 0.0, 1.0
        ]
        self.camera_info.p = [
            focal_x, 0.0, cx, 0.0,
            0.0, focal_y, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Offscreen MuJoCo Renderer
        self.renderer = mujoco.Renderer(self.model, height=self.cam_height, width=self.cam_width)

        # 360° 3D LiDAR Configuration (16 elevation rings x 72 horizontal beams = 1152 rays)
        self.num_horiz = 72
        self.num_vert = 16
        self.elevations = np.linspace(np.radians(-16), np.radians(16), self.num_vert)
        self.azimuths = np.linspace(0, 2 * np.pi, self.num_horiz, endpoint=False)
        self.lidar_ray_cache = []
        for el in self.elevations:
            cos_el, sin_el = np.cos(el), np.sin(el)
            for az in self.azimuths:
                self.lidar_ray_cache.append(
                    np.array([cos_el * np.cos(az), cos_el * np.sin(az), sin_el], dtype=np.float64)
                )

        # Connect to Simulation Shared Memory
        self.shm = None
        self.shm_buf = None
        self._connect_shared_memory()

        # Nav goal state
        self.nav_goal_active = False
        self.nav_goal_x = 0.0
        self.nav_goal_y = 0.0
        self.last_lidar_world_points = []

        # ROS 2 Publishers with SensorData QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.pub_image_rgb = self.create_publisher(Image, "/bruce/camera/image_raw", sensor_qos)
        self.pub_image_depth = self.create_publisher(Image, "/bruce/camera/depth/image_raw", sensor_qos)
        self.pub_cam_info = self.create_publisher(CameraInfo, "/bruce/camera/camera_info", sensor_qos)
        self.pub_pointcloud = self.create_publisher(PointCloud2, "/bruce/lidar/point_cloud", sensor_qos)
        self.pub_imu = self.create_publisher(Imu, "/bruce/imu/data", sensor_qos)
        self.pub_odom = self.create_publisher(Odometry, "/bruce/odom", 10)

        # Nav publishers
        self.pub_nav_goal = self.create_publisher(PoseStamped, "/bruce/nav/goal", 10)
        self.pub_nav_path = self.create_publisher(Path, "/bruce/nav/path", 10)
        self.pub_occupancy = self.create_publisher(OccupancyGrid, "/bruce/nav/occupancy", 10)

        # TF Broadcasters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        # Timers
        # 1. High-rate IMU & Odometry: 50 Hz
        self.timer_odom = self.create_timer(1.0 / 50.0, self.update_and_publish_odometry)
        # 2. Camera RGB-D stream: 30 Hz
        self.timer_camera = self.create_timer(1.0 / 30.0, self.render_and_publish_camera)
        # 3. 360° 3D LiDAR PointCloud: 15 Hz
        self.timer_lidar = self.create_timer(1.0 / 15.0, self.scan_and_publish_lidar)
        # 4. Nav goal & path: 5 Hz
        self.timer_nav = self.create_timer(1.0 / 5.0, self.publish_nav_topics)
        # 5. Occupancy grid: 2 Hz
        self.timer_occ = self.create_timer(1.0 / 2.0, self.publish_occupancy_grid)

        self.get_logger().info("BRUCE Perception Bridge successfully online!")

    def _connect_shared_memory(self):
        """Attempts to attach to the live MuJoCo simulation shared memory segment."""
        shm_name = "bruce_sim_state"
        try:
            self.shm = shared_memory.SharedMemory(name=shm_name)
            # Unregister from multiprocessing resource_tracker to prevent warnings/unlinking
            try:
                from multiprocessing.resource_tracker import unregister
                unregister(self.shm._name, 'shared_memory')
            except Exception:
                pass
            total_doubles = 3 + self.model.nq + self.model.nv
            self.shm_buf = np.ndarray((total_doubles,), dtype=np.float64, buffer=self.shm.buf)
            self.get_logger().info("Connected to live simulation shared memory buffer 'bruce_sim_state'.")
        except FileNotFoundError:
            self.get_logger().warn(
                "Shared memory 'bruce_sim_state' not found. "
                "Make sure 'run_game_controller.sh' is running! Waiting for connection..."
            )

    def destroy_node(self):
        """Closes shared memory attachment upon node destruction."""
        if self.shm is not None:
            try:
                self.shm.close()
            except Exception:
                pass
            self.shm = None
        super().destroy_node()

    def _sync_simulation_state(self):
        """Reads latest qpos/qvel from shared memory and updates MuJoCo kinematics."""
        if self.shm is None:
            self._connect_shared_memory()
            if self.shm is None:
                return False

        try:
            nq = self.model.nq
            nv = self.model.nv
            self.data.qpos[:] = self.shm_buf[3 : 3 + nq]
            self.data.qvel[:] = self.shm_buf[3 + nq : 3 + nq + nv]
            mujoco.mj_forward(self.model, self.data)
            # Read nav goal fields if present
            nav_offset = 3 + nq + nv
            if len(self.shm_buf) > nav_offset + 2:
                self.nav_goal_x = float(self.shm_buf[nav_offset])
                self.nav_goal_y = float(self.shm_buf[nav_offset + 1])
                self.nav_goal_active = float(self.shm_buf[nav_offset + 2]) > 0.5
            else:
                self.nav_goal_active = False
            return True
        except Exception as e:
            self.get_logger().warn(f"Shared memory read error: {e}")
            self.shm = None
            return False

    def _publish_static_transforms(self):
        """Broadcasts static TF transforms between robot frames."""
        now = self.get_clock().now().to_msg()
        transforms = []

        # 1. base_link -> head_camera
        tf_cam = TransformStamped()
        tf_cam.header.stamp = now
        tf_cam.header.frame_id = "base_link"
        tf_cam.child_frame_id = "head_camera"
        tf_cam.transform.translation.x = 0.08
        tf_cam.transform.translation.y = 0.0
        tf_cam.transform.translation.z = 0.16
        tf_cam.transform.rotation.w = 1.0
        transforms.append(tf_cam)

        # 2. head_camera -> head_camera_optical_frame (Standard ROS Optical: Z fwd, X right, Y down)
        tf_opt = TransformStamped()
        tf_opt.header.stamp = now
        tf_opt.header.frame_id = "head_camera"
        tf_opt.child_frame_id = "head_camera_optical_frame"
        # -90° yaw, +90° pitch
        tf_opt.transform.rotation.x = -0.5
        tf_opt.transform.rotation.y = 0.5
        tf_opt.transform.rotation.z = -0.5
        tf_opt.transform.rotation.w = 0.5
        transforms.append(tf_opt)

        # 3. base_link -> bruce_lidar_link
        tf_lidar = TransformStamped()
        tf_lidar.header.stamp = now
        tf_lidar.header.frame_id = "base_link"
        tf_lidar.child_frame_id = "bruce_lidar_link"
        tf_lidar.transform.translation.x = 0.02
        tf_lidar.transform.translation.y = 0.0
        tf_lidar.transform.translation.z = 0.22
        tf_lidar.transform.rotation.w = 1.0
        transforms.append(tf_lidar)

        # 4. base_link -> bruce_imu_link
        tf_imu = TransformStamped()
        tf_imu.header.stamp = now
        tf_imu.header.frame_id = "base_link"
        tf_imu.child_frame_id = "bruce_imu_link"
        tf_imu.transform.translation.z = 0.05
        tf_imu.transform.rotation.w = 1.0
        transforms.append(tf_imu)

        self.static_tf_broadcaster.sendTransform(transforms)

    # =========================================================================
    # SENSOR PUBLISHING CALLBACKS
    # =========================================================================

    def update_and_publish_odometry(self):
        """Publishes Odometry, IMU, and dynamic TF (odom -> base_link)."""
        if not self._sync_simulation_state():
            return

        now = self.get_clock().now().to_msg()
        pos = self.data.xpos[self.base_body_id]
        quat_mujoco = self.data.xquat[self.base_body_id]  # [w, x, y, z]

        # 1. Dynamic TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        t.transform.rotation.w = float(quat_mujoco[0])
        t.transform.rotation.x = float(quat_mujoco[1])
        t.transform.rotation.y = float(quat_mujoco[2])
        t.transform.rotation.z = float(quat_mujoco[3])
        self.tf_broadcaster.sendTransform(t)

        # 2. Odometry Message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        odom.pose.pose.orientation = Quaternion(
            w=float(quat_mujoco[0]), x=float(quat_mujoco[1]),
            y=float(quat_mujoco[2]), z=float(quat_mujoco[3])
        )
        linear_v = self.data.qvel[0:3]
        angular_w = self.data.qvel[3:6]
        odom.twist.twist.linear = Vector3(x=float(linear_v[0]), y=float(linear_v[1]), z=float(linear_v[2]))
        odom.twist.twist.angular = Vector3(x=float(angular_w[0]), y=float(angular_w[1]), z=float(angular_w[2]))
        try:
            if rclpy.ok():
                self.pub_odom.publish(odom)
        except Exception:
            return

        # 3. IMU Message
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "bruce_imu_link"
        imu.orientation = odom.pose.pose.orientation
        # Angular rate in body frame
        imu.angular_velocity = Vector3(x=float(angular_w[0]), y=float(angular_w[1]), z=float(angular_w[2]))
        # Approximate linear acceleration + gravity
        imu.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        try:
            if rclpy.ok():
                self.pub_imu.publish(imu)
        except Exception:
            return

    def render_and_publish_camera(self):
        """Renders head RGB-D camera and publishes Image and CameraInfo topics."""
        if not rclpy.ok() or not self._sync_simulation_state():
            return

        now = self.get_clock().now().to_msg()

        # Update offscreen renderer scene
        self.renderer.update_scene(self.data, camera="head_camera")

        # 1. Render RGB
        rgb_img = self.renderer.render()
        img_msg = Image()
        img_msg.header.stamp = now
        img_msg.header.frame_id = "head_camera_optical_frame"
        img_msg.height = self.cam_height
        img_msg.width = self.cam_width
        img_msg.encoding = "rgb8"
        img_msg.is_bigendian = 0
        img_msg.step = self.cam_width * 3
        img_msg.data = rgb_img.tobytes()
        try:
            if rclpy.ok():
                self.pub_image_rgb.publish(img_msg)
        except Exception:
            return

        # 2. Render Depth
        self.renderer.enable_depth_rendering()
        depth_img = self.renderer.render()
        self.renderer.disable_depth_rendering()

        depth_msg = Image()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "head_camera_optical_frame"
        depth_msg.height = self.cam_height
        depth_msg.width = self.cam_width
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = 0
        depth_msg.step = self.cam_width * 4
        depth_msg.data = depth_img.astype(np.float32).tobytes()
        try:
            if rclpy.ok():
                self.pub_image_depth.publish(depth_msg)
        except Exception:
            return

        # 3. Camera Info
        self.camera_info.header.stamp = now
        self.camera_info.header.frame_id = "head_camera_optical_frame"
        try:
            if rclpy.ok():
                self.pub_cam_info.publish(self.camera_info)
        except Exception:
            pass

    def scan_and_publish_lidar(self):
        """Performs 360° 3D LiDAR raycasts and publishes PointCloud2 message."""
        if not rclpy.ok() or not self._sync_simulation_state():
            return

        now = self.get_clock().now().to_msg()
        lidar_world_pos = self.data.site_xpos[self.lidar_site_id]
        lidar_world_mat = self.data.site_xmat[self.lidar_site_id].reshape(3, 3)

        geom_id = np.zeros(1, dtype=np.int32)
        local_points = []
        self.last_lidar_world_points = []

        # Raycast all elevation and azimuth beams
        for vec_local in self.lidar_ray_cache:
            # Rotate ray into world frame for mj_ray query (excluding robot's own base_link)
            vec_world = lidar_world_mat @ vec_local
            dist = mujoco.mj_ray(self.model, self.data, lidar_world_pos, vec_world, None, 1, self.base_body_id, geom_id)
            if 0.15 < dist < 25.0:
                hit_world = lidar_world_pos + lidar_world_mat @ (vec_local * dist)
                self.last_lidar_world_points.append(hit_world)
                # Store hit point in sensor local frame
                local_points.append(vec_local * dist)

        if len(local_points) > 0 and rclpy.ok():
            header = Header()
            header.stamp = now
            header.frame_id = "bruce_lidar_link"

            pts_array = np.array(local_points, dtype=np.float32)
            cloud_msg = pc2.create_cloud_xyz32(header, pts_array)
            try:
                if rclpy.ok():
                    self.pub_pointcloud.publish(cloud_msg)
            except Exception:
                pass

    def publish_nav_topics(self):
        """Publishes nav goal (PoseStamped) and straight-line visualization path (Path) at 5 Hz."""
        if not rclpy.ok() or not self._sync_simulation_state():
            return

        now = self.get_clock().now().to_msg()

        if not self.nav_goal_active:
            return

        # Publish goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = now
        goal_msg.header.frame_id = "odom"
        goal_msg.pose.position.x = self.nav_goal_x
        goal_msg.pose.position.y = self.nav_goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        try:
            if rclpy.ok():
                self.pub_nav_goal.publish(goal_msg)
        except Exception:
            return

        # Publish straight-line path for visualization
        robot_pos = self.data.xpos[self.base_body_id]
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "odom"

        # 10 intermediate waypoints along straight line
        for i in range(11):
            alpha = i / 10.0
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(robot_pos[0] + alpha * (self.nav_goal_x - robot_pos[0]))
            ps.pose.position.y = float(robot_pos[1] + alpha * (self.nav_goal_y - robot_pos[1]))
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        try:
            if rclpy.ok():
                self.pub_nav_path.publish(path_msg)
        except Exception:
            pass

    def publish_occupancy_grid(self):
        """Builds and publishes a 2D OccupancyGrid from last LiDAR scan at 2 Hz."""
        if not rclpy.ok() or not self._sync_simulation_state():
            return
        if not self.last_lidar_world_points:
            return

        now = self.get_clock().now().to_msg()
        robot_pos = self.data.xpos[self.base_body_id]

        # 100x100 grid at 0.15m resolution, centered on robot
        grid_size = 100
        res = 0.15
        origin_x = robot_pos[0] - (grid_size * res / 2.0)
        origin_y = robot_pos[1] - (grid_size * res / 2.0)
        grid = np.zeros(grid_size * grid_size, dtype=np.int8)

        for pt in self.last_lidar_world_points:
            gx = int((pt[0] - origin_x) / res)
            gy = int((pt[1] - origin_y) / res)
            if 0 <= gx < grid_size and 0 <= gy < grid_size:
                grid[gy * grid_size + gx] = 100  # occupied

        occ_msg = OccupancyGrid()
        occ_msg.header.stamp = now
        occ_msg.header.frame_id = "odom"
        occ_msg.info.resolution = res
        occ_msg.info.width = grid_size
        occ_msg.info.height = grid_size
        occ_msg.info.origin.position.x = float(origin_x)
        occ_msg.info.origin.position.y = float(origin_y)
        occ_msg.info.origin.orientation.w = 1.0
        occ_msg.data = grid.tolist()

        try:
            if rclpy.ok():
                self.pub_occupancy.publish(occ_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    bridge = BruceRos2Bridge()
    try:
        rclpy.spin(bridge)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        pass
    finally:
        try:
            bridge.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()

