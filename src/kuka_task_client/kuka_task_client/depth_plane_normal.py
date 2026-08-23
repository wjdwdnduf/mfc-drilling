"""Estimate the workpiece surface plane from the RGB-D stream.

Fits a plane to the depth point cloud with RANSAC and broadcasts its normal as
a TF frame (``plane_frame``), so the tool can be aligned perpendicular to the
surface.

Notes on this revision:
  * Topics are parameters. The previous defaults (/camera/camera/...) predate
    the two-camera setup and matched nothing once the cameras moved to the
    camera_1 / camera_2 namespaces.
  * The point cloud is built with vectorised numpy and an optional pixel
    stride. The original nested Python loop ran ~307k iterations per frame,
    which could not keep up with the camera.
"""

import numpy as np
import open3d as o3d
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image


class DepthPlaneNormal(Node):
    def __init__(self):
        super().__init__('depth_plane_normal')

        self.declare_parameter('depth_topic', '/camera/camera_1/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_1/depth/camera_info')
        self.declare_parameter('parent_frame', 'drill_link')
        self.declare_parameter('plane_frame', 'plane_frame')
        self.declare_parameter('ransac_distance_threshold', 0.01)
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('ransac_iterations', 1000)
        self.declare_parameter('pixel_stride', 4)
        self.declare_parameter('publish_period', 0.1)
        self.declare_parameter('min_points', 100)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.plane_frame = self.get_parameter('plane_frame').value
        self.ransac_distance_threshold = float(
            self.get_parameter('ransac_distance_threshold').value
        )
        self.ransac_n = int(self.get_parameter('ransac_n').value)
        self.ransac_iterations = int(self.get_parameter('ransac_iterations').value)
        self.pixel_stride = max(1, int(self.get_parameter('pixel_stride').value))
        self.min_points = int(self.get_parameter('min_points').value)

        self.bridge = CvBridge()
        self.camera_info = None
        self.last_plane_model = None
        self._warned_no_info = False

        self.depth_subscription = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, 10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(
            float(self.get_parameter('publish_period').value), self.timer_callback
        )

        self.get_logger().info(
            f"Plane estimation running on {self.depth_topic} "
            f"(stride={self.pixel_stride}) -> {self.parent_frame}/{self.plane_frame}"
        )

    # ------------------------------------------------------------------
    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        if self.camera_info is None:
            if not self._warned_no_info:
                self._warned_no_info = True
                self.get_logger().warn(
                    f"Depth frames arriving but no CameraInfo on "
                    f"{self.camera_info_topic} yet."
                )
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        point_cloud = self.depth_to_point_cloud(depth_image, self.camera_info)

        if len(point_cloud) < self.min_points:
            self.get_logger().warn(
                f"Only {len(point_cloud)} valid depth points "
                f"(need {self.min_points}); skipping plane fit."
            )
            return

        plane_model, inlier_cloud = self.find_plane_from_point_cloud(point_cloud)
        self.get_logger().debug(
            f"Plane: {plane_model}, inliers: {len(inlier_cloud.points)}"
        )
        self.last_plane_model = plane_model
        self.publish_tf(plane_model)

    # ------------------------------------------------------------------
    def depth_to_point_cloud(self, depth_image, camera_info):
        """Vectorised back-projection of the depth image into camera 3D."""
        fx, fy = camera_info.k[0], camera_info.k[4]
        cx, cy = camera_info.k[2], camera_info.k[5]

        s = self.pixel_stride
        depth = np.asarray(depth_image, dtype=np.float32)[::s, ::s] / 1000.0

        height, width = depth.shape
        us = (np.arange(width, dtype=np.float32) * s)
        vs = (np.arange(height, dtype=np.float32) * s)
        u_grid, v_grid = np.meshgrid(us, vs)

        valid = np.isfinite(depth) & (depth > 0)
        z = depth[valid]
        x = (u_grid[valid] - cx) * z / fx
        y = (v_grid[valid] - cy) * z / fy
        return np.stack([x, y, z], axis=-1)

    def find_plane_from_point_cloud(self, point_cloud):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.ransac_distance_threshold,
            ransac_n=self.ransac_n,
            num_iterations=self.ransac_iterations,
        )
        return plane_model, pcd.select_by_index(inliers)

    # ------------------------------------------------------------------
    def publish_tf(self, plane_model):
        normal = np.array(plane_model[:3], dtype=float)
        normal_unit = normal / np.linalg.norm(normal)
        d = float(plane_model[3])

        # Keep the normal pointing away from the camera.
        if normal_unit[2] < 0:
            normal_unit = -normal_unit
            d = -d

        # Build an orthonormal frame with the plane normal as Z.
        reference = np.array([1.0, 0.0, 0.0])
        if np.allclose(np.abs(normal_unit), reference):
            reference = np.array([0.0, 1.0, 0.0])

        x_axis = np.cross(reference, normal_unit)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(normal_unit, x_axis)

        rotation_matrix = np.vstack([x_axis, y_axis, normal_unit]).T
        # 90 deg about Z aligns the frame with the tool's convention.
        rotation_matrix = rotation_matrix @ R.from_euler('z', 90, degrees=True).as_matrix()
        quaternion = R.from_matrix(rotation_matrix).as_quat()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.plane_frame
        t.transform.translation.x = float(normal_unit[0] * d)
        t.transform.translation.y = float(normal_unit[1] * d)
        t.transform.translation.z = float(normal_unit[2] * d)
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        """Keep broadcasting the last fit so the TF does not go stale."""
        if self.last_plane_model is not None:
            self.publish_tf(self.last_plane_model)


def main(args=None):
    rclpy.init(args=args)
    node = DepthPlaneNormal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
