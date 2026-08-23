"""Command dispatcher for the MFC drilling workflow.

Subscribes to /kuka/command, executes the corresponding deterministic routine,
and reports the outcome on /kuka/status. The MCP function library only ever
publishes a command string here; all task logic lives in this node.

Every site-specific value (topics, frames, thresholds, distances) is a ROS
parameter — see kuka_eki/config/kuka_params.yaml.
"""

import os
import shutil
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from kuka_control_box_srvs.srv import KukaTransformInput


class KukaTopicMover(Node):
    def __init__(self):
        super().__init__('kuka_topic_mover')

        self._declare_parameters()
        self._load_parameters()

        # 1. IK 서비스 클라이언트
        self.cli = self.create_client(KukaTransformInput, 'kuka_transform_input')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for the kuka_transform_input service...')

        # 2. TF 관련 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. 명령 구독 (Topic 기반)
        self.subscription = self.create_subscription(
            String, '/kuka/command', self.command_callback, 10
        )

        # 4. 상태 보고용 퍼블리셔
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, '/kuka/status', status_qos)

        # 현재 작업 중인 workbench 추적
        self.current_workbench = None  # 'local_workbench1' / 'local_workbench2'

        # =============================================
        # 이미지 관련
        # =============================================
        self.bridge = CvBridge()
        self.latest_image_cam1 = None
        self.latest_image_cam2 = None
        os.makedirs(self.image_save_dir, exist_ok=True)

        self.image_sub_cam1 = self.create_subscription(
            Image, self.color_topic_cam1, self.image_callback_cam1, 10
        )
        self.image_sub_cam2 = self.create_subscription(
            Image, self.color_topic_cam2, self.image_callback_cam2, 10
        )

        # depth 이미지 구독 (픽셀 → 3D 변환용)
        self.latest_depth_cam1 = None
        self.depth_sub_cam1 = self.create_subscription(
            Image, self.depth_topic_cam1, self.depth_callback_cam1, 10
        )

        # 마킹 위치 리스트 (임시, save_point 전까지 누적)
        self.pending_marks = []  # [(x, y), ...]

        # 저장된 포인트 (base_link 기준, 태그 없이 이동 가능)
        self.saved_points = {}
        self.point_counter = 0

        # =============================================
        # 카메라 내부 파라미터
        #
        # Preferred source is the driver's CameraInfo topic; the parameter
        # values are only a fallback for bag playback or a simulator that
        # publishes no CameraInfo.
        # =============================================
        self.K = self._matrix_from_param('fallback_camera_matrix', (3, 3))
        self.dist_coeffs = np.array(
            self.get_parameter('fallback_distortion').value, dtype=np.float32
        )
        self.have_camera_info = False
        if self.get_parameter('use_camera_info').value:
            self.camera_info_sub = self.create_subscription(
                CameraInfo, self.camera_info_topic_cam1, self.camera_info_callback, 10
            )
            self.get_logger().info(
                f"Waiting for intrinsics on {self.camera_info_topic_cam1} "
                f"(using fallback values until then)."
            )

        self.get_logger().info("KUKA Topic Mover ready — listening on /kuka/command")

        self.current_status_msg = "STATUS: IDLE | DATA: Waiting for command"
        self.timer = self.create_timer(
            float(self.get_parameter('status_publish_period').value),
            self.timer_status_callback,
        )

    # ==================================================================
    # Parameters
    # ==================================================================
    def _declare_parameters(self):
        self.declare_parameter('color_topic_cam1', '/camera/camera_1/color/image_raw')
        self.declare_parameter('color_topic_cam2', '/camera/camera_2/color/image_raw')
        self.declare_parameter(
            'depth_topic_cam1', '/camera/camera_1/aligned_depth_to_color/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic_cam1', '/camera/camera_1/color/camera_info'
        )
        self.declare_parameter('camera_optical_frame', 'camera_1_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('flange_frame', 'link_6')
        self.declare_parameter('tool_frame', 'drill_link')

        self.declare_parameter('use_camera_info', True)
        self.declare_parameter('fallback_camera_matrix', [
            380.7698974609375, 0.0, 316.40167236328125,
            0.0, 380.4576416015625, 240.86056518554688,
            0.0, 0.0, 1.0,
        ])
        self.declare_parameter('fallback_distortion', [
            -0.0585184171795845, 0.06957467645406723,
            0.00023011499433778226, 0.0003711151657626033,
            -0.022242674604058266,
        ])

        self.declare_parameter('hsv_lower', [5, 80, 80])
        self.declare_parameter('hsv_upper', [25, 255, 255])
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('morph_close_iterations', 3)
        self.declare_parameter('morph_open_iterations', 1)
        self.declare_parameter('min_area_ratio', 0.002)
        self.declare_parameter('max_area_ratio', 0.15)

        self.declare_parameter('min_depth_m', 0.05)
        self.declare_parameter('max_depth_m', 10.0)

        self.declare_parameter('approach_distance_m', 0.10)
        self.declare_parameter('retract_distance_m', 0.10)
        self.declare_parameter('default_drill_depth_mm', 5.0)

        self.declare_parameter('image_save_dir', '~/kuka_images')
        self.declare_parameter('image_viewer', '')
        self.declare_parameter('status_publish_period', 0.1)
        self.declare_parameter('tf_timeout', 1.5)

    def _load_parameters(self):
        g = self.get_parameter
        self.color_topic_cam1 = g('color_topic_cam1').value
        self.color_topic_cam2 = g('color_topic_cam2').value
        self.depth_topic_cam1 = g('depth_topic_cam1').value
        self.camera_info_topic_cam1 = g('camera_info_topic_cam1').value
        self.camera_optical_frame = g('camera_optical_frame').value
        self.base_frame = g('base_frame').value
        self.flange_frame = g('flange_frame').value
        self.tool_frame = g('tool_frame').value

        self.hsv_lower = np.array(g('hsv_lower').value, dtype=np.uint8)
        self.hsv_upper = np.array(g('hsv_upper').value, dtype=np.uint8)
        self.morph_kernel_size = int(g('morph_kernel_size').value)
        self.morph_close_iterations = int(g('morph_close_iterations').value)
        self.morph_open_iterations = int(g('morph_open_iterations').value)
        self.min_area_ratio = float(g('min_area_ratio').value)
        self.max_area_ratio = float(g('max_area_ratio').value)

        self.min_depth_m = float(g('min_depth_m').value)
        self.max_depth_m = float(g('max_depth_m').value)

        self.approach_distance_m = float(g('approach_distance_m').value)
        self.retract_distance_m = float(g('retract_distance_m').value)
        self.default_drill_depth_mm = float(g('default_drill_depth_mm').value)

        self.image_save_dir = os.path.expanduser(g('image_save_dir').value)
        self.image_viewer = g('image_viewer').value
        self.tf_timeout = float(g('tf_timeout').value)

    def _matrix_from_param(self, name, shape):
        values = self.get_parameter(name).value
        return np.array(values, dtype=np.float32).reshape(shape)

    def camera_info_callback(self, msg):
        """Adopt the driver's intrinsics; logged once, then kept up to date."""
        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float32)
        if not self.have_camera_info:
            self.have_camera_info = True
            self.get_logger().info(
                f"Using intrinsics from {self.camera_info_topic_cam1}: "
                f"fx={self.K[0, 0]:.2f} fy={self.K[1, 1]:.2f} "
                f"cx={self.K[0, 2]:.2f} cy={self.K[1, 2]:.2f}"
            )

    # ==================================================================
    def _show_image(self, path):
        """Open an image in an external viewer, if one is configured.

        Headless machines leave 'image_viewer' empty; the file is still
        written to image_save_dir either way.
        """
        if not self.image_viewer:
            return
        if shutil.which(self.image_viewer) is None:
            self.get_logger().warn(
                f"image_viewer '{self.image_viewer}' not found on PATH; "
                f"image saved to {path}"
            )
            return
        try:
            subprocess.Popen([self.image_viewer, path])
        except OSError as exc:
            self.get_logger().warn(f"Could not open viewer for {path}: {exc}")

    # =============================================
    # 이미지 콜백
    # =============================================
    def image_callback_cam1(self, msg):
        self.latest_image_cam1 = msg

    def image_callback_cam2(self, msg):
        self.latest_image_cam2 = msg

    def depth_callback_cam1(self, msg):
        self.latest_depth_cam1 = msg

    # =============================================
    # 명령 처리
    # =============================================
    def command_callback(self, msg):
        target = msg.data
        self.get_logger().info(f"📨 명령 수신: '{target}'")
        threading.Thread(target=self.process_command, args=(target,)).start()

    def process_command(self, cmd):
        """Route a command string to its handler.

        Wrapped so that a parse error or an unexpected exception in the worker
        thread still produces a terminal status. Without this the LLM would
        poll check_robot_status forever on a stuck 'MOVING'.
        """
        try:
            success, result_data = self._dispatch(cmd)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Command '{cmd}' raised: {exc}", exc_info=True)
            success, result_data = False, f"Internal error while handling '{cmd}': {exc}"

        status_str = "STATUS: SUCCESS" if success else "STATUS: ERROR"
        self.publish_status(f"{status_str} | DATA: {result_data}")

    def _dispatch(self, cmd):
        self.publish_status(f"STATUS: MOVING | Command: {cmd}")

        if cmd == "scan":
            success, result_data = self.execute_scan()
        elif cmd.startswith("move_workbench:"):
            wb_num = cmd.split(":")[-1]
            success, result_data = self.execute_move_workbench(wb_num)
        elif cmd.startswith("save_image:"):
            camera = cmd.split(":")[-1]
            success, result_data = self.execute_save_image(camera)
        elif cmd.startswith("mark_point:"):
            parts = cmd.split(":")
            x, y = int(parts[1]), int(parts[2])
            success, result_data = self.execute_mark_point(x, y)
        elif cmd.startswith("adjust_point:"):
            parts = cmd.split(":")
            direction, pixels = parts[1], int(parts[2])
            mark_idx = int(parts[3]) if len(parts) >= 4 else None
            success, result_data = self.execute_adjust_point(direction, pixels, mark_idx)
        elif cmd == "save_point":
            success, result_data = self.execute_save_point()
        elif cmd == "detect":
            success, result_data = self.execute_detect_rectangles()
        elif cmd.startswith("insert_mark:"):
            parts = cmd.split(":")
            a, b = int(parts[1]), int(parts[2])
            success, result_data = self.execute_insert_mark(a, b)
        elif cmd.startswith("delete_mark:"):
            idx = int(cmd.split(":")[-1])
            success, result_data = self.execute_delete_mark(idx)
        elif cmd == "center_line":
            success, result_data = self.execute_center_line()
        elif cmd.startswith("go_to_point:"):
            idx = int(cmd.split(":")[-1])
            success, result_data = self.execute_go_to_point(idx)
        elif cmd == "go_to_all":
            success, result_data = self.execute_go_to_all()
        elif cmd.startswith("go_to_seq:"):
            indices = [int(x) for x in cmd.split(":")[1:]]
            success, result_data = self.execute_go_to_sequence(indices)
        elif cmd.startswith("drill_point:"):
            parts = cmd.split(":")
            idx = int(parts[1])
            depth_mm = float(parts[2]) if len(parts) >= 3 else self.default_drill_depth_mm
            success, result_data = self.execute_drill_point(idx, depth_mm)
        elif cmd.startswith("drill_seq:"):
            parts = cmd.split(":")
            depth_mm = float(parts[1])
            indices = [int(x) for x in parts[2:]]
            success, result_data = self.execute_drill_sequence(indices, depth_mm)
        elif cmd == "list_points":
            success, result_data = self.execute_list_points()
        elif cmd == "clear_points":
            success, result_data = self.execute_clear_points()
        elif cmd.startswith("confirm_and_move"):
            success, result_data = self.execute_confirm_and_move()
        else:
            success = self.execute_move(cmd)
            result_data = f"Arrived at {cmd}" if success else f"Failed to reach {cmd}"

        return success, result_data

    def timer_status_callback(self):
        msg = String()
        msg.data = self.current_status_msg
        self.status_publisher.publish(msg)

    def publish_status(self, message):
        self.current_status_msg = message
        self.get_logger().info(f"📢 {message}")

    # =============================================
    # scan: global camera로 작업대 탐색
    # =============================================
    def execute_scan(self):
        known = ['global_workbench1', 'global_workbench2']
        visible = [t for t in known if self.get_transform(self.base_frame, t, 0.5)]
        if visible:
            return True, f"Visible tags found: {', '.join(visible)}"
        return False, "No tags visible. Please check the camera view."

    # =============================================
    # move_workbench: global 대략 이동 → local 정밀 정렬 (2단계)
    # =============================================
    def execute_move_workbench(self, wb_num):
        """
        2단계 작업대 이동:
        1) global_workbenchN (global camera 태그) → 대략적 접근
        2) local_workbenchN  (local camera 태그)  → 정밀 정렬
        """
        global_tag = f"global_workbench{wb_num}"
        local_tag = f"local_workbench{wb_num}"

        # Step 1: Global approach
        self.publish_status(f"STATUS: MOVING | Step 1/2: Global approach → {global_tag}")
        success = self.execute_move(global_tag)
        if not success:
            return False, f"Global approach failed: {global_tag}"

        self.get_logger().info(f"✅ Global approach done. Waiting 1s before local re-align...")
        time.sleep(1.0)

        # Step 2: Local re-alignment
        self.publish_status(f"STATUS: MOVING | Step 2/2: Local re-align → {local_tag}")
        success = self.execute_move(local_tag)
        if not success:
            return False, f"Local re-alignment failed: {local_tag} (tag not visible from local camera?)"

        return True, f"Workbench {wb_num} reached: global approach + local re-alignment complete"

    # =============================================
    # move: 단일 태그 기준 이동
    # =============================================
    def execute_move(self, target_tag):
        # local 태그는 TF가 바로 안 잡힐 수 있으므로 재시도
        if "local_" in target_tag:
            t_base_tag = self._wait_for_transform(self.base_frame, target_tag, retries=10, interval=1.0)
        else:
            t_base_tag = self.get_transform(self.base_frame, target_tag)

        t_ee_drill = self.get_transform(self.flange_frame, self.tool_frame)

        if not t_base_tag:
            self.get_logger().error(f"TF not found: {self.base_frame} -> {target_tag}")
            return False
        if not t_ee_drill:
            self.get_logger().error("❌ TF를 찾을 수 없음: link_6 → drill_link")
            return False

        if "global_workbench" in target_tag:
            wb_num = target_tag.replace("global_workbench", "")
            self.current_workbench = f"local_workbench{wb_num}"
            self.get_logger().info(f"📌 현재 작업대 설정: {self.current_workbench}")
        elif "local_workbench" in target_tag:
            self.current_workbench = target_tag
            self.get_logger().info(f"📌 현재 작업대 설정: {self.current_workbench}")

        mat_base_tag = self.transform_to_matrix(t_base_tag)
        mat_ee_drill = self.transform_to_matrix(t_ee_drill)
        mat_target_drill = mat_base_tag.copy()

        if "point" in target_tag:
            mat_target_drill[0, 3] -= self.approach_distance_m
        elif "global_" in target_tag or "local_" in target_tag:
            mat_target_drill[0, 3] -= 0.755
            mat_target_drill[1, 3] -= 0.4
            mat_target_drill[2, 3] -= 0.29
        else:
            mat_target_drill[0, 3] -= 0.3

        mat_final_ee = mat_target_drill @ np.linalg.inv(mat_ee_drill)
        return self._send_ik_request(mat_final_ee, target_tag)

    # =============================================
    # 이미지 저장
    # =============================================
    def execute_save_image(self, camera):
        if camera == "camera_1":
            img_msg = self.latest_image_cam1
        elif camera == "camera_2":
            img_msg = self.latest_image_cam2
        else:
            return False, f"Unknown camera: {camera}"

        if img_msg is None:
            return False, f"{camera} image not received yet"

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            filename = f"{camera}_{timestamp}.jpg"
            filepath = os.path.join(self.image_save_dir, filename)
            cv2.imwrite(filepath, cv_image)
            return True, f"Image saved: {filepath}"
        except Exception as e:
            return False, f"Image save failed: {e}"

    # =============================================
    # 최신 카메라 이미지 유틸
    # =============================================
    def _capture_latest_cam1(self):
        if self.latest_image_cam1 is None:
            return None
        try:
            return self.bridge.imgmsg_to_cv2(self.latest_image_cam1, desired_encoding='bgr8')
        except Exception:
            return None

    def _draw_and_save_mark(self):
        img = self._capture_latest_cam1()
        if img is None:
            return False, "camera_1 image not received yet"

        for idx, pt_data in self.saved_points.items():
            px, py = pt_data['pixel']
            cv2.circle(img, (px, py), 6, (0, 255, 0), -1)
            cv2.putText(img, f"P{idx}", (px + 10, py - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        for i, (mx, my) in enumerate(self.pending_marks):
            cv2.circle(img, (mx, my), 8, (0, 0, 255), -1)
            cv2.putText(img, f"M{i+1}", (mx + 12, my - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        marked_filepath = os.path.join(self.image_save_dir, f"marked_{timestamp}.jpg")
        cv2.imwrite(marked_filepath, img)

        self._show_image(marked_filepath)
        return True, marked_filepath

    # =============================================
    # mark_point
    # =============================================
    def execute_mark_point(self, x, y):
        self.pending_marks.append((x, y))
        ok, result = self._draw_and_save_mark()
        if ok:
            n = len(self.pending_marks)
            return True, f"Marked M{n}. Pending: {n} marks. Use 'save_point' to register all."
        return False, result

    # =============================================
    # adjust_point
    # =============================================
    def execute_adjust_point(self, direction, pixels, mark_idx=None):
        if not self.pending_marks:
            return False, "No pending marks. Use mark_point first."

        if mark_idx is not None:
            i = mark_idx - 1
            if i < 0 or i >= len(self.pending_marks):
                return False, f"M{mark_idx} not found. Pending: M1~M{len(self.pending_marks)}"
        else:
            i = len(self.pending_marks) - 1

        x, y = self.pending_marks[i]

        if direction == "up":
            y -= pixels
        elif direction == "down":
            y += pixels
        elif direction == "left":
            x -= pixels
        elif direction == "right":
            x += pixels
        else:
            return False, f"Unknown direction: {direction}. Use up/down/left/right."

        self.pending_marks[i] = (x, y)
        ok, result = self._draw_and_save_mark()
        if ok:
            return True, f"M{i+1} adjusted {direction} {pixels}px"
        return False, result

    # =============================================
    # insert_mark
    # =============================================
    def execute_insert_mark(self, a, b):
        n = len(self.pending_marks)
        if n == 0:
            return False, "No pending marks."
        ia, ib = a - 1, b - 1
        if ia < 0 or ia >= n or ib < 0 or ib >= n:
            return False, f"Invalid index. Pending: M1~M{n}"
        if ia == ib:
            return False, "Same index. Use two different marks."

        ax, ay = self.pending_marks[ia]
        bx, by = self.pending_marks[ib]
        mx, my = (ax + bx) // 2, (ay + by) // 2
        insert_pos = min(ia, ib) + 1
        self.pending_marks.insert(insert_pos, (mx, my))

        ok, result = self._draw_and_save_mark()
        new_idx = insert_pos + 1
        if ok:
            return True, f"Inserted M{new_idx} between M{a} and M{b}. Total: {len(self.pending_marks)} marks."
        return False, result

    # =============================================
    # delete_mark
    # =============================================
    def execute_delete_mark(self, idx):
        n = len(self.pending_marks)
        if n == 0:
            return False, "No pending marks."
        if idx < 1 or idx > n:
            return False, f"Invalid index. Pending: M1~M{n}"
        removed = self.pending_marks.pop(idx - 1)
        ok, _ = self._draw_and_save_mark()
        return True, f"Deleted M{idx}. Remaining: {len(self.pending_marks)} marks."

    # =============================================
    # center_line
    # =============================================
    def execute_center_line(self):
        if len(self.pending_marks) < 2:
            return False, "Need at least 2 marks. Run 'detect' first."

        ys = [p[1] for p in self.pending_marks]
        y_mid = (min(ys) + max(ys)) / 2

        top_row = sorted([p for p in self.pending_marks if p[1] < y_mid], key=lambda p: p[0])
        bot_row = sorted([p for p in self.pending_marks if p[1] >= y_mid], key=lambda p: p[0])

        if not top_row or not bot_row:
            return False, f"Cannot split into 2 rows. Top:{len(top_row)} Bot:{len(bot_row)}"

        center_points = []
        smaller, larger = (top_row, bot_row) if len(top_row) <= len(bot_row) else (bot_row, top_row)
        used = set()

        for sp in smaller:
            best_dist = float('inf')
            best_lp = None
            best_idx = -1
            for j, lp in enumerate(larger):
                if j in used:
                    continue
                dist = abs(sp[0] - lp[0])
                if dist < best_dist:
                    best_dist = dist
                    best_lp = lp
                    best_idx = j
            if best_lp is not None:
                used.add(best_idx)
                cx = (sp[0] + best_lp[0]) // 2
                cy = (sp[1] + best_lp[1]) // 2
                center_points.append((cx, cy))

        if not center_points:
            return False, "No pairs matched."

        center_points.sort(key=lambda p: p[0])
        self.pending_marks = center_points
        ok, _ = self._draw_and_save_mark()
        n = len(center_points)
        return True, f"Center line: {n} points generated (M1~M{n}). Use 'save_point' or 'adjust_point'."

    # =============================================
    # detect
    # =============================================
    def execute_detect_rectangles(self):
        img = self._capture_latest_cam1()
        if img is None:
            return False, "camera_1 image not received yet"

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.morph_kernel_size, self.morph_kernel_size)
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, kernel, iterations=self.morph_close_iterations
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, kernel, iterations=self.morph_open_iterations
        )

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rectangles = []
        h_img, w_img = img.shape[:2]
        img_area = h_img * w_img

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if (area < img_area * self.min_area_ratio
                    or area > img_area * self.max_area_ratio):
                continue
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            rectangles.append({'center': (cx, cy), 'box': box, 'area': area})

        if not rectangles:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            debug_path = os.path.join(self.image_save_dir, f"detect_debug_{timestamp}.jpg")
            cv2.imwrite(debug_path, mask)
            self._show_image(debug_path)
            return False, "No orange rectangles detected. Debug mask saved."

        rectangles.sort(key=lambda r: (r['center'][1], r['center'][0]))
        self.pending_marks.clear()
        for r in rectangles:
            self.pending_marks.append(r['center'])

        vis_img = img.copy()
        for idx, pt_data in self.saved_points.items():
            px, py = pt_data['pixel']
            cv2.circle(vis_img, (px, py), 6, (0, 255, 0), -1)
            cv2.putText(vis_img, f"P{idx}", (px + 10, py - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        for i, r in enumerate(rectangles):
            cv2.drawContours(vis_img, [r['box']], -1, (0, 255, 255), 2)
            cx, cy = r['center']
            cv2.circle(vis_img, (cx, cy), 8, (0, 0, 255), -1)
            cv2.putText(vis_img, f"M{i+1}", (cx + 12, cy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        detect_path = os.path.join(self.image_save_dir, f"detected_{timestamp}.jpg")
        cv2.imwrite(detect_path, vis_img)
        self._show_image(detect_path)

        # 변경
        debug_path = os.path.join(self.image_save_dir, f"detect_mask_{timestamp}.jpg")
        cv2.imwrite(debug_path, mask)

        centers_str = ", ".join([f"M{i+1}" for i in range(len(self.pending_marks))])
        return True, f"Detected {len(rectangles)} orange rectangles: {centers_str}. Use 'save_point' or 'adjust_point'."

    # =============================================
    # 픽셀 → 카메라 3D
    # =============================================
    def pixel_to_3d_camera(self, px, py):
        if self.latest_depth_cam1 is None:
            return None
        try:
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_cam1, desired_encoding='passthrough')
        except Exception:
            return None
        h, w = depth_image.shape[:2]
        if not (0 <= px < w and 0 <= py < h):
            self.get_logger().warn(f"픽셀 ({px},{py}) 범위 초과 [{w}x{h}]")
            return None
        depth_val = float(depth_image[py, px])
        if not np.isfinite(depth_val) or depth_val <= 0:
            return None
        Z = depth_val / 1000.0
        if not (self.min_depth_m <= Z <= self.max_depth_m):
            return None
        pts = np.array([[[px, py]]], dtype=np.float32)
        undist = cv2.undistortPoints(pts, self.K, self.dist_coeffs, P=None)
        u, v = float(undist[0, 0, 0]), float(undist[0, 0, 1])
        X = u * Z
        Y = v * Z
        return (X, Y, Z)

    # =============================================
    # _compute_target_matrix
    # =============================================
    def _compute_target_matrix(self, px, py):
        p_cam = self.pixel_to_3d_camera(px, py)
        if p_cam is None:
            return None, f"No valid depth at pixel ({px},{py})"
        cam_x, cam_y, cam_z = p_cam

        t_base_cam = self.get_transform(self.base_frame, self.camera_optical_frame)
        if not t_base_cam:
            return None, f"Cannot get TF: {self.base_frame} -> {self.camera_optical_frame}"
        mat_base_cam = self.transform_to_matrix(t_base_cam)
        point_base = mat_base_cam @ np.array([cam_x, cam_y, cam_z, 1.0])

        if self.current_workbench is None:
            return None, "No workbench selected. Move to global_workbench first."
        t_base_wb = self.get_transform(self.base_frame, self.current_workbench)
        if not t_base_wb:
            return None, f"Cannot get TF: {self.base_frame} -> {self.current_workbench}"
        mat_base_wb = self.transform_to_matrix(t_base_wb)

        mat_target_drill = mat_base_wb.copy()
        mat_target_drill[:3, 3] = point_base[:3]
        mat_target_drill[0, 3] -= self.approach_distance_m

        base_xyz = [float(point_base[0]), float(point_base[1]), float(point_base[2])]
        return mat_target_drill, base_xyz

    # =============================================
    # save_point
    # =============================================
    def execute_save_point(self):
        if not self.pending_marks:
            return False, "No pending marks. Use mark_point first."
        if self.current_workbench is None:
            return False, "No workbench selected. Move to global_workbench first."

        saved_indices = []
        failed = []
        for px, py in self.pending_marks:
            result = self._compute_target_matrix(px, py)
            if result[0] is None:
                failed.append(f"({px},{py}): {result[1]}")
                continue
            mat_target_drill, base_xyz = result
            self.point_counter += 1
            idx = self.point_counter
            self.saved_points[idx] = {
                'mat_target_drill': mat_target_drill.copy(),
                'base_xyz': base_xyz,
                'pixel': (px, py),
                'workbench': self.current_workbench,
            }
            saved_indices.append(idx)
            self.get_logger().info(
                f"💾 Point {idx} saved: pixel=({px},{py}) "
                f"base=[{base_xyz[0]:.4f}, {base_xyz[1]:.4f}, {base_xyz[2]:.4f}] "
                f"wb={self.current_workbench}"
            )

        self.pending_marks.clear()
        msg_parts = []
        if saved_indices:
            msg_parts.append(f"Saved P{saved_indices} ({len(saved_indices)} points)")
        if failed:
            msg_parts.append(f"Failed: {failed}")
        msg_parts.append(f"Total saved: {len(self.saved_points)}")
        return len(saved_indices) > 0, " | ".join(msg_parts)

    # =============================================
    # go_to_point
    # =============================================
    def execute_go_to_point(self, idx):
        if idx not in self.saved_points:
            return False, f"Point {idx} not found. Saved: {list(self.saved_points.keys())}"
        pt = self.saved_points[idx]
        mat_target_drill = pt['mat_target_drill']

        t_ee_drill = self.get_transform(self.flange_frame, self.tool_frame)
        if not t_ee_drill:
            return False, "Cannot get TF: link_6 → drill_link"
        mat_ee_drill = self.transform_to_matrix(t_ee_drill)
        mat_final_ee = mat_target_drill @ np.linalg.inv(mat_ee_drill)

        self.get_logger().info(
            f"🎯 go_to_point {idx}: pixel={pt['pixel']} "
            f"base={pt['base_xyz']} wb={pt['workbench']}"
        )
        success = self._send_ik_request(mat_final_ee, f"go_to_point:{idx}")
        if success:
            return True, (
                f"Moved to Point {idx}: pixel={pt['pixel']} "
                f"base=[{pt['base_xyz'][0]:.3f}, {pt['base_xyz'][1]:.3f}, {pt['base_xyz'][2]:.3f}]"
            )
        return False, f"IK failed for Point {idx}"

    # =============================================
    # go_to_all
    # =============================================
    def execute_go_to_all(self):
        if not self.saved_points:
            return False, "No saved points."
        results = []
        for idx in sorted(self.saved_points.keys()):
            self.publish_status(f"STATUS: MOVING | go_to_all: Point {idx}/{max(self.saved_points.keys())}")
            success, msg = self.execute_go_to_point(idx)
            results.append(f"P{idx}: {'OK' if success else 'FAIL'}")
            if success:
                time.sleep(1.0)
        return True, f"go_to_all done: {', '.join(results)}"

    # =============================================
    # go_to_seq
    # =============================================
    def execute_go_to_sequence(self, indices):
        if not self.saved_points:
            return False, "No saved points."
        invalid = [i for i in indices if i not in self.saved_points]
        if invalid:
            return False, f"Points not found: {invalid}. Saved: {list(self.saved_points.keys())}"
        results = []
        for step, idx in enumerate(indices, 1):
            self.publish_status(f"STATUS: MOVING | go_to_seq: Step {step}/{len(indices)} → P{idx}")
            success, msg = self.execute_go_to_point(idx)
            results.append(f"P{idx}: {'OK' if success else 'FAIL'}")
            if success:
                time.sleep(1.0)
        return True, f"go_to_seq done: {' → '.join(results)}"

    # =============================================
    # drill_point (depth_mm 지정 가능)
    # =============================================
    def execute_drill_point(self, idx, depth_mm=None):
        if depth_mm is None:
            depth_mm = self.default_drill_depth_mm
        if idx not in self.saved_points:
            return False, f"Point {idx} not found. Saved: {list(self.saved_points.keys())}"
        pt = self.saved_points[idx]
        base_x = pt['base_xyz'][0]

        t_ee_drill = self.get_transform(self.flange_frame, self.tool_frame)
        if not t_ee_drill:
            return False, f"Cannot get TF: {self.flange_frame} -> {self.tool_frame}"
        mat_ee_drill = self.transform_to_matrix(t_ee_drill)
        drill_depth_m = depth_mm / 1000.0

        def move_with_offset(x_offset, label):
            mat = pt['mat_target_drill'].copy()
            mat[0, 3] = base_x - x_offset
            mat_final_ee = mat @ np.linalg.inv(mat_ee_drill)
            return self._send_ik_request(mat_final_ee, label)

        self.publish_status(f"STATUS: DRILL_APPROACH | P{idx}")
        if not move_with_offset(self.approach_distance_m, f"drill_approach P{idx}"):
            return False, f"Drill approach failed at P{idx}"
        time.sleep(0.5)

        self.publish_status(f"STATUS: DRILL_DOWN | P{idx} depth={depth_mm}mm")
        if not move_with_offset(-drill_depth_m, f"drill_down P{idx} {depth_mm}mm"):
            return False, f"Drill down failed at P{idx}"
        time.sleep(2.0)

        self.publish_status(f"STATUS: DRILL_RETRACT | P{idx}")
        if not move_with_offset(self.retract_distance_m, f"drill_retract P{idx}"):
            return False, f"Drill retract failed at P{idx}"

        return True, f"Drilling completed at P{idx} (depth={depth_mm}mm)"

    # =============================================
    # drill_seq (depth_mm 지정 가능)
    # =============================================
    def execute_drill_sequence(self, indices, depth_mm=None):
        if depth_mm is None:
            depth_mm = self.default_drill_depth_mm
        if not self.saved_points:
            return False, "No saved points."
        invalid = [i for i in indices if i not in self.saved_points]
        if invalid:
            return False, f"Points not found: {invalid}. Saved: {list(self.saved_points.keys())}"
        results = []
        for step, idx in enumerate(indices, 1):
            self.publish_status(f"STATUS: DRILLING | drill_seq: Step {step}/{len(indices)} → P{idx} depth={depth_mm}mm")
            success, msg = self.execute_drill_point(idx, depth_mm)
            results.append(f"P{idx}: {'OK' if success else 'FAIL'}")
            if success:
                time.sleep(1.0)
        # 드릴링 전부 완료 후 자동 클리어
        self.saved_points.clear()
        self.point_counter = 0
        
        return True, f"drill_seq done ({depth_mm}mm): {' → '.join(results)} | Points auto-cleared."

    # =============================================
    # list_points
    # =============================================
    def execute_list_points(self):
        if not self.saved_points:
            return True, "No saved points."
        lines = []
        for idx in sorted(self.saved_points.keys()):
            pt = self.saved_points[idx]
            xyz = pt['base_xyz']
            lines.append(
                f"P{idx}: pixel={pt['pixel']} "
                f"base=[{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}] "
                f"wb={pt['workbench']}"
            )
        return True, f"Saved points ({len(self.saved_points)}): " + " | ".join(lines)

    # =============================================
    # clear_points
    # =============================================
    def execute_clear_points(self):
        count = len(self.saved_points)
        self.saved_points.clear()
        self.point_counter = 0
        return True, f"Cleared {count} points."

    # =============================================
    # confirm_and_move
    # =============================================
    def execute_confirm_and_move(self):
        if not self.pending_marks:
            return False, "No pending marks. Use mark_point first."
        if self.current_workbench is None:
            return False, "No workbench selected. Move to global_workbench first."

        px, py = self.pending_marks[-1]
        result = self._compute_target_matrix(px, py)
        if result[0] is None:
            return False, result[1]
        mat_target_drill, base_xyz = result

        t_ee_drill = self.get_transform(self.flange_frame, self.tool_frame)
        if not t_ee_drill:
            return False, "Cannot get TF: link_6 → drill_link"
        mat_ee_drill = self.transform_to_matrix(t_ee_drill)
        mat_final_ee = mat_target_drill @ np.linalg.inv(mat_ee_drill)

        self.get_logger().info(
            f"🎯 confirm_and_move: pixel=({px},{py}) "
            f"base=[{base_xyz[0]:.4f}, {base_xyz[1]:.4f}, {base_xyz[2]:.4f}]"
        )
        success = self._send_ik_request(mat_final_ee, "confirm_and_move")
        if success:
            return True, (
                f"Moved to pixel ({px},{py}) → "
                f"base=[{base_xyz[0]:.3f}, {base_xyz[1]:.3f}, {base_xyz[2]:.3f}] "
                f"(orientation: {self.current_workbench})"
            )
        return False, f"IK failed for pixel ({px},{py})"

    # =============================================
    # IK 서비스 호출 공통 함수
    # =============================================
    def _send_ik_request(self, mat_final_ee, label=""):
        req = KukaTransformInput.Request()
        req.target_transform.transform.translation.x = mat_final_ee[0, 3]
        req.target_transform.transform.translation.y = mat_final_ee[1, 3]
        req.target_transform.transform.translation.z = mat_final_ee[2, 3]
        q = R.from_matrix(mat_final_ee[:3, :3]).as_quat()
        req.target_transform.transform.rotation.x = q[0]
        req.target_transform.transform.rotation.y = q[1]
        req.target_transform.transform.rotation.z = q[2]
        req.target_transform.transform.rotation.w = q[3]

        self.get_logger().info(
            f"🎯 IK 요청 [{label}]: pos=[{mat_final_ee[0,3]:.3f}, "
            f"{mat_final_ee[1,3]:.3f}, {mat_final_ee[2,3]:.3f}]"
        )
        future = self.cli.call_async(req)
        while not future.done():
            time.sleep(0.1)

        result = future.result()
        if result and result.success:
            self.get_logger().info(f"✅ 이동 완료: {label}")
            return True
        else:
            self.get_logger().error(f"❌ 이동 실패: {label}")
            return False

    # =============================================
    # 유틸리티 함수
    # =============================================
    def get_transform(self, source, target, timeout=None):
        if timeout is None:
            timeout = self.tf_timeout
        try:
            if self.tf_buffer.can_transform(
                source, target, rclpy.time.Time(),
                timeout=Duration(seconds=timeout)
            ):
                return self.tf_buffer.lookup_transform(source, target, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF 조회 실패: {source} → {target}: {e}")
        return None

    def transform_to_matrix(self, transform):
        t = transform.transform.translation
        r = transform.transform.rotation
        mat = np.eye(4)
        mat[:3, 3] = [t.x, t.y, t.z]
        mat[:3, :3] = R.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        return mat
    def _wait_for_transform(self, source, target, retries=10, interval=1.0):
        """TF가 잡힐 때까지 재시도 (local 태그용)"""
        for attempt in range(retries):
            t = self.get_transform(source, target, timeout=1.5)
            if t:
                self.get_logger().info(f"✅ TF 발견: {source} → {target} (시도 {attempt+1}/{retries})")
                return t
            self.get_logger().info(f"⏳ TF 대기 중: {source} → {target} (시도 {attempt+1}/{retries})")
            time.sleep(interval)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = KukaTopicMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()