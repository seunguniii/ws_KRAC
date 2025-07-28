#!/usr/bin/env python3
from __future__ import annotations

import os
import time
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry  # 오도메트리 메시지


def build_gst_pipeline(width: int, height: int, fps: int, flip_method: int = 0) -> str:
    return (
        f"nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},format=NV12,framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        "video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink"
    )


class AprilTagLidarFusionNode(Node):
    _ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    except AttributeError:  # OpenCV 버전 호환
        _ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

    # ★ 카메라 캘리브레이션 결과 (첨부 파일 참고)
    _CAMERA_MATRIX = np.array(
        [[827.99145461, 0.0, 249.63373237],
         [0.0, 826.30893069, 260.11920342],
         [0.0, 0.0, 1.0]]
    )
    _DIST_COEFFS = np.array([[-0.27436478, 0.31753802, 0.00183457, -0.01212723, 0.05024013]])

    def __init__(self) -> None:
        super().__init__("apriltag_lidar_fusion")

        os.environ.setdefault("GST_DEBUG", "2")

        # ROS 파라미터 선언
        self.declare_parameter("camera_source", "0")
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("flip_method", 0)
        self.declare_parameter("cam_rate_hz", 30)
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("i2c_addr", 0x62)
        self.declare_parameter("lidar_rate_hz", 10)
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("debug", True)
        self.declare_parameter("show_window", True)
        self.declare_parameter("use_filter", True)
        self.declare_parameter("lidar_alpha", 0.3)

        # 파라미터 값 읽기
        src_param = str(self.get_parameter("camera_source").value)
        width = int(self.get_parameter("camera_width").value)
        height = int(self.get_parameter("camera_height").value)
        fps = int(self.get_parameter("camera_fps").value)
        flip_method = int(self.get_parameter("flip_method").value)
        cam_rate = float(self.get_parameter("cam_rate_hz").value)
        bus_id = int(self.get_parameter("i2c_bus").value)
        self._i2c_addr = int(self.get_parameter("i2c_addr").value)
        lidar_rate = float(self.get_parameter("lidar_rate_hz").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publish_debug = bool(self.get_parameter("debug").value)
        self._show_window = bool(self.get_parameter("show_window").value)
        self._use_filter = bool(self.get_parameter("use_filter").value)
        self._alpha = float(self.get_parameter("lidar_alpha").value)
        self._filtered_z: Optional[float] = None

        # 카메라 열기
        self._cap = None

        if src_param.startswith("udp://") or src_param.endswith(".mp4"):
            # Use GStreamer pipeline for UDP stream or video file
            pipeline = (
                f"udpsrc port=5600 ! application/x-rtp, encoding-name=H264 ! "
                f"rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"
            )
            self.get_logger().info(f"Opening UDP stream pipeline:\n{pipeline}")
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self._cap = cap
            else:
                self.get_logger().error("Failed to open UDP video stream")
        else:
            try:
                idx = int(src_param)
                self.get_logger().info(f"Opening V4L2 index {idx}")
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    self._cap = cap
            except ValueError:
                self.get_logger().info(f"Trying to open as GStreamer pipeline:\n{src_param}")
                cap = cv2.VideoCapture(src_param, cv2.CAP_GSTREAMER)
                if cap.isOpened():
                    self._cap = cap

        if self._cap is None or not self._cap.isOpened():
            self.get_logger().error("Unable to open camera")
            raise RuntimeError("Camera open failed")

        # 오도메트리 구독 (쿼터니언 -> roll/pitch)
        self._odom_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._odom_cb,
            10
        )
        self._have_attitude = False
        self._roll = 0.0
        self._pitch = 0.0

        # 퍼블리셔
        self._bridge = CvBridge()
        self._pub_point = self.create_publisher(PointStamped, "/tag_fused_point", 10)
        if self._publish_debug:
            from sensor_msgs.msg import Image  # Import here to avoid circular dependency if unused
            self._pub_img = self.create_publisher(Image, "/tag_fusion/image_debug", 10)

        # ★ Add periodic timer to run camera frame processing ★
        self._camera_timer = self.create_timer(1.0 / cam_rate, self._camera_timer_cb)

    # 오도메트리 콜백: 자세(roll,pitch) 계산
    def _odom_cb(self, msg: VehicleOdometry) -> None:
        w, x, y, z = msg.q
        # Roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        self._roll = roll
        self._pitch = pitch
        self._have_attitude = True

    # 카메라 프레임 처리
    def _camera_timer_cb(self) -> None:
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().error("Frame capture failed")
            return

        tag_centre = self._detect_first_tag(frame)
        if tag_centre is not None:
            cx, cy = tag_centre

            # ★ 픽셀 → 미터 변환 (카메라 내부 파라미터 사용)
            fx = self._CAMERA_MATRIX[0, 0]
            fy = self._CAMERA_MATRIX[1, 1]
            cx0 = self._CAMERA_MATRIX[0, 2]
            cy0 = self._CAMERA_MATRIX[1, 2]

            dx = cx - cx0
            dy = cy - cy0

            # self._latest_z는 보정된 카메라 높이(수직 z). 카메라 optical axis와 정렬 가정.
            x_m = (dx / fx)
            y_m = (dy / fy)

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.point.x = float(x_m)
            msg.point.y = float(y_m)
            msg.point.z = 0.0
            self._pub_point.publish(msg)

            if self._publish_debug:
                cv2.drawMarker(
                    frame,
                    (int(cx), int(cy)),
                    (0, 255, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2,
                )
                cv2.drawMarker(
                    frame,
                    (int(cx0), int(cy0)),
                    (255, 0, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=10,
                    thickness=1,
                )

        if self._publish_debug:
            self._publish_image(frame)

        if self._show_window:
            cv2.imshow("tag_fusion_debug", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                self.get_logger().info("ESC pressed - shutting down")
                rclpy.shutdown()

    def _publish_image(self, frame: np.ndarray) -> None:
        img_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self._frame_id
        self._pub_img.publish(img_msg)

    # 첫 번째 태그 중심 픽셀 좌표 추출
    def _detect_first_tag(self, frame: np.ndarray) -> Optional[Tuple[float, float]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 왜곡 계수와 카메라 행렬 적용
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self._ARUCO_DICT,
            parameters=self._ARUCO_PARAMS,
            cameraMatrix=self._CAMERA_MATRIX,
            distCoeff=self._DIST_COEFFS
        )
        if ids is None or len(ids) == 0:
            return None
        pts = corners[0].reshape(4, 2)
        cx = float(np.mean(pts[:, 0]))
        cy = float(np.mean(pts[:, 1]))
        return cx, cy


def main(args=None):
    rclpy.init(args=args)
    node: Optional[AprilTagLidarFusionNode] = None
    try:
        node = AprilTagLidarFusionNode()
        rclpy.spin(node)
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
            if hasattr(node, "_cap"):
                node._cap.release()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
