#!/usr/bin/env python3
"""
bottle_detector_node.py
-----------------------
Two modes:
  1. User-selection: request provides (select_u, select_v) → depth lookup → 3D
  2. Auto-detect: select_u < 0 → YOLO finds bottle → depth lookup → 3D
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from common_msgs.srv import DetectBottle
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class BottleDetectorNode(Node):

    BOTTLE_CLASS_ID = 39  # COCO "bottle"

    def __init__(self):
        super().__init__('bottle_detector_node')

        self.declare_parameter('camera_ns', '/camera')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('depth_roi_half', 10)
        self.declare_parameter('min_depth_mm', 100.0)
        self.declare_parameter('max_depth_mm', 2000.0)

        self.camera_ns = self.get_parameter('camera_ns').value
        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.roi_half = self.get_parameter('depth_roi_half').value
        self.min_depth = self.get_parameter('min_depth_mm').value
        self.max_depth = self.get_parameter('max_depth_mm').value

        # YOLO is optional — user-selection mode works without it
        self.model = None
        if YOLO is not None:
            self.get_logger().info(f'Loading YOLO model: {model_path}')
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO model loaded')
        else:
            self.get_logger().warn('ultralytics not installed — auto-detect disabled, user-selection only')

        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.K = None

        self._subscribe_camera(self.camera_ns)

        self.srv = self.create_service(
            DetectBottle, '/bottle_detector/detect', self.detect_callback)
        self.get_logger().info('BottleDetectorNode ready')

    def _subscribe_camera(self, ns: str):
        ns = ns.rstrip('/')
        self.sub_color = self.create_subscription(
            Image, f'{ns}/color/image_raw', self._cb_color, 1)
        self.sub_depth = self.create_subscription(
            Image, f'{ns}/depth/image_raw', self._cb_depth, 1)
        self.sub_info = self.create_subscription(
            CameraInfo, f'{ns}/depth/camera_info', self._cb_info, 1)

    def _cb_color(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    def _cb_depth(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
    def _cb_info(self, msg):  self.K = msg.k

    def _robust_depth_sample(self, u_center, v_center):
        """Robust depth sampling with MAD outlier rejection and distance weighting.

        Returns depth in meters, or None if no valid depth found.
        """
        u = int(round(u_center))
        v = int(round(v_center))
        h_img, w_img = self.latest_depth.shape[:2]
        r = int(self.roi_half)

        def _extract_roi(roi_r):
            return self.latest_depth[
                max(0, v - roi_r):min(h_img, v + roi_r),
                max(0, u - roi_r):min(w_img, u + roi_r)
            ].astype(np.float64)

        roi = _extract_roi(r)
        valid_mask = (roi > self.min_depth) & (roi < self.max_depth)
        valid = roi[valid_mask]

        # Fallback: smaller ROI if too few valid pixels
        if len(valid) < 10:
            r2 = max(3, r // 2)
            roi2 = _extract_roi(r2)
            valid_mask2 = (roi2 > self.min_depth) & (roi2 < self.max_depth)
            valid2 = roi2[valid_mask2]
            if len(valid2) < 5:
                return None
            return float(np.median(valid2)) / 1000.0

        # MAD-based outlier rejection
        median_val = np.median(valid)
        mad = float(np.median(np.abs(valid - median_val)))
        if mad < 1e-6:
            return float(median_val) / 1000.0

        # Build combined mask: valid AND inlier
        inlier_thresh = 2.5 * mad
        inlier_2d = valid_mask & (np.abs(roi - median_val) < inlier_thresh)
        inliers = roi[inlier_2d]
        if len(inliers) < 5:
            return float(median_val) / 1000.0

        # Distance-weighted mean (Gaussian kernel centered on ROI midpoint)
        roi_h, roi_w = roi.shape
        cy, cx = (roi_h - 1) / 2.0, (roi_w - 1) / 2.0
        sigma = max(1.0, r / 2.0)
        yu, xu = np.mgrid[0:roi_h, 0:roi_w]
        weights = np.exp(-((yu - cy) ** 2 + (xu - cx) ** 2) / (2.0 * sigma ** 2))
        inlier_weights = weights[inlier_2d]

        weighted_mean = float(np.average(inliers, weights=inlier_weights))
        return weighted_mean / 1000.0

    def detect_callback(self, request, response):
        # Camera switch
        if request.camera_ns and request.camera_ns != self.camera_ns:
            self.camera_ns = request.camera_ns
            self._subscribe_camera(self.camera_ns)
            response.success = False
            response.message = f'Switched camera to {self.camera_ns}, retry'
            return response

        if self.latest_depth is None:
            response.success = False
            response.message = 'No depth image received yet'
            return response
        if self.K is None:
            response.success = False
            response.message = 'No camera_info received yet'
            return response

        user_mode = request.select_u >= 0 and request.select_v >= 0

        if user_mode:
            # --- User-selection mode: use provided pixel directly ---
            u_center = request.select_u
            v_center = request.select_v
            confidence = 1.0
            bbox_w = 0.0
            bbox_h = 0.0
            self.get_logger().info(f'User-selected pixel: ({u_center:.0f}, {v_center:.0f})')
        else:
            # --- Auto-detect mode: YOLO ---
            if self.model is None:
                response.success = False
                response.message = 'Auto-detect requires ultralytics (pip install ultralytics)'
                return response
            if self.latest_color is None:
                response.success = False
                response.message = 'No color image received yet'
                return response

            results = self.model(self.latest_color, classes=[self.BOTTLE_CLASS_ID], verbose=False)
            boxes = results[0].boxes
            if len(boxes) == 0:
                response.success = False
                response.message = 'No bottle detected'
                return response

            confs = boxes.conf.cpu().numpy()
            best_idx = int(np.argmax(confs))
            confidence = float(confs[best_idx])
            if confidence < self.conf_thresh:
                response.success = False
                response.message = f'Confidence {confidence:.2f} below threshold'
                return response

            xyxy = boxes.xyxy.cpu().numpy()[best_idx]
            u_center = (xyxy[0] + xyxy[2]) / 2.0
            v_center = xyxy[1] + (xyxy[3] - xyxy[1]) * 0.6
            bbox_w = float(xyxy[2] - xyxy[0])
            bbox_h = float(xyxy[3] - xyxy[1])

        # --- Depth lookup (shared by both modes) ---
        depth_m = self._robust_depth_sample(u_center, v_center)
        if depth_m is None:
            response.success = False
            response.message = f'No valid depth at ({u_center:.0f}, {v_center:.0f})'
            return response

        # --- 3D projection (pinhole) ---
        fx, fy = self.K[0], self.K[4]
        cx, cy = self.K[2], self.K[5]
        x_cam = (u_center - cx) * depth_m / fx
        y_cam = (v_center - cy) * depth_m / fy
        z_cam = depth_m

        response.success = True
        response.message = f'depth={depth_m*1000:.0f}mm conf={confidence:.2f}'
        response.u = float(u_center)
        response.v = float(v_center)
        response.confidence = confidence
        response.x = x_cam
        response.y = y_cam
        response.z = z_cam
        response.bbox_width = bbox_w
        response.bbox_height = bbox_h

        self.get_logger().info(
            f'Result: pixel=({u_center:.0f},{v_center:.0f}) '
            f'3D=({x_cam:.3f},{y_cam:.3f},{z_cam:.3f}) depth={depth_m*1000:.0f}mm')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BottleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
