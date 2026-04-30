#!/usr/bin/env python3
"""
hand_eye_calibration_node.py
-----------------------------
Touch-point based eye-in-hand calibration.

Method (much more robust than ArUco 6D pose estimation):
  1. Place a small, distinct marker dot on the work surface (fixed!).
  2. "Touch": jog robot so gripper tip exactly touches the dot.
     Record robot pose → marker position in base frame.
  3. "Camera": jog robot to N >= 4 different poses where camera sees the dot.
     At each pose, click the dot in the UI → pixel + depth → 3D in camera frame.
  4. Solve: minimize projection error across all camera poses to find
     T_flange→camera (X) that best explains all observations.
  5. Save: write hand_eye.yaml + publish static TF.

This avoids the unreliable ArUco orientation estimation on small markers.

Services:
  /hand_eye_calibration/calibrate  (common_msgs/srv/CalibrateHandEye)
"""

import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from common_msgs.srv import CalibrateHandEye
from duco_msg.msg import DucoRobotState
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np

from vision_grasp.hand_eye_calibration import (
    save_calibration_to_yaml,
    compose_4x4, rotation_vector_to_matrix,
    rotation_matrix_to_quaternion, quaternion_to_rotation_matrix,
)


class HandEyeCalibrationNode(Node):

    def __init__(self):
        super().__init__('hand_eye_calibration')
        self.cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._bridge = CvBridge()

        # Parameters
        self.declare_parameter('camera_ns', '/cam_305')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('flange_frame', 'L_base_link')
        self._load_params()

        # State
        self._robot_pose = None       # 4x4 T_base→flange
        self._robot_cart = None       # [x, y, z, rx, ry, rz]
        self._color_img = None        # decoded BGR numpy array
        self._depth_img = None        # decoded depth numpy array
        self._camera_info = None
        self._color_received = False
        self._touch_sample = None     # (T_base→flange, p_base) from touch step
        self._samples = []            # list of (T_W_E, p_cam) — flange pose + camera 3D point
        self._solved_X = None

        # Subscribers
        self._sub_robot = self.create_subscription(
            DucoRobotState, '/duco_cobot/robot_state',
            self._robot_state_cb, 10, callback_group=self.cb_group)

        camera_ns = self.camera_ns if self.camera_ns.startswith('/') else '/' + self.camera_ns
        self._sub_color = self.create_subscription(
            Image, f'{camera_ns}/color/image_raw',
            self._color_cb, 10, callback_group=self.cb_group)
        self._sub_depth = self.create_subscription(
            Image, f'{camera_ns}/depth/image_raw',
            self._depth_cb, 10, callback_group=self.cb_group)
        self._sub_info = self.create_subscription(
            CameraInfo, f'{camera_ns}/depth/camera_info',
            self._info_cb, 10, callback_group=self.cb_group)

        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self._srv = self.create_service(
            CalibrateHandEye, '/hand_eye_calibration/calibrate',
            self._calibrate_callback, callback_group=self.cb_group)

        self.get_logger().info(
            f'HandEyeCalibration ready (camera="{camera_ns}") — '
            f'touch-point method')

    def _load_params(self):
        self.camera_ns = self.get_parameter('camera_ns').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.flange_frame = self.get_parameter('flange_frame').value

    def _robot_state_cb(self, msg):
        if len(msg.cart_actual_position) >= 6:
            cart = list(msg.cart_actual_position[:6])
            self._robot_cart = cart
            x, y, z, rx, ry, rz = cart
            # DUCO uses RPY Euler angles (radians), NOT Rodrigues
            R = self._rpy_to_matrix(rx, ry, rz)
            self._robot_pose = compose_4x4(R, np.array([x, y, z]))

    @staticmethod
    def _rpy_to_matrix(rx, ry, rz):
        """X-Y-Z fixed angles (RPY): R = Rz(rz) * Ry(ry) * Rx(rx)."""
        cr, sr = np.cos(rx), np.sin(rx)
        cp, sp = np.cos(ry), np.sin(ry)
        cy, sy = np.cos(rz), np.sin(rz)
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr],
        ], dtype=np.float64)

    def _color_cb(self, msg):
        try:
            self._color_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._color_received = True
        except Exception:
            pass

    def _depth_cb(self, msg):
        try:
            self._depth_img = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception:
            pass

    def _info_cb(self, msg):
        self._camera_info = msg

    # ── Service ──────────────────────────────────────────────────────

    def _calibrate_callback(self, request, response):
        cmd = request.command.lower().strip()
        try:
            if cmd == 'touch':
                return self._handle_touch(response)
            elif cmd == 'collect':
                return self._handle_collect(request, response)
            elif cmd == 'solve':
                return self._handle_solve(response)
            elif cmd == 'clear':
                return self._handle_clear(response)
            elif cmd == 'save':
                return self._handle_save(response)
            else:
                response.success = False
                response.message = (
                    f'Unknown: "{request.command}". '
                    'Use: touch, collect, solve, clear, save')
        except Exception as e:
            self.get_logger().error(f'Command "{cmd}" failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = str(e)
        return response

    # ── Touch ────────────────────────────────────────────────────────

    def _handle_touch(self, response):
        """Record the marker position by touching it with the gripper tip."""
        if self._robot_pose is None:
            response.success = False
            response.message = 'No robot state — wait for /duco_cobot/robot_state'
            return response

        T_W_E = self._robot_pose.copy()
        # Approximate tool tip in flange frame (DH116 gripper)
        # Fingertip is about 0.15m in flange +Z direction
        p_tool = np.array([0.0, 0.0, 0.15])
        p_base = T_W_E[:3, :3] @ p_tool + T_W_E[:3, 3]

        with self._lock:
            self._touch_sample = (T_W_E, p_base)

        self.get_logger().info(
            f'Touch recorded: marker_base=({p_base[0]:.4f},{p_base[1]:.4f},{p_base[2]:.4f}) '
            f'flange=({self._robot_cart[0]:.3f},{self._robot_cart[1]:.3f},{self._robot_cart[2]:.3f})')

        response.success = True
        response.sample_count = 1
        response.message = (
            f'Marker touched at base=({p_base[0]:.4f},{p_base[1]:.4f},{p_base[2]:.4f})m. '
            f'Now move camera to see the dot and click "collect".')
        return response

    # ── Collect ──────────────────────────────────────────────────────

    def _handle_collect(self, request, response):
        if self._robot_pose is None:
            response.success = False
            response.message = 'No robot state yet'
            return response

        with self._lock:
            depth_img = self._depth_img.copy() if self._depth_img is not None else None
            camera_info = self._camera_info

        if depth_img is None or camera_info is None:
            response.success = False
            response.message = 'No depth image / camera_info yet'
            return response

        u, v = request.select_u, request.select_v
        if u < 0 or v < 0:
            response.success = False
            response.message = (
                'Click the marker dot in the camera view first to get pixel (u,v). '
                'The calibration UI captures your click coordinates.')
            return response

        # Depth → 3D in camera frame
        p_cam = self._pixel_to_camera(u, v, depth_img, camera_info)
        if p_cam is None:
            response.success = False
            response.message = f'No valid depth at pixel ({u:.0f}, {v:.0f})'
            return response

        T_W_E = self._robot_pose.copy()
        with self._lock:
            self._samples.append((T_W_E, p_cam))
            n = len(self._samples)

        self.get_logger().info(
            f'Camera sample {n}: pixel=({u:.0f},{v:.0f}) '
            f'cam_3D=({p_cam[0]:.4f},{p_cam[1]:.4f},{p_cam[2]:.4f}) '
            f'flange=({self._robot_cart[0]:.3f},{self._robot_cart[1]:.3f},{self._robot_cart[2]:.3f})')

        response.success = True
        response.sample_count = n
        response.message = f'Camera sample {n} recorded'
        return response

    def _pixel_to_camera(self, u, v, depth_img, camera_info):
        """Convert pixel + depth image → 3D point in camera optical frame."""
        u, v = int(round(u)), int(round(v))
        h, w = depth_img.shape
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        r = 8
        roi = depth_img[max(0, v-r):min(h, v+r),
                        max(0, u-r):min(w, u+r)].astype(np.float64)
        valid = roi[(roi > 100) & (roi < 5000)]
        if len(valid) < 5:
            return None

        depth_m = float(np.median(valid)) / 1000.0
        K = np.array(camera_info.k).reshape(3, 3)
        x = (u - K[0, 2]) * depth_m / K[0, 0]
        y = (v - K[1, 2]) * depth_m / K[1, 1]
        return np.array([x, y, depth_m])

    # ── Solve ────────────────────────────────────────────────────────

    def _handle_solve(self, response):
        with self._lock:
            if self._touch_sample is None:
                response.success = False
                response.message = 'Run "touch" first to record the marker position.'
                return response
            if len(self._samples) < 3:
                response.success = False
                response.message = (
                    f'Need at least 3 camera samples, have {len(self._samples)}.')
                return response

            _, p_base = self._touch_sample
            samples = list(self._samples)  # list of (T_W_E, p_cam)

        T_W_E_list = [s[0] for s in samples]
        p_cam_list = [s[1] for s in samples]

        X, err = self._optimize_hand_eye(T_W_E_list, p_cam_list, p_base)

        with self._lock:
            self._solved_X = X

        # Project all camera points to base using solved X and compare to touch p_base
        errors = []
        for T_W_E, p_cam in samples:
            proj = T_W_E[:3, :3] @ (X[:3, :3] @ p_cam + X[:3, 3]) + T_W_E[:3, 3]
            err_mm = np.linalg.norm(proj - p_base) * 1000.0
            errors.append(err_mm)

        mean_err = np.mean(errors)
        max_err = np.max(errors)

        t = X[:3, 3]
        rx, ry, rz = self._rpy_deg(X[:3, :3])

        response.success = True
        response.sample_count = len(samples)
        response.residual_rot_deg = 0.0
        response.residual_pos_mm = float(mean_err)

        kval = 'PASS' if mean_err < 5.0 else 'MARGINAL' if mean_err < 15.0 else 'FAIL'
        response.message = (
            f'Solved X: t=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f})m '
            f'rpy=({rx:.1f},{ry:.1f},{rz:.1f})° | '
            f'Projection error: mean={mean_err:.1f}mm max={max_err:.1f}mm | {kval}')

        self.get_logger().info(response.message)
        for i, e in enumerate(errors):
            self.get_logger().info(f'  Sample {i+1}: error={e:.1f}mm')
        return response

    def _optimize_hand_eye(self, T_W_E_list, p_cam_list, p_base):
        """Solve for camera translation only (rotation fixed from placeholder).

        For each sample: p_base = R_W_E * (R_X * p_cam + t_X) + t_W_E
        t_X = R_W_E^T * (p_base - t_W_E) - R_X * p_cam

        Solves t_X and refines p_base alternately (closed-form linear LS).
        """
        import yaml as _yaml
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('vision_grasp')
            yaml_path = os.path.join(pkg_share, 'config', 'hand_eye.yaml')
        except Exception:
            here = os.path.dirname(os.path.abspath(__file__))
            yaml_path = os.path.join(here, '..', '..', '..', '..',
                                     'src', 'vision_grasp', 'config', 'hand_eye.yaml')
            yaml_path = os.path.abspath(yaml_path)

        with open(yaml_path) as f:
            cfg = _yaml.safe_load(f)
        he = cfg.get('hand_eye', {})
        qx, qy = he.get('qx', 0), he.get('qy', 0)
        qz, qw = he.get('qz', 0.7071), he.get('qw', 0.7071)
        R_X = quaternion_to_rotation_matrix([qx, qy, qz, qw])

        n = len(T_W_E_list)
        cur_p_base = p_base.copy()

        # Alternating: solve t_X → solve p_base
        for iteration in range(10):
            # Solve t_X from current p_base
            t_ests = []
            for i in range(n):
                R_W_E = T_W_E_list[i][:3, :3]
                t_W_E = T_W_E_list[i][:3, 3]
                p_cam = p_cam_list[i]
                t_i = R_W_E.T @ (cur_p_base - t_W_E) - R_X @ p_cam
                t_ests.append(t_i)
            t_X = np.mean(t_ests, axis=0)
            t_std = np.std(t_ests, axis=0)

            # Solve p_base from current t_X
            p_ests = []
            for i in range(n):
                R_W_E = T_W_E_list[i][:3, :3]
                t_W_E = T_W_E_list[i][:3, 3]
                p_cam = p_cam_list[i]
                p_i = R_W_E @ (R_X @ p_cam + t_X) + t_W_E
                p_ests.append(p_i)
            new_p_base = np.mean(p_ests, axis=0)

            shift = np.linalg.norm(new_p_base - cur_p_base)
            cur_p_base = new_p_base
            if shift < 1e-6:
                self.get_logger().info(f'Translation solver converged after {iteration+1} iters')
                break

        # Final error
        errors = []
        for i in range(n):
            R_W_E = T_W_E_list[i][:3, :3]
            t_W_E = T_W_E_list[i][:3, 3]
            p_cam = p_cam_list[i]
            proj = R_W_E @ (R_X @ p_cam + t_X) + t_W_E
            errors.append(np.linalg.norm(proj - cur_p_base) * 1000.0)
        mean_err = float(np.mean(errors))

        self.get_logger().info(
            f'Translation: t=({t_X[0]:.4f},{t_X[1]:.4f},{t_X[2]:.4f})m '
            f'(std: {np.linalg.norm(t_std)*1000:.1f}mm)')
        self.get_logger().info(
            f'Refined marker_base=({cur_p_base[0]:.4f},{cur_p_base[1]:.4f},{cur_p_base[2]:.4f}) '
            f'(shift from touch: {np.linalg.norm(cur_p_base-p_base)*1000:.1f}mm)')
        self.get_logger().info(
            f'Mean projection error: {mean_err:.1f}mm')

        X = compose_4x4(R_X, t_X)
        return X, mean_err

    def _rpy_deg(self, R):
        """3x3 rotation → roll, pitch, yaw in degrees."""
        import math
        sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
        if sy > 1e-6:
            rx = math.atan2(R[2, 1], R[2, 2])
            ry = math.atan2(-R[2, 0], sy)
            rz = math.atan2(R[1, 0], R[0, 0])
        else:
            rx = math.atan2(-R[1, 2], R[1, 1])
            ry = math.atan2(-R[2, 0], sy)
            rz = 0.0
        return math.degrees(rx), math.degrees(ry), math.degrees(rz)

    # ── Clear ────────────────────────────────────────────────────────

    def _handle_clear(self, response):
        with self._lock:
            n = len(self._samples)
            self._samples.clear()
        self._touch_sample = None
        self._solved_X = None
        response.success = True
        response.sample_count = 0
        response.message = f'Cleared touch + {n} camera samples'
        return response

    # ── Save ──────────────────────────────────────────────────────────

    def _handle_save(self, response):
        import yaml
        with self._lock:
            if self._solved_X is None:
                response.success = False
                response.message = 'Run "solve" first'
                return response
            X = self._solved_X.copy()

        # Recompute projection error
        with self._lock:
            _, p_base = self._touch_sample
            samples = list(self._samples)
        errors = []
        for T_W_E, p_cam in samples:
            proj = T_W_E[:3, :3] @ (X[:3, :3] @ p_cam + X[:3, 3]) + T_W_E[:3, 3]
            errors.append(np.linalg.norm(proj - p_base) * 1000.0)
        mean_err = float(np.mean(errors))

        # Quality gate
        if mean_err > 15.0:
            response.success = False
            response.residual_pos_mm = mean_err
            response.message = (
                f'REJECTED: mean projection error = {mean_err:.1f}mm > 15mm. '
                f'Re-collect with more varied camera poses and ensure the '
                f'dot is precisely clicked.')
            self.get_logger().error(response.message)
            return response

        # Find yaml path
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_share = get_package_share_directory('vision_grasp')
            yaml_path = os.path.join(pkg_share, 'config', 'hand_eye.yaml')
        except Exception:
            here = os.path.dirname(os.path.abspath(__file__))
            yaml_path = os.path.join(here, '..', '..', '..', '..',
                                     'src', 'vision_grasp', 'config', 'hand_eye.yaml')
            yaml_path = os.path.abspath(yaml_path)

        residuals = {'rot_deg': 0, 'pos_mm': round(mean_err, 2)}
        report = save_calibration_to_yaml(X, yaml_path, residuals=residuals,
                                          yaml_obj=yaml)
        self.get_logger().info(report)

        # Publish static TF
        t = X[:3, 3]
        q = rotation_matrix_to_quaternion(X[:3, :3])
        child_frame = self._get_camera_link_name()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.flange_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])

        self._tf_broadcaster.sendTransform(tf_msg)
        self.get_logger().info(
            f'Published static TF: {self.flange_frame} → {child_frame}')

        response.success = True
        response.sample_count = len(samples)
        response.residual_pos_mm = mean_err
        response.filepath = yaml_path
        response.message = report
        return response

    def _get_camera_link_name(self):
        if '_color_optical_frame' in self.camera_frame:
            return self.camera_frame.replace('_color_optical_frame', '_link')
        return f'{self.camera_ns.strip("/")}_link'


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
