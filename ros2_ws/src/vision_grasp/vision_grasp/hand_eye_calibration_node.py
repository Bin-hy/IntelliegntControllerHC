#!/usr/bin/env python3
"""
hand_eye_calibration_node.py
-----------------------------
Eye-in-hand calibration supporting two methods:
  1. ArUco marker (auto-detect): collects T_camera→marker via solvePnP,
     then solves full AX=XB for rotation + translation.
  2. Touch-point (manual click fallback): translation-only refinement
     using a fixed reference point.

Service: /hand_eye_calibration/calibrate (common_msgs/srv/CalibrateHandEye)
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
import numpy as np
import cv2

from vision_grasp.hand_eye_calibration import (
    save_calibration_to_yaml,
    solve_hand_eye_ax_xb,
    compose_4x4, rotation_vector_to_matrix, matrix_to_rotation_vector,
    rotation_matrix_to_quaternion, quaternion_to_rotation_matrix,
    homogeneous_inv, rpy_to_matrix, matrix_to_rpy,
)
import math


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
        self.declare_parameter('marker_size', 0.04)        # ArUco marker side length (m)
        self.declare_parameter('aruco_dict_id', 0)         # DICT_4X4_50 = 0
        self.declare_parameter('aruco_marker_id', 0)       # marker ID to look for

        # Must set before _load_params() — it checks these attributes
        self._aruco_dict = None
        self._aruco_params = None
        self._load_params()

        # State
        self._robot_pose = None
        self._robot_cart = None
        self._color_img = None
        self._depth_img = None
        self._camera_info = None
        self._touch_sample = None
        self._samples = []            # list of (T_W_E, T_C_M, raw_cart) — robot flange pose, camera→marker, raw [x,y,z,rx,ry,rz]
        self._solved_X = None
        self._refined_p_base = None

        # Subscribers
        self._sub_robot = self.create_subscription(
            DucoRobotState, '/duco_cobot/robot_state',
            self._robot_state_cb, 10, callback_group=self.cb_group)

        camera_ns = self.camera_ns.lstrip('/')
        self._sub_color = self.create_subscription(
            Image, f'/{camera_ns}/color/image_raw',
            self._color_cb, 1, callback_group=self.cb_group)
        self._sub_depth = self.create_subscription(
            Image, f'/{camera_ns}/depth/image_raw',
            self._depth_cb, 1, callback_group=self.cb_group)
        self._sub_info = self.create_subscription(
            CameraInfo, f'/{camera_ns}/color/camera_info',
            self._info_cb, 1, callback_group=self.cb_group)

        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self._srv = self.create_service(
            CalibrateHandEye, '/hand_eye_calibration/calibrate',
            self._calibrate_callback, callback_group=self.cb_group)

        self.get_logger().info('HandEyeCalibration ready (v2 — manual PnP)')

    def _load_params(self):
        self.camera_ns = self.get_parameter('camera_ns').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.flange_frame = self.get_parameter('flange_frame').value
        self.marker_size = self.get_parameter('marker_size').value
        self.aruco_dict_id = self.get_parameter('aruco_dict_id').value
        self.aruco_marker_id = self.get_parameter('aruco_marker_id').value

        if self._aruco_dict is None:
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_id)
            self._aruco_params = cv2.aruco.DetectorParameters_create()

    def _robot_state_cb(self, msg):
        if len(msg.cart_actual_position) >= 6:
            cart = list(msg.cart_actual_position[:6])
            self._robot_cart = cart
            x, y, z, rx, ry, rz = cart
            R = rotation_vector_to_matrix([rx, ry, rz])
            self._robot_pose = compose_4x4(R, np.array([x, y, z]))

    def _color_cb(self, msg):
        try:
            self._color_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _depth_cb(self, msg):
        try:
            self._depth_img = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception:
            pass

    def _info_cb(self, msg):
        self._camera_info = msg

    # ── Service dispatch ──────────────────────────────────────────────

    def _calibrate_callback(self, request, response):
        cmd = request.command.lower().strip()
        try:
            if cmd == 'touch':
                return self._handle_touch(response)
            elif cmd == 'collect':
                return self._handle_collect(request, response)
            elif cmd == 'solve':
                return self._handle_solve(response)
            elif cmd == 'diagnose':
                return self._handle_diagnose(response)
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
            response.success = False
            response.message = str(e)
        return response

    # ── Touch ────────────────────────────────────────────────────────

    def _handle_touch(self, response):
        if self._robot_pose is None:
            response.success = False
            response.message = 'No robot state'
            return response

        T_W_E = self._robot_pose.copy()
        p_tool = np.array([0.0, 0.0, 0.12])
        p_base = T_W_E[:3, :3] @ p_tool + T_W_E[:3, 3]

        with self._lock:
            self._touch_sample = (T_W_E, p_base)

        self.get_logger().info(
            f'Touch: marker_base=({p_base[0]:.4f},{p_base[1]:.4f},{p_base[2]:.4f})')
        response.success = True
        response.sample_count = len(self._samples)
        response.message = 'Touch recorded'
        return response

    # ── Collect ──────────────────────────────────────────────────────

    def _detect_aruco(self, color_img, camera_info):
        """Try to detect ArUco marker. Returns T_camera->marker (4x4) or None."""
        if color_img is None:
            self.get_logger().warn('  No color image yet')
            return None
        if camera_info is None:
            self.get_logger().warn('  No camera_info yet')
            return None

        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)

        if ids is None or len(ids) == 0:
            self.get_logger().warn('  No ArUco marker found — is marker visible?')
            return None

        target = self.aruco_marker_id
        found_idx = None
        for i, mid in enumerate(ids):
            if int(mid[0]) == target:
                found_idx = i
                break
        if found_idx is None:
            self.get_logger().warn(
                f'  ID={target} not found (saw: {[int(m[0]) for m in ids]})')
            return None

        K = np.array(camera_info.k).reshape(3, 3)

        half = self.marker_size / 2.0
        objp = np.array([[-half, -half, 0.], [half, -half, 0.],
                         [half,  half, 0.], [-half,  half, 0.]], dtype=np.float64)
        c = corners[found_idx].reshape(4, 2).astype(np.float64)

        span = np.max(c, axis=0) - np.min(c, axis=0)
        self.get_logger().info(f'  Marker: {span[0]:.0f}x{span[1]:.0f}px')

        # Estimate initial depth from pixel size
        est_z = K[0, 0] * self.marker_size / max(span[0], span[1])
        rvec_init = np.zeros((3, 1), dtype=np.float64)
        tvec_init = np.array([[0.0], [0.0], [est_z]], dtype=np.float64)

        success, rvec, tvec = cv2.solvePnP(
            objp, c, K, None, rvec_init, tvec_init, True,
            flags=cv2.SOLVEPNP_ITERATIVE)

        if not success:
            self.get_logger().warn('  solvePnP failed')
            return None

        z = float(tvec[2])
        if z < 0.01:
            self.get_logger().warn(f'  Z={z:.3f} (behind camera)')
            return None

        proj, _ = cv2.projectPoints(objp, rvec, tvec, K, None)
        err = np.linalg.norm(proj.reshape(-1, 2) - c, axis=1).mean()
        if err > 15.0:
            self.get_logger().warn(f'  Reproj err {err:.1f}px — rejecting')
            return None
        elif err > 5.0:
            self.get_logger().warn(f'  Reproj err: {err:.1f}px')

        R = rotation_vector_to_matrix(rvec.ravel())
        return compose_4x4(R, tvec.ravel())  # T_camera->marker

    def _handle_collect(self, request, response):
        if self._robot_pose is None:
            response.success = False
            response.message = 'No robot state'
            return response

        with self._lock:
            color_img = self._color_img.copy() if self._color_img is not None else None
            camera_info = self._camera_info

        if camera_info is None:
            response.success = False
            response.message = 'No camera_info yet'
            return response
        if color_img is None:
            response.success = False
            response.message = 'No color image yet — is camera running?'
            return response

        # ── Method 1: Auto-detect ArUco marker ──
        T_C_M = self._detect_aruco(color_img, camera_info)
        if T_C_M is not None:
            T_W_E = self._robot_pose.copy()
            raw_cart = list(self._robot_cart[:6]) if self._robot_cart else [0]*6
            with self._lock:
                self._samples.append((T_W_E, T_C_M, raw_cart))
                n = len(self._samples)

            t_marker = T_C_M[:3, 3]
            self.get_logger().info(
                f'Sample {n} (ArUco): cam→marker=({t_marker[0]:.4f},{t_marker[1]:.4f},{t_marker[2]:.4f}) '
                f'flange=({self._robot_cart[0]:.3f},{self._robot_cart[1]:.3f},{self._robot_cart[2]:.3f})')
            response.success = True
            response.sample_count = n
            response.message = f'Sample {n} recorded (ArUco auto-detected)'
            return response

        # ── Method 2: Fallback — manual pixel click ──
        with self._lock:
            depth_img = self._depth_img.copy() if self._depth_img is not None else None

        if depth_img is None:
            response.success = False
            response.message = 'No ArUco found and no depth image for manual click'
            return response

        u, v = request.select_u, request.select_v
        if u <= 0 or v <= 0:
            response.success = False
            response.message = 'ArUco not detected — adjust camera so marker is clearly visible'
            return response

        u, v = int(round(u)), int(round(v))
        h, w = depth_img.shape
        if u < 0 or v < 0 or u >= w or v >= h:
            response.success = False
            response.message = 'Pixel out of bounds'
            return response

        r = 8
        roi = depth_img[max(0, v-r):min(h, v+r),
                        max(0, u-r):min(w, u+r)].astype(np.float64)
        valid = roi[(roi > 100) & (roi < 5000)]
        if len(valid) < 5:
            response.success = False
            response.message = f'No valid depth at ({u}, {v})'
            return response

        depth_m = float(np.median(valid)) / 1000.0
        K = np.array(camera_info.k).reshape(3, 3)
        x = (u - K[0, 2]) * depth_m / K[0, 0]
        y = (v - K[1, 2]) * depth_m / K[1, 1]
        p_cam = np.array([x, y, depth_m])
        # Store as 3D point (legacy touch-point mode) — solve will use translation-only
        T_W_E = self._robot_pose.copy()
        raw_cart = list(self._robot_cart[:6]) if self._robot_cart else [0]*6
        with self._lock:
            self._samples.append((T_W_E, p_cam, raw_cart))
            n = len(self._samples)

        self.get_logger().info(
            f'Sample {n} (manual): pixel=({u},{v}) cam=({x:.4f},{y:.4f},{depth_m:.4f}) '
            f'flange=({self._robot_cart[0]:.3f},{self._robot_cart[1]:.3f},{self._robot_cart[2]:.3f})')

        response.success = True
        response.sample_count = n
        response.message = f'Sample {n} recorded (manual click)'
        return response

    # ── Solve ─────────────────────────────────────────────────────────

    def _handle_solve(self, response):
        with self._lock:
            if len(self._samples) < 3:
                response.success = False
                response.message = (
                    f'Need at least 3 samples, have {len(self._samples)}')
                return response
            samples = list(self._samples)

        # Re-derive T_W_E from raw cart using Rodrigues (DUCO convention)
        T_W_E_list = []
        T_C_M_list = []
        for s in samples:
            raw = s[2]
            x, y, z, rx, ry, rz = raw[:6]
            R = rotation_vector_to_matrix([rx, ry, rz])
            T_W_E_list.append(compose_4x4(R, np.array([x, y, z])))
            T_C_M_list.append(s[1])

        second = samples[0][1]

        # ── Path A: ArUco data (full 4x4 T_C_M) → AX=XB full solve ──
        if isinstance(second, np.ndarray) and second.shape == (4, 4):
            result = solve_hand_eye_ax_xb(T_W_E_list, T_C_M_list,
                                          min_rot_deg=3.0)
            if not result['success']:
                response.success = False
                response.sample_count = len(samples)
                response.message = f'Solver failed: need at least 3 pair constraints (got {result["num_pairs_used"]})'
                return response

            X = result['X_4x4']
            with self._lock:
                self._solved_X = X
                self._refined_p_base = None

            t = X[:3, 3]
            q = rotation_matrix_to_quaternion(X[:3, :3])
            rv = matrix_to_rotation_vector(X[:3, :3])
            angle_deg = np.degrees(np.linalg.norm(rv))
            pos_err = result['residual_pos_mm']
            rot_err = result['residual_rot_deg']
            kval = 'PASS' if pos_err < 5.0 else 'MARGINAL' if pos_err < 15.0 else 'FAIL'

            response.success = True
            response.sample_count = len(samples)
            response.residual_pos_mm = float(pos_err)
            response.residual_rot_deg = float(rot_err)
            response.message = (
                f'Solved (AX=XB): t=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f})m '
                f'rot_err={rot_err:.2f}deg pos_err={pos_err:.1f}mm | '
                f'pairs={result["num_pairs_used"]} | {kval}')
            self.get_logger().info(response.message)
            self.get_logger().info(
                f'  Rotation: {angle_deg:.1f}deg '
                f'q=({q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f})')
            return response

        # ── Path B: Manual click data (3D points) → touch-point translation-only ──
        # This keeps backward compatibility with the old touch-point flow.
        if self._touch_sample is None:
            response.success = False
            response.message = 'Run "touch" first (manual mode requires a reference touch point)'
            return response

        _, p_base = self._touch_sample
        p_cam_list = [s[1] for s in samples]

        import yaml as _yaml
        yaml_path = self._find_yaml_path()
        with open(yaml_path) as f:
            cfg = _yaml.safe_load(f)
        he = cfg.get('hand_eye', {})
        R_X = quaternion_to_rotation_matrix([
            he.get('qx', 0.0), he.get('qy', 0.0),
            he.get('qz', 0.0), he.get('qw', 1.0),
        ])

        n = len(T_W_E_list)
        cur_p_base = p_base.copy()

        for _ in range(10):
            t_ests = []
            for i in range(n):
                R_W_E = T_W_E_list[i][:3, :3]
                t_W_E = T_W_E_list[i][:3, 3]
                t_ests.append(R_W_E.T @ (cur_p_base - t_W_E) - R_X @ p_cam_list[i])
            t_X = np.mean(t_ests, axis=0)

            p_ests = []
            for i in range(n):
                R_W_E = T_W_E_list[i][:3, :3]
                t_W_E = T_W_E_list[i][:3, 3]
                p_ests.append(R_W_E @ (R_X @ p_cam_list[i] + t_X) + t_W_E)
            new_p_base = np.mean(p_ests, axis=0)

            if np.linalg.norm(new_p_base - cur_p_base) < 1e-6:
                break
            cur_p_base = new_p_base

        errors = []
        for i in range(n):
            R_W_E = T_W_E_list[i][:3, :3]
            t_W_E = T_W_E_list[i][:3, 3]
            proj = R_W_E @ (R_X @ p_cam_list[i] + t_X) + t_W_E
            errors.append(np.linalg.norm(proj - cur_p_base) * 1000.0)
        mean_err = float(np.mean(errors))

        X = compose_4x4(R_X, t_X)

        with self._lock:
            self._solved_X = X
            self._refined_p_base = cur_p_base

        t = X[:3, 3]
        q = rotation_matrix_to_quaternion(X[:3, :3])
        p_shift = np.linalg.norm(cur_p_base - p_base) * 1000.0
        kval = 'PASS' if mean_err < 5.0 else 'MARGINAL' if mean_err < 15.0 else 'FAIL'

        response.success = True
        response.sample_count = n
        response.residual_pos_mm = float(mean_err)
        response.message = (
            f'Solved (translation-only): t=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f})m '
            f'q=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f}) | '
            f'err={mean_err:.1f}mm shift={p_shift:.1f}mm | {kval}')

        self.get_logger().info(response.message)
        for i, e in enumerate(errors):
            self.get_logger().info(f'  Sample {i+1}: err={e:.1f}mm')
        return response

    # ── Diagnose ──────────────────────────────────────────────────────

    @staticmethod
    def _compute_residuals_cv(X, R_W_list, t_W_list, R_C_list, t_C_list):
        """Compute residuals for OpenCV calibrateHandEye output."""
        import math as _m
        projected = []
        for i in range(len(R_W_list)):
            T_W_M = compose_4x4(R_W_list[i], t_W_list[i]) @ X @ compose_4x4(R_C_list[i], t_C_list[i])
            projected.append(T_W_M)
        mean_t = np.mean([p[:3, 3] for p in projected], axis=0)
        quats = [rotation_matrix_to_quaternion(p[:3, :3]) for p in projected]
        mean_q = np.mean(quats, axis=0)
        mean_q /= np.linalg.norm(mean_q)
        mean_R = quaternion_to_rotation_matrix(mean_q)
        rot_errs = []
        pos_errs = []
        for p in projected:
            pos_errs.append(np.linalg.norm(p[:3, 3] - mean_t) * 1000.0)
            R_rel = p[:3, :3] @ mean_R.T
            angle = _m.acos(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))
            rot_errs.append(_m.degrees(angle))
        return float(np.mean(rot_errs)), float(np.mean(pos_errs))

    def _handle_diagnose(self, response):
        """Test all rotation conventions on the current samples."""
        with self._lock:
            samples = list(self._samples)
        if len(samples) < 3:
            response.success = False
            response.message = f'Need 3+ samples, have {len(samples)}'
            return response

        second = samples[0][1]
        if not (isinstance(second, np.ndarray) and second.shape == (4, 4)):
            response.success = False
            response.message = 'Diagnose only works with ArUco samples'
            return response

        # Reconstruct T_W_E from the saved cartesian values using each convention
        # We need the raw [rx,ry,rz] values — stored in _samples as extra info
        # For now, back-compute rpy/rv from the saved rotation matrix
        T_C_M_matrices = [s[1] for s in samples]
        raw_carts = [s[2] for s in samples]

        # Convention 0: Rodrigues (rotation vector)
        # Convention 1: RPY fixed XYZ = Rz*Ry*Rx
        # Convention 2: RPY intrinsic ZYX = Rx*Ry*Rz

        results = []
        labels = [
            'Rodrigues (rotation vector)',
            'RPY fixed XYZ (Rz*Ry*Rx)',
            'RPY intrinsic ZYX (Rx*Ry*Rz)',
        ]
        T_W_E_rod = []  # store Rodrigues-converted matrices for OpenCV

        for mode in [0, 1, 2]:
            converted = []
            for raw in raw_carts:
                x, y, z, rx, ry, rz = raw
                t = np.array([x, y, z], dtype=np.float64)
                if mode == 0:
                    R2 = rotation_vector_to_matrix([rx, ry, rz])
                elif mode == 1:
                    R2 = rpy_to_matrix([rx, ry, rz])
                else:  # mode == 2: intrinsic ZYX
                    cx, sx = math.cos(rx), math.sin(rx)
                    cy, sy = math.cos(ry), math.sin(ry)
                    cz, sz = math.cos(rz), math.sin(rz)
                    R2 = np.array([
                        [cy*cz, -cy*sz, sy],
                        [cx*sz+cz*sx*sy, cx*cz-sx*sy*sz, -cy*sx],
                        [sx*sz-cx*cz*sy, cz*sx+cx*sy*sz, cx*cy],
                    ], dtype=np.float64)
                converted.append(compose_4x4(R2, t))
            if mode == 0:
                T_W_E_rod = converted

            result = solve_hand_eye_ax_xb(converted, T_C_M_matrices, min_rot_deg=3.0)
            results.append((labels[mode], result))

        lines = ['=== Rotation Convention Test ===', '']
        best_rot = float('inf')
        best_pos = float('inf')
        best_label_rot = ''
        best_label_pos = ''

        for label, r in results:
            rot = r['residual_rot_deg'] if r['success'] else 999.0
            pos = r['residual_pos_mm'] if r['success'] else 9999.0
            status = 'PASS' if (r['success'] and rot < 5.0 and pos < 15.0) else 'FAIL'
            t = r['X_4x4'][:3, 3] if r['success'] else np.zeros(3)
            lines.append(
                f'  {label}:')
            lines.append(
                f'    rot_err={rot:.2f}deg  pos_err={pos:.1f}mm  '
                f'pairs={r["num_pairs_used"]}  {status}')
            if r['success']:
                lines.append(
                    f'    X_t=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f})')
            lines.append('')
            if r['success'] and rot < best_rot:
                best_rot = rot
                best_label_rot = label
            if r['success'] and pos < best_pos:
                best_pos = pos
                best_label_pos = label

        # ── OpenCV calibrateHandEye methods (Rodrigues convention) ──
        lines.append('--- OpenCV calibrateHandEye (Rodrigues) ---')
        lines.append('')
        R_W_list = [s[:3, :3].copy() for s in T_W_E_rod]
        t_W_list = [s[:3, 3].copy() for s in T_W_E_rod]
        R_C_list = [s[:3, :3].copy() for s in T_C_M_matrices]
        t_C_list = [s[:3, 3].copy() for s in T_C_M_matrices]

        ocv_methods = [
            ('TSAI', cv2.CALIB_HAND_EYE_TSAI),
            ('PARK', cv2.CALIB_HAND_EYE_PARK),
            ('HORAUD', cv2.CALIB_HAND_EYE_HORAUD),
            ('ANDREFF', cv2.CALIB_HAND_EYE_ANDREFF),
            ('DANIILIDIS', cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]
        for name, method in ocv_methods:
            try:
                R_X_cv, t_X_cv = cv2.calibrateHandEye(
                    R_W_list, t_W_list,
                    R_C_list, t_C_list,
                    method=method)
                X_cv = compose_4x4(R_X_cv, t_X_cv.flatten())
                r_deg, p_mm = self._compute_residuals_cv(
                    X_cv, R_W_list, t_W_list, R_C_list, t_C_list)
                status = 'PASS' if r_deg < 5.0 and p_mm < 15.0 else 'FAIL'
                lines.append(
                    f'  {name}: rot_err={r_deg:.2f}deg  pos_err={p_mm:.1f}mm  {status}')
                lines.append(
                    f'    X_t=({t_X_cv[0]:.4f},{t_X_cv[1]:.4f},{t_X_cv[2]:.4f})')
                if r_deg < best_rot:
                    best_rot = r_deg
                    best_label_rot = f'OpenCV {name}'
                if p_mm < best_pos:
                    best_pos = p_mm
                    best_label_pos = f'OpenCV {name}'
            except Exception as e:
                lines.append(f'  {name}: ERROR — {e}')
            lines.append('')

        lines.append(f'  Best rotation: {best_label_rot} ({best_rot:.2f}deg)')
        lines.append(f'  Best position: {best_label_pos} ({best_pos:.1f}mm)')
        lines.append('')

        msg = '\n'.join(lines)
        self.get_logger().info('\n' + msg)
        response.success = True
        response.sample_count = len(samples)
        response.message = msg
        return response

    # ── Clear ────────────────────────────────────────────────────────

    def _handle_clear(self, response):
        with self._lock:
            n = len(self._samples)
            self._samples.clear()
            self._color_img = None
        self._touch_sample = None
        self._solved_X = None
        self._refined_p_base = None
        response.success = True
        response.sample_count = 0
        response.message = f'Cleared {n} samples'
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
            refined_p_base = (self._refined_p_base.copy()
                              if self._refined_p_base is not None else None)
            samples = list(self._samples)
            touch_sample = self._touch_sample

        second = samples[0][1] if samples else None
        is_aruco = isinstance(second, np.ndarray) and second.shape == (4, 4)

        if is_aruco:
            # ArUco mode: validate via AX=XB residual
            robot_poses = [s[0] for s in samples]
            marker_poses = [s[1] for s in samples]
            result = solve_hand_eye_ax_xb(robot_poses, marker_poses, min_rot_deg=3.0)
            pos_err = result['residual_pos_mm']
            rot_err = result['residual_rot_deg']
            if pos_err < 0 or pos_err > 15.0:
                response.success = False
                response.residual_pos_mm = float(pos_err)
                response.residual_rot_deg = float(rot_err)
                response.message = (
                    f'REJECTED: pos_err={pos_err:.1f}mm > 15mm '
                    f'or insufficient constraints')
                return response
            residuals = {'rot_deg': round(float(rot_err), 2),
                         'pos_mm': round(float(pos_err), 2)}
        else:
            # Touch-point mode: validate against fixed reference point
            if refined_p_base is None and touch_sample is None:
                response.success = False
                response.message = 'No reference point available'
                return response
            p_ref = refined_p_base if refined_p_base is not None else touch_sample[1]
            errors = []
            for T_W_E, p_cam in samples:
                proj = T_W_E[:3, :3] @ (X[:3, :3] @ p_cam + X[:3, 3]) + T_W_E[:3, 3]
                errors.append(np.linalg.norm(proj - p_ref) * 1000.0)
            mean_err = float(np.mean(errors))
            if mean_err > 15.0:
                response.success = False
                response.residual_pos_mm = mean_err
                response.message = f'REJECTED: mean_err={mean_err:.1f}mm > 15mm'
                return response
            residuals = {'rot_deg': 0, 'pos_mm': round(mean_err, 2)}

        yaml_path = self._find_yaml_path()
        report = save_calibration_to_yaml(X, yaml_path, residuals=residuals,
                                          yaml_obj=yaml)
        self.get_logger().info(report)

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
        self.get_logger().info(f'Published TF: {self.flange_frame} -> {child_frame}')

        response.success = True
        response.sample_count = len(samples)
        response.residual_pos_mm = float(residuals.get('pos_mm', 0))
        response.residual_rot_deg = float(residuals.get('rot_deg', 0))
        response.filepath = yaml_path
        response.message = report
        return response

    # ── Helpers ──────────────────────────────────────────────────────

    def _find_yaml_path(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('vision_grasp')
            return os.path.join(pkg_share, 'config', 'hand_eye.yaml')
        except Exception:
            here = os.path.dirname(os.path.abspath(__file__))
            return os.path.abspath(os.path.join(
                here, '..', '..', '..', '..',
                'src', 'vision_grasp', 'config', 'hand_eye.yaml'))

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
