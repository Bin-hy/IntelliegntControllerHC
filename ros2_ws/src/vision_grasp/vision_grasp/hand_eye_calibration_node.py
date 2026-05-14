#!/usr/bin/env python3
"""
hand_eye_calibration_node.py
-----------------------------
Touch-point based eye-in-hand calibration (translation only).

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
        self._robot_pose = None
        self._robot_cart = None
        self._depth_img = None
        self._camera_info = None
        self._touch_sample = None
        self._samples = []            # list of (T_W_E, p_cam)
        self._solved_X = None
        self._refined_p_base = None

        # Subscribers — only the ones that definitely work
        self._sub_robot = self.create_subscription(
            DucoRobotState, '/duco_cobot/robot_state',
            self._robot_state_cb, 10, callback_group=self.cb_group)

        camera_ns = self.camera_ns.lstrip('/')
        self._sub_depth = self.create_subscription(
            Image, f'/{camera_ns}/depth/image_raw',
            self._depth_cb, 1, callback_group=self.cb_group)
        self._sub_info = self.create_subscription(
            CameraInfo, f'/{camera_ns}/depth/camera_info',
            self._info_cb, 1, callback_group=self.cb_group)

        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self._srv = self.create_service(
            CalibrateHandEye, '/hand_eye_calibration/calibrate',
            self._calibrate_callback, callback_group=self.cb_group)

        self.get_logger().info('HandEyeCalibration ready')

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
            R = rotation_vector_to_matrix([rx, ry, rz])
            self._robot_pose = compose_4x4(R, np.array([x, y, z]))

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

    def _handle_collect(self, request, response):
        if self._robot_pose is None:
            response.success = False
            response.message = 'No robot state'
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
            response.message = 'Click the marker dot on the video stream first'
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

        T_W_E = self._robot_pose.copy()
        with self._lock:
            self._samples.append((T_W_E, p_cam))
            n = len(self._samples)

        self.get_logger().info(
            f'Sample {n}: pixel=({u},{v}) cam=({x:.4f},{y:.4f},{depth_m:.4f}) '
            f'flange=({self._robot_cart[0]:.3f},{self._robot_cart[1]:.3f},{self._robot_cart[2]:.3f})')

        response.success = True
        response.sample_count = n
        response.message = f'Sample {n} recorded'
        return response

    # ── Solve ─────────────────────────────────────────────────────────

    def _handle_solve(self, response):
        with self._lock:
            if self._touch_sample is None:
                response.success = False
                response.message = 'Run "touch" first'
                return response
            if len(self._samples) < 3:
                response.success = False
                response.message = (
                    f'Need at least 3 samples, have {len(self._samples)}')
                return response
            _, p_base = self._touch_sample
            samples = list(self._samples)

        T_W_E_list = [s[0] for s in samples]
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
            f'Solved: t=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f})m '
            f'q=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f}) | '
            f'err={mean_err:.1f}mm shift={p_shift:.1f}mm | {kval}')

        self.get_logger().info(response.message)
        for i, e in enumerate(errors):
            self.get_logger().info(f'  Sample {i+1}: err={e:.1f}mm')
        return response

    # ── Clear ────────────────────────────────────────────────────────

    def _handle_clear(self, response):
        with self._lock:
            n = len(self._samples)
            self._samples.clear()
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

        p_ref = refined_p_base if refined_p_base is not None else self._touch_sample[1]
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

        yaml_path = self._find_yaml_path()
        residuals = {'rot_deg': 0, 'pos_mm': round(mean_err, 2)}
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
        response.residual_pos_mm = mean_err
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
