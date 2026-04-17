#!/usr/bin/env python3
"""
grasp_coordinator_node.py
-------------------------
Orchestrates vision-guided bottle grasping:
  1. Receive user-selected pixel (u, v) via TriggerGrasp service
  2. Call detector with those coordinates → 3D position in camera frame
  3. TF transform camera_link → base_link
  4. Execute grasp sequence: open hand → pre-grasp → approach → close → lift
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from common_msgs.srv import DetectBottle, TriggerGrasp
from duco_msg.srv import RobotMove
from lhandpro_interfaces.srv import SetAllPosition, MoveMotors

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401


class GraspCoordinatorNode(Node):

    def __init__(self):
        super().__init__('grasp_coordinator_node')
        self.cb_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameter('camera_ns', '/camera')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pre_grasp_height', 0.08)
        self.declare_parameter('grasp_z_offset', 0.02)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('grasp_rx', math.pi)
        self.declare_parameter('grasp_ry', 0.0)
        self.declare_parameter('grasp_rz', 0.0)
        self.declare_parameter('move_speed', 0.20)
        self.declare_parameter('approach_speed', 0.05)
        self.declare_parameter('lift_speed', 0.10)
        self.declare_parameter('move_accel', 0.5)
        self.declare_parameter('hand_open',  [0, 0, 0, 0, 0, 0])
        self.declare_parameter('hand_close', [800, 800, 800, 800, 800, 800])
        self.declare_parameter('place_x', 0.3)
        self.declare_parameter('place_y', -0.3)
        self.declare_parameter('place_z', 0.25)
        self.declare_parameter('max_reach', 0.90)
        self.declare_parameter('shoulder_height', 0.16)
        self._load_params()

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Service clients ---
        self.cli_detect = self.create_client(
            DetectBottle, '/bottle_detector/detect', callback_group=self.cb_group)
        self.cli_move = self.create_client(
            RobotMove, '/duco_robot/robot_move', callback_group=self.cb_group)
        self.cli_hand_pos = self.create_client(
            SetAllPosition, '/lhandpro_service/set_all_position', callback_group=self.cb_group)
        self.cli_hand_move = self.create_client(
            MoveMotors, '/lhandpro_service/move_motors', callback_group=self.cb_group)

        # --- Trigger service (accepts user-selected pixel) ---
        self.srv_trigger = self.create_service(
            TriggerGrasp, '/grasp_coordinator/trigger_grasp',
            self.trigger_callback, callback_group=self.cb_group)

        # --- Subscribe to joint states for movel IK seed ---
        from sensor_msgs.msg import JointState
        self._current_joints = []
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10,
            callback_group=self.cb_group)

        self.get_logger().info('GraspCoordinatorNode ready')

    def _load_params(self):
        self.camera_ns = self.get_parameter('camera_ns').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pre_grasp_h = self.get_parameter('pre_grasp_height').value
        self.grasp_z_off = self.get_parameter('grasp_z_offset').value
        self.lift_h = self.get_parameter('lift_height').value
        self.grasp_rx = self.get_parameter('grasp_rx').value
        self.grasp_ry = self.get_parameter('grasp_ry').value
        self.grasp_rz = self.get_parameter('grasp_rz').value
        self.move_speed = self.get_parameter('move_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.lift_speed = self.get_parameter('lift_speed').value
        self.move_accel = self.get_parameter('move_accel').value
        self.hand_open = self.get_parameter('hand_open').value
        self.hand_close = self.get_parameter('hand_close').value
        self.place_x = self.get_parameter('place_x').value
        self.place_y = self.get_parameter('place_y').value
        self.place_z = self.get_parameter('place_z').value
        self.max_reach = self.get_parameter('max_reach').value
        self.shoulder_h = self.get_parameter('shoulder_height').value

    def _call_sync(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'Service {client.srv_name} not available')
        future = client.call_async(request)
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                future.cancel()
                raise RuntimeError(f'Service {client.srv_name} timed out')
            time.sleep(0.02)
        return future.result()

    # Arm joint names published by DucoRobotStatus (must match URDF)
    ARM_JOINT_NAMES = [
        'arm_1_joint_1', 'arm_1_joint_2', 'arm_1_joint_3',
        'arm_1_joint_4', 'arm_1_joint_5', 'arm_1_joint_6',
    ]

    def _joint_states_cb(self, msg):
        # Filter by joint name to avoid mixing up arm joints with hand joints.
        # Both DucoRobotStatus (arm) and lhandpro_state_publisher (hand)
        # publish to /joint_states; taking position[:6] blindly could grab
        # finger values (often 0) instead of arm values → bad IK seed.
        if not msg.name:
            return
        try:
            indices = [msg.name.index(n) for n in self.ARM_JOINT_NAMES]
            self._current_joints = [msg.position[i] for i in indices]
        except (ValueError, IndexError):
            pass  # message doesn't contain arm joints — skip

    def _check_reach(self, x, y, z):
        dz = z - self.shoulder_h
        dist = math.sqrt(x * x + y * y + dz * dz)
        if dist > self.max_reach:
            raise RuntimeError(
                f'Target ({x:.3f},{y:.3f},{z:.3f}) shoulder_dist={dist:.3f}m exceeds max_reach={self.max_reach}m')

    def _movel(self, x, y, z, rx, ry, rz, speed, accel=None):
        if len(self._current_joints) < 6:
            raise RuntimeError('No joint states received yet — cannot seed movel')
        self._check_reach(x, y, z)
        req = RobotMove.Request()
        req.command = 'movel'
        req.arm_num = 0
        req.p = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        req.q = [float(j) for j in self._current_joints]
        req.v = float(speed)
        req.a = float(accel or self.move_accel)
        req.r = 0.0
        req.block = True
        result = self._call_sync(self.cli_move, req, timeout=30.0)
        code = result.response.strip()
        if code == '4':
            self.get_logger().info(f'movel OK → ({x:.3f}, {y:.3f}, {z:.3f})')
            return
        # movel failed (code 6 = singularity, 7 = illegal) — try movej as fallback
        self.get_logger().warn(
            f'movel failed (code={code}), falling back to movej')
        self._movej_cart(x, y, z, rx, ry, rz, speed, accel)

    def _movej_cart(self, x, y, z, rx, ry, rz, speed, accel=None):
        """Move to Cartesian target using movejpose (joint-space path, avoids singularity)."""
        if len(self._current_joints) < 6:
            raise RuntimeError('No joint states received yet — cannot seed movejpose')
        req = RobotMove.Request()
        req.command = 'movejpose'
        req.arm_num = 0
        req.p = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        req.q = [float(j) for j in self._current_joints]
        req.v = float(speed)
        req.a = float(accel or self.move_accel)
        req.r = 0.0
        req.block = True
        result = self._call_sync(self.cli_move, req, timeout=30.0)
        code = result.response.strip()
        if code != '4':
            raise RuntimeError(f'movejpose also failed (code={code})')
        self.get_logger().info(f'movejpose OK → ({x:.3f}, {y:.3f}, {z:.3f})')

    def _set_hand(self, positions):
        req_pos = SetAllPosition.Request()
        req_pos.positions = [int(p) for p in positions]
        self._call_sync(self.cli_hand_pos, req_pos, timeout=10.0)
        req_move = MoveMotors.Request()
        req_move.joint_id = 0
        self._call_sync(self.cli_hand_move, req_move, timeout=10.0)
        time.sleep(1.0)
        self.get_logger().info(f'Hand → {positions}')

    def _transform_to_base(self, x_cam, y_cam, z_cam):
        p = PointStamped()
        p.header.frame_id = self.camera_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.point.x = x_cam
        p.point.y = y_cam
        p.point.z = z_cam
        p_base = self.tf_buffer.transform(
            p, self.base_frame, timeout=rclpy.duration.Duration(seconds=2.0))
        return p_base.point.x, p_base.point.y, p_base.point.z

    def trigger_callback(self, request, response):
        try:
            self._execute_grasp(request.u, request.v)
            response.success = True
            response.message = 'Grasp completed'
        except Exception as e:
            self.get_logger().error(f'Grasp failed: {e}')
            response.success = False
            response.message = str(e)
        return response

    def _execute_grasp(self, select_u, select_v):
        self.get_logger().info(f'=== Grasp at pixel ({select_u:.0f}, {select_v:.0f}) ===')

        # Step 1: Detect / locate target
        self.get_logger().info('[1/7] Locating target...')
        det_req = DetectBottle.Request()
        det_req.camera_ns = self.camera_ns
        det_req.select_u = float(select_u)
        det_req.select_v = float(select_v)
        det_result = self._call_sync(self.cli_detect, det_req, timeout=15.0)
        if not det_result.success:
            raise RuntimeError(f'Detection failed: {det_result.message}')
        self.get_logger().info(
            f'  Camera frame: ({det_result.x:.3f}, {det_result.y:.3f}, {det_result.z:.3f})')

        # Step 2: Transform to base frame
        self.get_logger().info('[2/7] Transforming to base frame...')
        bx, by, bz = self._transform_to_base(det_result.x, det_result.y, det_result.z)
        self.get_logger().info(f'  Base frame: ({bx:.3f}, {by:.3f}, {bz:.3f})')

        rx, ry, rz = self.grasp_rx, self.grasp_ry, self.grasp_rz

        # Adaptive tilt: when target is far from base, tilt end-effector inward
        # to help IK solve. Max tilt ~15° at max_reach.
        horiz = math.sqrt(bx * bx + by * by)
        tilt_threshold = 0.55  # start tilting beyond this distance
        if horiz > tilt_threshold:
            tilt_ratio = min((horiz - tilt_threshold) / (self.max_reach - tilt_threshold), 1.0)
            max_tilt = math.radians(15)
            tilt = tilt_ratio * max_tilt
            # Tilt toward base origin: adjust rx/ry based on direction
            angle_to_base = math.atan2(-by, -bx)
            rx = rx - tilt * math.cos(angle_to_base)
            ry = ry - tilt * math.sin(angle_to_base)
            self.get_logger().info(
                f'  Adaptive tilt: horiz={horiz:.3f}m, tilt={math.degrees(tilt):.1f}deg, rx={rx:.3f}, ry={ry:.3f}')

        # Compute pre-grasp: retract along horizontal direction toward base
        # This keeps z the same (avoids pushing z higher) and moves closer to base
        if horiz > 0.01:
            retract = self.pre_grasp_h  # reuse param as retract distance
            ratio = retract / horiz
            pre_bx = bx + (0 - bx) * ratio  # move toward base (0,0)
            pre_by = by + (0 - by) * ratio
        else:
            pre_bx, pre_by = bx, by
        pre_bz = bz + 0.03  # just slightly above target

        grasp_z = bz + self.grasp_z_off
        lift_z = bz + self.lift_h

        self.get_logger().info(
            f'  Pre-grasp: ({pre_bx:.3f}, {pre_by:.3f}, {pre_bz:.3f}), '
            f'Grasp: z={grasp_z:.3f}, Lift: z={lift_z:.3f}')

        # Step 3-7: Grasp sequence
        self.get_logger().info('[3/7] Opening hand...')
        self._set_hand(self.hand_open)

        self.get_logger().info('[4/7] Pre-grasp...')
        self._movel(pre_bx, pre_by, pre_bz, rx, ry, rz, self.move_speed)

        self.get_logger().info('[5/7] Approaching...')
        self._movel(bx, by, grasp_z, rx, ry, rz, self.approach_speed)

        self.get_logger().info('[6/7] Grasping...')
        self._set_hand(self.hand_close)

        self.get_logger().info('[7/7] Lifting...')
        self._movel(bx, by, lift_z, rx, ry, rz, self.lift_speed)

        self.get_logger().info('=== Grasp completed ===')


def main(args=None):
    rclpy.init(args=args)
    node = GraspCoordinatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
