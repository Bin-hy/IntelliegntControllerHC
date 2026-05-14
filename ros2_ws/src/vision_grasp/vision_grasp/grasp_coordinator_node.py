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
from duco_msg.srv import RobotMove, RobotControl
from duco_msg.msg import DucoRobotState
from lhandpro_interfaces.srv import SetAllPosition, MoveMotors

from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401

from vision_grasp.grasp_pose_safety_filter import GraspPoseSafetyFilter


class GraspCoordinatorNode(Node):

    def __init__(self):
        super().__init__('grasp_coordinator')
        self.cb_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameter('camera_ns', '/camera')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pre_grasp_height', 0.15)
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
        self.declare_parameter('grasp_offset_x', 0.0)
        self.declare_parameter('grasp_offset_y', 0.0)
        self.declare_parameter('grasp_offset_z', 0.0)
        self.declare_parameter('fingertip_x', 0.0)
        self.declare_parameter('fingertip_y', 0.0)
        self.declare_parameter('fingertip_z', 0.12)
        self._load_params()

        # --- Safety filter ---
        self._safety_filter = GraspPoseSafetyFilter(
            logger=self.get_logger(),
            max_reach=self.max_reach,
            shoulder_height=self.shoulder_h,
            pre_grasp_offset=self.pre_grasp_h,
        )

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Service clients ---
        self.cli_detect = self.create_client(
            DetectBottle, '/bottle_detector/detect', callback_group=self.cb_group)
        self.cli_move = self.create_client(
            RobotMove, '/duco_robot/robot_move', callback_group=self.cb_group)
        self.cli_control = self.create_client(
            RobotControl, '/duco_robot/robot_control', callback_group=self.cb_group)
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
        self._current_cart = []
        self._robot_state = -1  # robot_state field from DucoRobotState
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            DucoRobotState, '/duco_cobot/robot_state', self._robot_state_cb, 10,
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
        self.grasp_offset_x = self.get_parameter('grasp_offset_x').value
        self.grasp_offset_y = self.get_parameter('grasp_offset_y').value
        self.grasp_offset_z = self.get_parameter('grasp_offset_z').value
        self.fingertip_x = self.get_parameter('fingertip_x').value
        self.fingertip_y = self.get_parameter('fingertip_y').value
        self.fingertip_z = self.get_parameter('fingertip_z').value

    def _call_sync(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'Service {client.srv_name} not available')
        future = client.call_async(request)
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                future.cancel()
                raise RuntimeError(f'Service {client.srv_name} timed out after {timeout}s')
            # Abort immediately if robot was emergency-stopped or disabled
            if self._robot_state not in (-1, 6):
                future.cancel()
                raise RuntimeError(
                    f'Motion aborted: robot_state={self._robot_state} '
                    f'(e-stop or disabled) during {client.srv_name}')
            time.sleep(0.02)
        return future.result()

    # Pre-defined q_near seeds for different arm configurations.
    # joint 6 (index 5) = π/2 (1.5708 rad) — natural working orientation
    # where camera and gripper are correctly aligned.
    IK_SEEDS = [
        # Seed 1: front-low reach (elbow up)
        [0.0,   -0.5,  1.2,  -0.7, -1.57,  1.5708],
        # Seed 2: front-mid reach
        [0.0,   -0.8,  1.5,  -0.7, -1.57,  1.5708],
        # Seed 3: front-high reach
        [0.0,   -1.0,  1.0,  -0.5, -1.57,  1.5708],
        # Seed 4: slight right rotation
        [0.3,   -0.7,  1.4,  -0.8, -2.0,   1.5708],
        # Seed 5: slight left rotation
        [-0.3,  -0.7,  1.4,  -0.8, -2.0,   1.5708],
        # Seed 6: elbow down config
        [0.0,    0.5, -1.2,   0.7, -1.57,  1.5708],
    ]

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

    def _robot_state_cb(self, msg):
        if len(msg.cart_actual_position) >= 6:
            self._current_cart = list(msg.cart_actual_position[:6])
        if hasattr(msg, 'robot_state'):
            self._robot_state = int(msg.robot_state)

    def _check_reach(self, x, y, z):
        dz = z - self.shoulder_h
        dist = math.sqrt(x * x + y * y + dz * dz)
        if dist > self.max_reach:
            raise RuntimeError(
                f'Target ({x:.3f},{y:.3f},{z:.3f}) shoulder_dist={dist:.3f}m exceeds max_reach={self.max_reach}m')

    def _movel(self, x, y, z, rx, ry, rz, speed, accel=None):
        if len(self._current_joints) < 6:
            raise RuntimeError('No joint states received yet — cannot seed movel')
        req = RobotMove.Request()
        req.command = 'movel'
        req.arm_num = 0
        req.p = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        req.q = [float(j) for j in self._current_joints]
        req.v = float(speed)
        req.a = float(accel or self.move_accel)
        req.r = 0.0
        req.tool = 'default'
        req.wobj = 'default'
        req.block = True
        result = self._call_sync(self.cli_move, req, timeout=30.0)
        code = result.response.strip()
        if code == '4':
            self.get_logger().info(f'movel OK → ({x:.3f}, {y:.3f}, {z:.3f})')
            return
        # Don't attempt fallback if robot is in protective stop / e-stop
        if self._robot_state != 6:
            raise RuntimeError(
                f'movel failed (code={code}) and robot stopped '
                f'(state={self._robot_state}, likely safety zone violation). '
                f'Check robot safety zone config for target ({x:.3f},{y:.3f},{z:.3f})')
        self.get_logger().warn(f'movel failed (code={code}), falling back to movejpose')
        self._movej_cart(x, y, z, rx, ry, rz, speed, accel)

    def _movej_cart(self, x, y, z, rx, ry, rz, speed, accel=None):
        """Move to Cartesian target using movejpose with multi-seed IK retry.

        movejpose velocity unit: % of system max (0-100], NOT m/s.
        Convert: assume max Cartesian speed ~1.0 m/s → multiply by 100.
        Clamp to [5, 80] to keep motion safe.
        """
        if len(self._current_joints) < 6:
            raise RuntimeError('No joint states received yet — cannot seed movejpose')

        # Convert m/s → percentage for movejpose
        v_pct = float(max(5.0, min(80.0, speed * 100.0)))
        a_pct = float(max(5.0, min(80.0, (accel or self.move_accel) * 100.0)))

        # Build seed list: current joints first, then pre-defined configs
        seeds = [list(self._current_joints)] + self.IK_SEEDS
        last_code = '?'

        for i, seed in enumerate(seeds):
            req = RobotMove.Request()
            req.command = 'movejpose'
            req.arm_num = 0
            req.p = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
            req.q = [float(j) for j in seed]
            req.v = v_pct
            req.a = a_pct
            req.r = 0.0
            req.tool = 'default'
            req.wobj = 'default'
            req.block = True
            result = self._call_sync(self.cli_move, req, timeout=30.0)
            last_code = result.response.strip()
            if last_code == '4':
                self.get_logger().info(
                    f'movejpose OK (seed {i}) → ({x:.3f},{y:.3f},{z:.3f})')
                return
            self.get_logger().warn(
                f'movejpose seed {i} failed (code={last_code}), trying next seed...')

        raise RuntimeError(
            f'movejpose failed with all {len(seeds)} seeds (last code={last_code})')

    def _robot_control(self, command):
        """Send a robot control command (enable/disable/poweron/poweroff)."""
        req = RobotControl.Request()
        req.command = command
        req.arm_num = 0
        req.block = False
        result = self._call_sync(self.cli_control, req, timeout=5.0)
        self.get_logger().info(f'robot_control {command} → {result.response}')

    def _movej(self, joints, speed=1.0, accel=1.0, timeout=20.0):
        """Joint-space move via movej2 (rad/s), bypasses IK."""
        req = RobotMove.Request()
        req.command = 'movej2'
        req.arm_num = 0
        req.q = [float(j) for j in joints]
        req.p = []
        req.v = float(speed)   # rad/s
        req.a = float(accel)   # rad/s²
        req.r = 0.0
        # movej2 does NOT use tool/wobj
        req.tool = ''
        req.wobj = ''
        req.block = True
        result = self._call_sync(self.cli_move, req, timeout=timeout)
        code = result.response.strip()
        if code not in ('4', '5'):  # ST_Finished or ST_Interrupt both OK
            raise RuntimeError(f'movej2 failed (code={code})')
        self.get_logger().info(f'movej2 OK → {[f"{j:.3f}" for j in joints]}')

    def _escape_singularity(self):
        """Move to a known safe non-singular home pose via joint-space."""
        # joint 6 = π/2 (1.5708 rad) — natural orientation for camera+gripper alignment
        safe_joints = [0.0, -0.8, 1.4, -0.6, -1.57, 1.5708]

        # If not enabled, send enable and wait for robot_state == 6 (SR_Enable)
        STATE_ENABLE = 6
        if self._robot_state != STATE_ENABLE:
            self.get_logger().info(
                f'[escape] robot_state={self._robot_state}, sending enable...')
            self._robot_control('enable')
            # Poll up to 5s for enable to take effect
            deadline = time.time() + 5.0
            while time.time() < deadline:
                time.sleep(0.2)
                if self._robot_state == STATE_ENABLE:
                    break
            if self._robot_state != STATE_ENABLE:
                raise RuntimeError(
                    f'Robot not enabled after 5s (state={self._robot_state}). '
                    'Please enable manually via UI.')

        self.get_logger().info('[escape] Moving to safe home via movej...')
        self._movej(safe_joints, speed=0.5, timeout=20.0)

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
        p.header.stamp = rclpy.time.Time().to_msg()
        p.point.x = x_cam
        p.point.y = y_cam
        p.point.z = z_cam
        p_base = self.tf_buffer.transform(
            p, self.base_frame, timeout=rclpy.duration.Duration(seconds=2.0))

        # Debug: also transform to L_base_link (hand flange) to see intermediate
        try:
            p_flange = self.tf_buffer.transform(
                p, 'L_base_link', timeout=rclpy.duration.Duration(seconds=2.0))
            self.get_logger().info(
                f'  [DEBUG TF] camera_opt→L_base_link: '
                f'({p_flange.point.x:.3f}, {p_flange.point.y:.3f}, {p_flange.point.z:.3f})')
        except Exception:
            pass

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

        # Debug: current robot cartesian pose
        if len(self._current_cart) >= 6:
            self.get_logger().info(
                f'  [DEBUG] current L_base_link cart: '
                f'pos=({self._current_cart[0]:.3f},{self._current_cart[1]:.3f},{self._current_cart[2]:.3f}) '
                f'rpy=({self._current_cart[3]:.3f},{self._current_cart[4]:.3f},{self._current_cart[5]:.3f})')

        # Step 1: Detect FIRST while camera is still at the position user was looking from.
        # eye-in-hand: detection MUST happen before any robot motion.
        self.get_logger().info('[1/7] Locating target (camera in current pose)...')
        det_req = DetectBottle.Request()
        det_req.camera_ns = self.camera_ns
        det_req.select_u = float(select_u)
        det_req.select_v = float(select_v)
        det_result = self._call_sync(self.cli_detect, det_req, timeout=15.0)
        if not det_result.success:
            raise RuntimeError(f'Detection failed: {det_result.message}')
        self.get_logger().info(
            f'  Camera frame: ({det_result.x:.3f}, {det_result.y:.3f}, {det_result.z:.3f})')

        # Step 2: Transform to base frame NOW (robot hasn't moved yet, TF is valid).
        self.get_logger().info('[2/7] Transforming to base frame...')
        bx, by, bz = self._transform_to_base(det_result.x, det_result.y, det_result.z)

        # --- Fingertip offset compensation ---
        # The robot moves the FLANGE (L_base_link) to the target, but we need
        # the FINGERTIPS at the target. Fingertips are offset from flange in
        # L_base_link frame (DH116: ~120mm in +Z direction from L_base_link).
        ft_x = self.get_parameter('fingertip_x').value
        ft_y = self.get_parameter('fingertip_y').value
        ft_z = self.get_parameter('fingertip_z').value
        if ft_x or ft_y or ft_z:
            p_ft = PointStamped()
            p_ft.header.frame_id = 'L_base_link'
            p_ft.header.stamp = rclpy.time.Time().to_msg()
            p_ft.point.x = float(ft_x)
            p_ft.point.y = float(ft_y)
            p_ft.point.z = float(ft_z)
            try:
                p_ft_base = self.tf_buffer.transform(
                    p_ft, self.base_frame,
                    timeout=rclpy.duration.Duration(seconds=2.0))
                bx -= p_ft_base.point.x
                by -= p_ft_base.point.y
                bz -= p_ft_base.point.z
                self.get_logger().info(
                    f'  Fingertip offset L_base_link({ft_x:.3f},{ft_y:.3f},{ft_z:.3f})'
                    f' → base({p_ft_base.point.x:.3f},{p_ft_base.point.y:.3f},{p_ft_base.point.z:.3f})')
            except Exception as e:
                self.get_logger().warn(f'Failed to transform fingertip offset: {e}')

        # Read offset parameters fresh each call so UI changes take effect immediately
        ox = self.get_parameter('grasp_offset_x').value
        oy = self.get_parameter('grasp_offset_y').value
        oz = self.get_parameter('grasp_offset_z').value
        bx += ox; by += oy; bz += oz
        if ox or oy or oz:
            self.get_logger().info(
                f'  Offset applied: dx={ox:.3f} dy={oy:.3f} dz={oz:.3f}')
        self.get_logger().info(f'  Base frame: ({bx:.3f}, {by:.3f}, {bz:.3f})')

        # Step 2b: Safety filter (uses current cart/joints before escape)
        self.get_logger().info('[2b/7] Applying grasp pose safety filter...')
        filt = self._safety_filter.filter(bx, by, bz,
                                           list(self._current_cart),
                                           list(self._current_joints))
        if not filt.success:
            raise RuntimeError(f'SafetyFilter rejected pose: {filt.reason}')

        g  = filt.candidate
        pg = filt.pre_grasp
        grasp_z = g.z + self.grasp_z_off
        lift_z  = g.z + self.lift_h

        # Reach check on grasp point
        self._check_reach(g.x, g.y, grasp_z)

        # Clamp pre-grasp z within max_reach (DUCO reports out-of-workspace as "singular")
        dxy = math.sqrt(g.x * g.x + g.y * g.y)
        dz_avail = math.sqrt(max(0.0, self.max_reach ** 2 - dxy ** 2))
        max_pg_z = self.shoulder_h + dz_avail - 0.03
        pg_z = min(pg.z, max_pg_z)
        pg_z = max(pg_z, grasp_z + 0.05)
        if pg_z < pg.z:
            self.get_logger().info(
                f'  Pre-grasp z clamped {pg.z:.3f}→{pg_z:.3f}m (max_reach limit)')

        self.get_logger().info(
            f'  Pre-grasp: ({pg.x:.3f},{pg.y:.3f},{pg_z:.3f}) '
            f'Grasp z={grasp_z:.3f}, Lift z={lift_z:.3f}')

        # Step 0: Escape singularity AFTER detection — robot moves, but target
        # coordinates are already captured in base frame (which is fixed).
        self.get_logger().info('[0/7] Escaping singularity to safe home...')
        self._escape_singularity()

        # Step 3-7: Grasp sequence
        self.get_logger().info('[3/7] Opening hand...')
        self._set_hand(self.hand_open)

        self.get_logger().info('[4/7] Pre-grasp (trying orientation candidates)...')
        orient_list = self._safety_filter.orientation_candidates(list(self._current_cart))
        pre_grasp_ok = False
        chosen_rx, chosen_ry, chosen_rz = g.rx, g.ry, g.rz
        for i, (rx, ry, rz) in enumerate(orient_list):
            try:
                self._movej_cart(pg.x, pg.y, pg_z, rx, ry, rz, self.move_speed)
                chosen_rx, chosen_ry, chosen_rz = rx, ry, rz
                pre_grasp_ok = True
                self.get_logger().info(
                    f'  Pre-grasp OK with orientation candidate {i}: '
                    f'rv=({rx:.3f},{ry:.3f},{rz:.3f})')
                break
            except RuntimeError as e:
                self.get_logger().warn(f'  Orientation {i} failed: {e}')
        if not pre_grasp_ok:
            raise RuntimeError('Pre-grasp failed with all orientation candidates')

        self.get_logger().info('[5/7] Approaching...')
        self._movel(g.x, g.y, grasp_z, chosen_rx, chosen_ry, chosen_rz, self.approach_speed)

        self.get_logger().info('[6/7] Grasping...')
        self._set_hand(self.hand_close)

        self.get_logger().info('[7/7] Lifting...')
        lift_z_safe = min(lift_z, max_pg_z)
        self._movel(g.x, g.y, lift_z_safe, chosen_rx, chosen_ry, chosen_rz, self.lift_speed)

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
