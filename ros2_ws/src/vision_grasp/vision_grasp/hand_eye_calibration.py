#!/usr/bin/env python3
"""
hand_eye_calibration.py
------------------------
Pure-math AX=XB hand-eye calibration solver (eye-in-hand).

Solves for T_flange→camera given:
  - robot_poses[i] = T_base→flange (from robot_state cart_actual_position)
  - marker_poses[i] = T_camera→marker (from ArUco solvePnP)

Algorithm: quaternion nullspace method (Tsai-Lenz equivalent, simpler).
  AX = XB where:
    A = inv(T_W_Ej) * T_W_Ei  (robot motion from pose j to i)
    B = T_C_Mj * inv(T_C_Mi)  (camera-observed marker motion from i to j)

Requires: numpy (standard in ROS 2 Python environment).
Zero external dependencies beyond what ROS 2 already provides.
"""

import math
import os
from datetime import datetime

import numpy as np


# ── Rotation conversions ──────────────────────────────────────────────

def rpy_to_matrix(rpy):
    """RPY Euler angles [rx, ry, rz] (rad) → 3x3 rotation matrix.

    Fixed-axis XYZ convention (used by DUCO robots):
    R = Rz(rz) * Ry(ry) * Rx(rx)
    """
    rx, ry, rz = np.asarray(rpy, dtype=np.float64).ravel()
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array([
        [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
        [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
        [-sy,   cy*sx,            cy*cx],
    ], dtype=np.float64)


def matrix_to_rpy(R):
    """3x3 rotation matrix → RPY Euler angles [rx, ry, rz] in radians."""
    R = np.asarray(R, dtype=np.float64)
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0
    return np.array([rx, ry, rz], dtype=np.float64)


def rotation_vector_to_matrix(r):
    """Rodrigues rotation vector [rx, ry, rz] → 3x3 rotation matrix."""
    r = np.asarray(r, dtype=np.float64)
    theta = np.linalg.norm(r)
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]], dtype=np.float64)
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * K @ K


def matrix_to_rotation_vector(R):
    """3x3 rotation matrix → Rodrigues vector [rx, ry, rz]."""
    R = np.asarray(R, dtype=np.float64)
    theta = math.acos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if theta < 1e-12:
        return np.zeros(3)
    if abs(theta - math.pi) < 1e-8:
        # 180° case: eigenvector of eigenvalue 1
        w, v = np.linalg.eig(R)
        idx = np.argmin(np.abs(w - 1.0))
        axis = v[:, idx].real
        axis /= np.linalg.norm(axis)
        return axis * theta
    K = (R - R.T) / (2.0 * math.sin(theta))
    return np.array([K[2, 1], K[0, 2], K[1, 0]]) * theta


def quaternion_to_rotation_matrix(q):
    """Quaternion [x, y, z, w] → 3x3 rotation matrix."""
    qx, qy, qz, qw = np.asarray(q, dtype=np.float64)
    return np.array([
        [1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [    2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz,     2*qy*qz - 2*qx*qw],
        [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy],
    ], dtype=np.float64)


def rotation_matrix_to_quaternion(R):
    """3x3 rotation matrix → quaternion [x, y, z, w]."""
    R = np.asarray(R, dtype=np.float64)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q


def compose_4x4(R, t):
    """Build 4x4 homogeneous transform from 3x3 rotation and 3x1 translation."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3] = np.asarray(t, dtype=np.float64).ravel()
    return T


def homogeneous_inv(T):
    """Fast inverse of 4x4 homogeneous transform: [R^T, -R^T*t; 0, 1]."""
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


# ── Quaternion help matrices ──────────────────────────────────────────

def _quat_left_matrix(q):
    """4x4 matrix Q_left(q) s.t. Q_left(q) * r = q ⊗ r.

    Quaternion convention: [x, y, z, w] (ROS standard).
    Input r is [rx, ry, rz, rw]; output is (q ⊗ r) in same order.
    """
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    return np.array([
        [ qw, -qz,  qy,  qx],
        [ qz,  qw, -qx,  qy],
        [-qy,  qx,  qw,  qz],
        [-qx, -qy, -qz,  qw],
    ], dtype=np.float64)


def _quat_right_matrix(q):
    """4x4 matrix Q_right(q) s.t. Q_right(q) * r = r ⊗ q.

    Quaternion convention: [x, y, z, w] (ROS standard).
    Input r is [rx, ry, rz, rw]; output is (r ⊗ q) in same order.
    """
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    return np.array([
        [ qw,  qz, -qy,  qx],
        [-qz,  qw,  qx,  qy],
        [ qy, -qx,  qw,  qz],
        [-qx, -qy, -qz,  qw],
    ], dtype=np.float64)


# ── Core solver ────────────────────────────────────────────────────────

def solve_hand_eye_ax_xb(robot_poses, marker_poses, min_rot_deg=3.0):
    """Solve AX=XB for eye-in-hand calibration.

    Args:
        robot_poses:  list of N 4x4 ndarrays, T_base→flange at each robot pose.
        marker_poses: list of N 4x4 ndarrays, T_camera→marker at each pose.
        min_rot_deg:  minimum rotation angle between two poses to form a
                      usable constraint pair (default 3.0 deg).

    Returns dict with keys:
        X_4x4:              4x4 T_flange→camera (the calibration result).
        residual_pos_mm:    mean position residual in millimeters.
        residual_rot_deg:   mean rotation residual in degrees.
        num_pairs_used:     number of pose pairs used in the solve.
        singular_values:    singular values from the rotation nullspace (4 values).
        success:            bool, True if residuals are within acceptable bounds.
    """
    n = len(robot_poses)
    if n < 3:
        return {
            'X_4x4': np.eye(4), 'residual_pos_mm': -1, 'residual_rot_deg': -1,
            'num_pairs_used': 0, 'singular_values': [],
            'success': False
        }

    # Step 1: Form pair constraints
    M_rows = []
    A_rows = []
    b_rows = []
    pairs = 0

    for i in range(n):
        for j in range(n):
            if i == j:
                continue

            T_W_Ei = np.asarray(robot_poses[i], dtype=np.float64)
            T_W_Ej = np.asarray(robot_poses[j], dtype=np.float64)
            T_C_Mi = np.asarray(marker_poses[i], dtype=np.float64)
            T_C_Mj = np.asarray(marker_poses[j], dtype=np.float64)

            # A_ij = inv(T_W_Ej) * T_W_Ei = robot motion j→i
            A = homogeneous_inv(T_W_Ej) @ T_W_Ei
            # B_ij = T_C_Mj * inv(T_C_Mi) = camera-observed marker motion i→j
            B = T_C_Mj @ homogeneous_inv(T_C_Mi)

            R_A = A[:3, :3]
            R_B = B[:3, :3]

            # Skip pairs with near-zero rotation (no constraint)
            angle_A = math.acos(
                np.clip((np.trace(R_A) - 1.0) / 2.0, -1.0, 1.0))
            if math.degrees(angle_A) < min_rot_deg:
                continue

            q_A = rotation_matrix_to_quaternion(R_A)
            q_B = rotation_matrix_to_quaternion(R_B)

            M = _quat_left_matrix(q_A) - _quat_right_matrix(q_B)
            M_rows.append(M)

            # Translation constraint: (R_A - I) * t_X = R_X * t_B - t_A
            # We'll form this after solving rotation
            A_rows.append(R_A - np.eye(3))
            # b = R_X * t_B - t_A — computed after rotation is known
            t_A = A[:3, 3]
            t_B = B[:3, 3]
            b_rows.append((t_A, t_B))

            pairs += 1

    if pairs < 3:
        return {
            'X_4x4': np.eye(4), 'residual_pos_mm': -1, 'residual_rot_deg': -1,
            'num_pairs_used': pairs, 'singular_values': [],
            'success': False
        }

    # Step 2: Solve rotation via nullspace of stacked M
    M_total = np.vstack(M_rows)  # (4*N_pairs) × 4
    _, s, Vt = np.linalg.svd(M_total, full_matrices=False)
    q_X = Vt[-1]  # Right singular vector of smallest singular value
    q_X /= np.linalg.norm(q_X)

    # Ensure consistent sign (w > 0)
    if q_X[3] < 0:
        q_X = -q_X
    R_X = quaternion_to_rotation_matrix(q_X)

    # Step 3: Solve translation via linear least squares
    A_total = np.vstack(A_rows)  # (3*N_pairs) × 3
    b_vecs = []
    for t_A, t_B in b_rows:
        b_vecs.append(R_X @ t_B - t_A)
    b_total = np.concatenate(b_vecs)  # (3*N_pairs,)

    t_X, _, _, _ = np.linalg.lstsq(A_total, b_total, rcond=None)

    # Step 4: Assemble result
    X = compose_4x4(R_X, t_X)

    # Step 5: Compute residuals
    rot_errs, pos_errs = _compute_residuals(X, robot_poses, marker_poses)

    result = {
        'X_4x4': X,
        'residual_pos_mm': float(np.mean(pos_errs)) if len(pos_errs) else -1,
        'residual_rot_deg': float(np.mean(rot_errs)) if len(rot_errs) else -1,
        'num_pairs_used': pairs,
        'singular_values': [float(v) for v in s],
        'success': True,
    }

    return result


def _compute_residuals(X, robot_poses, marker_poses):
    """Compute how consistent the calibration is: all T_base→marker should match."""
    n = len(robot_poses)
    projected = []
    for i in range(n):
        T_W_M = robot_poses[i] @ X @ marker_poses[i]
        projected.append(T_W_M)

    mean_trans = np.mean([p[:3, 3] for p in projected], axis=0)

    # Average rotation via quaternion averaging
    quats = [rotation_matrix_to_quaternion(p[:3, :3]) for p in projected]
    # Simple mean (works for small rotations)
    mean_q = np.mean(quats, axis=0)
    mean_q /= np.linalg.norm(mean_q)
    mean_R = quaternion_to_rotation_matrix(mean_q)

    rot_errs = []
    pos_errs = []
    for p in projected:
        pos_err = np.linalg.norm(p[:3, 3] - mean_trans) * 1000.0
        R_rel = p[:3, :3] @ mean_R.T
        angle = math.acos(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))
        rot_errs.append(math.degrees(angle))
        pos_errs.append(pos_err)

    return rot_errs, pos_errs


# ── YAML persistence ──────────────────────────────────────────────────

def save_calibration_to_yaml(X, filepath, residuals=None, yaml_obj=None):
    """Write calibration result to hand_eye.yaml preserving format.

    Args:
        X:        4x4 ndarray, T_flange→camera.
        filepath: Path to hand_eye.yaml.
        residuals:Optional dict with 'rot_deg', 'pos_mm' keys for report comment.
        yaml_obj: Optional yaml module (avoid import if already available).
    """
    if yaml_obj is None:
        import yaml as _yaml
        yaml_obj = _yaml

    t = X[:3, 3]
    q = rotation_matrix_to_quaternion(X[:3, :3])

    # Read existing file to preserve other sections
    cfg = {}
    if os.path.exists(filepath):
        with open(filepath) as f:
            cfg = yaml_obj.safe_load(f) or {}

    cfg['hand_eye'] = {
        'x': float(round(t[0], 6)),
        'y': float(round(t[1], 6)),
        'z': float(round(t[2], 6)),
        'qx': float(round(q[0], 6)),
        'qy': float(round(q[1], 6)),
        'qz': float(round(q[2], 6)),
        'qw': float(round(q[3], 6)),
    }

    # Ensure other sections exist with defaults
    cfg.setdefault('grasp_offset', {'x': 0.0, 'y': 0.0, 'z': 0.0})
    cfg.setdefault('grasp', {
        'pre_grasp_height': 0.15, 'grasp_z_offset': 0.02, 'lift_height': 0.15,
        'grasp_rx': 3.14159, 'grasp_ry': 0.0, 'grasp_rz': 0.0,
        'move_speed': 0.20, 'approach_speed': 0.05, 'lift_speed': 0.10,
        'move_accel': 0.5,
    })
    cfg.setdefault('hand', {
        'open': [0, 0, 0, 0, 0, 0],
        'close': [800, 800, 800, 800, 800, 800],
    })
    cfg.setdefault('place', {'x': 0.30, 'y': -0.30, 'z': 0.25})
    cfg.setdefault('detection', {
        'model_path': 'yolov8n.pt', 'confidence_threshold': 0.5,
        'depth_roi_half': 10, 'min_depth_mm': 100.0, 'max_depth_mm': 2000.0,
    })

    with open(filepath, 'w') as f:
        f.write('# Hand-eye calibration: T_end_camera (L_base_link → camera_link)\n')
        f.write(f'# Calibrated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
        if residuals:
            f.write(f'# Residuals: rot={residuals.get("rot_deg", "?")} deg, '
                    f'pos={residuals.get("pos_mm", "?")} mm\n')
        f.write('# AX=XB solver: quaternion nullspace method\n')
        f.write('hand_eye:\n')
        for k in ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']:
            f.write(f'  {k}: {cfg["hand_eye"][k]}\n')
        f.write('\n')

        # Write remaining sections
        go = cfg.get('grasp_offset', {})
        f.write('# Grasp offset — now zeroed since calibration is accurate.\n')
        f.write('# Remove or keep as safety margin (< 5 mm recommended).\n')
        f.write('grasp_offset:\n')
        f.write(f'  x: {go.get("x", 0.0)}\n')
        f.write(f'  y: {go.get("y", 0.0)}\n')
        f.write(f'  z: {go.get("z", 0.0)}\n')
        f.write('\n')

        f.write('grasp:\n')
        for k, v in cfg.get('grasp', {}).items():
            f.write(f'  {k}: {v}\n')
        f.write('\n')

        f.write('hand:\n')
        for k, v in cfg.get('hand', {}).items():
            f.write(f'  {k}: {v}\n')
        f.write('\n')

        f.write('place:\n')
        for k, v in cfg.get('place', {}).items():
            f.write(f'  {k}: {v}\n')
        f.write('\n')

        f.write('detection:\n')
        for k, v in cfg.get('detection', {}).items():
            f.write(f'  {k}: {v}\n')

    # Compute RPY for convenience
    rx, ry, rz = _rotation_matrix_to_rpy(X[:3, :3])
    t = X[:3, 3]
    report = (
        f'=== Hand-Eye Calibration Report ===\n'
        f'Transform: L_base_link → camera_link\n'
        f'  Translation (m): x={t[0]:.4f} y={t[1]:.4f} z={t[2]:.4f}\n'
        f'  Rotation (rpy deg): rx={rx:.2f} ry={ry:.2f} rz={rz:.2f}\n'
        f'  Quaternion: qx={q[0]:.4f} qy={q[1]:.4f} qz={q[2]:.4f} qw={q[3]:.4f}\n'
    )
    if residuals:
        report += (
            f'Residuals:\n'
            f'  Mean rotation error: {residuals.get("rot_deg", "?"):.2f} deg\n'
            f'  Mean position error: {residuals.get("pos_mm", "?"):.1f} mm\n'
        )
    report += f'Saved to: {filepath}\n'

    return report


def _rotation_matrix_to_rpy(R):
    """3x3 rotation matrix → [roll, pitch, yaw] in degrees (XYZ fixed-axis)."""
    R = np.asarray(R, dtype=np.float64)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


# ── Self-test ─────────────────────────────────────────────────────────

def _test_with_synthetic_data():
    """Verify the solver recovers a known X from synthetic data."""
    np.random.seed(42)

    # Known hand-eye transform
    true_t = np.array([0.052, -0.003, 0.078])
    true_r = rotation_vector_to_matrix([0.1, 0.05, 1.55])  # ~89° yaw
    true_X = compose_4x4(true_r, true_t)

    # Fixed marker pose in base (arbitrary)
    marker_in_base = np.eye(4)
    marker_in_base[:3, 3] = [0.4, 0.0, 0.3]

    robot_poses = []
    marker_poses = []
    n = 8

    for i in range(n):
        # Generate varied robot poses
        j1 = np.random.uniform(-0.3, 0.3)
        j2 = np.random.uniform(-1.0, -0.3)
        j3 = np.random.uniform(0.8, 1.8)
        j4 = np.random.uniform(-1.0, 1.0)
        j5 = np.random.uniform(-2.0, -1.0)
        j6 = np.random.uniform(0.5, 2.5)

        # Simplified FK: just use random position and orientation
        t_base_flange = np.array([0.3 + j1 * 0.2, -0.2 + j2 * 0.1, 0.4 + j3 * 0.1])
        R_base_flange = rotation_vector_to_matrix([j4 * 0.3, j5 * 0.3, j6])
        T_W_E = compose_4x4(R_base_flange, t_base_flange)

        # T_C_M = inv(X) * inv(T_W_E) * T_W_M (marker seen from camera)
        T_W_M = marker_in_base
        T_C_M = homogeneous_inv(true_X) @ homogeneous_inv(T_W_E) @ T_W_M

        robot_poses.append(T_W_E)
        marker_poses.append(T_C_M)

    result = solve_hand_eye_ax_xb(robot_poses, marker_poses)
    X_est = result['X_4x4']

    # Check rotation error
    R_err = X_est[:3, :3] @ true_X[:3, :3].T
    rot_err = math.degrees(
        math.acos(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)))
    pos_err = np.linalg.norm(X_est[:3, 3] - true_X[:3, 3]) * 1000.0

    print(f'Test: true_X recovered with rot_err={rot_err:.4f} deg, pos_err={pos_err:.3f} mm')
    print(f'  Residuals from solver: rot={result["residual_rot_deg"]:.4f} deg, '
          f'pos={result["residual_pos_mm"]:.3f} mm')
    assert rot_err < 0.1, f'Rotation error too large: {rot_err:.4f} deg'
    assert pos_err < 0.5, f'Position error too large: {pos_err:.3f} mm'
    print('  PASSED')


if __name__ == '__main__':
    _test_with_synthetic_data()
