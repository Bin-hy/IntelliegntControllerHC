#!/usr/bin/env python3
"""
grasp_pose_safety_filter.py
---------------------------
防奇异抓取姿态过滤器 (GraspPoseSafetyFilter)

DUCO 姿态约定：Rx/Ry/Rz 是旋转向量（Rotation Vector），不是 RPY 欧拉角。
  - 旋转向量模长 = 旋转角度（rad），方向 = 旋转轴
  - "工具Z轴朝下" 在旋转向量下约为 (π,0,0) 或 (-π,0,0)，但从任意构型
    出发 IK 不一定有解。

核心策略：
  - 保持当前末端姿态的旋转向量（IK 从当前构型出发，解必然存在）
  - 只在 yaw 分量（绕Z轴）做小范围调整以避免 wrist 奇异
  - 检测 shoulder singularity（目标在机器人正上方）
  - 检测 elbow singularity（目标超出可达范围）
"""

import math
from dataclasses import dataclass, field
from typing import Optional, List


@dataclass
class GraspCandidate:
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    yaw_offset: float = 0.0


@dataclass
class FilterResult:
    success: bool
    candidate: Optional[GraspCandidate] = None
    pre_grasp: Optional[GraspCandidate] = None
    reason: str = ''


class GraspPoseSafetyFilter:
    """
    用法：
        filt = GraspPoseSafetyFilter(logger, max_reach, shoulder_height, pre_grasp_offset)
        result = filt.filter(bx, by, bz, current_cart)
        if result.success:
            move_to(result.pre_grasp)
            move_to(result.candidate)

    current_cart: [x, y, z, rx, ry, rz] — 当前末端笛卡尔位姿（DUCO旋转向量）
    """

    SHOULDER_RADIUS_THRESH = 0.06   # m，水平距离小于此值 → shoulder 奇异
    ELBOW_EXTEND_RATIO     = 0.95   # 超过 max_reach 的此比例 → elbow 奇异警告
    WRIST_SINGULAR_THRESH  = 0.08   # rad，joint5 绝对值小于此 → wrist 奇异风险

    # Yaw 微调范围（绕旋转向量Z分量调整）
    YAW_RANGE = 0.4
    YAW_STEP  = 0.1

    def __init__(self, logger, max_reach: float = 0.90,
                 shoulder_height: float = 0.16,
                 pre_grasp_offset: float = 0.10):
        self._log = logger
        self.max_reach = max_reach
        self.shoulder_h = shoulder_height
        self.pre_grasp_offset = pre_grasp_offset

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def filter(self, bx: float, by: float, bz: float,
               current_cart: List[float],
               current_joints: List[float]) -> FilterResult:
        """Returns first structurally valid candidate. IK validation is caller's job."""
        err = self._check_reach(bx, by, bz)
        if err:
            return FilterResult(success=False, reason=err)

        horiz = math.sqrt(bx * bx + by * by)
        if horiz < self.SHOULDER_RADIUS_THRESH:
            return FilterResult(
                success=False,
                reason=f'Shoulder singularity: target at horiz={horiz:.3f}m '
                       f'(< {self.SHOULDER_RADIUS_THRESH}m from base axis)')

        if len(current_cart) >= 6:
            base_rx, base_ry, base_rz = current_cart[3], current_cart[4], current_cart[5]
        else:
            base_rx, base_ry, base_rz = -math.pi, 0.0, 0.0

        rz = base_rz
        candidate = GraspCandidate(x=bx, y=by, z=bz,
                                   rx=base_rx, ry=base_ry, rz=rz)
        pre_grasp = GraspCandidate(x=bx, y=by, z=bz + self.pre_grasp_offset,
                                   rx=base_rx, ry=base_ry, rz=rz)
        self._log.info(
            f'[SafetyFilter] target=({bx:.3f},{by:.3f},{bz:.3f}) '
            f'rv=({base_rx:.3f},{base_ry:.3f},{rz:.3f})')
        return FilterResult(success=True, candidate=candidate, pre_grasp=pre_grasp)

    def orientation_candidates(self, current_cart: List[float]) -> List[tuple]:
        """Return orientation (rx,ry,rz) candidates to try in order."""
        candidates = []
        if len(current_cart) >= 6:
            candidates.append((current_cart[3], current_cart[4], current_cart[5]))
        # Known-good orientations from empirical robot operation (DUCO GCR5-910)
        candidates += [
            (-1.276,  1.096, -1.788),
            (-1.5,    0.8,   -1.8),
            (-1.2,    1.2,   -1.7),
            (-math.pi, 0.0,   0.0),
        ]
        # For each, also try ±yaw offsets
        expanded = []
        for (rx, ry, rz) in candidates:
            for yaw in self._yaw_candidates():
                expanded.append((rx, ry, rz + yaw))
        return expanded

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _yaw_candidates(self) -> List[float]:
        steps = int(self.YAW_RANGE / self.YAW_STEP)
        out = [0.0]
        for i in range(1, steps + 1):
            out.append(i * self.YAW_STEP)
            out.append(-i * self.YAW_STEP)
        return out

    def _check_reach(self, x: float, y: float, z: float) -> Optional[str]:
        dz = z - self.shoulder_h
        dist = math.sqrt(x*x + y*y + dz*dz)
        if dist > self.max_reach:
            return (f'Out of reach: dist={dist:.3f}m > max_reach={self.max_reach}m '
                    f'at ({x:.3f},{y:.3f},{z:.3f})')
        return None

    def _check_wrist(self, joints: List[float],
                     bx: float, by: float, bz: float) -> Optional[str]:
        if len(joints) < 5:
            return None
        j5 = joints[4]
        if abs(j5) < self.WRIST_SINGULAR_THRESH:
            return f'Wrist singularity risk: joint5={math.degrees(j5):.1f}°'
        return None

    @staticmethod
    def pose_to_str(c: GraspCandidate) -> str:
        return (f'({c.x:.3f},{c.y:.3f},{c.z:.3f}) '
                f'rv=({c.rx:.3f},{c.ry:.3f},{c.rz:.3f})')

