#!/usr/bin/env python3
"""
fake_obstacle_publisher.py — 碰撞检测系统测试工具

功能：发布一个假的 PointCloud2，模拟一个障碍物从远处逼近机械臂。
无需相机、无需真实机械臂，只需 TF 就能测试完整的检测流水线。

使用方式：
  # 终端1: 启动 TF (提供 base_link ~ link_6 的变换)
  ros2 launch collision_detector test_tf.launch.py

  # 终端2: 启动碰撞检测节点
  ros2 launch collision_detector collision.launch.py point_cloud_topic:=/fake/depth/points

  # 终端3: 发布假障碍物
  python3 fake_obstacle_publisher.py

  # 终端4 (可选): 监听碰撞状态
  ros2 topic echo /collision_detector/status

测试场景：
  --mode approach : 障碍物从 50cm 逐渐逼近到 1cm (默认)
  --mode static   : 障碍物静止在指定距离
  --mode sweep    : 障碍物左右扫过工作空间
"""

import argparse
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def make_cloud(frame_id: str, points: list[tuple[float, float, float]], stamp=None) -> PointCloud2:
    """
    从 [(x,y,z), ...] 列表构造 PointCloud2 消息。
    为什么手动构造而不用 pcl/sensor_msgs_py:
      - 不引入额外依赖
      - 测试脚本需要精确控制每个点的位置
    """
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    if stamp:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12  # 3 * float32
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    data = bytearray()
    for x, y, z in points:
        data.extend(struct.pack('<fff', x, y, z))
    msg.data = bytes(data)
    return msg


class FakeObstaclePublisher(Node):
    def __init__(self, args):
        super().__init__('fake_obstacle_publisher')
        self.mode = args.mode
        self.target_link_offset = [args.offset_x, args.offset_y, args.offset_z]
        self.start_distance = args.start_dist
        self.end_distance = args.end_dist
        self.speed = args.speed
        self.frame_id = args.frame

        # 使用 SensorDataQoS: BEST_EFFORT，与 collision_detector 的订阅匹配
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub = self.create_publisher(PointCloud2, args.topic, qos)
        self.timer = self.create_timer(1.0 / args.hz, self.publish_cloud)

        self.t = 0.0
        self.current_distance = self.start_distance
        self.get_logger().info(
            f'假障碍物发布器启动\n'
            f'  模式: {self.mode}\n'
            f'  话题: {args.topic}\n'
            f'  坐标系: {self.frame_id}\n'
            f'  偏移(模拟link位置): ({args.offset_x}, {args.offset_y}, {args.offset_z})\n'
            f'  距离范围: {self.start_distance}m → {self.end_distance}m\n'
            f'  速度: {self.speed} m/s'
        )

    def publish_cloud(self):
        now = self.get_clock().now().to_msg()

        if self.mode == 'approach':
            # 障碍物从远处匀速逼近
            self.current_distance -= self.speed / 30.0  # 每帧���动
            if self.current_distance < self.end_distance:
                self.current_distance = self.start_distance  # 循环
                self.get_logger().info('障碍物重置到起始位置，重新逼近...')

            # 在 link 偏移方向上放置障碍物点
            ox, oy, oz = self.target_link_offset
            # 障碍物沿 X 轴方向接近
            points = [
                (ox + self.current_distance, oy, oz),
                (ox + self.current_distance + 0.01, oy + 0.01, oz),
                (ox + self.current_distance - 0.01, oy - 0.01, oz),
            ]
            dist_cm = self.current_distance * 100
            if dist_cm <= 3:
                level = '🔴 EMERGENCY'
            elif dist_cm <= 10:
                level = '🟠 WARNING'
            elif dist_cm <= 20:
                level = '🟡 CAUTION'
            else:
                level = '🟢 SAFE'
            self.get_logger().info(f'{level}  距离: {dist_cm:.1f}cm')

        elif self.mode == 'static':
            ox, oy, oz = self.target_link_offset
            d = self.start_distance
            points = [
                (ox + d, oy, oz),
                (ox + d, oy + 0.02, oz),
                (ox + d, oy, oz + 0.02),
            ]

        elif self.mode == 'sweep':
            # 障碍物沿 Y 轴左右扫动
            self.t += 1.0 / 30.0
            ox, oy, oz = self.target_link_offset
            y_offset = 0.3 * math.sin(self.t * 2.0)
            d = self.start_distance
            points = [
                (ox + d, oy + y_offset, oz),
                (ox + d, oy + y_offset + 0.01, oz),
                (ox + d, oy + y_offset - 0.01, oz),
            ]
        else:
            points = [(0.5, 0.0, 0.5)]

        # 添加一些背景噪声点（模拟真实点云中的工作台等）
        for i in range(50):
            bx = 0.3 + (i % 10) * 0.05
            by = -0.25 + (i // 10) * 0.1
            bz = -0.02  # 略低于地面
            points.append((bx, by, bz))

        cloud = make_cloud(self.frame_id, points, now)
        self.pub.publish(cloud)


def main():
    parser = argparse.ArgumentParser(description='碰撞检测测试 — 假障碍物发布器')
    parser.add_argument('--mode', choices=['approach', 'static', 'sweep'],
                        default='approach', help='测试模式')
    parser.add_argument('--topic', default='/fake/depth/points',
                        help='发布的点云话题')
    parser.add_argument('--frame', default='base_link',
                        help='点云坐标系 (需要与 TF 树中的 frame 一致)')
    parser.add_argument('--offset-x', type=float, default=0.3,
                        help='目标 link 大致 X 坐标')
    parser.add_argument('--offset-y', type=float, default=0.0,
                        help='目标 link 大致 Y 坐标')
    parser.add_argument('--offset-z', type=float, default=0.4,
                        help='目标 link 大致 Z 坐标')
    parser.add_argument('--start-dist', type=float, default=0.50,
                        help='起始距离 (m)')
    parser.add_argument('--end-dist', type=float, default=0.01,
                        help='最终距离 (m)')
    parser.add_argument('--speed', type=float, default=0.05,
                        help='逼近速度 (m/s)')
    parser.add_argument('--hz', type=float, default=30.0,
                        help='发布频率')
    args = parser.parse_args()

    rclpy.init()
    node = FakeObstaclePublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
