#!/usr/bin/env python3
"""
publish_test_clouds.py — 先发布安全点云，再发布近距离点云
用于自动化测试中替代内联 Python
"""
import rclpy
import struct
import sys
import time
import threading

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud(node, frame_id, points):
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    data = bytearray()
    for x, y, z in points:
        data.extend(struct.pack('<fff', x, y, z))
    msg.data = bytes(data)
    return msg


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'safe'
    topic = sys.argv[2] if len(sys.argv) > 2 else '/fake/depth/points'
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

    rclpy.init()
    node = Node(f'test_cloud_{mode}')
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5
    )
    pub = node.create_publisher(PointCloud2, topic, qos)

    t = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    t.start()

    if mode == 'safe':
        points = [(2.0, 2.0, 2.0)]
    elif mode == 'close':
        # 3个点在 base_link (0,0,0) 正上方 0.20m
        # base_link r=0.15, self_margin=0.04, filter_r=0.19
        # d=0.20 > 0.19 → 不被过滤, 表面距离=0.05 → warning
        points = [
            (0.0, 0.0, 0.20),
            (0.01, 0.0, 0.20),
            (-0.01, 0.0, 0.20),
        ]
    else:
        points = [(1.0, 0.0, 1.0)]

    end_time = time.time() + duration if duration > 0 else float('inf')
    hz = 30.0
    node.get_logger().info(f'发布 {mode} 点云到 {topic}, 持续 {"无限" if duration <= 0 else f"{duration}s"}')
    try:
        while time.time() < end_time:
            cloud = make_cloud(node, 'base_link', points)
            pub.publish(cloud)
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        pass

    node.get_logger().info(f'{mode} 点云发布完成')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
