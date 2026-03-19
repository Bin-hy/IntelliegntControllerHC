#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
from threading import Lock


class LHandProStatePublisher(Node):
    def __init__(self):
        super().__init__('lhandpro_state_publisher')

        # 创建 JointState 发布器
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.data_lock = Lock()

        # 关节名前缀，需要和 URDF 中的关节名匹配 (L_ 或 R_)
        self.declare_parameter('joint_prefix', 'L_')
        prefix = self.get_parameter('joint_prefix').get_parameter_value().string_value

        # 初始化关节信息 (前缀 + 关节名，与 URDF 一致)
        base_names = [
            'finger11', 'finger12', 'finger13', 'finger14',
            'finger21', 'finger22', 'finger23',
            'finger31', 'finger32', 'finger33',
            'finger41', 'finger42', 'finger43',
            'finger51', 'finger52', 'finger53'
        ]
        self.joint_names = [prefix + n for n in base_names]
        self.joint_positions = [0.0] * len(self.joint_names)
        self.map_index = {
            1: prefix + 'finger11',
            2: prefix + 'finger12',
            3: prefix + 'finger21',
            4: prefix + 'finger31',
            5: prefix + 'finger41',
            6: prefix + 'finger51'
        }

        # joint_relations: {从动关节名: (主动关节名, 比例)}
        self.joint_relations = {
            prefix + 'finger13': (prefix + 'finger12', 1),
            prefix + 'finger22': (prefix + 'finger21', 1),
            prefix + 'finger32': (prefix + 'finger31', 1),
            prefix + 'finger42': (prefix + 'finger41', 1),
            prefix + 'finger52': (prefix + 'finger51', 1),
        }

        self.now_angles_sub = self.create_subscription(
            Float64MultiArray, '/lhandpro_service/now_angles', self.now_angles_callback, 10)

        self.publish_timer = self.create_timer(0.1, self.publish_joint_state)

    def now_angles_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        with self.data_lock:
            for joint_id in range(1, 7):
                angle_deg = msg.data[joint_id - 1]
                angle_rad = math.radians(angle_deg)
                joint_name = self.map_index[joint_id]
                idx = self.joint_names.index(joint_name)
                self.joint_positions[idx] = angle_rad
                for dep_joint, (src_name, ratio) in self.joint_relations.items():
                    if src_name == joint_name:
                        dep_idx = self.joint_names.index(dep_joint)
                        self.joint_positions[dep_idx] = angle_rad * ratio

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        with self.data_lock:
            msg.position = list(self.joint_positions)
        msg.velocity = []
        msg.effort = []

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init()

    try:
        node = LHandProStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
