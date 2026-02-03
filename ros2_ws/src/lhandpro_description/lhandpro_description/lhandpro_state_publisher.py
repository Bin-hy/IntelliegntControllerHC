#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from lhandpro_interfaces.srv import GetNowAngle
import math


class LHandProStatePublisher(Node):
    def __init__(self):
        super().__init__('lhandpro_state_publisher')

        # 创建 JointState 发布器
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # 创建客户端连接到服务
        self.cli = self.create_client(GetNowAngle, '/lhandpro_service/get_now_angle')
        self.service_ready = False

        # 初始化关节信息
        self.joint_names = [
            'finger11', 'finger12', 'finger13', 'finger14',
            'finger21', 'finger22', 'finger23',
            'finger31', 'finger32', 'finger33',
            'finger41', 'finger42', 'finger43',
            'finger51', 'finger52', 'finger53'
        ]
        self.joint_positions = [0.0] * len(self.joint_names)
        self.map_index = {
            1: 'finger11',
            2: 'finger12',
            3: 'finger21',
            4: 'finger31',
            5: 'finger41',
            6: 'finger51'
        }

        # joint_relations: {从动关节名: (主动关节名, 比例)}
        self.joint_relations = {
            'finger13': ('finger12', 1),
            'finger22': ('finger21', 1),
            'finger32': ('finger31', 1),
            'finger42': ('finger41', 1),
            'finger52': ('finger51', 1),
        }

        # 定时器：每秒检查一次服务是否上线
        self.service_check_timer = self.create_timer(1.0, self.check_service_callback)

        # JointState 发布器（保持高频更新）
        self.publish_timer = self.create_timer(0.1, self.publish_joint_state)

        # 启动第一次请求链
        self.get_logger().info("节点初始化完成，启动异步请求链")
        self.start_async_polling()

    def check_service_callback(self):
        if not self.service_ready and self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info(f"{self.cli.srv_name} 服务已上线")
            self.service_ready = True
        elif not self.cli.service_is_ready():
            self.get_logger().warn(f"{self.cli.srv_name} 服务未上线，保持所有关节角度为 0")

    def start_async_polling(self):
        """启动异步轮询请求链"""
        self.joint_request_queue = list(range(1, 7))  # [1, 2, 3, 4, 5, 6]
        self.process_next_joint()

    def process_next_joint(self):
        """处理下一个要请求的关节"""
        if not self.joint_request_queue:
            # 所有关节都已完成，重新开始下一轮
            self.start_async_polling()
            return

        joint_id = self.joint_request_queue.pop(0)
        req = GetNowAngle.Request()
        req.joint_id = joint_id

        future = self.cli.call_async(req)
        future.add_done_callback(lambda f: self.future_callback_and_continue(f, joint_id))

    def future_callback_and_continue(self, future, joint_id):
        try:
            response = future.result()
            angle_deg = response.angle
            angle_rad = math.radians(angle_deg)

            joint_name = self.map_index[joint_id]
            idx = self.joint_names.index(joint_name)
            self.joint_positions[idx] = angle_rad

            self.get_logger().info(f"更新 {joint_name} to {angle_rad:.3f}")

            # 更新从动关节
            for dep_joint, (src_name, ratio) in self.joint_relations.items():
                if src_name == joint_name:
                    dep_idx = self.joint_names.index(dep_joint)
                    self.joint_positions[dep_idx] = angle_rad * ratio
                    self.get_logger().info(f"更新 {dep_joint} to {self.joint_positions[dep_idx]:.3f} (比例: {ratio})")

        except Exception as e:
            self.get_logger().warn(f"服务调用失败 (joint_id={joint_id}): {e}")

        # 继续处理下一个关节
        self.process_next_joint()

    def publish_joint_state(self):
        """发布 JointState 消息，使用本地缓存的角度值"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
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