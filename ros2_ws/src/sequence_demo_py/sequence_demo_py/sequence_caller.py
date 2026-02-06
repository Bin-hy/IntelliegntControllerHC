import rclpy
from rclpy.node import Node
import os
import time

# 导入服务接口
from lhandpro_interfaces.srv import SetEnable, SetPosition, SetPositionVelocity, MoveMotors, HomeMotors


class SequenceCallerClient(Node):
    def __init__(self):
        super().__init__('sequence_caller')

        # 创建服务客户端
        self.set_enable_cli = self.create_client(SetEnable, 'set_enable')
        self.set_position_cli = self.create_client(SetPosition, 'set_position')
        self.set_position_velocity_cli = self.create_client(SetPositionVelocity, 'set_position_velocity')
        self.move_motors_cli = self.create_client(MoveMotors, 'move_motors')
        self.home_motors_cli = self.create_client(HomeMotors, 'home_motors')

        # 等待服务上线
        while not self.set_enable_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_enable 服务上线...')

        while not self.set_position_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_position 服务上线...')

        while not self.set_position_velocity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_position_velocity 服务上线...')

        while not self.move_motors_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 move_motors 服务上线...')

        while not self.home_motors_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 home_motors 服务上线...')

        # 执行动作序列
        self.execute_trajectory()

    def set_enable(self, joint_id, enable):
        req = SetEnable.Request()
        req.joint_id = joint_id
        req.enable = enable
        future = self.set_enable_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().result == 0:
            self.get_logger().debug(f'关节 {joint_id} 设置使能成功')
        else:
            self.get_logger().error(f'设置使能失败 (joint_id={joint_id})')

    def set_position(self, joint_id, position):
        req = SetPosition.Request()
        req.joint_id = joint_id
        req.position = position
        future = self.set_position_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().result == 0:
            self.get_logger().debug(f'关节 {joint_id} 设置位置 {position} 成功')
        else:
            self.get_logger().error(f'设置位置失败 (joint_id={joint_id}, position={position})')

    def set_position_velocity(self, joint_id, velocity):
        req = SetPositionVelocity.Request()
        req.joint_id = joint_id
        req.velocity = velocity
        future = self.set_position_velocity_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().result == 0:
            self.get_logger().debug(f'关节 {joint_id} 设置速度 {velocity} 成功')
        else:
            self.get_logger().error(f'设置速度失败 (joint_id={joint_id}, velocity={velocity})')

    def move_motors(self, joint_id):
        req = MoveMotors.Request()
        req.joint_id = joint_id
        future = self.move_motors_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().result == 0:
            self.get_logger().debug(f'关节 {joint_id} 下发运动成功')
        else:
            self.get_logger().error(f'下发运动失败 (joint_id={joint_id})')
    
    def home_motors(self, joint_id):
        req = HomeMotors.Request()
        req.joint_id = joint_id
        future = self.home_motors_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().result == 0:
            self.get_logger().debug(f'关节 {joint_id} 下发回零成功')
        else:
            self.get_logger().error(f'下发回零失败 (joint_id={joint_id})')

    def execute_trajectory(self):
        # 设置默认参数
        joint_ids = [1, 2, 3, 4, 5, 6]
        default_velocity = 20000

        # 内嵌动作数组
        positions = [
            [0, 0, 0, 0, 0, 0],
            [10000, 0, 0, 0, 0, 0],
            [0, 10000, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 10000, 0, 0, 0],
            [0, 0, 0, 10000, 0, 0],
            [0, 0, 0, 0, 10000, 0],
            [0, 0, 0, 0, 0, 10000],
            [0, 0, 0, 0, 0, 0],
        ]

        self.set_enable(0, 1)
        self.get_logger().info("正在使能")
        time.sleep(1.0)  # 等待 1 秒

        self.home_motors(0)
        self.get_logger().info("正在回零")
        time.sleep(5)  # 等待 5 秒


        for i, position_step in enumerate(positions, start=1):
            self.get_logger().info(f"正在执行第 {i} 步动作...")

            # 遍历每个关节
            for j, joint_id in enumerate(joint_ids):
                position = position_step[j]
                self.set_position(joint_id, position)
                self.set_position_velocity(joint_id, default_velocity)

            # 触发所有关节运动
            self.move_motors(0)
            self.get_logger().info(f"第 {i} 步动作已触发")
            time.sleep(1.0)  # 等待 1 秒

        self.get_logger().info("轨迹执行完成")


def main(args=None):
    rclpy.init()

    client = SequenceCallerClient()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()